use super::rest::{RestSuffixEvaluation, select_causal_rest_suffixes};
use super::{
    DyadicBlocker, PlateauSelection, RateBlock, RawVelocityObservation, RestSelectionDecision,
    RestSelectionDisposition, RestStructuralCode, VELOCITY_MIN_SELECTED_BLOCKS,
    VELOCITY_SUFFIX_LENGTHS, VelocityMeasurementError, confidence_multiplier_95_df15,
    estimate_confidence,
};

const NO_REPORTABLE_DRIFT_RATIO_PPM: u32 = u32::MAX;

const MICROS_PER_SECOND: f32 = 1_000_000.0;
const LAG_ONE_Q_SCALE: f32 = 1_000_000.0;
const REST_WITNESS_OBSERVATIONS_PER_STEP: usize = 16;

/// Reason a captured tail cannot support a stationary settled estimate.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StationarityExclusion {
    /// No two-consecutive-doubling variance plateau was found.
    NoBlockingPlateau,
    /// The selected scale has fewer than the required decorrelated blocks.
    TooFewDecorrelatedBlocks,
    /// The fitted rate slope remains distinguishable from zero.
    ResolvedSlope,
    /// Detrended residuals retain significant lag-one structure.
    StructuredResidual,
    /// Checked arithmetic or a numeric conversion failed.
    Arithmetic,
    /// The rate drifted too far across the regression span to be described by one value.
    ExcessiveDrift,
}

/// Result of selecting one target-independent stationary tail.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StationarityResult {
    /// A contiguous suffix passed every stationarity gate.
    Usable(StationarySuffix),
    /// No declared suffix passed every stationarity gate.
    Excluded(StationarityExclusion),
}

/// Statistics retained for the longest usable stationary suffix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarySuffix {
    /// Blocking plateau selected for the suffix.
    pub selection: PlateauSelection,
    /// Number of raw observations in the selected suffix.
    pub suffix_len: u16,
    /// Fitted rate slope in fixed-point rate units per second.
    pub slope_q: i64,
    /// Outward-rounded family-wise slope half-width in the same units.
    pub slope_half_width_q: u64,
    /// Lag-one residual correlation scaled by one million.
    pub lag_one_q: i32,
    /// Outward-rounded lag-one half-width scaled by one million.
    pub lag_one_half_width_q: u32,
    /// Mean squared detrended residual in squared fixed-point rate units.
    pub residual_magnitude_q2: u128,
    /// Doubled midpoint-to-midpoint regression span in microseconds.
    pub span_x2_us: u64,
    /// Drift across the regression span, parts per million of settled rate.
    /// [`u32::MAX`] means no finite reportable ratio.
    pub drift_ratio_ppm: u32,
    /// Number of nested suffix candidates examined.
    pub tested_suffixes: u8,
}

/// Non-binding evidence for one declared suffix length.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct SuffixDiagnostic {
    /// Number of raw observations in this suffix.
    pub suffix_len: u16,
    /// Exact blocker selection retained when post-fit metrics were available.
    ///
    /// `None` means the suffix failed before a plateau could be selected.
    pub selection: Option<PlateauSelection>,
    /// Whether the blocker found a usable two-doubling plateau.
    pub usable_plateau: bool,
    /// Whether the fitted slope confidence interval contains zero.
    pub slope_passed: bool,
    /// Whether the lag-one residual confidence interval contains its null.
    pub lag_one_passed: bool,
    /// Lag-one residual correlation scaled by one million.
    pub lag_one_q: i32,
    /// Outward-rounded settled-rate confidence interval.
    pub settled_rate_interval_q: [i64; 2],
    /// Positive clearance or negative rejection beyond the slope interval.
    pub slope_rejection_margin_q: i64,
    /// Positive clearance or negative rejection beyond the lag-one interval.
    pub lag_one_rejection_margin_q: i32,
    /// Fitted rate slope in fixed-point rate units per second.
    pub slope_q: i64,
    /// Outward-rounded family-wise slope half-width in the same units.
    pub slope_half_width_q: u64,
}

/// Binding stationarity result plus all declared suffix diagnostics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StationarityDiagnostics {
    /// Unchanged shortest-first, no-stitch stationarity decision.
    pub binding: StationarityResult,
    /// Available suffix diagnostics in [`VELOCITY_SUFFIX_LENGTHS`] order.
    pub suffixes: [Option<SuffixDiagnostic>; 3],
}

/// Compact non-binding evidence that longer suffixes bound short-scale structure.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StructuredStationarityEvidence {
    /// Bit set for each declared suffix that passes every stationarity gate.
    pub passing_suffix_mask: u8,
    /// Common settled-rate interval shared by the binding and passing suffixes.
    pub common_rate_interval_q: [i64; 2],
    /// Signed slope rejection margin from the binding 64-sample suffix.
    pub binding_slope_margin_q: i64,
    /// Signed lag-one rejection margin from the binding 64-sample suffix.
    pub binding_lag_one_margin_q: i32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct StationarityMetrics {
    slope_q: i64,
    slope_half_width_q: u64,
    lag_one_q: i32,
    lag_one_half_width_q: u32,
    residual_magnitude_q2: u128,
    slope_passed: bool,
    lag_one_passed: bool,
    slope_rejection_margin_q: i64,
    lag_one_rejection_margin_q: i32,
    span_x2_us: u64,
    drift_ratio_ppm: u32,
}

/// Maximum raw observations consumed by one cooperative stationarity step.
pub const STATIONARITY_WORK_OBSERVATIONS_PER_STEP: usize = 8;

/// Result of one bounded cooperative stationarity-analysis step.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationarityWorkPoll {
    /// More bounded work remains.
    Pending,
    /// The declared suffix family has been classified.
    ///
    /// `rejection` is `None` for a usable tail and for exclusions decided before the gates.
    Complete {
        /// Binding stationarity decision.
        result: StationarityResult,
        /// Gate statistics for a rejected tail, when one was evaluated.
        rejection: Option<SuffixDiagnostic>,
        /// Drift across the regression span, parts per million of settled rate.
        ///
        /// [`u32::MAX`] means no finite reportable ratio.
        drift_ratio_ppm: u32,
    },
    /// Every scored-rest suffix has been classified for causal reporting.
    RestDiagnostic(RestSelectionDecision),
    /// The scored-rest decision is complete, including any selected witness.
    RestComplete(RestSelectionCompletion),
}

/// Completed scored-rest decision after any selected witness is reconstructed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RestSelectionCompletion {
    /// Full accepted witness, or the recent usable witness retained for a rejection diagnostic.
    pub selected: Option<StationarySuffix>,
    /// Exact selected interval, conditioned on zero compatibility.
    pub exact_velocity_interval_q: Option<[i64; 2]>,
    /// Final selector disposition emitted by the preceding diagnostic.
    pub disposition: RestSelectionDisposition,
    /// Structural result for the most recent 64-sample suffix.
    pub recent_structural_code: RestStructuralCode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum StationarityWorkMode {
    Moving,
    Structured,
    RestDiagnostic { one_count_fallback: bool },
    RestWitness,
}

#[derive(Debug, Clone, Copy)]
enum StationarityWorkStage {
    Idle {
        mode: StationarityWorkMode,
    },
    Blocking {
        mode: StationarityWorkMode,
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        next_index: u16,
    },
    Regression {
        mode: StationarityWorkMode,
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        block_observations: u16,
        block_count: u16,
        next_block: u16,
        next_observation: u16,
        partial_block: Option<RateBlock>,
        cumulative_elapsed_us: u64,
        sum_x: i128,
        sum_y: i128,
        sum_x2: i128,
        sum_xy: i128,
    },
    Residual {
        mode: StationarityWorkMode,
        suffix_index: u8,
        suffix_start: u16,
        suffix_len: u16,
        block_observations: u16,
        block_count: u16,
        next_block: u16,
        next_observation: u16,
        partial_block: Option<RateBlock>,
        cumulative_elapsed_us: u64,
        first_block_elapsed_us: u64,
        slope_per_us: f32,
        intercept: f32,
        denominator: i128,
        residual_sum_q2: f32,
        lag_product_sum_q2: f32,
        previous_residual: Option<f32>,
    },
    RestDecisionReady,
    Done,
}

#[derive(Debug, Clone, Copy)]
struct StructuredStationarityAccumulator {
    binding_exclusion: StationarityExclusion,
    binding: RetainedPostFitDiagnostic,
    passing_suffix_mask: u8,
    common_interval_valid: bool,
    common_rate_interval_q: [i64; 2],
    binding_slope_margin_q: i64,
    binding_lag_one_margin_q: i32,
    binding_drift_ratio_ppm: u32,
    selected: Option<StationarySuffix>,
}

#[derive(Debug, Clone, Copy)]
struct SuffixRefusal {
    exclusion: StationarityExclusion,
    rejection: Option<SuffixDiagnostic>,
    drift_ratio_ppm: u32,
}

#[derive(Debug, Clone, Copy)]
struct RetainedPostFitDiagnostic {
    selection: PlateauSelection,
    suffix_len: u16,
    slope_q: i64,
    slope_half_width_q: u64,
    lag_one_q: i32,
}

impl RetainedPostFitDiagnostic {
    fn from_diagnostic(diagnostic: SuffixDiagnostic) -> Result<Self, VelocityMeasurementError> {
        Ok(Self {
            selection: diagnostic
                .selection
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
            suffix_len: diagnostic.suffix_len,
            slope_q: diagnostic.slope_q,
            slope_half_width_q: diagnostic.slope_half_width_q,
            lag_one_q: diagnostic.lag_one_q,
        })
    }

    fn reconstruct(
        self,
        slope_rejection_margin_q: i64,
        lag_one_rejection_margin_q: i32,
    ) -> Result<SuffixDiagnostic, VelocityMeasurementError> {
        let confidence = estimate_confidence(self.selection, 3)?;
        let mean_rate_q = self.selection.exact_totals.rate_q()?;
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
        let low = mean_rate_q
            .checked_sub(half_width)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let high = mean_rate_q
            .checked_add(half_width)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        Ok(SuffixDiagnostic {
            suffix_len: self.suffix_len,
            selection: Some(self.selection),
            usable_plateau: true,
            slope_passed: slope_rejection_margin_q >= 0,
            lag_one_passed: lag_one_rejection_margin_q >= 0,
            lag_one_q: self.lag_one_q,
            settled_rate_interval_q: [low, high],
            slope_rejection_margin_q,
            lag_one_rejection_margin_q,
            slope_q: self.slope_q,
            slope_half_width_q: self.slope_half_width_q,
        })
    }
}

#[derive(Debug, Clone, Copy)]
struct RestSelectionAccumulator {
    suffixes: [RestSuffixEvaluation; 3],
    one_count_fallback: bool,
}

impl RestSelectionAccumulator {
    const fn new(one_count_fallback: bool) -> Self {
        Self {
            suffixes: [RestSuffixEvaluation {
                structural_code: RestStructuralCode::InsufficientHistory,
                interval_present: false,
                interval_q: [0; 2],
            }; 3],
            one_count_fallback,
        }
    }

    fn observe(&mut self, suffix_index: u8, evaluation: RestSuffixEvaluation) {
        self.suffixes[usize::from(suffix_index)] = evaluation;
    }

    fn decision(self, retained_sample_count: u16) -> RestSelectionDecision {
        select_causal_rest_suffixes(
            self.suffixes,
            self.one_count_fallback,
            retained_sample_count,
        )
    }
}

impl StructuredStationarityAccumulator {
    fn new(
        binding_exclusion: StationarityExclusion,
        binding: SuffixDiagnostic,
        binding_drift_ratio_ppm: u32,
    ) -> Result<Self, VelocityMeasurementError> {
        Ok(Self {
            binding_exclusion,
            binding: RetainedPostFitDiagnostic::from_diagnostic(binding)?,
            passing_suffix_mask: 0,
            common_interval_valid: binding.usable_plateau,
            common_rate_interval_q: binding.settled_rate_interval_q,
            binding_slope_margin_q: binding.slope_rejection_margin_q,
            binding_lag_one_margin_q: binding.lag_one_rejection_margin_q,
            binding_drift_ratio_ppm,
            selected: None,
        })
    }

    fn observe(&mut self, suffix_index: u8, diagnostic: SuffixDiagnostic) {
        if !(diagnostic.usable_plateau && diagnostic.slope_passed && diagnostic.lag_one_passed) {
            return;
        }
        self.passing_suffix_mask |= 1 << suffix_index;
        if !self.common_interval_valid {
            return;
        }
        let low = self.common_rate_interval_q[0].max(diagnostic.settled_rate_interval_q[0]);
        let high = self.common_rate_interval_q[1].min(diagnostic.settled_rate_interval_q[1]);
        self.common_interval_valid = low <= high;
        self.common_rate_interval_q = [low, high];
    }

    fn evidence(self) -> Option<StructuredStationarityEvidence> {
        if !self.common_interval_valid || self.passing_suffix_mask & !1 == 0 {
            return None;
        }
        Some(StructuredStationarityEvidence {
            passing_suffix_mask: self.passing_suffix_mask,
            common_rate_interval_q: self.common_rate_interval_q,
            binding_slope_margin_q: self.binding_slope_margin_q,
            binding_lag_one_margin_q: self.binding_lag_one_margin_q,
        })
    }
}

#[derive(Debug, Clone, Copy)]
enum StationarityWorkOutput {
    Empty,
    Selected(StationarySuffix),
    Structured(StructuredStationarityAccumulator),
    RestAccumulator(RestSelectionAccumulator),
    RestDecision(RestSelectionDecision),
}

/// Resumable nested-suffix stationarity analysis for a fixed capture.
#[derive(Debug, Clone, Copy)]
pub struct StationarityWork {
    stage: StationarityWorkStage,
    output: StationarityWorkOutput,
}

impl StationarityWork {
    /// Construct an idle work item.
    pub const fn new() -> Self {
        Self {
            stage: StationarityWorkStage::Idle {
                mode: StationarityWorkMode::Moving,
            },
            output: StationarityWorkOutput::Empty,
        }
    }

    /// Reset all progress before analyzing another fixed capture.
    pub fn reset(&mut self) {
        *self = Self::new();
    }

    /// Reset progress and retain compact structured-boundary diagnostics.
    pub fn reset_with_structured_diagnostics(&mut self) {
        *self = Self {
            stage: StationarityWorkStage::Idle {
                mode: StationarityWorkMode::Structured,
            },
            output: StationarityWorkOutput::Empty,
        };
    }

    /// Reset progress before one scored-rest diagnostic and causal selection.
    pub fn reset_for_scored_rest(&mut self, one_count_fallback: bool) {
        *self = Self {
            stage: StationarityWorkStage::Idle {
                mode: StationarityWorkMode::RestDiagnostic { one_count_fallback },
            },
            output: StationarityWorkOutput::RestAccumulator(RestSelectionAccumulator::new(
                one_count_fallback,
            )),
        };
    }

    /// Return compact structured-boundary evidence after completed work.
    pub fn structured_evidence(&self) -> Option<StructuredStationarityEvidence> {
        match self.output {
            StationarityWorkOutput::Structured(accumulator) => accumulator.evidence(),
            StationarityWorkOutput::Empty
            | StationarityWorkOutput::Selected(_)
            | StationarityWorkOutput::RestAccumulator(_)
            | StationarityWorkOutput::RestDecision(_) => None,
        }
    }

    /// Advance analysis by one bounded raw-observation quantum.
    pub fn poll<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        if let StationarityWorkStage::Idle { mode } = self.stage {
            let Some((suffix_index, suffix_len)) = first_available_suffix(observations) else {
                self.stage = StationarityWorkStage::Done;
                if let StationarityWorkMode::RestDiagnostic { .. } = mode {
                    let StationarityWorkOutput::RestAccumulator(accumulator) = self.output else {
                        return Err(VelocityMeasurementError::ArithmeticOverflow);
                    };
                    let retained_sample_count = u16::try_from(observations.observation_len())
                        .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
                    let decision = accumulator.decision(retained_sample_count);
                    self.output = StationarityWorkOutput::RestDecision(decision);
                    self.stage = StationarityWorkStage::RestDecisionReady;
                    return Ok(StationarityWorkPoll::RestDiagnostic(decision));
                }
                return Ok(StationarityWorkPoll::Complete {
                    result: StationarityResult::Excluded(
                        StationarityExclusion::TooFewDecorrelatedBlocks,
                    ),
                    rejection: None,
                    drift_ratio_ppm: NO_REPORTABLE_DRIFT_RATIO_PPM,
                });
            };
            self.start_suffix(observations, blocker, suffix_index, suffix_len, mode)?;
        }

        match self.stage {
            StationarityWorkStage::Blocking { .. } => self.poll_blocking(observations, blocker),
            StationarityWorkStage::Regression { .. } => self.poll_regression(observations),
            StationarityWorkStage::Residual { .. } => self.poll_residual(observations, blocker),
            StationarityWorkStage::RestDecisionReady => {
                self.resume_rest_decision(observations, blocker)
            }
            StationarityWorkStage::Done => Err(VelocityMeasurementError::ArithmeticOverflow),
            StationarityWorkStage::Idle { .. } => {
                unreachable!("idle work starts before dispatch")
            }
        }
    }

    fn start_suffix<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        suffix_len: u16,
        mode: StationarityWorkMode,
    ) -> Result<(), VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        *blocker = DyadicBlocker::new();
        let suffix_start = observations
            .observation_len()
            .checked_sub(usize::from(suffix_len))
            .and_then(|value| u16::try_from(value).ok())
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        self.stage = StationarityWorkStage::Blocking {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            next_index: suffix_start,
        };
        Ok(())
    }

    fn poll_blocking<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Blocking {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            mut next_index,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let end = suffix_start
            .checked_add(suffix_len)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let step_end = end.min(
            next_index
                .checked_add(work_observations_per_step(mode) as u16)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
        );
        while next_index < step_end {
            blocker.observe(
                observations
                    .observation(usize::from(next_index))
                    .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
            )?;
            next_index += 1;
        }
        if next_index < end {
            self.stage = StationarityWorkStage::Blocking {
                mode,
                suffix_index,
                suffix_start,
                suffix_len,
                next_index,
            };
            return Ok(StationarityWorkPoll::Pending);
        }

        let Some(selection) = blocker.select_plateau() else {
            let exclusion = if blocker.has_two_doubling_plateau() {
                StationarityExclusion::TooFewDecorrelatedBlocks
            } else {
                StationarityExclusion::NoBlockingPlateau
            };
            if matches!(self.output, StationarityWorkOutput::Structured(_)) {
                return self.advance_structured(observations, blocker, suffix_index, None, None);
            }
            if matches!(self.output, StationarityWorkOutput::RestAccumulator(_)) {
                let structural = if blocker.has_two_doubling_plateau() {
                    RestExclusion::TooFewDecorrelatedBlocks
                } else {
                    RestExclusion::NoBlockingPlateau
                };
                return self.advance_rest_diagnostic(
                    observations,
                    blocker,
                    suffix_index,
                    RestSuffixEvaluation {
                        structural_code: rest_structural_code(structural),
                        interval_present: false,
                        interval_q: [0; 2],
                    },
                );
            }
            return self.finish_on_exclusion(
                observations,
                blocker,
                suffix_index,
                SuffixRefusal {
                    exclusion,
                    rejection: None,
                    drift_ratio_ppm: NO_REPORTABLE_DRIFT_RATIO_PPM,
                },
                mode,
            );
        };
        let block_observations = 1u16
            .checked_shl(u32::from(selection.level))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let block_count = suffix_len / block_observations;
        if usize::from(block_count) < VELOCITY_MIN_SELECTED_BLOCKS as usize {
            if matches!(self.output, StationarityWorkOutput::Structured(_)) {
                return self.advance_structured(observations, blocker, suffix_index, None, None);
            }
            if matches!(self.output, StationarityWorkOutput::RestAccumulator(_)) {
                return self.advance_rest_diagnostic(
                    observations,
                    blocker,
                    suffix_index,
                    RestSuffixEvaluation {
                        structural_code: RestStructuralCode::TooFewDecorrelatedBlocks,
                        interval_present: false,
                        interval_q: [0; 2],
                    },
                );
            }
            return self.finish_on_exclusion(
                observations,
                blocker,
                suffix_index,
                SuffixRefusal {
                    exclusion: StationarityExclusion::TooFewDecorrelatedBlocks,
                    rejection: None,
                    drift_ratio_ppm: NO_REPORTABLE_DRIFT_RATIO_PPM,
                },
                mode,
            );
        }
        self.stage = StationarityWorkStage::Regression {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            next_block: 0,
            next_observation: 0,
            partial_block: None,
            cumulative_elapsed_us: 0,
            sum_x: 0,
            sum_y: 0,
            sum_x2: 0,
            sum_xy: 0,
        };
        Ok(StationarityWorkPoll::Pending)
    }

    fn poll_regression<S>(
        &mut self,
        observations: &S,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Regression {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            mut next_block,
            mut next_observation,
            mut partial_block,
            mut cumulative_elapsed_us,
            mut sum_x,
            mut sum_y,
            mut sum_x2,
            mut sum_xy,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let mut remaining_visits = work_observations_per_step(mode);
        while next_block < block_count && remaining_visits > 0 {
            let index = usize::from(suffix_start)
                .checked_add(
                    usize::from(next_block)
                        .checked_mul(usize::from(block_observations))
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .and_then(|value| value.checked_add(usize::from(next_observation)))
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let observation = observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let next = RateBlock::from_observation(observation)?;
            partial_block = Some(match partial_block {
                Some(current) => current.checked_merge(next)?,
                None => next,
            });
            next_observation = next_observation
                .checked_add(1)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            remaining_visits -= 1;
            if next_observation < block_observations {
                continue;
            }
            let block = partial_block
                .take()
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let midpoint_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us / 2)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            cumulative_elapsed_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let x = i128::from(midpoint_us);
            let y = i128::from(block.rate_q()?);
            sum_x = sum_x
                .checked_add(x)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_y = sum_y
                .checked_add(y)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_x2 = sum_x2
                .checked_add(
                    x.checked_mul(x)
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            sum_xy = sum_xy
                .checked_add(
                    x.checked_mul(y)
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            next_block += 1;
            next_observation = 0;
        }
        if next_block < block_count {
            self.stage = StationarityWorkStage::Regression {
                mode,
                suffix_index,
                suffix_start,
                suffix_len,
                block_observations,
                block_count,
                next_block,
                next_observation,
                partial_block,
                cumulative_elapsed_us,
                sum_x,
                sum_y,
                sum_x2,
                sum_xy,
            };
            return Ok(StationarityWorkPoll::Pending);
        }

        let n = i128::from(block_count);
        let denominator = n
            .checked_mul(sum_x2)
            .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_x)?))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        if denominator <= 0 {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        let numerator = n
            .checked_mul(sum_xy)
            .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_y)?))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let slope_per_us = numerator as f32 / denominator as f32;
        let intercept = (sum_y as f32 - slope_per_us * sum_x as f32) / block_count as f32;
        if !slope_per_us.is_finite() || !intercept.is_finite() {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        self.stage = StationarityWorkStage::Residual {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            next_block: 0,
            next_observation: 0,
            partial_block: None,
            cumulative_elapsed_us: 0,
            first_block_elapsed_us: 0,
            slope_per_us,
            intercept,
            denominator,
            residual_sum_q2: 0.0,
            lag_product_sum_q2: 0.0,
            previous_residual: None,
        };
        Ok(StationarityWorkPoll::Pending)
    }

    fn poll_residual<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkStage::Residual {
            mode,
            suffix_index,
            suffix_start,
            suffix_len,
            block_observations,
            block_count,
            mut next_block,
            mut next_observation,
            mut partial_block,
            mut cumulative_elapsed_us,
            mut first_block_elapsed_us,
            slope_per_us,
            intercept,
            denominator,
            mut residual_sum_q2,
            mut lag_product_sum_q2,
            mut previous_residual,
        } = self.stage
        else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let mut remaining_visits = work_observations_per_step(mode);
        let mut last_block_elapsed_us = 0u64;
        while next_block < block_count && remaining_visits > 0 {
            let index = usize::from(suffix_start)
                .checked_add(
                    usize::from(next_block)
                        .checked_mul(usize::from(block_observations))
                        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
                )
                .and_then(|value| value.checked_add(usize::from(next_observation)))
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let observation = observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let next = RateBlock::from_observation(observation)?;
            partial_block = Some(match partial_block {
                Some(current) => current.checked_merge(next)?,
                None => next,
            });
            next_observation = next_observation
                .checked_add(1)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            remaining_visits -= 1;
            if next_observation < block_observations {
                continue;
            }
            let block = partial_block
                .take()
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            if next_block == 0 {
                first_block_elapsed_us = block.elapsed_us;
            }
            last_block_elapsed_us = block.elapsed_us;
            let midpoint_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us / 2)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            cumulative_elapsed_us = cumulative_elapsed_us
                .checked_add(block.elapsed_us)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
            let residual = block.rate_q()? as f32 - (intercept + slope_per_us * midpoint_us as f32);
            residual_sum_q2 += residual * residual;
            if let Some(previous) = previous_residual {
                lag_product_sum_q2 += residual * previous;
            }
            previous_residual = Some(residual);
            next_block += 1;
            next_observation = 0;
        }
        if next_block < block_count {
            self.stage = StationarityWorkStage::Residual {
                mode,
                suffix_index,
                suffix_start,
                suffix_len,
                block_observations,
                block_count,
                next_block,
                next_observation,
                partial_block,
                cumulative_elapsed_us,
                first_block_elapsed_us,
                slope_per_us,
                intercept,
                denominator,
                residual_sum_q2,
                lag_product_sum_q2,
                previous_residual,
            };
            return Ok(StationarityWorkPoll::Pending);
        }
        let selection = blocker
            .select_plateau()
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let settled = SettledRateContext {
            mean_rate_q: selection.exact_totals.rate_q()?,
            half_width_rate_q: estimate_confidence(selection, 3)?.half_width_rate_q,
        };
        let span_x2_us = cumulative_elapsed_us
            .checked_mul(2)
            .and_then(|value| value.checked_sub(first_block_elapsed_us))
            .and_then(|value| value.checked_sub(last_block_elapsed_us))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        let metrics = calculate_stationarity_metrics(
            block_count as usize,
            denominator,
            slope_per_us,
            residual_sum_q2,
            lag_product_sum_q2,
            3,
            selection.variance_of_mean_q2,
            span_x2_us,
            settled,
        );
        match metrics {
            Ok(mut metrics) => {
                let diagnostic = suffix_diagnostic_from_metrics(selection, suffix_len, metrics)?;
                if matches!(self.output, StationarityWorkOutput::RestAccumulator(_)) {
                    let structural_code = match classify_rest_metrics(metrics) {
                        Ok(_) => RestStructuralCode::Usable,
                        Err(exclusion) => rest_structural_code(exclusion),
                    };
                    return self.advance_rest_diagnostic(
                        observations,
                        blocker,
                        suffix_index,
                        RestSuffixEvaluation {
                            structural_code,
                            interval_present: true,
                            interval_q: diagnostic.settled_rate_interval_q,
                        },
                    );
                }
                let classified = match mode {
                    StationarityWorkMode::RestWitness
                    | StationarityWorkMode::RestDiagnostic { .. } => {
                        classify_rest_metrics(metrics).map_err(Into::into)
                    }
                    StationarityWorkMode::Moving | StationarityWorkMode::Structured => {
                        classify_moving_metrics(&mut metrics, settled).map(|()| metrics)
                    }
                };
                match classified {
                    Ok(metrics) => {
                        let selected = StationarySuffix {
                            selection,
                            suffix_len,
                            slope_q: metrics.slope_q,
                            slope_half_width_q: metrics.slope_half_width_q,
                            lag_one_q: metrics.lag_one_q,
                            lag_one_half_width_q: metrics.lag_one_half_width_q,
                            residual_magnitude_q2: metrics.residual_magnitude_q2,
                            span_x2_us: metrics.span_x2_us,
                            drift_ratio_ppm: metrics.drift_ratio_ppm,
                            tested_suffixes: suffix_index.saturating_add(1),
                        };
                        if let StationarityWorkOutput::Structured(_) = self.output {
                            return self.advance_structured(
                                observations,
                                blocker,
                                suffix_index,
                                Some(diagnostic),
                                Some(selected),
                            );
                        }
                        if mode == StationarityWorkMode::RestWitness {
                            return self
                                .finish_rest_witness(selected, diagnostic.settled_rate_interval_q);
                        }
                        self.output = StationarityWorkOutput::Selected(selected);
                        self.advance_or_complete(observations, blocker, suffix_index, mode)
                    }
                    Err(exclusion)
                        if mode == StationarityWorkMode::Structured && suffix_index == 0 =>
                    {
                        self.output = StationarityWorkOutput::Structured(
                            StructuredStationarityAccumulator::new(
                                exclusion,
                                diagnostic,
                                metrics.drift_ratio_ppm,
                            )?,
                        );
                        self.advance_structured(observations, blocker, suffix_index, None, None)
                    }
                    Err(_) if matches!(self.output, StationarityWorkOutput::Structured(_)) => {
                        self.advance_structured(observations, blocker, suffix_index, None, None)
                    }
                    Err(exclusion) => self.finish_on_exclusion(
                        observations,
                        blocker,
                        suffix_index,
                        SuffixRefusal {
                            exclusion,
                            rejection: Some(diagnostic),
                            drift_ratio_ppm: metrics.drift_ratio_ppm,
                        },
                        mode,
                    ),
                }
            }
            Err(_) if matches!(self.output, StationarityWorkOutput::Structured(_)) => {
                self.advance_structured(observations, blocker, suffix_index, None, None)
            }
            Err(StationarityExclusion::Arithmetic)
                if matches!(
                    self.output,
                    StationarityWorkOutput::RestAccumulator(_)
                        | StationarityWorkOutput::RestDecision(_)
                ) =>
            {
                Err(VelocityMeasurementError::ArithmeticOverflow)
            }
            Err(exclusion) => self.finish_on_exclusion(
                observations,
                blocker,
                suffix_index,
                SuffixRefusal {
                    exclusion,
                    rejection: None,
                    drift_ratio_ppm: NO_REPORTABLE_DRIFT_RATIO_PPM,
                },
                mode,
            ),
        }
    }

    fn finish_on_exclusion<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        refusal: SuffixRefusal,
        mode: StationarityWorkMode,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let SuffixRefusal {
            exclusion,
            rejection,
            drift_ratio_ppm,
        } = refusal;
        if let StationarityWorkOutput::Selected(mut usable) = self.output {
            usable.tested_suffixes = suffix_index.saturating_add(1);
            self.output = StationarityWorkOutput::Selected(usable);
            self.stage = StationarityWorkStage::Done;
            return Ok(StationarityWorkPoll::Complete {
                result: StationarityResult::Usable(usable),
                rejection: None,
                drift_ratio_ppm: usable.drift_ratio_ppm,
            });
        }
        let next_index = suffix_index.saturating_add(1);
        if let Some(&suffix_len) = VELOCITY_SUFFIX_LENGTHS.get(usize::from(next_index))
            && usize::from(suffix_len) <= observations.observation_len()
        {
            self.start_suffix(observations, blocker, next_index, suffix_len, mode)?;
            return Ok(StationarityWorkPoll::Pending);
        }
        self.stage = StationarityWorkStage::Done;
        Ok(StationarityWorkPoll::Complete {
            result: StationarityResult::Excluded(exclusion),
            rejection,
            drift_ratio_ppm,
        })
    }

    fn advance_or_complete<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        mode: StationarityWorkMode,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let next_index = suffix_index.saturating_add(1);
        if let Some(&suffix_len) = VELOCITY_SUFFIX_LENGTHS.get(usize::from(next_index))
            && usize::from(suffix_len) <= observations.observation_len()
        {
            self.start_suffix(observations, blocker, next_index, suffix_len, mode)?;
            return Ok(StationarityWorkPoll::Pending);
        }
        let StationarityWorkOutput::Selected(mut result) = self.output else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        result.tested_suffixes = next_index;
        self.output = StationarityWorkOutput::Selected(result);
        self.stage = StationarityWorkStage::Done;
        Ok(StationarityWorkPoll::Complete {
            result: StationarityResult::Usable(result),
            rejection: None,
            drift_ratio_ppm: result.drift_ratio_ppm,
        })
    }

    fn advance_structured<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        diagnostic: Option<SuffixDiagnostic>,
        passing: Option<StationarySuffix>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkOutput::Structured(mut accumulator) = self.output else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        if let Some(diagnostic) = diagnostic {
            accumulator.observe(suffix_index, diagnostic);
        }
        if passing.is_some() {
            accumulator.selected = passing;
        }
        self.output = StationarityWorkOutput::Structured(accumulator);
        let next_index = suffix_index.saturating_add(1);
        if let Some(&suffix_len) = VELOCITY_SUFFIX_LENGTHS.get(usize::from(next_index))
            && usize::from(suffix_len) <= observations.observation_len()
        {
            self.start_suffix(
                observations,
                blocker,
                next_index,
                suffix_len,
                StationarityWorkMode::Structured,
            )?;
            return Ok(StationarityWorkPoll::Pending);
        }
        self.stage = StationarityWorkStage::Done;
        if let Some(mut selected) = accumulator.selected {
            selected.tested_suffixes = next_index;
            return Ok(StationarityWorkPoll::Complete {
                result: StationarityResult::Usable(selected),
                rejection: None,
                drift_ratio_ppm: selected.drift_ratio_ppm,
            });
        }
        let rejection = accumulator.binding.reconstruct(
            accumulator.binding_slope_margin_q,
            accumulator.binding_lag_one_margin_q,
        )?;
        Ok(StationarityWorkPoll::Complete {
            result: StationarityResult::Excluded(accumulator.binding_exclusion),
            rejection: Some(rejection),
            drift_ratio_ppm: accumulator.binding_drift_ratio_ppm,
        })
    }

    fn advance_rest_diagnostic<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
        suffix_index: u8,
        evaluation: RestSuffixEvaluation,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkOutput::RestAccumulator(mut accumulator) = self.output else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        accumulator.observe(suffix_index, evaluation);
        self.output = StationarityWorkOutput::RestAccumulator(accumulator);
        let next_index = suffix_index.saturating_add(1);
        if let Some(&suffix_len) = VELOCITY_SUFFIX_LENGTHS.get(usize::from(next_index))
            && usize::from(suffix_len) <= observations.observation_len()
        {
            self.start_suffix(
                observations,
                blocker,
                next_index,
                suffix_len,
                StationarityWorkMode::RestDiagnostic {
                    one_count_fallback: accumulator.one_count_fallback,
                },
            )?;
            return Ok(StationarityWorkPoll::Pending);
        }

        let retained_sample_count = u16::try_from(observations.observation_len())
            .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
        let decision = accumulator.decision(retained_sample_count);
        self.output = StationarityWorkOutput::RestDecision(decision);
        self.stage = StationarityWorkStage::RestDecisionReady;
        Ok(StationarityWorkPoll::RestDiagnostic(decision))
    }

    fn resume_rest_decision<const LEVELS: usize, S>(
        &mut self,
        observations: &S,
        blocker: &mut DyadicBlocker<LEVELS>,
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError>
    where
        S: VelocityObservationSource + ?Sized,
    {
        let StationarityWorkOutput::RestDecision(decision) = self.output else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let witness_index =
            if decision.disposition == RestSelectionDisposition::StatisticalAcceptance {
                Some(
                    u8::try_from(decision.selected_mask.trailing_zeros())
                        .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?,
                )
            } else if decision.disposition == RestSelectionDisposition::Rejected
                && decision.structural_codes[0] == RestStructuralCode::Usable
            {
                Some(0)
            } else {
                None
            };
        let Some(witness_index) = witness_index else {
            self.stage = StationarityWorkStage::Done;
            return Ok(StationarityWorkPoll::RestComplete(
                RestSelectionCompletion {
                    selected: None,
                    exact_velocity_interval_q: None,
                    disposition: decision.disposition,
                    recent_structural_code: decision.structural_codes[0],
                },
            ));
        };

        let suffix_len = *VELOCITY_SUFFIX_LENGTHS
            .get(usize::from(witness_index))
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        if decision.disposition == RestSelectionDisposition::StatisticalAcceptance
            && suffix_len != decision.selected_suffix_len
        {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        self.start_suffix(
            observations,
            blocker,
            witness_index,
            suffix_len,
            StationarityWorkMode::RestWitness,
        )?;
        Ok(StationarityWorkPoll::Pending)
    }

    fn finish_rest_witness(
        &mut self,
        selected: StationarySuffix,
        interval_q: [i64; 2],
    ) -> Result<StationarityWorkPoll, VelocityMeasurementError> {
        let StationarityWorkOutput::RestDecision(decision) = self.output else {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        };
        let witness_index =
            if decision.disposition == RestSelectionDisposition::StatisticalAcceptance {
                usize::try_from(decision.selected_mask.trailing_zeros())
                    .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?
            } else if decision.disposition == RestSelectionDisposition::Rejected
                && decision.structural_codes[0] == RestStructuralCode::Usable
            {
                0
            } else {
                return Err(VelocityMeasurementError::ArithmeticOverflow);
            };
        if selected.suffix_len != VELOCITY_SUFFIX_LENGTHS[witness_index]
            || decision.intervals_q.get(witness_index).copied() != Some(interval_q)
        {
            return Err(VelocityMeasurementError::ArithmeticOverflow);
        }
        self.output = StationarityWorkOutput::Selected(selected);
        self.stage = StationarityWorkStage::Done;
        Ok(StationarityWorkPoll::RestComplete(
            RestSelectionCompletion {
                selected: Some(selected),
                exact_velocity_interval_q: (decision.disposition
                    == RestSelectionDisposition::StatisticalAcceptance)
                    .then_some(interval_q),
                disposition: decision.disposition,
                recent_structural_code: decision.structural_codes[0],
            },
        ))
    }
}

fn work_observations_per_step(mode: StationarityWorkMode) -> usize {
    if mode == StationarityWorkMode::RestWitness {
        REST_WITNESS_OBSERVATIONS_PER_STEP
    } else {
        STATIONARITY_WORK_OBSERVATIONS_PER_STEP
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RestExclusion {
    NoBlockingPlateau,
    TooFewDecorrelatedBlocks,
    ResolvedSlope,
    StructuredResidual,
}

impl From<RestExclusion> for StationarityExclusion {
    fn from(value: RestExclusion) -> Self {
        match value {
            RestExclusion::NoBlockingPlateau => Self::NoBlockingPlateau,
            RestExclusion::TooFewDecorrelatedBlocks => Self::TooFewDecorrelatedBlocks,
            RestExclusion::ResolvedSlope => Self::ResolvedSlope,
            RestExclusion::StructuredResidual => Self::StructuredResidual,
        }
    }
}

fn rest_structural_code(exclusion: RestExclusion) -> RestStructuralCode {
    match exclusion {
        RestExclusion::NoBlockingPlateau => RestStructuralCode::NoBlockingPlateau,
        RestExclusion::TooFewDecorrelatedBlocks => RestStructuralCode::TooFewDecorrelatedBlocks,
        RestExclusion::ResolvedSlope => RestStructuralCode::ResolvedSlope,
        RestExclusion::StructuredResidual => RestStructuralCode::StructuredResidual,
    }
}

impl Default for StationarityWork {
    fn default() -> Self {
        Self::new()
    }
}

fn first_available_suffix(
    observations: &(impl VelocityObservationSource + ?Sized),
) -> Option<(u8, u16)> {
    VELOCITY_SUFFIX_LENGTHS
        .iter()
        .copied()
        .enumerate()
        .find(|(_, suffix_len)| usize::from(*suffix_len) <= observations.observation_len())
        .and_then(|(index, suffix_len)| u8::try_from(index).ok().map(|index| (index, suffix_len)))
}

/// Random-access source for a fixed chronological velocity capture.
pub trait VelocityObservationSource {
    /// Number of chronological observations available.
    fn observation_len(&self) -> usize;

    /// Return one chronological observation.
    fn observation(&self, index: usize) -> Option<RawVelocityObservation>;
}

impl VelocityObservationSource for [RawVelocityObservation] {
    fn observation_len(&self) -> usize {
        self.len()
    }

    fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
        self.get(index).copied()
    }
}

/// Select the longest contiguous stationary suffix using one reusable blocker.
pub fn select_stationary_suffix<const LEVELS: usize>(
    observations: &[RawVelocityObservation],
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityResult, VelocityMeasurementError> {
    select_stationary_suffix_from_source(observations, blocker)
}

/// Analyze every available declared suffix without changing the binding verdict.
pub fn analyze_stationarity_diagnostics<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityDiagnostics, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    let binding = select_stationary_suffix_from_source(observations, blocker)?;
    let mut suffixes = [None; VELOCITY_SUFFIX_LENGTHS.len()];
    for (suffix_index, suffix_len) in VELOCITY_SUFFIX_LENGTHS.iter().copied().enumerate() {
        if usize::from(suffix_len) > observations.observation_len() {
            continue;
        }
        suffixes[suffix_index] = Some(analyze_suffix_diagnostic(
            observations,
            blocker,
            suffix_len,
        )?);
    }
    Ok(StationarityDiagnostics { binding, suffixes })
}

fn analyze_suffix_diagnostic<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
    suffix_len: u16,
) -> Result<SuffixDiagnostic, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    *blocker = DyadicBlocker::new();
    let suffix_start = observations
        .observation_len()
        .checked_sub(usize::from(suffix_len))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    for index in suffix_start..observations.observation_len() {
        blocker.observe(
            observations
                .observation(index)
                .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
        )?;
    }
    let Some(selection) = blocker.select_plateau() else {
        return Ok(SuffixDiagnostic {
            suffix_len,
            selection: None,
            usable_plateau: false,
            slope_passed: false,
            lag_one_passed: false,
            lag_one_q: 0,
            settled_rate_interval_q: [0, 0],
            slope_q: 0,
            slope_half_width_q: 0,
            slope_rejection_margin_q: 0,
            lag_one_rejection_margin_q: 0,
        });
    };
    let block_observations = 1usize
        .checked_shl(u32::from(selection.level))
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let block_observations_u32 = u32::try_from(block_observations)
        .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
    if u32::from(suffix_len) % block_observations_u32 != 0
        || selection.block_count != u32::from(suffix_len) / block_observations_u32
    {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let settled = SettledRateContext {
        mean_rate_q: selection.exact_totals.rate_q()?,
        half_width_rate_q: estimate_confidence(selection, 3)?.half_width_rate_q,
    };
    let metrics = calculate_stationarity_metrics_from_source(
        observations,
        suffix_start,
        usize::from(suffix_len),
        block_observations,
        3,
        selection.variance_of_mean_q2,
        settled,
    )
    .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
    suffix_diagnostic_from_metrics(selection, suffix_len, metrics)
}

fn suffix_diagnostic_from_metrics(
    selection: PlateauSelection,
    suffix_len: u16,
    metrics: StationarityMetrics,
) -> Result<SuffixDiagnostic, VelocityMeasurementError> {
    let confidence = estimate_confidence(selection, 3)?;
    let mean_rate_q = selection.exact_totals.rate_q()?;
    let half_width = i64::try_from(confidence.half_width_rate_q)
        .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?;
    let low = mean_rate_q
        .checked_sub(half_width)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    let high = mean_rate_q
        .checked_add(half_width)
        .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
    Ok(SuffixDiagnostic {
        suffix_len,
        selection: Some(selection),
        usable_plateau: true,
        slope_passed: metrics.slope_passed,
        lag_one_passed: metrics.lag_one_passed,
        lag_one_q: metrics.lag_one_q,
        settled_rate_interval_q: [low, high],
        slope_rejection_margin_q: metrics.slope_rejection_margin_q,
        lag_one_rejection_margin_q: metrics.lag_one_rejection_margin_q,
        slope_q: metrics.slope_q,
        slope_half_width_q: metrics.slope_half_width_q,
    })
}

/// Re-evaluate the lag-one residual gate for a declared confidence family.
///
/// Uses the binding classifier's estimate, population, null and scaled half-width.
pub fn lag_one_passes_family(
    lag_one_q: i32,
    block_count: u32,
    family_size: u16,
) -> Result<bool, VelocityMeasurementError> {
    if block_count < 3 {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    let multiplier = confidence_multiplier_95_df15(family_size)?;
    let block_count = block_count as f32;
    let lag_one = lag_one_q as f32 / LAG_ONE_Q_SCALE;
    let lag_null = -1.0 / block_count;
    let lag_half_width = multiplier / libm::sqrtf(block_count);
    if !lag_one.is_finite() || !lag_half_width.is_finite() {
        return Err(VelocityMeasurementError::ArithmeticOverflow);
    }
    Ok(lag_half_width - libm::fabsf(lag_one - lag_null) >= 0.0)
}

/// Select the longest stationary suffix from a fixed random-access source.
pub fn select_stationary_suffix_from_source<const LEVELS: usize, S>(
    observations: &S,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<StationarityResult, VelocityMeasurementError>
where
    S: VelocityObservationSource + ?Sized,
{
    let mut selected: Option<StationarySuffix> = None;
    let mut tested_suffixes = 0u8;
    let mut last_exclusion = StationarityExclusion::TooFewDecorrelatedBlocks;

    for suffix_len in VELOCITY_SUFFIX_LENGTHS {
        let suffix_len = usize::from(suffix_len);
        if suffix_len > observations.observation_len() {
            continue;
        }
        tested_suffixes = tested_suffixes
            .checked_add(1)
            .ok_or(VelocityMeasurementError::ArithmeticOverflow)?;
        *blocker = DyadicBlocker::new();
        let suffix_start = observations.observation_len() - suffix_len;
        for index in suffix_start..observations.observation_len() {
            blocker.observe(
                observations
                    .observation(index)
                    .ok_or(VelocityMeasurementError::ArithmeticOverflow)?,
            )?;
        }

        let Some(selection) = blocker.select_plateau() else {
            if let Some(mut usable) = selected {
                usable.tested_suffixes = tested_suffixes;
                return Ok(StationarityResult::Usable(usable));
            }
            last_exclusion = if blocker.has_two_doubling_plateau() {
                StationarityExclusion::TooFewDecorrelatedBlocks
            } else {
                StationarityExclusion::NoBlockingPlateau
            };
            continue;
        };
        let Some(block_observations) = 1usize.checked_shl(u32::from(selection.level)) else {
            return Ok(StationarityResult::Excluded(
                StationarityExclusion::Arithmetic,
            ));
        };
        let settled = SettledRateContext {
            mean_rate_q: selection.exact_totals.rate_q()?,
            half_width_rate_q: estimate_confidence(selection, 3)?.half_width_rate_q,
        };
        let metrics = match analyze_stationarity_from_source(
            observations,
            suffix_start,
            suffix_len,
            block_observations,
            3,
            selection.variance_of_mean_q2,
            settled,
        ) {
            Ok(metrics) => metrics,
            Err(exclusion) => {
                if let Some(mut usable) = selected {
                    usable.tested_suffixes = tested_suffixes;
                    return Ok(StationarityResult::Usable(usable));
                }
                last_exclusion = exclusion;
                continue;
            }
        };
        selected = Some(StationarySuffix {
            selection,
            suffix_len: u16::try_from(suffix_len)
                .map_err(|_| VelocityMeasurementError::ArithmeticOverflow)?,
            slope_q: metrics.slope_q,
            slope_half_width_q: metrics.slope_half_width_q,
            lag_one_q: metrics.lag_one_q,
            lag_one_half_width_q: metrics.lag_one_half_width_q,
            residual_magnitude_q2: metrics.residual_magnitude_q2,
            span_x2_us: metrics.span_x2_us,
            drift_ratio_ppm: metrics.drift_ratio_ppm,
            tested_suffixes,
        });
    }

    Ok(selected.map_or(
        StationarityResult::Excluded(last_exclusion),
        StationarityResult::Usable,
    ))
}

fn analyze_stationarity_from_source(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    suffix_len: usize,
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
    settled: SettledRateContext,
) -> Result<StationarityMetrics, StationarityExclusion> {
    let mut metrics = calculate_stationarity_metrics_from_source(
        observations,
        suffix_start,
        suffix_len,
        block_observations,
        family_size,
        blocking_variance_of_mean_q2,
        settled,
    )?;
    classify_moving_metrics(&mut metrics, settled)?;
    Ok(metrics)
}

fn calculate_stationarity_metrics_from_source(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    suffix_len: usize,
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
    settled: SettledRateContext,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if block_observations == 0 {
        return Err(StationarityExclusion::Arithmetic);
    }
    let block_count = suffix_len / block_observations;
    if block_count < VELOCITY_MIN_SELECTED_BLOCKS as usize {
        return Err(StationarityExclusion::TooFewDecorrelatedBlocks);
    }

    let mut cumulative_elapsed_us = 0u64;
    let mut sum_x = 0i128;
    let mut sum_y = 0i128;
    let mut sum_x2 = 0i128;
    let mut sum_xy = 0i128;
    for block_index in 0..block_count {
        let block = rate_block_at(observations, suffix_start, block_index, block_observations)?;
        let midpoint_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us / 2)
            .ok_or(StationarityExclusion::Arithmetic)?;
        cumulative_elapsed_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let x = i128::from(midpoint_us);
        let y = i128::from(
            block
                .rate_q()
                .map_err(|_| StationarityExclusion::Arithmetic)?,
        );
        sum_x = sum_x
            .checked_add(x)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_y = sum_y
            .checked_add(y)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_x2 = sum_x2
            .checked_add(x.checked_mul(x).ok_or(StationarityExclusion::Arithmetic)?)
            .ok_or(StationarityExclusion::Arithmetic)?;
        sum_xy = sum_xy
            .checked_add(x.checked_mul(y).ok_or(StationarityExclusion::Arithmetic)?)
            .ok_or(StationarityExclusion::Arithmetic)?;
    }

    let n = i128::try_from(block_count).map_err(|_| StationarityExclusion::Arithmetic)?;
    let denominator = n
        .checked_mul(sum_x2)
        .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_x)?))
        .ok_or(StationarityExclusion::Arithmetic)?;
    if denominator <= 0 {
        return Err(StationarityExclusion::Arithmetic);
    }
    let numerator = n
        .checked_mul(sum_xy)
        .and_then(|value| value.checked_sub(sum_x.checked_mul(sum_y)?))
        .ok_or(StationarityExclusion::Arithmetic)?;
    let slope_per_us = numerator as f32 / denominator as f32;
    let intercept = (sum_y as f32 - slope_per_us * sum_x as f32) / block_count as f32;
    if !slope_per_us.is_finite() || !intercept.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }

    let mut residual_sum_q2 = 0.0f32;
    let mut lag_product_sum_q2 = 0.0f32;
    let mut previous_residual = None;
    let mut first_block_elapsed_us = 0u64;
    let mut last_block_elapsed_us = 0u64;
    cumulative_elapsed_us = 0;
    for block_index in 0..block_count {
        let block = rate_block_at(observations, suffix_start, block_index, block_observations)?;
        if block_index == 0 {
            first_block_elapsed_us = block.elapsed_us;
        }
        last_block_elapsed_us = block.elapsed_us;
        let midpoint_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us / 2)
            .ok_or(StationarityExclusion::Arithmetic)?;
        cumulative_elapsed_us = cumulative_elapsed_us
            .checked_add(block.elapsed_us)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let rate_q = block
            .rate_q()
            .map_err(|_| StationarityExclusion::Arithmetic)? as f32;
        let residual = rate_q - (intercept + slope_per_us * midpoint_us as f32);
        residual_sum_q2 += residual * residual;
        if let Some(previous) = previous_residual {
            lag_product_sum_q2 += residual * previous;
        }
        previous_residual = Some(residual);
    }
    let span_x2_us = cumulative_elapsed_us
        .checked_mul(2)
        .and_then(|value| value.checked_sub(first_block_elapsed_us))
        .and_then(|value| value.checked_sub(last_block_elapsed_us))
        .ok_or(StationarityExclusion::Arithmetic)?;
    calculate_stationarity_metrics(
        block_count,
        denominator,
        slope_per_us,
        residual_sum_q2,
        lag_product_sum_q2,
        family_size,
        blocking_variance_of_mean_q2,
        span_x2_us,
        settled,
    )
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SettledRateContext {
    /// Exact settled mean rate in fixed-point rate units.
    pub mean_rate_q: i64,
    /// Outward-rounded confidence half-width in the same units.
    pub half_width_rate_q: u64,
}

pub(crate) fn drift_ratio_ppm(slope_q: i64, span_x2_us: u64, mean_rate_q: i64) -> u32 {
    let denominator = u128::from(mean_rate_q.unsigned_abs()).saturating_mul(2);
    if denominator == 0 {
        return u32::MAX;
    }
    let numerator = u128::from(slope_q.unsigned_abs()).saturating_mul(u128::from(span_x2_us));
    let ratio = numerator.div_ceil(denominator);
    if ratio > u128::from(u32::MAX) {
        u32::MAX
    } else {
        ratio as u32
    }
}

#[allow(clippy::too_many_arguments)]
fn calculate_stationarity_metrics(
    block_count: usize,
    denominator: i128,
    slope_per_us: f32,
    residual_sum_q2: f32,
    lag_product_sum_q2: f32,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
    span_x2_us: u64,
    settled: SettledRateContext,
) -> Result<StationarityMetrics, StationarityExclusion> {
    if block_count < 3 || !residual_sum_q2.is_finite() || !lag_product_sum_q2.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let multiplier = confidence_multiplier_95_df15(family_size)
        .map_err(|_| StationarityExclusion::Arithmetic)?;
    let degrees_of_freedom = (block_count - 2) as f32;
    let residual_variance_q2 = residual_sum_q2 / degrees_of_freedom;
    let blocker_variance_q2 = blocking_variance_of_mean_q2 as f32 * block_count as f32;
    if !blocker_variance_q2.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let slope_variance_q2 = libm::fmaxf(residual_variance_q2, blocker_variance_q2);
    let sxx_us2 = denominator as f32 / block_count as f32;
    let slope_per_second = slope_per_us * MICROS_PER_SECOND;
    let slope_half_width =
        multiplier * libm::sqrtf(slope_variance_q2 / sxx_us2) * MICROS_PER_SECOND;
    if !slope_per_second.is_finite() || !slope_half_width.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let slope_rejection_margin = slope_half_width - libm::fabsf(slope_per_second);

    let lag_one = if residual_sum_q2 == 0.0 {
        0.0
    } else {
        lag_product_sum_q2 / residual_sum_q2
    };
    let lag_null = -1.0 / block_count as f32;
    let lag_half_width = multiplier / libm::sqrtf(block_count as f32);
    if !lag_one.is_finite() || !lag_half_width.is_finite() {
        return Err(StationarityExclusion::Arithmetic);
    }
    let lag_rejection_margin = lag_half_width - libm::fabsf(lag_one - lag_null);

    Ok(StationarityMetrics {
        slope_q: rounded_i64(slope_per_second)?,
        slope_half_width_q: ceiled_u64(slope_half_width)?,
        lag_one_q: rounded_i32(lag_one * LAG_ONE_Q_SCALE)?,
        lag_one_half_width_q: ceiled_u32(lag_half_width * LAG_ONE_Q_SCALE)?,
        residual_magnitude_q2: libm::ceilf(residual_sum_q2 / block_count as f32) as u128,
        slope_passed: slope_rejection_margin >= 0.0,
        lag_one_passed: lag_rejection_margin >= 0.0,
        slope_rejection_margin_q: rounded_i64(slope_rejection_margin)?,
        lag_one_rejection_margin_q: rounded_i32(lag_rejection_margin * LAG_ONE_Q_SCALE)?,
        span_x2_us,
        drift_ratio_ppm: drift_ratio_ppm(
            rounded_i64(slope_per_second)?,
            span_x2_us,
            settled.mean_rate_q,
        ),
    })
}

pub(crate) const MAX_MOVING_DRIFT_PPM: u32 = 200_000;

fn classify_moving_metrics(
    metrics: &mut StationarityMetrics,
    settled: SettledRateContext,
) -> Result<(), StationarityExclusion> {
    let interval_contains_zero = settled.mean_rate_q.unsigned_abs() <= settled.half_width_rate_q;
    if interval_contains_zero {
        metrics.drift_ratio_ppm = NO_REPORTABLE_DRIFT_RATIO_PPM;
    } else if metrics.drift_ratio_ppm > MAX_MOVING_DRIFT_PPM {
        return Err(StationarityExclusion::ExcessiveDrift);
    }
    if !metrics.lag_one_passed {
        return Err(StationarityExclusion::StructuredResidual);
    }
    Ok(())
}

fn classify_rest_metrics(
    metrics: StationarityMetrics,
) -> Result<StationarityMetrics, RestExclusion> {
    if !metrics.slope_passed {
        return Err(RestExclusion::ResolvedSlope);
    }
    if !metrics.lag_one_passed {
        return Err(RestExclusion::StructuredResidual);
    }
    Ok(metrics)
}

#[cfg(test)]
fn analyze_stationarity_with_blocking_variance(
    observations: &[RawVelocityObservation],
    block_observations: usize,
    family_size: u16,
    blocking_variance_of_mean_q2: u128,
) -> Result<StationarityMetrics, StationarityExclusion> {
    let settled = SettledRateContext {
        mean_rate_q: rate_block_at(observations, 0, 0, observations.len())?
            .rate_q()
            .map_err(|_| StationarityExclusion::Arithmetic)?,
        half_width_rate_q: 0,
    };
    analyze_stationarity_from_source(
        observations,
        0,
        observations.len(),
        block_observations,
        family_size,
        blocking_variance_of_mean_q2,
        settled,
    )
}

fn rate_block_at(
    observations: &(impl VelocityObservationSource + ?Sized),
    suffix_start: usize,
    block_index: usize,
    block_observations: usize,
) -> Result<RateBlock, StationarityExclusion> {
    let start = suffix_start
        .checked_add(
            block_index
                .checked_mul(block_observations)
                .ok_or(StationarityExclusion::Arithmetic)?,
        )
        .ok_or(StationarityExclusion::Arithmetic)?;
    let end = start
        .checked_add(block_observations)
        .ok_or(StationarityExclusion::Arithmetic)?;
    let mut block: Option<RateBlock> = None;
    for index in start..end {
        let observation = observations
            .observation(index)
            .ok_or(StationarityExclusion::Arithmetic)?;
        let next = RateBlock::from_observation(observation)
            .map_err(|_| StationarityExclusion::Arithmetic)?;
        block = Some(match block {
            Some(current) => current
                .checked_merge(next)
                .map_err(|_| StationarityExclusion::Arithmetic)?,
            None => next,
        });
    }
    block.ok_or(StationarityExclusion::Arithmetic)
}

fn rounded_i64(value: f32) -> Result<i64, StationarityExclusion> {
    if !value.is_finite() || value <= i64::MIN as f32 || value >= i64::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::roundf(value) as i64)
}

fn rounded_i32(value: f32) -> Result<i32, StationarityExclusion> {
    if !value.is_finite() || value <= i32::MIN as f32 || value >= i32::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::roundf(value) as i32)
}

fn ceiled_u64(value: f32) -> Result<u64, StationarityExclusion> {
    if !value.is_finite() || value < 0.0 || value >= u64::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::ceilf(value) as u64)
}

fn ceiled_u32(value: f32) -> Result<u32, StationarityExclusion> {
    if !value.is_finite() || value < 0.0 || value >= u32::MAX as f32 {
        return Err(StationarityExclusion::Arithmetic);
    }
    Ok(libm::ceilf(value) as u32)
}

#[cfg(test)]
#[path = "stationarity_tests.rs"]
mod tests;
