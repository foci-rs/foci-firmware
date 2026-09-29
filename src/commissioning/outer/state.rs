//! Outer commissioning phase state and phase-local data.

use crate::autotune::{CommissionError, CommissionPhase};
use crate::commissioning::types::{OuterSafetyFault, RestSelectionOwner};

use crate::autotune::velocity_measurement::{
    DyadicBlocker, RawVelocityObservation, StationarityWork, VelocityMeasurementError,
};
use crate::commissioning::outer::velocity::recovery::{
    RungOriginRecoveryReport, RungOriginRecoveryRuntime,
};
use crate::commissioning::outer::velocity::velocity_clamp::{
    VelocityCurrentObserver, VelocityCurrentPhaseEvidence,
};
use crate::commissioning::outer::velocity::{
    integral_evidence::ExactRateInterval, integral_plan::VelocityIntegralExecutionPlan,
    sweep_evidence::VelocityObservationClass,
};

use super::safety::OuterSafetyLimits;

pub(crate) const VELOCITY_OBSERVATION_CAPACITY: usize = 256;
/// Dyadic levels required to analyze every retained suffix through 256 samples.
pub(crate) const VELOCITY_SWEEP_BLOCK_LEVELS: usize = 5;

#[derive(Debug)]
pub(crate) struct VelocityObservationRing {
    delta_counts: [i32; VELOCITY_OBSERVATION_CAPACITY],
    elapsed_us: [u16; VELOCITY_OBSERVATION_CAPACITY],
    next: u16,
    len: u16,
}

impl VelocityObservationRing {
    pub(crate) const fn new() -> Self {
        Self {
            delta_counts: [0; VELOCITY_OBSERVATION_CAPACITY],
            elapsed_us: [0; VELOCITY_OBSERVATION_CAPACITY],
            next: 0,
            len: 0,
        }
    }

    pub(crate) fn push(
        &mut self,
        delta_counts: i32,
        elapsed_us: u16,
    ) -> Result<(), VelocityMeasurementError> {
        if elapsed_us == 0 {
            return Err(VelocityMeasurementError::ZeroElapsed);
        }
        let index = usize::from(self.next);
        self.delta_counts[index] = delta_counts;
        self.elapsed_us[index] = elapsed_us;
        self.next = if index + 1 == VELOCITY_OBSERVATION_CAPACITY {
            0
        } else {
            self.next + 1
        };
        self.len = self
            .len
            .saturating_add(1)
            .min(VELOCITY_OBSERVATION_CAPACITY as u16);
        Ok(())
    }

    pub(crate) const fn len(&self) -> usize {
        self.len as usize
    }

    pub(crate) fn clear(&mut self) {
        self.next = 0;
        self.len = 0;
    }

    #[cfg(test)]
    pub(crate) fn iter(&self) -> VelocityObservationIter<'_> {
        let start = if self.len() == VELOCITY_OBSERVATION_CAPACITY {
            usize::from(self.next)
        } else {
            0
        };
        VelocityObservationIter {
            ring: self,
            start,
            offset: 0,
        }
    }
}

#[cfg(test)]
pub(crate) struct VelocityObservationIter<'a> {
    ring: &'a VelocityObservationRing,
    start: usize,
    offset: usize,
}

#[cfg(test)]
impl Iterator for VelocityObservationIter<'_> {
    type Item = RawVelocityObservation;

    fn next(&mut self) -> Option<Self::Item> {
        if self.offset == self.ring.len() {
            return None;
        }
        let index = (self.start + self.offset) % VELOCITY_OBSERVATION_CAPACITY;
        self.offset += 1;
        Some(RawVelocityObservation {
            delta_counts: self.ring.delta_counts[index],
            elapsed_us: u32::from(self.ring.elapsed_us[index]),
        })
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        let remaining = self.ring.len() - self.offset;
        (remaining, Some(remaining))
    }
}

#[cfg(test)]
impl ExactSizeIterator for VelocityObservationIter<'_> {}

impl crate::autotune::velocity_measurement::VelocityObservationSource for VelocityObservationRing {
    fn observation_len(&self) -> usize {
        self.len()
    }

    fn observation(&self, index: usize) -> Option<RawVelocityObservation> {
        if index >= self.len() {
            return None;
        }
        let start = if self.len() == VELOCITY_OBSERVATION_CAPACITY {
            usize::from(self.next)
        } else {
            0
        };
        let source_index = (start + index) % VELOCITY_OBSERVATION_CAPACITY;
        Some(RawVelocityObservation {
            delta_counts: self.delta_counts[source_index],
            elapsed_us: u32::from(self.elapsed_us[source_index]),
        })
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocitySettleTransition {
    pub(crate) sample_index: u16,
    pub(crate) elapsed_us: u32,
    pub(crate) delta_counts: i32,
    pub(crate) displacement_counts: i32,
}

impl VelocitySettleTransition {
    const EMPTY: Self = Self {
        sample_index: 0,
        elapsed_us: 0,
        delta_counts: 0,
        displacement_counts: 0,
    };
}

/// Incremental whole-window metrics for one zero-target capture.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocitySettleMetrics {
    pub(crate) sample_count: u16,
    pub(crate) duration_us: u32,
    pub(crate) net_displacement_counts: i32,
    pub(crate) minimum_displacement_counts: i32,
    pub(crate) maximum_displacement_counts: i32,
    pub(crate) transition_count: u16,
    pub(crate) first_transition: VelocitySettleTransition,
    pub(crate) last_transition: VelocitySettleTransition,
}

impl VelocitySettleMetrics {
    pub(crate) const EMPTY: Self = Self {
        sample_count: 0,
        duration_us: 0,
        net_displacement_counts: 0,
        minimum_displacement_counts: 0,
        maximum_displacement_counts: 0,
        transition_count: 0,
        first_transition: VelocitySettleTransition::EMPTY,
        last_transition: VelocitySettleTransition::EMPTY,
    };

    /// # Errors
    ///
    /// Returns [`CommissionError::InvalidSchedule`] if the `sample_count`,
    /// `duration_us`, or `transition_count` counters overflow their
    /// representation, and [`CommissionError::NetDisplacementOverflow`] if
    /// the `net_displacement_counts` accumulator does.
    pub(crate) fn observe(
        &mut self,
        delta_counts: i32,
        elapsed_us: u32,
    ) -> Result<(), CommissionError> {
        self.sample_count = self
            .sample_count
            .checked_add(1)
            .ok_or(CommissionError::InvalidSchedule)?;
        self.duration_us = self
            .duration_us
            .checked_add(elapsed_us)
            .ok_or(CommissionError::InvalidSchedule)?;
        self.net_displacement_counts = self
            .net_displacement_counts
            .checked_add(delta_counts)
            .ok_or(CommissionError::NetDisplacementOverflow)?;
        self.minimum_displacement_counts = self
            .minimum_displacement_counts
            .min(self.net_displacement_counts);
        self.maximum_displacement_counts = self
            .maximum_displacement_counts
            .max(self.net_displacement_counts);
        if delta_counts != 0 {
            self.transition_count = self
                .transition_count
                .checked_add(1)
                .ok_or(CommissionError::InvalidSchedule)?;
            let transition = VelocitySettleTransition {
                sample_index: self.sample_count,
                elapsed_us: self.duration_us,
                delta_counts,
                displacement_counts: self.net_displacement_counts,
            };
            if self.transition_count == 1 {
                self.first_transition = transition;
            }
            self.last_transition = transition;
        }
        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct PositiveIRestBoundary {
    pub(crate) rung_index: u8,
    pub(crate) slot: u8,
    pub(crate) i_raw: u16,
    pub(crate) direction: i8,
    pub(crate) active_stationarity: crate::autotune::velocity_measurement::StationarityResult,
    pub(crate) active_settle: VelocitySettleMetrics,
    pub(crate) zero_phase_current: VelocityCurrentPhaseEvidence,
    pub(crate) cleanup_confirmed: bool,
}

/// Phase-local state for the Integral velocity-integral response sweep.
#[derive(Debug, defmt::Format)]
pub(crate) enum VelocityIntegralState {
    Setup,
    PrepareMovingSlot {
        reset_rung: bool,
    },
    CommandTarget {
        entry_transition_us: u32,
    },
    Capture {
        next_sample_us: u64,
        deadline_us: u64,
        entry_transition_us: u32,
    },
    PrepareZeroSettle {
        entry_transition_us: u32,
    },
    AnalyzeMoving {
        settle_deadline_us: u64,
        analysis_deadline_us: u64,
    },
    BeginSettleCapture {
        settle_deadline_us: u64,
    },
    SettleCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    AnalyzeSettle {
        analysis_deadline_us: u64,
    },
    BeginPositiveIRestCleanup,
    PositiveIRestCleanupCapture {
        next_sample_us: u64,
        deadline_us: u64,
    },
    AnalyzePositiveIRestCleanup {
        analysis_deadline_us: u64,
    },
    Advance,
    AnalyzeRung,
    RecoverRung(RungOriginRecoveryRuntime),
    FinalizeRecovery(RungOriginRecoveryReport),
    FaultTerminal {
        error: CommissionError,
    },
    Terminal,
}

#[derive(Debug, defmt::Format)]
pub(crate) struct VelocityIntegralRuntime {
    pub(crate) plan: VelocityIntegralExecutionPlan,
    pub(crate) integrity: super::velocity::integral_reporting::VelocityIntegralIntegrity,
    pub(crate) rung_index: u8,
    pub(crate) slot: u8,
    pub(crate) previous_encoder_raw: u32,
    pub(crate) previous_read_us: u64,
    pub(crate) run_started_us: u64,
    pub(crate) current_observer: VelocityCurrentObserver,
    pub(crate) observation_current_headroom: bool,
    pub(crate) settle_displacement_counts: i8,
    pub(crate) settle_min_displacement_counts: i8,
    pub(crate) settle_max_displacement_counts: i8,
    pub(crate) settle_metrics: VelocitySettleMetrics,
    pub(crate) positive_i_rest_boundary: Option<PositiveIRestBoundary>,
    pub(crate) positive_i_rest_boundary_after_sufficiency: bool,
    #[defmt(Debug2Format)]
    pub(crate) observations: VelocityObservationRing,
    #[defmt(Debug2Format)]
    pub(crate) blocker: DyadicBlocker<VELOCITY_SWEEP_BLOCK_LEVELS>,
    #[defmt(Debug2Format)]
    pub(crate) analysis: StationarityWork,
    pub(crate) rung_classes: [VelocityObservationClass; 8],
    pub(crate) rung_intervals: [ExactRateInterval; 8],
    pub(crate) rung_blocking_levels: u32,
    pub(crate) eligible_masks: [u32; 2],
    pub(crate) bookend_available_mask: u8,
    pub(crate) current_terminus: Option<u8>,
    pub(crate) target_terminus: Option<u8>,
    pub(crate) opening_direction_mask: u8,
    pub(crate) recovery_failure: bool,
    pub(crate) recovery_unavailable: bool,
    pub(crate) rest_unconfirmed: bool,
    pub(crate) rest_unconfirmed_owner: Option<RestSelectionOwner>,
    pub(crate) recovered_with_current_headroom: bool,
    pub(crate) emitted_observations: u16,
    pub(crate) emitted_rungs: u16,
}

impl VelocityIntegralRuntime {
    pub(crate) fn new(run_sequence: u32, plan: VelocityIntegralExecutionPlan) -> Self {
        Self {
            plan,
            integrity: super::velocity::integral_reporting::VelocityIntegralIntegrity::new(
                run_sequence,
                plan,
            ),
            rung_index: 0,
            slot: 0,
            previous_encoder_raw: 0,
            previous_read_us: 0,
            run_started_us: 0,
            current_observer: VelocityCurrentObserver::EMPTY,
            observation_current_headroom: false,
            settle_displacement_counts: 0,
            settle_min_displacement_counts: 0,
            settle_max_displacement_counts: 0,
            settle_metrics: VelocitySettleMetrics::EMPTY,
            positive_i_rest_boundary: None,
            positive_i_rest_boundary_after_sufficiency: false,
            observations: VelocityObservationRing::new(),
            blocker: DyadicBlocker::new(),
            analysis: StationarityWork::new(),
            rung_classes: [VelocityObservationClass::InvalidEvidenceFault; 8],
            rung_intervals: [ExactRateInterval::EMPTY; 8],
            rung_blocking_levels: 0,
            eligible_masks: [0; 2],
            bookend_available_mask: 0,
            current_terminus: None,
            target_terminus: None,
            opening_direction_mask: 0,
            recovery_failure: false,
            recovery_unavailable: false,
            rest_unconfirmed: false,
            rest_unconfirmed_owner: None,
            recovered_with_current_headroom: false,
            emitted_observations: 0,
            emitted_rungs: 0,
        }
    }
}

/// Outer commissioning phase state machine.
#[derive(Debug, defmt::Format)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum OuterPhase {
    VelocityIntegral {
        state: VelocityIntegralState,
        data: VelocityIntegralRuntime,
        safety: OuterSafetyLimits,
    },
    BreakawayCampaign {
        campaign: super::velocity::breakaway_campaign::BreakawayCampaign,
        workspace: super::velocity::breakaway_campaign::BreakawayProbeWorkspace,
        safety: OuterSafetyLimits,
    },
    RobustnessReversal {
        state: super::velocity::robustness_reversal::RobustnessReversalState,
        workspace: super::velocity::robustness_reversal::RobustnessReversalWorkspace,
        session: super::velocity::robustness_reversal::RobustnessReversalSession,
        safety: OuterSafetyLimits,
    },
    PositionTune {
        state: super::position_tune::PositionTuneState,
        safety: OuterSafetyLimits,
    },
    Failed(CommissionError),
    /// The position-P sweep failed for a specific diagnostic reason; behaves as [`Self::Failed`].
    PositionTuneFailed {
        /// `PositionTuneFailed` when `outcome` is `Some`, else the generic engine error.
        error: CommissionError,
        /// `None` for a generic engine-level failure, which reports outcome code 0.
        outcome: Option<crate::autotune::PositionTuneOutcome>,
        provenance: crate::commissioning::types::PositionTuneProvenance,
    },
    FailedWithSafety {
        error: CommissionError,
        safety_fault: OuterSafetyFault,
        /// Position-P evidence when the failing phase was `PositionTune`; `Default` otherwise.
        position_tune: crate::commissioning::types::PositionTuneProvenance,
    },
}

impl OuterPhase {
    /// Return the stable phase ID for the active non-terminal outer phase.
    ///
    /// `VelocityIntegral` dispatches on its plan's `acquisition_kind`.
    pub(crate) fn current_phase_id(&self) -> Option<CommissionPhase> {
        use super::velocity::integral_plan::VelocityIntegralAcquisitionKind;

        match self {
            Self::VelocityIntegral { data, .. } => Some(match data.plan.acquisition_kind {
                VelocityIntegralAcquisitionKind::ResponseLadder => {
                    CommissionPhase::IntegralGainResponseCharacterization
                }
                VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending
                | VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending => {
                    CommissionPhase::FixedGainAmplitudeValidation
                }
            }),
            Self::BreakawayCampaign { .. } => {
                Some(CommissionPhase::BreakawayProportionalAcquisition)
            }
            Self::RobustnessReversal { .. } => {
                Some(CommissionPhase::ReversalStandstillRobustnessGate)
            }
            Self::PositionTune { .. } => Some(CommissionPhase::PositionTune),
            Self::Failed(_) | Self::PositionTuneFailed { .. } | Self::FailedWithSafety { .. } => {
                None
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use core::mem::size_of;

    use super::{
        OuterPhase, VelocityIntegralRuntime, VelocityIntegralState, VelocitySettleMetrics,
    };
    use crate::autotune::{CommissionError, CommissionPhase};

    use crate::commissioning::outer::OuterSafetyLimits;
    use crate::commissioning::outer::velocity::integral_plan::{
        AcceptanceSource, IntegralSlotOrder, VelocityIntegralAcquisitionKind,
        VelocityIntegralExecutionPlan,
    };
    use crate::commissioning::outer::velocity::velocity_clamp::VelocityLimitEvidenceCapability;

    #[test]
    fn observe_reports_net_displacement_overflow_not_generic_schedule_fault() {
        let mut metrics = VelocitySettleMetrics::EMPTY;
        metrics.net_displacement_counts = i32::MAX - 1;
        let error = metrics.observe(2, 1_000).unwrap_err();
        assert_eq!(error, CommissionError::NetDisplacementOverflow);
    }

    fn minimal_velocity_integral_plan() -> VelocityIntegralExecutionPlan {
        VelocityIntegralExecutionPlan {
            slot_order: IntegralSlotOrder::ForwardFirst,
            plan_digest: 7,
            acceptance_plan_digest: 0,
            qualifier_bits: 0,
            fixed_p: 1_448,
            planned_velocity_mrev_s: 2_929,
            target_velocity_rpm: 176,
            pwm_hz: 25_000,
            encoder_counts_per_rev: 4_000,
            i_start: 1,
            positive_rung_count: 5,
            i_top_cap: 0,
            family_size: 28,
            moving_stroke_us: 512_000,
            zero_settle_us: 500_000,
            analysis_budget_us: 236_000,
            maximum_workflow_ms: 69_888,
            recovery_origin_band_counts: 1_000,
            recovery_nominal_slot_us: 1_816_958,
            recovery_maximum_slot_us: 3_000_000,
            recovery_slot_count: 7,
            hard_torque_limit: 2_816,
            usable_torque_limit: 2_534,
            limit_evidence_capability: VelocityLimitEvidenceCapability::BestEffortPolling,
            acquisition_kind: VelocityIntegralAcquisitionKind::ResponseLadder,
            acceptance_source: AcceptanceSource::NonBreakaway,
            amplitude_targets_rpm: [0; 5],
            selected_response_intervals_q: [[0; 2]; 2],
            recovery_lower_rate_q: [13_217_792, 12_827_648],
            recovery_quantization_exposed: true,
            probe_constrained_test_point: false,
        }
    }

    #[test]
    fn outer_phase_size_stays_within_reasonable_stack_budget() {
        assert!(
            size_of::<OuterPhase>() <= 4288,
            "OuterPhase grew to {} bytes",
            size_of::<OuterPhase>()
        );
    }

    fn velocity_integral_phase(acquisition_kind: VelocityIntegralAcquisitionKind) -> OuterPhase {
        let mut plan = minimal_velocity_integral_plan();
        plan.acquisition_kind = acquisition_kind;
        OuterPhase::VelocityIntegral {
            state: VelocityIntegralState::Setup,
            data: VelocityIntegralRuntime::new(0, plan),
            safety: OuterSafetyLimits::new(
                0,
                crate::commissioning::outer::OuterMotionBudget::firmware_caps(),
            ),
        }
    }

    #[test]
    fn outer_phase_id_maps_active_and_failed_phases() {
        assert_eq!(
            velocity_integral_phase(VelocityIntegralAcquisitionKind::ResponseLadder)
                .current_phase_id(),
            Some(CommissionPhase::IntegralGainResponseCharacterization)
        );
        assert_eq!(
            velocity_integral_phase(VelocityIntegralAcquisitionKind::FixedGainAmplitudeAscending)
                .current_phase_id(),
            Some(CommissionPhase::FixedGainAmplitudeValidation)
        );
        assert_eq!(
            velocity_integral_phase(VelocityIntegralAcquisitionKind::FixedGainAmplitudeDescending)
                .current_phase_id(),
            Some(CommissionPhase::FixedGainAmplitudeValidation)
        );
        assert_eq!(
            OuterPhase::Failed(CommissionError::Spi).current_phase_id(),
            None
        );
    }

    #[test]
    fn velocity_observation_ring_overwrites_oldest_and_iterates_chronologically() {
        let mut ring = super::VelocityObservationRing::new();
        for sample in 0..=super::VELOCITY_OBSERVATION_CAPACITY as i32 {
            ring.push(sample, 1_000).unwrap();
        }

        let observations: heapless::Vec<_, { super::VELOCITY_OBSERVATION_CAPACITY }> =
            ring.iter().collect();

        assert_eq!(ring.len(), super::VELOCITY_OBSERVATION_CAPACITY);
        assert_eq!(observations.len(), super::VELOCITY_OBSERVATION_CAPACITY);
        assert_eq!(observations.first().unwrap().delta_counts, 1);
        assert_eq!(observations.last().unwrap().delta_counts, 256);
        assert!(observations.iter().all(|item| item.elapsed_us == 1_000));
    }

    #[test]
    fn velocity_observation_ring_has_the_audited_split_array_layout() {
        assert_eq!(size_of::<[i32; 256]>(), 1_024);
        assert_eq!(size_of::<[u16; 256]>(), 512);
        assert_eq!(size_of::<super::VelocityObservationRing>(), 1_540);
    }
}
