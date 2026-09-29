//! Bounded post-capture analysis for the velocity sweep.

use foci_tmc4671::pid::PidPGain;

use crate::autotune::velocity_measurement::{
    DyadicBlocker, RestSelectionDecision, RestSelectionDisposition, RestStructuralCode,
    StationarityExclusion, StationarityResult, StationarityWork, StationarityWorkPoll,
    StationarySuffix, SuffixDiagnostic, VELOCITY_RATE_Q_SCALE, VelocityDirection,
    VelocityObservationSource, estimate_confidence, lag_one_passes_family,
};
use crate::autotune::{
    CommissionError, TmcMechanicalRpm, VelocityUnitError, equivalent_disturbance_from_rate,
    equivalent_disturbance_variance_from_rate,
};

use super::sweep_evidence::VelocityObservationInput;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct SettleConfirmation {
    pub(super) selected: Option<StationarySuffix>,
    pub(super) confirmed_rest: bool,
    pub(super) exact_velocity_interval_q: Option<[i64; 2]>,
    pub(super) selected_suffix_len: u16,
    pub(super) conditioned_rest_evidence: bool,
    pub(super) disposition: RestSelectionDisposition,
    pub(super) recent_structural_code: RestStructuralCode,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum ScoredRestPoll {
    Pending,
    Diagnostic(RestSelectionDecision),
    Complete(SettleConfirmation),
}

/// Signed mechanical direction: `1` for [`VelocityDirection::Forward`], `-1` for
/// [`VelocityDirection::Reverse`].
pub(super) const fn direction_sign(direction: VelocityDirection) -> i32 {
    match direction {
        VelocityDirection::Forward => 1,
        VelocityDirection::Reverse => -1,
    }
}

/// Frozen inputs [`assemble_velocity_observation_input`] needs from either the
/// sweep's per-rung plan or the breakaway probe's per-stroke session.
#[derive(Clone, Copy)]
pub(super) struct VelocityObservationInputRequest {
    /// Commanded mechanical direction.
    pub(super) direction: VelocityDirection,
    /// Velocity proportional gain used for the observation.
    pub(super) p: PidPGain,
    /// Target-independent stationary-suffix result.
    pub(super) stationarity: StationarityResult,
    /// Post-fit evidence retained when excessive drift rejected the suffix.
    pub(super) rejected_post_fit: Option<SuffixDiagnostic>,
    /// Drift ratio; replaced by the suffix ratio when `stationarity` is usable.
    pub(super) drift_ratio_ppm: u32,
    /// Largest sampled absolute velocity-PID output during the moving phase.
    pub(super) moving_pid_output_peak: u16,
    /// Unsigned commanded velocity magnitude.
    pub(super) target_velocity_rpm: TmcMechanicalRpm,
    /// Inclusive current-headroom terminus.
    pub(super) usable_torque_limit: u16,
    /// Hard controller-output clip boundary.
    pub(super) hard_torque_limit: u16,
    /// Whether this observation lacks the normal reversal lead-in.
    pub(super) initial_lead_in: bool,
    /// Confidence family size used for derived comparison intervals.
    pub(super) confidence_family_size: u16,
    /// Validated encoder counts per mechanical revolution.
    pub(super) counts_per_rev: u32,
}

/// Assemble one observation's classifier input from already-collected evidence.
///
/// Shared by the sweep and the breakaway probe.
pub(super) fn assemble_velocity_observation_input(
    request: VelocityObservationInputRequest,
) -> Result<VelocityObservationInput, CommissionError> {
    let predicted = predicted_torque_target_abs(request)?;

    let mut input = VelocityObservationInput {
        direction: request.direction,
        p: request.p,
        stationarity: request.stationarity,
        settled_rate_low_q: 0,
        settled_rate_high_q: 0,
        settled_rate_mean_q: 0,
        variance_of_mean_q2: 0,
        deficit_low_q: 0,
        deficit_high_q: 0,
        disturbance: None,
        disturbance_variance_q2: 0,
        predicted_torque_target_abs: predicted,
        usable_torque_limit: request.usable_torque_limit,
        hard_torque_limit: request.hard_torque_limit,
        initial_lead_in: request.initial_lead_in,
        drift_ratio_ppm: request.drift_ratio_ppm,
        post_fit_lag_one_state: 0,
    };
    populate_derived_observation_fields(&mut input, request)?;
    Ok(input)
}

/// Assemble probe diagnostics without allowing their derivation to bind probe control.
///
/// A failed comparison-only interval derivation leaves the derived fields zero.
pub(super) fn assemble_probe_observation_input(
    request: VelocityObservationInputRequest,
) -> Result<VelocityObservationInput, CommissionError> {
    let predicted = predicted_torque_target_abs(request)?;
    match assemble_velocity_observation_input(request) {
        Ok(input) => Ok(input),
        Err(_) => Ok(VelocityObservationInput {
            direction: request.direction,
            p: request.p,
            stationarity: request.stationarity,
            settled_rate_low_q: 0,
            settled_rate_high_q: 0,
            settled_rate_mean_q: 0,
            variance_of_mean_q2: 0,
            deficit_low_q: 0,
            deficit_high_q: 0,
            disturbance: None,
            disturbance_variance_q2: 0,
            predicted_torque_target_abs: predicted,
            usable_torque_limit: request.usable_torque_limit,
            hard_torque_limit: request.hard_torque_limit,
            initial_lead_in: request.initial_lead_in,
            drift_ratio_ppm: request.drift_ratio_ppm,
            post_fit_lag_one_state: 0,
        }),
    }
}

fn predicted_torque_target_abs(
    request: VelocityObservationInputRequest,
) -> Result<u16, CommissionError> {
    Ok(u32::from(request.p.raw())
        .checked_mul(request.target_velocity_rpm.raw().unsigned_abs())
        .and_then(|value| value.checked_add(255))
        .map(|value| value / 256)
        .ok_or(CommissionError::InvalidSchedule)?
        .max(u32::from(request.moving_pid_output_peak))
        .min(u32::from(u16::MAX)) as u16)
}

fn populate_derived_observation_fields(
    input: &mut VelocityObservationInput,
    request: VelocityObservationInputRequest,
) -> Result<(), CommissionError> {
    if let StationarityResult::Usable(suffix) = request.stationarity {
        input.drift_ratio_ppm = suffix.drift_ratio_ppm;
        let confidence = estimate_confidence(suffix.selection, request.confidence_family_size)
            .map_err(|_| CommissionError::VelocityConfidenceEstimationFailed)?;
        let mean_rate_q = suffix
            .selection
            .exact_totals
            .rate_q()
            .map_err(|_| CommissionError::VelocityRateCalculationFailed)?;
        input.settled_rate_mean_q = mean_rate_q;
        input.variance_of_mean_q2 = confidence.variance_of_mean_q2;
        input.post_fit_lag_one_state = 2;
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| CommissionError::VelocityConversionInvalid)?;
        input.settled_rate_low_q = mean_rate_q.saturating_sub(half_width);
        input.settled_rate_high_q = mean_rate_q.saturating_add(half_width);
        let signed_target = request
            .target_velocity_rpm
            .with_direction(direction_sign(request.direction))
            .map_err(|error| match error {
                VelocityUnitError::ArithmeticOverflow => CommissionError::TargetRateInvalid,
                VelocityUnitError::ZeroTarget => CommissionError::InvalidSchedule,
                VelocityUnitError::InvalidDirection => CommissionError::InvalidSchedule,
            })?;
        (input.deficit_low_q, input.deficit_high_q) = deficit_interval_q(
            signed_target.raw(),
            request.counts_per_rev,
            input.settled_rate_low_q,
            input.settled_rate_high_q,
        )?;
        input.disturbance = Some(
            equivalent_disturbance_from_rate(
                request.p.raw(),
                signed_target,
                suffix.selection.exact_totals,
                confidence.half_width_rate_q,
                request.counts_per_rev,
            )
            .map_err(|_| CommissionError::VelocityDisturbanceCalculationFailed)?,
        );
        input.disturbance_variance_q2 = equivalent_disturbance_variance_from_rate(
            request.p.raw(),
            confidence.variance_of_mean_q2,
            request.counts_per_rev,
        )
        .map_err(|_| CommissionError::VelocityDisturbanceCalculationFailed)?;
    } else if matches!(
        request.stationarity,
        StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift)
    ) && let Some(diagnostic) = request.rejected_post_fit
        && let Some(selection) = diagnostic.selection
    {
        let block_observations = 1u32
            .checked_shl(u32::from(selection.level))
            .ok_or(CommissionError::VelocityBlockGeometryInvalid)?;
        if u32::from(diagnostic.suffix_len) % block_observations != 0
            || selection.block_count != u32::from(diagnostic.suffix_len) / block_observations
        {
            return Err(CommissionError::VelocityBlockGeometryInvalid);
        }
        let confidence = estimate_confidence(selection, request.confidence_family_size)
            .map_err(|_| CommissionError::VelocityConfidenceEstimationFailed)?;
        let mean_rate_q = selection
            .exact_totals
            .rate_q()
            .map_err(|_| CommissionError::VelocityRateCalculationFailed)?;
        let half_width = i64::try_from(confidence.half_width_rate_q)
            .map_err(|_| CommissionError::VelocityConversionInvalid)?;
        input.settled_rate_low_q = mean_rate_q
            .checked_sub(half_width)
            .ok_or(CommissionError::VelocityIntervalInvalid)?;
        input.settled_rate_high_q = mean_rate_q
            .checked_add(half_width)
            .ok_or(CommissionError::VelocityIntervalInvalid)?;
        if input.settled_rate_low_q > input.settled_rate_high_q {
            return Err(CommissionError::VelocityIntervalInvalid);
        }
        input.settled_rate_mean_q = mean_rate_q;
        input.variance_of_mean_q2 = confidence.variance_of_mean_q2;
        input.post_fit_lag_one_state = if lag_one_passes_family(
            diagnostic.lag_one_q,
            selection.block_count,
            request.confidence_family_size,
        )
        .map_err(|_| CommissionError::VelocityLagOneEvaluationFailed)?
        {
            2
        } else {
            1
        };
        let signed_target = request
            .target_velocity_rpm
            .with_direction(direction_sign(request.direction))
            .map_err(|error| match error {
                VelocityUnitError::ArithmeticOverflow => CommissionError::TargetRateInvalid,
                VelocityUnitError::ZeroTarget => CommissionError::InvalidSchedule,
                VelocityUnitError::InvalidDirection => CommissionError::InvalidSchedule,
            })?;
        (input.deficit_low_q, input.deficit_high_q) = deficit_interval_q(
            signed_target.raw(),
            request.counts_per_rev,
            input.settled_rate_low_q,
            input.settled_rate_high_q,
        )?;
    }
    Ok(())
}

pub(super) fn deficit_interval_q(
    target_velocity_rpm: i32,
    counts_per_rev: u32,
    settled_rate_low_q: i64,
    settled_rate_high_q: i64,
) -> Result<(i64, i64), CommissionError> {
    if counts_per_rev == 0 {
        return Err(CommissionError::InvalidSchedule);
    }
    let target_numerator = i128::from(target_velocity_rpm)
        .checked_mul(i128::from(counts_per_rev))
        .and_then(|value| value.checked_mul(i128::from(VELOCITY_RATE_Q_SCALE)))
        .ok_or(CommissionError::TargetRateInvalid)?;
    let low_numerator = target_numerator
        .checked_sub(i128::from(settled_rate_high_q) * 60)
        .ok_or(CommissionError::TargetRateInvalid)?;
    let high_numerator = target_numerator
        .checked_sub(i128::from(settled_rate_low_q) * 60)
        .ok_or(CommissionError::TargetRateInvalid)?;
    let low = floor_div_60(low_numerator)?;
    let high = ceil_div_60(high_numerator)?;
    Ok((
        i64::try_from(low).map_err(|_| CommissionError::TargetRateInvalid)?,
        i64::try_from(high).map_err(|_| CommissionError::TargetRateInvalid)?,
    ))
}

pub(super) fn floor_div_60(numerator: i128) -> Result<i128, CommissionError> {
    let quotient = numerator / 60;
    if numerator < 0 && numerator % 60 != 0 {
        quotient
            .checked_sub(1)
            .ok_or(CommissionError::InvalidSchedule)
    } else {
        Ok(quotient)
    }
}

pub(super) fn ceil_div_60(numerator: i128) -> Result<i128, CommissionError> {
    let quotient = numerator / 60;
    if numerator > 0 && numerator % 60 != 0 {
        quotient
            .checked_add(1)
            .ok_or(CommissionError::InvalidSchedule)
    } else {
        Ok(quotient)
    }
}

/// Poll one moving-tail analysis, returning its result and any gate rejection.
///
/// The rejection is transient and never retained in commissioning state.
pub(super) fn poll_stationarity<const LEVELS: usize>(
    analysis: &mut StationarityWork,
    observations: &impl VelocityObservationSource,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<Option<(StationarityResult, Option<SuffixDiagnostic>, u32)>, CommissionError> {
    match analysis
        .poll(observations, blocker)
        .map_err(|_| CommissionError::VelocityAnalysisPollFailed)?
    {
        StationarityWorkPoll::Pending => Ok(None),
        StationarityWorkPoll::Complete {
            result,
            rejection,
            drift_ratio_ppm,
        } => Ok(Some((result, rejection, drift_ratio_ppm))),
        StationarityWorkPoll::RestDiagnostic(_) | StationarityWorkPoll::RestComplete(_) => {
            Err(CommissionError::VelocityUnexpectedPollVariant)
        }
    }
}

pub(super) fn poll_scored_rest_confirmation<const LEVELS: usize>(
    analysis: &mut StationarityWork,
    observations: &impl VelocityObservationSource,
    blocker: &mut DyadicBlocker<LEVELS>,
) -> Result<ScoredRestPoll, CommissionError> {
    match analysis
        .poll(observations, blocker)
        .map_err(|_| CommissionError::VelocityAnalysisPollFailed)?
    {
        StationarityWorkPoll::Pending => Ok(ScoredRestPoll::Pending),
        StationarityWorkPoll::RestDiagnostic(decision) => Ok(ScoredRestPoll::Diagnostic(decision)),
        StationarityWorkPoll::RestComplete(result) => {
            let selected_suffix_len =
                if result.disposition == RestSelectionDisposition::StatisticalAcceptance {
                    result.selected.map_or(0, |suffix| suffix.suffix_len)
                } else {
                    0
                };
            Ok(ScoredRestPoll::Complete(SettleConfirmation {
                selected: result.selected,
                confirmed_rest: result.disposition != RestSelectionDisposition::Rejected,
                exact_velocity_interval_q: result.exact_velocity_interval_q,
                selected_suffix_len,
                conditioned_rest_evidence: result.exact_velocity_interval_q.is_some(),
                disposition: result.disposition,
                recent_structural_code: result.recent_structural_code,
            }))
        }
        StationarityWorkPoll::Complete { .. } => {
            Err(CommissionError::VelocityUnexpectedPollVariant)
        }
    }
}

/// Accept a zero-target settle that never leaves one encoder-count band.
///
/// The bounds include the settle origin at zero.
pub(super) fn bounded_rest_displacement(minimum: i8, maximum: i8) -> bool {
    maximum - minimum <= 1
}

#[cfg(test)]
mod tests {
    use foci_tmc4671::pid::PidPGain;

    use crate::autotune::velocity_measurement::{
        DyadicBlocker, PlateauSelection, RateBlock, RestSelectionDisposition,
        StationarityExclusion, StationarityResult, StationarityWork, StationarySuffix,
        SuffixDiagnostic, VelocityDirection,
    };
    use crate::autotune::{CommissionError, TmcMechanicalRpm};
    use crate::commissioning::outer::state::VelocityObservationRing;

    use super::{
        ScoredRestPoll, VelocityObservationInputRequest, assemble_probe_observation_input,
        assemble_velocity_observation_input, bounded_rest_displacement, deficit_interval_q,
        poll_scored_rest_confirmation,
    };

    fn usable_suffix() -> StationarySuffix {
        StationarySuffix {
            selection: PlateauSelection {
                level: 1,
                block_count: 16,
                variance_of_mean_q2: 4,
                exact_totals: RateBlock {
                    delta_counts: 1,
                    elapsed_us: 1_000,
                    observation_count: 64,
                },
            },
            suffix_len: 64,
            slope_q: 0,
            slope_half_width_q: 1,
            lag_one_q: 0,
            lag_one_half_width_q: 1,
            residual_magnitude_q2: 1,
            span_x2_us: 0,
            drift_ratio_ppm: u32::MAX,
            tested_suffixes: 1,
        }
    }

    fn excessive_drift_rejection() -> SuffixDiagnostic {
        let selection = PlateauSelection {
            level: 2,
            block_count: 16,
            variance_of_mean_q2: 1_024,
            exact_totals: RateBlock {
                delta_counts: 32,
                elapsed_us: 64_000,
                observation_count: 64,
            },
        };
        SuffixDiagnostic {
            suffix_len: 64,
            selection: Some(selection),
            usable_plateau: true,
            slope_passed: true,
            lag_one_passed: false,
            lag_one_q: 750_000,
            settled_rate_interval_q: [1, 2],
            slope_rejection_margin_q: 0,
            lag_one_rejection_margin_q: 1,
            slope_q: 1,
            slope_half_width_q: 1,
        }
    }

    fn request_with_target(
        stationarity: StationarityResult,
        rejected_post_fit: Option<SuffixDiagnostic>,
        target_velocity_rpm: TmcMechanicalRpm,
    ) -> VelocityObservationInputRequest {
        request_with_target_and_p_gain(
            stationarity,
            rejected_post_fit,
            target_velocity_rpm,
            PidPGain::new(256).unwrap(),
        )
    }

    fn request_with_target_and_p_gain(
        stationarity: StationarityResult,
        rejected_post_fit: Option<SuffixDiagnostic>,
        target_velocity_rpm: TmcMechanicalRpm,
        p: PidPGain,
    ) -> VelocityObservationInputRequest {
        VelocityObservationInputRequest {
            direction: VelocityDirection::Forward,
            p,
            stationarity,
            rejected_post_fit,
            drift_ratio_ppm: 300_000,
            moving_pid_output_peak: 1,
            target_velocity_rpm,
            usable_torque_limit: 2_000,
            hard_torque_limit: 3_000,
            initial_lead_in: false,
            confidence_family_size: 16,
            counts_per_rev: 4_000,
        }
    }

    #[test]
    fn signed_target_overflow_is_a_target_rate_fault_when_usable() {
        let request = request_with_target_and_p_gain(
            StationarityResult::Usable(usable_suffix()),
            None,
            TmcMechanicalRpm::from_register(i32::MIN),
            PidPGain::ZERO,
        );

        assert_eq!(
            assemble_velocity_observation_input(request),
            Err(CommissionError::TargetRateInvalid)
        );
    }

    #[test]
    fn signed_target_overflow_is_a_target_rate_fault_when_drift_excluded() {
        let request = request_with_target_and_p_gain(
            StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift),
            Some(excessive_drift_rejection()),
            TmcMechanicalRpm::from_register(i32::MIN),
            PidPGain::ZERO,
        );

        assert_eq!(
            assemble_velocity_observation_input(request),
            Err(CommissionError::TargetRateInvalid)
        );
    }

    #[test]
    fn signed_target_zero_target_stays_invalid_schedule_when_usable() {
        let request = request_with_target(
            StationarityResult::Usable(usable_suffix()),
            None,
            TmcMechanicalRpm::from_register(0),
        );

        assert_eq!(
            assemble_velocity_observation_input(request),
            Err(CommissionError::InvalidSchedule)
        );
    }

    #[test]
    fn signed_target_zero_target_stays_invalid_schedule_when_drift_excluded() {
        let request = request_with_target(
            StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift),
            Some(excessive_drift_rejection()),
            TmcMechanicalRpm::from_register(0),
        );

        assert_eq!(
            assemble_velocity_observation_input(request),
            Err(CommissionError::InvalidSchedule)
        );
    }

    #[test]
    fn excessive_drift_uses_rejected_selection_at_the_requested_family() {
        let selection = PlateauSelection {
            level: 2,
            block_count: 16,
            variance_of_mean_q2: 1_024,
            exact_totals: RateBlock {
                delta_counts: 32,
                elapsed_us: 64_000,
                observation_count: 64,
            },
        };
        let rejection = SuffixDiagnostic {
            suffix_len: 64,
            selection: Some(selection),
            usable_plateau: true,
            slope_passed: true,
            lag_one_passed: false,
            lag_one_q: 750_000,
            settled_rate_interval_q: [1, 2],
            slope_rejection_margin_q: 0,
            lag_one_rejection_margin_q: 1,
            slope_q: 1,
            slope_half_width_q: 1,
        };

        let input = assemble_velocity_observation_input(VelocityObservationInputRequest {
            direction: VelocityDirection::Forward,
            p: PidPGain::new(256).unwrap(),
            stationarity: StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift),
            rejected_post_fit: Some(rejection),
            drift_ratio_ppm: 300_000,
            moving_pid_output_peak: 1,
            target_velocity_rpm: TmcMechanicalRpm::from_register(100),
            usable_torque_limit: 2_000,
            hard_torque_limit: 3_000,
            initial_lead_in: false,
            confidence_family_size: 16,
            counts_per_rev: 4_000,
        })
        .unwrap();

        assert_eq!(
            input.settled_rate_mean_q,
            selection.exact_totals.rate_q().unwrap()
        );
        assert_eq!(input.variance_of_mean_q2, selection.variance_of_mean_q2);
        assert!(input.settled_rate_low_q < input.settled_rate_mean_q);
        assert!(input.settled_rate_high_q > input.settled_rate_mean_q);
        assert_eq!(input.post_fit_lag_one_state, 2);
        assert!(input.disturbance.is_none());

        let malformed = SuffixDiagnostic {
            selection: Some(PlateauSelection {
                block_count: 15,
                ..selection
            }),
            ..rejection
        };
        let malformed_request = VelocityObservationInputRequest {
            direction: VelocityDirection::Forward,
            p: PidPGain::new(256).unwrap(),
            stationarity: StationarityResult::Excluded(StationarityExclusion::ExcessiveDrift),
            rejected_post_fit: Some(malformed),
            drift_ratio_ppm: 300_000,
            moving_pid_output_peak: 1,
            target_velocity_rpm: TmcMechanicalRpm::from_register(100),
            usable_torque_limit: 2_000,
            hard_torque_limit: 3_000,
            initial_lead_in: false,
            confidence_family_size: 16,
            counts_per_rev: 4_000,
        };
        assert!(
            assemble_velocity_observation_input(malformed_request).is_err(),
            "a retained selection with a different block population is malformed"
        );

        let probe_input = assemble_probe_observation_input(malformed_request).unwrap();
        assert_eq!(probe_input.settled_rate_low_q, 0);
        assert_eq!(probe_input.settled_rate_high_q, 0);
        assert_eq!(probe_input.predicted_torque_target_abs, 100);
        assert_eq!(probe_input.drift_ratio_ppm, 300_000);
    }

    #[test]
    fn scored_rest_contract_yields_diagnostic_before_conditioned_witness() {
        let mut ring = VelocityObservationRing::new();
        for _ in 0..256 {
            ring.push(0, 1_000).unwrap();
        }
        let mut analysis = StationarityWork::new();
        analysis.reset_for_scored_rest(false);
        let mut blocker = DyadicBlocker::<5>::new();
        let mut diagnostic = None;

        loop {
            match poll_scored_rest_confirmation(&mut analysis, &ring, &mut blocker).unwrap() {
                ScoredRestPoll::Pending => {}
                ScoredRestPoll::Diagnostic(decision) => {
                    assert!(diagnostic.replace(decision).is_none());
                }
                ScoredRestPoll::Complete(confirmation) => {
                    let diagnostic = diagnostic.expect("diagnostic must precede completion");
                    assert!(confirmation.confirmed_rest);
                    assert!(confirmation.conditioned_rest_evidence);
                    assert_eq!(confirmation.selected_suffix_len, 256);
                    assert_eq!(
                        confirmation.exact_velocity_interval_q,
                        Some(diagnostic.intervals_q[2])
                    );
                    assert_eq!(
                        confirmation.disposition,
                        RestSelectionDisposition::StatisticalAcceptance
                    );
                    assert_eq!(
                        confirmation.selected.map(|suffix| suffix.suffix_len),
                        Some(256)
                    );
                    assert_eq!(
                        confirmation.recent_structural_code,
                        diagnostic.structural_codes[0]
                    );
                    break;
                }
            }
        }
    }

    #[test]
    fn bounded_boundary_dither_is_accepted() {
        assert!(bounded_rest_displacement(0, 1));
    }

    #[test]
    fn multi_count_creep_is_not_accepted_as_bounded_rest() {
        assert!(!bounded_rest_displacement(-1, 2));
    }

    #[test]
    fn deficit_interval_uses_exact_mechanical_rpm_target() {
        assert_eq!(
            deficit_interval_q(176, 4_000, 11_733_333, 11_733_334),
            Ok((-1, 1))
        );
        assert_ne!(
            deficit_interval_q(8, 4_000, 11_733_333, 11_733_334),
            Ok((-1, 1))
        );
    }

    #[test]
    fn low_bound_narrowing_overflow_is_a_target_rate_fault() {
        assert_eq!(
            deficit_interval_q(i32::MAX, u32::MAX, 0, 0),
            Err(CommissionError::TargetRateInvalid)
        );
    }

    #[test]
    fn high_bound_narrowing_overflow_is_a_target_rate_fault() {
        assert_eq!(
            deficit_interval_q(2_000_000_000, 300_000_000, 0, i64::MAX),
            Err(CommissionError::TargetRateInvalid)
        );
    }
}
