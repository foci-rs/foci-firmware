//! Config-derived tracking bound and the closed-form start rung.

use crate::autotune::{CommissionProfile, PositionTuneOutcome};
use foci_tmc4671::pid::PidPGain;

const MODEL_NUMERATOR_HUNDREDTHS: u64 = 1_536;

pub(crate) fn margin_permille(profile: CommissionProfile) -> u32 {
    match profile {
        CommissionProfile::Conservative => 1_500,
        CommissionProfile::Balanced => 2_000,
        CommissionProfile::Stiff => 3_000,
    }
}

pub(crate) fn bound_units(
    stall_threshold_units: u32,
    margin_permille: u32,
) -> Result<u32, PositionTuneOutcome> {
    if stall_threshold_units == 0 || margin_permille == 0 {
        return Err(PositionTuneOutcome::ConfigurationInvalid);
    }
    let bound = u64::from(stall_threshold_units) * 1_000 / u64::from(margin_permille);
    u32::try_from(bound.max(1)).map_err(|_| PositionTuneOutcome::ConfigurationInvalid)
}

fn feedforward_residual_permille(ff_permille: u32) -> u64 {
    u64::from(ff_permille.abs_diff(1_000))
}

pub(crate) fn predicted_error_units(
    speed_mrev_s: u32,
    ff_permille: u32,
    position_p_raw: u16,
) -> u32 {
    if position_p_raw == 0 {
        return u32::MAX;
    }
    let numerator = MODEL_NUMERATOR_HUNDREDTHS
        * u64::from(speed_mrev_s)
        * feedforward_residual_permille(ff_permille);
    let denominator = 100 * 1_000 * u64::from(position_p_raw);
    u32::try_from(numerator / denominator).unwrap_or(u32::MAX)
}

pub(crate) fn predicted_start_p(
    speed_mrev_s: u32,
    ff_permille: u32,
    bound_units: u32,
) -> Result<PidPGain, PositionTuneOutcome> {
    if bound_units == 0 {
        return Err(PositionTuneOutcome::ConfigurationInvalid);
    }
    let numerator = MODEL_NUMERATOR_HUNDREDTHS
        * u64::from(speed_mrev_s)
        * feedforward_residual_permille(ff_permille);
    let denominator = 100 * 1_000 * u64::from(bound_units);
    let raw = numerator.div_ceil(denominator).max(1);
    u16::try_from(raw)
        .ok()
        .and_then(|raw| PidPGain::new(raw).ok())
        .ok_or(PositionTuneOutcome::ConfigurationInvalid)
}

pub(crate) fn start_rung(
    homing_speed_mrev_s: u32,
    motion_speed_mrev_s: u32,
    ff_permille: u32,
    bound_units: u32,
) -> Result<PidPGain, PositionTuneOutcome> {
    let homing = predicted_start_p(homing_speed_mrev_s, 0, bound_units)?;
    let motion = predicted_start_p(motion_speed_mrev_s, ff_permille, bound_units)?;
    Ok(if motion.raw() >= homing.raw() {
        motion
    } else {
        homing
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn model_reproduces_the_homing_worked_example() {
        assert_eq!(predicted_error_units(4_375, 0, 100), 672);
        assert_eq!(predicted_error_units(4_375, 0, 22), 3_054);
    }

    #[test]
    fn bound_is_stall_over_margin() {
        assert_eq!(bound_units(819, 2_000), Ok(409));
        assert_eq!(bound_units(819, 1_500), Ok(546));
        assert_eq!(
            bound_units(0, 2_000),
            Err(PositionTuneOutcome::ConfigurationInvalid)
        );
        assert_eq!(
            bound_units(819, 0),
            Err(PositionTuneOutcome::ConfigurationInvalid)
        );
    }

    #[test]
    fn start_rung_takes_the_larger_of_homing_and_motion() {
        assert_eq!(predicted_start_p(4_375, 0, 409).unwrap().raw(), 165);
        assert_eq!(predicted_start_p(10_000, 1_750, 409).unwrap().raw(), 282);
        assert_eq!(start_rung(4_375, 10_000, 1_750, 409).unwrap().raw(), 282);
        assert_eq!(start_rung(1_250, 10_000, 1_750, 409).unwrap().raw(), 282);
        assert_eq!(start_rung(4_375, 10_000, 1_000, 409).unwrap().raw(), 165);
    }

    #[test]
    fn start_rung_rejects_unrepresentable_gains() {
        assert_eq!(
            predicted_start_p(10_000, 0, 1),
            Err(PositionTuneOutcome::ConfigurationInvalid)
        );
        assert!(predicted_start_p(0, 0, 409).unwrap().raw() >= 1);
    }

    #[test]
    fn margins_follow_the_profile() {
        assert_eq!(margin_permille(CommissionProfile::Conservative), 1_500);
        assert_eq!(margin_permille(CommissionProfile::Balanced), 2_000);
        assert_eq!(margin_permille(CommissionProfile::Stiff), 3_000);
    }

    #[test]
    fn extreme_speed_and_feedforward_do_not_overflow() {
        assert_eq!(
            predicted_error_units(u32::MAX, u32::from(u16::MAX), 1),
            u32::MAX
        );
        assert_eq!(predicted_error_units(u32::MAX, 0, 1), u32::MAX);
        assert_eq!(
            predicted_start_p(u32::MAX, u32::from(u16::MAX), 1),
            Err(PositionTuneOutcome::ConfigurationInvalid)
        );
        assert_eq!(
            predicted_start_p(u32::MAX, u32::from(u16::MAX), u32::MAX)
                .unwrap()
                .raw(),
            992
        );
    }
}
