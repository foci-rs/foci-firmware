//! Conversion of the outer-safety loop's peak window velocity into a permille ratio of a
//! stroke's commanded target.

use super::sweep_plan::VELOCITY_SWEEP_SAMPLE_PERIOD_US;

/// Length of the outer-safety velocity window in microseconds.
pub(crate) const OUTER_SAFETY_VELOCITY_WINDOW_US: u32 = 2 * VELOCITY_SWEEP_SAMPLE_PERIOD_US;

fn target_milli_counts_per_s(target_rpm_abs: u32, counts_per_rev: u32) -> u128 {
    u128::from(target_rpm_abs) * u128::from(counts_per_rev) * 1000 / 60
}

/// Peak window velocity as permille of the commanded target.
///
/// `peak_milli_counts_per_s` is in milli-counts per second. Returns 0 when the target or
/// the encoder resolution is zero, meaning "not scored"; saturates at `u16::MAX`.
pub(crate) fn peak_permille(
    peak_milli_counts_per_s: u64,
    target_rpm_abs: u32,
    counts_per_rev: u32,
) -> u16 {
    let target = target_milli_counts_per_s(target_rpm_abs, counts_per_rev);
    if target == 0 {
        return 0;
    }
    let ratio = u128::from(peak_milli_counts_per_s) * 1000 / target;
    u16::try_from(ratio).unwrap_or(u16::MAX)
}

/// Whether one encoder count per window is fine enough to judge `bound_permille`.
///
/// True when one count per window, as permille of the target, is at most a quarter of the
/// headroom `bound_permille - 1000`. False for a zero target or a bound at or below 1000.
pub(crate) fn peak_scorable(target_rpm_abs: u32, counts_per_rev: u32, bound_permille: u16) -> bool {
    let target = target_milli_counts_per_s(target_rpm_abs, counts_per_rev);
    if target == 0 || bound_permille <= 1000 {
        return false;
    }
    let one_count = u128::from(1_000_000_000 / OUTER_SAFETY_VELOCITY_WINDOW_US);
    one_count * 1000 / target <= u128::from((bound_permille - 1000) / 4)
}

/// Peak window velocity as permille of the target, or 0 ("not scored") when the resolution
/// is too coarse to judge `bound_permille`.
pub(crate) fn scored_peak_permille(
    peak_milli_counts_per_s: u64,
    target_rpm_abs: u32,
    counts_per_rev: u32,
    bound_permille: u16,
) -> u16 {
    if peak_scorable(target_rpm_abs, counts_per_rev, bound_permille) {
        peak_permille(peak_milli_counts_per_s, target_rpm_abs, counts_per_rev)
    } else {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const CPR: u32 = 4000;
    const RPM: u32 = 322;

    fn target() -> u64 {
        target_milli_counts_per_s(RPM, CPR) as u64
    }

    #[test]
    fn a_stroke_at_target_reports_one_thousand() {
        assert_eq!(peak_permille(target(), RPM, CPR), 1000);
    }

    #[test]
    fn a_33_counts_per_ms_peak_reports_about_1530() {
        let ratio = peak_permille(33_000_000, RPM, CPR);
        assert!((1527..=1547).contains(&ratio), "{ratio}");
    }

    #[test]
    fn zero_target_or_zero_cpr_reports_zero() {
        assert_eq!(peak_permille(1_000_000, 0, CPR), 0);
        assert_eq!(peak_permille(1_000_000, RPM, 0), 0);
        assert_eq!(peak_permille(1_000_000, 0, 0), 0);
    }

    #[test]
    fn a_runaway_peak_saturates() {
        assert_eq!(peak_permille(u64::MAX, RPM, 1), u16::MAX);
    }

    #[test]
    fn extreme_valid_inputs_do_not_wrap() {
        let cpr = i32::MAX as u32;
        let t = u64::try_from(target_milli_counts_per_s(60_000, cpr)).unwrap();
        assert_eq!(peak_permille(t, 60_000, cpr), 1000);
        assert_eq!(peak_permille(2 * t, 60_000, cpr), 2000);
    }

    #[test]
    fn the_live_ouroboros_target_is_scorable_at_1400() {
        assert!(peak_scorable(RPM, CPR, 1400));
    }

    #[test]
    fn a_target_under_ten_counts_per_window_is_not_scorable_at_1400() {
        assert!(!peak_scorable(70, CPR, 1400));
    }

    #[test]
    fn bound_at_or_below_target_is_never_scorable() {
        assert!(!peak_scorable(RPM, CPR, 1000));
        assert!(!peak_scorable(RPM, CPR, 900));
        assert!(!peak_scorable(0, CPR, 1400));
    }

    #[test]
    fn scored_peak_is_zero_when_not_scorable() {
        assert_eq!(scored_peak_permille(target(), 70, CPR, 1400), 0);
        assert_eq!(scored_peak_permille(target(), RPM, CPR, 1400), 1000);
    }
}
