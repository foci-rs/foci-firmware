use super::FilterConfig;

// ---------------------------------------------------------------------------
// Filter recommendation (Phase 12)
// ---------------------------------------------------------------------------

/// Recommend the Stage 2 filter configuration.
///
/// Velocity chatter and early position oscillation remain inputs so those observations are not
/// discarded. Stage 2 does not currently select velocity or position filters.
///
/// Current ringing retains the existing interim torque/flux policy: both target filters use twice
/// the current-loop bandwidth through the shared legacy 10–1000 Hz clamp.
///
/// # Parameters
/// - `_velocity_chatter`: retained velocity observation; not consumed by the interim policy
/// - `_velocity_bandwidth_hz`: retained velocity bandwidth; not consumed by the interim policy
/// - `_position_oscillation_early`: retained position observation; not consumed by the interim
///   policy
/// - `_position_bandwidth_hz`: retained position bandwidth; not consumed by the interim policy
/// - `current_ringing`: true if current validation detected ringing (count > 1)
/// - `current_bandwidth_hz`: current-loop effective bandwidth
pub fn recommend_filters(
    _velocity_chatter: bool,
    _velocity_bandwidth_hz: u32,
    _position_oscillation_early: bool,
    _position_bandwidth_hz: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
) -> FilterConfig {
    let (flux_filter_hz, torque_filter_hz) = if current_ringing {
        let cutoff = clamp_filter_cutoff(current_bandwidth_hz * 2);
        (cutoff, cutoff)
    } else {
        (None, None)
    };

    FilterConfig {
        velocity_filter_hz: None,
        position_filter_hz: None,
        flux_filter_hz,
        torque_filter_hz,
    }
}

/// Clamp a filter cutoff to the valid biquad range (10-1000 Hz).
///
/// Returns `None` if the cutoff is below the minimum (filter too low
/// to be useful). Clamps to 1000 Hz if above the maximum.
fn clamp_filter_cutoff(cutoff_hz: u32) -> Option<u32> {
    const MIN_HZ: u32 = 10;
    const MAX_HZ: u32 = 1000;
    if cutoff_hz < MIN_HZ {
        None
    } else if cutoff_hz > MAX_HZ {
        Some(MAX_HZ)
    } else {
        Some(cutoff_hz)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Filter recommendations ---

    #[test]
    fn filter_no_issues_no_filters() {
        let config = recommend_filters(false, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
        assert_eq!(config.torque_filter_hz, None);
    }

    #[test]
    fn filter_velocity_chatter_keeps_motion_filters_disabled() {
        let config = recommend_filters(true, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
        assert_eq!(config.torque_filter_hz, None);
    }

    #[test]
    fn filter_position_oscillation_keeps_motion_filters_disabled() {
        let config = recommend_filters(false, 100, true, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
        assert_eq!(config.torque_filter_hz, None);
    }

    #[test]
    fn filter_current_ringing_enables_flux_torque() {
        let config = recommend_filters(false, 100, false, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, Some(1000)); // 500 * 2
        assert_eq!(config.torque_filter_hz, Some(1000));
    }

    #[test]
    fn filter_all_issues_only_selects_current_filters() {
        let config = recommend_filters(true, 100, true, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, Some(1000));
        assert_eq!(config.torque_filter_hz, Some(1000));
    }
}
