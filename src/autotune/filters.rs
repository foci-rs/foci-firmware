use super::FilterConfig;

// ---------------------------------------------------------------------------
// Filter recommendation (Phase 12)
// ---------------------------------------------------------------------------

/// Recommend filter cutoff frequencies based on prior validation metrics.
///
/// Follows the spec policy:
/// - Velocity chatter detected: enable velocity biquad, cutoff = velocity_bw * 2
/// - Position ramp-up hit oscillation early: enable position filter, cutoff = position_bw * 3
/// - Current ringing detected: enable flux/torque biquad, cutoff = current_bw * 2
///
/// Policy ordering: prefer velocity filtering before position filtering.
/// Leave flux/torque filters off unless current noise demands it.
///
/// # Parameters
/// - `velocity_chatter`: true if velocity validation detected zero-speed chatter
/// - `velocity_bandwidth_hz`: velocity-loop effective bandwidth
/// - `position_oscillation_early`: true if position P ramp-up hit oscillation
///   before reaching a useful gain value
/// - `position_bandwidth_hz`: estimated position-loop bandwidth (from velocity BW / 4)
/// - `current_ringing`: true if current validation detected ringing (count > 1)
/// - `current_bandwidth_hz`: current-loop effective bandwidth
pub fn recommend_filters(
    velocity_chatter: bool,
    velocity_bandwidth_hz: u32,
    position_oscillation_early: bool,
    position_bandwidth_hz: u32,
    current_ringing: bool,
    current_bandwidth_hz: u32,
) -> FilterConfig {
    let velocity_filter_hz = if velocity_chatter {
        clamp_filter_cutoff(velocity_bandwidth_hz * 2)
    } else {
        None
    };

    let position_filter_hz = if position_oscillation_early {
        clamp_filter_cutoff(position_bandwidth_hz * 3)
    } else {
        None
    };

    let (flux_filter_hz, torque_filter_hz) = if current_ringing {
        let cutoff = clamp_filter_cutoff(current_bandwidth_hz * 2);
        (cutoff, cutoff)
    } else {
        (None, None)
    };

    FilterConfig {
        velocity_filter_hz,
        position_filter_hz,
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
    fn filter_velocity_chatter_enables_velocity_filter() {
        let config = recommend_filters(true, 100, false, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, Some(200)); // 100 * 2
        assert_eq!(config.position_filter_hz, None);
        assert_eq!(config.flux_filter_hz, None);
    }

    #[test]
    fn filter_position_oscillation_enables_position_filter() {
        let config = recommend_filters(false, 100, true, 25, false, 500);
        assert_eq!(config.velocity_filter_hz, None);
        assert_eq!(config.position_filter_hz, Some(75)); // 25 * 3
        assert_eq!(config.flux_filter_hz, None);
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
    fn filter_all_issues_all_filters() {
        let config = recommend_filters(true, 100, true, 25, true, 500);
        assert_eq!(config.velocity_filter_hz, Some(200));
        assert_eq!(config.position_filter_hz, Some(75));
        assert_eq!(config.flux_filter_hz, Some(1000));
        assert_eq!(config.torque_filter_hz, Some(1000));
    }
}
