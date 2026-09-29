//! Board-agnostic wrapping tick-clock arithmetic.

/// Ticks elapsed from `last` to `now` on a wrapping 32-bit hardware clock.
/// A `last` newer than `now` (written by a preempting task) yields zero. Valid while the
/// true difference is under half the 32-bit range.
pub fn elapsed_ticks(now: u32, last: u32) -> u32 {
    let raw_diff = now.wrapping_sub(last);
    if (raw_diff as i32) < 0 { 0 } else { raw_diff }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn elapsed_ticks_returns_forward_difference() {
        assert_eq!(elapsed_ticks(1005, 1000), 5);
    }

    #[test]
    fn elapsed_ticks_is_zero_for_identical_ticks() {
        assert_eq!(elapsed_ticks(1000, 1000), 0);
    }

    #[test]
    fn elapsed_ticks_is_zero_when_last_races_ahead_of_now() {
        assert_eq!(elapsed_ticks(999, 1000), 0);
    }

    #[test]
    fn elapsed_ticks_handles_a_32_bit_wrap() {
        assert_eq!(elapsed_ticks(50, u32::MAX - 100), 151);
    }
}
