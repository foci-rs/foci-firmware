//! Board-agnostic wrapping tick-clock arithmetic.

/// Ticks elapsed from `last` to `now` on a wrapping 32-bit hardware clock.
///
/// A `last` timestamp written by a higher-priority task between the caller
/// sampling `now` and calling this function can be chronologically newer
/// than `now` itself. A plain `wrapping_sub` reads that as an enormous
/// unsigned duration instead of the small negative real difference it is;
/// this treats it as zero elapsed. Correct as long as the true difference
/// is under half the 32-bit tick range.
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
