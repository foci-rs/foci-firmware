//! Per-channel diagnostic report sequence counters.
//!
//! Every fragment and terminal reply emitted for one diagnostic report
//! shares a `report_seq` value drawn once per report from the channel the
//! request targeted. The host uses it to detect a dropped fragment when
//! the USB staging buffer silently discards a frame: a terminal reply
//! whose fragments do not carry a matching sequence is treated as if the
//! fragment never arrived.

use core::sync::atomic::{AtomicU8, Ordering::Relaxed};

use super::tmc::MAX_TMC_CHANNELS;

static REPORT_SEQ: [AtomicU8; MAX_TMC_CHANNELS] = [const { AtomicU8::new(0) }; MAX_TMC_CHANNELS];

/// Draw the next report sequence value for `channel`, wrapping on overflow.
///
/// An out-of-range channel returns 0 rather than panicking: diagnostics are
/// already gated on a valid channel before this is called.
pub fn next_report_seq(channel: u8) -> u8 {
    match REPORT_SEQ.get(channel as usize) {
        Some(counter) => counter.fetch_add(1, Relaxed),
        None => 0,
    }
}

/// Reset every channel's report sequence counter to zero.
///
/// Test seam only: lets tests assert on deterministic sequence values.
#[cfg(test)]
pub fn reset_report_seq() {
    for counter in &REPORT_SEQ {
        counter.store(0, Relaxed);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sequence_advances_per_channel_independently() {
        reset_report_seq();

        assert_eq!(next_report_seq(0), 0);
        assert_eq!(next_report_seq(0), 1);
        assert_eq!(next_report_seq(1), 0);
    }

    #[test]
    fn out_of_range_channel_returns_zero() {
        assert_eq!(next_report_seq(MAX_TMC_CHANNELS as u8), 0);
    }
}
