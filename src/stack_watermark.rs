//! MSP stack high-water measurement from a boot-time sentinel paint.
//!
//! A board paints the unused span of its main stack with
//! [`STACK_PAINT_SENTINEL`] during startup and registers the span's bounds
//! here. The stack grows down toward the span, so the surviving sentinel run
//! starting at the bottom is the headroom the stack never consumed.
//!
//! Only the bounds are kept, as plain addresses. The span becomes ordinary
//! stack again as call depth grows into it, so holding a typed reference to it
//! would assert an aliasing guarantee the memory does not have; the readout
//! instead re-derives a raw pointer and reads through it at the moment the host
//! asks.

use core::sync::atomic::{AtomicUsize, Ordering};

/// Word written across the unused stack span at startup.
///
/// Chosen to be implausible as a live stack value: the scan's correctness in
/// the presence of concurrent stack use rests on the stack never *restoring*
/// this pattern once it has overwritten it.
pub const STACK_PAINT_SENTINEL: u32 = 0xC0DE_D00D;

static PAINT_BASE: AtomicUsize = AtomicUsize::new(0);
static PAINT_LEN_WORDS: AtomicUsize = AtomicUsize::new(0);

/// Stack occupancy derived from the surviving sentinel run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StackUsage {
    /// Bytes at the bottom of the painted span the stack never reached.
    pub unused_bytes: u32,
    /// Total span painted at startup.
    pub painted_bytes: u32,
}

/// Record the bounds of the span a board painted with [`STACK_PAINT_SENTINEL`].
///
/// # Safety
///
/// `base` must be 4-byte aligned and `[base, base + len_words * 4)` must be a
/// span the caller has just painted with [`STACK_PAINT_SENTINEL`] that stays
/// readable for the rest of the program's life. Registering any other range
/// makes [`stack_usage`] read unrelated memory.
pub unsafe fn register_painted_region(base: usize, len_words: usize) {
    PAINT_LEN_WORDS.store(len_words, Ordering::Relaxed);
    PAINT_BASE.store(base, Ordering::Relaxed);
}

/// Count the leading words that still hold the sentinel.
fn intact_prefix_words(len_words: usize, mut read_word: impl FnMut(usize) -> u32) -> usize {
    (0..len_words)
        .take_while(|&index| read_word(index) == STACK_PAINT_SENTINEL)
        .count()
}

/// Measure the registered span, or `None` on a board that never painted.
///
/// Walks the intact sentinel run one word at a time, so an untouched span costs
/// a full walk. That cost is why the readout is host-triggered rather than
/// periodic.
pub fn stack_usage() -> Option<StackUsage> {
    let base = PAINT_BASE.load(Ordering::Relaxed);
    if base == 0 {
        return None;
    }
    let len_words = PAINT_LEN_WORDS.load(Ordering::Relaxed);

    let intact = intact_prefix_words(len_words, |index| {
        // SAFETY: `register_painted_region`'s contract makes `base` a 4-byte
        // aligned address with `len_words` readable words above it, and `index`
        // is bounded by `len_words`, so the read is in range and aligned. The
        // pointer is derived and discarded per word rather than held, so no
        // reference outlives the load.
        //
        // Interrupts are deliberately left enabled. Masking them for the whole
        // walk would be a multi-hundred-microsecond blackout on a firmware with
        // a 10 kHz position tick, which is a far worse defect than the race it
        // would close. A higher-priority handler can therefore push a frame
        // into the span mid-walk, but that can only turn a sentinel word into a
        // non-sentinel one and so can only *shorten* the reported run: the
        // stack never writes the sentinel pattern back. The result is a
        // conservative lower bound on headroom, which is the safe direction for
        // every use this value has. `read_volatile` keeps each load a real,
        // untorn, single word-aligned access that the compiler cannot cache,
        // reorder, or synthesize.
        unsafe { (base as *const u32).add(index).read_volatile() }
    });

    Some(StackUsage {
        unused_bytes: (intact * size_of::<u32>()) as u32,
        painted_bytes: (len_words * size_of::<u32>()) as u32,
    })
}

#[cfg(test)]
pub(crate) fn clear_painted_region_for_test() {
    PAINT_BASE.store(0, Ordering::Relaxed);
    PAINT_LEN_WORDS.store(0, Ordering::Relaxed);
}

/// Register a host-side array as the painted span.
///
/// # Safety
///
/// `words` must outlive every later [`stack_usage`] call, so callers pass a
/// leaked allocation.
#[cfg(test)]
pub(crate) unsafe fn set_painted_region_for_test(words: &'static [u32]) {
    unsafe { register_painted_region(words.as_ptr() as usize, words.len()) }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intact_words_of(words: &[u32]) -> usize {
        intact_prefix_words(words.len(), |index| words[index])
    }

    #[test]
    fn untouched_paint_reports_the_whole_span_intact() {
        assert_eq!(intact_words_of(&[STACK_PAINT_SENTINEL; 4]), 4);
    }

    #[test]
    fn the_run_stops_at_the_first_overwritten_word() {
        assert_eq!(
            intact_words_of(&[
                STACK_PAINT_SENTINEL,
                STACK_PAINT_SENTINEL,
                0xDEAD_BEEF,
                STACK_PAINT_SENTINEL,
            ]),
            2
        );
    }

    #[test]
    fn a_fully_consumed_paint_reports_no_headroom() {
        assert_eq!(intact_words_of(&[0, 1, 2, 3]), 0);
    }

    #[test]
    fn an_empty_span_reports_zero() {
        assert_eq!(intact_words_of(&[]), 0);
    }

    #[test]
    fn stack_usage_reads_the_registered_span_through_raw_pointers() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let words: &'static [u32] = std::vec::Vec::leak(std::vec![
            STACK_PAINT_SENTINEL,
            STACK_PAINT_SENTINEL,
            STACK_PAINT_SENTINEL,
            0xDEAD_BEEF,
            STACK_PAINT_SENTINEL,
        ]);
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { set_painted_region_for_test(words) };

        assert_eq!(
            stack_usage(),
            Some(StackUsage {
                unused_bytes: 12,
                painted_bytes: 20,
            })
        );
    }

    #[test]
    fn stack_usage_is_none_before_any_board_registers() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        clear_painted_region_for_test();

        assert_eq!(stack_usage(), None);
    }
}
