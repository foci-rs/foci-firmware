//! MSP stack high-water measurement from a boot-time sentinel paint. Boards register the painted
//! span's bounds; the readout walks it in a critical section admitted by a caller-supplied predicate.

use core::sync::atomic::{AtomicUsize, Ordering};

/// Word written across the unused stack span at startup.
pub const STACK_PAINT_SENTINEL: u32 = 0xC0DE_D00D;

/// Upper bound on words read per readout; bounds the interrupt-masked window.
const MAX_SCAN_WORDS: usize = 8 * 1024;

static PAINT_BASE: AtomicUsize = AtomicUsize::new(0);
static PAINT_LEN_WORDS: AtomicUsize = AtomicUsize::new(0);

/// Stack occupancy derived from the surviving sentinel run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StackUsage {
    /// Bytes at the bottom of the painted span the stack never reached.
    pub unused_bytes: u32,
    pub painted_bytes: u32,
    /// False when every scanned word still held the sentinel, making `unused_bytes` a lower bound.
    pub watermark_found: bool,
}

/// Outcome of an admitted readout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StackScan {
    Measured(StackUsage),
    /// The admission predicate declined, so nothing was read.
    Refused,
    /// No board ever registered a painted span.
    NotPainted,
}

/// Record the bounds of the span a board painted with [`STACK_PAINT_SENTINEL`].
///
/// # Safety
///
/// `base` must be 4-byte aligned and `[base, base + len_words * 4)` must be just-painted with
/// [`STACK_PAINT_SENTINEL`] and stay readable for the rest of the program's life.
pub unsafe fn register_painted_region(base: usize, len_words: usize) {
    PAINT_LEN_WORDS.store(len_words, Ordering::Relaxed);
    PAINT_BASE.store(base, Ordering::Relaxed);
}

fn intact_prefix_words(len_words: usize, mut read_word: impl FnMut(usize) -> u32) -> usize {
    (0..len_words)
        .take_while(|&index| read_word(index) == STACK_PAINT_SENTINEL)
        .count()
}

/// Measure the registered span if `admit` allows it. `admit` runs with interrupts masked, before
/// the first read, so it must be short and non-blocking.
pub fn scan_admitted_by(admit: impl FnOnce() -> bool) -> StackScan {
    critical_section::with(|_| {
        if !admit() {
            return StackScan::Refused;
        }
        let base = PAINT_BASE.load(Ordering::Relaxed);
        if base == 0 {
            return StackScan::NotPainted;
        }
        let len_words = PAINT_LEN_WORDS.load(Ordering::Relaxed);
        let scan_words = len_words.min(MAX_SCAN_WORDS);

        let intact = intact_prefix_words(scan_words, |index| {
            // SAFETY: two obligations, exclusion and validity.
            //
            // Exclusion: this runs with interrupts masked on a single-core MCU
            // whose every task is scheduled through an interrupt, so no other
            // execution context runs for the duration. Nothing can push a stack
            // frame into the span while it is read, which is the only way the
            // memory could change. Masking is what makes the read sound -- a
            // pushed word can hold any bit pattern, so a racing read could
            // report a coincidental sentinel as intact headroom and overstate
            // the result.
            //
            // Validity: `register_painted_region`'s contract makes `base` a
            // 4-byte aligned address with `len_words` readable words above it
            // that stay live for the program's life, and `index` is bounded by
            // `scan_words <= len_words`, so every read is aligned and in range.
            // The pointer is derived and discarded per word, so no reference
            // into the span outlives the load.
            unsafe { (base as *const u32).add(index).read_volatile() }
        });

        StackScan::Measured(StackUsage {
            unused_bytes: (intact * size_of::<u32>()) as u32,
            painted_bytes: (len_words * size_of::<u32>()) as u32,
            watermark_found: intact < scan_words,
        })
    })
}

#[cfg(test)]
pub(crate) fn clear_painted_region_for_test() {
    PAINT_BASE.store(0, Ordering::Relaxed);
    PAINT_LEN_WORDS.store(0, Ordering::Relaxed);
}

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

    fn register_leaked(words: std::vec::Vec<u32>) {
        let words: &'static [u32] = std::vec::Vec::leak(words);
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { set_painted_region_for_test(words) };
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
    fn an_admitted_scan_reads_the_registered_span_through_raw_pointers() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        register_leaked(std::vec![
            STACK_PAINT_SENTINEL,
            STACK_PAINT_SENTINEL,
            STACK_PAINT_SENTINEL,
            0xDEAD_BEEF,
            STACK_PAINT_SENTINEL,
        ]);

        assert_eq!(
            scan_admitted_by(|| true),
            StackScan::Measured(StackUsage {
                unused_bytes: 12,
                painted_bytes: 20,
                watermark_found: true,
            })
        );
    }

    #[test]
    fn an_entirely_intact_span_reports_a_lower_bound() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        register_leaked(std::vec![STACK_PAINT_SENTINEL; 6]);

        assert_eq!(
            scan_admitted_by(|| true),
            StackScan::Measured(StackUsage {
                unused_bytes: 24,
                painted_bytes: 24,
                watermark_found: false,
            })
        );
    }

    #[test]
    fn a_span_longer_than_the_scan_window_stops_at_the_window() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let mut words = std::vec![STACK_PAINT_SENTINEL; MAX_SCAN_WORDS + 64];
        words[MAX_SCAN_WORDS + 8] = 0xDEAD_BEEF;
        register_leaked(words);

        let StackScan::Measured(usage) = scan_admitted_by(|| true) else {
            panic!("expected a measured scan");
        };

        assert_eq!(
            usage,
            StackUsage {
                unused_bytes: (MAX_SCAN_WORDS * size_of::<u32>()) as u32,
                painted_bytes: ((MAX_SCAN_WORDS + 64) * size_of::<u32>()) as u32,
                watermark_found: false,
            }
        );
        assert!(usage.unused_bytes < usage.painted_bytes);
    }

    #[test]
    fn an_admitted_scan_reports_not_painted_before_any_board_registers() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        clear_painted_region_for_test();

        assert_eq!(scan_admitted_by(|| true), StackScan::NotPainted);
    }

    #[test]
    fn a_declining_predicate_refuses_before_the_span_is_consulted() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        register_leaked(std::vec![STACK_PAINT_SENTINEL; 4]);
        let mut calls = 0;

        let scan = scan_admitted_by(|| {
            calls += 1;
            false
        });

        assert_eq!(scan, StackScan::Refused);
        assert_eq!(calls, 1);
    }

    #[test]
    fn a_declining_predicate_outranks_an_unregistered_span() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        clear_painted_region_for_test();

        assert_eq!(scan_admitted_by(|| false), StackScan::Refused);
    }

    #[test]
    fn the_span_is_read_only_after_the_predicate_admits_the_scan() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        let words: &'static mut [u32] = std::vec::Vec::leak(std::vec![STACK_PAINT_SENTINEL; 4]);
        let base = words.as_ptr() as usize;
        // SAFETY: the span is leaked, so it outlives every later read.
        unsafe { register_painted_region(base, words.len()) };

        let scan = scan_admitted_by(|| {
            words[2] = 0xDEAD_BEEF;
            true
        });

        assert_eq!(
            scan,
            StackScan::Measured(StackUsage {
                unused_bytes: 8,
                painted_bytes: 16,
                watermark_found: true,
            })
        );
    }
}
