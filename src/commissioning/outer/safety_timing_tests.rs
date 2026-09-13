use core::mem::size_of;

use crate::autotune::CommissionError;

use super::{
    OuterSafetyTimingAccumulator, OuterSafetyTimingContext, OuterSafetyTimingPhase,
    OuterSafetyTimingStage, TraceSummarySeqs,
};

fn context(
    stage: OuterSafetyTimingStage,
    phase: OuterSafetyTimingPhase,
    rung: u8,
    slot: u8,
    target_zero: bool,
) -> OuterSafetyTimingContext {
    OuterSafetyTimingContext::new(stage, phase, rung, slot, target_zero).unwrap()
}

#[test]
fn outer_safety_timing_uses_the_existing_sixteen_byte_storage() {
    assert_eq!(size_of::<OuterSafetyTimingAccumulator>(), 16);
    assert_eq!(size_of::<TraceSummarySeqs>(), 16);
}

#[test]
fn outer_safety_timing_resets_storage_before_legacy_restore_sequences() {
    let mut storage = TraceSummarySeqs::new();
    storage
        .outer_safety_timing_mut()
        .observe(
            14_329,
            context(
                OuterSafetyTimingStage::Proportional,
                OuterSafetyTimingPhase::EvidenceEmission,
                21,
                7,
                true,
            ),
        )
        .unwrap();

    let summary = storage.take_outer_safety_timing();

    assert_eq!(summary.sample_count, 1);
    assert_eq!(summary.max_gap_us, 14_329);
    assert_eq!(storage.next_restore(), 0);
    assert_eq!(storage.next_restore(), 1);
}

#[test]
fn outer_safety_timing_assigns_closed_bins_and_preserves_the_first_maximum() {
    let mut timing = OuterSafetyTimingAccumulator::new();
    let first = context(
        OuterSafetyTimingStage::Proportional,
        OuterSafetyTimingPhase::MovingCapture,
        7,
        3,
        false,
    );
    let tied = context(
        OuterSafetyTimingStage::Integral,
        OuterSafetyTimingPhase::RecoveryMove,
        4,
        6,
        false,
    );

    for gap_us in [1, 5_000, 5_001, 10_000, 10_001, 15_000, 15_001, 20_000] {
        timing.observe(gap_us, first).unwrap();
    }
    timing.observe(20_000, tied).unwrap();

    let summary = timing.summary();
    assert_eq!(summary.sample_count, 9);
    assert_eq!(summary.bin_counts, [2, 2, 2, 3]);
    assert_eq!(summary.max_gap_us, 20_000);
    assert_eq!(summary.max_context, Some(first));
    assert_eq!(summary.zero_count, 0);
    assert_eq!(summary.over_bound_count, 0);
}

#[test]
fn outer_safety_timing_counts_faulting_gaps_before_abort() {
    let mut timing = OuterSafetyTimingAccumulator::new();
    let zero = context(
        OuterSafetyTimingStage::Proportional,
        OuterSafetyTimingPhase::ZeroSettle,
        21,
        7,
        true,
    );
    let over = context(
        OuterSafetyTimingStage::Proportional,
        OuterSafetyTimingPhase::RecoverySetup,
        21,
        7,
        true,
    );

    timing.observe(0, zero).unwrap();
    timing.observe(20_001, over).unwrap();

    let summary = timing.summary();
    assert_eq!(summary.sample_count, 2);
    assert_eq!(summary.bin_counts, [0; 4]);
    assert_eq!(summary.max_gap_us, 20_001);
    assert_eq!(summary.max_context, Some(over));
    assert_eq!(summary.zero_count, 1);
    assert_eq!(summary.over_bound_count, 1);
    assert_eq!(
        summary.sample_count,
        summary.bin_counts.iter().sum::<u32>() + summary.zero_count + summary.over_bound_count
    );
}

#[test]
fn outer_safety_timing_rejects_gap_exceeding_u32_representable_microseconds() {
    let mut timing = OuterSafetyTimingAccumulator::new();
    let gap_us = u64::from(u32::MAX) + 1;

    let error = timing
        .observe(
            gap_us,
            context(
                OuterSafetyTimingStage::Integral,
                OuterSafetyTimingPhase::MovingCapture,
                0,
                0,
                false,
            ),
        )
        .unwrap_err();

    assert_eq!(error, CommissionError::SafetyTimingGapInvalid);
}

#[test]
fn outer_safety_timing_rejects_context_and_counter_overflow() {
    assert!(
        OuterSafetyTimingContext::new(
            OuterSafetyTimingStage::Proportional,
            OuterSafetyTimingPhase::MovingCapture,
            32,
            0,
            false,
        )
        .is_err()
    );
    assert!(
        OuterSafetyTimingContext::new(
            OuterSafetyTimingStage::Proportional,
            OuterSafetyTimingPhase::MovingCapture,
            0,
            16,
            false,
        )
        .is_err()
    );

    let mut timing = OuterSafetyTimingAccumulator::new();
    timing.set_bin_count_for_test(0, (1 << 19) - 1).unwrap();
    assert!(
        timing
            .observe(
                1,
                context(
                    OuterSafetyTimingStage::Integral,
                    OuterSafetyTimingPhase::Terminalization,
                    0,
                    0,
                    true,
                ),
            )
            .is_err()
    );
}
