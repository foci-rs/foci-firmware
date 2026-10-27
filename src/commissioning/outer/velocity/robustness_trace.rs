//! Trace-only late-tracking evidence for the reversal robustness gate.
//!
//! At the campaign terminal, one canonical record per direction and filled cycle slot carries
//! the slot's late-tracking window together with the direction's target rate and reported
//! median, so an offline audit can recompute both without cross-record state.

use crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord;
use crate::commissioning::backend::CommissioningBackend;
use crate::commissioning::types::RobustnessReversalTerminal;

use super::robustness_late_tracking::LateTracking;

/// Record kind of one late-tracking evidence record.
const LATE_TRACKING_RECORD_KIND: u8 = 41;

struct DirectionContext {
    run_sequence: u32,
    direction: u8,
    filled_cycles: u8,
    target_rate_q: i64,
    median_ppm: u32,
}

fn encode_late_tracking_record(
    context: &DirectionContext,
    slot: u8,
    late: LateTracking,
) -> VelocitySweepCanonicalRecord {
    let mut record = VelocitySweepCanonicalRecord::new();
    record.push_u8(LATE_TRACKING_RECORD_KIND);
    record.push_u32(context.run_sequence);
    record.push_u8(context.direction);
    record.push_u8(slot);
    record.push_u8(context.filled_cycles);
    record.push_u32(late.ppm);
    record.push_i32(late.window_delta_counts);
    record.push_u32(late.window_elapsed_us);
    record.push_u32(late.window_start_us);
    let target = context.target_rate_q as u64;
    record.push_u32(target as u32);
    record.push_u32((target >> 32) as u32);
    record.push_u32(context.median_ppm);
    record
}

/// Emit the late-tracking evidence of every filled cycle slot in both directions.
pub(crate) fn emit_late_tracking_records<B: CommissioningBackend>(
    backend: &mut B,
    terminal: &RobustnessReversalTerminal,
) {
    let evidence = &terminal.evidence;
    for (direction, cycles) in evidence.late_cycles.iter().enumerate() {
        let context = DirectionContext {
            run_sequence: terminal.run_sequence,
            direction: direction as u8,
            filled_cycles: cycles.iter().flatten().count() as u8,
            target_rate_q: evidence.late_target_rate_q[direction],
            median_ppm: evidence.late_tracking_medians_ppm[direction],
        };
        for (slot, late) in cycles.iter().enumerate() {
            if let Some(late) = late {
                backend.record_velocity_sweep_trace_record(encode_late_tracking_record(
                    &context, slot as u8, *late,
                ));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::commissioning::CommissioningEngine;
    use crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
    use crate::commissioning::outer::velocity::robustness_evidence::{
        RobustnessDirectionSummary, RobustnessEvidence,
    };
    use crate::commissioning::outer::velocity::robustness_late_tracking::LateTracking;
    use crate::commissioning::outer::velocity::robustness_reporting::ROBUSTNESS_SCHEMA_REVISION;
    use crate::commissioning::types::{RobustnessOutcome, RobustnessReversalTerminal, StepResult};

    fn direction() -> RobustnessDirectionSummary {
        RobustnessDirectionSummary {
            median_reconvergence_ms: 120,
            max_reconvergence_ms: 140,
            median_forward_settle_ms: 90,
            peak_rate_permille: 1_100,
            valid_cycles: 3,
            trip_count: 0,
            retry_count: 0,
            inconclusive: false,
            peak_unscored: false,
        }
    }

    fn late(
        ppm: u32,
        window_delta_counts: i32,
        window_elapsed_us: u32,
        window_start_us: u32,
    ) -> Option<LateTracking> {
        Some(LateTracking {
            ppm,
            window_delta_counts,
            window_elapsed_us,
            window_start_us,
        })
    }

    fn terminal() -> RobustnessReversalTerminal {
        let mut late_cycles = [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2];
        late_cycles[0][0] = late(6_250, 161, 16_000, 600_000);
        late_cycles[0][1] = late(12_500, 158, 16_000, 612_000);
        late_cycles[0][3] = late(187, 160, 16_003, 684_000);
        late_cycles[1][2] = late(18_750, -163, 16_000, 650_000);
        RobustnessReversalTerminal {
            schema_revision: ROBUSTNESS_SCHEMA_REVISION,
            run_sequence: 0x0102_0304,
            outcome: RobustnessOutcome::Inconclusive,
            cause: 0,
            selected_p: 1_024,
            selected_i: 512,
            target_velocity_rpm: 150,
            plant_rate_q: 60_000,
            iae_max_q: 120_000_000,
            evidence: RobustnessEvidence {
                directions: [direction(), direction()],
                residual_medians_q: [0, 0],
                iae_medians_qs: [0, 0],
                dac_rms_medians_q: [0, 0],
                cycles: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
                tail_counts: [0, 0],
                displaced_tails: [None, None],
                late_tracking_medians_ppm: [6_250, 18_750],
                late_target_rate_q: [10_000_000, -10_000_000],
                late_cycles,
                late_suffix_excluded: [[false; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
            },
            run_started_us: 1_000,
            run_completed_us: 9_000_000,
            safety_fault: None,
        }
    }

    // Golden bytes: foci-trace's decoder round-trip test carries this exact first record.
    #[test]
    fn trace_build_emits_one_record_per_filled_cycle_with_the_audit_tuple() {
        let mut engine = CommissioningEngine::new_selftest(0);
        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();

        engine.emit_trace_after_step(
            &mut backend,
            &StepResult::RobustnessReversalTerminal(terminal()),
        );

        let records = &backend.velocity_sweep_trace_records;
        assert_eq!(records.len(), 4);
        #[rustfmt::skip]
        let first: [u8; 36] = [
            41,
            0x04, 0x03, 0x02, 0x01,
            0, 0, 3,
            0x6A, 0x18, 0x00, 0x00,
            0xA1, 0x00, 0x00, 0x00,
            0x80, 0x3E, 0x00, 0x00,
            0xC0, 0x27, 0x09, 0x00,
            0x80, 0x96, 0x98, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x6A, 0x18, 0x00, 0x00,
        ];
        assert_eq!(records[0].as_bytes(), &first);
        let tuple = |index: usize| {
            let bytes = records[index].as_bytes();
            assert_eq!(bytes.len(), 36);
            let u32_at = |at: usize| u32::from_le_bytes(bytes[at..at + 4].try_into().unwrap());
            (
                bytes[5],
                bytes[6],
                bytes[7],
                u32_at(8),
                u32_at(12) as i32,
                u32_at(16),
                u32_at(20),
                i64::from_le_bytes(bytes[24..32].try_into().unwrap()),
                u32_at(32),
            )
        };
        assert_eq!(
            tuple(1),
            (0, 1, 3, 12_500, 158, 16_000, 612_000, 10_000_000, 6_250)
        );
        assert_eq!(
            tuple(2),
            (0, 3, 3, 187, 160, 16_003, 684_000, 10_000_000, 6_250)
        );
        assert_eq!(
            tuple(3),
            (1, 2, 1, 18_750, -163, 16_000, 650_000, -10_000_000, 18_750)
        );
    }
}
