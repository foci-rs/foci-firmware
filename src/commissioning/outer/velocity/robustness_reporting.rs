//! Compact wire reporting for reversal-standstill robustness gates.

use crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
use crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence;
use crate::commissioning::types::RobustnessReversalTerminal;

/// Independent robustness evidence schema revision.
///
/// The wire terminal packs this as a single `u8` (see
/// [`encode_robustness_terminal_reply`]); the constant and
/// [`RobustnessReversalTerminal::schema_revision`](crate::commissioning::types::RobustnessReversalTerminal::schema_revision)
/// stay `u16` so schema comparisons elsewhere (the detailed reply, host
/// tooling) do not need a second, narrower type.
pub const ROBUSTNESS_SCHEMA_REVISION: u16 = 2;

/// Bytes in the compact schema-2 robustness-terminal reply payload: a
/// 15-byte header plus 12 bytes per reversal direction (two directions).
pub const ROBUSTNESS_TERMINAL_REPLY_BYTES: usize = 39;

/// Encode the compact robustness-terminal ordinary reply payload.
///
/// Fixed 15-byte header (schema revision, run sequence, outcome, cause,
/// selected gains, target velocity) plus 12 bytes per reversal direction
/// (median/max reconvergence, median forward-settle, overshoot-peak count,
/// valid-cycle/trip/retry counts, inconclusive flag), for 39 bytes total.
///
/// `residual_medians_q` and `iae_medians_qs` are deliberately absent from
/// this encoding: they live off-terminal and are carried by the detailed
/// reply instead, keeping the ordinary terminal inside the single-frame
/// data budget.
pub(crate) fn encode_robustness_terminal_reply(
    terminal: &RobustnessReversalTerminal,
) -> [u8; ROBUSTNESS_TERMINAL_REPLY_BYTES] {
    let mut bytes = [0_u8; ROBUSTNESS_TERMINAL_REPLY_BYTES];
    let mut offset = 0;
    // Wire field is u8; the struct field stays u16 for schema comparisons.
    push_reply(&mut bytes, &mut offset, &[terminal.schema_revision as u8]);
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.run_sequence.to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &[terminal.outcome as u8, terminal.cause],
    );
    push_reply(&mut bytes, &mut offset, &terminal.selected_p.to_le_bytes());
    push_reply(&mut bytes, &mut offset, &terminal.selected_i.to_le_bytes());
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.target_velocity_rpm.to_le_bytes(),
    );
    for direction in terminal.evidence.directions {
        push_reply(
            &mut bytes,
            &mut offset,
            &direction.median_reconvergence_ms.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &direction.max_reconvergence_ms.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &direction.median_forward_settle_ms.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &direction.overshoot_peak_counts.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &[
                direction.valid_cycles,
                direction.trip_count,
                direction.retry_count,
                u8::from(direction.inconclusive),
            ],
        );
    }
    debug_assert_eq!(offset, ROBUSTNESS_TERMINAL_REPLY_BYTES);
    bytes
}

/// Bytes in the schema-2 per-cycle robustness-evidence payload: a 10-byte
/// header (direction, schema revision, off-terminal residual/IAE medians
/// for this direction), [`ROBUSTNESS_CYCLES_PER_DIRECTION`] 6-byte cycle
/// triples, a 1-byte tail count, and one 6-byte displaced-tail triple.
pub(crate) const ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES: usize = 47;

/// Sentinel value marking an unfilled cycle slot's `u16` fields.
const CYCLE_SLOT_UNFILLED: u16 = 0xFFFF;

/// One reversal-probe cycle's wire-narrow evidence triple.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CycleWire {
    /// Time to reconverge to the target band after the reversal, in ms.
    pub reconvergence_ms: u16,
    /// Saturating count of overshoot peaks observed during this cycle.
    pub overshoot_counts: u16,
    /// Time to settle in the forward direction after reconvergence, in ms.
    pub forward_settle_ms: u16,
}

/// Encode one direction's schema-2 per-cycle robustness-evidence payload.
///
/// Fixed 10-byte header (direction, schema revision, this direction's
/// off-terminal residual/IAE medians) plus
/// [`ROBUSTNESS_CYCLES_PER_DIRECTION`] cycle triples (reconvergence,
/// overshoot, forward-settle), 6 bytes each; then this direction's tail
/// count (1 byte) and its displaced (grace-retried) tail triple (6 bytes),
/// for [`ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES`] total. An unfilled cycle
/// slot and an absent displaced tail both encode all three `u16` fields as
/// [`CYCLE_SLOT_UNFILLED`].
pub(crate) fn encode_robustness_cycle_evidence(
    direction: u8,
    evidence: &RobustnessEvidence,
    cycles: &[Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION],
) -> [u8; ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES] {
    debug_assert!((direction as usize) < evidence.directions.len());
    let mut bytes = [0_u8; ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES];
    let mut offset = 0;
    let index = direction as usize;
    push_reply(
        &mut bytes,
        &mut offset,
        &[direction, ROBUSTNESS_SCHEMA_REVISION as u8],
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &evidence.residual_medians_q[index].to_le_bytes(),
    );
    push_reply(
        &mut bytes,
        &mut offset,
        &evidence.iae_medians_qs[index].to_le_bytes(),
    );
    for cycle in cycles {
        push_cycle_triple(&mut bytes, &mut offset, cycle);
    }
    push_reply(&mut bytes, &mut offset, &[evidence.tail_counts[index]]);
    push_cycle_triple(&mut bytes, &mut offset, &evidence.displaced_tails[index]);
    debug_assert_eq!(offset, ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES);
    bytes
}

/// Encode one cycle triple: three `u16` fields when present, or three
/// [`CYCLE_SLOT_UNFILLED`] sentinels when absent.
fn push_cycle_triple<const N: usize>(
    destination: &mut [u8; N],
    offset: &mut usize,
    cycle: &Option<CycleWire>,
) {
    match cycle {
        Some(cycle) => {
            push_reply(destination, offset, &cycle.reconvergence_ms.to_le_bytes());
            push_reply(destination, offset, &cycle.overshoot_counts.to_le_bytes());
            push_reply(destination, offset, &cycle.forward_settle_ms.to_le_bytes());
        }
        None => {
            for _ in 0..3 {
                push_reply(destination, offset, &CYCLE_SLOT_UNFILLED.to_le_bytes());
            }
        }
    }
}

fn push_reply<const N: usize>(destination: &mut [u8; N], offset: &mut usize, value: &[u8]) {
    let end = offset
        .checked_add(value.len())
        .expect("fixed robustness reply length fits usize");
    destination[*offset..end].copy_from_slice(value);
    *offset = end;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commissioning::outer::velocity::robustness_evidence::{
        RobustnessDirectionSummary, RobustnessEvidence,
    };
    use crate::commissioning::types::RobustnessOutcome;

    fn direction_a() -> RobustnessDirectionSummary {
        RobustnessDirectionSummary {
            median_reconvergence_ms: 1_000,
            max_reconvergence_ms: 1_200,
            median_forward_settle_ms: 300,
            overshoot_peak_counts: 7,
            valid_cycles: 5,
            trip_count: 2,
            retry_count: 1,
            inconclusive: false,
        }
    }

    fn direction_b() -> RobustnessDirectionSummary {
        RobustnessDirectionSummary {
            median_reconvergence_ms: 2_000,
            max_reconvergence_ms: 2_500,
            median_forward_settle_ms: 450,
            overshoot_peak_counts: 3,
            valid_cycles: 4,
            trip_count: 1,
            retry_count: 2,
            inconclusive: true,
        }
    }

    fn cycle(reconvergence_ms: u16, overshoot_counts: u16, forward_settle_ms: u16) -> CycleWire {
        CycleWire {
            reconvergence_ms,
            overshoot_counts,
            forward_settle_ms,
        }
    }

    fn evidence() -> RobustnessEvidence {
        RobustnessEvidence {
            directions: [direction_a(), direction_b()],
            residual_medians_q: [17, -17],
            iae_medians_qs: [9, -9],
            cycles: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
            // Direction 0 grace-retried one tail; direction 1 had none.
            tail_counts: [1, 0],
            displaced_tails: [Some(cycle(700, 5, 40)), None],
        }
    }

    fn terminal() -> RobustnessReversalTerminal {
        RobustnessReversalTerminal {
            schema_revision: ROBUSTNESS_SCHEMA_REVISION,
            run_sequence: 0x1122_3344,
            outcome: RobustnessOutcome::Rejected,
            cause: 2,
            selected_p: 1_024,
            selected_i: 512,
            target_velocity_rpm: -1_500,
            evidence: RobustnessEvidence {
                directions: [direction_a(), direction_b()],
                residual_medians_q: [17, -17],
                iae_medians_qs: [9, -9],
                cycles: [[None; ROBUSTNESS_CYCLES_PER_DIRECTION]; 2],
                tail_counts: [0, 0],
                displaced_tails: [None, None],
            },
            run_started_us: 10_000,
            run_completed_us: 90_000,
            safety_fault: None,
        }
    }

    #[test]
    fn encode_robustness_terminal_reply_is_stable_and_within_wire_budget() {
        let terminal = terminal();
        let bytes = encode_robustness_terminal_reply(&terminal);
        assert_eq!(bytes.len(), 39);

        #[rustfmt::skip]
        let expected: [u8; ROBUSTNESS_TERMINAL_REPLY_BYTES] = [
            // schema_revision (wire u8)
            0x02,
            // run_sequence (u32 LE)
            0x44, 0x33, 0x22, 0x11,
            // outcome, cause
            0x01, 0x02,
            // selected_p, selected_i (u16 LE)
            0x00, 0x04, 0x00, 0x02,
            // target_velocity_rpm (i32 LE)
            0x24, 0xFA, 0xFF, 0xFF,
            // direction 0: median/max reconvergence, median settle, overshoot (u16 LE)
            0xE8, 0x03, 0xB0, 0x04, 0x2C, 0x01, 0x07, 0x00,
            // direction 0: valid_cycles, trip_count, retry_count, inconclusive
            0x05, 0x02, 0x01, 0x00,
            // direction 1: median/max reconvergence, median settle, overshoot (u16 LE)
            0xD0, 0x07, 0xC4, 0x09, 0xC2, 0x01, 0x03, 0x00,
            // direction 1: valid_cycles, trip_count, retry_count, inconclusive
            0x04, 0x01, 0x02, 0x01,
        ];
        assert_eq!(bytes, expected);

        // Decoded spot checks documenting the field order/offsets.
        assert_eq!(bytes[0], terminal.schema_revision as u8);
        assert_eq!(
            u32::from_le_bytes([bytes[1], bytes[2], bytes[3], bytes[4]]),
            terminal.run_sequence
        );
        assert_eq!(bytes[5], terminal.outcome as u8);
        assert_eq!(bytes[6], terminal.cause);
        assert_eq!(
            u16::from_le_bytes([bytes[7], bytes[8]]),
            terminal.selected_p
        );
        assert_eq!(
            u16::from_le_bytes([bytes[9], bytes[10]]),
            terminal.selected_i
        );
        assert_eq!(
            i32::from_le_bytes([bytes[11], bytes[12], bytes[13], bytes[14]]),
            terminal.target_velocity_rpm
        );

        for (index, direction) in terminal.evidence.directions.into_iter().enumerate() {
            let base = 15 + index * 12;
            assert_eq!(
                u16::from_le_bytes([bytes[base], bytes[base + 1]]),
                direction.median_reconvergence_ms
            );
            assert_eq!(
                u16::from_le_bytes([bytes[base + 2], bytes[base + 3]]),
                direction.max_reconvergence_ms
            );
            assert_eq!(
                u16::from_le_bytes([bytes[base + 4], bytes[base + 5]]),
                direction.median_forward_settle_ms
            );
            assert_eq!(
                u16::from_le_bytes([bytes[base + 6], bytes[base + 7]]),
                direction.overshoot_peak_counts
            );
            assert_eq!(bytes[base + 8], direction.valid_cycles);
            assert_eq!(bytes[base + 9], direction.trip_count);
            assert_eq!(bytes[base + 10], direction.retry_count);
            assert_eq!(bytes[base + 11], u8::from(direction.inconclusive));
        }
        // residual_medians_q / iae_medians_qs are off-terminal: not encoded.
    }

    #[test]
    fn terminal_fits_a_single_klipper_frame_at_any_oid() {
        assert!(ROBUSTNESS_TERMINAL_REPLY_BYTES <= 54);
    }

    /// Every schema-2 per-direction field is already wire-narrow, so nothing
    /// in the encoder itself clamps a wider value. This instead proves the
    /// wire format round-trips a field already saturated to its type's
    /// boundary (`u16::MAX`/`u8::MAX`, as a saturating upstream conversion --
    /// the ledger's own saturating i64-to-i32 clamp for the off-terminal
    /// fields, or an equivalent narrowing for its ms fields -- would produce)
    /// without further truncation or corruption.
    #[test]
    fn encoder_saturates_out_of_range_values() {
        let mut terminal = terminal();
        let saturated = RobustnessDirectionSummary {
            median_reconvergence_ms: u16::MAX,
            max_reconvergence_ms: u16::MAX,
            median_forward_settle_ms: u16::MAX,
            overshoot_peak_counts: u16::MAX,
            valid_cycles: u8::MAX,
            trip_count: u8::MAX,
            retry_count: u8::MAX,
            inconclusive: true,
        };
        terminal.evidence.directions = [saturated, saturated];

        let bytes = encode_robustness_terminal_reply(&terminal);

        for index in 0..2 {
            let base = 15 + index * 12;
            assert_eq!(u16::from_le_bytes([bytes[base], bytes[base + 1]]), u16::MAX);
            assert_eq!(
                u16::from_le_bytes([bytes[base + 2], bytes[base + 3]]),
                u16::MAX
            );
            assert_eq!(
                u16::from_le_bytes([bytes[base + 4], bytes[base + 5]]),
                u16::MAX
            );
            assert_eq!(
                u16::from_le_bytes([bytes[base + 6], bytes[base + 7]]),
                u16::MAX
            );
            assert_eq!(bytes[base + 8], u8::MAX);
            assert_eq!(bytes[base + 9], u8::MAX);
            assert_eq!(bytes[base + 10], u8::MAX);
            assert_eq!(bytes[base + 11], 1);
        }
    }

    #[test]
    fn encode_robustness_cycle_evidence_is_stable_and_within_wire_budget() {
        let evidence = evidence();
        let cycles: [Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION] = [
            Some(cycle(100, 1, 50)),
            Some(cycle(110, 0, 55)),
            None,
            Some(cycle(120, 2, 60)),
            None,
        ];

        let bytes = encode_robustness_cycle_evidence(0, &evidence, &cycles);
        assert_eq!(bytes.len(), ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES);

        #[rustfmt::skip]
        let expected: [u8; ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES] = [
            // direction, schema
            0x00, 0x02,
            // residual_median_q (i32 LE) = 17
            0x11, 0x00, 0x00, 0x00,
            // iae_median_qs (i32 LE) = 9
            0x09, 0x00, 0x00, 0x00,
            // cycle 0: reconvergence, overshoot, forward_settle (u16 LE)
            0x64, 0x00, 0x01, 0x00, 0x32, 0x00,
            // cycle 1
            0x6E, 0x00, 0x00, 0x00, 0x37, 0x00,
            // cycle 2: unfilled sentinel
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            // cycle 3
            0x78, 0x00, 0x02, 0x00, 0x3C, 0x00,
            // cycle 4: unfilled sentinel
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            // tail_count = 1
            0x01,
            // displaced tail triple: reconvergence 700, overshoot 5, settle 40
            0xBC, 0x02, 0x05, 0x00, 0x28, 0x00,
        ];
        assert_eq!(bytes, expected);
    }

    #[test]
    fn encode_robustness_cycle_evidence_encodes_unfilled_slots_as_sentinel() {
        let evidence = evidence();
        let cycles: [Option<CycleWire>; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [None, None, None, None, None];

        let bytes = encode_robustness_cycle_evidence(1, &evidence, &cycles);

        assert_eq!(bytes[0], 1);
        assert_eq!(bytes[1], ROBUSTNESS_SCHEMA_REVISION as u8);
        assert_eq!(
            i32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
            evidence.residual_medians_q[1]
        );
        assert_eq!(
            i32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]),
            evidence.iae_medians_qs[1]
        );
        // 30 cycle-slot sentinel bytes, then the tail count (0), then the
        // displaced-tail triple as a 6-byte sentinel (no grace tail).
        assert_eq!(&bytes[10..40], &[0xFF_u8; 30][..]);
        assert_eq!(bytes[40], 0);
        assert_eq!(&bytes[41..], &[0xFF_u8; 6][..]);
    }

    #[test]
    fn robustness_cycle_evidence_fits_a_single_klipper_frame_at_any_oid() {
        assert!(ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES <= 54);
    }
}
