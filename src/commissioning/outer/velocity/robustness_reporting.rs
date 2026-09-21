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
pub const ROBUSTNESS_SCHEMA_REVISION: u16 = 4;

/// Bytes in the compact schema-4 robustness-terminal reply payload: a
/// 15-byte header, 12 bytes per reversal direction (two directions), and a
/// 12-byte run-level tail carrying the gate's constructed `plant_rate_q`
/// (i64) and `iae_max_q` (i32, scaled to rate-Q seconds).
pub const ROBUSTNESS_TERMINAL_REPLY_BYTES: usize = 51;

/// Scale factor from wire IAE encoding to internal microseconds.
///
/// The `RobustnessEvidence.iae_medians_qs` wire field and the terminal's
/// `iae_max_q` tail field both report IAE quantities in rate-Q seconds; the
/// internal `recovery_iae_q`/`iae_max_q` gate comparison uses rate-Q
/// microseconds. Both encodings convert internal microseconds to seconds via
/// division by this factor with no additional scaling; the coefficient
/// calibration multiplies by this factor to restore the internal unit. This
/// constant guards the load-bearing 1e6 factor against drift.
pub const IAE_WIRE_TO_INTERNAL_US_PER_S: i64 = 1_000_000;

/// Saturating `i64 -> i32` clamp; never panics on out-of-range magnitudes.
/// Mirrors `robustness_cycles::saturating_i64_to_i32`.
fn saturating_i64_to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

/// Encode the compact robustness-terminal ordinary reply payload.
///
/// Fixed 15-byte header (schema revision, run sequence, outcome, cause,
/// selected gains, target velocity), 12 bytes per reversal direction
/// (median/max reconvergence, median forward-settle, overshoot-peak count,
/// valid-cycle/trip/retry counts, inconclusive flag), then a run-level
/// 12-byte tail carrying the gate's constructed `plant_rate_q` (i64, rate-Q
/// units, unscaled) and `iae_max_q` (i32, scaled to rate-Q seconds by
/// [`IAE_WIRE_TO_INTERNAL_US_PER_S`] -- the same scaling `iae_medians_qs`
/// already uses, so the two compare directly once both are on the host),
/// for [`ROBUSTNESS_TERMINAL_REPLY_BYTES`] bytes total.
///
/// `residual_medians_q`, `iae_medians_qs`, and `dac_rms_medians_q` are
/// deliberately absent from this encoding: they live off-terminal and are
/// carried by the detailed reply instead, keeping the ordinary terminal
/// inside the single-frame data budget. `plant_rate_q`/`iae_max_q` are
/// run-level (identical for both directions), so they ride the terminal
/// instead.
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
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.plant_rate_q.to_le_bytes(),
    );
    let iae_max_q_qs = saturating_i64_to_i32(terminal.iae_max_q / IAE_WIRE_TO_INTERNAL_US_PER_S);
    push_reply(&mut bytes, &mut offset, &iae_max_q_qs.to_le_bytes());
    debug_assert_eq!(offset, ROBUSTNESS_TERMINAL_REPLY_BYTES);
    bytes
}

/// Bytes in the schema-4 per-cycle robustness-evidence payload: a 14-byte
/// header (direction, schema revision, off-terminal residual/IAE/dAC_rms
/// medians for this direction), [`ROBUSTNESS_CYCLES_PER_DIRECTION`] 6-byte
/// cycle triples, a 1-byte tail count, and one 6-byte displaced-tail triple.
pub(crate) const ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES: usize = 51;

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

/// Encode one direction's schema-4 per-cycle robustness-evidence payload.
///
/// Fixed 14-byte header (direction, schema revision, this direction's
/// off-terminal residual/IAE/dAC_rms medians) plus
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
    push_reply(
        &mut bytes,
        &mut offset,
        &evidence.dac_rms_medians_q[index].to_le_bytes(),
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
            dac_rms_medians_q: [21, -21],
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
            plant_rate_q: 60_000,
            iae_max_q: 120_000_000_000,
            evidence: RobustnessEvidence {
                directions: [direction_a(), direction_b()],
                residual_medians_q: [17, -17],
                iae_medians_qs: [9, -9],
                dac_rms_medians_q: [21, -21],
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
        assert_eq!(bytes.len(), ROBUSTNESS_TERMINAL_REPLY_BYTES);

        #[rustfmt::skip]
        let expected: [u8; ROBUSTNESS_TERMINAL_REPLY_BYTES] = [
            // schema_revision (wire u8)
            0x04,
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
            // plant_rate_q (i64 LE) = 60_000
            0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            // iae_max_q scaled to rate-Q seconds (i32 LE) = 120_000_000_000 / 1e6 = 120_000
            0xC0, 0xD4, 0x01, 0x00,
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

        assert_eq!(
            i64::from_le_bytes(bytes[39..47].try_into().unwrap()),
            terminal.plant_rate_q
        );
        assert_eq!(
            i32::from_le_bytes(bytes[47..51].try_into().unwrap()),
            i32::try_from(terminal.iae_max_q / IAE_WIRE_TO_INTERNAL_US_PER_S).unwrap()
        );
    }

    /// `plant_rate_q` and `iae_max_q` round-trip through the wire encoding:
    /// `plant_rate_q` unchanged, `iae_max_q` scaled to rate-Q seconds by the
    /// same [`IAE_WIRE_TO_INTERNAL_US_PER_S`] factor the per-cycle IAE medians
    /// already use, so a host reading both fields back can compare them
    /// directly in the same unit.
    #[test]
    fn plant_rate_and_iae_max_round_trip_through_the_wire() {
        let mut terminal = terminal();
        terminal.plant_rate_q = 4_200_000;
        terminal.iae_max_q = 8_400_000_000_000;

        let bytes = encode_robustness_terminal_reply(&terminal);

        let decoded_plant_rate_q = i64::from_le_bytes(bytes[39..47].try_into().unwrap());
        let decoded_iae_max_q_qs = i32::from_le_bytes(bytes[47..51].try_into().unwrap());
        assert_eq!(decoded_plant_rate_q, terminal.plant_rate_q);
        assert_eq!(
            i64::from(decoded_iae_max_q_qs) * IAE_WIRE_TO_INTERNAL_US_PER_S,
            terminal.iae_max_q
        );
    }

    #[test]
    fn terminal_fits_a_single_klipper_frame_at_any_oid() {
        const { assert!(ROBUSTNESS_TERMINAL_REPLY_BYTES <= 54) };
    }

    /// Every schema-4 per-direction field is already wire-narrow, so nothing
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
            0x00, 0x04,
            // residual_median_q (i32 LE) = 17
            0x11, 0x00, 0x00, 0x00,
            // iae_median_qs (i32 LE) = 9
            0x09, 0x00, 0x00, 0x00,
            // dac_rms_median_q (i32 LE) = 21
            0x15, 0x00, 0x00, 0x00,
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
        assert_eq!(
            i32::from_le_bytes([bytes[10], bytes[11], bytes[12], bytes[13]]),
            evidence.dac_rms_medians_q[1]
        );
        // 30 cycle-slot sentinel bytes, then the tail count (0), then the
        // displaced-tail triple as a 6-byte sentinel (no grace tail).
        assert_eq!(&bytes[14..44], &[0xFF_u8; 30][..]);
        assert_eq!(bytes[44], 0);
        assert_eq!(&bytes[45..], &[0xFF_u8; 6][..]);
    }

    #[test]
    fn robustness_cycle_evidence_fits_a_single_klipper_frame_at_any_oid() {
        const { assert!(ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES <= 54) };
    }

    #[test]
    fn iae_wire_seconds_convert_to_internal_microseconds_by_1e6() {
        // The gate compares recovery_iae_q in rate-Q microseconds; the wire field
        // reports rate-Q seconds. The coefficient calibration divides internal-unit
        // IAE by the plant scale, so this factor is load-bearing.
        assert_eq!(IAE_WIRE_TO_INTERNAL_US_PER_S, 1_000_000);
        let wire_seconds_q: i64 = 675_000; // representative reference median
        assert_eq!(
            wire_seconds_q * IAE_WIRE_TO_INTERNAL_US_PER_S,
            675_000_000_000
        );
    }
}
