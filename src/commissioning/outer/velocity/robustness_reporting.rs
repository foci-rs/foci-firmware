//! Compact wire reporting for reversal-standstill robustness gates.

use crate::commissioning::outer::velocity::robustness_cycles::ROBUSTNESS_CYCLES_PER_DIRECTION;
use crate::commissioning::outer::velocity::robustness_evidence::RobustnessEvidence;
use crate::commissioning::types::RobustnessReversalTerminal;

/// Independent robustness evidence schema revision.
///
/// Packed as a `u8` on the wire; kept `u16` for schema comparisons elsewhere.
pub const ROBUSTNESS_SCHEMA_REVISION: u16 = 5;

/// Bytes in the compact schema-5 robustness-terminal reply payload.
pub const ROBUSTNESS_TERMINAL_REPLY_BYTES: usize = 51;

/// Scale factor from wire IAE encoding to internal microseconds.
///
/// Converts internal rate-Q microseconds to the rate-Q seconds reported on the wire.
pub const IAE_WIRE_TO_INTERNAL_US_PER_S: i64 = 1_000_000;

fn saturating_i64_to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

/// Encode the compact robustness-terminal ordinary reply payload.
///
/// Layout: 15-byte header, 12 bytes per reversal direction (byte 11 is a flags byte: bit 0
/// `inconclusive`, bit 1 `peak_unscored`), 12-byte run-level tail
/// (`plant_rate_q` i64, `iae_max_q` i32 scaled by [`IAE_WIRE_TO_INTERNAL_US_PER_S`]).
/// Off-terminal medians are carried by the detailed reply instead.
pub(crate) fn encode_robustness_terminal_reply(
    terminal: &RobustnessReversalTerminal,
) -> [u8; ROBUSTNESS_TERMINAL_REPLY_BYTES] {
    let mut bytes = [0_u8; ROBUSTNESS_TERMINAL_REPLY_BYTES];
    let mut offset = 0;
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
            &direction.peak_rate_permille.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &[
                direction.valid_cycles,
                direction.trip_count,
                direction.retry_count,
                u8::from(direction.inconclusive) | (u8::from(direction.peak_unscored) << 1),
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

/// Bytes in the schema-5 per-cycle robustness-evidence payload.
pub(crate) const ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES: usize = 51;

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

/// Encode one direction's schema-5 per-cycle robustness-evidence payload.
///
/// Layout: 14-byte header, [`ROBUSTNESS_CYCLES_PER_DIRECTION`] 6-byte cycle triples, tail
/// count, and displaced-tail triple. Unfilled slots encode as [`CYCLE_SLOT_UNFILLED`].
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
            peak_rate_permille: 0,
            valid_cycles: 5,
            trip_count: 2,
            retry_count: 1,
            inconclusive: false,
            peak_unscored: true,
        }
    }

    fn direction_b() -> RobustnessDirectionSummary {
        RobustnessDirectionSummary {
            median_reconvergence_ms: 2_000,
            max_reconvergence_ms: 2_500,
            median_forward_settle_ms: 450,
            peak_rate_permille: 1_234,
            valid_cycles: 4,
            trip_count: 1,
            retry_count: 2,
            inconclusive: true,
            peak_unscored: false,
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
            0x05,
            0x44, 0x33, 0x22, 0x11,
            0x01, 0x02,
            0x00, 0x04, 0x00, 0x02,
            0x24, 0xFA, 0xFF, 0xFF,
            0xE8, 0x03, 0xB0, 0x04, 0x2C, 0x01, 0x00, 0x00,
            0x05, 0x02, 0x01, 0x02,
            0xD0, 0x07, 0xC4, 0x09, 0xC2, 0x01, 0xD2, 0x04,
            0x04, 0x01, 0x02, 0x01,
            0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0xC0, 0xD4, 0x01, 0x00,
        ];
        assert_eq!(bytes, expected);

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
                direction.peak_rate_permille
            );
            assert_eq!(bytes[base + 8], direction.valid_cycles);
            assert_eq!(bytes[base + 9], direction.trip_count);
            assert_eq!(bytes[base + 10], direction.retry_count);
            assert_eq!(
                bytes[base + 11],
                u8::from(direction.inconclusive) | (u8::from(direction.peak_unscored) << 1)
            );
        }

        assert_eq!(
            i64::from_le_bytes(bytes[39..47].try_into().unwrap()),
            terminal.plant_rate_q
        );
        assert_eq!(
            i32::from_le_bytes(bytes[47..51].try_into().unwrap()),
            i32::try_from(terminal.iae_max_q / IAE_WIRE_TO_INTERNAL_US_PER_S).unwrap()
        );
    }

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

    #[test]
    fn encoder_saturates_out_of_range_values() {
        let mut terminal = terminal();
        let saturated = RobustnessDirectionSummary {
            median_reconvergence_ms: u16::MAX,
            max_reconvergence_ms: u16::MAX,
            median_forward_settle_ms: u16::MAX,
            peak_rate_permille: u16::MAX,
            valid_cycles: u8::MAX,
            trip_count: u8::MAX,
            retry_count: u8::MAX,
            inconclusive: true,
            peak_unscored: true,
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
            assert_eq!(bytes[base + 11], 0b11);
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
            0x00, 0x05,
            0x11, 0x00, 0x00, 0x00,
            0x09, 0x00, 0x00, 0x00,
            0x15, 0x00, 0x00, 0x00,
            0x64, 0x00, 0x01, 0x00, 0x32, 0x00,
            0x6E, 0x00, 0x00, 0x00, 0x37, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x78, 0x00, 0x02, 0x00, 0x3C, 0x00,
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0x01,
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
        assert_eq!(IAE_WIRE_TO_INTERNAL_US_PER_S, 1_000_000);
        let wire_seconds_q: i64 = 675_000;
        assert_eq!(
            wire_seconds_q * IAE_WIRE_TO_INTERNAL_US_PER_S,
            675_000_000_000
        );
    }
}
