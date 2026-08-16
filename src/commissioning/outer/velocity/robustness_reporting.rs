//! Compact wire reporting for reversal-standstill robustness gates.

use crate::commissioning::types::RobustnessReversalTerminal;

/// Independent robustness evidence schema revision.
#[allow(dead_code)]
pub const ROBUSTNESS_SCHEMA_REVISION: u16 = 1;

/// Bytes in the compact ordinary robustness-terminal reply payload.
#[allow(dead_code)]
pub const ROBUSTNESS_TERMINAL_REPLY_BYTES: usize = 54;

/// Encode the compact robustness-terminal ordinary reply payload.
///
/// Fixed 16-byte header plus 19 bytes per reversal direction (two
/// directions), for 54 bytes total. `reconvergence_ratio_ppm` is
/// deliberately absent: both of its inputs (`reconvergence_time_us`,
/// `forward_settle_time_us`) are encoded, so the host derives the ratio
/// instead of the firmware shipping a third, redundant field.
///
/// The IAE wire field is `recovery_iae_qs` (rate-Q seconds): the internal
/// `recovery_iae_q` is rate-Q microseconds, which a real reversal transient
/// drives past `i32::MAX` in a single tick, so it is divided by 1_000_000
/// before the saturating i32 clamp. The i64 gate still compares the full
/// microsecond value; only the wire readout is rescaled.
#[allow(dead_code)]
pub(crate) fn encode_robustness_terminal_reply(
    terminal: &RobustnessReversalTerminal,
) -> [u8; ROBUSTNESS_TERMINAL_REPLY_BYTES] {
    let mut bytes = [0_u8; ROBUSTNESS_TERMINAL_REPLY_BYTES];
    let mut offset = 0;
    push_reply(
        &mut bytes,
        &mut offset,
        &terminal.schema_revision.to_le_bytes(),
    );
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
            &direction.metrics.reconvergence_time_us.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &saturating_i64_to_i32(direction.metrics.settled_residual_q).to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &saturating_i64_to_i32(direction.metrics.recovery_iae_q / 1_000_000).to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &direction.metrics.forward_settle_time_us.to_le_bytes(),
        );
        push_reply(
            &mut bytes,
            &mut offset,
            &[
                direction.tripped,
                direction.retry_count,
                u8::from(direction.inconclusive),
            ],
        );
    }
    debug_assert_eq!(offset, ROBUSTNESS_TERMINAL_REPLY_BYTES);
    bytes
}

/// Saturating `i64 -> i32` clamp; never panics on out-of-range magnitudes.
#[allow(dead_code)]
fn saturating_i64_to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

#[allow(dead_code)]
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
        RobustnessDirectionEvidence, RobustnessEvidence,
    };
    use crate::commissioning::outer::velocity::robustness_metrics::ReversalMetrics;
    use crate::commissioning::types::RobustnessOutcome;

    fn direction(sign: i64) -> RobustnessDirectionEvidence {
        RobustnessDirectionEvidence {
            metrics: ReversalMetrics {
                reconvergence_time_us: 12_345,
                settled_residual_q: 250 * sign,
                recovery_iae_q: 9_000_000 * sign,
                forward_settle_time_us: 8_765,
                reconvergence_ratio_ppm: 1_409_000,
            },
            tripped: 0b0000_0101,
            retry_count: 2,
            inconclusive: false,
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
                directions: [direction(1), direction(-1)],
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
        assert_eq!(bytes.len(), 54);

        assert_eq!(
            u16::from_le_bytes([bytes[0], bytes[1]]),
            terminal.schema_revision
        );
        assert_eq!(
            u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]),
            terminal.run_sequence
        );
        assert_eq!(bytes[6], terminal.outcome as u8);
        assert_eq!(bytes[7], terminal.cause);
        assert_eq!(
            u16::from_le_bytes([bytes[8], bytes[9]]),
            terminal.selected_p
        );
        assert_eq!(
            u16::from_le_bytes([bytes[10], bytes[11]]),
            terminal.selected_i
        );
        assert_eq!(
            i32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]),
            terminal.target_velocity_rpm
        );

        for (index, direction) in terminal.evidence.directions.into_iter().enumerate() {
            let base = 16 + index * 19;
            assert_eq!(
                u32::from_le_bytes([
                    bytes[base],
                    bytes[base + 1],
                    bytes[base + 2],
                    bytes[base + 3]
                ]),
                direction.metrics.reconvergence_time_us
            );
            assert_eq!(
                i32::from_le_bytes([
                    bytes[base + 4],
                    bytes[base + 5],
                    bytes[base + 6],
                    bytes[base + 7]
                ]),
                i32::try_from(direction.metrics.settled_residual_q)
                    .expect("fixture residual fits i32")
            );
            // Wire field `recovery_iae_qs`: internal rate-Q microseconds
            // rescaled to rate-Q seconds (/ 1_000_000) before the i32 clamp.
            assert_eq!(
                i32::from_le_bytes([
                    bytes[base + 8],
                    bytes[base + 9],
                    bytes[base + 10],
                    bytes[base + 11]
                ]),
                saturating_i64_to_i32(direction.metrics.recovery_iae_q / 1_000_000)
            );
            assert_eq!(
                u32::from_le_bytes([
                    bytes[base + 12],
                    bytes[base + 13],
                    bytes[base + 14],
                    bytes[base + 15]
                ]),
                direction.metrics.forward_settle_time_us
            );
            assert_eq!(bytes[base + 16], direction.tripped);
            assert_eq!(bytes[base + 17], direction.retry_count);
            assert_eq!(bytes[base + 18], u8::from(direction.inconclusive));
        }
        // reconvergence_ratio_ppm is intentionally absent from the wire encoding.
    }

    #[test]
    fn encode_robustness_terminal_reply_saturates_out_of_range_i64_fields() {
        let mut terminal = terminal();
        terminal.evidence.directions[0].metrics.settled_residual_q = i64::MAX;
        terminal.evidence.directions[0].metrics.recovery_iae_q = i64::MIN;

        let bytes = encode_robustness_terminal_reply(&terminal);

        let residual = i32::from_le_bytes([bytes[20], bytes[21], bytes[22], bytes[23]]);
        let iae_qs = i32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        assert_eq!(residual, i32::MAX);
        assert_eq!(iae_qs, i32::MIN);
    }
}
