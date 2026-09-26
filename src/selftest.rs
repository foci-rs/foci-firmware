//! Selftest stage-result types and commissioning diagnostic events.
//!
//! Contains the [`SelftestStageValue`] / [`SelftestResults`] containers
//! streamed to the host during a selftest run.

/// Result of one selftest stage, streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestStageValue {
    /// 0 = PASS, 1 = FAIL, 2 = SKIP.
    pub status: u8,
    /// Stage-specific encoded value.
    pub value: u32,
}

/// Maximum number of commissioning diagnostic events retained per run.
pub const COMMISSION_DIAGNOSTIC_CAPACITY: usize = 16;

/// Structured commissioning diagnostic event streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct CommissionDiagnostic {
    /// Commissioning phase identifier.
    pub phase: u8,
    /// Phase-local diagnostic code.
    pub code: u8,
    /// Diagnostic status (0 = measurement, 1 = rejection/failure, 2 = warning).
    pub status: u8,
    /// First code-specific value.
    pub value0: u32,
    /// Second code-specific value.
    pub value1: u32,
    /// Third code-specific value.
    pub value2: u32,
}

/// Bounded commissioning diagnostic event buffer.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct CommissionDiagnostics {
    events: heapless::Vec<CommissionDiagnostic, COMMISSION_DIAGNOSTIC_CAPACITY>,
}

impl CommissionDiagnostics {
    /// Append one event. Events beyond capacity are dropped.
    pub fn push(&mut self, event: CommissionDiagnostic) {
        let _ = self.events.push(event);
    }

    /// Iterate retained events in insertion order.
    pub fn iter(&self) -> impl Iterator<Item = CommissionDiagnostic> + '_ {
        self.events.iter().copied()
    }
}

impl SelftestStageValue {
    /// Pack two ADC channel offsets into a single u32 for stage 1.
    pub fn adc_offsets(offset_i0: u16, offset_i1: u16) -> Self {
        Self {
            status: 0,
            value: ((offset_i1 as u32) << 16) | offset_i0 as u32,
        }
    }

    /// Coil current measurement (stages 2, 3).
    ///
    /// The i16 is bit-reinterpreted as u16 (two's-complement bit pattern
    /// preserved), then zero-extended to u32. The host decoder must treat
    /// bit 15 of the lower 16 bits as the sign bit to recover negative
    /// values (values >= 0x8000 in the low 16 bits represent negative i16).
    pub fn coil_current(current: i16) -> Self {
        Self {
            status: 0,
            value: current as u16 as u32,
        }
    }

    /// Constructor for a failed stage. `value` may carry an error code or
    /// zero if no detail is available.
    pub fn fail(value: u32) -> Self {
        Self { status: 1, value }
    }

    /// Constructor for a skipped stage.
    pub fn skip() -> Self {
        Self {
            status: 2,
            value: 0,
        }
    }
}

/// Per-stage results collected during a selftest run.
///
/// Stages are populated in phase order as they complete; `None` means
/// the stage was skipped or aborted before running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestResults {
    stages: [Option<SelftestStageValue>; 8],
}

impl SelftestResults {
    /// Write a stage result. `stage` is 1-indexed (1..=8). Out-of-range
    /// stages are silently ignored.
    pub fn set(&mut self, stage: u8, value: SelftestStageValue) {
        if (1..=8).contains(&stage) {
            self.stages[(stage - 1) as usize] = Some(value);
        }
    }

    /// Read a stage result. Returns `None` if unpopulated or out of range.
    pub fn stage(&self, stage: u8) -> Option<SelftestStageValue> {
        if (1..=8).contains(&stage) {
            self.stages[(stage - 1) as usize]
        } else {
            None
        }
    }

    /// Iterate populated stages in order, yielding `(stage_id, value)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (u8, SelftestStageValue)> + '_ {
        self.stages
            .iter()
            .enumerate()
            .filter_map(|(i, o)| o.map(|v| (i as u8 + 1, v)))
    }
}

#[cfg(test)]
mod selftest_results_tests {
    use super::*;

    #[test]
    fn results_default_has_all_stages_unpopulated() {
        let r = SelftestResults::default();
        for stage in 1..=8u8 {
            assert!(r.stage(stage).is_none(), "stage {} should be None", stage);
        }
    }

    #[test]
    fn set_and_get_round_trips_value() {
        let mut r = SelftestResults::default();
        r.set(
            3,
            SelftestStageValue {
                status: 0,
                value: 0x1234,
            },
        );
        let got = r.stage(3).expect("stage 3 populated");
        assert_eq!(got.status, 0);
        assert_eq!(got.value, 0x1234);
    }

    #[test]
    fn adc_offsets_pack_into_value() {
        let v = SelftestStageValue::adc_offsets(0x1111, 0x2222);
        assert_eq!(v.value, (0x2222_u32 << 16) | 0x1111_u32);
        assert_eq!(v.status, 0);
    }

    #[test]
    fn fail_constructor_sets_status_one() {
        let v = SelftestStageValue::fail(42);
        assert_eq!(v.status, 1);
        assert_eq!(v.value, 42);
    }

    #[test]
    fn skip_constructor_sets_status_two_and_zero_value() {
        let v = SelftestStageValue::skip();
        assert_eq!(v.status, 2);
        assert_eq!(v.value, 0);
    }

    #[test]
    fn commission_diagnostics_iterate_in_push_order() {
        let mut diagnostics = CommissionDiagnostics::default();
        let first = CommissionDiagnostic {
            phase: 5,
            code: 1,
            status: 0,
            value0: 10,
            value1: 20,
            value2: 30,
        };
        let second = CommissionDiagnostic {
            phase: 5,
            code: 29,
            status: 1,
            value0: 40,
            value1: 50,
            value2: 60,
        };

        diagnostics.push(first);
        diagnostics.push(second);

        let items: heapless::Vec<_, 4> = diagnostics.iter().collect();
        assert_eq!(items.as_slice(), &[first, second]);
    }

    #[test]
    fn commission_diagnostics_drop_events_after_capacity() {
        let mut diagnostics = CommissionDiagnostics::default();
        for code in 0..(COMMISSION_DIAGNOSTIC_CAPACITY as u8 + 2) {
            diagnostics.push(CommissionDiagnostic {
                phase: 5,
                code,
                status: 0,
                value0: code as u32,
                value1: 0,
                value2: 0,
            });
        }

        let items: heapless::Vec<_, COMMISSION_DIAGNOSTIC_CAPACITY> = diagnostics.iter().collect();
        assert_eq!(items.len(), COMMISSION_DIAGNOSTIC_CAPACITY);
        assert_eq!(items[0].code, 0);
        assert_eq!(
            items[COMMISSION_DIAGNOSTIC_CAPACITY - 1].code,
            COMMISSION_DIAGNOSTIC_CAPACITY as u8 - 1
        );
    }
}
