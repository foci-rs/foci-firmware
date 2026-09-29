//! Selftest stage-result types and commissioning diagnostic events.

/// Result of one selftest stage, streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestStageValue {
    /// 0 = PASS, 1 = FAIL, 2 = SKIP.
    pub status: u8,
    pub value: u32,
}

/// Maximum number of commissioning diagnostic events retained per run.
pub const COMMISSION_DIAGNOSTIC_CAPACITY: usize = 16;

/// Structured commissioning diagnostic event streamed to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct CommissionDiagnostic {
    pub phase: u8,
    pub code: u8,
    /// Diagnostic status (0 = measurement, 1 = rejection/failure, 2 = warning).
    pub status: u8,
    pub value0: u32,
    pub value1: u32,
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
    /// Pack two ADC channel offsets into a single u32 for
    /// [`SelftestStage::AdcCalibration`].
    pub fn adc_offsets(offset_i0: u16, offset_i1: u16) -> Self {
        Self {
            status: 0,
            value: ((offset_i1 as u32) << 16) | offset_i0 as u32,
        }
    }

    /// Coil current for `CoilA`/`CoilB`: the i16's two's-complement bits zero-extended to u32;
    /// the host must treat bit 15 as the sign.
    pub fn coil_current(current: i16) -> Self {
        Self {
            status: 0,
            value: current as u16 as u32,
        }
    }

    /// Failed stage; `value` may carry an error code.
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

/// A selftest stage. The discriminant is the stage id sent to the host,
/// which names stages by the same ids (`SELFTEST_STAGES` in
/// klipper-foci's `selftest.py`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum SelftestStage {
    AdcCalibration = 1,
    CoilA = 2,
    CoilB = 3,
    PhaseWiring = 4,
    Encoder = 5,
    /// Encoder direction relative to PHI_E (0 = forward, 1 = reversed).
    EncoderDirection = 6,
    RModelEvidence = 7,
    LModelEvidence = 8,
}

impl SelftestStage {
    /// Every stage, in wire-id order.
    pub const ALL: [Self; 8] = [
        Self::AdcCalibration,
        Self::CoilA,
        Self::CoilB,
        Self::PhaseWiring,
        Self::Encoder,
        Self::EncoderDirection,
        Self::RModelEvidence,
        Self::LModelEvidence,
    ];

    /// Stage id sent to the host.
    pub const fn id(self) -> u8 {
        self as u8
    }

    const fn index(self) -> usize {
        self as usize - 1
    }
}

/// Per-stage results; `None` means the stage was skipped or aborted before running.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, defmt::Format)]
pub struct SelftestResults {
    stages: [Option<SelftestStageValue>; SelftestStage::ALL.len()],
}

impl SelftestResults {
    /// Write a stage result.
    pub fn set(&mut self, stage: SelftestStage, value: SelftestStageValue) {
        self.stages[stage.index()] = Some(value);
    }

    /// Read a stage result. Returns `None` if unpopulated.
    pub fn stage(&self, stage: SelftestStage) -> Option<SelftestStageValue> {
        self.stages[stage.index()]
    }

    /// Iterate populated stages in order, yielding `(stage_id, value)` pairs.
    pub fn iter(&self) -> impl Iterator<Item = (u8, SelftestStageValue)> + '_ {
        SelftestStage::ALL
            .into_iter()
            .zip(self.stages)
            .filter_map(|(stage, o)| o.map(|v| (stage.id(), v)))
    }
}

#[cfg(test)]
mod selftest_results_tests {
    use super::*;

    #[test]
    fn stage_wire_ids_match_host_selftest_stage_table() {
        let ids: [(SelftestStage, u8); 8] = [
            (SelftestStage::AdcCalibration, 1),
            (SelftestStage::CoilA, 2),
            (SelftestStage::CoilB, 3),
            (SelftestStage::PhaseWiring, 4),
            (SelftestStage::Encoder, 5),
            (SelftestStage::EncoderDirection, 6),
            (SelftestStage::RModelEvidence, 7),
            (SelftestStage::LModelEvidence, 8),
        ];
        assert_eq!(SelftestStage::ALL.len(), ids.len());
        for ((stage, id), listed) in ids.into_iter().zip(SelftestStage::ALL) {
            assert_eq!(stage.id(), id);
            assert_eq!(stage, listed);
        }
    }

    #[test]
    fn results_default_has_all_stages_unpopulated() {
        let r = SelftestResults::default();
        for stage in SelftestStage::ALL {
            assert!(r.stage(stage).is_none(), "{stage:?} should be None");
        }
    }

    #[test]
    fn set_and_get_round_trips_value_under_its_wire_id() {
        let mut r = SelftestResults::default();
        r.set(
            SelftestStage::CoilB,
            SelftestStageValue {
                status: 0,
                value: 0x1234,
            },
        );
        let got = r.stage(SelftestStage::CoilB).expect("coil B populated");
        assert_eq!(got.status, 0);
        assert_eq!(got.value, 0x1234);
        let emitted: heapless::Vec<(u8, SelftestStageValue), 8> = r.iter().collect();
        assert_eq!(emitted.as_slice(), &[(3, got)]);
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
