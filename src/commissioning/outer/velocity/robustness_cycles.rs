//! Per-direction cycle ledger for the K-cycle reversal-robustness probe.
//!
//! Tracks filled slots, the inconclusive-retry and tail-grace budgets, and the summary
//! statistics the robustness gate consumes.

use super::robustness_metrics::ReversalMetrics;

/// Number of reversal-cycle slots run per direction (K).
pub(crate) const ROBUSTNESS_CYCLES_PER_DIRECTION: usize = 5;

/// Minimum count of filled slots required for a direction's statistics to
/// be trustworthy.
pub(crate) const ROBUSTNESS_MIN_VALID_CYCLES: u8 = 4;

const MAX_INCONCLUSIVE_RETRIES: u8 = super::robustness_metrics::ROBUSTNESS_MAX_RETRIES;

const MICROS_PER_MILLI: u32 = 1_000;

const MICROS_PER_SECOND_I64: i64 = 1_000_000;

/// Outcome of one reversal-robustness probe cycle, as reported by the caller
/// to [`CycleLedger::record`].
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) enum CycleRecord {
    /// The probe cycle reached band entry and yielded metrics.
    Conclusive(ReversalMetrics),
    /// The probe cycle did not reach band entry within its measurement
    /// budget.
    Inconclusive,
}

/// Instruction to the caller for what to do after a [`CycleLedger::record`]
/// call.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub(crate) enum LedgerAction {
    /// Run the next cycle slot.
    NextCycle,
    /// Re-run: inconclusive cycle, budget remained.
    RetryInconclusive,
    /// Re-run: first tail cycle, grace consumed.
    RetryTail,
    /// All slots resolved; read the verdict.
    DirectionDone,
    /// Second conclusive-tail cycle: the tail rejection is latched. The
    /// caller continues the direction (and the other direction) normally
    /// for evidence symmetry; the verdict is applied at assembly.
    RejectTail,
}

/// Per-direction accounting for the K-cycle reversal-robustness probe.
///
/// Holds up to [`ROBUSTNESS_CYCLES_PER_DIRECTION`] filled slots, `ROBUSTNESS_MAX_RETRIES`
/// inconclusive retries, and one tail grace; a second tail cycle latches
/// [`CycleLedger::tail_rejected`].
#[derive(Debug, defmt::Format)]
pub(crate) struct CycleLedger {
    slots: [Option<ReversalMetrics>; ROBUSTNESS_CYCLES_PER_DIRECTION],
    cursor: usize,
    inconclusive_retries: u8,
    tail_grace_used: bool,
    tail_count: u8,
    displaced_tail_reconv_ms: u16,
    tail_rejected: bool,
    iae_max_q: i64,
}

impl CycleLedger {
    /// Start a new, empty ledger for one direction against the probe's
    /// Start an empty ledger against the calibrated `iae_max_q` tail threshold.
    pub(crate) fn new(iae_max_q: i64) -> Self {
        Self {
            slots: [None; ROBUSTNESS_CYCLES_PER_DIRECTION],
            cursor: 0,
            inconclusive_retries: 0,
            tail_grace_used: false,
            tail_count: 0,
            displaced_tail_reconv_ms: 0,
            tail_rejected: false,
            iae_max_q,
        }
    }

    /// Record the outcome of the current cycle slot and return what the
    /// caller should do next.
    ///
    /// # Panics
    ///
    /// Panics (via array indexing) if called again after a prior call
    /// returned [`LedgerAction::DirectionDone`]; callers must stop issuing
    /// cycles once the direction is done.
    pub(crate) fn record(&mut self, record: CycleRecord) -> LedgerAction {
        match record {
            CycleRecord::Inconclusive => self.record_inconclusive(),
            CycleRecord::Conclusive(metrics) => self.record_conclusive(metrics),
        }
    }

    fn record_inconclusive(&mut self) -> LedgerAction {
        if self.inconclusive_retries < MAX_INCONCLUSIVE_RETRIES {
            self.inconclusive_retries += 1;
            return LedgerAction::RetryInconclusive;
        }
        self.advance()
    }

    fn record_conclusive(&mut self, metrics: ReversalMetrics) -> LedgerAction {
        if self.is_tail(&metrics) {
            if !self.tail_grace_used {
                self.tail_grace_used = true;
                self.tail_count += 1;
                self.displaced_tail_reconv_ms = to_ms_saturating(metrics.reconvergence_time_us);
                return LedgerAction::RetryTail;
            }
            self.tail_count += 1;
            self.tail_rejected = true;
            self.slots[self.cursor] = Some(metrics);
            return match self.advance() {
                LedgerAction::DirectionDone => LedgerAction::DirectionDone,
                _ => LedgerAction::RejectTail,
            };
        }
        self.slots[self.cursor] = Some(metrics);
        self.advance()
    }

    fn is_tail(&self, metrics: &ReversalMetrics) -> bool {
        metrics.recovery_iae_q > self.iae_max_q
    }

    fn advance(&mut self) -> LedgerAction {
        self.cursor += 1;
        if self.cursor >= ROBUSTNESS_CYCLES_PER_DIRECTION {
            LedgerAction::DirectionDone
        } else {
            LedgerAction::NextCycle
        }
    }

    /// Median reconvergence time over filled slots, in milliseconds.
    pub(crate) fn median_reconvergence_ms(&self) -> u16 {
        let mut values: [u16; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [0; ROBUSTNESS_CYCLES_PER_DIRECTION];
        let count = self.collect_ms(&mut values, |m| m.reconvergence_time_us);
        median_u16(&mut values[..count])
    }

    /// Maximum reconvergence time over filled slots, in milliseconds,
    /// including a displaced (grace-retried) tail cycle that never occupied
    /// a slot.
    pub(crate) fn max_reconvergence_ms(&self) -> u16 {
        let mut max_ms = if self.tail_grace_used {
            self.displaced_tail_reconv_ms
        } else {
            0
        };
        for slot in self.slots.iter().flatten() {
            let ms = to_ms_saturating(slot.reconvergence_time_us);
            if ms > max_ms {
                max_ms = ms;
            }
        }
        max_ms
    }

    /// Median forward-settle time over filled slots, in milliseconds.
    pub(crate) fn median_forward_settle_ms(&self) -> u16 {
        let mut values: [u16; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [0; ROBUSTNESS_CYCLES_PER_DIRECTION];
        let count = self.collect_ms(&mut values, |m| m.forward_settle_time_us);
        median_u16(&mut values[..count])
    }

    /// Median settled residual rate error over filled slots, in rate-Q
    /// units, saturating into `i32`.
    pub(crate) fn median_settled_residual_q(&self) -> i32 {
        let mut values: [i64; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [0; ROBUSTNESS_CYCLES_PER_DIRECTION];
        let count = self.collect_i64(&mut values, |m| m.settled_residual_q);
        saturating_i64_to_i32(median_i64(&mut values[..count]))
    }

    /// Median recovery IAE over filled slots, converted from the stored
    /// rate-Q microseconds into rate-Q seconds, saturating into `i32`.
    pub(crate) fn median_recovery_iae_qs(&self) -> i32 {
        let mut values: [i64; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [0; ROBUSTNESS_CYCLES_PER_DIRECTION];
        let count = self.collect_i64(&mut values, |m| m.recovery_iae_q / MICROS_PER_SECOND_I64);
        saturating_i64_to_i32(median_i64(&mut values[..count]))
    }

    /// Median settled dAC_rms over filled slots, in raw rate-Q units (no IAE
    /// seconds conversion), saturating into `i32`.
    pub(crate) fn median_dac_rms_q(&self) -> i32 {
        let mut values: [i64; ROBUSTNESS_CYCLES_PER_DIRECTION] =
            [0; ROBUSTNESS_CYCLES_PER_DIRECTION];
        let count = self.collect_i64(&mut values, |m| m.dac_rms_q);
        saturating_i64_to_i32(median_i64(&mut values[..count]))
    }

    /// Count of filled slots.
    pub(crate) fn valid_cycles(&self) -> u8 {
        self.slots.iter().flatten().count() as u8
    }

    /// Count of conclusive cycles classified as tail (reconvergence time
    /// beyond `ROBUSTNESS_TAIL_FACTOR * t_max_us`), including the displaced
    /// grace cycle and any later cycle that latched the rejection.
    pub(crate) fn tail_count(&self) -> u8 {
        self.tail_count
    }

    /// Count of inconclusive retries spent so far (bounded by
    /// `ROBUSTNESS_MAX_RETRIES`).
    pub(crate) fn inconclusive_retries_used(&self) -> u8 {
        self.inconclusive_retries
    }

    /// Whether the direction reached the minimum valid-cycle floor.
    pub(crate) fn meets_floor(&self) -> bool {
        self.valid_cycles() >= ROBUSTNESS_MIN_VALID_CYCLES
    }

    /// Whether a second tail cycle latched the tail rejection.
    pub(crate) fn tail_rejected(&self) -> bool {
        self.tail_rejected
    }

    /// Raw slot contents, in cycle order.
    pub(crate) fn cycles(&self) -> &[Option<ReversalMetrics>; ROBUSTNESS_CYCLES_PER_DIRECTION] {
        &self.slots
    }

    fn collect_ms(
        &self,
        out: &mut [u16; ROBUSTNESS_CYCLES_PER_DIRECTION],
        field: impl Fn(&ReversalMetrics) -> u32,
    ) -> usize {
        let mut count = 0;
        for slot in self.slots.iter().flatten() {
            out[count] = to_ms_saturating(field(slot));
            count += 1;
        }
        count
    }

    fn collect_i64(
        &self,
        out: &mut [i64; ROBUSTNESS_CYCLES_PER_DIRECTION],
        field: impl Fn(&ReversalMetrics) -> i64,
    ) -> usize {
        let mut count = 0;
        for slot in self.slots.iter().flatten() {
            out[count] = field(slot);
            count += 1;
        }
        count
    }
}

fn to_ms_saturating(value_us: u32) -> u16 {
    let ms = value_us.div_ceil(MICROS_PER_MILLI);
    u16::try_from(ms).unwrap_or(u16::MAX)
}

fn saturating_i64_to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

fn median_u16(values: &mut [u16]) -> u16 {
    if values.is_empty() {
        return 0;
    }
    values.sort_unstable();
    let mid = values.len() / 2;
    if values.len() % 2 == 1 {
        values[mid]
    } else {
        let sum = u32::from(values[mid - 1]) + u32::from(values[mid]);
        (sum / 2) as u16
    }
}

fn median_i64(values: &mut [i64]) -> i64 {
    if values.is_empty() {
        return 0;
    }
    values.sort_unstable();
    let mid = values.len() / 2;
    if values.len() % 2 == 1 {
        values[mid]
    } else {
        let sum = i128::from(values[mid - 1]) + i128::from(values[mid]);
        (sum / 2) as i64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metrics(reconv_ms: u32) -> ReversalMetrics {
        metrics_with_iae(reconv_ms, LOW_IAE_Q)
    }

    const NON_TAIL_IAE_MAX_Q: i64 = 50_000_000;

    const LOW_IAE_Q: i64 = 100_000;

    const HIGH_IAE_Q: i64 = 600_000;

    const HIGH_IAE_Q_2: i64 = 550_000;

    fn metrics_with_iae(reconv_ms: u32, recovery_iae_q: i64) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: reconv_ms * 1_000,
            settled_residual_q: 42,
            recovery_iae_q,
            dac_rms_q: 0,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
        }
    }

    const TAIL_IAE_MAX_Q: i64 = 500_000;

    #[test]
    fn tail_classification_is_driven_by_recovery_iae_not_reconvergence_time() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(900, 100_000))),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(50, 600_000))),
            LedgerAction::RetryTail
        );
    }

    #[test]
    fn median_of_five_valid_cycles_is_middle_element() {
        let mut ledger = CycleLedger::new(NON_TAIL_IAE_MAX_Q);
        let order_ms = [100, 300, 200, 250, 150];
        let mut last_action = LedgerAction::NextCycle;
        for ms in order_ms {
            last_action = ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(last_action, LedgerAction::DirectionDone);
        assert_eq!(ledger.median_reconvergence_ms(), 200);
        assert_eq!(ledger.max_reconvergence_ms(), 300);
        assert_eq!(ledger.valid_cycles(), 5);
    }

    #[test]
    fn median_of_four_valid_cycles_rounds_toward_zero() {
        let mut ledger = CycleLedger::new(NON_TAIL_IAE_MAX_Q);
        for ms in [100, 201, 150, 250] {
            ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(ledger.median_reconvergence_ms(), 175);
    }

    #[test]
    fn inconclusive_retries_are_bounded_and_only_inconclusive_draws_them() {
        let mut ledger = CycleLedger::new(NON_TAIL_IAE_MAX_Q);
        for _ in 0..4 {
            assert_eq!(
                ledger.record(CycleRecord::Inconclusive),
                LedgerAction::RetryInconclusive
            );
        }
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        assert_eq!(ledger.inconclusive_retries_used(), 4);
        assert_eq!(ledger.cycles()[0], None);
    }

    #[test]
    fn first_tail_cycle_gets_exactly_one_grace_retry() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(100))),
            LedgerAction::NextCycle
        );
        assert_eq!(ledger.tail_count(), 1);
        assert_eq!(ledger.max_reconvergence_ms(), 700);
        assert_eq!(ledger.cycles()[0], Some(metrics(100)));
    }

    #[test]
    fn tail_replacement_that_is_tail_latches_rejection() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(650, HIGH_IAE_Q_2))),
            LedgerAction::RejectTail
        );
        assert!(ledger.tail_rejected());
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(100))),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(110))),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(120))),
            LedgerAction::NextCycle
        );
        assert_eq!(ledger.valid_cycles(), 4);
    }

    #[test]
    fn second_tail_in_later_slot_latches_rejection() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(100))),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(650, HIGH_IAE_Q_2))),
            LedgerAction::RejectTail
        );
        assert!(ledger.tail_rejected());
    }

    #[test]
    fn second_tail_on_last_slot_signals_direction_done_not_reject() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        for _ in 0..4 {
            assert_eq!(
                ledger.record(CycleRecord::Conclusive(metrics(100))),
                LedgerAction::NextCycle
            );
        }
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(650, HIGH_IAE_Q_2))),
            LedgerAction::DirectionDone
        );
        assert!(ledger.tail_rejected());
        assert_eq!(ledger.valid_cycles(), 5);
    }

    #[test]
    fn inconclusive_budget_cannot_consume_tail_grace() {
        let mut ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
    }

    #[test]
    fn floor_breach_reachable_only_through_inconclusive_budget() {
        let mut ledger = CycleLedger::new(NON_TAIL_IAE_MAX_Q);
        for _ in 0..4 {
            assert_eq!(
                ledger.record(CycleRecord::Inconclusive),
                LedgerAction::RetryInconclusive
            );
        }
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        for ms in [100, 150, 200] {
            ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(ledger.valid_cycles(), 3);
        assert!(!ledger.meets_floor());

        let mut grace_ledger = CycleLedger::new(TAIL_IAE_MAX_Q);
        assert_eq!(
            grace_ledger.record(CycleRecord::Conclusive(metrics_with_iae(700, HIGH_IAE_Q))),
            LedgerAction::RetryTail
        );
        for ms in [100, 150, 200, 250, 300] {
            grace_ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(grace_ledger.valid_cycles(), 5);
        assert!(grace_ledger.meets_floor());
    }

    #[test]
    fn median_dac_rms_q_reduces_filled_slots_with_no_unit_conversion() {
        let mut ledger = CycleLedger::new(i64::MAX);
        let metrics = |dac_rms_q: i64| ReversalMetrics {
            reconvergence_time_us: 10_000,
            settled_residual_q: 0,
            recovery_iae_q: 0,
            dac_rms_q,
            forward_settle_time_us: 5_000,
            reconvergence_ratio_ppm: 0,
        };
        for value in [100_i64, 300, 200] {
            ledger.record(CycleRecord::Conclusive(metrics(value)));
        }
        assert_eq!(ledger.median_dac_rms_q(), 200);
    }

    #[test]
    fn median_dac_rms_q_is_zero_for_an_empty_ledger() {
        let ledger = CycleLedger::new(i64::MAX);
        assert_eq!(ledger.median_dac_rms_q(), 0);
    }
}
