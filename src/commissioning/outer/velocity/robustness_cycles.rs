//! Per-direction cycle ledger for the K-cycle reversal-robustness probe.
//!
//! Pure accounting over a fixed run of [`ROBUSTNESS_CYCLES_PER_DIRECTION`]
//! probe cycles: it tracks which slots are filled, spends the two disjoint
//! per-direction budgets (inconclusive retries and the single tail grace),
//! and reduces the filled slots to the summary statistics the robustness
//! gate consumes. No phase wiring: this module has no knowledge of the
//! commissioning state machine, backend, or wire encoding.

use super::robustness_metrics::ReversalMetrics;

/// Number of reversal-cycle slots run per direction (K).
pub(crate) const ROBUSTNESS_CYCLES_PER_DIRECTION: usize = 5;

/// Minimum count of filled slots required for a direction's statistics to
/// be trustworthy.
pub(crate) const ROBUSTNESS_MIN_VALID_CYCLES: u8 = 4;

/// Multiplier applied to `t_max_us` to derive the tail-cycle threshold: a
/// conclusive cycle whose reconvergence time exceeds
/// `ROBUSTNESS_TAIL_FACTOR * t_max_us` is a tail cycle.
pub(crate) const ROBUSTNESS_TAIL_FACTOR: u32 = 2;

/// Bounded retry count for an inconclusive probe leg, shared with
/// [`super::robustness_metrics::ROBUSTNESS_MAX_RETRIES`]. Duplicated here as
/// a `u8` constant so the ledger's budget arithmetic does not need to reach
/// into a sibling module for a single scalar.
const MAX_INCONCLUSIVE_RETRIES: u8 = super::robustness_metrics::ROBUSTNESS_MAX_RETRIES;

/// Microseconds per millisecond, used for the wire-unit conversions the
/// ledger's summary accessors perform.
const MICROS_PER_MILLI: u32 = 1_000;

/// Microseconds per second, used to convert `recovery_iae_q` (rate-Q
/// microseconds) into rate-Q seconds for [`CycleLedger::median_recovery_iae_qs`].
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
/// Tracks up to [`ROBUSTNESS_CYCLES_PER_DIRECTION`] filled slots plus two
/// disjoint budgets: `ROBUSTNESS_MAX_RETRIES` inconclusive retries (an
/// exhausted budget leaves a slot permanently unfilled, the only path under
/// the [`ROBUSTNESS_MIN_VALID_CYCLES`] floor) and a single per-direction
/// tail grace (one free re-run for the first conclusive cycle whose
/// reconvergence time exceeds `ROBUSTNESS_TAIL_FACTOR * t_max_us`; a second
/// tail cycle latches [`CycleLedger::tail_rejected`] instead of retrying).
#[derive(Debug, defmt::Format)]
pub(crate) struct CycleLedger {
    slots: [Option<ReversalMetrics>; ROBUSTNESS_CYCLES_PER_DIRECTION],
    cursor: usize,
    inconclusive_retries: u8,
    tail_grace_used: bool,
    tail_count: u8,
    /// Reconvergence time, in milliseconds, of the first (grace-retried)
    /// tail cycle. It never occupies a slot, so it is tracked separately to
    /// still feed [`CycleLedger::max_reconvergence_ms`].
    displaced_tail_reconv_ms: u16,
    tail_rejected: bool,
    t_max_us: u32,
}

impl CycleLedger {
    /// Start a new, empty ledger for one direction against the probe's
    /// `t_max_us` time gate.
    pub(crate) fn new(t_max_us: u32) -> Self {
        Self {
            slots: [None; ROBUSTNESS_CYCLES_PER_DIRECTION],
            cursor: 0,
            inconclusive_retries: 0,
            tail_grace_used: false,
            tail_count: 0,
            displaced_tail_reconv_ms: 0,
            tail_rejected: false,
            t_max_us,
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
        // Budget exhausted: the slot stays unfilled and the direction moves on.
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
            self.advance();
            return LedgerAction::RejectTail;
        }
        self.slots[self.cursor] = Some(metrics);
        self.advance()
    }

    fn is_tail(&self, metrics: &ReversalMetrics) -> bool {
        let threshold_us = u64::from(self.t_max_us) * u64::from(ROBUSTNESS_TAIL_FACTOR);
        u64::from(metrics.reconvergence_time_us) > threshold_us
    }

    /// Advance the cursor past the current slot and report whether more
    /// slots remain.
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

    /// Fill `out` with the millisecond conversion of `field` for each filled
    /// slot and return how many were written.
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

    /// Fill `out` with `field` for each filled slot and return how many
    /// were written.
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

/// Convert a microsecond duration to milliseconds, rounding up, saturating
/// into `u16`.
fn to_ms_saturating(value_us: u32) -> u16 {
    let ms = value_us.div_ceil(MICROS_PER_MILLI);
    u16::try_from(ms).unwrap_or(u16::MAX)
}

/// Saturating `i64 -> i32` clamp; never panics on out-of-range magnitudes.
/// Mirrors `robustness_reporting::saturating_i64_to_i32`.
fn saturating_i64_to_i32(value: i64) -> i32 {
    i32::try_from(value).unwrap_or(if value < 0 { i32::MIN } else { i32::MAX })
}

/// Integer median of a `u16` slice, sorted in place: for an odd length, the
/// middle element; for an even length, the mean of the two middle elements
/// rounded toward zero. Returns `0` for an empty slice.
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

/// Integer median of an `i64` slice, sorted in place: for an odd length, the
/// middle element; for an even length, the mean of the two middle elements
/// rounded toward zero (via `i128` to avoid overflow on the sum). Returns
/// `0` for an empty slice.
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

    /// Build a `ReversalMetrics` with only `reconvergence_time_us` varying;
    /// the other fields are fixed so tests can assert on reconvergence
    /// alone.
    fn metrics(reconv_ms: u32) -> ReversalMetrics {
        ReversalMetrics {
            reconvergence_time_us: reconv_ms * 1_000,
            settled_residual_q: 42,
            recovery_iae_q: 9_000_000,
            forward_settle_time_us: 10_000,
            reconvergence_ratio_ppm: 0,
        }
    }

    /// t_max large enough that none of the plain median-test reconvergence
    /// values (<= 300 ms) are ever classified as tail.
    const NON_TAIL_T_MAX_US: u32 = 1_000_000;

    /// t_max used by the tail-focused tests: threshold is
    /// `ROBUSTNESS_TAIL_FACTOR * 300_000 = 600_000` us (600 ms).
    const TAIL_T_MAX_US: u32 = 300_000;

    #[test]
    fn median_of_five_valid_cycles_is_middle_element() {
        let mut ledger = CycleLedger::new(NON_TAIL_T_MAX_US);
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
        let mut ledger = CycleLedger::new(NON_TAIL_T_MAX_US);
        for ms in [100, 201, 150, 250] {
            ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(ledger.median_reconvergence_ms(), 175);
    }

    #[test]
    fn inconclusive_retries_are_bounded_and_only_inconclusive_draws_them() {
        let mut ledger = CycleLedger::new(NON_TAIL_T_MAX_US);
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        assert_eq!(ledger.inconclusive_retries_used(), 2);
        assert_eq!(ledger.cycles()[0], None);
    }

    #[test]
    fn first_tail_cycle_gets_exactly_one_grace_retry() {
        let mut ledger = CycleLedger::new(TAIL_T_MAX_US);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(700))),
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
        let mut ledger = CycleLedger::new(TAIL_T_MAX_US);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(700))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(650))),
            LedgerAction::RejectTail
        );
        assert!(ledger.tail_rejected());
        // Subsequent slots keep filling normally.
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
        let mut ledger = CycleLedger::new(TAIL_T_MAX_US);
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(700))),
            LedgerAction::RetryTail
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(100))),
            LedgerAction::NextCycle
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(650))),
            LedgerAction::RejectTail
        );
        assert!(ledger.tail_rejected());
    }

    #[test]
    fn inconclusive_budget_cannot_consume_tail_grace() {
        let mut ledger = CycleLedger::new(TAIL_T_MAX_US);
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Conclusive(metrics(700))),
            LedgerAction::RetryTail
        );
    }

    #[test]
    fn floor_breach_reachable_only_through_inconclusive_budget() {
        let mut ledger = CycleLedger::new(NON_TAIL_T_MAX_US);
        // Slot 0 spends the entire per-direction retry budget and is still
        // abandoned.
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::RetryInconclusive
        );
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        // Slot 1 has no budget left for even one retry.
        assert_eq!(
            ledger.record(CycleRecord::Inconclusive),
            LedgerAction::NextCycle
        );
        for ms in [100, 150, 200] {
            ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(ledger.valid_cycles(), 3);
        assert!(!ledger.meets_floor());

        let mut grace_ledger = CycleLedger::new(TAIL_T_MAX_US);
        assert_eq!(
            grace_ledger.record(CycleRecord::Conclusive(metrics(700))),
            LedgerAction::RetryTail
        );
        for ms in [100, 150, 200, 250, 300] {
            grace_ledger.record(CycleRecord::Conclusive(metrics(ms)));
        }
        assert_eq!(grace_ledger.valid_cycles(), 5);
        assert!(grace_ledger.meets_floor());
    }
}
