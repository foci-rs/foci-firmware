//! Streamed velocity-sweep evidence sequencing and integrity.

use crate::autotune::velocity_measurement::StationarityResult;
use crate::commissioning::types::{
    VelocitySweepEvent, VelocitySweepObservation, VelocitySweepPlanned, VelocitySweepRungReport,
};

use super::sweep_evidence::{
    band_closure_report_code, direction_report_code, observation_class_report_code,
    rung_class_report_code,
};

const FNV1A64_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const FNV1A64_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Fixed-memory sequencing and digest state for one report-only sweep.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocitySweepIntegrity {
    run_sequence: u32,
    next_evidence_sequence: u16,
    emitted_observations: u16,
    emitted_rungs: u16,
    digest: u64,
}

impl VelocitySweepIntegrity {
    /// Construct integrity state for one caller-assigned run sequence.
    pub(crate) const fn new(run_sequence: u32) -> Self {
        Self {
            run_sequence,
            next_evidence_sequence: 0,
            emitted_observations: 0,
            emitted_rungs: 0,
            digest: FNV1A64_OFFSET,
        }
    }

    /// Return the run sequence copied into every report fragment.
    pub(crate) const fn run_sequence(self) -> u32 {
        self.run_sequence
    }

    /// Return the sequence assigned to the next emitted evidence group.
    pub(crate) const fn next_evidence_sequence(self) -> u16 {
        self.next_evidence_sequence
    }

    /// Fold one event into the digest and advance its evidence sequence.
    pub(crate) fn record(&mut self, event: VelocitySweepEvent) -> VelocitySweepEvent {
        match event {
            VelocitySweepEvent::Planned(report) => self.hash_plan(report),
            VelocitySweepEvent::Observation(report) => {
                self.emitted_observations = self.emitted_observations.saturating_add(1);
                self.hash_observation(report);
            }
            VelocitySweepEvent::Rung(report) => {
                self.emitted_rungs = self.emitted_rungs.saturating_add(1);
                self.hash_rung(report);
            }
        }
        self.next_evidence_sequence = self.next_evidence_sequence.saturating_add(1);
        event
    }

    /// Number of observation groups emitted so far.
    pub(crate) const fn emitted_observations(self) -> u16 {
        self.emitted_observations
    }

    /// Number of rung groups emitted so far.
    pub(crate) const fn emitted_rungs(self) -> u16 {
        self.emitted_rungs
    }

    /// Current exact FNV-1a digest.
    pub(crate) const fn digest(self) -> u64 {
        self.digest
    }

    fn hash_plan(&mut self, report: VelocitySweepPlanned) {
        self.hash_u8(1);
        self.hash_u32(report.run_sequence);
        self.hash_u16(report.evidence_sequence);
        self.hash_u32(report.requested_velocity_mrev_s);
        self.hash_u8(report.requested_velocity_source);
        self.hash_u32(report.used_velocity_mrev_s);
        self.hash_u32(report.effective_ceiling_mrev_s);
        self.hash_u16(report.clamp_flags);
        self.hash_u8(report.effective_ceiling_source);
        self.hash_i32(report.target_velocity_register);
        self.hash_u16(report.p_start);
        self.hash_u16(report.p_top);
        self.hash_u8(report.rung_count);
        self.hash_u16(report.observations_per_direction);
        self.hash_u32(report.moving_stroke_us);
        self.hash_u32(report.zero_settle_us);
        self.hash_u32(report.nominal_workflow_ms);
        self.hash_u32(report.maximum_workflow_ms);
        self.hash_u32(report.max_stroke_travel_mrev);
        self.hash_u32(report.settle_travel_reserve_mrev);
        self.hash_u32(report.negative_position_headroom_mrev);
        self.hash_u32(report.positive_position_headroom_mrev);
        self.hash_u16(report.hard_torque_limit);
        self.hash_u16(report.usable_torque_limit);
    }

    fn hash_observation(&mut self, report: VelocitySweepObservation) {
        self.hash_u8(2);
        self.hash_u32(report.run_sequence);
        self.hash_u16(report.evidence_sequence);
        self.hash_u8(report.rung_index);
        self.hash_u8(report.slot);
        self.hash_i32(report.target_velocity_register);
        self.hash_u8(direction_report_code(report.summary.direction));
        self.hash_u16(report.summary.p.raw());
        self.hash_u8(observation_class_report_code(report.summary.classification));
        for edge in report.summary.settled_rate_interval_q {
            self.hash_i64(edge);
        }
        for edge in report.summary.deficit_interval_q {
            self.hash_i64(edge);
        }
        self.hash_i64(report.selected_delta_counts);
        self.hash_u64(report.selected_elapsed_us);
        self.hash_u8(report.selected_level);
        self.hash_u32(report.selected_blocks);
        self.hash_u128(report.variance_of_mean_q2);
        self.hash_i64(report.settled_rate_mean_q);
        if let StationarityResult::Usable(suffix) = report.stationarity {
            self.hash_u16(suffix.suffix_len);
            self.hash_i64(suffix.slope_q);
            self.hash_u64(suffix.slope_half_width_q);
            self.hash_i32(suffix.lag_one_q);
            self.hash_u32(suffix.lag_one_half_width_q);
            self.hash_u128(suffix.residual_magnitude_q2);
            self.hash_u8(suffix.tested_suffixes);
        }
    }

    fn hash_rung(&mut self, report: VelocitySweepRungReport) {
        self.hash_u8(3);
        self.hash_u32(report.run_sequence);
        self.hash_u16(report.evidence_sequence);
        self.hash_u8(report.rung_index);
        for verdict in [report.forward, report.reverse] {
            self.hash_u8(direction_report_code(verdict.direction));
            self.hash_u16(verdict.p.raw());
            self.hash_u8(rung_class_report_code(verdict.classification));
            self.hash_u8(u8::from(verdict.reproduced_moving));
            self.hash_u8(u8::from(verdict.initial_lead_in));
        }
        for band in [report.forward_band, report.reverse_band] {
            self.hash_u8(band_closure_report_code(band.closure));
            self.hash_u8(band.eligible_rungs);
            self.hash_u16(band.eligible_observations);
            self.hash_u8(u8::from(band.sufficient));
        }
    }

    fn hash_bytes(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.digest ^= u64::from(*byte);
            self.digest = self.digest.wrapping_mul(FNV1A64_PRIME);
        }
    }

    fn hash_u8(&mut self, value: u8) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_u16(&mut self, value: u16) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_u32(&mut self, value: u32) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_i32(&mut self, value: i32) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_u64(&mut self, value: u64) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_i64(&mut self, value: i64) {
        self.hash_bytes(&value.to_le_bytes());
    }

    fn hash_u128(&mut self, value: u128) {
        self.hash_bytes(&value.to_le_bytes());
    }
}
