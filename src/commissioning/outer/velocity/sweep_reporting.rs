//! Streamed velocity-sweep evidence sequencing and integrity.

use crate::autotune::velocity_measurement::StationarityResult;
use crate::commissioning::types::{
    VelocitySweepEvent, VelocitySweepObservation, VelocitySweepPlanned, VelocitySweepRungReport,
};

use super::sweep_evidence::{
    band_closure_report_code, observation_class_report_code, rung_class_report_code,
};
use crate::ankyra::velocity_sweep_encoding::{
    encode_rate_values, upper_bound_i64, upper_bound_u128,
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
        self.hash_u8(observation_class_report_code(report.summary.classification));
        self.hash_u8(u8::from(report.summary.initial_lead_in));
        self.hash_u8(u8::from(report.selected_delta_counts < 0));
        let (delta_mantissa, delta_shift) =
            upper_bound_u128(report.selected_delta_counts.unsigned_abs() as u128);
        self.hash_u32(delta_mantissa);
        self.hash_u8(delta_shift);
        let (elapsed_mantissa, elapsed_shift) =
            upper_bound_u128(u128::from(report.selected_elapsed_us));
        self.hash_u32(elapsed_mantissa);
        self.hash_u8(elapsed_shift);
        let rate_values = [
            report.summary.settled_rate_interval_q[0],
            report.settled_rate_mean_q,
            report.summary.settled_rate_interval_q[1],
            report.summary.deficit_interval_q[0],
            report.summary.deficit_interval_q[1],
        ];
        let (rate_values, rate_shift) = encode_rate_values(rate_values);
        for value in rate_values {
            self.hash_i32(value);
        }
        self.hash_u8(rate_shift);
        let suffix_len = match report.stationarity {
            StationarityResult::Usable(suffix) => suffix.suffix_len,
            StationarityResult::Excluded(_) => 0,
        };
        self.hash_u16(suffix_len);
        self.hash_u8(report.selected_level);
        self.hash_u32(report.selected_blocks);
        let (variance_mantissa, variance_shift) = upper_bound_u128(report.variance_of_mean_q2);
        self.hash_u32(variance_mantissa);
        self.hash_u8(variance_shift);
        let (
            slope_mantissa,
            slope_shift,
            slope_half_width_mantissa,
            slope_half_width_shift,
            lag_one_q,
            lag_one_half_width_q,
            residual_mantissa,
            residual_shift,
            tested_suffixes,
        ) = match report.stationarity {
            StationarityResult::Usable(suffix) => {
                let (slope_mantissa, slope_shift) = upper_bound_i64(suffix.slope_q);
                let (slope_half_width_mantissa, slope_half_width_shift) =
                    upper_bound_u128(u128::from(suffix.slope_half_width_q));
                let (residual_mantissa, residual_shift) =
                    upper_bound_u128(suffix.residual_magnitude_q2);
                (
                    slope_mantissa,
                    slope_shift,
                    slope_half_width_mantissa,
                    slope_half_width_shift,
                    suffix.lag_one_q,
                    suffix.lag_one_half_width_q,
                    residual_mantissa,
                    residual_shift,
                    suffix.tested_suffixes,
                )
            }
            StationarityResult::Excluded(_) => (0, 0, 0, 0, 0, 0, 0, 0, 0),
        };
        self.hash_i32(slope_mantissa);
        self.hash_u8(slope_shift);
        self.hash_u32(slope_half_width_mantissa);
        self.hash_u8(slope_half_width_shift);
        self.hash_i32(lag_one_q);
        self.hash_u32(lag_one_half_width_q);
        self.hash_u32(residual_mantissa);
        self.hash_u8(residual_shift);
        self.hash_u8(tested_suffixes);
        self.hash_u16(report.summary.p.raw());
        let disturbance = report.summary.disturbance.map_or(0, |value| value.raw());
        let (disturbance_low_q16, disturbance_high_q16) = report
            .summary
            .disturbance_interval
            .map_or((0, 0), |interval| {
                (interval.low().raw(), interval.high().raw())
            });
        self.hash_i32(disturbance);
        self.hash_i32(disturbance_low_q16);
        self.hash_i32(disturbance_high_q16);
        let (disturbance_variance_mantissa, disturbance_variance_shift) =
            upper_bound_u128(u128::from(report.summary.disturbance_variance_q2));
        self.hash_u32(disturbance_variance_mantissa);
        self.hash_u8(disturbance_variance_shift);
        self.hash_u16(report.summary.predicted_torque_target_abs);
    }

    fn hash_rung(&mut self, report: VelocitySweepRungReport) {
        self.hash_u8(3);
        self.hash_u32(report.run_sequence);
        self.hash_u16(report.evidence_sequence);
        self.hash_u8(report.rung_index);
        self.hash_u16(report.forward.p.raw());
        let interval = |value: Option<crate::autotune::EquivalentTorqueTargetInterval>| {
            value.map_or((0, 0), |item| (item.low().raw(), item.high().raw()))
        };
        for verdict in [report.forward, report.reverse] {
            let (low, high) = interval(verdict.disturbance_interval);
            self.hash_i32(low);
            self.hash_i32(high);
        }
        for band in [report.forward_band, report.reverse_band] {
            self.hash_u16(band.p_low.map_or(0, |value| value.raw()));
            self.hash_u16(band.p_high.map_or(0, |value| value.raw()));
        }
        self.hash_u8(rung_class_report_code(report.forward.classification));
        self.hash_u8(rung_class_report_code(report.reverse.classification));
        self.hash_u8(band_closure_report_code(report.forward_band.closure));
        self.hash_u8(band_closure_report_code(report.reverse_band.closure));
        let flags = u8::from(report.forward.reproduced_moving)
            | (u8::from(report.reverse.reproduced_moving) << 1)
            | (u8::from(report.forward.initial_lead_in) << 2)
            | (u8::from(report.reverse.initial_lead_in) << 3);
        self.hash_u8(flags);
        self.hash_u8(report.forward_band.eligible_rungs);
        self.hash_u8(report.reverse_band.eligible_rungs);
        self.hash_u16(report.forward_band.eligible_observations);
        self.hash_u16(report.reverse_band.eligible_observations);
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
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::autotune::velocity_measurement::{
        StationarityExclusion, StationarityResult, VelocityDirection,
    };
    use crate::autotune::{EquivalentTorqueTargetInterval, EquivalentTorqueTargetQ16};
    use crate::commissioning::outer::velocity::sweep_evidence::{
        VelocityObservationClass, VelocityObservationSummary,
    };
    use foci_tmc4671::pid::PidGain;

    fn fixture_plan() -> VelocitySweepPlanned {
        VelocitySweepPlanned {
            run_sequence: 7,
            evidence_sequence: 0,
            requested_velocity_mrev_s: 5_000,
            requested_velocity_source: 1,
            used_velocity_mrev_s: 4_800,
            effective_ceiling_mrev_s: 7_500,
            clamp_flags: 1,
            effective_ceiling_source: 0,
            target_velocity_register: 3_200,
            p_start: 8,
            p_top: 128,
            rung_count: 5,
            observations_per_direction: 2,
            moving_stroke_us: 256_000,
            zero_settle_us: 200_000,
            nominal_workflow_ms: 9_120,
            maximum_workflow_ms: 10_000,
            max_stroke_travel_mrev: 1_000,
            settle_travel_reserve_mrev: 250,
            negative_position_headroom_mrev: 1_250,
            positive_position_headroom_mrev: 1_250,
            hard_torque_limit: 1_000,
            usable_torque_limit: 900,
        }
    }

    #[test]
    fn plan_digest_matches_host_wire_fixture() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::Planned(fixture_plan()));

        assert_eq!(integrity.digest(), 0x53e2_5af6_0105_c518);
    }

    #[test]
    fn compressed_observation_digest_matches_host_wire_fixture() {
        let mut integrity = VelocitySweepIntegrity::new(7);
        integrity.record(VelocitySweepEvent::Planned(fixture_plan()));
        let disturbance = EquivalentTorqueTargetQ16::from_raw(123_456);
        let disturbance_interval = EquivalentTorqueTargetInterval::new(
            EquivalentTorqueTargetQ16::from_raw(120_000),
            EquivalentTorqueTargetQ16::from_raw(130_000),
        )
        .unwrap();
        integrity.record(VelocitySweepEvent::Observation(VelocitySweepObservation {
            run_sequence: 7,
            evidence_sequence: 1,
            rung_index: 0,
            slot: 0,
            target_velocity_register: 3_200,
            summary: VelocityObservationSummary {
                direction: VelocityDirection::Forward,
                p: PidGain::new(350).unwrap(),
                classification: VelocityObservationClass::Eligible,
                settled_rate_interval_q: [-5_000_000_001, 5_000_000_001],
                deficit_interval_q: [-6_000_000_001, 6_000_000_001],
                disturbance: Some(disturbance),
                disturbance_interval: Some(disturbance_interval),
                disturbance_variance_q2: 1 << 40,
                predicted_torque_target_abs: 500,
                initial_lead_in: true,
            },
            stationarity: StationarityResult::Excluded(StationarityExclusion::NoBlockingPlateau),
            selected_delta_counts: 4_294_967_297,
            selected_elapsed_us: 5_000_000_001,
            selected_level: 3,
            selected_blocks: 16,
            variance_of_mean_q2: 1 << 80,
            settled_rate_mean_q: 1,
        }));

        assert_eq!(integrity.digest(), 0xa4d1_29e7_0765_6385);
    }
}
