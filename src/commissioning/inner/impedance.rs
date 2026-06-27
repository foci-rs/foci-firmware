use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, DSADC_MDEC_B_MDEC_A, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, Register,
    STATUS_FLAGS, UQ_UD_EXT,
};

use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE};
use crate::commissioning::types::CommissioningBackend;
use crate::impedance_identification::{
    IMPEDANCE_PROFILE_POINTS, ImpedanceAngleObservation, ImpedanceBackend, ImpedanceBackendError,
    ImpedanceError, ImpedancePowerStageReadback, ImpedanceReadbackMismatch, ImpedanceRunResult,
    ImpedanceWarningFlags, run_impedance_identification,
};

/// Conservative maximum current-axis split consumed by commissioning v1.
pub(crate) const CURRENT_GAIN_MAX_AXIS_SPLIT_PERMILLE: u16 = 1500;

/// Accepted rotating-carrier impedance evidence used by current-gain synthesis.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CommissioningImpedanceEvidence {
    pub measured_axis_split_permille: u16,
    pub applied_axis_split_permille: u16,
    pub axis_split_clamped: bool,
    pub high_inductance_axis: u8,
    pub selected_axis_max_count_per_ud_milli: u32,
    pub selected_axis_min_count_per_ud_milli: u32,
    pub saliency_permille: u16,
    pub fitted_axis_electrical_counts: u16,
    pub residual_permille: u16,
    pub warning_flags: u32,
}

/// Run commissioning-owned rotating-carrier impedance and map accepted evidence.
pub(crate) async fn run_current_gain_impedance<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<CommissioningImpedanceEvidence, ImpedanceError> {
    let mut impedance_backend = CommissioningImpedanceBackend::new(backend);
    let result = run_impedance_identification(&mut impedance_backend).await;
    impedance_backend.backend.set_enable_pin(false);

    let result = result?;
    current_gain_impedance_evidence(&result, CURRENT_GAIN_MAX_AXIS_SPLIT_PERMILLE)
        .ok_or(ImpedanceError::AxisDeviation)
}

pub(crate) fn current_gain_impedance_evidence(
    result: &ImpedanceRunResult,
    max_split_permille: u16,
) -> Option<CommissioningImpedanceEvidence> {
    if !accepted_for_current_gains(result) {
        return None;
    }

    let high_response = result.summary.axis_max_count_per_ud_milli.max(1);
    let low_response = result.summary.axis_min_count_per_ud_milli.max(1);
    let measured =
        ((high_response as u64 * 1000) / low_response as u64).min(u16::MAX as u64) as u16;
    let applied = measured.min(max_split_permille.max(1000));

    // This commissioning v1 deliberately trusts only the inverse response
    // ratio. The carrier-frequency evidence is in the R << omega*L regime;
    // switch to the full resistance-corrected formula only after hardware
    // validation shows this approximation is not valid for a board/motor class.
    let low_l_axis = result.summary.axis_angle_electrical_counts;
    let dist_to_flux = folded_half_turn_distance(low_l_axis, 0);
    let dist_to_torque = folded_half_turn_distance(low_l_axis, 16_384);
    let high_inductance_axis = if dist_to_torque < dist_to_flux {
        CURRENT_AXIS_FLUX
    } else {
        CURRENT_AXIS_TORQUE
    };

    Some(CommissioningImpedanceEvidence {
        measured_axis_split_permille: measured,
        applied_axis_split_permille: applied,
        axis_split_clamped: applied < measured,
        high_inductance_axis,
        selected_axis_max_count_per_ud_milli: result.summary.axis_max_count_per_ud_milli,
        selected_axis_min_count_per_ud_milli: result.summary.axis_min_count_per_ud_milli,
        saliency_permille: result.summary.saliency_permille,
        fitted_axis_electrical_counts: result.summary.axis_angle_electrical_counts,
        residual_permille: max_residual_permille(result),
        warning_flags: result.warning_flags.0,
    })
}

fn accepted_for_current_gains(result: &ImpedanceRunResult) -> bool {
    if !result.summary.saliency_determinable {
        return false;
    }
    if max_residual_permille(result) > result.profile.residual_fail_permille {
        return false;
    }
    if result.status_flags_or != 0 {
        return false;
    }

    !result.warning_flags.intersects(actionable_warning_flags())
        && !result
            .summary
            .warning_flags
            .intersects(actionable_warning_flags())
}

fn actionable_warning_flags() -> ImpedanceWarningFlags {
    let mut flags = ImpedanceWarningFlags::STATUS_FLAGS;
    flags.insert(ImpedanceWarningFlags::ENCODER_MOVED);
    flags.insert(ImpedanceWarningFlags::ANGLE_LOCAL_MOTION);
    flags.insert(ImpedanceWarningFlags::FILTER_STATE);
    flags.insert(ImpedanceWarningFlags::AXIS_UNDETERMINED);
    flags
}

fn max_residual_permille(result: &ImpedanceRunResult) -> u16 {
    result
        .angle_observations
        .iter()
        .map(|observation| observation.residual_permille)
        .max()
        .unwrap_or(0)
}

fn folded_half_turn_distance(axis: u16, reference: u16) -> u16 {
    let diff = u32::from(axis.abs_diff(reference));
    let turn_distance = diff.min(65_536 - diff);
    turn_distance.min(32_768 - turn_distance) as u16
}

struct CommissioningImpedanceBackend<'a, B: CommissioningBackend> {
    backend: &'a mut B,
    status_flags_or: u32,
    power_stage_readback: ImpedancePowerStageReadback,
    observations: [Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
    readback_mismatch: ImpedanceReadbackMismatch,
    enable_asserted: bool,
}

impl<'a, B: CommissioningBackend> CommissioningImpedanceBackend<'a, B> {
    fn new(backend: &'a mut B) -> Self {
        Self {
            backend,
            status_flags_or: 0,
            power_stage_readback: ImpedancePowerStageReadback::EMPTY,
            observations: [None; IMPEDANCE_PROFILE_POINTS],
            readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
            enable_asserted: false,
        }
    }
}

impl<B: CommissioningBackend> ImpedanceBackend for CommissioningImpedanceBackend<'_, B> {
    async fn read_register(&mut self, addr: u8) -> Result<u32, ImpedanceBackendError> {
        if addr == ABN_DECODER_COUNT.addr() {
            return self
                .backend
                .try_read_encoder_count()
                .await
                .map_err(|_| ImpedanceBackendError::Spi);
        }
        let value = self
            .backend
            .read_register(Register::new(addr))
            .await
            .map_err(|_| ImpedanceBackendError::Spi)?;
        if addr == STATUS_FLAGS.addr() {
            self.status_flags_or |= value;
        } else if addr == PWM_MAXCNT.addr() {
            self.power_stage_readback.pwm_maxcnt_readback = value;
        } else if addr == PWM_BBM_H_BBM_L.addr() {
            self.power_stage_readback.bbm_readback = value;
        } else if addr == DSADC_MDEC_B_MDEC_A.addr() {
            self.power_stage_readback.dsadc_mdec_readback = value;
        } else if addr == PWM_SV_CHOP.addr() {
            self.power_stage_readback.pwm_sv_chop_readback = value;
        }
        Ok(value)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ImpedanceBackendError> {
        if addr == UQ_UD_EXT.addr() && value != 0 && !self.enable_asserted {
            self.backend.set_enable_pin(true);
            self.enable_asserted = true;
        }
        self.backend
            .write_register(Register::new(addr), value)
            .await
            .map_err(|_| ImpedanceBackendError::Spi)
    }

    async fn delay_us(&mut self, us: u32) {
        self.backend.delay_us(us).await;
    }

    async fn delay_ms(&mut self, ms: u64) {
        let mut remaining_ms = ms;
        while remaining_ms > 0 {
            let chunk_ms = remaining_ms.min(1_000);
            self.backend.delay_us((chunk_ms * 1_000) as u32).await;
            remaining_ms -= chunk_ms;
        }
    }

    fn timestamp_us(&self) -> u32 {
        self.backend.elapsed_us() as u32
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        // The commissioning adapter owns enable-pin assertion and defers it
        // until the runner's first nonzero external-voltage write.
        true
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.backend.encoder_counts_per_rev()
    }

    fn record_angle_observation(
        &mut self,
        point_index: usize,
        observation: ImpedanceAngleObservation,
    ) {
        if let Some(slot) = self.observations.get_mut(point_index) {
            *slot = Some(observation);
        }
    }

    fn record_profile_readback_mismatch(&mut self, addr: u8, expected: u32, actual: u32) {
        self.readback_mismatch = ImpedanceReadbackMismatch {
            addr,
            expected,
            actual,
        };
    }
}

#[cfg(test)]
mod tests {
    use crate::autotune::{CURRENT_AXIS_FLUX, CURRENT_AXIS_TORQUE};
    use crate::impedance_identification::{
        IMPEDANCE_PROFILE_POINTS, ImpedanceAngleObservation, ImpedanceFilterState,
        ImpedanceFitSummary, ImpedancePowerStageReadback, ImpedanceProfile,
        ImpedanceReadbackMismatch, ImpedanceRunResult, ImpedanceWarningFlags,
    };

    use super::*;

    #[test]
    fn accepted_impedance_maps_inverse_response_ratio_and_flux_high_l() {
        let result = sample_impedance_result(
            300, 180, 16_384, // low-L/high-response axis is near torque, so high-L is flux
        );
        let evidence = current_gain_impedance_evidence(&result, 1500).unwrap();

        assert_eq!(evidence.measured_axis_split_permille, 1666);
        assert_eq!(evidence.applied_axis_split_permille, 1500);
        assert_eq!(evidence.axis_split_clamped, true);
        assert_eq!(evidence.high_inductance_axis, CURRENT_AXIS_FLUX);
    }

    #[test]
    fn accepted_impedance_maps_torque_high_l_when_low_l_axis_is_flux() {
        let result = sample_impedance_result(
            300, 180, 0, // low-L/high-response axis is near flux, so high-L is torque
        );
        let evidence = current_gain_impedance_evidence(&result, 2000).unwrap();

        assert_eq!(evidence.high_inductance_axis, CURRENT_AXIS_TORQUE);
    }

    #[test]
    fn rejected_impedance_returns_symmetric_evidence() {
        let result = sample_rejected_impedance_result();
        assert!(current_gain_impedance_evidence(&result, 1500).is_none());
    }

    #[test]
    fn nonzero_status_flags_reject_even_when_profile_masks_do_not_match() {
        let mut result = sample_impedance_result(300, 180, 0);
        result.status_flags_or = 1;

        assert!(current_gain_impedance_evidence(&result, 1500).is_none());
    }

    #[test]
    fn residual_failure_rejects_impedance_evidence() {
        let mut result = sample_impedance_result(300, 180, 0);
        result.angle_observations[0].residual_permille =
            result.profile.residual_fail_permille.saturating_add(1);

        assert!(current_gain_impedance_evidence(&result, 1500).is_none());
    }

    #[test]
    fn actionable_warning_rejects_impedance_evidence() {
        let mut result = sample_impedance_result(300, 180, 0);
        result.warning_flags = ImpedanceWarningFlags::STATUS_FLAGS;

        assert!(current_gain_impedance_evidence(&result, 1500).is_none());
    }

    fn sample_impedance_result(
        axis_max_count_per_ud_milli: u32,
        axis_min_count_per_ud_milli: u32,
        axis_angle_electrical_counts: u16,
    ) -> ImpedanceRunResult {
        let mut observations = [ImpedanceAngleObservation::EMPTY; IMPEDANCE_PROFILE_POINTS];
        observations[0] = ImpedanceAngleObservation {
            valid: true,
            residual_permille: 80,
            ..ImpedanceAngleObservation::EMPTY
        };
        ImpedanceRunResult {
            profile: ImpedanceProfile::standard_held_angle_rotating_carrier(),
            angle_observations: observations,
            summary: ImpedanceFitSummary {
                mean_response_count_per_ud_milli: 240,
                axis_max_count_per_ud_milli,
                axis_min_count_per_ud_milli,
                saliency_permille: 500,
                axis_angle_electrical_counts,
                axis_deviation_counts: 0,
                saliency_determinable: true,
                angle_coverage_permille: 1000,
                repeatability_permille: 20,
                warning_flags: ImpedanceWarningFlags::NONE,
            },
            warning_flags: ImpedanceWarningFlags::NONE,
            status_flags_or: 0,
            power_stage_readback: ImpedancePowerStageReadback::EMPTY,
            readback_mismatch: ImpedanceReadbackMismatch::EMPTY,
            filter_state: ImpedanceFilterState {
                torque_enable_before: 0,
                flux_enable_before: 0,
                torque_enable_during: 0,
                flux_enable_during: 0,
            },
        }
    }

    fn sample_rejected_impedance_result() -> ImpedanceRunResult {
        let mut result = sample_impedance_result(300, 180, 0);
        result.summary.saliency_determinable = false;
        result.summary.warning_flags = ImpedanceWarningFlags::AXIS_UNDETERMINED;
        result.warning_flags = ImpedanceWarningFlags::AXIS_UNDETERMINED;
        result
    }
}
