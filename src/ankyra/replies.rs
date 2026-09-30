//! `#[klipper_reply]` structs shared by every board.
//!
//! Ankyra derives each wire name from the struct identifier, converting PascalCase to
//! `snake_case`. Field declaration order is the wire order; reordering changes the format.

use ankyra::prelude::klipper_reply;

/// `clock` reply — emitted by the `get_clock` handler.
#[klipper_reply]
pub struct Clock {
    /// Low 32 bits of the MCU systick counter.
    pub clock: u32,
}

/// `config` reply — emitted by `get_config` and `finalize_config`.
#[klipper_reply]
pub struct Config {
    pub is_config: bool,
    pub crc: u32,
    pub is_shutdown: bool,
    pub move_count: u16,
}

/// Motion-scale rejection, emitted immediately before shutdown.
#[klipper_reply]
pub struct FociMotionScaleRejected {
    pub oid: u8,
    /// Task stage that detected the rejection (`0` = P2, `1` = P1).
    pub stage: u8,
    /// Cause code from [`crate::motion_scale::MotionScaleRejection`].
    pub reason: u8,
}

/// `endstop_state` reply — emitted by `endstop_home` / `endstop_query_state`.
#[klipper_reply]
pub struct EndstopState {
    pub oid: u8,
    /// Non-zero while the endstop is still arming for a homing move.
    pub homing: u8,
    pub next_clock: u32,
    pub pin_value: u8,
}

/// `analog_in_state` reply — emitted by `query_analog_in`.
#[klipper_reply]
pub struct AnalogInState {
    pub oid: u8,
    pub next_clock: u32,
    pub value: u16,
}

/// `foci_calibrate_result` reply — emitted by the ADC-calibration flow.
#[klipper_reply]
pub struct FociCalibrateResult {
    pub oid: u8,
    /// 0 = success, non-zero = error code.
    pub status: u8,
    pub adc_i0: u16,
    pub adc_i1: u16,
    pub encoder_count: u32,
}

/// Cached ADC_VM mode-5 startup offset.
#[klipper_reply]
pub struct FociAdcVmOffsetResult {
    pub oid: u8,
    /// Averaged raw ADC_VM offset, valid when `status == 0`.
    pub offset_raw: u16,
    pub sample_count: u8,
    /// Status code (0 = cached offset available, 1 = not ready).
    pub status: u8,
}

/// `foci_commission_phase` reply — emitted by the autotune cascade on
/// every phase transition.
#[klipper_reply]
pub struct FociCommissionPhase {
    pub oid: u8,
    pub phase: u8,
    /// 0 = success, non-zero = error code.
    pub status: u8,
}

/// Structured commissioning diagnostic.
#[klipper_reply]
pub struct FociCommissionDetail {
    pub oid: u8,
    pub phase: u8,
    pub code: u8,
    /// Diagnostic status (0 = measurement, non-zero = failure/rejection).
    pub status: u8,
    pub value0: u32,
    pub value1: u32,
    pub value2: u32,
}

/// Timing quality for one electrical method.
#[klipper_reply]
pub struct FociCommissionTiming {
    pub oid: u8,
    pub method: u8,
    pub status: u8,
    pub requested_period_us: u16,
    pub valid_samples: u16,
    pub missed_samples: u16,
    pub max_consecutive_misses: u8,
    pub max_lateness_us: u32,
    pub max_interval_us: u32,
    pub max_poll_wall_us: u32,
    pub max_spi_wall_us: u32,
}

/// `foci_commission_result` reply — emitted at the end of inner
/// commissioning with measured electrical and fallback gain values.
#[klipper_reply]
pub struct FociCommissionResult {
    pub oid: u8,
    /// 0 = success, non-zero = error code.
    pub status: u8,
    pub r_count_milli: u32,
    pub l_count_micro: u32,
    pub flux_p: u16,
    pub flux_i: u16,
    pub torque_p: u16,
    pub torque_i: u16,
    pub lambda_us: u32,
    pub theta_e_us: u32,
    /// Source of the accepted `theta_e_us`: 0 = high-rate measurement, 1 =
    /// board-configured fallback.
    pub theta_source: u8,
    /// Closed-loop bandwidth in Hz, capped at `MAX_PHYSICAL_CURRENT_BANDWIDTH_HZ`.
    pub bandwidth_hz: u16,
    pub fallback_velocity_p: u16,
    pub fallback_velocity_i: u16,
    pub fallback_position_p: u16,
    /// Inner-confidence warning bits: 0, 1, 3 model warnings; 5 fallback current gains;
    /// 6 host-default confidence; 2 and 7 reserved.
    pub inner_warning_flags: u8,
    pub timing_summary: u32,
}

/// Run-level production inductance evidence.
#[klipper_reply]
pub struct FociInductanceRun {
    pub oid: u8,
    pub source: u8,
    pub status: u8,
    /// Warning bitfield.
    pub warning_flags: u16,
    pub ud_count: i16,
    pub realized_frequency_millihz: u32,
    pub elapsed_us: u32,
    pub openloop_phi_delta_counts: u32,
    pub sample_count: u16,
    pub encoder_delta_counts: i32,
    pub status_flags_or: u32,
}

/// Rotating-frame current evidence.
#[klipper_reply]
pub struct FociInductanceFrame {
    pub oid: u8,
    pub id_mean_milli_count: i32,
    pub iq_mean_milli_count: i32,
    pub id_rms_milli_count: u32,
    pub iq_rms_milli_count: u32,
    pub drift_permille: u16,
    pub zero_id_mean_milli_count: i32,
    pub zero_iq_mean_milli_count: i32,
}

/// Accepted count-space inductance evidence.
#[klipper_reply]
pub struct FociInductanceEstimate {
    pub oid: u8,
    pub x_average_count_ratio_milli: u32,
    pub x_d_count_ratio_milli: u32,
    pub x_q_count_ratio_milli: u32,
    /// `AC_SALIENCY_*` status code.
    pub saliency_status: u8,
    pub saliency_permille: u16,
    pub x_mag_nominal_count_ratio_milli: u32,
    pub x_mag_shift_minus_permille: u16,
    pub x_mag_shift_plus_permille: u16,
    pub x_mag_vs_quad_permille: u16,
}

/// Run-level current-loop commissioning evidence.
#[klipper_reply]
pub struct FociCurrentLoopRun {
    pub oid: u8,
    pub status: u8,
    /// Stable `CURRENT_GAINS_SOURCE_*` code.
    pub gains_source: u8,
    /// Candidate `CURRENT_GAINS_SOURCE_*` code before validation verdict.
    pub candidate_gains_source: u8,
    /// Stable `AXIS_SPLIT_SOURCE_*` code.
    pub axis_split_source: u8,
    /// Candidate `AXIS_SPLIT_SOURCE_*` code before validation verdict.
    pub candidate_axis_split_source: u8,
    /// Stable `CURRENT_GAINS_TIER_*` code.
    pub gains_tier: u8,
    /// Candidate `CURRENT_GAINS_TIER_*` code before validation verdict.
    pub candidate_gains_tier: u8,
    /// Measured axis split ratio, permille where 1000 = 1.000x.
    pub measured_axis_split_permille: u16,
    pub candidate_measured_axis_split_permille: u16,
    pub applied_axis_split_permille: u16,
    pub candidate_applied_axis_split_permille: u16,
    /// Non-zero when the applied split was clamped below the measured split.
    pub axis_split_clamped: u8,
    pub candidate_axis_split_clamped: u8,
    pub candidate_flux_p: u16,
    pub candidate_flux_i: u16,
    pub candidate_torque_p: u16,
    pub candidate_torque_i: u16,
    pub candidate_attempt: u8,
    /// Bitmask of `CURRENT_VALIDATION_AXIS_*` axes that passed validation.
    pub current_validation_axes: u8,
    pub flux_validation_sample_count: u8,
    pub torque_validation_sample_count: u8,
    /// Non-zero when the section-level retry budget was exhausted.
    pub retry_budget_exhausted: u8,
    /// Stable `CURRENT_LOOP_FAILURE_*` code.
    pub failure_reason: u8,
}

/// Applied filter cutoffs for the commissioned closed-loop current frame.
#[klipper_reply]
pub struct FociCurrentLoopFilters {
    pub oid: u8,
    /// Applied velocity-filter cutoff in Hz (0 = disabled).
    pub velocity_filter_hz: u16,
    /// Applied torque-filter cutoff in Hz (0 = disabled).
    pub torque_filter_hz: u16,
    /// Applied position-filter cutoff in Hz (0 = disabled).
    pub position_filter_hz: u16,
    /// Applied flux-filter cutoff in Hz (0 = disabled).
    pub flux_filter_hz: u16,
}

/// Fast closed-loop entry stability evidence.
#[klipper_reply]
pub struct FociClosedLoopActivation {
    pub oid: u8,
    /// `CLOSED_LOOP_ACTIVATION_STATUS_*`.
    pub entry_status: u8,
    pub position_1: i32,
    pub position_2: i32,
    pub drift_count: u16,
    pub threshold_count: u16,
    /// Non-zero when the check rejected runaway growth.
    pub runaway: u8,
}

/// Sustained current-loop hold evidence.
#[klipper_reply]
pub struct FociCurrentLoopHold {
    pub oid: u8,
    /// Sustained-hold gate status (`HOLD_GATE_STATUS_*`).
    pub hold_status: u8,
    /// Sustained-hold warning bitfield (`HOLD_GATE_WARNING_*`).
    pub warning_flags: u16,
    pub elapsed_us: u32,
    pub requested_sample_period_us: u16,
    pub sample_count: u8,
    pub position_span_count: u16,
    pub position_drift_count: u16,
    pub torque_mean_count: i16,
    pub torque_rms_count: u16,
    pub torque_peak_to_peak_count: u16,
    pub torque_crossing_count: u8,
    pub flux_mean_count: i16,
    pub flux_rms_count: u16,
    pub flux_peak_to_peak_count: u16,
    pub flux_crossing_count: u8,
    /// OR of all sampled TMC4671 `STATUS_FLAGS`.
    pub status_flags_or: u32,
    /// Count of samples with actionable status bits.
    pub actionable_status_count: u8,
}

/// Per-axis, per-delay current validation evidence.
#[klipper_reply]
pub struct FociCurrentValidationAxis {
    pub oid: u8,
    /// Stable `CURRENT_AXIS_*` axis identifier.
    pub axis: u8,
    pub sample_index: u8,
    pub status: u8,
    pub attempt: u8,
    pub target: i16,
    pub sample_delay_ms: u16,
    pub positive_response_permille: u16,
    pub negative_response_permille: u16,
    pub cross_axis_permille: u16,
    pub cross_axis_peak_permille: u16,
    pub voltage_output_permille: u16,
    pub encoder_delta_counts: u16,
    pub positive_encoder_delta_counts: i32,
    pub negative_encoder_delta_counts: i32,
    /// TMC4671 `STATUS_FLAGS` accumulated across the sample.
    pub status_flags_or: u32,
}

/// `foci_dump_done` reply — emitted at the end of a register dump.
#[klipper_reply]
pub struct FociDumpDone {
    pub oid: u8,
}

/// `foci_dump_value` reply — emitted once per register during a dump.
#[klipper_reply]
pub struct FociDumpValue {
    pub oid: u8,
    pub addr: u8,
    pub value: u32,
}

/// MSP stack high-water evidence.
#[klipper_reply]
pub struct FociStackWatermarkResult {
    pub oid: u8,
    pub stack_unused_bytes: u32,
    pub painted_bytes: u32,
    /// Status code (0 = measured, 1 = never painted, 2 = lower bound,
    /// 3 = refused because motion was active).
    pub status: u8,
}

/// Recovered previous-boot fault context.
#[klipper_reply]
pub struct FociLastBootDiagnostic {
    pub oid: u8,
    /// Fault origin: 0 = none, 1 = Rust panic, 2 = HardFault.
    pub fault_kind: u8,
    /// Source line of a captured Rust panic, or 0.
    pub line: u32,
    /// Faulting program counter of a captured HardFault, or 0.
    pub pc: u32,
    pub file_hash: u32,
    /// Last validated [`crate::diagnostic::BreakawayCheckpoint`] code, or 0.
    pub breakaway_checkpoint: u8,
    /// Nonzero when reset interrupted an active breakaway campaign.
    pub breakaway_interrupted: u8,
}

/// Best-effort detail before fatal shutdown.
#[klipper_reply]
pub struct FociDiagnostic {
    pub fault: u8,
    pub operation: u8,
    /// Stepper or FOCI OID when available, otherwise 255.
    pub oid: u8,
    /// Board-local channel when available, otherwise 255.
    pub channel: u8,
    pub data0: u32,
    pub data1: u32,
}

/// Bounded current-step diagnostic result.
#[klipper_reply]
pub struct FociCurrentStepResult {
    pub oid: u8,
    pub status: u8,
    /// Driven axis: `CurrentStepAxis::as_u8()` of the axis this step targeted.
    pub axis: u8,
    pub target: i16,
    pub torque_before: i16,
    pub torque_during: i16,
    pub torque_after: i16,
    pub flux_during: i16,
    pub iq_during: i16,
    pub id_during: i16,
    pub uq_limited: i16,
    pub ud_limited: i16,
    pub encoder_before: u32,
    pub encoder_after: u32,
    pub adc_vm_raw: u16,
}

/// Bounded current-vector diagnostic.
#[klipper_reply]
pub struct FociCurrentVectorStepResult {
    pub oid: u8,
    pub status: u8,
    pub torque_target: i16,
    pub flux_target: i16,
    pub torque_before: i16,
    pub torque_during: i16,
    pub torque_after: i16,
    pub flux_during: i16,
    pub iq_during: i16,
    pub id_during: i16,
    pub uq_limited: i16,
    pub ud_limited: i16,
    pub encoder_before: u32,
    pub encoder_after: u32,
    pub adc_vm_raw: u16,
}

/// Non-terminal PID monitor fragment.
#[klipper_reply]
pub struct FociCurrentTorqueSamplePidResult {
    pub oid: u8,
    pub report_seq: u8,
    pub pidin_target_torque: i32,
    pub pidin_target_flux: i32,
    pub pidout_target_torque: i32,
    pub pidout_target_flux: i32,
    pub pid_torque_target_monitor: i16,
    pub torque_error: i32,
    pub flux_error: i32,
    pub torque_error_sum: i32,
    pub flux_error_sum: i32,
}

/// Torque-pulse sample terminal result.
#[klipper_reply]
pub struct FociCurrentTorqueSampleResult {
    pub oid: u8,
    pub report_seq: u8,
    /// Which torque-sample diagnostic produced this report: `TorqueSampleKind::as_u8()`.
    pub kind: u8,
    pub status: u8,
    pub target: i16,
    pub flux_target: i16,
    pub sample_delay_ms: u16,
    /// Temporary PIDOUT_UQ_UD_LIMITS used during the diagnostic.
    pub voltage_limit: u16,
    pub torque_before: i16,
    pub torque_sample: i16,
    pub torque_after: i16,
    pub flux_sample: i16,
    pub iq_sample: i16,
    pub id_sample: i16,
    pub uq_limited: i16,
    pub ud_limited: i16,
    pub adc_vm_raw: u16,
    pub status_flags: u32,
}

/// Non-terminal encoder/prelimit/feedforward fragment.
#[klipper_reply]
pub struct FociCurrentTorqueSampleDetailResult {
    pub oid: u8,
    pub report_seq: u8,
    pub encoder_before: u32,
    pub encoder_sample: u32,
    pub encoder_after: u32,
    pub encoder_delta_sample: i32,
    pub encoder_delta_after: i32,
    pub uq_prelimit: i16,
    pub ud_prelimit: i16,
    pub ff_velocity: i32,
    pub ff_torque: i16,
}

/// Bounded open-loop voltage diagnostic terminal.
#[klipper_reply]
pub struct FociVoltageStepResult {
    pub oid: u8,
    pub report_seq: u8,
    pub status: u8,
    /// Requested signed q-axis voltage component for UQ_UD_EXT.
    pub uq_ext: i16,
    /// Requested signed d-axis voltage component for UQ_UD_EXT.
    pub ud_ext: i16,
    pub sample_delay_ms: u16,
    pub torque_before: i16,
    pub torque_sample: i16,
    pub torque_after: i16,
    pub flux_sample: i16,
    pub iq_sample: i16,
    pub id_sample: i16,
    pub uq_limited: i16,
    pub ud_limited: i16,
    pub adc_vm_raw: u16,
    pub status_flags: u32,
}

/// Non-terminal encoder fragment.
#[klipper_reply]
pub struct FociVoltageStepDetailResult {
    pub oid: u8,
    pub report_seq: u8,
    pub encoder_before: u32,
    pub encoder_sample: u32,
    pub encoder_after: u32,
}

/// Board/profile constants for display.
#[klipper_reply]
pub struct FociResistanceProfile {
    pub oid: u8,
    /// PWM_MAXCNT register value used during the sweep.
    pub pwm_maxcnt: u16,
    pub bbm_h: u8,
    pub bbm_l: u8,
    /// DSADC_MDEC_A decimation/filter setting.
    pub dsadc_mdec_a: u16,
    /// DSADC_MDEC_B decimation/filter setting.
    pub dsadc_mdec_b: u16,
    pub linear_current_threshold_count: u16,
    pub encoder_move_warn_counts: u16,
    /// STATUS_FLAGS bitmask that raises the warning flag during a window.
    pub status_flags_warn_mask: u32,
    /// Non-zero when the board's scale metadata has been validated.
    pub scale_metadata_validated: u8,
}

/// Per-run status, warnings, and readbacks.
#[klipper_reply]
pub struct FociResistanceRun {
    pub oid: u8,
    pub status: u8,
    pub selected_r_count_slope_milli: i32,
    pub gain_path_count_slope_milli: i32,
    /// Run-level warning bitfield.
    pub warning_flags: u32,
    /// STATUS_FLAGS accumulated (OR) across the run.
    pub status_flags_or: u32,
    pub peak_abs_current_count: u16,
    pub max_abs_steady_mean_current_count: u16,
    pub current_ceiling_count: u16,
    /// Non-zero when sample-level containment synchronously tripped the power stage.
    pub power_stage_tripped: u8,
    /// PWM_MAXCNT readback captured during the run.
    pub pwm_maxcnt_readback: u16,
    /// Packed PWM_BBM_H_BBM_L readback captured during the run.
    pub bbm_readback: u16,
    /// Packed DSADC_MDEC_B_MDEC_A readback captured during the run.
    pub dsadc_mdec_readback: u32,
    /// PWM_SV_CHOP readback captured during the run.
    pub pwm_sv_chop_readback: u32,
}

/// Per-electrical-axis fit result.
#[klipper_reply]
pub struct FociResistanceAxis {
    pub oid: u8,
    pub electrical_axis: u8,
    pub phi_e_ext: u16,
    pub r_count_slope_milli: i32,
    pub intercept_count: i32,
    pub rmse_permille: u16,
    /// Bitmask of points selected for the production linear-region fit.
    pub selected_mask: u16,
    /// Bitmask of points excluded from the fit.
    pub excluded_point_mask: u16,
    pub selected_count: u8,
    pub signed_count_slope_milli: i32,
    pub signed_asymmetry_permille: u16,
    pub drift_permille: u16,
    /// Per-axis warning bitfield.
    pub warning_flags: u32,
}

#[cfg(test)]
mod resistance_reply_size_tests {
    use super::{FociResistanceRun, FociRobustnessCycleEvidence};
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    const KLIPPER_MESSAGE_PAYLOAD_MAX: usize = 59;

    const REPLY_ID_WORST_CASE_BYTES: usize = 3;

    fn encoded_len<T: Writable>(reply: &T) -> usize {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        scratch.result().len()
    }

    fn assert_fits_with_id<T: Writable>(reply: &T, reply_kind: &str) {
        let payload_len = encoded_len(reply);
        let total_len = payload_len + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            total_len <= KLIPPER_MESSAGE_PAYLOAD_MAX,
            "{reply_kind} encodes to {payload_len} payload bytes + {REPLY_ID_WORST_CASE_BYTES} \
             worst-case id bytes = {total_len}, exceeds {KLIPPER_MESSAGE_PAYLOAD_MAX}"
        );
    }

    #[test]
    fn resistance_run_current_evidence_has_stable_wire_order() {
        let reply = FociResistanceRun {
            oid: 1,
            status: 2,
            selected_r_count_slope_milli: 3,
            gain_path_count_slope_milli: 4,
            warning_flags: 5,
            status_flags_or: 6,
            peak_abs_current_count: 7,
            max_abs_steady_mean_current_count: 8,
            current_ceiling_count: 9,
            power_stage_tripped: 10,
            pwm_maxcnt_readback: 11,
            bbm_readback: 12,
            dsadc_mdec_readback: 13,
            pwm_sv_chop_readback: 14,
        };
        let mut scratch = ScratchOutput::<64>::new();

        reply.write(&mut scratch);

        assert_eq!(
            scratch.result(),
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        );
    }

    #[test]
    fn robustness_cycle_evidence_reply_fits_klipper_message_limit() {
        let payload = [u8::MAX;
            crate::commissioning::outer::velocity::robustness_reporting::
                ROBUSTNESS_CYCLE_EVIDENCE_REPLY_BYTES];
        let reply = FociRobustnessCycleEvidence {
            oid: u8::MAX,
            payload: &payload,
        };
        assert_fits_with_id(&reply, "foci_robustness_cycle_evidence");
    }
}

#[cfg(test)]
mod breakaway_reply_wire_tests {
    use super::{
        FociBreakawayCampaignTerminal, FociBreakawayConfirmationPlan,
        FociBreakawayConfirmationTerminalIdentity, FociBreakawayConfirmationTerminalMasks,
        FociBreakawayDiscoveryPlanGeometry, FociBreakawayDiscoveryPlanIdentity,
        FociBreakawayDiscoveryTerminal, FociBreakawayProbePlan, FociBreakawayProbeResult,
        FociBreakawayProbeTerminal, FociBreakawayRungZeroDiagnostic,
    };
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    fn assert_wire_bytes<T: Writable>(reply: &T, expected: &[u8]) {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        assert_eq!(scratch.result(), expected);
    }

    #[test]
    fn breakaway_probe_plan_has_stable_wire_order() {
        let reply = FociBreakawayProbePlan {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            max_observations: 6,
            search_count: 7,
            p_start_raw: 8,
            p_top_raw: 9,
            motion_threshold_counts: 10,
            max_capture_interval_us: 11,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]);
    }

    #[test]
    fn breakaway_probe_result_has_stable_wire_order() {
        let reply = FociBreakawayProbeResult {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            rung_index: 6,
            breakaway_p_raw: 7,
            motion_threshold_counts: 8,
            observation_count: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }

    #[test]
    fn breakaway_probe_terminal_has_stable_wire_order() {
        let reply = FociBreakawayProbeTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            terminal_cause: 6,
            has_safety_fault: 7,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn breakaway_discovery_plan_identity_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryPlanIdentity {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8]);
    }

    #[test]
    fn breakaway_discovery_plan_geometry_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryPlanGeometry {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            schema_revision: 4,
            rung_count: 5,
            observations_per_direction: 6,
            floor_p_raw: 7,
            floor_origin: 8,
            breakaway_p_raw: 9,
            ceiling_p_raw: 10,
            first_additive_step_raw: 11,
            maximum_workflow_ms: 12,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
    }

    #[test]
    fn breakaway_rung_zero_diagnostic_has_stable_wire_order() {
        let reply = FociBreakawayRungZeroDiagnostic {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            forward_moved: 6,
            reverse_moved: 7,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7]);
    }

    #[test]
    fn breakaway_discovery_terminal_has_stable_wire_order() {
        let reply = FociBreakawayDiscoveryTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            terminal_cause: 9,
            collected_count: 10,
            has_safety_fault: 11,
            stop_rung_index: 12,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12]);
    }

    #[test]
    fn breakaway_confirmation_plan_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationPlan {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            observations_per_direction: 9,
            nominated_p_raw: 10,
            band_lower_percent: 11,
            band_upper_percent: 12,
            capture_profile: 13,
            acceptance_rule: 14,
            nominated_margin_percent_milli: 15,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15]);
    }

    #[test]
    fn breakaway_confirmation_terminal_identity_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationTerminalIdentity {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            plan_digest_low: 4,
            plan_digest_high: 5,
            prior_plan_digest_low: 6,
            prior_plan_digest_high: 7,
            family_size: 8,
            terminal_cause: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }

    #[test]
    fn breakaway_confirmation_terminal_masks_has_stable_wire_order() {
        let reply = FociBreakawayConfirmationTerminalMasks {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            forward_collected_mask: 4,
            forward_eligible_mask: 5,
            forward_included_mask: 6,
            reverse_collected_mask: 7,
            reverse_eligible_mask: 8,
            reverse_included_mask: 9,
            accepted: 10,
            confirmed_p_raw: 11,
            max_relative_se_permille: 12,
            required_relative_se_permille: 13,
            has_safety_fault: 14,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14]);
    }

    #[test]
    fn breakaway_campaign_terminal_has_stable_wire_order() {
        let reply = FociBreakawayCampaignTerminal {
            oid: 1,
            run_sequence: 2,
            evidence_sequence: 3,
            phase: 4,
            terminal_cause: 5,
            accepted: 6,
            integral_plan_digest_low: 7,
            integral_plan_digest_high: 8,
            error_code: 9,
        };
        assert_wire_bytes(&reply, &[1, 2, 3, 4, 5, 6, 7, 8, 9]);
    }
}

/// Bounded stepper runtime diagnostic.
#[klipper_reply]
pub struct FociStepperEvent {
    pub reason: u8,
    pub channel: u8,
    pub position: i32,
    pub clock: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: u8,
    /// Board-local queue length, or 65535 when unknown.
    pub queue_len: u16,
    pub direction: u8,
    pub data0: u32,
    pub data1: u32,
}

/// Step queue/execution counters.
#[klipper_reply]
pub struct FociStepperStatsResult {
    pub oid: u8,
    pub channel: u8,
    pub position: i32,
    pub queued_segments: u32,
    pub queued_steps: u32,
    pub loaded_segments: u32,
    pub loaded_steps: u32,
    pub discarded_segments: u32,
    pub discarded_steps: u32,
    /// Non-zero when the board believes the step timer is active.
    pub timer_active: u8,
    pub queue_len: u16,
    pub oversize_frame_drops: u32,
}

/// Step execution counters.
#[klipper_reply]
pub struct FociStepperExecStatsResult {
    pub oid: u8,
    pub executed_pos_steps: u32,
    pub executed_neg_steps: u32,
    pub physical_pos_pulses: u32,
    pub physical_neg_pulses: u32,
    /// Configured logical planner steps per mechanical revolution, or zero when unconfigured.
    pub planner_steps_per_rev: u32,
    /// Configured encoder pulses per mechanical revolution, or zero when unconfigured.
    pub encoder_ppr: u32,
    pub queue_empty_count: u32,
    pub missed_deadline_count: u32,
}

/// First-step timing counters.
#[klipper_reply]
pub struct FociStepperTimingStatsResult {
    pub oid: u8,
    pub channel: u8,
    pub activation_count: u32,
    /// TIM5 clock when the step timer was most recently armed.
    pub last_activation_clock: u32,
    /// TIM5 clock when the first segment of the last activation loaded.
    pub first_load_now: u32,
    pub first_load_scheduled: u32,
    pub first_load_compare: u32,
    pub first_load_lead_ticks: i32,
    pub first_load_compare_delay_ticks: i32,
    pub first_step_clock: u32,
    pub first_step_delay_ticks: i32,
}

/// Physical stop/reset counters.
#[klipper_reply]
pub struct FociStepperStopStatsResult {
    pub oid: u8,
    pub channel: u8,
    pub stop_count: u32,
    pub stop_drained_segments: u32,
    pub stop_drained_steps: u32,
    pub reset_count: u32,
    pub reset_drained_segments: u32,
    pub reset_drained_steps: u32,
    pub last_stop_reason: u8,
    pub last_stop_remaining_events: u32,
    pub last_stop_queue_len: u16,
    pub last_stop_drained_segments: u32,
    pub last_stop_drained_steps: u32,
}

/// `foci_selftest_done` reply — emitted at the end of a self-test run.
#[klipper_reply]
pub struct FociSelftestDone {
    pub oid: u8,
    /// 0 = success, non-zero = error code.
    pub status: u8,
}

/// `foci_selftest_result` reply — emitted once per self-test stage
/// carrying the per-stage measurement.
#[klipper_reply]
pub struct FociSelftestResult {
    pub oid: u8,
    pub stage: u8,
    /// Status code for this stage (0 = success, non-zero = error code).
    pub status: u8,
    pub value: u32,
}

/// Breakaway-probe plan evidence.
#[klipper_reply]
pub struct FociBreakawayProbePlan {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub max_observations: u8,
    pub search_count: u8,
    pub p_start_raw: u16,
    pub p_top_raw: u16,
    pub motion_threshold_counts: u32,
    pub max_capture_interval_us: u32,
}

/// Resolved repeatable-motion rung for the upward probe.
#[klipper_reply]
pub struct FociBreakawayProbeResult {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub rung_index: u8,
    pub breakaway_p_raw: u16,
    pub motion_threshold_counts: u32,
    pub observation_count: u8,
}

/// Detailed terminal evidence for a probe campaign that did not resolve a breakaway.
#[klipper_reply]
pub struct FociBreakawayProbeTerminal {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
}

/// Additive discovery plan identity: digests and confidence family.
#[klipper_reply]
pub struct FociBreakawayDiscoveryPlanIdentity {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub prior_plan_digest_low: u32,
    pub prior_plan_digest_high: u32,
    pub family_size: u16,
}

/// Additive discovery plan geometry: floor, breakaway, ceiling, and step.
#[klipper_reply]
pub struct FociBreakawayDiscoveryPlanGeometry {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub schema_revision: u16,
    pub rung_count: u8,
    pub observations_per_direction: u16,
    pub floor_p_raw: u16,
    pub floor_origin: u8,
    pub breakaway_p_raw: u16,
    pub ceiling_p_raw: u16,
    pub first_additive_step_raw: u32,
    /// Reserved maximum campaign workflow duration.
    pub maximum_workflow_ms: u32,
}

/// Diagnostic evidence for whether the additive ladder's rung zero itself showed motion.
#[klipper_reply]
pub struct FociBreakawayRungZeroDiagnostic {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    /// Nonzero when the forward-direction rung-zero observation showed motion.
    pub forward_moved: u8,
    /// Nonzero when the reverse-direction rung-zero observation showed motion.
    pub reverse_moved: u8,
}

/// Detailed terminal evidence for an additive discovery campaign.
#[klipper_reply]
pub struct FociBreakawayDiscoveryTerminal {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub prior_plan_digest_low: u32,
    pub prior_plan_digest_high: u32,
    pub family_size: u16,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    pub collected_count: u8,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
    /// Rung index the discovery walk had reached when it stopped, whether by
    /// nominating, exhausting the ladder, or a discovery-owned fault.
    pub stop_rung_index: u8,
}

/// Held-out confirmation plan evidence.
#[klipper_reply]
pub struct FociBreakawayConfirmationPlan {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub prior_plan_digest_low: u32,
    pub prior_plan_digest_high: u32,
    pub family_size: u16,
    pub observations_per_direction: u16,
    pub nominated_p_raw: u16,
    pub band_lower_percent: u8,
    pub band_upper_percent: u8,
    /// Stable wire code for the confirmation capture profile.
    pub capture_profile: u8,
    /// Stable wire code for the acceptance rule.
    pub acceptance_rule: u8,
    pub nominated_margin_percent_milli: i32,
}

/// Confirmation terminal identity: digests and confidence family.
#[klipper_reply]
pub struct FociBreakawayConfirmationTerminalIdentity {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub prior_plan_digest_low: u32,
    pub prior_plan_digest_high: u32,
    pub family_size: u16,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
}

/// Confirmation terminal masks: collected/eligible/included per direction,
/// acceptance verdict, and measured/required standard error.
#[klipper_reply]
pub struct FociBreakawayConfirmationTerminalMasks {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    /// Forward-direction collected mask.
    pub forward_collected_mask: u8,
    /// Forward-direction eligible mask.
    pub forward_eligible_mask: u8,
    /// Forward-direction included mask.
    pub forward_included_mask: u8,
    /// Reverse-direction collected mask.
    pub reverse_collected_mask: u8,
    /// Reverse-direction eligible mask.
    pub reverse_eligible_mask: u8,
    /// Reverse-direction included mask.
    pub reverse_included_mask: u8,
    /// Nonzero when the held-out block was accepted.
    pub accepted: u8,
    /// Confirmed proportional gain, or zero when not accepted.
    pub confirmed_p_raw: u16,
    pub max_relative_se_permille: u16,
    pub required_relative_se_permille: u16,
    /// Nonzero when the terminal carries retained safety-fault evidence.
    pub has_safety_fault: u8,
}

/// Canonical closure record for one complete breakaway campaign.
#[klipper_reply]
pub struct FociBreakawayCampaignTerminal {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub phase: u8,
    /// Unified breakaway-campaign terminal-cause wire code.
    pub terminal_cause: u8,
    /// Nonzero when the confirmed candidate was accepted and admitted to Integral.
    pub accepted: u8,
    pub integral_plan_digest_low: u32,
    pub integral_plan_digest_high: u32,
    /// `CommissionError::status_code()` for a campaign terminal that carries a
    /// causally-relevant error, or `0` when the terminal carries no error.
    pub error_code: u8,
}

/// Which `p_top` candidate bound the additive-discovery authority ceiling.
#[klipper_reply]
pub struct FociBreakawayCeilingSource {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    /// `sweep_plan::CEILING_BINDING_CURRENT_LIMIT` or
    /// `sweep_plan::CEILING_BINDING_REPRESENTABILITY_CLAMP`.
    pub binding_source: u8,
}

/// Command-level commissioning workflow disclosed before any motion.
#[klipper_reply]
pub struct FociCommissioningWorkflowPlan {
    pub oid: u8,
    pub run_sequence: u32,
    pub shape: u8,
    pub digest_low: u32,
    pub digest_high: u32,
    pub nominal_workflow_ms: u32,
    pub maximum_workflow_ms: u32,
}

/// Integral plan identity and reproduced proportional authority.
#[klipper_reply]
pub struct FociVelocityIntegralPlanCore {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub fragment: u8,
    pub plan_digest_low: u32,
    pub plan_digest_high: u32,
    pub proportional_digest_low: u32,
    pub proportional_digest_high: u32,
    pub build_features: u16,
    pub build_identity: u32,
    pub schema_revision: u16,
    pub channel: u8,
    pub fixed_p: u16,
}

/// Integral plan geometry and controller-domain limits.
#[klipper_reply]
pub struct FociVelocityIntegralPlanGeometry {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub fragment: u8,
    pub planned_velocity_mrev_s: u32,
    pub target_velocity_rpm: i32,
    pub pwm_hz: u32,
    pub encoder_counts_per_rev: u32,
    pub i_start: u16,
    pub positive_rung_count: u8,
    pub family_size: u16,
    pub total_rung_count: u8,
    pub expected_observations: u16,
    pub hard_torque_limit: u16,
    pub usable_torque_limit: u16,
}

/// One direction of Proportional evidence authorizing Integral.
#[klipper_reply]
pub struct FociVelocityIntegralPlanAuthority {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub direction: u8,
    pub directional_membership: u32,
    pub joint_membership: u32,
    pub directional_validity: u8,
    pub reduced_margin: u8,
    pub pooled_low_q16: i32,
    pub pooled_high_q16: i32,
}

/// Integral fixed schedule and travel identity.
#[klipper_reply]
pub struct FociVelocityIntegralPlanTiming {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub fragment: u8,
    pub moving_stroke_us: u32,
    pub zero_settle_us: u32,
    pub analysis_budget_us: u32,
    pub nominal_workflow_ms: u32,
    pub maximum_workflow_ms: u32,
}

/// Integral travel and absolute-position identity.
#[klipper_reply]
pub struct FociVelocityIntegralPlanTravel {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub fragment: u8,
    pub max_stroke_travel_mrev: u32,
    /// Reserved settle/braking travel.
    pub settle_travel_reserve_mrev: u32,
    pub negative_position_headroom_mrev: u32,
    pub positive_position_headroom_mrev: u32,
}

/// Integral rung-origin recovery geometry and reserved schedule.
#[klipper_reply]
pub struct FociVelocityIntegralPlanRecovery {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub origin_band_counts: u32,
    pub nominal_slot_us: u32,
    pub maximum_slot_us: u32,
    /// Maximum recovery-record count reserved by the plan.
    pub slot_count: u8,
    pub flags: u8,
}

/// One exact positive-I plan rung.
#[klipper_reply]
pub struct FociVelocityIntegralPlanRung {
    pub oid: u8,
    pub run_sequence: u32,
    pub evidence_sequence: u16,
    pub rung_index: u8,
    pub i_raw: u16,
    pub tau_us: u32,
}

/// Compact reversal-standstill robustness-gate terminal.
#[klipper_reply]
pub struct FociRobustnessReversalTerminal<'a> {
    pub oid: u8,
    pub payload: &'a [u8],
}

/// Compact single-reply Integral velocity-integral terminal.
#[klipper_reply]
pub struct FociVelocityIntegralTerminal<'a> {
    pub oid: u8,
    pub payload: &'a [u8],
}

/// Compact per-cycle reversal-standstill robustness evidence, one per direction.
#[klipper_reply]
pub struct FociRobustnessCycleEvidence<'a> {
    pub oid: u8,
    pub payload: &'a [u8],
}

#[cfg(test)]
mod velocity_sweep_reply_tests {
    use super::*;
    use ankyra::ScratchOutput;
    use ankyra::encoding::Writable;

    const PAYLOAD_WITH_ID_MAX: usize = 59;
    const FRAME_MAX: usize = 64;
    const REPLY_ID_WORST_CASE_BYTES: usize = 3;
    const FRAME_OVERHEAD_BYTES: usize = 5;

    fn assert_fits<T: Writable>(reply: &T, name: &str) {
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let payload = scratch.result().len() + REPLY_ID_WORST_CASE_BYTES;
        assert!(
            payload <= PAYLOAD_WITH_ID_MAX,
            "{name} payload is {payload}"
        );
        assert!(
            payload + FRAME_OVERHEAD_BYTES <= FRAME_MAX,
            "{name} frame is {}",
            payload + FRAME_OVERHEAD_BYTES
        );
    }

    #[test]
    fn velocity_integral_terminal_fits_one_protocol_frame() {
        assert_fits(
            &FociVelocityIntegralTerminal {
                oid: u8::MAX,
                payload: &[u8::MAX;
                    crate::commissioning::outer::velocity::integral_reporting::VELOCITY_INTEGRAL_TERMINAL_REPLY_BYTES],
            },
            "integral_terminal",
        );
    }
}

/// Installed-tuning safety-envelope evidence.
#[klipper_reply]
pub struct FociOuterSafetyFault {
    pub oid: u8,
    /// `OUTER_SAFETY_FAULT_*` reason code.
    pub reason: u8,
    pub max_travel_mrev: u16,
    pub max_velocity_mrev_s: u16,
    pub max_duration_ms: u16,
    /// Reserved direction mask carried from the host request.
    pub direction_mask: u8,
    pub delta_counts: i32,
    pub dt_us: u32,
    pub velocity_counts_per_ms: i32,
    pub velocity_cap_counts_per_ms: i32,
    pub position_counts: i32,
    pub position_window_counts: i32,
    pub elapsed_us: u32,
    pub duration_cap_us: u32,
}

/// Outcome of the most recent homing mode.
#[klipper_reply]
pub struct FociStallResult {
    pub oid: u8,
    /// 1 if the stall latch set during the most recent homing mode.
    pub latched: u8,
    pub peak_error_units: u32,
    /// Detector tick at which the latch set, 0 if it never did.
    pub trigger_tick: u32,
    /// 1 while the homing clamp overrides the run-current limit.
    pub clamp_active: u8,
    /// Which check latched: 0 = none, 1 = ceiling, 2 = margin.
    pub trigger_path: u8,
    /// Largest error-minus-baseline observed while the baseline was valid,
    /// in TMC position units. 0 if the baseline never warmed up.
    pub peak_margin_delta_units: u32,
}

/// Final outer-loop tune output.
#[klipper_reply]
pub struct FociTuneResult {
    pub oid: u8,
    pub report_seq: u8,
    /// 0 = success, non-zero = error code.
    pub status: u8,
    pub velocity_p: u16,
    pub velocity_i: u16,
    pub position_p: u16,
    pub probed_velocity_mrev_s: u32,
    pub d_eq_q: i32,
    /// Q0.16 normalized confidence fraction (0 = none, 65535 ~= 1.0).
    pub confidence_q: u16,
    pub band_lower_percent: u8,
    pub band_upper_percent: u8,
    /// Q0.16 position within `[band_lower_percent, band_upper_percent]`
    /// (0 = lower edge, 65535 = upper edge).
    pub band_position_q: u16,
    pub predicted_start_p: u16,
    pub rungs_measured: u8,
    /// `PositionTune` feedforward paths live on the last admitted
    /// motion stroke (bit 0 velocity, 2 acceleration; bits 1, 3 and 4 are
    /// reserved and never set).
    pub stimulus_feedforward_paths: u8,
    /// Wire code of the specific `PositionTuneOutcome` behind a
    /// `CommissionError::PositionTuneFailed` (0 = not applicable).
    pub position_tune_outcome_code: u8,
}

/// Non-terminal `PositionTune` fragment.
#[klipper_reply]
pub struct FociTunePositionEvidence {
    pub oid: u8,
    pub report_seq: u8,
    pub last_rung_p: u16,
    pub min_overshoot_p: u16,
    pub min_overshoot_units: u32,
    pub bound_units: u32,
    pub homing_speed_mrev_s: u32,
    pub motion_speed_mrev_s: u32,
    pub homing_peak_abs_units: u32,
    pub motion_cruise_mean_abs_units: u32,
    pub motion_overshoot_units: u32,
    pub motion_cruise_rms_units: u32,
    pub dither_margin_milli: i32,
}

#[cfg(test)]
mod tune_result_reply_wire_tests {
    use super::{FociTunePositionEvidence, FociTuneResult};
    use ankyra::ScratchOutput;
    use ankyra::encoding::{Readable, Writable};

    fn reply_with_known_fields() -> FociTuneResult {
        FociTuneResult {
            oid: 1,
            report_seq: 2,
            status: 3,
            velocity_p: 4,
            velocity_i: 5,
            position_p: 6,
            probed_velocity_mrev_s: 7,
            d_eq_q: 8,
            confidence_q: 9,
            band_lower_percent: 10,
            band_upper_percent: 11,
            band_position_q: 12,
            predicted_start_p: 13,
            rungs_measured: 14,
            stimulus_feedforward_paths: 15,
            position_tune_outcome_code: 16,
        }
    }

    fn evidence_with_known_fields() -> FociTunePositionEvidence {
        FociTunePositionEvidence {
            oid: 1,
            report_seq: 2,
            last_rung_p: 3,
            min_overshoot_p: 4,
            min_overshoot_units: 5,
            bound_units: 6,
            homing_speed_mrev_s: 7,
            motion_speed_mrev_s: 8,
            homing_peak_abs_units: 9,
            motion_cruise_mean_abs_units: 10,
            motion_overshoot_units: 11,
            motion_cruise_rms_units: 12,
            dither_margin_milli: 13,
        }
    }

    #[test]
    fn terminal_fields_encode_in_declared_order() {
        let reply = reply_with_known_fields();
        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let encoded = scratch.result();

        assert_eq!(
            encoded,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
            "terminal fields must serialize in declared order"
        );
    }

    #[test]
    fn evidence_fields_encode_in_declared_order() {
        let evidence = evidence_with_known_fields();
        let mut scratch = ScratchOutput::<64>::new();
        evidence.write(&mut scratch);
        let encoded = scratch.result();

        assert_eq!(
            encoded,
            &[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13],
            "evidence fields must serialize in declared order"
        );
    }

    #[test]
    fn position_tune_failure_reply_carries_the_last_rungs_evidence() {
        let reply = FociTuneResult {
            oid: 1,
            report_seq: 0,
            status: crate::autotune::CommissionError::PositionTuneFailed.status_code(),
            velocity_p: 0,
            velocity_i: 0,
            position_p: 0,
            probed_velocity_mrev_s: 0,
            d_eq_q: 0,
            confidence_q: 0,
            band_lower_percent: 0,
            band_upper_percent: 0,
            band_position_q: 0,
            predicted_start_p: 0,
            rungs_measured: 3,
            stimulus_feedforward_paths: 0,
            position_tune_outcome_code: crate::autotune::PositionTuneOutcome::Conflict
                .to_wire_code(),
        };
        let evidence = FociTunePositionEvidence {
            oid: 1,
            report_seq: 0,
            last_rung_p: 388,
            min_overshoot_p: 195,
            min_overshoot_units: 40,
            bound_units: 0,
            homing_speed_mrev_s: 0,
            motion_speed_mrev_s: 0,
            homing_peak_abs_units: 0,
            motion_cruise_mean_abs_units: 0,
            motion_overshoot_units: 500,
            motion_cruise_rms_units: 0,
            dither_margin_milli: 0,
        };

        let mut scratch = ScratchOutput::<64>::new();
        reply.write(&mut scratch);
        let mut cursor = scratch.result();

        let _oid = u8::read(&mut cursor).expect("oid");
        let _report_seq = u8::read(&mut cursor).expect("report_seq");
        let status = u8::read(&mut cursor).expect("status");
        let _velocity_p = u16::read(&mut cursor).expect("velocity_p");
        let _velocity_i = u16::read(&mut cursor).expect("velocity_i");
        let _position_p = u16::read(&mut cursor).expect("position_p");
        let _probed_velocity_mrev_s = u32::read(&mut cursor).expect("probed_velocity_mrev_s");
        let _d_eq_q = i32::read(&mut cursor).expect("d_eq_q");
        let _confidence_q = u16::read(&mut cursor).expect("confidence_q");
        let _band_lower_percent = u8::read(&mut cursor).expect("band_lower_percent");
        let _band_upper_percent = u8::read(&mut cursor).expect("band_upper_percent");
        let _band_position_q = u16::read(&mut cursor).expect("band_position_q");
        let predicted_start_p = u16::read(&mut cursor).expect("predicted_start_p");
        let rungs_measured = u8::read(&mut cursor).expect("rungs_measured");
        let _stimulus_feedforward_paths =
            u8::read(&mut cursor).expect("stimulus_feedforward_paths");
        let outcome_code = u8::read(&mut cursor).expect("position_tune_outcome_code");

        assert_eq!(status, 12);
        assert_eq!(predicted_start_p, 0);
        assert_eq!(rungs_measured, 3);
        assert_eq!(outcome_code, 8);
        assert!(
            cursor.is_empty(),
            "every terminal field must have been decoded"
        );

        let mut evidence_scratch = ScratchOutput::<64>::new();
        evidence.write(&mut evidence_scratch);
        let mut evidence_cursor = evidence_scratch.result();

        let _oid = u8::read(&mut evidence_cursor).expect("oid");
        let _report_seq = u8::read(&mut evidence_cursor).expect("report_seq");
        let last_rung_p = u16::read(&mut evidence_cursor).expect("last_rung_p");
        let min_overshoot_p = u16::read(&mut evidence_cursor).expect("min_overshoot_p");
        let min_overshoot_units = u32::read(&mut evidence_cursor).expect("min_overshoot_units");
        let bound_units = u32::read(&mut evidence_cursor).expect("bound_units");
        let _homing_speed_mrev_s = u32::read(&mut evidence_cursor).expect("homing_speed_mrev_s");
        let _motion_speed_mrev_s = u32::read(&mut evidence_cursor).expect("motion_speed_mrev_s");
        let _homing_peak_abs_units =
            u32::read(&mut evidence_cursor).expect("homing_peak_abs_units");
        let _motion_cruise_mean_abs_units =
            u32::read(&mut evidence_cursor).expect("motion_cruise_mean_abs_units");
        let motion_overshoot_units =
            u32::read(&mut evidence_cursor).expect("motion_overshoot_units");
        let _motion_cruise_rms_units =
            u32::read(&mut evidence_cursor).expect("motion_cruise_rms_units");
        let _dither_margin_milli = i32::read(&mut evidence_cursor).expect("dither_margin_milli");

        assert_eq!(last_rung_p, 388);
        assert_eq!(min_overshoot_p, 195);
        assert_eq!(min_overshoot_units, 40);
        assert_eq!(bound_units, 0);
        assert_eq!(motion_overshoot_units, 500);
        assert!(
            evidence_cursor.is_empty(),
            "every evidence field must have been decoded"
        );
    }
}

/// Echo of a completed SPI transfer.
#[klipper_reply]
pub struct SpiTransferResponse<'a> {
    pub oid: u8,
    pub response: &'a [u8],
}

/// Periodic MCU heartbeat; must be a reply so klippy's `stats` response handler fires.
#[klipper_reply]
pub struct Stats {
    pub count: u32,
    pub sum: u32,
    pub sumsq: u32,
}

/// Current commanded stepper position.
#[klipper_reply]
pub struct StepperPosition {
    pub oid: u8,
    pub pos: i32,
}

/// Value read from a TMC4671 register.
#[klipper_reply]
pub struct TmcRegisterValue {
    pub oid: u8,
    pub addr: u8,
    pub value: u32,
}

/// `trsync_state` reply — emitted by the trigger subsystem as trsync
/// snapshots are drained.
#[klipper_reply]
pub struct TrsyncState {
    pub oid: u8,
    /// Non-zero while the trsync can still fire its trigger.
    pub can_trigger: u8,
    /// Reason code captured when the trsync transitioned out of
    /// `can_trigger`.
    pub trigger_reason: u8,
    pub clock: u32,
}

/// High/low 64-bit systick reading.
#[klipper_reply]
pub struct Uptime {
    /// High 32 bits of the MCU systick counter.
    pub high: u32,
    /// Low 32 bits of the MCU systick counter.
    pub clock: u32,
}
