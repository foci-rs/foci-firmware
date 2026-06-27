use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, CONFIG_BIQUAD_F_ENABLE, CONFIG_BIQUAD_T_ENABLE,
    DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, MotorType,
    MotorTypeNPolePairs, OPENLOOP_ACCELERATION, OPENLOOP_MODE, OPENLOOP_PHI,
    OPENLOOP_VELOCITY_ACTUAL, OPENLOOP_VELOCITY_TARGET, PHI_E_EXT, PHI_E_SELECTION,
    PID_TORQUE_FLUX_ACTUAL, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP,
    STATUS_FLAGS, SubRegister, UQ_UD_EXT, UqUdExt,
};
use pollster::block_on;

use super::fit::fit_impedance;
use super::profile::ImpedanceProfile;
use super::runner::{
    EncoderWindow, ImpedanceAngleAccumulator, ImpedanceBackend, ImpedanceBackendError,
    run_impedance_identification,
};
use super::types::{
    IMPEDANCE_POST_SWEEP_ZERO_INDEX, IMPEDANCE_PROFILE_POINTS, IMPEDANCE_SAMPLE_COUNT,
    IMPEDANCE_STABILITY_ZERO_INDEX, ImpedanceAngleObservation, ImpedanceBaselineObservation,
    ImpedanceError, ImpedanceSample, ImpedanceWarningFlags,
};

const TURN: f64 = core::f64::consts::TAU;

fn synthetic_observations(
    profile: &ImpedanceProfile,
    mean: u32,
    saliency: u32,
    axis_angle: u16,
    residual: u16,
) -> [ImpedanceAngleObservation; IMPEDANCE_PROFILE_POINTS] {
    core::array::from_fn(|index| {
        let angle = profile.angle_points[index].angle_electrical_counts;
        let theta = TURN * f64::from(angle.wrapping_sub(axis_angle)) / 65_536.0;
        let response = f64::from(mean) + f64::from(saliency) * libm::cos(2.0 * theta);
        ImpedanceAngleObservation {
            angle_electrical_counts: angle,
            ud_abs: profile.angle_points[index].ud_abs,
            inductive_response_count_per_ud_milli: response.round() as u32,
            in_phase_response_count_per_ud_milli: mean / 2,
            secondary_adc_response_count_per_ud_milli: response.round() as u32,
            baseline_d_count: 0,
            baseline_q_count: 0,
            baseline_adc_ch0_count: 0,
            baseline_adc_ch1_count: 0,
            residual_permille: residual,
            coverage_permille: 1000,
            encoder_before: 0,
            encoder_after: 0,
            encoder_min: 0,
            encoder_max: 0,
            encoder_peak_counts: 0,
            valid: true,
            warning_flags: ImpedanceWarningFlags::NONE,
        }
    })
}

#[test]
fn standard_profile_uses_rotating_carrier_shape() {
    let profile = ImpedanceProfile::standard_openffboard();

    assert_eq!(IMPEDANCE_PROFILE_POINTS, 10);
    assert_eq!(IMPEDANCE_SAMPLE_COUNT, 160);
    assert_eq!(profile.hold_ud_abs, 640);
    assert_eq!(profile.angle_points[0].angle_electrical_counts, 0);
    assert_eq!(profile.angle_points[2].angle_electrical_counts, 8192);
    assert_eq!(profile.angle_points[6].angle_electrical_counts, 40960);
    assert_eq!(profile.angle_points[9].angle_electrical_counts, 0);
    assert_eq!(profile.carrier_frequency_millihz(), 25_000);
    assert_eq!(profile.samples_per_cycle, 20);
    assert_eq!(profile.cycles_per_angle, 8);
    assert_eq!(profile.sample_interval_us, 2_000);
    assert_eq!(profile.carrier_settle_us, 1_000);
    assert_eq!(profile.sample_count, IMPEDANCE_SAMPLE_COUNT as u16);
    assert_eq!(profile.max_abs_ud(), 1104);

    let mut faster_profile = profile;
    faster_profile.sample_interval_us = 1_000;
    assert_eq!(faster_profile.carrier_frequency_millihz(), 50_000);
}

#[test]
fn fit_impedance_recovers_two_theta_saliency() {
    let profile = ImpedanceProfile::standard_openffboard();
    let observations = synthetic_observations(&profile, 120, 40, 16_384, 0);

    let summary = fit_impedance(&profile, &observations).unwrap();

    assert_eq!(summary.mean_response_count_per_ud_milli, 120);
    assert_eq!(summary.axis_max_count_per_ud_milli, 160);
    assert_eq!(summary.axis_min_count_per_ud_milli, 80);
    assert_eq!(summary.axis_angle_electrical_counts, 16_384);
    assert_eq!(summary.axis_deviation_counts, 16_384);
    assert!(summary.saliency_determinable);
    assert!(summary.angle_coverage_permille >= profile.angle_coverage_fail_permille);
}

#[test]
fn fit_impedance_reports_repeatability_without_warning_flag() {
    let profile = ImpedanceProfile::standard_openffboard();
    let mut observations = synthetic_observations(&profile, 120, 40, 0, 0);
    observations[IMPEDANCE_POST_SWEEP_ZERO_INDEX].inductive_response_count_per_ud_milli = 90;
    observations[IMPEDANCE_STABILITY_ZERO_INDEX].inductive_response_count_per_ud_milli = 120;

    let summary = fit_impedance(&profile, &observations).unwrap();

    assert_eq!(summary.repeatability_permille, 286);
    assert!(
        !summary
            .warning_flags
            .contains(ImpedanceWarningFlags::REPEATABILITY)
    );
}

#[test]
fn fit_impedance_rejects_clustered_angle_coverage() {
    let profile = ImpedanceProfile::standard_openffboard();
    let mut observations = synthetic_observations(&profile, 120, 40, 0, 0);
    for observation in &mut observations[2..7] {
        observation.angle_electrical_counts = 0;
    }

    let err = fit_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::PoorCoverage);
}

#[test]
fn fit_impedance_rejects_high_residual_failure() {
    let profile = ImpedanceProfile::standard_openffboard();
    let observations = synthetic_observations(&profile, 120, 40, 0, 800);

    let err = fit_impedance(&profile, &observations).unwrap_err();

    assert_eq!(err, ImpedanceError::HighResidual);
}

#[test]
fn angle_accumulator_projects_current_onto_probe_axis() {
    let profile = ImpedanceProfile::standard_openffboard();
    let mut accumulator = ImpedanceAngleAccumulator::new(16_384, 1000);

    accumulator.accumulate(
        0,
        ImpedanceSample {
            t_us: 0,
            openloop_phi: 0,
            i_ch0: 0,
            i_ch1: 1000,
        },
        ImpedanceSample {
            t_us: 0,
            openloop_phi: 0,
            i_ch0: 0,
            i_ch1: 0,
        },
    );
    let observation = accumulator.finish(
        &profile,
        ImpedanceBaselineObservation {
            d_count: 0,
            q_count: 0,
            adc_ch0_count: 0,
            adc_ch1_count: 0,
        },
        EncoderWindow {
            before: 0,
            after: 0,
            min: 0,
            max: 0,
            counts_per_rev: 0,
        },
    );

    assert_eq!(observation.inductive_response_count_per_ud_milli, 2000);
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RecordedWrite {
    addr: u8,
    value: u32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RecordedOp {
    Read { addr: u8 },
    Write { addr: u8, value: u32 },
    DelayUs { us: u32 },
}

struct FakeImpedanceBackend {
    writes: heapless::Vec<RecordedWrite, 4096>,
    ops: heapless::Vec<RecordedOp, 32768>,
    armed_for_open_loop_voltage: bool,
    motor_type: u8,
    saved_voltage_limit: u32,
    pidout_limit: u32,
    uq_ud_ext: u32,
    mode_ramp_mode_motion: u32,
    openloop_mode: u32,
    openloop_acceleration: u32,
    openloop_velocity_target: u32,
    openloop_velocity_actual: u32,
    openloop_phi: u32,
    phi_e_selection: u32,
    phi_e_ext: u32,
    pwm_sv_chop: u32,
    pwm_maxcnt_readback: u32,
    bbm_readback: u32,
    dsadc_mdec_readback: u32,
    config_sub_addr: u8,
    torque_filter_enable: u32,
    flux_filter_enable: u32,
    timestamp_us: u32,
    status_flags: u32,
    readback_mismatch_addr: Option<u8>,
    readback_mismatch: Option<(u8, u32, u32)>,
    scripted_encoder_counts: heapless::Vec<u32, 128>,
    scripted_encoder_index: usize,
    observations: [Option<ImpedanceAngleObservation>; IMPEDANCE_PROFILE_POINTS],
    encoder_counts_per_rev: u32,
}

impl FakeImpedanceBackend {
    fn standard() -> Self {
        let saved_voltage_limit = 16_000;
        Self {
            writes: heapless::Vec::new(),
            ops: heapless::Vec::new(),
            armed_for_open_loop_voltage: true,
            motor_type: MotorType::TwoPhase as u8,
            saved_voltage_limit,
            pidout_limit: saved_voltage_limit,
            uq_ud_ext: 0,
            mode_ramp_mode_motion: 0,
            openloop_mode: 0,
            openloop_acceleration: 0,
            openloop_velocity_target: 0,
            openloop_velocity_actual: 0,
            openloop_phi: 0,
            phi_e_selection: 1,
            phi_e_ext: 0,
            pwm_sv_chop: 0,
            pwm_maxcnt_readback: 3999,
            bbm_readback: 0x0909,
            dsadc_mdec_readback: 0x014B_014B,
            config_sub_addr: 0,
            torque_filter_enable: 0,
            flux_filter_enable: 0,
            timestamp_us: 0,
            status_flags: 0,
            readback_mismatch_addr: None,
            readback_mismatch: None,
            scripted_encoder_counts: heapless::Vec::new(),
            scripted_encoder_index: 0,
            observations: [None; IMPEDANCE_PROFILE_POINTS],
            encoder_counts_per_rev: 0,
        }
    }

    fn with_encoder_script(mut self, counts: &[u32]) -> Self {
        for count in counts {
            self.scripted_encoder_counts
                .push(*count)
                .expect("encoder script overflow");
        }
        self
    }

    fn wrote_nothing(&self) -> bool {
        self.writes.is_empty()
    }

    fn wrote_safe_state(&self) -> bool {
        let expected = [
            (UQ_UD_EXT.addr(), 0),
            (OPENLOOP_VELOCITY_TARGET.addr(), 0),
            (OPENLOOP_VELOCITY_ACTUAL.addr(), 0),
            (OPENLOOP_ACCELERATION.addr(), 0),
            (OPENLOOP_PHI.addr(), 0),
            (OPENLOOP_MODE.addr(), 0),
            (MODE_RAMP_MODE_MOTION.addr(), 0),
            (PHI_E_SELECTION.addr(), 0),
            (PWM_SV_CHOP.addr(), 0),
            (PIDOUT_UQ_UD_LIMITS.addr(), self.saved_voltage_limit),
        ];
        self.writes.windows(expected.len()).any(|window| {
            expected.iter().enumerate().all(|(index, (addr, value))| {
                window[index].addr == *addr && window[index].value == *value
            })
        })
    }

    fn seed_subregister(&mut self, sub: SubRegister, value: u32) {
        if sub.sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
            self.torque_filter_enable = value;
        } else if sub.sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
            self.flux_filter_enable = value;
        } else {
            panic!("unsupported subregister");
        }
    }

    fn sub_write_indices(&self, sub: SubRegister, value: u32) -> heapless::Vec<usize, 16> {
        let mut indices = heapless::Vec::new();
        for (index, pair) in self.writes.windows(2).enumerate() {
            if pair[0].addr == sub.addr.addr()
                && pair[0].value == u32::from(sub.sub_addr)
                && pair[1].addr == sub.data.addr()
                && pair[1].value == value
            {
                indices.push(index).expect("sub write index overflow");
            }
        }
        indices
    }

    fn first_nonzero_uq_write_index(&self) -> usize {
        self.ops
            .iter()
            .position(|op| {
                matches!(
                    op,
                    RecordedOp::Write { addr, value }
                        if *addr == UQ_UD_EXT.addr() && *value != 0
                )
            })
            .expect("nonzero UQ_UD_EXT write missing")
    }

    fn read_index(&self, addr: u8) -> usize {
        self.ops
            .iter()
            .position(|op| matches!(op, RecordedOp::Read { addr: read_addr } if *read_addr == addr))
            .expect("read address missing")
    }

    fn write_index(&self, addr: u8, value: u32) -> usize {
        self.ops
            .iter()
            .position(|op| {
                matches!(
                    op,
                    RecordedOp::Write { addr: write_addr, value: write_value }
                        if *write_addr == addr && *write_value == value
                )
            })
            .expect("write missing")
    }

    fn last_ud(&self) -> i16 {
        (self.uq_ud_ext & 0xffff) as u16 as i16
    }

    fn last_uq(&self) -> i16 {
        (self.uq_ud_ext >> 16) as u16 as i16
    }

    fn packed_current(ch0: i16, ch1: i16) -> u32 {
        ((ch1 as u16 as u32) << 16) | ch0 as u16 as u32
    }

    fn current_sample(&self) -> u32 {
        let profile = ImpedanceProfile::standard_openffboard();
        let carrier_d = i32::from(self.last_ud()) - i32::from(profile.hold_ud_abs);
        let carrier_q = i32::from(self.last_uq());
        let carrier_sq = carrier_d
            .saturating_mul(carrier_d)
            .saturating_add(carrier_q.saturating_mul(carrier_q));
        if carrier_sq == 0 {
            return Self::packed_current(0, 0);
        }

        let angle = libm::atan2(f64::from(carrier_q), f64::from(carrier_d));
        let normalized_response = 360.0 + 140.0 * libm::cos(2.0 * (angle - TURN / 4.0));
        let current_amplitude =
            normalized_response * f64::from(profile.angle_points[0].ud_abs) / 1_000.0 * 1.3;
        let carrier = libm::sqrt(f64::from(carrier_sq));
        let d_count = (f64::from(carrier_d) * current_amplitude / carrier).round() as i16;
        let q_count = (f64::from(carrier_q) * current_amplitude / carrier).round() as i16;
        Self::packed_current(d_count, q_count)
    }

    fn readback_value(&self, addr: u8) -> u32 {
        if Some(addr) == self.readback_mismatch_addr {
            return 0xffff_ffff;
        }
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            return self.pidout_limit;
        }
        if addr == UQ_UD_EXT.addr() {
            return self.uq_ud_ext;
        }
        if addr == MODE_RAMP_MODE_MOTION.addr() {
            return self.mode_ramp_mode_motion;
        }
        if addr == OPENLOOP_MODE.addr() {
            return self.openloop_mode;
        }
        if addr == OPENLOOP_ACCELERATION.addr() {
            return self.openloop_acceleration;
        }
        if addr == OPENLOOP_VELOCITY_TARGET.addr() {
            return self.openloop_velocity_target;
        }
        if addr == OPENLOOP_VELOCITY_ACTUAL.addr() {
            return self.openloop_velocity_actual;
        }
        if addr == OPENLOOP_PHI.addr() {
            return self.openloop_phi;
        }
        if addr == PHI_E_SELECTION.addr() {
            return self.phi_e_selection;
        }
        if addr == PHI_E_EXT.addr() {
            return self.phi_e_ext;
        }
        if addr == PWM_SV_CHOP.addr() {
            return self.pwm_sv_chop;
        }
        if addr == PWM_MAXCNT.addr() {
            return self.pwm_maxcnt_readback;
        }
        if addr == PWM_BBM_H_BBM_L.addr() {
            return self.bbm_readback;
        }
        if addr == DSADC_MDEC_B_MDEC_A.addr() {
            return self.dsadc_mdec_readback;
        }
        0
    }
}

impl ImpedanceBackend for FakeImpedanceBackend {
    async fn read_register(&mut self, addr: u8) -> Result<u32, ImpedanceBackendError> {
        self.ops
            .push(RecordedOp::Read { addr })
            .expect("op log overflow");
        if addr == MOTOR_TYPE_N_POLE_PAIRS.addr() {
            return Ok(MotorTypeNPolePairs::new()
                .with_motor_type(self.motor_type)
                .with_n_pole_pairs(50)
                .into());
        }
        if addr == STATUS_FLAGS.addr() {
            return Ok(self.status_flags);
        }
        if addr == ABN_DECODER_COUNT.addr() {
            if !self.scripted_encoder_counts.is_empty() {
                let index = self
                    .scripted_encoder_index
                    .min(self.scripted_encoder_counts.len() - 1);
                let count = self.scripted_encoder_counts[index];
                self.scripted_encoder_index += 1;
                return Ok(count);
            }
            return Ok(0);
        }
        if addr == ADC_IWY_IUX.addr() || addr == PID_TORQUE_FLUX_ACTUAL.addr() {
            return Ok(self.current_sample());
        }
        if addr == CONFIG_BIQUAD_T_ENABLE.data.addr() {
            if self.config_sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
                return Ok(self.torque_filter_enable);
            }
            if self.config_sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
                return Ok(self.flux_filter_enable);
            }
        }
        Ok(self.readback_value(addr))
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ImpedanceBackendError> {
        self.writes
            .push(RecordedWrite { addr, value })
            .expect("write log overflow");
        self.ops
            .push(RecordedOp::Write { addr, value })
            .expect("op log overflow");
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            self.pidout_limit = value;
        } else if addr == UQ_UD_EXT.addr() {
            self.uq_ud_ext = value;
        } else if addr == MODE_RAMP_MODE_MOTION.addr() {
            self.mode_ramp_mode_motion = value;
        } else if addr == OPENLOOP_MODE.addr() {
            self.openloop_mode = value;
        } else if addr == OPENLOOP_ACCELERATION.addr() {
            self.openloop_acceleration = value;
        } else if addr == OPENLOOP_VELOCITY_TARGET.addr() {
            self.openloop_velocity_target = value;
        } else if addr == OPENLOOP_VELOCITY_ACTUAL.addr() {
            self.openloop_velocity_actual = value;
        } else if addr == OPENLOOP_PHI.addr() {
            self.openloop_phi = value;
        } else if addr == PHI_E_SELECTION.addr() {
            self.phi_e_selection = value;
        } else if addr == PHI_E_EXT.addr() {
            self.phi_e_ext = value;
        } else if addr == PWM_SV_CHOP.addr() {
            self.pwm_sv_chop = value;
        } else if addr == CONFIG_BIQUAD_T_ENABLE.addr.addr() {
            self.config_sub_addr = value as u8;
        } else if addr == CONFIG_BIQUAD_T_ENABLE.data.addr() {
            if self.config_sub_addr == CONFIG_BIQUAD_T_ENABLE.sub_addr {
                self.torque_filter_enable = value;
            } else if self.config_sub_addr == CONFIG_BIQUAD_F_ENABLE.sub_addr {
                self.flux_filter_enable = value;
            }
        }
        Ok(())
    }

    async fn delay_us(&mut self, us: u32) {
        self.ops
            .push(RecordedOp::DelayUs { us })
            .expect("op log overflow");
        self.timestamp_us = self.timestamp_us.wrapping_add(us);
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.timestamp_us = self
            .timestamp_us
            .wrapping_add((ms as u32).saturating_mul(1_000));
    }

    fn timestamp_us(&self) -> u32 {
        self.timestamp_us
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.armed_for_open_loop_voltage
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.encoder_counts_per_rev
    }

    fn record_angle_observation(
        &mut self,
        point_index: usize,
        observation: ImpedanceAngleObservation,
    ) {
        self.observations[point_index] = Some(observation);
    }

    fn record_profile_readback_mismatch(&mut self, addr: u8, expected: u32, actual: u32) {
        self.readback_mismatch = Some((addr, expected, actual));
    }
}

fn assert_no_nonzero_uq_writes(writes: &[RecordedWrite]) {
    assert!(
        !writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value != 0),
        "expected no non-zero UQ_UD_EXT writes, got {writes:?}"
    );
}

#[test]
fn runner_unarmed_mode_writes_nothing() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.armed_for_open_loop_voltage = false;

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::NotArmed);
    assert!(backend.wrote_nothing());
}

#[test]
fn runner_never_writes_openloop_velocity_target() {
    let mut backend = FakeImpedanceBackend::standard();

    block_on(run_impedance_identification(&mut backend)).unwrap();

    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == OPENLOOP_VELOCITY_TARGET.addr() && write.value != 0),
        "runner must not command the open-loop velocity generator: {:?}",
        backend.writes
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == PHI_E_EXT.addr() && write.value != 0),
        "runner must keep the held electrical angle fixed"
    );
}

#[test]
fn runner_verifies_power_stage_before_first_carrier_write() {
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    let first_carrier = backend.first_nonzero_uq_write_index();
    let pwm_write = backend.write_index(PWM_SV_CHOP.addr(), 7);
    assert!(pwm_write < first_carrier);
    for addr in [
        PWM_MAXCNT.addr(),
        PWM_BBM_H_BBM_L.addr(),
        DSADC_MDEC_B_MDEC_A.addr(),
        PWM_SV_CHOP.addr(),
    ] {
        assert!(
            backend.read_index(addr) < first_carrier,
            "read {addr:#x} did not happen before first carrier write"
        );
    }
}

#[test]
fn runner_setup_readback_mismatch_blocks_first_carrier_write() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.readback_mismatch_addr = Some(OPENLOOP_MODE.addr());

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::ProfileReadbackMismatch);
    assert_eq!(
        backend.readback_mismatch,
        Some((OPENLOOP_MODE.addr(), 0, 0xffff_ffff))
    );
    assert_no_nonzero_uq_writes(&backend.writes);
}

#[test]
fn runner_rotates_carrier_vector_around_fixed_hold_angle() {
    let profile = ImpedanceProfile::standard_openffboard();
    let mut backend = FakeImpedanceBackend::standard();

    let result = block_on(run_impedance_identification(&mut backend));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    let hold_ud = profile.hold_ud_abs as i16;
    let carrier_ud = profile.angle_points[3].ud_abs as i16;
    assert_eq!(profile.angle_points[3].angle_electrical_counts, 16_384);
    let q_axis_carrier_high: u32 = UqUdExt::new()
        .with_ud_ext(hold_ud)
        .with_uq_ext(carrier_ud)
        .into();
    let q_axis_carrier_low: u32 = UqUdExt::new()
        .with_ud_ext(hold_ud)
        .with_uq_ext(-carrier_ud)
        .into();

    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == q_axis_carrier_high),
        "90-degree carrier high vector missing"
    );
    assert!(
        backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == q_axis_carrier_low),
        "90-degree carrier low vector missing"
    );
}

#[test]
fn runner_snapshots_disables_and_restores_torque_flux_filters() {
    let mut backend = FakeImpedanceBackend::standard();
    backend.seed_subregister(CONFIG_BIQUAD_T_ENABLE, 1);
    backend.seed_subregister(CONFIG_BIQUAD_F_ENABLE, 1);

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.filter_state.torque_enable_before, 1);
    assert_eq!(result.filter_state.flux_enable_before, 1);
    assert_eq!(result.filter_state.torque_enable_during, 0);
    assert_eq!(result.filter_state.flux_enable_during, 0);
    let torque_disable = backend.sub_write_indices(CONFIG_BIQUAD_T_ENABLE, 0);
    let flux_disable = backend.sub_write_indices(CONFIG_BIQUAD_F_ENABLE, 0);
    let torque_restore = backend.sub_write_indices(CONFIG_BIQUAD_T_ENABLE, 1);
    let flux_restore = backend.sub_write_indices(CONFIG_BIQUAD_F_ENABLE, 1);
    assert!(!torque_disable.is_empty(), "torque filter was not disabled");
    assert!(!flux_disable.is_empty(), "flux filter was not disabled");
    assert!(!torque_restore.is_empty(), "torque filter was not restored");
    assert!(!flux_restore.is_empty(), "flux filter was not restored");
    assert!(torque_disable[0] < torque_restore[0]);
    assert!(flux_disable[0] < flux_restore[0]);
}

#[test]
fn runner_records_failing_angle_observation_on_encoder_hard_abort() {
    let profile = ImpedanceProfile::standard_openffboard();
    let mut backend = FakeImpedanceBackend::standard().with_encoder_script(&[1000, 1136, 1136]);

    let err = block_on(run_impedance_identification(&mut backend)).unwrap_err();

    assert_eq!(err, ImpedanceError::EncoderMoved);
    let observation = backend.observations[0].expect("failing observation recorded");
    assert_eq!(
        observation.angle_electrical_counts,
        profile.angle_points[0].angle_electrical_counts
    );
    assert_eq!(observation.encoder_before, 1000);
    assert_eq!(
        observation.encoder_peak_counts,
        profile.encoder_move_fail_counts + 8
    );
    assert!(!observation.valid);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_standard_profile_treats_latched_hold_status_as_warning_only() {
    let mut backend = FakeImpedanceBackend::standard();
    let observed_hold_status = 0x7000_4480;
    backend.status_flags = observed_hold_status;

    let result = block_on(run_impedance_identification(&mut backend)).unwrap();

    assert_eq!(result.status_flags_or, observed_hold_status);
    assert!(
        result
            .warning_flags
            .contains(ImpedanceWarningFlags::STATUS_FLAGS)
    );
    assert!(backend.wrote_safe_state());
}
