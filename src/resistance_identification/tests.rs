//! Host-side runner tests with a fake backend.
//!
//! The fake backend records every register write in order, returns scripted
//! window samples, and tracks whether safe state was restored. The tests assert
//! the safety-critical sequencing of `run_resistance_identification`: the
//! voltage limit is bounded before the chopper is enabled, every failure path
//! restores safe state, and the exposure-budget and peak-current gates reject
//! before energizing.

use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION, MotorType,
    MotorTypeNPolePairs, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP,
    STATUS_FLAGS, UQ_UD_EXT,
};
use pollster::block_on;

use super::profile::ResistanceProfile;
use super::runner::{
    ResistanceBackend, ResistanceBackendError, ResistanceRunMode, run_resistance_identification,
};
use super::types::{ResistanceError, ResistanceWarningFlags};

/// One register write recorded by the fake backend, in issue order.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct RecordedWrite {
    addr: u8,
    value: u32,
}

/// Fake backend that records writes and returns scripted window samples.
struct FakeResistanceBackend {
    /// Ordered log of every register write.
    writes: heapless::Vec<RecordedWrite, 256>,
    /// Profile returned to the runner.
    profile: ResistanceProfile,
    /// Whether `is_armed_for_open_loop_voltage` reports true.
    armed_for_open_loop_voltage: bool,
    /// Whether ADC offsets are ready.
    adc_offsets_ready: bool,
    /// Motor type returned via MOTOR_TYPE_N_POLE_PAIRS.
    motor_type: u8,
    /// Saved PIDOUT_UQ_UD_LIMITS value read at entry and restored on exit.
    saved_voltage_limit: u32,
    /// PWM_MAXCNT readback.
    readback_pwm_maxcnt: u32,
    /// PWM_BBM_H_BBM_L readback.
    readback_pwm_bbm: u32,
    /// DSADC_MDEC_B_MDEC_A readback.
    readback_dsadc: u32,
    /// PWM_SV_CHOP readback after the runner writes 7.
    readback_pwm_sv_chop: u32,
    /// Encoder counts per revolution returned to the runner.
    encoder_counts_per_rev: u32,
    /// STATUS_FLAGS sampled in every window.
    scripted_status_flags: u32,
    /// Physical phase-current count sampled during energized windows.
    ///
    /// Built proportional to the active UD so the fit produces a clean line.
    scripted_energized_current_count: i32,
    /// Encoder delta applied across each energized window.
    scripted_encoder_delta_count: u32,
    /// Last UD written via UQ_UD_EXT, used to script current proportional to UD.
    last_ud: i16,
    /// Last motion mode written via MODE_RAMP_MODE_MOTION.
    last_motion_mode: u32,
    /// Monotonic encoder count returned by ABN_DECODER_COUNT.
    encoder_count: u32,
    /// Whether the enable pin is asserted.
    enable_pin: bool,
    /// Monotonic microsecond clock.
    elapsed_us: u64,
    /// When `Some(n)`, the nth write (1-based) and beyond fail with Spi.
    fail_after_writes: Option<usize>,
    /// Count of writes issued so far, for `fail_after_writes`.
    write_count: usize,
}

impl FakeResistanceBackend {
    /// Build a standard, healthy fake backend.
    fn standard() -> Self {
        let profile = ResistanceProfile::standard();
        Self {
            writes: heapless::Vec::new(),
            profile,
            armed_for_open_loop_voltage: true,
            adc_offsets_ready: true,
            motor_type: MotorType::TwoPhase as u8,
            saved_voltage_limit: 16_000,
            readback_pwm_maxcnt: u32::from(profile.pwm_maxcnt),
            readback_pwm_bbm: (u32::from(profile.bbm_h) << 8) | u32::from(profile.bbm_l),
            readback_dsadc: (u32::from(profile.dsadc_mdec_b) << 16)
                | u32::from(profile.dsadc_mdec_a),
            readback_pwm_sv_chop: profile.pwm_sv_chop,
            encoder_counts_per_rev: 4000,
            scripted_status_flags: 0,
            // Scale current ~= UD so the high-current points clear the linear
            // threshold and the fit stays well-conditioned.
            scripted_energized_current_count: 0,
            scripted_encoder_delta_count: 0,
            last_ud: 0,
            last_motion_mode: 0,
            encoder_count: 0,
            enable_pin: false,
            elapsed_us: 0,
            fail_after_writes: None,
            write_count: 0,
        }
    }

    /// Returns true when the recorded writes end in the safe-state restore set.
    ///
    /// Safe-state restore writes, in order: motion mode 0, UQ_UD_EXT 0,
    /// PWM_SV_CHOP 0, and the saved PIDOUT_UQ_UD_LIMITS.
    fn wrote_safe_state(&self) -> bool {
        let mode = MODE_RAMP_MODE_MOTION.addr();
        let uq_ud = UQ_UD_EXT.addr();
        let chop = PWM_SV_CHOP.addr();
        let limits = PIDOUT_UQ_UD_LIMITS.addr();

        let has = |addr: u8, value: u32| {
            self.writes
                .iter()
                .rev()
                .find(|write| write.addr == addr)
                .is_some_and(|write| write.value == value)
        };

        has(mode, 0) && has(uq_ud, 0) && has(chop, 0) && has(limits, self.saved_voltage_limit)
    }

    /// Current sampled for the active energized window, proportional to UD.
    fn energized_current_for_ud(&self) -> i32 {
        if self.scripted_energized_current_count != 0 {
            // Tests that force a fixed peak current use the override directly.
            return self.scripted_energized_current_count;
        }
        // Clean line: current count tracks UD count one-to-one.
        i32::from(self.last_ud)
    }

    /// Packed ADC_IWY_IUX value for the active window.
    ///
    /// Both halves carry the same scripted current so either axis channel reads
    /// the intended value.
    fn packed_phase_current(&self) -> u32 {
        let energized = self.last_motion_mode == 8;
        let value = if energized {
            self.energized_current_for_ud()
        } else {
            0
        };
        let half = (value as i16) as u16 as u32;
        (half << 16) | half
    }
}

impl ResistanceBackend for FakeResistanceBackend {
    async fn read_register(&mut self, addr: u8) -> Result<u32, ResistanceBackendError> {
        if addr == foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr() {
            return Ok(MotorTypeNPolePairs::new()
                .with_motor_type(self.motor_type)
                .with_n_pole_pairs(50)
                .into());
        }
        if addr == PIDOUT_UQ_UD_LIMITS.addr() {
            return Ok(self.saved_voltage_limit);
        }
        if addr == PWM_MAXCNT.addr() {
            return Ok(self.readback_pwm_maxcnt);
        }
        if addr == PWM_BBM_H_BBM_L.addr() {
            return Ok(self.readback_pwm_bbm);
        }
        if addr == DSADC_MDEC_B_MDEC_A.addr() {
            return Ok(self.readback_dsadc);
        }
        if addr == PWM_SV_CHOP.addr() {
            return Ok(self.readback_pwm_sv_chop);
        }
        if addr == STATUS_FLAGS.addr() {
            return Ok(self.scripted_status_flags);
        }
        if addr == ADC_IWY_IUX.addr() {
            return Ok(self.packed_phase_current());
        }
        if addr == ABN_DECODER_COUNT.addr() {
            return Ok(self.encoder_count);
        }
        Ok(0)
    }

    async fn write_register(&mut self, addr: u8, value: u32) -> Result<(), ResistanceBackendError> {
        self.write_count += 1;
        // Record every attempted write, including ones that fail, so that
        // safe-state restore is observable even after a backend error.
        self.writes
            .push(RecordedWrite { addr, value })
            .expect("write log overflow");
        if addr == UQ_UD_EXT.addr() {
            self.last_ud = (value & 0xffff) as u16 as i16;
        }
        if addr == MODE_RAMP_MODE_MOTION.addr() {
            self.last_motion_mode = value;
        }
        if let Some(threshold) = self.fail_after_writes
            && self.write_count > threshold
        {
            return Err(ResistanceBackendError::Spi);
        }
        Ok(())
    }

    async fn delay_ms(&mut self, ms: u64) {
        self.elapsed_us = self.elapsed_us.saturating_add(ms.saturating_mul(1000));
        if self.last_motion_mode == 8 {
            // Model bounded rotor motion while an energized window dwells, so the
            // encoder-move detector sees a delta between the before/after reads.
            self.encoder_count = self
                .encoder_count
                .wrapping_add(self.scripted_encoder_delta_count);
        }
    }

    fn elapsed_us(&self) -> u64 {
        self.elapsed_us
    }

    fn set_enable_pin(&mut self, enabled: bool) {
        self.enable_pin = enabled;
    }

    fn is_armed_for_open_loop_voltage(&self) -> bool {
        self.armed_for_open_loop_voltage
    }

    fn profile(&self) -> ResistanceProfile {
        self.profile
    }

    fn encoder_counts_per_rev(&self) -> u32 {
        self.encoder_counts_per_rev
    }

    fn adc_offsets_ready(&self) -> bool {
        self.adc_offsets_ready
    }
}

/// Assert that `first_addr` is written before `second_addr` in the log.
fn assert_write_order(writes: &[RecordedWrite], first_addr: u8, second_addr: u8) {
    let first = writes
        .iter()
        .position(|write| write.addr == first_addr)
        .expect("first address never written");
    let second = writes
        .iter()
        .position(|write| write.addr == second_addr)
        .expect("second address never written");
    assert!(
        first < second,
        "expected addr {first_addr:#x} before {second_addr:#x}, got {first} >= {second}"
    );
}

#[test]
fn runner_writes_voltage_limit_before_chopper() {
    let mut backend = FakeResistanceBackend::standard();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert_write_order(
        &backend.writes,
        PIDOUT_UQ_UD_LIMITS.addr(),
        PWM_SV_CHOP.addr(),
    );
}

#[test]
fn runner_rejects_wrong_power_stage() {
    let mut backend = FakeResistanceBackend::standard();
    backend.readback_pwm_maxcnt = 2047;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::WrongPowerStage);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_restores_safe_state_on_backend_error() {
    let mut backend = FakeResistanceBackend::standard();
    backend.fail_after_writes = Some(4);

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::BackendIo);
    assert!(backend.wrote_safe_state());
}

#[test]
fn externally_armed_mode_rejects_when_motor_is_not_armed() {
    let mut backend = FakeResistanceBackend::standard();
    backend.armed_for_open_loop_voltage = false;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::NotArmed);
}

#[test]
fn runner_aborts_when_profile_peak_current_is_exceeded() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_energized_current_count = 1800;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::PeakCurrentExceeded);
    assert!(backend.wrote_safe_state());
}

#[test]
fn runner_sets_encoder_moved_warning_when_motion_exceeds_profile_threshold() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_encoder_delta_count = 129;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap();

    assert!(
        result
            .warning_flags
            .contains(ResistanceWarningFlags::ENCODER_MOVED)
    );
}

#[test]
fn runner_sets_status_flags_warning_when_sampled_status_matches_mask() {
    let mut backend = FakeResistanceBackend::standard();
    let expected_status_flags = backend.profile().status_flags_warn_mask;
    backend.scripted_status_flags = expected_status_flags;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap();

    assert!(
        result
            .warning_flags
            .contains(ResistanceWarningFlags::STATUS_FLAGS)
    );
    assert_eq!(result.status_flags_or, expected_status_flags);
}

#[test]
fn runner_rejects_profile_exceeding_exposure_budget() {
    let mut backend = FakeResistanceBackend::standard();
    // Drop the total-exposure budget below the fixed per-axis exposure so the
    // static budget gate must reject before any energizing.
    backend.profile.max_total_exposure = backend.profile.exposure_per_axis() - 1;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::UnsupportedProfile);
    assert!(backend.wrote_safe_state());
}
