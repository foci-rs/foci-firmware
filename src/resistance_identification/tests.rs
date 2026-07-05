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
    /// Current gain used by the default UD-proportional script, in milli-counts per UD count.
    scripted_current_per_ud_milli: i32,
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
    /// Scripted zero-window currents, consumed in capture order per axis.
    ///
    /// When non-empty, each zero window returns the next entry instead of `0`,
    /// modelling an offset that drifts across the sweep. The cursor advances
    /// once per zero window (on the MODE_RAMP_MODE_MOTION=Stopped write) and
    /// wraps so a single axis's 11 zero windows can reuse a shorter script.
    scripted_zero_currents: heapless::Vec<i32, 32>,
    /// Count of sampled zero windows, advanced on the first ADC read of each
    /// Stopped window. The active zero window indexes
    /// `scripted_zero_currents[count - 1]`. Counting on first sample (not on the
    /// mode write) excludes the unsampled pre-sweep and restore Stopped writes.
    zero_windows_sampled: usize,
    /// Whether the current Stopped window has already advanced the zero cursor.
    /// Reset on every MODE_RAMP_MODE_MOTION write.
    zero_window_counted: bool,
    /// Scripted positive signed-anchor current (UD = +signed_anchor_ud).
    ///
    /// When `Some`, the first and repeated positive-anchor energized windows
    /// return this raw current instead of the UD-proportional default.
    scripted_positive_anchor_current: Option<i32>,
    /// Scripted negative signed-anchor current (UD = -signed_anchor_ud).
    scripted_negative_anchor_current: Option<i32>,
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
            scripted_current_per_ud_milli: 1000,
            scripted_encoder_delta_count: 0,
            last_ud: 0,
            last_motion_mode: 0,
            encoder_count: 0,
            enable_pin: false,
            elapsed_us: 0,
            fail_after_writes: None,
            write_count: 0,
            scripted_zero_currents: heapless::Vec::new(),
            zero_windows_sampled: 0,
            zero_window_counted: false,
            scripted_positive_anchor_current: None,
            scripted_negative_anchor_current: None,
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

    /// Returns true when the backend recorded no register writes at all.
    ///
    /// A pre-energize validation reject (authority/motor-type/exposure) runs
    /// before the runner has written anything -- not even the power-stage
    /// setup writes the safe-state restore would otherwise undo -- so the
    /// write log must be completely empty. This is a stronger assertion than
    /// "no restore writes": it also proves the runner never reached the
    /// energizing body.
    fn wrote_nothing(&self) -> bool {
        self.writes.is_empty()
    }

    /// Current sampled for the active energized window, proportional to UD.
    fn energized_current_for_ud(&self) -> i32 {
        if self.scripted_energized_current_count != 0 {
            // Tests that force a fixed peak current use the override directly.
            return self.scripted_energized_current_count;
        }
        // Scripted signed-anchor currents override the UD-proportional default
        // so a test can pin the exact raw anchor reads independently of the
        // zero windows.
        let anchor_ud = self.profile.signed_anchor_ud;
        if self.last_ud == anchor_ud
            && let Some(current) = self.scripted_positive_anchor_current
        {
            return current;
        }
        if self.last_ud == -anchor_ud
            && let Some(current) = self.scripted_negative_anchor_current
        {
            return current;
        }
        // Clean line: current count tracks UD count with a configurable gain
        // so tests can model higher Vbus without changing the fit linearity.
        i32::from(self.last_ud) * self.scripted_current_per_ud_milli / 1000
    }

    /// Current sampled for the active zero window.
    ///
    /// Returns `0` unless a drift script is loaded, in which case the entry at
    /// the current cursor is returned (the cursor advances per zero window when
    /// MODE_RAMP_MODE_MOTION is set to Stopped).
    fn zero_current_for_window(&self) -> i32 {
        if self.scripted_zero_currents.is_empty() {
            return 0;
        }
        let sampled = self.zero_windows_sampled.max(1);
        let index = (sampled - 1) % self.scripted_zero_currents.len();
        self.scripted_zero_currents[index]
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
            self.zero_current_for_window()
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
            // The first ADC read of a Stopped window advances the drift-script
            // cursor, so only sampled zero windows consume a script entry.
            if self.last_motion_mode == 0 && !self.zero_window_counted {
                self.zero_windows_sampled += 1;
                self.zero_window_counted = true;
            }
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
            // A new window begins; arm the zero cursor to advance on this
            // window's first ADC sample (if it is a Stopped window).
            self.zero_window_counted = false;
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
fn runner_stops_after_good_fit_before_unsafe_high_points() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_current_per_ud_milli = 1200;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(
        result.is_ok(),
        "expected adaptive sweep to fit, got {result:?}"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1440u16)),
        "adaptive sweep must not apply the first predicted-unsafe high point"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1600u16)),
        "adaptive sweep must not continue to the fixed maximum once the fit is good"
    );
}

#[test]
fn runner_skips_predicted_unsafe_point_when_fit_is_not_yet_supported() {
    let mut backend = FakeResistanceBackend::standard();
    backend.scripted_current_per_ud_milli = 1400;
    backend.profile.min_selected_points = 6;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .unwrap_err();

    assert_eq!(
        err,
        ResistanceError::MeasurementEnvelopeUnsupported,
        "predicted-unsafe next point should be rejected as an unsupported measurement envelope"
    );
    assert!(
        !backend
            .writes
            .iter()
            .any(|write| write.addr == UQ_UD_EXT.addr() && write.value == u32::from(1280u16)),
        "next predicted-unsafe point must be skipped before energizing"
    );
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

/// Assert that `addr` was written with `value` before the safe-state restore
/// pass (i.e. somewhere among the first `writes.len() - 4` entries, since
/// `restore_safe_state` issues exactly 4 writes at the end of every run).
fn assert_written_before_restore(writes: &[RecordedWrite], addr: u8, value: u32) {
    let restore_start = writes.len().saturating_sub(4);
    assert!(
        writes[..restore_start]
            .iter()
            .any(|write| write.addr == addr && write.value == value),
        "expected addr {addr:#x}={value} written before the safe-state restore, got {writes:?}"
    );
}

#[test]
fn commissioning_mode_uses_full_sweep_authority_despite_low_saved_baseline() {
    // ADC calibration intentionally leaves PIDOUT_UQ_UD_LIMITS=1024 as a
    // restore baseline (see commissioning::inner::adc::step_adc_calibrate).
    // The runner must not clamp the sweep's PIDOUT limit to that baseline:
    // doing so silently clamps every sweep point below the baseline,
    // corrupting the fit while still recording the unclamped requested UD.
    let mut backend = FakeResistanceBackend::standard();
    backend.saved_voltage_limit = 1024;
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    assert!(
        max_abs_ud > backend.saved_voltage_limit,
        "fixture must exercise the corrupted-clamp scenario"
    );

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert_written_before_restore(&backend.writes, PIDOUT_UQ_UD_LIMITS.addr(), max_abs_ud);
    assert!(backend.wrote_safe_state());
}

#[test]
fn externally_armed_mode_rejects_when_saved_limit_is_below_sweep_authority() {
    // For a diagnostic run over an externally-armed motor, the operational
    // voltage limit is the authority and must not be exceeded by the sweep.
    //
    // This is a pre-energize validation: it runs before any power-stage
    // setup write, so the rejection must not disturb a motor the runner
    // never energized. Asserting `wrote_nothing` (not `wrote_safe_state`)
    // proves the runner did not write the safe-state restore sequence over
    // an externally armed motor it left untouched.
    let mut backend = FakeResistanceBackend::standard();
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    backend.saved_voltage_limit = max_abs_ud - 1;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::OutOfBoundsParameter);
    assert!(
        backend.wrote_nothing(),
        "an authority-reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn externally_armed_mode_rejects_wrong_motor_type_without_restore() {
    // The MOTOR_TYPE check is a pre-energize validation (a register read, no
    // energizing write): rejecting here must not write the safe-state
    // restore sequence over an externally armed motor the runner never
    // energized.
    let mut backend = FakeResistanceBackend::standard();
    backend.motor_type = foci_tmc4671::registers::MotorType::ThreePhase as u8;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::InvalidMotorType);
    assert!(
        backend.wrote_nothing(),
        "a motor-type reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn externally_armed_mode_proceeds_when_saved_limit_meets_sweep_authority() {
    let mut backend = FakeResistanceBackend::standard();
    let max_abs_ud = u32::from(backend.profile().max_abs_ud().unsigned_abs());
    backend.saved_voltage_limit = max_abs_ud;

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ));

    assert!(result.is_ok(), "expected ok, got {result:?}");
    assert_written_before_restore(&backend.writes, PIDOUT_UQ_UD_LIMITS.addr(), max_abs_ud);
}

#[test]
fn runner_rejects_profile_exceeding_exposure_budget() {
    // The exposure-budget gate is a pre-energize validation: it runs before
    // any power-stage setup write, so the rejection must not write the
    // safe-state restore sequence -- there is nothing to restore, and a
    // restore write would disturb a motor (in `ExternallyArmed` mode) the
    // runner never energized. Use `ExternallyArmed` here specifically so the
    // assertion proves the externally-armed motor is left untouched, not
    // merely that commissioning's enable pin was never asserted.
    let mut backend = FakeResistanceBackend::standard();
    // Drop the total-exposure budget below the fixed per-axis exposure so the
    // static budget gate must reject before any energizing.
    backend.profile.max_total_exposure = backend.profile.exposure_per_axis() - 1;

    let err = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::ExternallyArmed,
    ))
    .unwrap_err();

    assert_eq!(err, ResistanceError::UnsupportedProfile);
    assert!(
        backend.wrote_nothing(),
        "an exposure-budget reject must not touch the externally-armed motor: {:?}",
        backend.writes
    );
}

#[test]
fn sweep_point_is_bracketed_against_adjacent_zero_windows_not_initial_zero() {
    // Regression for the drift-compensation bug: every sweep point was
    // zero-subtracted against `(initial_zero + own trailing)/2`, biasing late
    // points against a stale start-of-sweep zero. The fix brackets each point
    // against its ADJACENT zeros: the prior point's trailing zero and its own
    // trailing zero.
    //
    // The script drifts only the trailing zero of the point preceding the
    // ud=800 sweep point (index 2). Energized currents track UD one-to-one, so
    // point 2 reads 800. Its adjacent zeros are (120 + 0)/2 = 60, giving a
    // delta of 740 -- just below the 750 linear threshold, so the point is
    // EXCLUDED. Under the old initial-zero bracket the zero would be
    // (0 + 0)/2 = 0, giving a delta of 800 and SELECTING the point. The
    // selected-point mask therefore pins which bracketing math ran.
    let mut backend = FakeResistanceBackend::standard();
    // Sampled zero windows per axis, in capture order:
    //   [initial, trailing(p0..p7) x8, repeated_zero, negative_zero] = 11.
    // Only the trailing zero of point 1 (index 2) carries drift.
    backend.scripted_zero_currents =
        heapless::Vec::from_slice(&[0, 0, 120, 0, 0, 0, 0, 0, 0, 0, 0]).unwrap();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .expect("adjacent-bracket sweep should fit");

    // ud=800 is sweep index 2. Adjacent bracket excludes it (delta 740 < 750).
    assert_eq!(
        result.axis0.selected_mask & (1 << 2),
        0,
        "ud=800 point must be excluded under adjacent-zero bracketing (delta 740), got mask {:#010b}",
        result.axis0.selected_mask
    );
    // The old initial-zero bracket would have selected it (delta 800 >= 750);
    // assert the remaining high-current points are still selected so the fit is
    // well-formed and the exclusion is specifically point 2.
    assert_eq!(
        result.axis0.selected_mask, 0b1111_1000,
        "only the five high-current points clear the threshold under adjacent bracketing"
    );
}

#[test]
fn signed_anchor_positive_current_is_zero_subtracted_like_the_negative_anchor() {
    // Regression for the signed-anchor consistency bug: the first positive
    // anchor was stored RAW while the negative and repeated anchors were
    // zero-subtracted, so `build_signed_anchor` mixed raw and zero-subtracted
    // currents and corrupted slope, asymmetry, and drift. With a constant
    // nonzero zero offset of 100, the positive anchor (raw 960) must be reported
    // as the zero-subtracted 860 -- consistent with the negative anchor.
    let mut backend = FakeResistanceBackend::standard();
    // Every zero window reads 100, so each anchor's bracket zero is also 100.
    backend.scripted_zero_currents = heapless::Vec::from_slice(&[100; 11]).unwrap();

    let result = block_on(run_resistance_identification(
        &mut backend,
        ResistanceRunMode::CommissioningManagedEnable,
    ))
    .expect("nonzero-offset sweep should fit");

    let anchor = result
        .axis0
        .signed_anchor
        .expect("axis must report signed-anchor evidence");

    // Positive anchor: raw energized 960 minus zero 100 = 860, NOT the raw 960.
    assert_eq!(
        anchor.positive_current_count, 860,
        "first positive anchor must be zero-subtracted (960 - 100), not raw 960"
    );
    // Repeated positive anchor: same zero-subtraction.
    assert_eq!(anchor.repeated_positive_current_count, 860);
    // Negative anchor: raw -960 minus zero 100 = -1060 (already zero-subtracted
    // before the fix; asserted here to pin the consistent treatment).
    assert_eq!(anchor.negative_current_count, -1060);
}
