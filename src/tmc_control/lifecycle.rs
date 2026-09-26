//! Shared motor lifecycle helpers for terminal TMC control paths.

use crate::commission_dispatch::DispatchState;
use crate::commissioning::types::CommissioningBackend;

const CLOSED_LOOP_POSITION_MODE: u32 = 0x8000_0003;

/// A motor lifecycle flag transitioned by [`arm_motor_electrical`],
/// [`arm_motor_for_steps`], or [`disarm_motor`].
///
/// Recorded through `CommissionMotorControl::record_lifecycle` at the exact
/// mutation site so a test backend can observe these transitions
/// interleaved with SPI/GPIO ordering, not just their final values.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum LifecycleFlag {
    /// `DispatchState::motor_enabled` (electrical arm/disarm).
    MotorEnabled,
    /// `TmcChannelState::motor_armed` (step-timer processing arm/disarm).
    MotorArmed,
    /// `DispatchState::calibrated`.
    Calibrated,
}

pub(crate) async fn settle_closed_loop_hold_before_enable<B: CommissioningBackend>(
    backend: &mut B,
) -> Result<(), crate::autotune::CommissionError> {
    use foci_tmc4671::registers::{
        MODE_RAMP_MODE_MOTION, PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
        PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_POSITION_ACTUAL,
    };

    let mode = backend
        .read_register(MODE_RAMP_MODE_MOTION)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;
    if mode != CLOSED_LOOP_POSITION_MODE {
        return Ok(());
    }

    let actual = backend
        .read_register(PID_POSITION_ACTUAL)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;
    backend
        .write_register(PID_POSITION_ACTUAL, actual)
        .await
        .map_err(|_| crate::autotune::CommissionError::Spi)?;

    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_TORQUE_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_FLUX_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_VELOCITY_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::commissioning::helpers::write_sub_register_checked(
        backend,
        PID_ERROR_PID_POSITION_ERROR_SUM,
        0,
    )
    .await
    .map_err(|_| crate::autotune::CommissionError::Spi)?;
    crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await
}

/// Mark the motor electrically energized after verified calibration.
///
/// Asserts `state.calibrated`; clears any stale homing latch/clamp flags;
/// drains any recorded-but-unwritten configuration; verifies the calibration
/// sentinel register; rewrites `PID_TORQUE_FLUX_LIMITS` from the stored
/// run-current limit; sets the enable pin high; updates
/// `state.motor_enabled`. Does NOT touch channel motor arming — stepper
/// arming is a separate concern (see [`arm_motor_for_steps`]).
///
/// The drain runs before anything else touches the chip because config-time
/// setters only record their values; a host that changes the run current and
/// enables in the same breath would otherwise energize against a
/// `PID_TORQUE_FLUX_LIMITS` the chip never received. The sentinel does not
/// cover this — it detects chip resets, not stale configuration.
///
/// The drain covers what the host had already sent when this arm began, but
/// the handlers run at a higher priority than this path and can record more
/// configuration during any of the awaiting steps that follow. A recorded
/// value is not necessarily reflected in what this function energizes
/// against -- `tmc_set_current` updates only the pending run current, leaving
/// the torque-flux rewrite below to use the limit the host has just replaced
/// -- so a final `config_apply_pending` check immediately before the enable
/// pin refuses the arm instead. It returns an error rather than asserting:
/// this interleaving is ordinary traffic, and the next control tick drains
/// the bit so the host's retry arms against the value it sent.
///
/// That check and the enable pin share one critical section. Split apart
/// they are a check-then-act pair on state a higher-priority task owns, and
/// a handler landing in between would leave the arm energizing against the
/// very configuration it had just established was absent. Masking is what
/// makes the check binding rather than advisory. The section holds an
/// atomic load and a GPIO write and nothing else -- the refusal is logged
/// after it, so an RTT write never runs masked.
///
/// The torque-flux rewrite makes this path truthful even when a homing
/// clamp narrowed the register on a prior arm: a shutdown or disable
/// landing mid-homing could otherwise leave the clamped value in the chip,
/// and a plain re-arm would run at it. Clearing `stall_latched` and
/// `homing_clamp_active` here is safe precisely because this rewrite
/// always restores the full run-current limit before the motor is
/// energized.
///
/// `assert!`, not `debug_assert!`: this is a hard motor-safety
/// contract and must enforce in release builds. Costs one branch
/// per arming (not on the hot path). On failure: panic -> RTIC fault
/// handler -> safe-state via the GPIO Drop and `PIDOUT_UQ_UD_LIMITS` cap.
///
/// # Errors
///
/// Returns [`crate::autotune::CommissionError::SafeStateIncomplete`] if the
/// most recent safe-state restore pass had unresolved write failures.
/// Returns [`crate::autotune::CommissionError::ChipResetDetected`] if the
/// sentinel is inactive or does not match the TMC4671 readback.
/// Returns [`crate::autotune::CommissionError::InvalidCurrentLimit`] if the
/// stored run-current limit is zero (unconfigured).
/// Returns [`crate::autotune::CommissionError::Spi`] if the pending-config
/// drain fails (the failed item is named in the log), if the torque-flux
/// limit rewrite fails, or if configuration was recorded after the drain.
/// Returns [`crate::autotune::CommissionError::FaultInterlock`] if the fault
/// interlock is tripped when the enable pin would be raised.
pub async fn arm_motor_electrical<B: CommissioningBackend>(
    backend: &mut B,
    state: &mut DispatchState<'_>,
) -> Result<(), crate::autotune::CommissionError> {
    let mut step = ArmMotorElectricalStep::start();
    loop {
        if let Some(result) = step.advance(backend, state).await {
            return result;
        }
    }
}

/// Arm the stepper for step-timer processing.
///
/// Asserts `state.motor_enabled` (set only by `arm_motor_electrical`);
/// stores the channel motor-armed flag. Caller is responsible for any
/// prerequisite state (e.g. zeroing the stepper channel position) before
/// invoking. Always-on assert.
pub fn arm_motor_for_steps(state: &mut DispatchState<'_>) {
    assert!(
        *state.motor_enabled,
        "FATAL: arm_motor_for_steps called with motor_enabled=false"
    );
    state
        .tmc
        .motor_armed
        .store(true, core::sync::atomic::Ordering::Relaxed);
}

/// De-energize the motor and clear all armed/enabled state.
///
/// Clears the channel motor-armed flag first so the step-timer ISR stops
/// accepting step events before async SPI cleanup begins, then routes the
/// safe-state register sequence through
/// [`crate::tmc_safe_state::restore_safe_state_via_backend`] and clears the
/// latched velocity feedforward clamp, `state.calibrated`, and
/// `state.motor_enabled`.
///
/// Idempotent: safe to call when the motor is already disarmed.
/// Does not touch the stepper queue or invoke `stepper_stop_from_p1` —
/// those remain board-side responsibilities.
pub async fn disarm_motor<B: CommissioningBackend>(backend: &mut B, state: &mut DispatchState<'_>) {
    state
        .tmc
        .motor_armed
        .store(false, core::sync::atomic::Ordering::Relaxed);
    backend.record_lifecycle(LifecycleFlag::MotorArmed, false);
    crate::tmc_safe_state::restore_safe_state_via_backend(backend).await;
    state.tmc.clear_velocity_clamp_latch();
    *state.calibrated = false;
    *state.motor_enabled = false;
    backend.record_lifecycle(LifecycleFlag::MotorEnabled, false);
}

/// Apply the shared shutdown safe-state policy for one TMC channel.
///
/// Drops any runtime-owned homing state (stall detector, clamp retry
/// budget) and clears the channel's homing latch/clamp flags, then
/// de-energizes the motor through [`disarm_motor`], then restores phase
/// advance, then stops any active commissioning engine. A retained
/// breakaway campaign is completed only after both safe-state operations
/// succeed.
///
/// Both boards' shutdown loops funnel through this shared helper, so a
/// shutdown landing mid-homing clears the detector synchronously on either
/// board; the homing tick's state-0 self-heal remains the backstop for a
/// config reset.
///
/// Returns `true` when the phase-advance restore either was unnecessary or
/// succeeded. Returns `false` when restore failed; callers should keep retrying
/// this helper while shutdown remains active.
pub async fn apply_shutdown_safe_state<B: CommissioningBackend>(
    runtime: &mut crate::tmc_control::TmcControlRuntime,
    backend: &mut B,
    tmc: &'static crate::state::tmc::TmcChannelState,
    stepper: &'static crate::state::stepper::StepperChannelState,
) -> bool {
    // Halt the step-queue stimulus first, before anything below clears
    // `motor_armed` or writes a safe-state register. Clearing `motor_armed`
    // stops the step-timer ISR from emitting further steps, but does not
    // itself drain the queue, cancel the scheduler's stepper entry, or close
    // the stimulus's motor-armed capture session (see
    // `TmcChannelState::stimulus_restore_motor_armed`) -- left open, that
    // session corrupts the `motor_armed` restore of whatever stimulus runs
    // next. Unconditional, matching the engine-level shutdown abort in
    // `CommissioningEngine::step`: `stimulus_stop`/
    // `set_stimulus_feedforward(false)` are documented idempotent, a safe
    // no-op when no stimulus session is open, and this helper is called
    // repeatedly while shutdown remains active.
    crate::commissioning::outer::position_tune::abort_stimulus(backend);
    // A terminal restore caught mid-sequence is driven to safe state here,
    // before the disarm and engine take below clear the state it depends on.
    // Both boards funnel their shutdown path through this helper.
    if runtime.is_restore_pending() {
        runtime.request_restore_cancel();
    }
    while runtime.restore_cancel_pending() {
        let _ = runtime
            .drive_restore_cancel(
                backend,
                tmc,
                stepper,
                crate::autotune::CommissionError::Shutdown,
            )
            .await;
    }
    runtime.clear_homing_runtime();
    tmc.homing_clamp_active
        .store(false, core::sync::atomic::Ordering::Relaxed);
    tmc.stall_latched
        .store(false, core::sync::atomic::Ordering::Relaxed);
    let mut state = runtime.dispatch_state(tmc, stepper);
    disarm_motor(backend, &mut state).await;
    let phase_advance_restored = runtime.restore_phase_advance_via_backend(backend).await;
    if !phase_advance_restored {
        defmt::warn!("phase advance: failed to restore ABN offset on shutdown");
    }
    if phase_advance_restored {
        runtime.take_commission_engine_after_handled_stop();
    } else {
        runtime.take_commission_engine();
    }
    phase_advance_restored
}

/// The [`arm_motor_electrical`] sequence, advanced one backend operation (or
/// one `drain_pending_config` call, kept atomic — see the module-level
/// Scope Note) per `advance()` tick. `arm_motor_electrical` drives it to
/// completion in one call; the terminal restore polls it once per tick.
///
/// The final tick (`FinalizeEnable`) performs the `config_apply_pending`
/// recheck and the enable-pin assertion inside one `critical_section::with`
/// closure -- splitting them would reopen the fail-closed race
/// `arm_motor_electrical`'s doc comment describes (config recorded by a
/// preempting handler between the recheck and the enable-pin write).
#[derive(Clone, Copy)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum ArmMotorElectricalStep {
    Drain,
    /// FOCI's re-arm gate after a chip reset. A reset restores
    /// `MOTOR_TYPE_N_POLE_PAIRS` to its power-on value (`0x0000_0001`,
    /// TMC4671-LA datasheet Rev 1 02/2024, register 0x1B), which differs from
    /// the commissioned
    /// [`crate::state::tmc::TmcChannelState::motor_sentinel`], so the readback
    /// fails until commissioning re-establishes it. The TMC4671 has no reset
    /// indicator like Klipper's `CS_ACTUAL`, so this readback and the
    /// PI-representation check that follows stand in for one.
    VerifySentinelReadback,
    VerifyPiRepresentation(crate::tmc_control::pi_representation::PiRepresentationVerifyStep),
    WriteCurrentLimit,
    SettleCheckMode,
    SettleReadPositionActual,
    SettleWritePositionActual {
        actual: u32,
    },
    SettleClearIntegrators {
        ops: [(foci_tmc4671::registers::Register, u32); 8],
        next: u8,
    },
    SettleClearStatusFlags,
    FinalizeEnable,
}

impl ArmMotorElectricalStep {
    pub(crate) const fn start() -> Self {
        Self::Drain
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        state: &mut DispatchState<'_>,
    ) -> Option<Result<(), crate::autotune::CommissionError>> {
        use crate::autotune::CommissionError;
        use core::sync::atomic::Ordering::Relaxed;

        match self {
            Self::Drain => {
                assert!(
                    *state.calibrated,
                    "FATAL: arm_motor_electrical called with calibrated=false; \
                     SPI restore must precede arm"
                );
                state.tmc.stall_latched.store(false, Relaxed);
                state.tmc.homing_clamp_active.store(false, Relaxed);
                if state.tmc.safe_state_last_pass_failed.load(Relaxed) {
                    defmt::error!(
                        "arm_motor_electrical: safe-state cleanup incomplete, refusing arm"
                    );
                    return Some(Err(CommissionError::SafeStateIncomplete));
                }
                if let Err(item) = backend
                    .drain_pending_config(crate::tmc_control::pending_config::PendingConfigSink {
                        stored_inner_gains: state.stored_gains,
                    })
                    .await
                {
                    defmt::error!(
                        "arm_motor_electrical: pending config apply failed ({}), refusing arm",
                        item
                    );
                    return Some(Err(CommissionError::Spi));
                }
                *self = Self::VerifySentinelReadback;
                None
            }
            Self::VerifySentinelReadback => {
                use foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS;
                let expected = state.tmc.motor_sentinel.load(Relaxed);
                if expected == 0 {
                    defmt::error!("arm_motor_electrical: missing MOTOR_TYPE sentinel");
                    return Some(Err(CommissionError::ChipResetDetected));
                }
                if backend
                    .read_register_verified(MOTOR_TYPE_N_POLE_PAIRS, expected)
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::ChipResetDetected));
                }
                *self = Self::VerifyPiRepresentation(
                    crate::tmc_control::pi_representation::PiRepresentationVerifyStep::start(),
                );
                None
            }
            Self::VerifyPiRepresentation(step) => match step.advance(backend, state.tmc).await {
                None => None,
                Some(Ok(())) => {
                    // The compound "safe-state pass AND fresh sentinel pass"
                    // latch clears only here, once both halves have actually
                    // succeeded -- matching `arm_motor_electrical` exactly.
                    state.tmc.safe_state_incomplete.store(false, Relaxed);
                    *self = Self::WriteCurrentLimit;
                    None
                }
                Some(Err(_)) => Some(Err(CommissionError::ChipResetDetected)),
            },
            Self::WriteCurrentLimit => {
                let run_limit = state.tmc.load_current_limit(Relaxed);
                if run_limit.raw() == 0 {
                    defmt::error!("arm_motor_electrical: no run current configured");
                    return Some(Err(CommissionError::InvalidCurrentLimit));
                }
                if backend
                    .write_register(
                        foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS,
                        run_limit.register_value(),
                    )
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::Spi));
                }
                *self = Self::SettleCheckMode;
                None
            }
            Self::SettleCheckMode => {
                let mode = match backend
                    .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                    .await
                {
                    Ok(mode) => mode,
                    Err(_) => return Some(Err(CommissionError::Spi)),
                };
                *self = if mode == CLOSED_LOOP_POSITION_MODE {
                    Self::SettleReadPositionActual
                } else {
                    Self::FinalizeEnable
                };
                None
            }
            Self::SettleReadPositionActual => {
                match backend
                    .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                    .await
                {
                    Ok(actual) => {
                        *self = Self::SettleWritePositionActual { actual };
                        None
                    }
                    Err(_) => Some(Err(CommissionError::Spi)),
                }
            }
            Self::SettleWritePositionActual { actual } => {
                let actual = *actual;
                if backend
                    .write_register(foci_tmc4671::registers::PID_POSITION_ACTUAL, actual)
                    .await
                    .is_err()
                {
                    return Some(Err(CommissionError::Spi));
                }
                use foci_tmc4671::registers::{
                    PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
                    PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, Register,
                };
                let mut ops = [(Register::new(0), 0u32); 8];
                for (i, sub) in [
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                ]
                .into_iter()
                .enumerate()
                {
                    ops[i * 2] = (sub.addr, u32::from(sub.sub_addr));
                    ops[i * 2 + 1] = (sub.data, 0);
                }
                *self = Self::SettleClearIntegrators { ops, next: 0 };
                None
            }
            Self::SettleClearIntegrators { ops, next } => {
                let (reg, value) = ops[*next as usize];
                if backend.write_register(reg, value).await.is_err() {
                    return Some(Err(CommissionError::Spi));
                }
                *next += 1;
                if *next >= 8 {
                    *self = Self::SettleClearStatusFlags;
                }
                None
            }
            Self::SettleClearStatusFlags => {
                match crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await {
                    Ok(()) => {
                        *self = Self::FinalizeEnable;
                        None
                    }
                    Err(error) => Some(Err(error)),
                }
            }
            Self::FinalizeEnable => {
                if backend.interlock_tripped() {
                    return Some(Err(CommissionError::FaultInterlock));
                }
                let armed = critical_section::with(|_| {
                    if state.tmc.config_apply_pending.load(Relaxed) != 0 {
                        return false;
                    }
                    backend.set_enable_pin(true);
                    true
                });
                if !armed {
                    defmt::error!(
                        "arm_motor_electrical: configuration recorded after the drain, refusing arm"
                    );
                    return Some(Err(CommissionError::Spi));
                }
                *state.motor_enabled = true;
                backend.record_lifecycle(LifecycleFlag::MotorEnabled, true);
                Some(Ok(()))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::breakaway_campaign_engine;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp, fresh_state_locals};
    use crate::tmc_control::pending_config::PendingConfigItem;
    use foci_firmware_macros::fixture_test;

    fn dispatch_state<'a>(
        calibrated: &'a mut bool,
        motor_enabled: &'a mut bool,
        stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
    ) -> DispatchState<'a> {
        crate::state::tmc::tmc_channel0().mark_pi_representation_established();
        DispatchState::new(
            0,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
            calibrated,
            motor_enabled,
            stored_gains,
        )
    }

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[test]
    #[should_panic(expected = "calibrated=false")]
    fn arm_motor_electrical_panics_when_uncalibrated() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let (mut cal, mut en, mut gains) = fresh_state_locals();
            let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
            let _ = arm_motor_electrical(&mut backend, &mut state).await;
        });
    }

    #[test]
    #[should_panic(expected = "motor_enabled=false")]
    fn arm_motor_for_steps_panics_when_motor_disabled() {
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        arm_motor_for_steps(&mut state);
    }

    #[fixture_test]
    async fn arm_then_disarm_returns_to_baseline() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match");
        arm_motor_for_steps(&mut state);
        assert!(backend.enable_pin);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        disarm_motor(&mut backend, &mut state).await;
        assert!(!backend.enable_pin);
        assert!(!*state.calibrated);
        assert!(!*state.motor_enabled);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_drains_pending_config_before_energizing() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_CURRENT, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("a clean drain and matching sentinel should arm");

        let drained = backend
            .spi_log
            .iter()
            .position(|op| *op == SpiOp::DrainPendingConfig)
            .expect("arming must drain recorded configuration");
        let energized = backend
            .spi_log
            .iter()
            .position(|op| *op == SpiOp::EnablePin(true))
            .expect("a successful arm must assert the enable pin");
        assert!(
            drained < energized,
            "recorded configuration must reach the chip before it is energized"
        );
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_config_is_recorded_while_it_drains() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        // The config handlers run in the P2 USB dispatch task and preempt this
        // P1 arming path, so a bit can appear after the drain has already
        // observed the mask it will clear. That interleaving is ordinary
        // traffic, not a fault: the arm is refused, not panicked out of.
        backend.pending_config_set_during_drain = crate::state::tmc::PENDING_FILTERS;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "must not energize against configuration recorded after the drain"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            tmc_state().config_apply_pending.load(Ordering::Relaxed),
            crate::state::tmc::PENDING_FILTERS,
            "the late bit stays pending for the next control tick to drain"
        );

        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_config_lands_just_before_the_enable_pin() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        // A `tmc_set_current` landing this late records only the pending run
        // current, so the torque-flux limit this arm already wrote is the old
        // one -- energizing now runs the motor at a limit the host has just
        // replaced, possibly with a lower one.
        backend.pending_config_set_at_interlock_check = crate::state::tmc::PENDING_CURRENT;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "the pending check must sit after every awaiting step, not just after the drain"
        );
        assert!(!*state.motor_enabled);

        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_the_pending_config_drain_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_CURRENT, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_result = Err(PendingConfigItem::Current);
        backend.spi_log.clear();

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "must not energize while recorded configuration is unwritten"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            backend.spi_log.as_slice(),
            &[SpiOp::DrainPendingConfig],
            "a failed drain must return before the sentinel verify touches the chip"
        );
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_last_safe_state_pass_failed() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state()
            .safe_state_last_pass_failed
            .store(true, Ordering::Relaxed);
        let spi_log_len_before = backend.spi_log.len();

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::SafeStateIncomplete)
        );
        assert_eq!(
            backend.spi_log.len(),
            spi_log_len_before,
            "must not touch the chip at all while the last restore pass failed"
        );
        assert!(
            !backend.enable_pin,
            "must not enable while the last restore pass failed"
        );
        assert!(!*state.motor_enabled);

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_clears_incomplete_latch_only_after_sentinel_succeeds() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        // Simulate a clean restore pass: last_pass_failed clear, but the
        // compound latch still set because no sentinel pass has happened
        // since.
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
        tmc_state()
            .safe_state_incomplete
            .store(true, Ordering::Relaxed);

        // A sentinel mismatch must leave the compound latch set.
        backend.read_override = Some((
            foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(),
            0x0002_0032 ^ 1,
        ));
        let mismatched = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(
            mismatched,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(
            tmc_state().safe_state_incomplete.load(Ordering::Relaxed),
            "a failed sentinel check must not clear the compound latch"
        );
        backend.read_override = None;

        // A successful sentinel pass clears it.
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::new(0x1234).unwrap(),
            Ordering::Relaxed,
        );
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match once the override is removed");
        assert!(
            !tmc_state().safe_state_incomplete.load(Ordering::Relaxed),
            "a successful sentinel pass must clear the compound latch"
        );

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state()
            .safe_state_incomplete
            .store(false, Ordering::Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Ordering::Relaxed,
        );
        *state.motor_enabled = false;
    }

    #[fixture_test]
    async fn disarm_motor_clears_latched_velocity_clamp() {
        let mut backend = MockBackend::new();
        let (mut cal, mut en, mut gains) = fresh_state_locals();
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state().record_velocity_clamp_engagement();
        assert!(tmc_state().velocity_clamp_latched());

        disarm_motor(&mut backend, &mut state).await;

        assert!(!tmc_state().velocity_clamp_latched());
        assert_eq!(tmc_state().velocity_clamp_count(), 0);
    }

    #[fixture_test]
    async fn arm_motor_success_path() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel should match");
        assert!(backend.enable_pin);
        assert!(*state.motor_enabled);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        arm_motor_for_steps(&mut state);
        assert!(tmc_state().motor_armed.load(Ordering::Relaxed));
        tmc_state().motor_armed.store(false, Ordering::Relaxed);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_sentinel_mismatches() {
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        backend.read_override = Some((foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS.addr(), 0));
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(
            !backend.enable_pin,
            "enable pin must remain low on mismatch"
        );
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_when_pi_representation_mismatches() {
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().mark_pi_representation_established();
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.read_override = Some((
            foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .data
                .addr(),
            0,
        ));
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
        tmc_state().clear_pi_representation_established();
    }

    #[fixture_test]
    async fn arm_motor_electrical_succeeds_when_sentinel_matches() {
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        let run_limit = PidTorqueFluxLimit::new(0x1234).unwrap();
        tmc_state().store_current_limit(run_limit, core::sync::atomic::Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.spi_log.clear();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;
        assert_eq!(result, Ok(()));
        assert!(backend.enable_pin);
        let torque_limit_write_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, value)
                        if *addr == foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS.addr()
                            && *value == run_limit.register_value()
                )
            })
            .expect("arm should rewrite PID_TORQUE_FLUX_LIMITS from the stored run-current limit");
        let enable_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
                )
            })
            .expect("arm should assert enable");
        assert!(torque_limit_write_index < enable_index);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_clears_stale_stall_latch() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state().motor_sentinel.store(0x0002_0032, Relaxed);
        tmc_state().store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state().stall_latched.store(true, Relaxed);
        tmc_state().homing_clamp_active.store(true, Relaxed);
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("sentinel and run current should be valid");

        assert!(!tmc_state().stall_latched.load(Relaxed));
        assert!(!tmc_state().homing_clamp_active.load(Relaxed));
        tmc_state().motor_sentinel.store(0, Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_refuses_without_run_current() {
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().store_current_limit(
            PidTorqueFluxLimit::ZERO,
            core::sync::atomic::Ordering::Relaxed,
        );
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::InvalidCurrentLimit)
        );
        assert!(
            !backend.enable_pin,
            "enable pin must remain low without a configured run current"
        );
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_sets_hold_target_before_enable_in_position_mode() {
        use foci_tmc4671::registers::{
            MODE_RAMP_MODE_MOTION, MOTOR_TYPE_N_POLE_PAIRS, PID_POSITION_ACTUAL,
            PID_POSITION_TARGET, STATUS_FLAGS,
        };

        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, core::sync::atomic::Ordering::Relaxed);
        tmc_state().store_current_limit(
            PidTorqueFluxLimit::new(0x1234).unwrap(),
            core::sync::atomic::Ordering::Relaxed,
        );
        let _ = backend
            .write_register(MOTOR_TYPE_N_POLE_PAIRS, 0x0002_0032)
            .await;
        let _ = backend
            .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
            .await;
        let _ = backend.write_register(PID_POSITION_TARGET, 0).await;
        let _ = backend.write_register(PID_POSITION_ACTUAL, 11).await;
        backend.spi_log.clear();

        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(result, Ok(()));
        let enable_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::EnablePin(true)
                )
            })
            .expect("arm should assert enable");
        let target_rewrite_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, 11)
                        if *addr == PID_POSITION_ACTUAL.addr()
                )
            })
            .expect("arm should rewrite PID_POSITION_ACTUAL to observed actual");
        let status_clear_index = backend
            .spi_log
            .iter()
            .position(|op| {
                matches!(
                    op,
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, 0)
                        if *addr == STATUS_FLAGS.addr()
                )
            })
            .expect("arm should clear STATUS_FLAGS");
        assert!(target_rewrite_index < enable_index);
        assert!(status_clear_index < enable_index);
        tmc_state()
            .motor_sentinel
            .store(0, core::sync::atomic::Ordering::Relaxed);
    }

    #[fixture_test]
    async fn shutdown_safe_state_restores_phase_advance_and_disarms() {
        use core::sync::atomic::Ordering;

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        let complete = apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(complete);
        assert!(!runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
        assert_eq!(
            backend
                .registers
                .get(&foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            Some(&0x1111_2222)
        );
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(!runtime.is_calibrated());
        assert!(!runtime.is_motor_enabled());
    }

    #[fixture_test]
    async fn shutdown_safe_state_reports_incomplete_when_phase_restore_fails() {
        use core::sync::atomic::Ordering;

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.fail_on_write =
            Some(foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);

        let complete = apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(!complete);
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(!runtime.is_calibrated());
        assert!(!runtime.is_motor_enabled());
    }

    #[fixture_test]
    fn shutdown_breakaway_cancellation_completes_after_safe_state() {
        use core::sync::atomic::Ordering;

        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(breakaway_campaign_engine());
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        {
            let state = runtime.dispatch_state(
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            );
            *state.calibrated = true;
            *state.motor_enabled = true;
        }
        tmc_state().motor_armed.store(true, Ordering::Relaxed);
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        crate::diagnostic::begin_breakaway_campaign();
        crate::diagnostic::panic_after_breakaway_completion_for_test();

        let reset = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            pollster::block_on(apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                crate::state::tmc::tmc_channel0(),
                crate::state::stepper::stepper_channel0(),
            ));
        }));

        assert!(reset.is_err(), "completion must reach the reset injection");
        assert!(!runtime.phase_advance_active_for_test());
        assert!(!backend.enable_pin, "safe disarm must precede completion");
        assert!(!tmc_state().motor_armed.load(Ordering::Relaxed));
        assert!(
            runtime.is_commission_active(),
            "engine removal must follow completion"
        );
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignTerminalHandled,
                interrupted: false,
            })
        );

        assert!(pollster::block_on(apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )));
        assert!(!runtime.is_commission_active());
    }

    #[test]
    fn apply_shutdown_safe_state_lowers_gpio_before_phase_advance_write() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.enable_pin = true;

            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            runtime.set_phase_advance_for_test(true, 0, 0);

            apply_shutdown_safe_state(
                &mut runtime,
                &mut backend,
                tmc_state(),
                crate::state::stepper::stepper_channel0(),
            )
            .await;

            let mut enable_pin_false_index = None;
            let mut phase_advance_write_index = None;
            for (i, op) in backend.spi_log.iter().enumerate() {
                match op {
                    crate::commission_dispatch::test_support::SpiOp::EnablePin(false)
                        if enable_pin_false_index.is_none() =>
                    {
                        enable_pin_false_index = Some(i);
                    }
                    crate::commission_dispatch::test_support::SpiOp::Write(addr, _)
                        if *addr
                            == foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr() =>
                    {
                        phase_advance_write_index.get_or_insert(i);
                    }
                    _ => {}
                }
            }

            let enable_pin_false_index =
                enable_pin_false_index.expect("EnablePin(false) must be logged");
            let phase_advance_write_index = phase_advance_write_index
                .expect("phase-advance write must be logged when phase_advance_was_active");
            assert!(
                enable_pin_false_index < phase_advance_write_index,
                "GPIO must be lowered before the phase-advance SPI write: enable_pin_false_index={enable_pin_false_index}, phase_advance_write_index={phase_advance_write_index}"
            );
        });
    }

    #[test]
    fn apply_shutdown_safe_state_clears_homing_runtime() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
            let tmc = crate::state::tmc::tmc_channel0();
            let stepper = crate::state::stepper::stepper_channel0();
            use core::sync::atomic::Ordering::Relaxed;
            runtime.arm_stall_detector_for_test(100, 50, 2);
            tmc.homing_clamp_active.store(true, Relaxed);
            tmc.stall_latched.store(true, Relaxed);

            let _ = apply_shutdown_safe_state(&mut runtime, &mut backend, tmc, stepper).await;

            assert!(!runtime.stall_detector_armed_for_test());
            assert!(!tmc.homing_clamp_active.load(Relaxed));
            assert!(!tmc.stall_latched.load(Relaxed));

            tmc.homing_clamp_active.store(false, Relaxed);
            tmc.stall_latched.store(false, Relaxed);
        });
    }

    #[fixture_test]
    async fn shutdown_breakaway_cancellation_restore_failure_stays_active() {
        let mut runtime = crate::tmc_control::TmcControlRuntime::new_for_channel(0);
        runtime.set_commission_engine(breakaway_campaign_engine());
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.fail_on_write =
            Some(foci_tmc4671::registers::ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        backend.enable_pin = true;
        crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
        crate::diagnostic::begin_breakaway_campaign();

        let complete = apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(!complete);
        assert!(!runtime.is_commission_active());
        assert!(!backend.enable_pin, "failed restoration must still disarm");
        assert_eq!(
            crate::diagnostic::current_breakaway_breadcrumb_for_test(),
            Some(crate::diagnostic::BreakawayBreadcrumb {
                checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignInstalled,
                interrupted: true,
            })
        );
    }

    #[fixture_test]
    async fn arm_refuses_to_raise_the_enable_pin_when_the_interlock_is_tripped() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        state.tmc.motor_sentinel.store(0x0002_0032, Relaxed);
        state
            .tmc
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.interlock_tripped = true;

        let result = arm_motor_electrical(&mut backend, &mut state).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(!backend.enable_pin);
        assert!(!*state.motor_enabled);

        state.tmc.motor_sentinel.store(0, Relaxed);
    }

    #[fixture_test]
    async fn pre_arm_reset_does_not_clear_status_flags_when_the_interlock_is_tripped() {
        use foci_tmc4671::registers::{MODE_RAMP_MODE_MOTION, STATUS_FLAGS};

        let mut backend = MockBackend::new();
        backend
            .registers
            .insert(MODE_RAMP_MODE_MOTION.addr(), CLOSED_LOOP_POSITION_MODE)
            .unwrap();
        backend.interlock_tripped = true;

        let result = settle_closed_loop_hold_before_enable(&mut backend).await;

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(
            !backend.spi_log.iter().any(|op| matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(addr, _)
                    if *addr == STATUS_FLAGS.addr()
            )),
            "STATUS_FLAGS must not be written while the interlock is tripped"
        );
    }

    /// The pending-config check and the enable-pin write must share one
    /// critical section.
    ///
    /// The config handlers run in the P2 USB dispatch task, which preempts
    /// this P1 arming path. A plain check-then-act pair can therefore be
    /// split by an interrupt landing between the load and the GPIO write,
    /// and the arm would energize against configuration it had just
    /// established was absent. Masking interrupts across both is what closes
    /// that window.
    ///
    /// Guarded at the source because no host test can reproduce it: the fix
    /// works by making the interleaving unrepresentable, so a synchronous
    /// test has nothing left to interleave. What a test can still check is
    /// that the two halves have not drifted back apart.
    #[test]
    fn the_pending_config_check_and_the_enable_pin_share_one_critical_section() {
        let source = include_str!("lifecycle.rs");
        let source = source
            .split("#[cfg(test)]")
            .next()
            .expect("the module has a non-test half");
        let body = source
            .split("Self::FinalizeEnable => {")
            .nth(1)
            .expect("the arm step machine has a FinalizeEnable arm")
            .split("Some(Ok(()))")
            .next()
            .expect("the FinalizeEnable arm ends in success");

        let mask = body
            .find("critical_section::with")
            .expect("the arm decision must run with interrupts masked");
        let check = body
            .find("config_apply_pending.load(")
            .expect("the arm must check for configuration recorded after the drain");
        let energize = body
            .find("set_enable_pin(true)")
            .expect("the arm must raise the enable pin");

        assert!(
            mask < check && check < energize,
            "the critical section must open before the pending check, which \
             must precede the enable pin"
        );
        assert!(
            !body[mask..energize].contains(".await"),
            "an awaiting step between the check and the enable pin hands the \
             window back to the higher-priority config handlers"
        );
    }

    async fn arm_op_log(position_mode: bool) -> std::vec::Vec<SpiOp> {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        if position_mode {
            let _ = backend
                .write_register(
                    foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                    CLOSED_LOOP_POSITION_MODE,
                )
                .await;
        }
        backend.spi_log.clear();

        arm_motor_electrical(&mut backend, &mut state)
            .await
            .expect("arm must succeed against a healthy mock backend");
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        backend.spi_log
    }

    fn arm_prefix() -> [SpiOp; 6] {
        use foci_tmc4671::registers::{CONFIG_ADVANCED_PI_REPRESENT, PID_TORQUE_FLUX_LIMITS};
        [
            SpiOp::DrainPendingConfig,
            SpiOp::Read,
            SpiOp::Write(
                CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
            ),
            SpiOp::Read,
            SpiOp::Write(PID_TORQUE_FLUX_LIMITS.addr(), 0x1234),
            SpiOp::Read,
        ]
    }

    fn arm_suffix() -> [SpiOp; 2] {
        [
            SpiOp::EnablePin(true),
            SpiOp::Lifecycle {
                flag: LifecycleFlag::MotorEnabled,
                value: true,
            },
        ]
    }

    #[fixture_test]
    async fn arm_motor_electrical_op_log_not_in_position_mode() {
        let expected: std::vec::Vec<SpiOp> = arm_prefix().into_iter().chain(arm_suffix()).collect();

        assert_eq!(arm_op_log(false).await, expected);
    }

    #[fixture_test]
    async fn arm_motor_electrical_op_log_in_position_mode() {
        use foci_tmc4671::registers::{
            PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM,
            PID_ERROR_PID_TORQUE_ERROR_SUM, PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_POSITION_ACTUAL,
            STATUS_FLAGS,
        };
        let mut expected: std::vec::Vec<SpiOp> = arm_prefix().into();
        expected.push(SpiOp::Read);
        expected.push(SpiOp::Write(PID_POSITION_ACTUAL.addr(), 0));
        for sum in [
            PID_ERROR_PID_TORQUE_ERROR_SUM,
            PID_ERROR_PID_FLUX_ERROR_SUM,
            PID_ERROR_PID_VELOCITY_ERROR_SUM,
            PID_ERROR_PID_POSITION_ERROR_SUM,
        ] {
            expected.push(SpiOp::Write(sum.addr.addr(), u32::from(sum.sub_addr)));
            expected.push(SpiOp::Write(sum.data.addr(), 0));
        }
        expected.push(SpiOp::Write(STATUS_FLAGS.addr(), 0));
        expected.extend(arm_suffix());

        assert_eq!(arm_op_log(true).await, expected);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_advances_one_op_per_call_in_position_mode() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;

        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.spi_log.clear();

        let mut step = ArmMotorElectricalStep::start();
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend, &mut state).await;
            let raw_after = backend.spi_log[before..]
                .iter()
                .filter(|op| matches!(op, SpiOp::Write(_, _) | SpiOp::Read))
                .count();
            assert!(
                raw_after <= 1,
                "tick {}: advance() issued {} raw register op(s), expected at most 1",
                ticks,
                raw_after
            );
            ticks += 1;
            if result.is_some() {
                break;
            }
        }
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_last_safe_state_pass_failed() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .safe_state_last_pass_failed
            .store(true, Ordering::Relaxed);
        let spi_log_len_before = backend.spi_log.len();

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::SafeStateIncomplete)
        );
        assert_eq!(
            backend.spi_log.len(),
            spi_log_len_before,
            "must not touch the chip at all while the last restore pass failed"
        );
        assert!(!backend.enable_pin);
        tmc_state()
            .safe_state_last_pass_failed
            .store(false, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_the_pending_config_drain_fails() {
        use crate::tmc_control::pending_config::PendingConfigItem;
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        backend.pending_config_result = Err(PendingConfigItem::Current);

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        assert_eq!(
            backend.spi_log.as_slice(),
            &[SpiOp::DrainPendingConfig],
            "a failed drain must return before the sentinel verify touches the chip"
        );
        let _ = Ordering::Relaxed;
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_sentinel_mismatches() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032 ^ 1,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_without_run_current() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::InvalidCurrentLimit)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_pi_representation_not_established() {
        use core::sync::atomic::Ordering;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        tmc_state().clear_pi_representation_established();

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::ChipResetDetected)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().mark_pi_representation_established();
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_mode_read_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.fail_on_read = Some(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_position_read_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.fail_on_read = Some(foci_tmc4671::registers::PID_POSITION_ACTUAL.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_settle_position_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.fail_on_write = Some(foci_tmc4671::registers::PID_POSITION_ACTUAL.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_an_integrator_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.fail_on_write = Some(
            foci_tmc4671::registers::PID_ERROR_PID_TORQUE_ERROR_SUM
                .data
                .addr(),
        );

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_status_flags_write_fails() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.fail_on_write = Some(foci_tmc4671::registers::STATUS_FLAGS.addr());

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_interlock_tripped_during_status_clear() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;
        backend.interlock_tripped = true;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(
            !backend
                .spi_log
                .iter()
                .any(|op| *op == SpiOp::Write(foci_tmc4671::registers::STATUS_FLAGS.addr(), 0)),
            "an interlock-tripped status-clear must not write STATUS_FLAGS"
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_interlock_trips_only_after_status_clear() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MODE_RAMP_MODE_MOTION,
                CLOSED_LOOP_POSITION_MODE,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        loop {
            match step.advance(&mut backend, &mut state).await {
                None if matches!(step, ArmMotorElectricalStep::FinalizeEnable) => break,
                None => continue,
                Some(_) => unreachable!("must not complete before FinalizeEnable"),
            }
        }
        backend.interlock_tripped = true;

        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(
            result,
            Err(crate::autotune::CommissionError::FaultInterlock)
        );
        assert!(!backend.enable_pin);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_config_lands_just_before_the_enable_pin() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;
        backend.pending_config_set_at_interlock_check = crate::state::tmc::PENDING_CURRENT;

        let mut step = ArmMotorElectricalStep::start();
        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "the pending check must sit after every awaiting step, not just after the drain"
        );
        assert!(!*state.motor_enabled);
        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn arm_motor_electrical_step_refuses_when_config_recorded_between_polls() {
        use core::sync::atomic::Ordering;
        use foci_tmc4671::current_limit::PidTorqueFluxLimit;
        let mut backend = MockBackend::new();
        let (_, mut en, mut gains) = fresh_state_locals();
        let mut cal = true;
        let mut state = dispatch_state(&mut cal, &mut en, &mut gains);
        tmc_state()
            .motor_sentinel
            .store(0x0002_0032, Ordering::Relaxed);
        tmc_state()
            .store_current_limit(PidTorqueFluxLimit::new(0x1234).unwrap(), Ordering::Relaxed);
        let _ = backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                0x0002_0032,
            )
            .await;

        let mut step = ArmMotorElectricalStep::start();
        assert!(
            step.advance(&mut backend, &mut state).await.is_none(),
            "the Drain phase must not complete the arm in a single tick"
        );
        tmc_state()
            .config_apply_pending
            .store(crate::state::tmc::PENDING_FILTERS, Ordering::Relaxed);

        let result = loop {
            match step.advance(&mut backend, &mut state).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(crate::autotune::CommissionError::Spi));
        assert!(
            !backend.enable_pin,
            "configuration recorded between two poll_step calls must still refuse the arm"
        );
        assert!(!*state.motor_enabled);
        assert_eq!(
            tmc_state().config_apply_pending.load(Ordering::Relaxed),
            crate::state::tmc::PENDING_FILTERS,
            "the bit recorded between polls stays pending for the next control tick to drain"
        );

        tmc_state().motor_sentinel.store(0, Ordering::Relaxed);
        tmc_state().config_apply_pending.store(0, Ordering::Relaxed);
    }

    #[fixture_test]
    async fn drain_pending_config_worst_case_op_count_is_bounded() {
        use crate::tmc_control::pending_config::{PendingConfigSink, drain_pending_config};
        use crate::tmc_control::test_spi::RuntimeMockSpi;
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12;
        use foci_tmc4671::spi::Tmc4671Spi;

        const WORST_CASE_OP_COUNT: usize = 59;

        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state.config_apply_pending.store(
            crate::state::tmc::PENDING_VOLTAGE_LIMIT
                | crate::state::tmc::PENDING_CURRENT
                | crate::state::tmc::PENDING_ENCODER_DIR
                | crate::state::tmc::PENDING_INNER_GAINS
                | crate::state::tmc::PENDING_OUTER_GAINS
                | crate::state::tmc::PENDING_FILTERS,
            Relaxed,
        );
        tmc_state.voltage_limit_cfg.store(
            crate::state::tmc::MIN_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            Relaxed,
        );
        tmc_state.run_current_ma_cfg.store(1500, Relaxed);
        tmc_state.encoder_invert_cfg.store(Some(false), Relaxed);
        tmc_state.store_pending_inner_gains(
            crate::pid::InnerLoopGains {
                flux_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                flux_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
                torque_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                torque_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
            },
            Relaxed,
        );
        tmc_state.store_pending_outer_gains(
            crate::pid::OuterLoopGains {
                velocity_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                velocity_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
                position_p: foci_tmc4671::pid::PidPGain::new(100).unwrap(),
                position_i: foci_tmc4671::pid::PidIGain::new(100).unwrap(),
            },
            Relaxed,
        );
        tmc_state.velocity_filter_hz.store(Some(200), Relaxed);
        tmc_state.torque_filter_hz.store(Some(300), Relaxed);
        tmc_state.position_filter_hz.store(Some(400), Relaxed);
        tmc_state.flux_filter_hz.store(Some(500), Relaxed);
        tmc_state.mark_pi_representation_established();

        let mut stored_inner_gains = None;
        let raw_spi = RuntimeMockSpi::default()
            .with_read(ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12)
            .with_read(ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
        let mut tmc = Tmc4671Spi::new(raw_spi);
        drain_pending_config(
            &mut tmc,
            tmc_state,
            crate::config::TmcConfig {
                pwm_maxcnt: 3999,
                tmc_clock_hz: 25_000_000,
                bbm_h: 10,
                bbm_l: 10,
                dsadc_mdec_a: 331,
                dsadc_mdec_b: 331,
                adc_i_select: 0x1800_0100,
                excitation_voltage: 512,
                electrical_model_theta_e_us: 160,
                current_validation_flux_max_encoder_delta_counts: 4,
                current_validation_torque_max_encoder_delta_counts: 16,
                resistance_max_expected_current_count: 2000,
                resistance_scale_metadata_validated: true,
                current_scale_ma_per_lsb: 1.155,
            },
            PendingConfigSink {
                stored_inner_gains: &mut stored_inner_gains,
            },
        )
        .await
        .expect("a fully-populated, valid pending config must drain cleanly");

        let raw_spi = tmc.into_inner();
        let op_count = raw_spi.writes.len() + raw_spi.read_requests.len();
        assert_eq!(
            op_count, WORST_CASE_OP_COUNT,
            "drain_pending_config's worst-case op count changed -- if this is \
             an intentional new pending-config item or a wider filter set, \
             update WORST_CASE_OP_COUNT deliberately; if not, this is new, \
             uncapped growth in arm_motor_electrical's single Drain tick"
        );

        tmc_state.config_apply_pending.store(0, Relaxed);
    }
}
