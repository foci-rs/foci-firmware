//! Per-channel driver: one scheduler unit in, one bounded piece of TMC work out.

use core::sync::atomic::Ordering::Relaxed;

use crate::commission_dispatch::{CommissionRetention, PostActions, ReplyPayload};
use crate::commissioning::backend::{
    CommissionTmcSpiAccess, CommissionTmcStateAccess, CommissioningBackend,
};
use crate::commissioning::schedule::CommissionEvent;
use crate::state::tmc::TmcChannelState;
use crate::tmc_control::bootstrap::{TmcSafetyBootstrapStage, bootstrap_tmc_safety};
use crate::tmc_control::commission_poll::CommissionPollDriver;
use crate::tmc_control::config::{TmcControlConfig, TmcControlStateRefs};
use crate::tmc_control::scheduler::{ChannelMode, Unit, UnitClass};
use crate::tmc_control::state::{HomingMaintenanceResult, TmcControlRuntime};

/// TMC4671 `CHIPINFO_SI_TYPE` value for a genuine chip.
const EXPECTED_CHIP_ID: u32 = 0x3436_3731;

/// Startup steps, one per `Bootstrap` unit, in OpenFFBoard order.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum BootstrapStep {
    /// Read and verify `CHIPINFO_SI_TYPE`.
    ChipId,
    /// Establish the PI representation.
    PiRepresentation,
    /// Capture the ADC_VM zero offset (failure is a warning, not a park).
    AdcVmOffset,
    /// `ADC_VM_LIMITS`, `STATUS_MASK`, and the `STATUS_FLAGS` clear.
    SafetyBootstrap,
}

/// Why a channel stopped bootstrapping.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ParkReason {
    /// The chip-id read failed.
    SpiUnavailable,
    /// The chip answered with an unexpected id.
    ChipIdMismatch(u32),
    /// The PI representation bootstrap failed.
    PiRepresentation,
    /// The safety bootstrap failed at this stage.
    SafetyBootstrap(TmcSafetyBootstrapStage),
}

/// Driver-owned lifecycle; commissioning and restore are read from the runtime.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum DriverPhase {
    /// Startup in progress.
    Bootstrap(BootstrapStep),
    /// Normal operation.
    Ready,
    /// Startup failed; only audits run.
    Parked(ParkReason),
}

/// Result of one read-only `STATUS_FLAGS` audit.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum AuditResult {
    /// No flag of interest set.
    Clean,
    /// A limiter or PWM saturation flag while the motor is enabled.
    Warning {
        /// Raw `STATUS_FLAGS` value observed.
        flags: u32,
    },
    /// A critical flag while the pin is asserted (containment already due).
    Critical {
        /// Raw `STATUS_FLAGS` value observed.
        flags: u32,
    },
    /// A critical flag while the pin reads inactive: stuck or mis-wired pin.
    PinMismatch {
        /// Raw `STATUS_FLAGS` value observed.
        flags: u32,
    },
    /// The register read failed.
    ReadFailed,
}

/// How the board should schedule the next commission poll.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CommissionNext {
    /// Poll again as soon as the scheduler allows.
    PollNow,
    /// Let other classes run once before polling again.
    Yield,
    /// Poll at the absolute timestamp.
    WaitUntil(u64),
}

/// What one `drive` call produced. Data only; the board applies effects.
#[allow(clippy::large_enum_variant)]
pub enum UnitOutcome {
    /// Nothing to report.
    Idle,
    /// Bootstrap finished; the channel is `Ready`.
    Ready,
    /// Bootstrap failed; the channel is parked.
    Parked(ParkReason),
    /// A fast tick ran; re-arm the channel's fast grid with this period.
    Reschedule {
        /// New fast-grid period in microseconds.
        fast_period_us: u32,
    },
    /// This channel's maintenance share ran.
    Maintenance {
        /// Outcome of the channel's homing maintenance step.
        homing: HomingMaintenanceResult,
    },
    /// A status audit ran.
    Audit(AuditResult),
    /// A commission poll ran without reaching a terminal.
    Commission {
        /// Event for the board to emit, if any.
        event: Option<CommissionEvent>,
        /// How to schedule the next poll.
        next: CommissionNext,
    },
    /// The engine reached its terminal and the restore has completed.
    CommissionTerminal {
        /// Reply payload for the board to emit.
        reply: ReplyPayload,
        /// Post-actions the board executes after the reply.
        actions: PostActions,
    },
}

/// Per-call inputs the board supplies to `drive`.
pub struct DriveCtx<'a> {
    /// Monotonic microseconds at pick time.
    pub now_us: u64,
    /// Global homing state is nonzero.
    pub homing_active: bool,
    /// Homing state frozen for the current maintenance group.
    pub homing_snapshot: u8,
    /// Polled STATUS level for this channel.
    pub status_pin_asserted: bool,
    /// `STATUS_MASK` value for the safety bootstrap and the audit.
    pub status_mask: u32,
    /// Board constants.
    pub config: &'a TmcControlConfig,
    /// Stage-C retention slot.
    pub retention: CommissionRetention<'a>,
}

/// Per-channel state machine driven one unit at a time.
pub struct ChannelDriver {
    /// Board-agnostic channel runtime; request handling reaches it directly.
    pub runtime: TmcControlRuntime,
    phase: DriverPhase,
    // Consumed by the Commission arm landing in a later task.
    #[allow(dead_code)]
    poll_driver: CommissionPollDriver,
    reset_for_fast: bool,
    reset_for_maintenance: bool,
    last_command_velocity_reference: i32,
    /// Sequence number for commission trace engine events.
    pub commission_trace_engine_seq: u32,
    /// Homing state the last maintenance unit acted on, for tests only.
    #[cfg(test)]
    pub(crate) last_maintenance_homing_active: bool,
    /// Engine and phase currently sampled by the trace stream.
    #[cfg(feature = "trace")]
    pub commission_trace_phase: Option<(
        crate::commissioning::types::EngineKind,
        crate::autotune::PhaseId,
    )>,
}

/// `STATUS_FLAGS` bits that are limiter warnings, not faults.
pub const PID_LIMIT_MASK: u32 = 0x0000_FFFF;
/// `STATUS_FLAGS` PWM saturation bits.
pub const PWM_LIMIT_MASK: u32 = (1 << 24) | (1 << 25);

impl ChannelDriver {
    /// Create a driver for `channel`, starting at the first bootstrap step.
    pub fn new(channel: u8) -> Self {
        Self {
            runtime: TmcControlRuntime::new_for_channel(channel),
            phase: DriverPhase::Bootstrap(BootstrapStep::ChipId),
            poll_driver: CommissionPollDriver::new(),
            reset_for_fast: false,
            reset_for_maintenance: false,
            last_command_velocity_reference: 0,
            commission_trace_engine_seq: 0,
            #[cfg(test)]
            last_maintenance_homing_active: false,
            #[cfg(feature = "trace")]
            commission_trace_phase: None,
        }
    }

    /// Channel index.
    pub const fn channel(&self) -> u8 {
        self.runtime.channel()
    }

    /// Driver-owned phase.
    pub const fn phase(&self) -> DriverPhase {
        self.phase
    }

    /// Scheduler mode for this channel right now.
    pub fn mode(&self) -> ChannelMode {
        match self.phase {
            DriverPhase::Bootstrap(_) => ChannelMode::Bootstrap,
            DriverPhase::Parked(_) => ChannelMode::Parked,
            DriverPhase::Ready => {
                if self.runtime.is_restore_pending() {
                    ChannelMode::Restore
                } else if self.runtime.is_commission_active() {
                    ChannelMode::Commissioning
                } else {
                    ChannelMode::Ready
                }
            }
        }
    }

    /// Run one bounded unit of work.
    pub async fn drive<B>(&mut self, unit: Unit, backend: &mut B, ctx: DriveCtx<'_>) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        match (unit.class, self.phase) {
            (UnitClass::Bootstrap, DriverPhase::Bootstrap(step)) => {
                self.drive_bootstrap(step, backend, &ctx).await
            }
            (UnitClass::Bootstrap, _) => UnitOutcome::Idle,
            (_, DriverPhase::Bootstrap(_)) => UnitOutcome::Idle,
            (UnitClass::StatusAudit, _) => self.drive_audit(backend, &ctx).await,
            (_, DriverPhase::Parked(_)) => UnitOutcome::Idle,
            (UnitClass::Fast, DriverPhase::Ready) => self.drive_fast(backend, &ctx).await,
            (UnitClass::Maintenance, DriverPhase::Ready) => {
                self.drive_maintenance(backend, &ctx).await
            }
            (UnitClass::Commission | UnitClass::Restore, DriverPhase::Ready) => {
                self.drive_commission(backend, ctx).await
            }
            (UnitClass::MaintenanceFinalize, _) => UnitOutcome::Idle,
        }
    }

    async fn drive_bootstrap<B>(
        &mut self,
        step: BootstrapStep,
        backend: &mut B,
        ctx: &DriveCtx<'_>,
    ) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        let tmc_state = backend.tmc_state();
        let next = match step {
            BootstrapStep::ChipId => {
                match backend
                    .tmc_spi()
                    .read_sub_register(foci_tmc4671::registers::CHIPINFO_SI_TYPE)
                    .await
                {
                    Ok(EXPECTED_CHIP_ID) => {
                        defmt::info!("TMC4671 channel {} chip ID verified", self.channel());
                        BootstrapStep::PiRepresentation
                    }
                    Ok(other) => return self.park(ParkReason::ChipIdMismatch(other)),
                    Err(_) => return self.park(ParkReason::SpiUnavailable),
                }
            }
            BootstrapStep::PiRepresentation => {
                match crate::tmc_control::pi_representation::bootstrap_pi_representation(
                    backend, tmc_state,
                )
                .await
                {
                    Ok(()) => BootstrapStep::AdcVmOffset,
                    Err(error) => {
                        defmt::error!(
                            "TMC4671 channel {} PI representation failed at boot: {:?}",
                            self.channel(),
                            error
                        );
                        return self.park(ParkReason::PiRepresentation);
                    }
                }
            }
            BootstrapStep::AdcVmOffset => {
                match crate::adc_vm::capture_adc_vm_offset(backend.tmc_spi()).await {
                    Ok(capture) => {
                        tmc_state.store_adc_vm_offset(capture.offset_raw, capture.sample_count);
                    }
                    Err(()) => defmt::warn!(
                        "TMC4671 channel {} ADC_VM offset capture failed; host connect will reject VM decode",
                        self.channel()
                    ),
                }
                BootstrapStep::SafetyBootstrap
            }
            BootstrapStep::SafetyBootstrap => {
                match bootstrap_tmc_safety(backend, ctx.status_mask).await {
                    Ok(()) => {
                        self.phase = DriverPhase::Ready;
                        return UnitOutcome::Ready;
                    }
                    Err(failure) => return self.park(ParkReason::SafetyBootstrap(failure.stage)),
                }
            }
        };
        self.phase = DriverPhase::Bootstrap(next);
        UnitOutcome::Idle
    }

    fn park(&mut self, reason: ParkReason) -> UnitOutcome {
        defmt::error!("TMC4671 channel {} parked: {:?}", self.channel(), reason);
        self.phase = DriverPhase::Parked(reason);
        UnitOutcome::Parked(reason)
    }

    fn refs<'a>(&self, tmc_state: &'a TmcChannelState) -> TmcControlStateRefs<'a> {
        let stepper =
            crate::state::stepper::stepper_channel(self.channel()).expect("validated TMC channel");
        TmcControlStateRefs::new(tmc_state, stepper)
    }

    /// Consume the shared reset latch for one arm and hand it to the other.
    fn take_reset(&mut self, tmc_state: &TmcChannelState, fast: bool) -> bool {
        let latched = tmc_state.feedforward_reset.swap(false, Relaxed);
        if latched {
            if fast {
                self.reset_for_maintenance = true;
            } else {
                self.reset_for_fast = true;
            }
        }
        let own = if fast {
            core::mem::take(&mut self.reset_for_fast)
        } else {
            core::mem::take(&mut self.reset_for_maintenance)
        };
        latched || own
    }

    async fn drive_fast<B>(&mut self, backend: &mut B, ctx: &DriveCtx<'_>) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        let tmc_state = backend.tmc_state();
        let refs = self.refs(tmc_state);
        if tmc_state.config_apply_pending.load(Relaxed)
            && crate::tmc_control::filters::apply_pending_config(
                backend.tmc_spi(),
                tmc_state,
                ctx.config.tmc.pwm_maxcnt,
            )
            .await
            .is_err()
        {
            defmt::error!("pending TMC config apply failed");
            tmc_state.config_apply_pending.store(true, Relaxed);
        }
        let reset = self.take_reset(tmc_state, true);
        self.last_command_velocity_reference =
            self.runtime
                .command_velocity_reference(refs, ctx.homing_active, reset);
        if self
            .runtime
            .apply_feedforward_tick(
                backend.tmc_spi(),
                refs,
                ctx.config,
                ctx.homing_active,
                reset,
                false,
            )
            .await
            .is_err()
        {
            defmt::warn!("TMC feedforward tick failed");
        }
        UnitOutcome::Reschedule {
            fast_period_us: self.runtime.control_period_us(refs),
        }
    }

    async fn drive_maintenance<B>(&mut self, backend: &mut B, ctx: &DriveCtx<'_>) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        let tmc_state = backend.tmc_state();
        let refs = self.refs(tmc_state);
        let reset = self.take_reset(tmc_state, false);
        let homing_active = ctx.homing_snapshot != 0;
        #[cfg(test)]
        {
            self.last_maintenance_homing_active = homing_active;
        }
        if self
            .runtime
            .apply_feedforward_tick(
                backend.tmc_spi(),
                refs,
                ctx.config,
                homing_active,
                reset,
                true,
            )
            .await
            .is_err()
        {
            defmt::warn!("TMC slow feedforward tick failed");
        }
        self.runtime
            .apply_phase_and_position_tick(
                backend.tmc_spi(),
                refs,
                self.last_command_velocity_reference,
                homing_active,
                reset,
            )
            .await;
        let homing = self
            .runtime
            .apply_slow_channel_maintenance_tick(
                backend.tmc_spi(),
                refs,
                ctx.config,
                ctx.homing_snapshot,
            )
            .await;
        UnitOutcome::Maintenance { homing }
    }

    async fn drive_audit<B>(&mut self, backend: &mut B, ctx: &DriveCtx<'_>) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        let flags = match backend
            .tmc_spi()
            .read_register(foci_tmc4671::registers::STATUS_FLAGS)
            .await
        {
            Ok(flags) => flags,
            Err(_) => {
                defmt::error!(
                    "TMC4671 channel {} STATUS audit read failed",
                    self.channel()
                );
                return UnitOutcome::Audit(AuditResult::ReadFailed);
            }
        };
        let result = if flags & ctx.status_mask != 0 {
            if ctx.status_pin_asserted {
                AuditResult::Critical { flags }
            } else {
                defmt::error!(
                    "TMC4671 channel {} STATUS fault {:#010x} with inactive pin",
                    self.channel(),
                    flags
                );
                AuditResult::PinMismatch { flags }
            }
        } else if self.runtime.is_motor_enabled() && flags & (PID_LIMIT_MASK | PWM_LIMIT_MASK) != 0
        {
            defmt::warn!("TMC4671 channel {} STATUS: {:#010x}", self.channel(), flags);
            AuditResult::Warning { flags }
        } else {
            AuditResult::Clean
        };
        UnitOutcome::Audit(result)
    }

    async fn drive_commission<B>(&mut self, _backend: &mut B, _ctx: DriveCtx<'_>) -> UnitOutcome
    where
        B: CommissioningBackend + CommissionTmcSpiAccess + CommissionTmcStateAccess,
    {
        UnitOutcome::Idle
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::CommissionRetention;
    use crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot;
    use crate::tmc_control::TmcControlConfig;
    use crate::tmc_control::scheduler::{ChannelMode, Unit, UnitClass};
    use crate::tmc_control::test_spi::{RuntimeMockSpi, TEST_CONFIG, TestBackend, TestPin};
    use foci_firmware_macros::fixture_test;
    use foci_tmc4671::registers::{STATUS_FLAGS, STATUS_MASK};
    use foci_tmc4671::spi::Tmc4671Spi;

    const CHIP_ID: u32 = 0x3436_3731;

    fn config() -> TmcControlConfig {
        TmcControlConfig::new(168_000_000, 25_000, 1.0, TEST_CONFIG)
    }

    fn unit(class: UnitClass) -> Unit {
        Unit {
            class,
            channel: 0,
            deadline_us: 0,
        }
    }

    struct Rig {
        driver: ChannelDriver,
        tmc: Tmc4671Spi<RuntimeMockSpi>,
        pin: TestPin,
        slot: IntegralRetentionSlot,
        config: TmcControlConfig,
    }

    impl Rig {
        fn new(spi: RuntimeMockSpi) -> Self {
            Self {
                driver: ChannelDriver::new(0),
                tmc: Tmc4671Spi::new(spi),
                pin: TestPin::default(),
                slot: IntegralRetentionSlot::new(),
                config: config(),
            }
        }

        async fn drive(&mut self, class: UnitClass) -> UnitOutcome {
            self.drive_with(class, false, false, 0).await
        }

        async fn drive_with_pin(
            &mut self,
            class: UnitClass,
            status_pin_asserted: bool,
        ) -> UnitOutcome {
            self.drive_with(class, status_pin_asserted, false, 0).await
        }

        async fn drive_with_homing(
            &mut self,
            class: UnitClass,
            homing_active: bool,
            homing_snapshot: u8,
        ) -> UnitOutcome {
            self.drive_with(class, false, homing_active, homing_snapshot)
                .await
        }

        async fn drive_with(
            &mut self,
            class: UnitClass,
            status_pin_asserted: bool,
            homing_active: bool,
            homing_snapshot: u8,
        ) -> UnitOutcome {
            let mut backend = TestBackend::new(
                0,
                &mut self.tmc,
                &mut self.pin,
                crate::state::tmc::tmc_channel0(),
            );
            let ctx = DriveCtx {
                now_us: 0,
                homing_active,
                homing_snapshot,
                status_pin_asserted,
                status_mask: 1 << 26,
                config: &self.config,
                retention: CommissionRetention::new(&mut self.slot),
            };
            self.driver.drive(unit(class), &mut backend, ctx).await
        }

        fn spi(&self) -> &RuntimeMockSpi {
            self.tmc.inner()
        }
    }

    /// `CHIPINFO_SI_TYPE` selects sub-register 0 of ADDR register 0x01; the
    /// value is read back from DATA register 0x00 (`SubRegister::new` takes
    /// `(data_addr, addr_addr, sub_addr)`, so `CHIPINFO_SI_TYPE.data.addr()`
    /// is 0x00 -- that's the address `read_sub_register` actually reads).
    const CHIPINFO_DATA_ADDR: u8 = 0x00;

    #[fixture_test]
    async fn new_driver_starts_in_bootstrap_chip_id() {
        let driver = ChannelDriver::new(0);
        assert_eq!(
            driver.phase(),
            DriverPhase::Bootstrap(BootstrapStep::ChipId)
        );
        assert_eq!(driver.mode(), ChannelMode::Bootstrap);
    }

    #[fixture_test]
    async fn bootstrap_advances_one_step_per_drive_in_openffboard_order() {
        // Reads: chip id, then whatever PI bootstrap, ADC_VM capture and the
        // safety bootstrap read back. Zeros satisfy every verified write.
        let spi = RuntimeMockSpi::default().echo_writes().with_read(CHIP_ID);
        let mut rig = Rig::new(spi);

        assert!(matches!(
            rig.drive(UnitClass::Bootstrap).await,
            UnitOutcome::Idle
        ));
        assert_eq!(
            rig.driver.phase(),
            DriverPhase::Bootstrap(BootstrapStep::PiRepresentation)
        );
        assert!(rig.spi().read_requests.contains(&CHIPINFO_DATA_ADDR));

        assert!(matches!(
            rig.drive(UnitClass::Bootstrap).await,
            UnitOutcome::Idle
        ));
        assert_eq!(
            rig.driver.phase(),
            DriverPhase::Bootstrap(BootstrapStep::AdcVmOffset)
        );

        assert!(matches!(
            rig.drive(UnitClass::Bootstrap).await,
            UnitOutcome::Idle
        ));
        assert_eq!(
            rig.driver.phase(),
            DriverPhase::Bootstrap(BootstrapStep::SafetyBootstrap)
        );

        assert!(matches!(
            rig.drive(UnitClass::Bootstrap).await,
            UnitOutcome::Ready
        ));
        assert_eq!(rig.driver.phase(), DriverPhase::Ready);
        assert_eq!(rig.driver.mode(), ChannelMode::Ready);
        assert!(rig.spi().attempted_write_to(STATUS_MASK));
        assert!(rig.spi().attempted_write_to(STATUS_FLAGS));
        assert!(!rig.pin.high, "bootstrap must never raise the enable pin");
    }

    #[fixture_test]
    async fn bootstrap_parks_on_chip_id_mismatch_and_stays_parked() {
        let spi = RuntimeMockSpi::default().with_read(0xDEAD_BEEF);
        let mut rig = Rig::new(spi);

        let outcome = rig.drive(UnitClass::Bootstrap).await;

        assert!(matches!(
            outcome,
            UnitOutcome::Parked(ParkReason::ChipIdMismatch(0xDEAD_BEEF))
        ));
        assert_eq!(rig.driver.mode(), ChannelMode::Parked);
        assert!(matches!(
            rig.drive(UnitClass::Bootstrap).await,
            UnitOutcome::Idle
        ));
        assert_eq!(rig.driver.mode(), ChannelMode::Parked);
        assert!(!rig.spi().attempted_write_to(STATUS_MASK));
    }

    #[fixture_test]
    async fn bootstrap_parks_when_the_safety_bootstrap_fails() {
        let spi = RuntimeMockSpi::default()
            .echo_writes()
            .with_read(CHIP_ID)
            .fail_writes_to(STATUS_MASK.addr());
        let mut rig = Rig::new(spi);
        for _ in 0..3 {
            let _ = rig.drive(UnitClass::Bootstrap).await;
        }

        let outcome = rig.drive(UnitClass::Bootstrap).await;

        assert!(matches!(
            outcome,
            UnitOutcome::Parked(ParkReason::SafetyBootstrap(
                crate::tmc_control::TmcSafetyBootstrapStage::StatusMaskWrite
            ))
        ));
        assert!(!rig.pin.high);
    }

    #[fixture_test]
    async fn bootstrap_writes_no_motion_mode_pwm_or_current_limit_register() {
        use foci_tmc4671::registers::{
            MODE_RAMP_MODE_MOTION, PID_TORQUE_FLUX_LIMITS, PIDOUT_UQ_UD_LIMITS, PWM_POLARITIES,
            PWM_SV_CHOP,
        };
        let mut rig = Rig::new(RuntimeMockSpi::default().echo_writes().with_read(CHIP_ID));
        for _ in 0..4 {
            let _ = rig.drive(UnitClass::Bootstrap).await;
        }
        assert_eq!(rig.driver.phase(), DriverPhase::Ready);
        for reg in [
            MODE_RAMP_MODE_MOTION,
            PID_TORQUE_FLUX_LIMITS,
            PIDOUT_UQ_UD_LIMITS,
            PWM_POLARITIES,
            PWM_SV_CHOP,
        ] {
            assert!(
                !rig.spi().attempted_write_to(reg),
                "{:?} written during bootstrap",
                reg
            );
        }
    }

    #[fixture_test]
    async fn non_bootstrap_units_are_idle_while_bootstrapping() {
        let mut rig = Rig::new(RuntimeMockSpi::default());
        assert!(matches!(
            rig.drive(UnitClass::Fast).await,
            UnitOutcome::Idle
        ));
        assert!(matches!(
            rig.drive(UnitClass::Maintenance).await,
            UnitOutcome::Idle
        ));
        assert!(rig.spi().writes.is_empty());
    }

    fn ready_rig(spi: RuntimeMockSpi) -> Rig {
        let mut rig = Rig::new(spi);
        rig.driver.phase = DriverPhase::Ready;
        rig
    }

    #[fixture_test]
    async fn fast_unit_applies_pending_config_before_feedforward_and_reschedules() {
        use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;
        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state
            .config_apply_pending
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut rig = ready_rig(RuntimeMockSpi::default());

        let outcome = rig.drive(UnitClass::Fast).await;

        let UnitOutcome::Reschedule { fast_period_us } = outcome else {
            panic!("fast unit must reschedule");
        };
        assert_eq!(
            fast_period_us,
            rig.driver
                .runtime
                .control_period_us(crate::tmc_control::TmcControlStateRefs::new(
                    tmc_state,
                    crate::state::stepper::stepper_channel0(),
                ))
        );
        assert!(
            !tmc_state
                .config_apply_pending
                .load(core::sync::atomic::Ordering::Relaxed)
        );
        // apply_pending_config's first write is the voltage limit; it must
        // precede any feedforward write in the log.
        let writes = &rig.spi().writes;
        assert!(
            writes
                .first()
                .is_some_and(|(addr, _)| *addr == PIDOUT_UQ_UD_LIMITS.addr())
        );
    }

    #[fixture_test]
    async fn failed_pending_config_apply_leaves_the_flag_set_for_retry() {
        use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;
        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state
            .config_apply_pending
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut rig =
            ready_rig(RuntimeMockSpi::default().fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr()));

        let outcome = rig.drive(UnitClass::Fast).await;

        assert!(matches!(outcome, UnitOutcome::Reschedule { .. }));
        assert!(
            tmc_state
                .config_apply_pending
                .load(core::sync::atomic::Ordering::Relaxed)
        );
    }

    #[fixture_test]
    async fn reset_latch_reaches_maintenance_after_a_fast_unit_consumed_it() {
        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state
            .feedforward_reset
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut rig = ready_rig(RuntimeMockSpi::default().echo_writes());

        let _ = rig.drive(UnitClass::Fast).await;
        assert!(
            !tmc_state
                .feedforward_reset
                .load(core::sync::atomic::Ordering::Relaxed)
        );
        assert!(rig.driver.reset_for_maintenance);
        assert!(!rig.driver.reset_for_fast);

        let _ = rig.drive(UnitClass::Maintenance).await;
        assert!(!rig.driver.reset_for_maintenance);
        assert!(!rig.driver.reset_for_fast);
    }

    #[fixture_test]
    async fn reset_latch_reaches_fast_after_a_maintenance_unit_consumed_it() {
        let tmc_state = crate::state::tmc::tmc_channel0();
        tmc_state
            .feedforward_reset
            .store(true, core::sync::atomic::Ordering::Relaxed);
        let mut rig = ready_rig(RuntimeMockSpi::default().echo_writes());

        let _ = rig.drive(UnitClass::Maintenance).await;
        assert!(
            !tmc_state
                .feedforward_reset
                .load(core::sync::atomic::Ordering::Relaxed)
        );
        assert!(rig.driver.reset_for_fast);
        assert!(!rig.driver.reset_for_maintenance);

        let _ = rig.drive(UnitClass::Fast).await;
        assert!(!rig.driver.reset_for_fast);
        assert!(!rig.driver.reset_for_maintenance);
    }

    #[fixture_test]
    async fn maintenance_uses_the_frozen_homing_snapshot_not_the_live_flag() {
        let mut rig = ready_rig(RuntimeMockSpi::default().echo_writes());
        // Live flag says homing, snapshot says not: maintenance must follow the snapshot.
        let outcome = rig.drive_with_homing(UnitClass::Maintenance, true, 0).await;
        assert!(matches!(outcome, UnitOutcome::Maintenance { .. }));
        assert!(!rig.driver.last_maintenance_homing_active);
        let outcome = rig
            .drive_with_homing(UnitClass::Maintenance, false, 2)
            .await;
        assert!(matches!(outcome, UnitOutcome::Maintenance { .. }));
        assert!(rig.driver.last_maintenance_homing_active);
    }

    #[fixture_test]
    async fn maintenance_unit_reports_the_homing_result_for_the_snapshot() {
        let mut rig = ready_rig(RuntimeMockSpi::default().echo_writes());
        let outcome = rig.drive(UnitClass::Maintenance).await;
        assert!(matches!(
            outcome,
            UnitOutcome::Maintenance {
                homing: HomingMaintenanceResult::ReadyToAdvance
            }
        ));
    }

    #[fixture_test]
    async fn audit_reads_status_flags_and_never_writes_them() {
        let mut rig = ready_rig(RuntimeMockSpi::default().with_read(0));
        let outcome = rig.drive(UnitClass::StatusAudit).await;
        assert!(matches!(outcome, UnitOutcome::Audit(AuditResult::Clean)));
        assert!(rig.spi().read_requested(STATUS_FLAGS));
        assert!(!rig.spi().attempted_write_to(STATUS_FLAGS));
    }

    #[fixture_test]
    async fn audit_reports_pin_mismatch_when_a_critical_flag_is_set_but_the_pin_is_inactive() {
        let mut rig = ready_rig(RuntimeMockSpi::default().with_read(1 << 26));
        let outcome = rig.drive(UnitClass::StatusAudit).await;
        assert!(matches!(
            outcome,
            UnitOutcome::Audit(AuditResult::PinMismatch { flags }) if flags == 1 << 26
        ));
    }

    #[fixture_test]
    async fn audit_reports_critical_when_the_pin_is_asserted_too() {
        let mut rig = ready_rig(RuntimeMockSpi::default().with_read(1 << 26));
        let outcome = rig.drive_with_pin(UnitClass::StatusAudit, true).await;
        assert!(matches!(
            outcome,
            UnitOutcome::Audit(AuditResult::Critical { .. })
        ));
    }

    #[fixture_test]
    async fn audit_reports_read_failure() {
        let mut rig = ready_rig(RuntimeMockSpi::default().fail_reads());
        let outcome = rig.drive(UnitClass::StatusAudit).await;
        assert!(matches!(
            outcome,
            UnitOutcome::Audit(AuditResult::ReadFailed)
        ));
    }

    #[fixture_test]
    async fn audit_runs_for_a_parked_channel() {
        let mut rig = Rig::new(
            RuntimeMockSpi::default()
                .echo_writes()
                .with_read(0xDEAD_BEEF)
                .with_read(0),
        );
        let _ = rig.drive(UnitClass::Bootstrap).await;
        assert_eq!(rig.driver.mode(), ChannelMode::Parked);
        assert!(matches!(
            rig.drive(UnitClass::StatusAudit).await,
            UnitOutcome::Audit(AuditResult::Clean)
        ));
    }
}
