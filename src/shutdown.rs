//! Shutdown-reason enumeration shared across every FOCI firmware.
//!
//! # Why an enum?
//!
//! The `klipper_shutdown!` / `klipper_shutdown_from!` macros produced by
//! ankyra expand to a path of the form
//! `crate::_ankyra_config::static_strings::__ANKYRA_SS_<hash>`. That path
//! only resolves inside the crate that invokes `ankyra_config!` — in this
//! workspace that is the board binary (e.g. `openffboard-fw`). Handler
//! bodies lifted into `foci-firmware` cannot reach
//! `crate::_ankyra_config` and therefore cannot call the shutdown macros
//! directly.
//!
//! The indirection is this module's [`ShutdownReason`] enum: lifted
//! handlers pass a variant to
//! [`ShutdownRecovery::send_shutdown`](crate::context::ShutdownRecovery::send_shutdown),
//! and the board-side impl maps each variant to the corresponding
//! `klipper_shutdown_from!` call via the `dispatch_shutdown!` macro.
//!
//! # Adding a new shutdown reason
//!
//! To add a shutdown reason, edit **all three** parallel lists below:
//!
//! 1. The [`ShutdownReason`] enum variant.
//! 2. The [`ShutdownReason::message`] match arm (same literal).
//! 3. The `dispatch_shutdown!` match arm (same variant + same literal).
//! 4. The `foci_ankyra_config!` `static_strings` list entry.
//!
//! All four lists are in this single file so a reviewer can check them
//! at a glance. The literal strings must be byte-identical; they are
//! FNV-1a-hashed by ankyra and the build fails loudly on any mismatch.

/// Every shutdown reason a FOCI firmware can emit from a lifted handler.
///
/// Cross-board: every binary that pulls in `foci-firmware` accepts the
/// same set of reasons. A board that needs a board-specific literal
/// still has the full `::ankyra::klipper_shutdown!` form available from
/// its own handlers; this enum is only the bridge used by lifted
/// handlers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShutdownReason {
    ShutdownClearedWhenNotShutdown,
    ConfigResetOnlyAvailableWhenShutdown,
    CommandRequest,
    ConfigDigitalOutFailed,
    MotorEnableDisableLostQueueFull,
    QueueDigitalOutFailed,
    UpdateDigitalOutFailed,
    ConfigStepperFailed,
    UnknownStepperOid,
    InvalidStepperChannel,
    QueueStepFailed,
    StepperTooFarInPast,
    CannotResetTimeWhenStepperActive,
    StepperStopOnTriggerUnknownStepper,
    StepperStopOnTriggerFailed,
    ConfigEndstopFailed,
    EndstopHomeFailed,
    EndstopQueryStateFailed,
    ConfigTrsyncFailed,
    TrsyncStartFailed,
    TrsyncSetTimeoutFailed,
    TrsyncTriggerFailed,
    InvalidTmcChannel,
    TmcSetCurrentLostQueueFull,
    TmcSetMotionScaleLostQueueFull,
    TmcSetEncoderDirLostQueueFull,
    TmcSetPidGainsLostQueueFull,
    TmcSetPositionGainsLostQueueFull,
    TmcSetFilterLostQueueFull,
    TmcSetVoltageLimitLostQueueFull,
    TmcDiagnosticLostQueueFull,
    Tmc4671ChipIdMismatch,
    Tmc4671SpiFailedAtBoot,
    HostClockSyncExpired,
    TmcBootstrapCh0AdcVmLimitsWriteFailed,
    TmcBootstrapCh0StatusMaskWriteFailed,
    TmcBootstrapCh0StatusFlagsReadFailed,
    TmcBootstrapCh0StatusFlagsClearFailed,
    TmcBootstrapCh1AdcVmLimitsWriteFailed,
    TmcBootstrapCh1StatusMaskWriteFailed,
    TmcBootstrapCh1StatusFlagsReadFailed,
    TmcBootstrapCh1StatusFlagsClearFailed,
    TmcRequestValidationFailed,
    Tmc4671PllNotLocked,
    Tmc4671AdcCurrentClipped,
    Tmc4671Fault,
}

impl ShutdownReason {
    /// Static-string literal for this reason. Matches exactly one entry
    /// in the binary's `ankyra_config! { static_strings = [...] }` list
    /// (auto-included via `foci_ankyra_config!`).
    pub const fn message(self) -> &'static str {
        match self {
            Self::ShutdownClearedWhenNotShutdown => "Shutdown cleared when not shutdown",
            Self::ConfigResetOnlyAvailableWhenShutdown => {
                "config_reset only available when shutdown"
            }
            Self::CommandRequest => "Command request",
            Self::ConfigDigitalOutFailed => "config_digital_out failed",
            Self::MotorEnableDisableLostQueueFull => "Motor enable/disable lost (queue full)",
            Self::QueueDigitalOutFailed => "queue_digital_out failed",
            Self::UpdateDigitalOutFailed => "update_digital_out failed",
            Self::ConfigStepperFailed => "config_stepper failed",
            Self::UnknownStepperOid => "Unknown stepper OID",
            Self::InvalidStepperChannel => "Invalid stepper channel",
            Self::QueueStepFailed => "queue_step failed",
            Self::StepperTooFarInPast => "Stepper too far in past",
            Self::CannotResetTimeWhenStepperActive => "Can't reset time when stepper active",
            Self::StepperStopOnTriggerUnknownStepper => "stepper_stop_on_trigger: unknown stepper",
            Self::StepperStopOnTriggerFailed => "stepper_stop_on_trigger failed",
            Self::ConfigEndstopFailed => "config_endstop failed",
            Self::EndstopHomeFailed => "endstop_home failed",
            Self::EndstopQueryStateFailed => "endstop_query_state failed",
            Self::ConfigTrsyncFailed => "config_trsync failed",
            Self::TrsyncStartFailed => "trsync_start failed",
            Self::TrsyncSetTimeoutFailed => "trsync_set_timeout failed",
            Self::TrsyncTriggerFailed => "trsync_trigger failed",
            Self::InvalidTmcChannel => "Invalid TMC channel",
            Self::TmcSetCurrentLostQueueFull => "tmc_set_current lost (queue full)",
            Self::TmcSetMotionScaleLostQueueFull => "tmc_set_motion_scale lost (queue full)",
            Self::TmcSetEncoderDirLostQueueFull => "tmc_set_encoder_dir lost (queue full)",
            Self::TmcSetPidGainsLostQueueFull => "tmc_set_pid_gains lost (queue full)",
            Self::TmcSetPositionGainsLostQueueFull => "tmc_set_position_gains lost (queue full)",
            Self::TmcSetFilterLostQueueFull => "tmc_set_filter lost (queue full)",
            Self::TmcSetVoltageLimitLostQueueFull => "tmc_set_voltage_limit lost (queue full)",
            Self::TmcDiagnosticLostQueueFull => "TMC diagnostic lost (queue full)",
            Self::Tmc4671ChipIdMismatch => "TMC4671 chip ID mismatch",
            Self::Tmc4671SpiFailedAtBoot => "TMC4671 SPI failed at boot",
            Self::HostClockSyncExpired => "Host clock-sync expired",
            Self::TmcBootstrapCh0AdcVmLimitsWriteFailed => {
                "TMC4671 ch0 bootstrap: ADC_VM_LIMITS write failed"
            }
            Self::TmcBootstrapCh0StatusMaskWriteFailed => {
                "TMC4671 ch0 bootstrap: STATUS_MASK write failed"
            }
            Self::TmcBootstrapCh0StatusFlagsReadFailed => {
                "TMC4671 ch0 bootstrap: STATUS_FLAGS read failed"
            }
            Self::TmcBootstrapCh0StatusFlagsClearFailed => {
                "TMC4671 ch0 bootstrap: STATUS_FLAGS clear failed"
            }
            Self::TmcBootstrapCh1AdcVmLimitsWriteFailed => {
                "TMC4671 ch1 bootstrap: ADC_VM_LIMITS write failed"
            }
            Self::TmcBootstrapCh1StatusMaskWriteFailed => {
                "TMC4671 ch1 bootstrap: STATUS_MASK write failed"
            }
            Self::TmcBootstrapCh1StatusFlagsReadFailed => {
                "TMC4671 ch1 bootstrap: STATUS_FLAGS read failed"
            }
            Self::TmcBootstrapCh1StatusFlagsClearFailed => {
                "TMC4671 ch1 bootstrap: STATUS_FLAGS clear failed"
            }
            Self::TmcRequestValidationFailed => "TMC request validation failed",
            Self::Tmc4671PllNotLocked => "TMC4671: PLL not locked (clock lost)",
            Self::Tmc4671AdcCurrentClipped => "TMC4671: ADC current clipped (overcurrent)",
            Self::Tmc4671Fault => "TMC4671 fault",
        }
    }
}

/// Dispatch a [`ShutdownReason`] to `::ankyra::klipper_shutdown_from!`.
///
/// Invoked by the board's `ShutdownRecovery::send_shutdown` impl:
///
/// ```ignore
/// impl ShutdownRecovery for FirmwareCtx<'_> {
///     fn send_shutdown(&mut self, reason: ShutdownReason, clock: u32) {
///         foci_firmware::dispatch_shutdown!(
///             reason, clock, &mut crate::_ankyra_config::Sender
///         );
///     }
/// }
/// ```
///
/// Each match arm is a `klipper_shutdown_from!(sender, "literal", clock)`
/// call that ankyra hashes at build time. The literal list here must
/// match [`ShutdownReason::message`] above and the `static_strings`
/// list in `foci_ankyra_config!`.
#[macro_export]
macro_rules! dispatch_shutdown {
    ($reason:expr, $clock:expr, $sender:expr) => {
        match $reason {
            $crate::shutdown::ShutdownReason::ShutdownClearedWhenNotShutdown => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "Shutdown cleared when not shutdown",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::ConfigResetOnlyAvailableWhenShutdown => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "config_reset only available when shutdown",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::CommandRequest => {
                ::ankyra::klipper_shutdown_from!($sender, "Command request", $clock);
            }
            $crate::shutdown::ShutdownReason::ConfigDigitalOutFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "config_digital_out failed", $clock);
            }
            $crate::shutdown::ShutdownReason::MotorEnableDisableLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "Motor enable/disable lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::QueueDigitalOutFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "queue_digital_out failed", $clock);
            }
            $crate::shutdown::ShutdownReason::UpdateDigitalOutFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "update_digital_out failed", $clock);
            }
            $crate::shutdown::ShutdownReason::ConfigStepperFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "config_stepper failed", $clock);
            }
            $crate::shutdown::ShutdownReason::UnknownStepperOid => {
                ::ankyra::klipper_shutdown_from!($sender, "Unknown stepper OID", $clock);
            }
            $crate::shutdown::ShutdownReason::InvalidStepperChannel => {
                ::ankyra::klipper_shutdown_from!($sender, "Invalid stepper channel", $clock);
            }
            $crate::shutdown::ShutdownReason::QueueStepFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "queue_step failed", $clock);
            }
            $crate::shutdown::ShutdownReason::StepperTooFarInPast => {
                ::ankyra::klipper_shutdown_from!($sender, "Stepper too far in past", $clock);
            }
            $crate::shutdown::ShutdownReason::CannotResetTimeWhenStepperActive => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "Can't reset time when stepper active",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::StepperStopOnTriggerUnknownStepper => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "stepper_stop_on_trigger: unknown stepper",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::StepperStopOnTriggerFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "stepper_stop_on_trigger failed", $clock);
            }
            $crate::shutdown::ShutdownReason::ConfigEndstopFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "config_endstop failed", $clock);
            }
            $crate::shutdown::ShutdownReason::EndstopHomeFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "endstop_home failed", $clock);
            }
            $crate::shutdown::ShutdownReason::EndstopQueryStateFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "endstop_query_state failed", $clock);
            }
            $crate::shutdown::ShutdownReason::ConfigTrsyncFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "config_trsync failed", $clock);
            }
            $crate::shutdown::ShutdownReason::TrsyncStartFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "trsync_start failed", $clock);
            }
            $crate::shutdown::ShutdownReason::TrsyncSetTimeoutFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "trsync_set_timeout failed", $clock);
            }
            $crate::shutdown::ShutdownReason::TrsyncTriggerFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "trsync_trigger failed", $clock);
            }
            $crate::shutdown::ShutdownReason::InvalidTmcChannel => {
                ::ankyra::klipper_shutdown_from!($sender, "Invalid TMC channel", $clock);
            }
            $crate::shutdown::ShutdownReason::TmcSetCurrentLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_current lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetMotionScaleLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_motion_scale lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetEncoderDirLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_encoder_dir lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetPidGainsLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_pid_gains lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetPositionGainsLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_position_gains lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetFilterLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_filter lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcSetVoltageLimitLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "tmc_set_voltage_limit lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcDiagnosticLostQueueFull => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC diagnostic lost (queue full)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::Tmc4671ChipIdMismatch => {
                ::ankyra::klipper_shutdown_from!($sender, "TMC4671 chip ID mismatch", $clock);
            }
            $crate::shutdown::ShutdownReason::Tmc4671SpiFailedAtBoot => {
                ::ankyra::klipper_shutdown_from!($sender, "TMC4671 SPI failed at boot", $clock);
            }
            $crate::shutdown::ShutdownReason::HostClockSyncExpired => {
                ::ankyra::klipper_shutdown_from!($sender, "Host clock-sync expired", $clock);
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh0AdcVmLimitsWriteFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch0 bootstrap: ADC_VM_LIMITS write failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh0StatusMaskWriteFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch0 bootstrap: STATUS_MASK write failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh0StatusFlagsReadFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch0 bootstrap: STATUS_FLAGS read failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh0StatusFlagsClearFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch0 bootstrap: STATUS_FLAGS clear failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh1AdcVmLimitsWriteFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch1 bootstrap: ADC_VM_LIMITS write failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh1StatusMaskWriteFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch1 bootstrap: STATUS_MASK write failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh1StatusFlagsReadFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch1 bootstrap: STATUS_FLAGS read failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcBootstrapCh1StatusFlagsClearFailed => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671 ch1 bootstrap: STATUS_FLAGS clear failed",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::TmcRequestValidationFailed => {
                ::ankyra::klipper_shutdown_from!($sender, "TMC request validation failed", $clock);
            }
            $crate::shutdown::ShutdownReason::Tmc4671PllNotLocked => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671: PLL not locked (clock lost)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::Tmc4671AdcCurrentClipped => {
                ::ankyra::klipper_shutdown_from!(
                    $sender,
                    "TMC4671: ADC current clipped (overcurrent)",
                    $clock
                );
            }
            $crate::shutdown::ShutdownReason::Tmc4671Fault => {
                ::ankyra::klipper_shutdown_from!($sender, "TMC4671 fault", $clock);
            }
        }
    };
}

/// Foci-flavored wrapper around `::ankyra::ankyra_config!` that bakes in
/// the full FOCI shutdown-literal list.
///
/// The board-specific extras go via the `board_static_strings = [...]`
/// key; every other `ankyra_config!` key is forwarded verbatim.
///
/// # Example
///
/// ```ignore
/// foci_firmware::foci_ankyra_config! {
///     transport = crate::transport::TRANSPORT_OUTPUT: crate::transport::BufferTransportOutput,
///     context = &'ctx mut crate::firmware_ctx::FirmwareCtx<'ctx>,
///     providers = [crate::ankyra::provider::BOARD_PROVIDER],
///     board_static_strings = [],
///     app = "foci",
///     version = env!("CARGO_PKG_VERSION"),
///     build_versions = "",
///     license = "MIT OR Apache-2.0",
/// }
/// ```
#[macro_export]
macro_rules! foci_ankyra_config {
    (
        transport = $transport_static:path : $transport_ty:path,
        context = $context:ty,
        providers = [ $($provider:path),* $(,)? ],
        board_static_strings = [ $($board_lit:literal),* $(,)? ],
        app = $app:expr,
        version = $version:expr,
        build_versions = $build_versions:expr,
        license = $license:expr $(,)?
    ) => {
        ::ankyra::ankyra_config! {
            transport = $transport_static : $transport_ty,
            context = $context,
            providers = [ $($provider),* ],
            static_strings = [
                $($board_lit,)*
                // FOCI firmware-wide shutdown literals. Keep top-level reasons in sync with
                // `ShutdownReason::message` and `dispatch_shutdown!`, and include literals
                // emitted by shared-control board glue.
                "Shutdown cleared when not shutdown",
                "config_reset only available when shutdown",
                "Command request",
                "config_digital_out failed",
                "Motor enable/disable lost (queue full)",
                "queue_digital_out failed",
                "update_digital_out failed",
                "config_stepper failed",
                "Unknown stepper OID",
                "Invalid stepper channel",
                "queue_step failed",
                "Stepper too far in past",
                "Can't reset time when stepper active",
                "stepper_stop_on_trigger: unknown stepper",
                "stepper_stop_on_trigger failed",
                "config_endstop failed",
                "endstop_home failed",
                "endstop_query_state failed",
                "config_trsync failed",
                "trsync_start failed",
                "trsync_set_timeout failed",
                "trsync_trigger failed",
                "Invalid TMC channel",
                "tmc_set_current lost (queue full)",
                "tmc_set_motion_scale lost (queue full)",
                "tmc_set_encoder_dir lost (queue full)",
                "tmc_set_pid_gains lost (queue full)",
                "tmc_set_position_gains lost (queue full)",
                "tmc_set_filter lost (queue full)",
                "tmc_set_voltage_limit lost (queue full)",
                "TMC diagnostic lost (queue full)",
                "TMC4671 chip ID mismatch",
                "TMC4671 SPI failed at boot",
                "TMC4671 ch0 bootstrap: ADC_VM_LIMITS write failed",
                "TMC4671 ch0 bootstrap: STATUS_MASK write failed",
                "TMC4671 ch0 bootstrap: STATUS_FLAGS read failed",
                "TMC4671 ch0 bootstrap: STATUS_FLAGS clear failed",
                "TMC4671 ch1 bootstrap: ADC_VM_LIMITS write failed",
                "TMC4671 ch1 bootstrap: STATUS_MASK write failed",
                "TMC4671 ch1 bootstrap: STATUS_FLAGS read failed",
                "TMC4671 ch1 bootstrap: STATUS_FLAGS clear failed",
                "TMC request validation failed",
                "TMC current limit invalid",
                "Live gain application failed",
                "TMC4671: PLL not locked (clock lost)",
                "TMC4671: ADC current clipped (overcurrent)",
                "TMC4671 fault",
                "Host clock-sync expired",
            ],
            app = $app,
            version = $version,
            build_versions = $build_versions,
            license = $license,
        }
    };
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn host_clock_sync_expired_has_a_message() {
        assert_eq!(
            ShutdownReason::HostClockSyncExpired.message(),
            "Host clock-sync expired"
        );
    }
}
