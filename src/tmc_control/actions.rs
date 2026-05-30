//! Board-visible actions returned by the shared TMC control runtime.

use crate::autotune::PhaseId;
use crate::commission_dispatch::{PostActions, ReplyPayload, StepperPostAction};
use crate::commissioning::types::EngineKind;
use crate::tmc::TmcRequest;

/// Shutdown reason emitted by board-local Klipper shutdown glue.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ShutdownReason {
    /// TMC4671 chip identification read failed during boot.
    BootSpiFailed,
    /// TMC4671 chip identification did not match the expected value.
    ChipIdMismatch,
    /// A request referenced a channel unsupported by this board.
    InvalidTmcChannel,
    /// A core TMC request failed validation.
    RequestValidationFailed,
    /// STATUS_FLAGS reported ADC current clipping.
    AdcCurrentClipped,
    /// STATUS_FLAGS reported a non-clipping TMC fault.
    TmcFault,
}

/// Board-owned stepper follow-up work.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum StepperAction {
    /// No stepper follow-up work.
    None,
    /// Stop physical stepping immediately and schedule stop cleanup.
    StopNow,
    /// Run the board-local electrical arm verification and arm step/dir output.
    ArmCheck {
        /// Post-action to apply after `arm_motor_electrical` succeeds.
        post_action: StepperPostAction,
    },
    /// Apply a terminal commissioning post-action.
    ApplyPostAction(StepperPostAction),
}

/// Active diagnostic request intentionally left board-local.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub struct DelegatedDiagnostic {
    /// Original request for board-local diagnostic handling.
    pub request: TmcRequest,
}

/// One immediate effect for the board shell to apply.
///
/// The large reply variants are intentionally stored inline. This crate is
/// `no_std`/no-heap, so boxing would violate the firmware memory model.
#[allow(clippy::large_enum_variant)]
pub enum BoardAction {
    /// Enter Klipper shutdown with the provided reason.
    Shutdown(ShutdownReason),
    /// Signal the board-local emergency-disable path.
    SignalEmergencyDisable,
    /// Perform board-local stepper work.
    Stepper(StepperAction),
    /// Emit a commissioning/self-test/calibration reply.
    Reply(ReplyPayload),
    /// Run a board-local active diagnostic flow.
    DelegatedDiagnostic(DelegatedDiagnostic),
}

/// Fixed-capacity list of board actions emitted by one runtime call.
///
/// Capacity proof for this plan: current paths emit at most three actions
/// (stop, shutdown/reply, and one arm/diagnostic action). Keep capacity at 8
/// so a future path can add replies without silently dropping safety actions.
pub type BoardActions = heapless::Vec<BoardAction, 8>;

/// Push an action and assert the fixed action buffer is large enough.
pub fn push_action(actions: &mut BoardActions, action: BoardAction) {
    assert!(
        actions.push(action).is_ok(),
        "TMC runtime action buffer capacity exhausted"
    );
}

/// Runtime result after one commissioning-engine tick.
///
/// The terminal reply payload stays inline for the same no-heap reason as
/// [`BoardAction`].
#[allow(clippy::large_enum_variant)]
pub enum CommissionTickAction {
    /// Engine continues; board should delay before the next loop iteration.
    Continue,
    /// Engine transitioned to a new phase.
    PhaseChanged {
        /// Object id of the commissioning channel.
        oid: u8,
        /// New phase id.
        phase_id: PhaseId,
        /// Engine kind for reply routing.
        engine_kind: EngineKind,
    },
    /// Engine reached a terminal outcome.
    Terminal {
        /// Reply payload for the board to emit.
        reply: ReplyPayload,
        /// Post-actions the board must execute after emitting the reply.
        actions: PostActions,
    },
}
