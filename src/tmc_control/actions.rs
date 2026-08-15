//! Board-visible actions returned by the shared TMC control runtime.

use crate::commission_dispatch::{PostActions, ReplyPayload, StepperPostAction};
use crate::commissioning::schedule::CommissionEvent;
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
    /// The requested current exceeds the TMC4671 torque/flux limit field.
    CurrentLimitInvalid,
    /// Applying a live inner- or outer-loop gain pair failed.
    LiveGainApplyFailed,
    /// STATUS_FLAGS reported ADC current clipping.
    AdcCurrentClipped,
    /// STATUS_FLAGS reported a non-clipping TMC fault.
    TmcFault,
}

/// Whether a board-local request handler should run the tick tail after a request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TmcControlRequestTail {
    /// Preserve the board's normal post-request tick-tail work.
    FallThrough,
    /// Skip the tick tail for this control-loop iteration.
    SkipTickTail,
}

/// Board-local TMC request outcome shared by OpenFFBoard and Ouroboros shells.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TmcControlRequestOutcome {
    /// Tick-tail behavior after request handling.
    pub tail: TmcControlRequestTail,
    /// Optional board-local Klipper shutdown reason to emit from the caller.
    pub shutdown: Option<ShutdownReason>,
    /// Whether the caller should refresh the runtime velocity scale.
    pub should_refresh_velocity_scale: bool,
}

impl TmcControlRequestOutcome {
    /// Continue through the board tick tail with no shutdown or refresh.
    pub const fn fall_through() -> Self {
        Self {
            tail: TmcControlRequestTail::FallThrough,
            shutdown: None,
            should_refresh_velocity_scale: false,
        }
    }

    /// Skip the board tick tail with no shutdown or refresh.
    pub const fn skip_tick_tail() -> Self {
        Self {
            tail: TmcControlRequestTail::SkipTickTail,
            shutdown: None,
            should_refresh_velocity_scale: false,
        }
    }

    /// Emit a board-local shutdown, then apply the requested tick-tail behavior.
    pub const fn shutdown(reason: ShutdownReason, tail: TmcControlRequestTail) -> Self {
        Self {
            tail,
            shutdown: Some(reason),
            should_refresh_velocity_scale: false,
        }
    }

    /// Refresh velocity scale, then apply the requested tick-tail behavior.
    pub const fn refresh_velocity_scale(tail: TmcControlRequestTail) -> Self {
        Self {
            tail,
            shutdown: None,
            should_refresh_velocity_scale: true,
        }
    }

    /// Emit shutdown and refresh velocity scale, then apply the requested tick-tail behavior.
    pub const fn shutdown_then_refresh_velocity_scale(
        reason: ShutdownReason,
        tail: TmcControlRequestTail,
    ) -> Self {
        Self {
            tail,
            shutdown: Some(reason),
            should_refresh_velocity_scale: true,
        }
    }
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
pub enum BoardAction {
    /// Enter Klipper shutdown with the provided reason.
    Shutdown(ShutdownReason),
    /// Signal the board-local emergency-disable path.
    SignalEmergencyDisable,
    /// Perform board-local stepper work.
    Stepper(StepperAction),
    /// Run a board-local active diagnostic flow.
    DelegatedDiagnostic(DelegatedDiagnostic),
}

/// Fixed-capacity list of board actions emitted by one runtime call.
///
/// Capacity proof for this plan: current paths emit at most three actions
/// (stop, shutdown, and one arm/diagnostic action). Keep capacity at 8 so a
/// future path can add compact routing actions without silently dropping safety work.
pub type BoardActions = heapless::Vec<BoardAction, 8>;

const _: () = assert!(core::mem::size_of::<BoardActions>() <= 400);

/// Push an action and assert the fixed action buffer is large enough.
pub fn push_action(actions: &mut BoardActions, action: BoardAction) {
    assert!(
        actions.push(action).is_ok(),
        "TMC runtime action buffer capacity exhausted"
    );
}

/// Runtime result from the explicit commissioning scheduler.
#[allow(clippy::large_enum_variant)]
pub enum ScheduledCommissionTickAction {
    /// Poll again without adding an artificial delay.
    ReadyNow {
        /// Optional nonterminal event produced by this poll.
        event: Option<CommissionEvent>,
    },
    /// Poll again at the absolute microsecond timestamp.
    At {
        /// Absolute timestamp in the commissioning timing domain.
        timestamp_us: u64,
        /// Optional nonterminal event produced by this poll.
        event: Option<CommissionEvent>,
    },
    /// A causal workflow handoff was emitted; keep polling the live engine.
    WorkflowContinuation,
    /// Engine reached a terminal result.
    Terminal {
        /// Reply payload for the board to emit.
        reply: ReplyPayload,
        /// Post-actions the board must execute after emitting the reply.
        actions: PostActions,
    },
}

// This routing enum still carries the pre-existing terminal reply so the board
// can preserve stop-trace -> stepper action -> reply ordering. Keep it at the
// compact size; continuation evidence must never add a second
// inline reply carrier.
const _: () = assert!(core::mem::size_of::<ScheduledCommissionTickAction>() <= 1_400);
