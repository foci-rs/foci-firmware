//! Shared TMC control runtime for board firmware.
//!
//! The runtime owns board-agnostic TMC control state. Board crates own RTIC
//! resources, SPI handles, GPIO pins, reply macros, shutdown macros, and task
//! spawning.

pub mod actions;
pub mod bootstrap;
pub mod commission_poll;
pub mod config;
pub mod current_limit;
pub mod dispatch;
pub mod driver;
pub mod filters;
pub mod helpers;
pub mod interlock;
pub mod lifecycle;
pub mod pending_config;
pub mod pi_representation;
pub mod request;
pub mod request_writes;
pub mod scheduler;
pub mod settle;
pub mod signals;
pub mod stall_detector;
pub mod state;

#[cfg(test)]
mod pi_representation_tests;
#[cfg(test)]
pub(crate) mod test_spi;
#[cfg(test)]
mod tests;

pub use actions::{
    BoardAction, BoardActions, DelegatedDiagnostic, ScheduledCommissionTickAction, ShutdownReason,
    StepperAction, TmcControlRequestOutcome, TmcControlRequestTail, push_action,
};
pub use bootstrap::{
    TmcSafetyBootstrapFailure, TmcSafetyBootstrapStage, bootstrap_tmc_safety,
    first_tmc_safety_bootstrap_failure,
};
pub use commission_poll::{
    AbsoluteWaitAction, COMMISSION_WAIT_SLICE_US, CommissionPollDriver,
    CommissionWaitRequestDisposition, FORCED_READY_YIELD_US, MAX_CONSECUTIVE_READY_POLLS,
    PollDriverAction, classify_absolute_wait, classify_wait_request, nonterminal_continuation,
};
pub use config::{TmcControlConfig, TmcControlHomingRefs, TmcControlStateRefs};
pub use driver::{
    AuditResult, BootstrapStep, ChannelDriver, CommissionNext, DriveCtx, DriverPhase, ParkReason,
    UnitOutcome,
};
pub use interlock::fault_interlock_tripped;
pub use request::{TmcPreRequestDisposition, TmcRequestBoard, handle_tmc_control_request};
pub use scheduler::{
    ChannelMode, ChannelScheduler, MAINTENANCE_PERIOD_US, MaintenanceArm, MaintenanceArmStats,
    MaintenanceGroup, STATUS_AUDIT_PERIOD_US, SourceStats, Unit, UnitClass,
};
pub use signals::TmcControlSignals;
pub use state::{
    EmergencyDisableFault, HomingMaintenanceResult, TmcControlRuntime, TmcControlTraceState,
    TmcControlTraceTick, classify_emergency_disable_fault, combine_emergency_disable_fault,
    is_adc_current_clipped,
};

/// Slow-control ticks to wait before clearing settled PID error sums.
pub const HOLD_SETTLE_DELAY_MS: u8 = 100;
