//! Trsync protocol state: homing FSM, homing-triggered oid, endstop
//! pin map.

use core::sync::atomic::AtomicU8;

use foci_klipper::state::NUM_ENDSTOPS;

/// One past the highest valid physical-GPIO pin number in the Klipper
/// pin enumeration (`PA0`..`PE15` = 0..79). Pin values at or above this
/// are virtual (no GPIO backing) and must not be passed to GPIO helpers.
///
/// Matches `GPIO_NUM_PORTS * 16` (5 ports × 16 pins) defined in
/// `openffboard-fw::helpers::trsync`.
pub const GPIO_MAX_PIN_VALUE: u8 = 5 * 16;

/// Homing I-suppression state machine.
/// 0=Idle, 1=SuppressPending, 2=Suppressing, 3=RestorePending
pub static HOMING_STATE: AtomicU8 = AtomicU8::new(0);

/// TrSync OID that fired the trigger. 0xFF = no trigger pending.
/// Written by TriggerDomain on first trigger, cleared by
/// trsync_trigger command cleanup or config_reset.
pub static HOMING_TRIGGERED: AtomicU8 = AtomicU8::new(0xFF);

/// Per-endstop pin identifier (enum value from `config_endstop`).
/// `0xFF` means no pin configured.
pub static ENDSTOP_PIN_IDS: [AtomicU8; NUM_ENDSTOPS] = [
    AtomicU8::new(0xFF),
    AtomicU8::new(0xFF),
    AtomicU8::new(0xFF),
    AtomicU8::new(0xFF),
];
