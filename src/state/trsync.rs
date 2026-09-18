//! Trsync protocol state: homing FSM, homing-triggered oid, endstop
//! pin map.

use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32};

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

/// Klipper clock at which the pending homing move's steps begin.
///
/// Klipper derives a trsync's `report_clock` from the homing move's start
/// clock, so `stepper_stop_on_trigger` records the linked trsync's report
/// clock here. That command itself arrives as soon as the host plans the
/// home, often while the previous move is still stepping, so neither the
/// state-1 step hold nor the P1 clamp may key off its receipt: the hold
/// defers only steps scheduled at or after this clock, and P1 applies the
/// clamp a fixed lead ahead of it. On a later MCU of a multi-MCU home Klipper
/// offsets the report clock by up to 0.3 x the trsync timeout after the start.
pub static HOMING_START_CLOCK: AtomicU32 = AtomicU32::new(0);

/// A homing pass registered while the previous pass was still being torn
/// down (state 2 or 3), so the tick must re-enter state 1 after the restore
/// instead of returning to idle.
///
/// Klipper's second homing pass registers within milliseconds of the first
/// pass's trigger, before the tick has restored run current; without this
/// the registration would be lost and the second pass would run unclamped.
pub static HOMING_REENTER_PENDING: AtomicBool = AtomicBool::new(false);

/// Begin homing preparation for a move whose steps start at `start_clock`.
///
/// Publishes the start clock before the idle-to-pending transition so no
/// reader can observe state 1 with a stale clock. Returns whether this call
/// made the transition; a second stepper registering on the same trsync
/// records the same clock and returns `false`.
pub fn begin_homing_preparation(start_clock: u32) -> bool {
    use core::sync::atomic::Ordering::Relaxed;
    HOMING_START_CLOCK.store(start_clock, Relaxed);
    match HOMING_STATE.compare_exchange(0, 1, Relaxed, Relaxed) {
        Ok(_) => true,
        Err(2 | 3) => {
            HOMING_REENTER_PENDING.store(true, Relaxed);
            false
        }
        Err(_) => false,
    }
}

/// Whether a step scheduled at `step_clock` must wait for homing preparation.
///
/// Only steps at or after [`HOMING_START_CLOCK`] are held, while
/// [`HOMING_STATE`] is 1 or a re-entry is queued behind a restore; steps of
/// a move still in flight when the host announced the home run on schedule.
pub fn step_held_for_homing(step_clock: u32) -> bool {
    use core::sync::atomic::Ordering::Relaxed;
    (HOMING_STATE.load(Relaxed) == 1 || HOMING_REENTER_PENDING.load(Relaxed))
        && (step_clock.wrapping_sub(HOMING_START_CLOCK.load(Relaxed)) as i32) >= 0
}

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

#[cfg(test)]
mod tests {
    use core::sync::atomic::Ordering::Relaxed;

    use super::*;

    #[test]
    fn step_hold_applies_only_to_steps_at_or_after_the_homing_start_clock() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_START_CLOCK.store(1_000_000, Relaxed);

        HOMING_STATE.store(0, Relaxed);
        assert!(!step_held_for_homing(1_000_000));

        HOMING_STATE.store(1, Relaxed);
        assert!(
            !step_held_for_homing(999_999),
            "in-flight steps of the previous move run"
        );
        assert!(step_held_for_homing(1_000_000));
        assert!(step_held_for_homing(1_000_500));

        HOMING_START_CLOCK.store(100, Relaxed);
        assert!(
            !step_held_for_homing(u32::MAX - 5),
            "clock wrap: just before the start"
        );
        assert!(step_held_for_homing(150));

        HOMING_STATE.store(2, Relaxed);
        assert!(!step_held_for_homing(1_000_500));
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn registration_during_restore_queues_a_re_entry_instead_of_being_lost() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_REENTER_PENDING.store(false, Relaxed);

        HOMING_STATE.store(3, Relaxed);
        assert!(!begin_homing_preparation(500));
        assert_eq!(HOMING_STATE.load(Relaxed), 3);
        assert!(HOMING_REENTER_PENDING.load(Relaxed));
        assert_eq!(HOMING_START_CLOCK.load(Relaxed), 500);

        HOMING_REENTER_PENDING.store(false, Relaxed);
        HOMING_STATE.store(1, Relaxed);
        assert!(
            !begin_homing_preparation(600),
            "a companion registration in state 1 is not a re-entry"
        );
        assert!(!HOMING_REENTER_PENDING.load(Relaxed));

        HOMING_STATE.store(0, Relaxed);
        assert!(begin_homing_preparation(700));
        assert_eq!(HOMING_STATE.load(Relaxed), 1);
        assert!(!HOMING_REENTER_PENDING.load(Relaxed));
        HOMING_STATE.store(0, Relaxed);
    }

    #[test]
    fn step_hold_covers_a_queued_re_entry_while_restore_is_pending() {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        HOMING_START_CLOCK.store(1_000_000, Relaxed);
        HOMING_STATE.store(3, Relaxed);
        HOMING_REENTER_PENDING.store(true, Relaxed);

        assert!(!step_held_for_homing(999_999));
        assert!(step_held_for_homing(1_000_000));

        HOMING_REENTER_PENDING.store(false, Relaxed);
        assert!(!step_held_for_homing(1_000_000));
        HOMING_STATE.store(0, Relaxed);
    }
}
