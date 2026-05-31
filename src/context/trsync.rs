//! Shared trsync/endstop context boundary and policy helpers.
//!
//! `TrsyncHost` is the handler-facing trait. The remaining items in this
//! module are the shared trigger-domain policy surface used by board crates.

/// Trsync + endstop-trigger subsystem bridge. Each method corresponds
/// one-to-one to a single handler's `with_trigger_domain(|td| ...)`
/// closure body: the trait exposes handler-specific coarse methods.
///
/// `TriggerDomain` itself stays in the board crate; this trait is the
/// dyn-safe bridge used from the lifted handlers.
#[allow(clippy::result_unit_err)]
pub trait TrsyncHost: crate::context::StateAccess {
    /// Full trigger-domain reset used by `config_reset` to bring
    /// steppers, trsync, endstops, and scheduler back to power-on
    /// defaults.
    fn reset_all(&mut self);

    /// Allocate a trsync slot for `oid`.
    ///
    /// Returns `Err` if the OID is already registered or no free slot exists.
    fn register_trsync(&mut self, oid: u8) -> Result<(), ()>;

    /// Arm a trsync for synchronized triggering.
    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()>;

    /// Update the expiry deadline for a trsync and schedule its expire event.
    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()>;

    /// Manually fire a trsync and return the effective trigger reason.
    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()>;

    /// Allocate an endstop slot for `oid`.
    ///
    /// Returns the allocated index on success.
    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()>;

    /// Configure an endstop runtime for homing and schedule its first sample.
    ///
    /// `sample_count == 0` acts as a disarm.
    #[allow(clippy::too_many_arguments)]
    fn endstop_start_homing(
        &mut self,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value_expected: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) -> Result<(), ()>;

    /// Read the endstop runtime state tuple `(is_homing, next_clock, pin_id)`.
    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()>;

    /// Link a physical stepper to a trsync stop signal.
    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<(), ()>;
}

#[allow(clippy::result_unit_err)]
impl<T: ?Sized + TrsyncHost> TrsyncHost for &mut T {
    fn reset_all(&mut self) {
        (**self).reset_all()
    }

    fn register_trsync(&mut self, oid: u8) -> Result<(), ()> {
        (**self).register_trsync(oid)
    }

    fn trsync_arm(
        &mut self,
        oid: u8,
        report_clock: u32,
        report_ticks: u32,
        expire_reason: u8,
    ) -> Result<(), ()> {
        (**self).trsync_arm(oid, report_clock, report_ticks, expire_reason)
    }

    fn trsync_set_timeout(&mut self, oid: u8, clock: u32) -> Result<(), ()> {
        (**self).trsync_set_timeout(oid, clock)
    }

    fn trsync_trigger(&mut self, oid: u8, reason: u8) -> Result<u8, ()> {
        (**self).trsync_trigger(oid, reason)
    }

    fn register_endstop(&mut self, oid: u8) -> Result<usize, ()> {
        (**self).register_endstop(oid)
    }

    fn endstop_start_homing(
        &mut self,
        oid: u8,
        clock: u32,
        sample_ticks: u32,
        sample_count: u8,
        rest_ticks: u32,
        pin_value_expected: bool,
        trsync_oid: u8,
        trigger_reason: u8,
    ) -> Result<(), ()> {
        (**self).endstop_start_homing(
            oid,
            clock,
            sample_ticks,
            sample_count,
            rest_ticks,
            pin_value_expected,
            trsync_oid,
            trigger_reason,
        )
    }

    fn endstop_query(&mut self, oid: u8) -> Result<(bool, u32, u8), ()> {
        (**self).endstop_query(oid)
    }

    fn link_stepper_stop(&mut self, stepper_idx: usize, trsync_oid: u8) -> Result<(), ()> {
        (**self).link_stepper_stop(stepper_idx, trsync_oid)
    }
}
