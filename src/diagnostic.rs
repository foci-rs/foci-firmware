//! Structured diagnostic codes emitted before fatal shutdowns when transport is usable.

/// Fatal fault class carried by `foci_diagnostic`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum FaultClass {
    /// A queue could not accept an operation.
    QueueFull = 1,
    /// A command referenced an unknown OID.
    UnknownOid = 2,
    /// A reset was requested while a channel was active.
    ActiveReset = 3,
    /// A command referenced an invalid channel.
    InvalidChannel = 4,
}

/// Operation code carried by `foci_diagnostic`.
pub mod operation {
    /// `queue_step`.
    pub const QUEUE_STEP: u8 = 1;
    /// `set_next_step_dir`.
    pub const SET_NEXT_STEP_DIR: u8 = 2;
    /// `reset_step_clock`.
    pub const RESET_STEP_CLOCK: u8 = 3;
    /// `tmc_control`.
    pub const TMC_CONTROL: u8 = 4;
    /// `stepper_stop`.
    pub const STEPPER_STOP: u8 = 5;
    /// Stepper OID/channel lookup before a motion command.
    pub const STEPPER_LOOKUP: u8 = 6;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fault_class_codes_are_stable() {
        assert_eq!(FaultClass::QueueFull as u8, 1);
        assert_eq!(FaultClass::UnknownOid as u8, 2);
        assert_eq!(FaultClass::ActiveReset as u8, 3);
        assert_eq!(FaultClass::InvalidChannel as u8, 4);
    }

    #[test]
    fn operation_codes_are_stable() {
        assert_eq!(operation::QUEUE_STEP, 1);
        assert_eq!(operation::SET_NEXT_STEP_DIR, 2);
        assert_eq!(operation::RESET_STEP_CLOCK, 3);
        assert_eq!(operation::TMC_CONTROL, 4);
        assert_eq!(operation::STEPPER_STOP, 5);
        assert_eq!(operation::STEPPER_LOOKUP, 6);
    }
}
