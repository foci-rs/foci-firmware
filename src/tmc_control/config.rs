//! Configuration and borrowed state bundles for TMC control runtime calls.

use core::sync::atomic::{AtomicBool, AtomicU8};

use crate::config::TmcConfig;
use crate::state::stepper::StepperChannelState;
use crate::state::tmc::TmcChannelState;

/// Board constants needed by shared TMC runtime policy.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct TmcControlConfig {
    /// Klipper timer clock frequency reported by this board.
    pub clock_freq: u32,
    /// TMC PWM frequency in Hz for velocity-scale and control calculations.
    pub pwm_freq_hz: u32,
    /// Board current-scale conversion from TMC register LSB to mA.
    pub current_scale_ma_per_lsb: f32,
    /// Existing board-specific TMC4671 hardware configuration.
    pub tmc: TmcConfig,
}

impl TmcControlConfig {
    /// Build runtime configuration from board constants.
    pub const fn new(
        clock_freq: u32,
        pwm_freq_hz: u32,
        current_scale_ma_per_lsb: f32,
        tmc: TmcConfig,
    ) -> Self {
        Self {
            clock_freq,
            pwm_freq_hz,
            current_scale_ma_per_lsb,
            tmc,
        }
    }
}

/// Borrowed channel state passed to runtime calls without ownership transfer.
#[derive(Clone, Copy)]
pub struct TmcControlStateRefs<'a> {
    /// Shared TMC channel atomics.
    pub tmc: &'a TmcChannelState,
    /// Shared stepper channel atomics.
    pub stepper: &'a StepperChannelState,
}

impl<'a> TmcControlStateRefs<'a> {
    /// Build a state-reference bundle.
    pub const fn new(tmc: &'a TmcChannelState, stepper: &'a StepperChannelState) -> Self {
        Self { tmc, stepper }
    }
}

/// Borrowed homing signals used by slow maintenance.
pub struct TmcControlHomingRefs<'a> {
    /// Homing state atomically mirrored by board code.
    pub state: &'a AtomicU8,
    /// Homing triggered flag atomically mirrored by board code.
    pub triggered: &'a AtomicU8,
    /// A pass registered during the previous pass's restore.
    pub reenter: &'a AtomicBool,
}

impl<'a> TmcControlHomingRefs<'a> {
    /// Build a homing-reference bundle.
    pub const fn new(
        state: &'a AtomicU8,
        triggered: &'a AtomicU8,
        reenter: &'a AtomicBool,
    ) -> Self {
        Self {
            state,
            triggered,
            reenter,
        }
    }
}
