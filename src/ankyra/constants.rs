//! Firmware-wide `#[klipper_constant]` items registered into the
//! Klipper data dictionary's `config` section. Board-specific
//! constants (`CLOCK_FREQ`, `MCU`, `RESERVE_PINS_*`) live in the
//! board crate.

use ankyra::prelude::klipper_constant;

/// Klipper-protocol receive-window size in bytes. Matches the
/// anchor reference implementation.
#[klipper_constant]
pub const RECEIVE_WINDOW: u32 = 192;

/// Scale factor for stats reporting's sum-of-squares field.
#[klipper_constant]
pub const STATS_SUMSQ_BASE: u32 = 256;

/// Host-side flag: MCU supports step-on-both-edges. FOCI reports
/// true because the TMC4671 STEP input counts rising edges only.
#[klipper_constant]
pub const STEPPER_STEP_BOTH_EDGE: u32 = 1;
