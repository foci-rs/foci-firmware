//! Platform traits — the hardware abstraction surface between foci-firmware
//! and each board binary. All I/O in foci-firmware routes through one of
//! these traits; impls live in the board crate (boards/<board>-fw/).
//!
//! The traits are intentionally synchronous and narrow: they describe what
//! the firmware needs, not how any particular MCU provides it. Board crates
//! wrap their RTIC resources, embassy-stm32 handles, cortex-m peripherals,
//! etc. behind these interfaces so foci-firmware stays board-agnostic.

/// Monotonic clock exposed to the Klipper protocol.
///
/// The firmware reports `frequency_hz()` as `CLOCK_FREQ` and ticks from
/// `now_ticks()` as the Klipper `clock` value. The tick counter wraps at
/// `u32::MAX`; all comparisons elsewhere in the firmware must be done in
/// wraparound-safe arithmetic.
pub trait Clock {
    /// Current monotonic tick count, at `frequency_hz()`. Used for the
    /// Klipper clock.
    fn now_ticks(&self) -> u32;

    /// CPU/timer clock in Hz. Reported as `CLOCK_FREQ` to the host.
    fn frequency_hz(&self) -> u32;
}

/// Hardware step-pulse timer (e.g. an STM32 compare channel).
///
/// The firmware arms and cancels step events from the step scheduler; the
/// board binary drives an actual timer channel so step pulses are emitted
/// deterministically without CPU wake-up.
pub trait StepTimer {
    /// Arm the next step event at the given absolute clock tick.
    fn arm(&mut self, clock: u32);

    /// Cancel any armed step event. Idempotent.
    fn cancel(&mut self);
}

/// GPIO pair that toggles the STEP/DIR pins of a single stepper.
pub trait StepPins {
    /// Drive the STEP pin.
    fn set_step(&mut self, value: bool);

    /// Drive the DIR pin.
    fn set_dir(&mut self, value: bool);
}

/// Digital-in pins used as Klipper endstops / trsync inputs.
///
/// `pin_id` is an opaque index the firmware assigned when it called
/// `configure`; the board impl maps it to a real GPIO pin internally.
pub trait EndstopInput {
    /// Read the current logical level of the pin.
    fn read(&self, pin_id: u8) -> bool;

    /// Configure a pin as a digital input with the requested pull resistor.
    fn configure(&mut self, pin_id: u8, pull_up: bool);
}

/// Controls the hardware motor enable signal (TMC4671 enable pin).
pub trait MotorEnable {
    /// Assert (`true`) or de-assert (`false`) the enable pin.
    fn set_enabled(&mut self, value: bool);
}

/// Latched fault indicator from the TMC4671 `STATUS_FLAGS` interrupt.
pub trait FaultInput {
    /// Read-and-clear the latched fault state. Returns `true` if a fault
    /// interrupt has been observed since the last call.
    fn take_latched(&mut self) -> bool;
}

/// SPI bus to the TMC4671.
///
/// This is just an alias for `embedded_hal_async::spi::SpiDevice` so the
/// firmware can depend on the HAL trait directly while keeping the
/// platform surface discoverable in one place.
pub trait TmcBus: embedded_hal_async::spi::SpiDevice {}

// SAFETY: blanket impl — any `SpiDevice` is a valid `TmcBus`.
impl<T> TmcBus for T where T: embedded_hal_async::spi::SpiDevice {}

/// Host transport: the firmware's sink for Klipper protocol responses.
///
/// Board impls typically forward bytes to the USB CDC-ACM TX buffer and
/// signal the USB task to drain the buffer to the host.
pub trait TransportTx {
    /// Queue `bytes` for transmission on the host transport (USB CDC-ACM).
    /// Returns the number of bytes actually queued; may be less than
    /// `bytes.len()` if the transport's TX buffer is full.
    fn write(&mut self, bytes: &[u8]) -> usize;

    /// Flush queued bytes to the transport (signal the TX task, etc.).
    fn flush(&mut self);
}
