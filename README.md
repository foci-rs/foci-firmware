# foci-firmware

Board-agnostic FOCI firmware library

Part of the [FOCI](https://github.com/foci-rs/foci) project — a
Klipper/Kalico-compatible MCU firmware family for STM32 boards with
TMC4671 FOC servo controller ICs.

## Status

Pre-1.0; APIs may change without notice. Used in production firmware
on the OpenFFBoard test rig but not yet versioned for external
consumers.

## Use

Add as a git dependency in your `Cargo.toml`:

```toml
foci-firmware = { git = "https://github.com/foci-rs/foci-firmware" }
```

## License

Dual-licensed under MIT or Apache-2.0 at your option. See `LICENSE-MIT` and `LICENSE-APACHE` at the repo root..
