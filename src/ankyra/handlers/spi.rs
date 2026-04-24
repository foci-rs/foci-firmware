//! SPI command handlers: `config_spi`, `config_spi_without_cs`,
//! `spi_set_bus`, `config_spi_shutdown`, `spi_send`, `spi_transfer`.
//!
//! Lifted out of `boards/openffboard-fw/src/ankyra/handlers/spi.rs`
//! in Task 14 of the foci-firmware handler-lift plan. These are stub
//! handlers: the TMC4671 SPI bus is managed internally by the firmware,
//! so host-side SPI commands are accepted for protocol compatibility
//! but forward to no-op helpers in `foci_klipper::commands::spi`.
//!
//! See the handler audit at
//! `docs/notes/foci-firmware-lift-handler-audit.md` (lines 323–342) for
//! the exact capability bound used by each handler.

use ankyra::prelude::klipper_command;

use crate::context::StateAccess;

/// SPI configuration stub (TMC4671 SPI is handled internally).
#[klipper_command]
pub fn config_spi(_ctx: &mut dyn StateAccess, oid: u8, pin: u32, cs_active_high: u8) {
    foci_klipper::commands::spi::config_spi(oid, pin, cs_active_high);
}

/// SPI configuration without chip-select stub.
#[klipper_command]
pub fn config_spi_without_cs(_ctx: &mut dyn StateAccess, oid: u8) {
    foci_klipper::commands::spi::config_spi_without_cs(oid);
}

/// SPI bus assignment stub.
#[klipper_command]
pub fn spi_set_bus(_ctx: &mut dyn StateAccess, oid: u8, spi_bus: u32, mode: u32, rate: u32) {
    foci_klipper::commands::spi::spi_set_bus(oid, spi_bus, mode, rate);
}

/// SPI shutdown message configuration stub.
#[klipper_command]
pub fn config_spi_shutdown(_ctx: &mut dyn StateAccess, oid: u8, spi_oid: u8, shutdown_msg: &[u8]) {
    foci_klipper::commands::spi::config_spi_shutdown(oid, spi_oid, shutdown_msg);
}

/// SPI send stub (data is accepted but not forwarded).
#[klipper_command]
pub fn spi_send(_ctx: &mut dyn StateAccess, oid: u8, data: &[u8]) {
    foci_klipper::commands::spi::spi_send(oid, data);
}

/// SPI transfer stub (returns empty response).
#[klipper_command]
pub fn spi_transfer(_ctx: &mut dyn StateAccess, oid: u8, _data: &[u8]) {
    let empty: &[u8] = &[];
    ::ankyra::klipper_reply!(
        crate::ankyra::replies::SpiTransferResponse::<'static>,
        oid: u8 = oid,
        response: &[u8] = empty
    );
}
