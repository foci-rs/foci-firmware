//! Connect-time TMC configuration drain.
//!
//! Config-time setters record their value in [`TmcChannelState`] and set a
//! bit in `config_apply_pending` instead of enqueueing an SPI request, so a
//! connect-time configuration burst cannot overflow the bounded TMC request
//! channel. This module owns the one drain that pushes that recorded state
//! to the chip: the board control tick runs it on its own schedule, and
//! `arm_motor_electrical` runs it before energizing so a motor never arms
//! against configuration the host believes was applied.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;

use crate::pid::InnerLoopGains;
use crate::state::tmc::TmcChannelState;
use crate::tmc_control::filters::{BiquadFilterTarget, apply_live_filter};

/// Where the pending-config drain publishes state that must only be updated
/// after a confirmed successful write.
pub struct PendingConfigSink<'a> {
    /// Where confirmed inner gains are published after a successful write.
    pub stored_inner_gains: &'a mut Option<InnerLoopGains>,
}

/// Which recorded configuration item the pending-config drain failed on.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PendingConfigItem {
    /// `PIDOUT_UQ_UD_LIMITS`.
    VoltageLimit,
    /// `PID_TORQUE_FLUX_LIMITS`.
    Current,
    /// `ABN_DECODER_MODE`.
    EncoderDir,
    /// Flux and torque PI gains.
    InnerGains,
    /// Velocity and position PI gains.
    OuterGains,
    /// Biquad feedback filters.
    Filters,
}

/// Drain configuration recorded by config-time setters, if any is pending.
///
/// Config-time setters record their value in [`TmcChannelState`] and set
/// `config_apply_pending` rather than enqueueing an SPI request, so a
/// connect-time configuration burst cannot overflow the bounded TMC request
/// channel. The TMC task calls this from its periodic control tick to push
/// that recorded state to the chip.
///
/// Each bit is checked and cleared individually, so a failure writing one
/// item does not prevent retrying on the next control tick. Routes through
/// [`apply_live_filter`] so a recorded `0` disables its filter, which the
/// `apply_stored_filters` fast path does not do.
///
/// On `Ok`, every bit observed on entry has been applied and cleared. Bits
/// raised after that first load are deliberately left alone: config handlers
/// run at a higher priority than the control and arming paths, so a bit can
/// appear mid-drain, and it belongs to the next drain rather than this one.
/// An always-on assert enforces the first half of that against a local mask
/// rather than the live atomic, so a concurrent setter cannot trip it.
///
/// # Errors
///
/// Returns the [`PendingConfigItem`] that failed, so the caller can name it
/// before shutting the MCU down.
///
/// # Panics
///
/// Panics if a future branch returns success while leaving an observed item
/// unapplied -- a firmware bug, not a runtime condition.
pub async fn drain_pending_config<B>(
    tmc: &mut Tmc4671Spi<B>,
    tmc_state: &TmcChannelState,
    tmc_config: crate::config::TmcConfig,
    sink: PendingConfigSink<'_>,
) -> Result<(), PendingConfigItem>
where
    B: SpiDevice<u8>,
{
    use core::sync::atomic::Ordering::Relaxed;

    let pending = tmc_state.config_apply_pending.load(Relaxed);
    if pending == 0 {
        return Ok(());
    }
    let mut handled = 0u8;

    if (pending & crate::state::tmc::PENDING_VOLTAGE_LIMIT) != 0 {
        crate::tmc_control::request_writes::apply_set_voltage_limit(
            tmc,
            tmc_state,
            tmc_state.voltage_limit_cfg.load(Relaxed),
        )
        .await
        .map_err(|()| PendingConfigItem::VoltageLimit)?;
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_VOLTAGE_LIMIT, Relaxed);
        handled |= crate::state::tmc::PENDING_VOLTAGE_LIMIT;
    }

    if (pending & crate::state::tmc::PENDING_CURRENT) != 0 {
        crate::tmc_control::request_writes::apply_set_current(
            tmc,
            tmc_state,
            tmc_state.run_current_ma_cfg.load(Relaxed),
            tmc_config.current_scale_ma_per_lsb,
        )
        .await
        .map_err(|_| PendingConfigItem::Current)?;
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_CURRENT, Relaxed);
        handled |= crate::state::tmc::PENDING_CURRENT;
    }

    if (pending & crate::state::tmc::PENDING_ENCODER_DIR) != 0 {
        // An unset sentinel means the host has not chosen a direction, so
        // there is nothing to write -- but the item is still drained. Leaving
        // the bit set here would strand it forever, since no later drain has
        // anything more to go on.
        if let Some(invert) = tmc_state.encoder_invert_cfg.load(Relaxed) {
            crate::tmc_control::request_writes::apply_set_encoder_dir(tmc, tmc_state, invert)
                .await
                .map_err(|()| PendingConfigItem::EncoderDir)?;
        }
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_ENCODER_DIR, Relaxed);
        handled |= crate::state::tmc::PENDING_ENCODER_DIR;
    }

    if (pending & crate::state::tmc::PENDING_INNER_GAINS) != 0 {
        let gains = tmc_state
            .load_pending_inner_gains(Relaxed)
            .map_err(|_| PendingConfigItem::InnerGains)?;
        let status = crate::tmc_control::helpers::apply_live_inner_gains(
            sink.stored_inner_gains,
            tmc_state,
            tmc,
            gains,
        )
        .await;
        if !status.all_ok() {
            return Err(PendingConfigItem::InnerGains);
        }
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_INNER_GAINS, Relaxed);
        handled |= crate::state::tmc::PENDING_INNER_GAINS;
    }

    if (pending & crate::state::tmc::PENDING_OUTER_GAINS) != 0 {
        let gains = tmc_state
            .load_pending_outer_gains(Relaxed)
            .map_err(|_| PendingConfigItem::OuterGains)?;
        let status =
            crate::tmc_control::helpers::apply_live_outer_gains(tmc_state, tmc, gains).await;
        if !status.all_ok() {
            return Err(PendingConfigItem::OuterGains);
        }
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_OUTER_GAINS, Relaxed);
        handled |= crate::state::tmc::PENDING_OUTER_GAINS;
    }

    if (pending & crate::state::tmc::PENDING_FILTERS) != 0 {
        for (target, filter_hz) in [
            (
                BiquadFilterTarget::Velocity,
                tmc_state.velocity_filter_hz.load_or_zero(Relaxed),
            ),
            (
                BiquadFilterTarget::Torque,
                tmc_state.torque_filter_hz.load_or_zero(Relaxed),
            ),
            (
                BiquadFilterTarget::Position,
                tmc_state.position_filter_hz.load_or_zero(Relaxed),
            ),
            (
                BiquadFilterTarget::Flux,
                tmc_state.flux_filter_hz.load_or_zero(Relaxed),
            ),
        ] {
            apply_live_filter(tmc, tmc_state, target, filter_hz, tmc_config.pwm_maxcnt)
                .await
                .map_err(|()| PendingConfigItem::Filters)?;
        }
        tmc_state
            .config_apply_pending
            .fetch_and(!crate::state::tmc::PENDING_FILTERS, Relaxed);
        handled |= crate::state::tmc::PENDING_FILTERS;
    }

    assert!(
        handled == pending,
        "FATAL: drain_pending_config reported success with items it never \
         applied; every observed bit must be written or fail"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tmc_control::test_spi::MockRawSpi;
    use foci_tmc4671::spi::Tmc4671Spi;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        crate::state::tmc::tmc_channel0()
    }

    #[test]
    fn drain_pending_config_with_only_filters_does_not_write_voltage_limit() {
        use core::sync::atomic::Ordering::Relaxed;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            tmc_state().velocity_filter_hz.store(Some(100), Relaxed);
            tmc_state().torque_filter_hz.store(Some(0), Relaxed);
            tmc_state().position_filter_hz.store(Some(0), Relaxed);
            tmc_state().flux_filter_hz.store(Some(0), Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_FILTERS, Relaxed);

            let raw_spi = MockRawSpi::default();
            let mut tmc = Tmc4671Spi::new(raw_spi);

            let mut config = crate::config::TmcConfig::default_for_test();
            config.pwm_maxcnt = 4095;
            let mut stored_inner_gains = None;
            drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await
            .expect("apply should succeed");

            let raw_spi = tmc.into_inner();
            use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;
            let has_voltage_limit_write = raw_spi
                .writes
                .iter()
                .any(|w| w.len() >= 1 && w[0] == (PIDOUT_UQ_UD_LIMITS.addr() | 0x80));

            assert!(
                !has_voltage_limit_write,
                "drain_pending_config should not write PIDOUT_UQ_UD_LIMITS \
                 when only PENDING_FILTERS bit is set"
            );

            tmc_state().velocity_filter_hz.store(None, Relaxed);
            tmc_state().torque_filter_hz.store(None, Relaxed);
            tmc_state().position_filter_hz.store(None, Relaxed);
            tmc_state().flux_filter_hz.store(None, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_current_bit_writes_the_converted_torque_flux_limit() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let mut config = crate::config::TmcConfig::default_for_test();
            config.pwm_maxcnt = 4095;
            config.current_scale_ma_per_lsb = 1.155;

            tmc_state().run_current_ma_cfg.store(1500, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_CURRENT, Relaxed);

            let raw_spi = MockRawSpi::default();
            let mut tmc = Tmc4671Spi::new(raw_spi);

            let mut stored_inner_gains = None;
            drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await
            .expect("apply should succeed");

            let expected = crate::tmc_control::current_limit::current_limit_from_run_ma(
                1500,
                config.current_scale_ma_per_lsb,
            )
            .expect("test scale should produce a valid limit")
            .register_value();

            let raw_spi = tmc.into_inner();
            let frame = |value: u32| -> std::vec::Vec<u8> {
                let bytes = value.to_be_bytes();
                std::vec![
                    PID_TORQUE_FLUX_LIMITS.addr() | 0x80,
                    bytes[0],
                    bytes[1],
                    bytes[2],
                    bytes[3]
                ]
            };
            assert!(
                raw_spi.writes.contains(&frame(expected)),
                "expected a PID_TORQUE_FLUX_LIMITS write of {expected}, got {:?}",
                raw_spi.writes
            );
            assert_eq!(
                tmc_state().config_apply_pending.load(Relaxed) & crate::state::tmc::PENDING_CURRENT,
                0,
                "current bit should be cleared after a successful apply"
            );

            tmc_state().run_current_ma_cfg.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_encoder_dir_bit_writes_abn_decoder_mode() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::ABN_DECODER_MODE;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let config = crate::config::TmcConfig::default_for_test();

            tmc_state().encoder_invert_cfg.store(Some(true), Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_ENCODER_DIR, Relaxed);

            let raw_spi = MockRawSpi::default();
            let mut tmc = Tmc4671Spi::new(raw_spi);

            let mut stored_inner_gains = None;
            drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await
            .expect("apply should succeed");

            let mode = 1u32 << 12;
            let raw_spi = tmc.into_inner();
            let frame = |value: u32| -> std::vec::Vec<u8> {
                let bytes = value.to_be_bytes();
                std::vec![
                    ABN_DECODER_MODE.addr() | 0x80,
                    bytes[0],
                    bytes[1],
                    bytes[2],
                    bytes[3]
                ]
            };
            assert!(
                raw_spi.writes.contains(&frame(mode)),
                "expected an ABN_DECODER_MODE write of {mode:#x}, got {:?}",
                raw_spi.writes
            );
            assert_eq!(
                tmc_state().config_apply_pending.load(Relaxed)
                    & crate::state::tmc::PENDING_ENCODER_DIR,
                0,
                "encoder-dir bit should be cleared after a successful apply"
            );

            tmc_state().encoder_invert_cfg.store(None, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_inner_gains_bit_publishes_into_sink_on_success() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::{PID_FLUX_P_FLUX_I, PID_TORQUE_P_TORQUE_I};

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let config = crate::config::TmcConfig::default_for_test();

            tmc_state().mark_pi_representation_established();
            let gains = InnerLoopGains {
                flux_p: crate::pid::p_gain(11),
                flux_i: crate::pid::i_gain(12),
                torque_p: crate::pid::p_gain(13),
                torque_i: crate::pid::i_gain(14),
            };
            tmc_state().store_pending_inner_gains(gains, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_INNER_GAINS, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await
            .expect("apply should succeed");

            let spi = tmc.into_inner();
            assert!(spi.attempted_write_to(PID_FLUX_P_FLUX_I));
            assert!(spi.attempted_write_to(PID_TORQUE_P_TORQUE_I));
            assert_eq!(stored_inner_gains, Some(gains));
            assert_eq!(
                tmc_state().config_apply_pending.load(Relaxed)
                    & crate::state::tmc::PENDING_INNER_GAINS,
                0,
                "inner-gains bit should be cleared after a successful apply"
            );

            tmc_state().clear_pi_representation_established();
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_inner_gains_bit_leaves_sink_untouched_on_write_failure() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::PID_FLUX_P_FLUX_I;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let config = crate::config::TmcConfig::default_for_test();

            tmc_state().mark_pi_representation_established();
            let previous = InnerLoopGains {
                flux_p: crate::pid::p_gain(1),
                flux_i: crate::pid::i_gain(2),
                torque_p: crate::pid::p_gain(3),
                torque_i: crate::pid::i_gain(4),
            };
            let requested = InnerLoopGains {
                flux_p: crate::pid::p_gain(11),
                flux_i: crate::pid::i_gain(12),
                torque_p: crate::pid::p_gain(13),
                torque_i: crate::pid::i_gain(14),
            };
            tmc_state().store_pending_inner_gains(requested, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_INNER_GAINS, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12)
                .fail_writes_to(PID_FLUX_P_FLUX_I.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains = Some(previous);
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::InnerGains),
                "a failed inner-gain write must name the inner-gains item"
            );
            assert_eq!(stored_inner_gains, Some(previous));
            assert_ne!(
                tmc_state().config_apply_pending.load(Relaxed)
                    & crate::state::tmc::PENDING_INNER_GAINS,
                0,
                "inner-gains bit should stay set for retry after a failed apply"
            );

            tmc_state().clear_pi_representation_established();
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_outer_gains_bit_publishes_on_success() {
        use crate::pid::OuterLoopGains;
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::{PID_POSITION_P_POSITION_I, PID_VELOCITY_P_VELOCITY_I};

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let config = crate::config::TmcConfig::default_for_test();

            tmc_state().mark_pi_representation_established();
            let gains = OuterLoopGains {
                position_p: crate::pid::p_gain(11),
                position_i: crate::pid::i_gain(12),
                velocity_p: crate::pid::p_gain(13),
                velocity_i: crate::pid::i_gain(14),
            };
            tmc_state().store_pending_outer_gains(gains, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_OUTER_GAINS, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await
            .expect("apply should succeed");

            let spi = tmc.into_inner();
            assert!(spi.attempted_write_to(PID_VELOCITY_P_VELOCITY_I));
            assert!(spi.attempted_write_to(PID_POSITION_P_POSITION_I));
            assert_eq!(tmc_state().load_outer_gains(Relaxed), Ok(gains));
            assert_eq!(
                tmc_state().config_apply_pending.load(Relaxed)
                    & crate::state::tmc::PENDING_OUTER_GAINS,
                0,
                "outer-gains bit should be cleared after a successful apply"
            );

            tmc_state().clear_pi_representation_established();
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_with_outer_gains_bit_leaves_outer_gains_unchanged_on_write_failure() {
        use crate::pid::OuterLoopGains;
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            let config = crate::config::TmcConfig::default_for_test();

            tmc_state().mark_pi_representation_established();
            let previous = OuterLoopGains {
                position_p: crate::pid::p_gain(1),
                position_i: crate::pid::i_gain(2),
                velocity_p: crate::pid::p_gain(3),
                velocity_i: crate::pid::i_gain(4),
            };
            let requested = OuterLoopGains {
                position_p: crate::pid::p_gain(11),
                position_i: crate::pid::i_gain(12),
                velocity_p: crate::pid::p_gain(13),
                velocity_i: crate::pid::i_gain(14),
            };
            tmc_state().store_outer_gains(previous, Relaxed);
            tmc_state().store_pending_outer_gains(requested, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_OUTER_GAINS, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12)
                .fail_writes_to(PID_VELOCITY_P_VELOCITY_I.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::OuterGains),
                "a failed outer-gain write must name the outer-gains item"
            );
            assert_eq!(
                tmc_state().load_outer_gains(Relaxed),
                Ok(previous),
                "load_outer_gains must stay untouched after a failed apply"
            );
            assert_ne!(
                tmc_state().config_apply_pending.load(Relaxed)
                    & crate::state::tmc::PENDING_OUTER_GAINS,
                0,
                "outer-gains bit should stay set for retry after a failed apply"
            );

            tmc_state().clear_pi_representation_established();
            tmc_state().config_apply_pending.store(0, Relaxed);
            tmc_state().store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_names_the_voltage_limit_item_on_write_failure() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            tmc_state().voltage_limit_cfg.store(8_000, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_VOLTAGE_LIMIT, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                crate::config::TmcConfig::default_for_test(),
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::VoltageLimit),
                "a failed voltage-limit write must name the voltage-limit item"
            );

            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_names_the_current_item_on_write_failure() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::PID_TORQUE_FLUX_LIMITS;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            tmc_state().run_current_ma_cfg.store(1500, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_CURRENT, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                crate::config::TmcConfig::default_for_test(),
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::Current),
                "a failed current write must name the current item"
            );

            tmc_state().run_current_ma_cfg.store(0, Relaxed);
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_names_the_encoder_dir_item_on_write_failure() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::ABN_DECODER_MODE;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            tmc_state().encoder_invert_cfg.store(Some(true), Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_ENCODER_DIR, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .fail_writes_to(ABN_DECODER_MODE.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                crate::config::TmcConfig::default_for_test(),
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::EncoderDir),
                "a failed encoder-direction write must name the encoder-direction item"
            );

            tmc_state().encoder_invert_cfg.store(None, Relaxed);
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_names_the_filters_item_on_write_failure() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::CONFIG_BIQUAD_V_A_1;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            tmc_state().velocity_filter_hz.store(Some(100), Relaxed);
            tmc_state().torque_filter_hz.store(Some(0), Relaxed);
            tmc_state().position_filter_hz.store(Some(0), Relaxed);
            tmc_state().flux_filter_hz.store(Some(0), Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_FILTERS, Relaxed);

            let spi = crate::tmc_control::test_spi::RuntimeMockSpi::default()
                .fail_writes_to(CONFIG_BIQUAD_V_A_1.data.addr());
            let mut tmc = Tmc4671Spi::new(spi);

            let mut config = crate::config::TmcConfig::default_for_test();
            config.pwm_maxcnt = 4095;
            let mut stored_inner_gains: Option<InnerLoopGains> = None;
            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                config,
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(
                result,
                Err(PendingConfigItem::Filters),
                "a failed filter write must name the filters item"
            );

            tmc_state().velocity_filter_hz.store(None, Relaxed);
            tmc_state().torque_filter_hz.store(None, Relaxed);
            tmc_state().position_filter_hz.store(None, Relaxed);
            tmc_state().flux_filter_hz.store(None, Relaxed);
            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }

    #[test]
    fn drain_pending_config_clears_the_encoder_dir_bit_when_no_direction_is_recorded() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::ABN_DECODER_MODE;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
        pollster::block_on(async {
            // `TmcChannelState::reset` nulls `encoder_invert_cfg` without
            // touching `config_apply_pending`, so a deferred queue flush can
            // leave this pairing behind.
            tmc_state().encoder_invert_cfg.store(None, Relaxed);
            tmc_state()
                .config_apply_pending
                .store(crate::state::tmc::PENDING_ENCODER_DIR, Relaxed);
            let mut tmc = Tmc4671Spi::new(MockRawSpi::default());
            let mut stored_inner_gains = None;

            let result = drain_pending_config(
                &mut tmc,
                tmc_state(),
                crate::config::TmcConfig::default_for_test(),
                PendingConfigSink {
                    stored_inner_gains: &mut stored_inner_gains,
                },
            )
            .await;

            assert_eq!(result, Ok(()));
            assert_eq!(
                tmc_state().config_apply_pending.load(Relaxed),
                0,
                "nothing to write is itself a drained item; leaving the bit set \
                 strands it and misreports the drain as incomplete"
            );
            let spi = tmc.into_inner();
            assert!(
                !spi.writes
                    .iter()
                    .any(|frame| frame[0] & 0x7F == ABN_DECODER_MODE.addr()),
                "with no recorded direction there is nothing to write"
            );

            tmc_state().config_apply_pending.store(0, Relaxed);
        });
    }
}
