use crate::commissioning::encoder_domain::EncoderDomain;
use crate::tmc::{TmcRequest, TmcRequestKind, TorqueSampleKind};

use super::helpers::{RunnerFailure, checked_delay_ms};
use super::helpers::{clear_current_step_integrators, split_signed_halves};
use super::{
    CURRENT_STEP_STATUS_HOST_LIVENESS, CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI,
    CurrentTorqueSampleReport, DiagnosticContext, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReport, empty_current_torque_sample_report,
};

const CURRENT_STEP_TORQUE_MODE: u32 = 0x8000_0001;
const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_SETTLE_MS: u64 = 20;

fn handled_torque_sample(report: CurrentTorqueSampleReport) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::CurrentTorqueSample(report))
}

// `Option::as_deref_mut()` reborrowed inline at each `checked_delay_ms` call
// site, across intervening `.await`s in the same async fn, hits a rustc
// borrow-checker limitation with `&mut dyn Trait` reborrows spanning
// suspension points (verified with a minimal repro outside this crate). An
// explicit, non-generic reborrow function sidesteps it.
fn reborrow_liveness_check<'a>(
    check: &'a mut Option<&mut dyn FnMut() -> bool>,
) -> Option<&'a mut dyn FnMut() -> bool> {
    match check {
        Some(f) => Some(&mut **f),
        None => None,
    }
}

/// Run a current-torque-sample diagnostic request.
pub async fn run_current_torque_sample_request<B, D>(
    req: TmcRequest,
    ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let DiagnosticContext {
        tmc,
        runtime,
        delay,
        readiness,
        mut host_liveness_expired,
        ..
    } = ctx;
    let tmc_state =
        crate::state::tmc::tmc_channel(runtime.channel()).expect("validated TMC channel");

    match req {
        TmcRequest {
            kind:
                TmcRequestKind::CurrentTorqueSampleTest {
                    oid,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
                },
            ..
        } => {
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || readiness.homing_active
                || readiness.stepper_active
            {
                return handled_torque_sample(empty_current_torque_sample_report(
                    oid,
                    TorqueSampleKind::CurrentTorqueSample,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
                ));
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_torque_sample(empty_current_torque_sample_report(
                        oid,
                        TorqueSampleKind::CurrentTorqueSample,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        flux_target,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                }
            };
            let saved_voltage_limit = readiness.saved_voltage_limit;
            let position_actual = match tmc
                .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_torque_sample(empty_current_torque_sample_report(
                        oid,
                        TorqueSampleKind::CurrentTorqueSample,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        flux_target,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                }
            };

            let mut status = 0u8;
            let mut torque_before = 0i16;
            let mut torque_sample = 0i16;
            let mut torque_after = 0i16;
            let mut flux_sample = 0i16;
            let mut iq_sample = 0i16;
            let mut id_sample = 0i16;
            let mut uq_limited = 0i16;
            let mut ud_limited = 0i16;
            let mut encoder_before = 0u32;
            let mut encoder_sample = 0u32;
            let mut encoder_after = 0u32;
            let mut encoder_delta_sample = 0i32;
            let mut encoder_delta_after = 0i32;
            let mut adc_vm_raw = 0u16;
            let mut pidin_target_torque = 0i32;
            let mut pidin_target_flux = 0i32;
            let mut pidout_target_torque = 0i32;
            let mut pidout_target_flux = 0i32;
            let mut pid_torque_target_monitor = 0i16;
            let mut torque_error = 0i32;
            let mut flux_error = 0i32;
            let mut torque_error_sum = 0i32;
            let mut flux_error_sum = 0i32;
            let mut uq_prelimit = 0i16;
            let mut ud_prelimit = 0i16;
            let mut ff_velocity = 0i32;
            let mut ff_torque = 0i16;
            let mut status_flags = 0u32;

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before torque sample");
                return handled_torque_sample(empty_current_torque_sample_report(
                    oid,
                    TorqueSampleKind::CurrentTorqueSample,
                    CURRENT_STEP_STATUS_SPI,
                    target,
                    flux_target,
                    sample_delay_ms,
                    voltage_limit,
                ));
            }

            let result: Result<(), RunnerFailure> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                tmc.write_register(PIDOUT_UQ_UD_LIMITS, u32::from(voltage_limit))
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                clear_current_step_integrators(tmc)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                crate::tmc_control::interlock::clear_status_flags(tmc, tmc_state)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_TORQUE_MODE)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;

                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    CURRENT_STEP_SETTLE_MS,
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                encoder_before = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let encoder_counts_per_rev = tmc
                    .read_register(ABN_DECODER_PPR)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let encoder_domain =
                    EncoderDomain::new(encoder_counts_per_rev).map_err(|_| RunnerFailure::Other)?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                torque_before = crate::tmc::decode_signed_upper_half(before);

                let target_reg = crate::tmc::pack_torque_flux_target(target, flux_target);
                tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    u64::from(sample_delay_ms),
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                pidin_target_torque =
                    tmc.read_sub_register(INTERIM_PIDIN_TARGET_TORQUE)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                pidin_target_flux = tmc
                    .read_sub_register(INTERIM_PIDIN_TARGET_FLUX)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                pidout_target_torque =
                    tmc.read_sub_register(INTERIM_PIDOUT_TARGET_TORQUE)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                pidout_target_flux = tmc
                    .read_sub_register(INTERIM_PIDOUT_TARGET_FLUX)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                let torque_target_actual = tmc
                    .read_sub_register(INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                pid_torque_target_monitor =
                    crate::tmc::decode_signed_upper_half(torque_target_actual);
                torque_error = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                flux_error = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                torque_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                flux_error_sum = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                status_flags = tmc
                    .read_register(STATUS_FLAGS)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;

                encoder_sample = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                encoder_delta_sample = encoder_domain
                    .delta(
                        encoder_domain.count(encoder_before),
                        encoder_domain.count(encoder_sample),
                    )
                    .map_err(|_| RunnerFailure::Other)?
                    .signed_counts();

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (torque, flux) = split_signed_halves(actual);
                torque_sample = torque;
                flux_sample = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (iq, id) = split_signed_halves(iq_id);
                iq_sample = iq;
                id_sample = id;

                let uq_ud_prelimit = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (uq, ud) = split_signed_halves(uq_ud_prelimit);
                uq_prelimit = uq;
                ud_prelimit = ud;
                ff_velocity = tmc
                    .read_sub_register(INTERIM_FF_VELOCITY)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                ff_torque = tmc
                    .read_sub_register(INTERIM_FF_TORQUE)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i16;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (uq, ud) = split_signed_halves(uq_ud);
                uq_limited = uq;
                ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as u16;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    CURRENT_STEP_SETTLE_MS,
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                encoder_after = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                encoder_delta_after = encoder_domain
                    .delta(
                        encoder_domain.count(encoder_before),
                        encoder_domain.count(encoder_after),
                    )
                    .map_err(|_| RunnerFailure::Other)?
                    .signed_counts();

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                torque_after = crate::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if let Err(failure) = result {
                status = match failure {
                    RunnerFailure::Other => CURRENT_STEP_STATUS_SPI,
                    RunnerFailure::HostLiveness => CURRENT_STEP_STATUS_HOST_LIVENESS,
                };
            }

            let _ = tmc
                .write_register(foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET, 0)
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS,
                    saved_voltage_limit,
                )
                .await;
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PID_POSITION_TARGET,
                    position_actual,
                )
                .await;
            let restore_mode = if saved_mode == 0 {
                CURRENT_STEP_POSITION_MODE
            } else {
                saved_mode
            };
            let _ = tmc
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, restore_mode)
                .await;

            let report = CurrentTorqueSampleReport {
                oid,
                kind: TorqueSampleKind::CurrentTorqueSample,
                status,
                target,
                flux_target,
                sample_delay_ms,
                voltage_limit,
                torque_before,
                torque_sample,
                torque_after,
                flux_sample,
                iq_sample,
                id_sample,
                uq_limited,
                ud_limited,
                encoder_before,
                encoder_sample,
                encoder_after,
                encoder_delta_sample,
                encoder_delta_after,
                adc_vm_raw,
                pidin_target_torque,
                pidin_target_flux,
                pidout_target_torque,
                pidout_target_flux,
                pid_torque_target_monitor,
                torque_error,
                flux_error,
                torque_error_sum,
                flux_error_sum,
                uq_prelimit,
                ud_prelimit,
                ff_velocity,
                ff_torque,
                status_flags,
            };
            handled_torque_sample(report)
        }
        req => DiagnosticOutcome::NotDiagnostic(req),
    }
}

/// Run a position torque-offset sample diagnostic request.
pub async fn run_position_torque_offset_sample_request<B, D>(
    req: TmcRequest,
    ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let DiagnosticContext {
        tmc,
        runtime,
        delay,
        readiness,
        mut host_liveness_expired,
        ..
    } = ctx;
    let tmc_state =
        crate::state::tmc::tmc_channel(runtime.channel()).expect("validated TMC channel");

    match req {
        TmcRequest {
            kind:
                TmcRequestKind::PositionTorqueOffsetSampleTest {
                    oid,
                    target,
                    sample_delay_ms,
                    voltage_limit,
                },
            ..
        } => {
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || readiness.homing_active
                || readiness.stepper_active
            {
                return handled_torque_sample(empty_current_torque_sample_report(
                    oid,
                    TorqueSampleKind::PositionTorqueOffset,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    0,
                    sample_delay_ms,
                    voltage_limit,
                ));
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_torque_sample(empty_current_torque_sample_report(
                        oid,
                        TorqueSampleKind::PositionTorqueOffset,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        0,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                }
            };
            if saved_mode != CURRENT_STEP_POSITION_MODE {
                return handled_torque_sample(empty_current_torque_sample_report(
                    oid,
                    TorqueSampleKind::PositionTorqueOffset,
                    CURRENT_STEP_STATUS_NOT_READY,
                    target,
                    0,
                    sample_delay_ms,
                    voltage_limit,
                ));
            }

            let saved_voltage_limit = readiness.saved_voltage_limit;
            let saved_offset = match tmc
                .read_register(foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_torque_sample(empty_current_torque_sample_report(
                        oid,
                        TorqueSampleKind::PositionTorqueOffset,
                        CURRENT_STEP_STATUS_SPI,
                        target,
                        0,
                        sample_delay_ms,
                        voltage_limit,
                    ));
                }
            };

            let mut report = empty_current_torque_sample_report(
                oid,
                TorqueSampleKind::PositionTorqueOffset,
                0,
                target,
                0,
                sample_delay_ms,
                voltage_limit,
            );

            let result: Result<(), RunnerFailure> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PIDOUT_UQ_UD_LIMITS, u32::from(voltage_limit))
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                crate::tmc_control::interlock::clear_status_flags(tmc, tmc_state)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;

                report.encoder_before = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let encoder_counts_per_rev = tmc
                    .read_register(ABN_DECODER_PPR)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let encoder_domain =
                    EncoderDomain::new(encoder_counts_per_rev).map_err(|_| RunnerFailure::Other)?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                report.torque_before = crate::tmc::decode_signed_upper_half(before);

                let offset_reg = u32::from(target as u16) << 16;
                tmc.write_register(PID_TORQUE_FLUX_OFFSET, offset_reg)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    u64::from(sample_delay_ms),
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                report.pidin_target_torque =
                    tmc.read_sub_register(INTERIM_PIDIN_TARGET_TORQUE)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.pidin_target_flux =
                    tmc.read_sub_register(INTERIM_PIDIN_TARGET_FLUX)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.pidout_target_torque =
                    tmc.read_sub_register(INTERIM_PIDOUT_TARGET_TORQUE)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.pidout_target_flux =
                    tmc.read_sub_register(INTERIM_PIDOUT_TARGET_FLUX)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                let torque_target_actual = tmc
                    .read_sub_register(INTERIM_PID_TORQUE_TARGET_TORQUE_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                report.pid_torque_target_monitor =
                    crate::tmc::decode_signed_upper_half(torque_target_actual);
                report.torque_error =
                    tmc.read_sub_register(PID_ERROR_PID_TORQUE_ERROR)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.flux_error = tmc
                    .read_sub_register(PID_ERROR_PID_FLUX_ERROR)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                report.torque_error_sum =
                    tmc.read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.flux_error_sum =
                    tmc.read_sub_register(PID_ERROR_PID_FLUX_ERROR_SUM)
                        .await
                        .map_err(|_| RunnerFailure::Other)? as i32;
                report.status_flags = tmc
                    .read_register(STATUS_FLAGS)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;

                report.encoder_sample = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                report.encoder_delta_sample = encoder_domain
                    .delta(
                        encoder_domain.count(report.encoder_before),
                        encoder_domain.count(report.encoder_sample),
                    )
                    .map_err(|_| RunnerFailure::Other)?
                    .signed_counts();

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (torque, flux) = split_signed_halves(actual);
                report.torque_sample = torque;
                report.flux_sample = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (iq, id) = split_signed_halves(iq_id);
                report.iq_sample = iq;
                report.id_sample = id;

                let uq_ud_prelimit = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (uq, ud) = split_signed_halves(uq_ud_prelimit);
                report.uq_prelimit = uq;
                report.ud_prelimit = ud;
                report.ff_velocity = tmc
                    .read_sub_register(INTERIM_FF_VELOCITY)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i32;
                report.ff_torque = tmc
                    .read_sub_register(INTERIM_FF_TORQUE)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as i16;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (uq, ud) = split_signed_halves(uq_ud);
                report.uq_limited = uq;
                report.ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                report.adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| RunnerFailure::Other)? as u16;

                tmc.write_register(PID_TORQUE_FLUX_OFFSET, saved_offset)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    CURRENT_STEP_SETTLE_MS,
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                report.encoder_after = tmc
                    .read_register(ABN_DECODER_COUNT)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                report.encoder_delta_after = encoder_domain
                    .delta(
                        encoder_domain.count(report.encoder_before),
                        encoder_domain.count(report.encoder_after),
                    )
                    .map_err(|_| RunnerFailure::Other)?
                    .signed_counts();

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                report.torque_after = crate::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if let Err(failure) = result {
                report.status = match failure {
                    RunnerFailure::Other => CURRENT_STEP_STATUS_SPI,
                    RunnerFailure::HostLiveness => CURRENT_STEP_STATUS_HOST_LIVENESS,
                };
            }

            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PID_TORQUE_FLUX_OFFSET,
                    saved_offset,
                )
                .await;
            runtime.invalidate_torque_flux_offset_cache();
            let _ = tmc
                .write_register(
                    foci_tmc4671::registers::PIDOUT_UQ_UD_LIMITS,
                    saved_voltage_limit,
                )
                .await;
            let _ = tmc
                .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, saved_mode)
                .await;

            handled_torque_sample(report)
        }
        req => DiagnosticOutcome::NotDiagnostic(req),
    }
}
