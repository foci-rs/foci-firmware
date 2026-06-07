use crate::tmc::{TmcRequest, TmcRequestKind};

use super::helpers::{
    clear_current_step_integrators, current_step_encoder_delta, split_signed_halves,
};
use super::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, CurrentStepReport, DiagnosticContext,
    DiagnosticDelay, DiagnosticOutcome, DiagnosticReport,
};

const CURRENT_STEP_TORQUE_MODE: u32 = 0x8000_0001;
const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_SETTLE_MS: u64 = 20;

fn empty_current_step_report(
    oid: u8,
    status: u8,
    torque_target: i16,
    flux_target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) -> CurrentStepReport {
    CurrentStepReport {
        oid,
        status,
        torque_target,
        flux_target,
        duration_ms,
        voltage_limit,
        torque_before: 0,
        torque_during: 0,
        torque_after: 0,
        flux_during: 0,
        iq_during: 0,
        id_during: 0,
        uq_limited: 0,
        ud_limited: 0,
        encoder_before: 0,
        encoder_after: 0,
        encoder_delta: 0,
        adc_vm_raw: 0,
    }
}

fn handled_current_step(vector_reply: bool, report: CurrentStepReport) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep {
        vector_reply,
        report,
    })
}

/// Run a current-step or current-vector-step diagnostic request.
pub async fn run_current_step_request<B, D>(
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
    } = ctx;

    match req {
        TmcRequest {
            kind:
                current_step_kind @ (TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }),
            ..
        } => {
            let (oid, torque_target, flux_target, duration_ms, voltage_limit, vector_reply) =
                match current_step_kind {
                    TmcRequestKind::CurrentStepTest {
                        oid,
                        target,
                        duration_ms,
                        voltage_limit,
                    } => (oid, target, 0, duration_ms, voltage_limit, false),
                    TmcRequestKind::CurrentVectorStepTest {
                        oid,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                    } => (
                        oid,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                        true,
                    ),
                    _ => unreachable!(),
                };
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || readiness.homing_active
                || readiness.stepper_active
            {
                return handled_current_step(
                    vector_reply,
                    empty_current_step_report(
                        oid,
                        CURRENT_STEP_STATUS_NOT_READY,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                    ),
                );
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_current_step(
                        vector_reply,
                        empty_current_step_report(
                            oid,
                            CURRENT_STEP_STATUS_SPI,
                            torque_target,
                            flux_target,
                            duration_ms,
                            voltage_limit,
                        ),
                    );
                }
            };
            let saved_voltage_limit = readiness.saved_voltage_limit;
            let position_actual = match tmc
                .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_current_step(
                        vector_reply,
                        empty_current_step_report(
                            oid,
                            CURRENT_STEP_STATUS_SPI,
                            torque_target,
                            flux_target,
                            duration_ms,
                            voltage_limit,
                        ),
                    );
                }
            };

            let mut status = 0u8;
            let mut torque_before = 0i16;
            let mut torque_during = 0i16;
            let mut torque_after = 0i16;
            let mut flux_during = 0i16;
            let mut iq_during = 0i16;
            let mut id_during = 0i16;
            let mut uq_limited = 0i16;
            let mut ud_limited = 0i16;
            let mut encoder_before = 0u32;
            let mut encoder_after = 0u32;
            let mut encoder_delta = 0i32;
            let mut adc_vm_raw = 0u16;

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before current step");
                return handled_current_step(
                    vector_reply,
                    CurrentStepReport {
                        oid,
                        status: CURRENT_STEP_STATUS_SPI,
                        torque_target,
                        flux_target,
                        duration_ms,
                        voltage_limit,
                        torque_before,
                        torque_during,
                        torque_after,
                        flux_during,
                        iq_during,
                        id_during,
                        uq_limited,
                        ud_limited,
                        encoder_before,
                        encoder_after,
                        encoder_delta,
                        adc_vm_raw,
                    },
                );
            }

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(PIDOUT_UQ_UD_LIMITS, u32::from(voltage_limit))
                    .await
                    .map_err(|_| ())?;
                clear_current_step_integrators(tmc).await?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_TORQUE_MODE)
                    .await
                    .map_err(|_| ())?;

                delay.delay_ms(CURRENT_STEP_SETTLE_MS).await;

                encoder_before = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_before = crate::tmc::decode_signed_upper_half(before);

                let target_reg = crate::tmc::pack_torque_flux_target(torque_target, flux_target);
                tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg)
                    .await
                    .map_err(|_| ())?;
                delay.delay_ms(u64::from(duration_ms)).await;

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                let (torque, flux) = split_signed_halves(actual);
                torque_during = torque;
                flux_during = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| ())?;
                let (iq, id) = split_signed_halves(iq_id);
                iq_during = iq;
                id_during = id;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                uq_limited = uq;
                ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;

                tmc.write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .map_err(|_| ())?;
                delay.delay_ms(CURRENT_STEP_SETTLE_MS).await;

                encoder_after = tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                encoder_delta = current_step_encoder_delta(
                    encoder_before,
                    encoder_after,
                    encoder_counts_per_rev,
                );

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                torque_after = crate::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if result.is_err() {
                status = CURRENT_STEP_STATUS_SPI;
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

            handled_current_step(
                vector_reply,
                CurrentStepReport {
                    oid,
                    status,
                    torque_target,
                    flux_target,
                    duration_ms,
                    voltage_limit,
                    torque_before,
                    torque_during,
                    torque_after,
                    flux_during,
                    iq_during,
                    id_during,
                    uq_limited,
                    ud_limited,
                    encoder_before,
                    encoder_after,
                    encoder_delta,
                    adc_vm_raw,
                },
            )
        }
        req => DiagnosticOutcome::NotDiagnostic(req),
    }
}
