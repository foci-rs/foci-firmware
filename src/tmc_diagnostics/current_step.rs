use crate::commissioning::encoder_domain::EncoderDomain;
use crate::tmc::{CurrentStepAxis, TmcRequest, TmcRequestKind};

use super::helpers::{RunnerFailure, checked_delay_ms};
use super::helpers::{clear_current_step_integrators, split_signed_halves};
use super::{
    CURRENT_STEP_STATUS_HOST_LIVENESS, CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI,
    CurrentStepReply, CurrentStepReport, DiagnosticContext, DiagnosticDelay, DiagnosticOutcome,
    DiagnosticReport,
};

const CURRENT_STEP_TORQUE_MODE: u32 = 0x8000_0001;
const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_SETTLE_MS: u64 = 20;

pub(super) fn empty_current_step_report(
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

fn handled_current_step(reply: CurrentStepReply, report: CurrentStepReport) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::CurrentStep { reply, report })
}

fn axis_value(axis: CurrentStepAxis, torque: i16, flux: i16) -> i16 {
    match axis {
        CurrentStepAxis::Torque => torque,
        CurrentStepAxis::Flux => flux,
    }
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
        mut host_liveness_expired,
        ..
    } = ctx;

    match req {
        TmcRequest {
            kind:
                current_step_kind @ (TmcRequestKind::CurrentStepTest { .. }
                | TmcRequestKind::CurrentVectorStepTest { .. }),
            ..
        } => {
            let (
                oid,
                torque_target,
                flux_target,
                drive_torque_target,
                drive_flux_target,
                scalar_axis,
                duration_ms,
                voltage_limit,
                vector_reply,
            ) = match current_step_kind {
                TmcRequestKind::CurrentStepTest {
                    oid,
                    axis,
                    target,
                    duration_ms,
                    voltage_limit,
                } => {
                    let (drive_torque_target, drive_flux_target) = match axis {
                        CurrentStepAxis::Torque => (target, 0),
                        CurrentStepAxis::Flux => (0, target),
                    };
                    (
                        oid,
                        target,
                        drive_flux_target,
                        drive_torque_target,
                        drive_flux_target,
                        axis,
                        duration_ms,
                        voltage_limit,
                        false,
                    )
                }
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
                    torque_target,
                    flux_target,
                    CurrentStepAxis::Torque,
                    duration_ms,
                    voltage_limit,
                    true,
                ),
                _ => unreachable!(),
            };
            let reply = if vector_reply {
                CurrentStepReply::Vector
            } else {
                CurrentStepReply::Scalar(scalar_axis)
            };
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || readiness.homing_active
                || readiness.stepper_active
            {
                return handled_current_step(
                    reply,
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
                        reply,
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
                        reply,
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
                    reply,
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
                let (torque, flux) = split_signed_halves(before);
                torque_before = axis_value(scalar_axis, torque, flux);

                let target_reg =
                    crate::tmc::pack_torque_flux_target(drive_torque_target, drive_flux_target);
                tmc.write_register(PID_TORQUE_FLUX_TARGET, target_reg)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                checked_delay_ms(
                    delay,
                    reborrow_liveness_check(&mut host_liveness_expired),
                    u64::from(duration_ms),
                )
                .await
                .map_err(|_| RunnerFailure::HostLiveness)?;

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (torque, flux) = split_signed_halves(actual);
                torque_during = axis_value(scalar_axis, torque, flux);
                flux_during = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| RunnerFailure::Other)?;
                let (iq, id) = split_signed_halves(iq_id);
                iq_during = iq;
                id_during = id;

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
                encoder_delta = encoder_domain
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
                let (torque, flux) = split_signed_halves(after);
                torque_after = axis_value(scalar_axis, torque, flux);
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

            handled_current_step(
                reply,
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
