use crate::tmc::{TmcRequest, TmcRequestKind};

use super::helpers::{current_step_encoder_delta, pack_voltage_ext, split_signed_halves};
use super::{
    CURRENT_STEP_STATUS_NOT_READY, CURRENT_STEP_STATUS_SPI, DiagnosticContext, DiagnosticDelay,
    DiagnosticOutcome, DiagnosticReport, empty_voltage_step_report,
};

const CURRENT_STEP_POSITION_MODE: u32 = 0x8000_0003;
const CURRENT_STEP_OPEN_LOOP_VOLTAGE_MODE: u32 = 0x0000_0008;
const CURRENT_STEP_SETTLE_MS: u64 = 20;

fn handled_voltage_step(report: super::VoltageStepReport) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::VoltageStep(report))
}

/// Run an open-loop voltage-step diagnostic request.
pub async fn run_voltage_step_request<B, D>(
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
                TmcRequestKind::VoltageStepTest {
                    oid,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                },
            ..
        } => {
            if !runtime.is_motor_enabled()
                || !runtime.is_calibrated()
                || readiness.homing_active
                || readiness.stepper_active
            {
                return handled_voltage_step(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
            }

            let saved_mode = match tmc
                .read_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION)
                .await
            {
                Ok(value) => value,
                Err(_) => {
                    return handled_voltage_step(empty_voltage_step_report(
                        oid,
                        CURRENT_STEP_STATUS_SPI,
                        uq_ext,
                        ud_ext,
                        sample_delay_ms,
                    ));
                }
            };
            if saved_mode != CURRENT_STEP_POSITION_MODE {
                return handled_voltage_step(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_NOT_READY,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
            }

            let mut report = empty_voltage_step_report(oid, 0, uq_ext, ud_ext, sample_delay_ms);

            if !runtime.restore_phase_advance_now(tmc).await {
                defmt::warn!("phase advance: failed to restore ABN offset before voltage step");
                return handled_voltage_step(empty_voltage_step_report(
                    oid,
                    CURRENT_STEP_STATUS_SPI,
                    uq_ext,
                    ud_ext,
                    sample_delay_ms,
                ));
            }

            let result: Result<(), ()> = async {
                use foci_tmc4671::registers::*;

                report.encoder_before =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                let encoder_counts_per_rev =
                    tmc.read_register(ABN_DECODER_PPR).await.map_err(|_| ())?;

                let before = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.torque_before = crate::tmc::decode_signed_upper_half(before);

                tmc.write_register(STATUS_FLAGS, 0).await.map_err(|_| ())?;
                tmc.write_register(UQ_UD_EXT, pack_voltage_ext(uq_ext, ud_ext))
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, CURRENT_STEP_OPEN_LOOP_VOLTAGE_MODE)
                    .await
                    .map_err(|_| ())?;

                delay.delay_ms(u64::from(sample_delay_ms)).await;

                report.encoder_sample =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                report.encoder_delta_sample = current_step_encoder_delta(
                    report.encoder_before,
                    report.encoder_sample,
                    encoder_counts_per_rev,
                );

                let actual = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                let (torque, flux) = split_signed_halves(actual);
                report.torque_sample = torque;
                report.flux_sample = flux;

                let iq_id = tmc
                    .read_sub_register(INTERIM_FOC_IQ_ID)
                    .await
                    .map_err(|_| ())?;
                let (iq, id) = split_signed_halves(iq_id);
                report.iq_sample = iq;
                report.id_sample = id;

                let uq_ud = tmc
                    .read_sub_register(INTERIM_FOC_UQ_UD_LIMITED)
                    .await
                    .map_err(|_| ())?;
                let (uq, ud) = split_signed_halves(uq_ud);
                report.uq_limited = uq;
                report.ud_limited = ud;

                // TMC4671-LA Rev 1, 02/2024, INTERIM_DATA selector 15 packs
                // FOC_UWY in bits 31:16 and FOC_UUX in bits 15:0.
                let uwy_uux = tmc
                    .read_sub_register(INTERIM_FOC_UWY_UUX)
                    .await
                    .map_err(|_| ())?;
                let (uwy, uux) = split_signed_halves(uwy_uux);
                report.uwy_sample = uwy;
                report.uux_sample = uux;

                // INTERIM_DATA selector 17 packs PWM_WY in bits 31:16 and
                // PWM_UX in bits 15:0.
                let pwm_wy_ux = tmc
                    .read_sub_register(INTERIM_PWM_WY_UX)
                    .await
                    .map_err(|_| ())?;
                let (pwm_wy, pwm_ux) = split_signed_halves(pwm_wy_ux);
                report.pwm_wy_sample = pwm_wy;
                report.pwm_ux_sample = pwm_ux;

                report.pwm_sv_chop = tmc.read_register(PWM_SV_CHOP).await.map_err(|_| ())?;
                report.pwm_bbm = tmc.read_register(PWM_BBM_H_BBM_L).await.map_err(|_| ())?;
                report.pwm_maxcnt = tmc.read_register(PWM_MAXCNT).await.map_err(|_| ())?;

                // ABN_DECODER_PHI_E_PHI_M packs PHI_E in bits 31:16 and
                // PHI_M in bits 15:0.
                let phi_e_phi_m = tmc
                    .read_register(ABN_DECODER_PHI_E_PHI_M)
                    .await
                    .map_err(|_| ())?;
                let (phi_e, phi_m) = split_signed_halves(phi_e_phi_m);
                report.phi_e_sample = phi_e;
                report.phi_m_sample = phi_m;

                // TMC4671-LA Rev 1, 02/2024, ADC_RAW_DATA 0x02 selected by
                // ADC_RAW_ADDR 0x03: selector 1 packs ADC_AGPI_A_RAW in
                // bits 31:16 and ADC_VM_RAW in bits 15:0.
                report.adc_vm_raw = tmc
                    .read_sub_register(ADC_AGPI_A_RAW_ADC_VM_RAW)
                    .await
                    .map_err(|_| ())? as u16;
                report.status_flags = tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;

                tmc.write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .map_err(|_| ())?;
                tmc.write_register(UQ_UD_EXT, 0).await.map_err(|_| ())?;
                tmc.write_register(MODE_RAMP_MODE_MOTION, saved_mode)
                    .await
                    .map_err(|_| ())?;
                delay.delay_ms(CURRENT_STEP_SETTLE_MS).await;

                report.encoder_after =
                    tmc.read_register(ABN_DECODER_COUNT).await.map_err(|_| ())?;
                report.encoder_delta_after = current_step_encoder_delta(
                    report.encoder_before,
                    report.encoder_after,
                    encoder_counts_per_rev,
                );

                let after = tmc
                    .read_register(PID_TORQUE_FLUX_ACTUAL)
                    .await
                    .map_err(|_| ())?;
                report.torque_after = crate::tmc::decode_signed_upper_half(after);
                Ok(())
            }
            .await;

            if result.is_err() {
                report.status = CURRENT_STEP_STATUS_SPI;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, 0)
                    .await;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::UQ_UD_EXT, 0)
                    .await;
                let _ = tmc
                    .write_register(foci_tmc4671::registers::MODE_RAMP_MODE_MOTION, saved_mode)
                    .await;
            }

            handled_voltage_step(report)
        }
        req => DiagnosticOutcome::NotDiagnostic(req),
    }
}
