use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, MODE_RAMP_MODE_MOTION, MotionMode, PID_ERROR_PID_VELOCITY_ERROR_SUM,
    PID_TORQUE_FLUX_LIMITS, PID_VELOCITY_P_VELOCITY_I, PID_VELOCITY_TARGET, PidGains, Register,
    STATUS_FLAGS,
};
use foci_tmc4671::spi::Tmc4671Spi;

use crate::tmc::{TmcRequest, TmcRequestKind};

use super::{
    DiagnosticContext, DiagnosticDelay, DiagnosticOutcome, DiagnosticReport,
    VELOCITY_LIMIT_LATCH_STATUS_NOT_READY, VELOCITY_LIMIT_LATCH_STATUS_SPI,
    VelocityLimitLatchReport,
};

const TEST_LIMIT: u16 = 1;
const TEST_P_RAW: u16 = 512;
const TEST_I_RAW: u16 = 0;
const TEST_TARGET_RPM: i32 = 1;
const TEST_PULSE_US: u32 = 100;
const PERSISTENCE_DELAY_US: u32 = 1_000;
const PID_V_OUTPUT_LIMIT_STATUS: u32 = 1 << 7;
const PID_IQ_TARGET_LIMIT_STATUS: u32 = 1 << 12;
const EXPECTED_STIMULUS_STATUS: u32 = PID_V_OUTPUT_LIMIT_STATUS | PID_IQ_TARGET_LIMIT_STATUS;

const RESTORED_TARGET: u8 = 1 << 0;
const RESTORED_GAINS: u8 = 1 << 1;
const RESTORED_LIMIT: u8 = 1 << 2;
const RESTORED_MODE: u8 = 1 << 3;
const RESTORED_ALL: u8 = RESTORED_TARGET | RESTORED_GAINS | RESTORED_LIMIT | RESTORED_MODE;

#[derive(Default)]
struct SavedRegisters {
    mode: Option<u32>,
    limit: Option<u32>,
    gains: Option<u32>,
    target: Option<u32>,
}

impl SavedRegisters {
    fn copy_into(&self, report: &mut VelocityLimitLatchReport) {
        report.saved_mode = self.mode.unwrap_or(0);
        report.saved_limit = self.limit.unwrap_or(0);
        report.saved_gains = self.gains.unwrap_or(0);
        report.saved_target = self.target.unwrap_or(0) as i32;
    }
}

pub(super) fn empty_report(oid: u8, status: u8) -> VelocityLimitLatchReport {
    VelocityLimitLatchReport {
        oid,
        status,
        test_limit: TEST_LIMIT,
        p_raw: TEST_P_RAW,
        i_raw: TEST_I_RAW,
        target_velocity_rpm: TEST_TARGET_RPM,
        limit_readback: 0,
        gains_readback: 0,
        target_readback: 0,
        active_status_flags: 0,
        post_pulse_status_flags: 0,
        delayed_status_flags: 0,
        post_clear_status_flags: 0,
        pulse_elapsed_us: 0,
        delayed_read_elapsed_us: 0,
        encoder_before: 0,
        encoder_after: 0,
        encoder_delta: 0,
        saved_limit: 0,
        saved_gains: 0,
        saved_target: 0,
        saved_mode: 0,
        restored_limit_readback: 0,
        restored_gains_readback: 0,
        restored_target_readback: 0,
        restored_mode_readback: 0,
        restore_verified_mask: 0,
        power_stage_tripped: false,
    }
}

fn handled(report: VelocityLimitLatchReport) -> DiagnosticOutcome {
    DiagnosticOutcome::Handled(DiagnosticReport::VelocityLimitLatch(report))
}

fn elapsed_u32(start_us: u64, end_us: u64) -> u32 {
    end_us.wrapping_sub(start_us).min(u64::from(u32::MAX)) as u32
}

fn reject_actionable_status(
    flags: u32,
    feedback_config: crate::config::FeedbackConfig,
) -> Result<(), ()> {
    if crate::autotune::is_actionable_status(
        flags,
        crate::autotune::current_loop_actionable_status_mask(feedback_config),
    ) {
        Err(())
    } else {
        Ok(())
    }
}

fn reject_unexpected_stimulus_status(
    flags: u32,
    feedback_config: crate::config::FeedbackConfig,
) -> Result<(), ()> {
    let actionable = flags & crate::autotune::current_loop_actionable_status_mask(feedback_config);
    if actionable & !EXPECTED_STIMULUS_STATUS == 0 {
        Ok(())
    } else {
        Err(())
    }
}

async fn write_verified<B>(
    tmc: &mut Tmc4671Spi<B>,
    register: Register,
    value: u32,
) -> Result<u32, ()>
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    tmc.write_register(register, value).await.map_err(|_| ())?;
    let readback = tmc.read_register(register).await.map_err(|_| ())?;
    if readback == value {
        Ok(readback)
    } else {
        Err(())
    }
}

async fn restore_registers<B>(
    tmc: &mut Tmc4671Spi<B>,
    saved: &SavedRegisters,
    report: &mut VelocityLimitLatchReport,
) where
    B: embedded_hal_async::spi::SpiDevice<u8>,
{
    if let Some(value) = saved.target
        && let Ok(readback) = write_verified(tmc, PID_VELOCITY_TARGET, value).await
    {
        report.restored_target_readback = readback as i32;
        report.restore_verified_mask |= RESTORED_TARGET;
    }
    if let Some(value) = saved.gains
        && let Ok(readback) = write_verified(tmc, PID_VELOCITY_P_VELOCITY_I, value).await
    {
        report.restored_gains_readback = readback;
        report.restore_verified_mask |= RESTORED_GAINS;
    }
    if let Some(value) = saved.limit
        && let Ok(readback) = write_verified(tmc, PID_TORQUE_FLUX_LIMITS, value).await
    {
        report.restored_limit_readback = readback;
        report.restore_verified_mask |= RESTORED_LIMIT;
    }
    if let Some(value) = saved.mode
        && let Ok(readback) = write_verified(tmc, MODE_RAMP_MODE_MOTION, value).await
    {
        report.restored_mode_readback = readback;
        report.restore_verified_mask |= RESTORED_MODE;
    }
}

/// Run the focused trace-only velocity-output-limit persistence diagnostic.
pub async fn run_velocity_limit_latch_request<B, D>(
    req: TmcRequest,
    ctx: DiagnosticContext<'_, B, D>,
) -> DiagnosticOutcome
where
    B: embedded_hal_async::spi::SpiDevice<u8>,
    D: DiagnosticDelay,
{
    let oid = match req {
        TmcRequest {
            kind: TmcRequestKind::VelocityLimitLatchTest { oid },
            ..
        } => oid,
        req => return DiagnosticOutcome::NotDiagnostic(req),
    };

    if !ctx.runtime.is_motor_enabled()
        || !ctx.runtime.is_calibrated()
        || ctx.readiness.homing_active
        || ctx.readiness.stepper_active
    {
        return handled(empty_report(oid, VELOCITY_LIMIT_LATCH_STATUS_NOT_READY));
    }

    let mut report = empty_report(oid, 0);
    let mut saved = SavedRegisters::default();
    let run_result: Result<(), ()> = async {
        saved.mode = Some(
            ctx.tmc
                .read_register(MODE_RAMP_MODE_MOTION)
                .await
                .map_err(|_| ())?,
        );
        saved.limit = Some(
            ctx.tmc
                .read_register(PID_TORQUE_FLUX_LIMITS)
                .await
                .map_err(|_| ())?,
        );
        saved.gains = Some(
            ctx.tmc
                .read_register(PID_VELOCITY_P_VELOCITY_I)
                .await
                .map_err(|_| ())?,
        );
        saved.target = Some(
            ctx.tmc
                .read_register(PID_VELOCITY_TARGET)
                .await
                .map_err(|_| ())?,
        );
        report.encoder_before = ctx
            .tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| ())?;

        let stale_flags = ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
        reject_actionable_status(stale_flags, ctx.feedback_config)?;

        ctx.tmc
            .write_register(PID_VELOCITY_TARGET, 0)
            .await
            .map_err(|_| ())?;
        ctx.tmc
            .write_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM, 0)
            .await
            .map_err(|_| ())?;
        report.limit_readback =
            write_verified(ctx.tmc, PID_TORQUE_FLUX_LIMITS, u32::from(TEST_LIMIT)).await?;
        let test_gains = PidGains::new()
            .with_p_gain(TEST_P_RAW)
            .with_i_gain(TEST_I_RAW)
            .into_bits();
        report.gains_readback =
            write_verified(ctx.tmc, PID_VELOCITY_P_VELOCITY_I, test_gains).await?;

        let velocity_mode = saved.mode.ok_or(())? & !0xff | MotionMode::Velocity as u32;
        write_verified(ctx.tmc, MODE_RAMP_MODE_MOTION, velocity_mode).await?;
        // TMC4671 Rev 1 (02/2024), section 4.7.5 defines
        // PID_TORQUE_FLUX_LIMITS as the PI-controller output clamp.
        // STATUS_FLAGS is RW and bit 7 is pid_v_output_limit; writing zero
        // explicitly resets the flags before the focused stimulus.
        ctx.tmc
            .write_register(STATUS_FLAGS, 0)
            .await
            .map_err(|_| ())?;

        let pulse_start_us = ctx.delay.now_us();
        report.target_readback =
            write_verified(ctx.tmc, PID_VELOCITY_TARGET, TEST_TARGET_RPM as u32).await? as i32;
        ctx.delay.delay_us(TEST_PULSE_US).await;
        report.active_status_flags = ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
        // TMC4671 Rev 1 Table 30 ties both pid_v_output_limit (bit 7) and
        // pid_iq_target_limit (bit 12) to PID_TORQUE_FLUX_LIMITS (0x5E).
        // The two-count request against the one-count test limit deliberately
        // exercises both bounded limiters. No other actionable bit is allowed.
        reject_unexpected_stimulus_status(report.active_status_flags, ctx.feedback_config)?;
        report.pulse_elapsed_us = elapsed_u32(pulse_start_us, ctx.delay.now_us());

        ctx.tmc
            .write_register(PID_VELOCITY_TARGET, 0)
            .await
            .map_err(|_| ())?;
        report.post_pulse_status_flags =
            ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
        reject_unexpected_stimulus_status(report.post_pulse_status_flags, ctx.feedback_config)?;

        let delayed_start_us = ctx.delay.now_us();
        ctx.delay.delay_us(PERSISTENCE_DELAY_US).await;
        report.delayed_status_flags = ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
        reject_unexpected_stimulus_status(report.delayed_status_flags, ctx.feedback_config)?;
        report.delayed_read_elapsed_us = elapsed_u32(delayed_start_us, ctx.delay.now_us());

        ctx.tmc
            .write_register(STATUS_FLAGS, 0)
            .await
            .map_err(|_| ())?;
        report.post_clear_status_flags =
            ctx.tmc.read_register(STATUS_FLAGS).await.map_err(|_| ())?;
        reject_actionable_status(report.post_clear_status_flags, ctx.feedback_config)?;
        report.encoder_after = ctx
            .tmc
            .read_register(ABN_DECODER_COUNT)
            .await
            .map_err(|_| ())?;
        report.encoder_delta = report.encoder_after.wrapping_sub(report.encoder_before) as i32;
        Ok(())
    }
    .await;

    ctx.power_stage.trip_power_stage();
    ctx.runtime.contain_diagnostic_trip();
    report.power_stage_tripped = true;
    saved.copy_into(&mut report);
    restore_registers(ctx.tmc, &saved, &mut report).await;
    if run_result.is_err() || report.restore_verified_mask != RESTORED_ALL {
        report.status = VELOCITY_LIMIT_LATCH_STATUS_SPI;
    }

    handled(report)
}
