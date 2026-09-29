//! TMC4671 request types and the constructors that build them for a board's SPI task.

use core::sync::atomic::Ordering;

/// Request type for TMC4671 task communication.
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub struct TmcRequest {
    pub channel: u8,
    /// Firmware-owned autotune selector; ignored by non-tune requests.
    pub autotune_action: AutotuneAction,
    pub kind: TmcRequestKind,
}

impl TmcRequest {
    /// Build a TMC request for `channel`.
    pub const fn new(channel: u8, kind: TmcRequestKind) -> Self {
        Self {
            channel,
            autotune_action: AutotuneAction::BreakawaySeeded,
            kind,
        }
    }

    /// Build a TMC request with an explicit firmware-owned autotune selector.
    pub const fn new_with_autotune_action(
        channel: u8,
        autotune_action: AutotuneAction,
        kind: TmcRequestKind,
    ) -> Self {
        Self {
            channel,
            autotune_action,
            kind,
        }
    }
}

/// Runtime acceleration feedforward gains in permille.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct AccelFeedforwardGains {
    pub accel_permille: u16,
    pub decel_permille: u16,
}

/// Axis selected by the scalar current-step diagnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum CurrentStepAxis {
    /// TMC4671 torque/q-axis current target and actual.
    Torque,
    /// TMC4671 flux/d-axis current target and actual.
    Flux,
}

impl CurrentStepAxis {
    /// Wire value used by `tmc_current_step_test`.
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::Torque => 0,
            Self::Flux => 1,
        }
    }

    /// Decode a `tmc_current_step_test` wire value.
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::Torque),
            1 => Some(Self::Flux),
            _ => None,
        }
    }
}

/// Which torque-sample diagnostic produced a `CurrentTorqueSampleReport`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum TorqueSampleKind {
    CurrentTorqueSample,
    PositionTorqueOffset,
}

impl TorqueSampleKind {
    /// Wire value used by the `foci_current_torque_sample_result` reply.
    pub const fn as_u8(self) -> u8 {
        match self {
            Self::CurrentTorqueSample => 0,
            Self::PositionTorqueOffset => 1,
        }
    }
}

/// Control-tick period while a channel is not `motor_enabled` or planner paths
/// are not allowed, in microseconds (100 Hz).
pub const CONTROL_PERIOD_IDLE_US: u32 = 10_000;
/// Control-tick period while the motor is enabled and planner paths are
/// allowed, in microseconds (1 kHz).
pub const CONTROL_PERIOD_ACTIVE_US: u32 = 1_000;

/// Operation carried by a [`TmcRequest`].
#[derive(Debug, Clone, PartialEq, Eq, defmt::Format)]
pub enum TmcRequestKind {
    SetEnable(bool),
    /// Run ADC offset calibration, encoder alignment, closed-loop entry.
    Calibrate {
        oid: u8,
    },
    /// Set motor run current (milliamps to register counts).
    SetCurrent {
        run_ma: u32,
    },
    /// Configure independent encoder-feedback and planner-motion scales.
    SetMotionScale {
        oid: u8,
        /// Encoder pulses per mechanical revolution before quadrature.
        encoder_ppr: u32,
        planner_steps_per_rev: u32,
    },
    SetEncoderDir {
        invert: bool,
    },
    SelfTest {
        oid: u8,
    },
    DumpRegisters {
        oid: u8,
    },
    SetPidGains {
        flux_p: foci_tmc4671::pid::PidPGain,
        flux_i: foci_tmc4671::pid::PidIGain,
        torque_p: foci_tmc4671::pid::PidPGain,
        torque_i: foci_tmc4671::pid::PidIGain,
    },
    /// Start commissioning (diagnostics + current tune + closed-loop entry).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    Commission {
        oid: u8,
        profile: u8,
    },
    /// Start installed tuning (mechanical ID + velocity/position + filters).
    /// Profile: 0=Conservative, 1=Balanced, 2=Stiff.
    /// Mode: 0=Unloaded, 1=Nominal, 2=HighInertia.
    Tune {
        oid: u8,
        profile: u8,
        mode: u8,
        inner_lambda_us: u32,
        theta_e_us: u32,
        /// Current-loop effective bandwidth Hz from inner commissioning.
        current_bandwidth_hz: u32,
        inner_warning_flags: u8,
        /// Requested mechanical velocity amplitude in milli-revolutions/s.
        requested_velocity_mrev_s: u32,
        /// Kinematics-derived absolute machine velocity ceiling.
        machine_velocity_ceiling_mrev_s: u32,
        requested_velocity_source:
            crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource,
        /// Full local displacement window spanning moving and settle.
        max_stroke_travel_mrev: u32,
        /// Host planning reserve for braking and settle drift.
        settle_travel_reserve_mrev: u32,
        /// Negative absolute-position headroom from installed-tuning entry.
        negative_position_headroom_mrev: u32,
        /// Positive absolute-position headroom from installed-tuning entry.
        positive_position_headroom_mrev: u32,
        /// Host-computed max primitive duration in milliseconds.
        max_duration_ms: u16,
        /// Axis homing speed, mrev/s; the homing bound's speed.
        homing_speed_mrev_s: u32,
        /// Machine acceleration, mrev/s^2; the stroke ramp rate.
        max_accel_mrev_s2: u32,
    },
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetVelocityFilter {
        filter_hz: u16,
    },
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetTorqueFilter {
        filter_hz: u16,
    },
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetPositionFilter {
        filter_hz: u16,
    },
    /// 0 = disabled, 10..1000 Hz = enabled.
    SetFluxFilter {
        filter_hz: u16,
    },
    /// Range: 0..32767 with P in Q8.8 and I in Q4.12.
    SetPositionGains {
        position_p: foci_tmc4671::pid::PidPGain,
        position_i: foci_tmc4671::pid::PidIGain,
        velocity_p: foci_tmc4671::pid::PidPGain,
        velocity_i: foci_tmc4671::pid::PidIGain,
    },
    SetAccelFeedforward {
        enable: bool,
        /// Gain in permille applied while step intervals shrink.
        accel_gain_permille: u16,
        /// Gain in permille applied while step intervals grow.
        decel_gain_permille: u16,
    },
    /// Set operational PIDOUT_UQ_UD_LIMITS for authority diagnostics.
    SetVoltageLimit {
        voltage_limit: u32,
    },
    /// Run a bounded current-loop step diagnostic while already enabled.
    CurrentStepTest {
        oid: u8,
        axis: CurrentStepAxis,
        /// Signed current target in TMC4671 current counts.
        target: i16,
        duration_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded current-vector step diagnostic while already enabled.
    CurrentVectorStepTest {
        oid: u8,
        /// Signed torque-current target in TMC4671 current counts.
        torque_target: i16,
        /// Signed flux-current target in TMC4671 current counts.
        flux_target: i16,
        duration_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded torque-current pulse and sample it before the old dwell floor.
    CurrentTorqueSampleTest {
        oid: u8,
        /// Signed torque-current target in TMC4671 current counts.
        target: i16,
        /// Signed flux-current target in TMC4671 current counts.
        flux_target: i16,
        /// Delay between applying the target and sampling, in milliseconds.
        sample_delay_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded torque-offset pulse in position mode and sample it.
    PositionTorqueOffsetSampleTest {
        oid: u8,
        /// Signed torque offset in TMC4671 current counts.
        target: i16,
        /// Delay between applying the offset and sampling, in milliseconds.
        sample_delay_ms: u16,
        /// Temporary PIDOUT_UQ_UD_LIMITS value for the diagnostic.
        voltage_limit: u16,
    },
    /// Run a bounded open-loop voltage pulse and sample it.
    VoltageStepTest {
        oid: u8,
        /// Signed q-axis voltage component for UQ_UD_EXT.
        uq_ext: i16,
        /// Signed d-axis voltage component for UQ_UD_EXT.
        ud_ext: i16,
        /// Delay between applying the voltage vector and sampling, in milliseconds.
        sample_delay_ms: u16,
    },
    /// Run the resistance-identification sweep over an already-armed motor; commissioning state is
    /// untouched.
    ResistanceTest {
        oid: u8,
        /// Reserved verbosity/selector byte for future detail replies.
        detail: u8,
    },
    /// Write a TMC4671 register via SPI (dev-only).
    #[cfg(feature = "dev")]
    Write {
        addr: u8,
        value: u32,
    },
    /// Read a TMC4671 register via SPI (dev-only).
    #[cfg(feature = "dev")]
    Read {
        oid: u8,
        addr: u8,
    },
    /// Trigger TMC4671 initialization sequence (dev-only).
    #[cfg(feature = "dev")]
    Init,
    /// Test motor rotation by sweeping PHI_E_EXT (dev-only).
    #[cfg(feature = "dev")]
    TestSpin {
        revolutions: u8,
    },
}

pub use crate::commissioning::vocabulary_generated::AutotuneAction;

impl AutotuneAction {
    /// Observation slot order this acquisition requests.
    pub const fn slot_order(
        self,
    ) -> crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder {
        use crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder;
        match self {
            Self::FixedGainAmplitudeAscending
            | Self::FixedGainAmplitudeDescending
            | Self::BreakawaySeeded
            | Self::IntegralResume
            | Self::RobustnessReversal
            | Self::PositionTune => IntegralSlotOrder::ForwardFirst,
        }
    }
}

/// Build a `SetCurrent` request.
pub fn tmc_set_current(channel: u8, run_ma: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetCurrent { run_ma })
}

/// Validate wire inputs and build a motion-scale request; errors with
/// [`crate::motion_scale::MotionScaleError`] when either resolution is invalid.
pub fn tmc_set_encoder_hw(
    channel: u8,
    oid: u8,
    encoder_ppr: u32,
    planner_steps_per_rev: u32,
) -> Result<TmcRequest, crate::motion_scale::MotionScaleError> {
    crate::motion_scale::MotionScale::new(encoder_ppr, planner_steps_per_rev)?;
    Ok(TmcRequest::new(
        channel,
        TmcRequestKind::SetMotionScale {
            oid,
            encoder_ppr,
            planner_steps_per_rev,
        },
    ))
}

/// Build a `SetEncoderDir` request.
pub fn tmc_set_encoder_dir(channel: u8, invert: bool) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetEncoderDir { invert })
}

/// Build a `SelfTest` request.
pub fn tmc_selftest(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SelfTest { oid })
}

/// Build a `Calibrate` request.
pub fn tmc_calibrate(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Calibrate { oid })
}

/// Build a `DumpRegisters` request.
pub fn tmc_dump_registers(channel: u8, oid: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::DumpRegisters { oid })
}

/// Build a `SetPidGains` request.
pub fn tmc_set_pid_gains(
    channel: u8,
    flux_p: foci_tmc4671::pid::PidPGain,
    flux_i: foci_tmc4671::pid::PidIGain,
    torque_p: foci_tmc4671::pid::PidPGain,
    torque_i: foci_tmc4671::pid::PidIGain,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPidGains {
            flux_p,
            flux_i,
            torque_p,
            torque_i,
        },
    )
}

/// Build a `Commission` request (diagnostics + current tune + closed-loop entry).
pub fn tmc_commission(channel: u8, oid: u8, profile: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Commission { oid, profile })
}

/// Parameters of a `Tune` request beyond its channel, OID and acquisition action.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct TuneParameters {
    /// Tuning profile: 0=Conservative, 1=Balanced, 2=Stiff.
    pub profile: u8,
    /// Plant mode: 0=Unloaded, 1=Nominal, 2=HighInertia.
    pub mode: u8,
    /// Inner-loop lambda from inner commissioning, in microseconds.
    pub inner_lambda_us: u32,
    /// Electrical-model `theta_e` from inner commissioning, in microseconds.
    pub theta_e_us: u32,
    /// Current-loop effective bandwidth Hz from inner commissioning.
    pub current_bandwidth_hz: u32,
    /// Warning flags from inner commissioning.
    pub inner_warning_flags: u8,
    /// Requested mechanical velocity amplitude in milli-revolutions/s.
    pub requested_velocity_mrev_s: u32,
    /// Kinematics-derived absolute machine velocity ceiling.
    pub machine_velocity_ceiling_mrev_s: u32,
    /// Whether the requested velocity was explicit or defaulted by the host.
    pub requested_velocity_source:
        crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource,
    /// Full local displacement window spanning moving and settle.
    pub max_stroke_travel_mrev: u32,
    /// Host planning reserve for braking and settle drift.
    pub settle_travel_reserve_mrev: u32,
    /// Negative absolute-position headroom from installed-tuning entry.
    pub negative_position_headroom_mrev: u32,
    /// Positive absolute-position headroom from installed-tuning entry.
    pub positive_position_headroom_mrev: u32,
    /// Host-computed max primitive duration in milliseconds.
    pub max_duration_ms: u16,
    /// Axis homing speed, mrev/s; the homing bound's speed.
    pub homing_speed_mrev_s: u32,
    /// Machine acceleration, mrev/s^2; the stroke ramp rate.
    pub max_accel_mrev_s2: u32,
}

/// Build a `Tune` request with an explicit firmware-owned acquisition action.
pub fn tmc_tune_with_action(
    channel: u8,
    oid: u8,
    action: AutotuneAction,
    parameters: TuneParameters,
) -> TmcRequest {
    let TuneParameters {
        profile,
        mode,
        inner_lambda_us,
        theta_e_us,
        current_bandwidth_hz,
        inner_warning_flags,
        requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s,
        requested_velocity_source,
        max_stroke_travel_mrev,
        settle_travel_reserve_mrev,
        negative_position_headroom_mrev,
        positive_position_headroom_mrev,
        max_duration_ms,
        homing_speed_mrev_s,
        max_accel_mrev_s2,
    } = parameters;
    TmcRequest::new_with_autotune_action(
        channel,
        action,
        TmcRequestKind::Tune {
            oid,
            profile,
            mode,
            inner_lambda_us,
            theta_e_us,
            current_bandwidth_hz,
            inner_warning_flags,
            requested_velocity_mrev_s,
            machine_velocity_ceiling_mrev_s,
            requested_velocity_source,
            max_stroke_travel_mrev,
            settle_travel_reserve_mrev,
            negative_position_headroom_mrev,
            positive_position_headroom_mrev,
            max_duration_ms,
            homing_speed_mrev_s,
            max_accel_mrev_s2,
        },
    )
}

/// Build a `SetVelocityFilter` request.
pub fn tmc_set_velocity_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetVelocityFilter { filter_hz })
}

/// Build a `SetTorqueFilter` request.
pub fn tmc_set_torque_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetTorqueFilter { filter_hz })
}

/// Build a `SetPositionFilter` request.
pub fn tmc_set_position_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetPositionFilter { filter_hz })
}

/// Build a `SetFluxFilter` request.
pub fn tmc_set_flux_filter(channel: u8, filter_hz: u16) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetFluxFilter { filter_hz })
}

/// Build a `SetPositionGains` request.
pub fn tmc_set_position_gains(
    channel: u8,
    position_p: foci_tmc4671::pid::PidPGain,
    position_i: foci_tmc4671::pid::PidIGain,
    velocity_p: foci_tmc4671::pid::PidPGain,
    velocity_i: foci_tmc4671::pid::PidIGain,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetPositionGains {
            position_p,
            position_i,
            velocity_p,
            velocity_i,
        },
    )
}

/// Build a `SetAccelFeedforward` request.
pub fn tmc_set_accel_feedforward(
    channel: u8,
    enable: bool,
    accel_gain_permille: u16,
    decel_gain_permille: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::SetAccelFeedforward {
            enable,
            accel_gain_permille,
            decel_gain_permille,
        },
    )
}

/// Build a `SetVoltageLimit` request.
pub fn tmc_set_voltage_limit(channel: u8, voltage_limit: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetVoltageLimit { voltage_limit })
}

/// Build a `CurrentStepTest` request.
pub fn tmc_current_step_test(
    channel: u8,
    oid: u8,
    axis: CurrentStepAxis,
    target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentStepTest {
            oid,
            axis,
            target,
            duration_ms,
            voltage_limit,
        },
    )
}

/// Build a `CurrentVectorStepTest` request.
pub fn tmc_current_vector_step_test(
    channel: u8,
    oid: u8,
    torque_target: i16,
    flux_target: i16,
    duration_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentVectorStepTest {
            oid,
            torque_target,
            flux_target,
            duration_ms,
            voltage_limit,
        },
    )
}

/// Build a `CurrentTorqueSampleTest` request.
pub fn tmc_current_torque_sample_test(
    channel: u8,
    oid: u8,
    target: i16,
    flux_target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::CurrentTorqueSampleTest {
            oid,
            target,
            flux_target,
            sample_delay_ms,
            voltage_limit,
        },
    )
}

/// Build a `PositionTorqueOffsetSampleTest` request.
pub fn tmc_position_torque_offset_sample_test(
    channel: u8,
    oid: u8,
    target: i16,
    sample_delay_ms: u16,
    voltage_limit: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::PositionTorqueOffsetSampleTest {
            oid,
            target,
            sample_delay_ms,
            voltage_limit,
        },
    )
}

/// Build a `VoltageStepTest` request.
pub fn tmc_voltage_step_test(
    channel: u8,
    oid: u8,
    uq_ext: i16,
    ud_ext: i16,
    sample_delay_ms: u16,
) -> TmcRequest {
    TmcRequest::new(
        channel,
        TmcRequestKind::VoltageStepTest {
            oid,
            uq_ext,
            ud_ext,
            sample_delay_ms,
        },
    )
}

/// Build a `ResistanceTest` request.
pub fn tmc_resistance_test(channel: u8, oid: u8, detail: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::ResistanceTest { oid, detail })
}

/// Build a `SetEnable` request.
pub fn tmc_set_enable(channel: u8, enable: bool) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::SetEnable(enable))
}

/// Set whether a raw enable may trigger firmware-side auto-calibration.
#[cfg(test)]
pub fn set_auto_calibrate_on_enable_allowed(channel: u8, enable: bool) -> bool {
    let Some(state) = crate::state::tmc::tmc_channel(channel) else {
        return false;
    };
    state
        .auto_calibrate_on_enable_allowed
        .store(enable, Ordering::Relaxed);
    true
}

/// Return whether raw enable auto-calibration is currently allowed.
pub fn auto_calibrate_on_enable_allowed(channel: u8) -> bool {
    crate::state::tmc::tmc_channel(channel)
        .map(|state| {
            state
                .auto_calibrate_on_enable_allowed
                .load(Ordering::Relaxed)
        })
        .unwrap_or(false)
}

/// Signed command-velocity reference for trace diagnostics, in `PID_VELOCITY_OFFSET` sign
/// convention and without the feedforward multiplier; derived from STEP/DIR timing, not a register
/// read.
pub fn compute_planner_command_velocity_reference(
    scale: crate::motion_scale::PlannerVelocityScale,
    step_interval: u32,
    dir_high: bool,
) -> i32 {
    compute_planner_velocity_offset_scaled(scale, step_interval, dir_high, 1)
}

/// Velocity feedforward offset with a runtime multiplier; `0` disables it. Products saturate
/// symmetrically at `i32::MAX` magnitude.
pub fn compute_planner_velocity_offset_scaled(
    scale: crate::motion_scale::PlannerVelocityScale,
    step_interval: u32,
    dir_high: bool,
    multiplier: u16,
) -> i32 {
    if step_interval == 0 || multiplier == 0 {
        return 0;
    }
    let mut numerator_factors = [scale.numerator(), u64::from(multiplier)];
    let mut denominator_factors = [scale.denominator(), u64::from(step_interval)];
    cross_reduce_factors(&mut numerator_factors, &mut denominator_factors);

    let numerator = numerator_factors[0]
        .checked_mul(numerator_factors[1])
        .expect("validated planner velocity numerator fits u64");
    let rounded = match denominator_factors[0].checked_mul(denominator_factors[1]) {
        Some(denominator) => {
            let quotient = numerator / denominator;
            let remainder = numerator % denominator;
            quotient + u64::from(remainder >= denominator / 2 + denominator % 2)
        }
        None => u64::from(ceil_half_product_leq(
            denominator_factors[0],
            denominator_factors[1],
            numerator,
        )),
    };
    let magnitude = rounded.min(i32::MAX as u64) as i32;
    if dir_high { -magnitude } else { magnitude }
}

/// Compute the velocity feedforward offset directly in mechanical RPM.
///
/// # Parameters
/// - `scale`: exact planner-timing mechanical-RPM ratio
/// - `step_interval`: current step interval in TIM5 clock ticks (from atomic)
/// - `dir_high`: true if the TMC4671 DIR input is high
/// - `gain_permille`: unitless feedforward gain, in thousandths (1000 = unity)
///
/// Returns 0 if `step_interval == 0` or `gain_permille == 0` (motor idle or
/// feedforward disabled).
///
/// With a positive STEP_WIDTH, the TMC4671 STEP/DIR input reports a
/// negative velocity for DIR high and a positive velocity for DIR low.
pub fn compute_planner_velocity_offset_rpm(
    scale: crate::motion_scale::PlannerRpmScale,
    step_interval: u32,
    dir_high: bool,
    gain_permille: u16,
) -> i32 {
    if step_interval == 0 || gain_permille == 0 {
        return 0;
    }
    let mut numerators = [scale.numerator(), u64::from(gain_permille)];
    let mut denominators = [scale.denominator(), u64::from(step_interval), 1_000];
    let magnitude = round_ratio_to_cap(&mut numerators, &mut denominators, i32::MAX as u64) as i32;
    if dir_high { -magnitude } else { magnitude }
}

/// Unity-gain (`gain_permille == 1_000`) case of `compute_planner_velocity_offset_rpm`: the signed
/// command-RPM reference the RPM feedforward clamp bounds against.
pub fn compute_planner_command_rpm_reference(
    scale: crate::motion_scale::PlannerRpmScale,
    step_interval: u32,
    dir_high: bool,
) -> i32 {
    compute_planner_velocity_offset_rpm(scale, step_interval, dir_high, 1_000)
}

/// Default multiple of commanded RPM the velocity feedforward offset may not exceed.
pub const VELOCITY_FF_CLAMP_K: u16 = 3;

/// Clamp a velocity feedforward offset to `k` times the magnitude of `command_rpm`.
///
/// Returns the (possibly clamped) offset and whether clamping engaged. A zero
/// `command_rpm` forces the bound to zero, so any nonzero offset is clamped.
pub fn clamp_velocity_offset_rpm(offset: i32, command_rpm: i32, k: u16) -> (i32, bool) {
    let bound = i64::from(command_rpm)
        .unsigned_abs()
        .saturating_mul(u64::from(k))
        .min(i32::MAX as u64) as i32;
    if offset > bound {
        (bound, true)
    } else if offset < -bound {
        (-bound, true)
    } else {
        (offset, false)
    }
}

fn cross_reduce_factors(numerators: &mut [u64; 2], denominators: &mut [u64; 2]) {
    for numerator in numerators {
        for denominator in denominators.iter_mut() {
            let divisor = crate::motion_scale::greatest_common_divisor(*numerator, *denominator);
            *numerator /= divisor;
            *denominator /= divisor;
        }
    }
}

fn ceil_half_product_leq(left: u64, right: u64, limit: u64) -> bool {
    if left.is_multiple_of(2) {
        return (left / 2)
            .checked_mul(right)
            .is_some_and(|half_product| half_product <= limit);
    }
    if right.is_multiple_of(2) {
        return (right / 2)
            .checked_mul(left)
            .is_some_and(|half_product| half_product <= limit);
    }

    (left / 2)
        .checked_mul(right)
        .and_then(|half_product| half_product.checked_add(right / 2))
        .and_then(|half_product| half_product.checked_add(1))
        .is_some_and(|half_product| half_product <= limit)
}

fn round_ratio_to_cap(numerators: &mut [u64], denominators: &mut [u64], cap: u64) -> u64 {
    for numerator in numerators.iter_mut() {
        for denominator in denominators.iter_mut() {
            let divisor = crate::motion_scale::greatest_common_divisor(*numerator, *denominator);
            *numerator /= divisor;
            *denominator /= divisor;
        }
    }

    let numerator = numerators.iter().fold(1u128, |product, factor| {
        product
            .checked_mul(u128::from(*factor))
            .expect("planner feedforward numerator fits u128")
    });
    let denominator = denominators.iter().try_fold(1u128, |product, factor| {
        product.checked_mul(u128::from(*factor))
    });
    let rounded = if let Some(denominator) = denominator {
        let quotient = numerator / denominator;
        let remainder = numerator % denominator;
        quotient + u128::from(remainder >= denominator / 2 + denominator % 2)
    } else {
        let twice_numerator = numerator
            .checked_mul(2)
            .expect("planner feedforward numerator leaves one rounding bit");
        u128::from(product_leq(denominators, twice_numerator))
    };
    rounded.min(u128::from(cap)) as u64
}

fn product_leq(factors: &[u64], limit: u128) -> bool {
    factors
        .iter()
        .try_fold(1u128, |product, factor| {
            product
                .checked_mul(u128::from(*factor))
                .filter(|product| *product <= limit)
        })
        .is_some()
}

/// Acceleration feedforward torque offset for the torque field of `PID_TORQUE_FLUX_OFFSET`.
/// `interval_add` is Klipper's `queue_step add`: interval change in TIM5 ticks per step, negative
/// while velocity is increasing. `gain_permille` 1000 applies the raw estimate, `0` disables it.
/// The result is clamped to `torque_limit`, which callers set to the torque-offset headroom of the
/// committed current limit.
pub fn compute_planner_accel_torque_offset(
    scale: crate::motion_scale::PlannerVelocityScale,
    clock_freq: u32,
    step_interval: u32,
    interval_add: i16,
    dir_high: bool,
    gain_permille: u16,
    torque_limit: u32,
) -> i16 {
    if clock_freq == 0
        || step_interval == 0
        || interval_add == 0
        || gain_permille == 0
        || torque_limit == 0
    {
        return 0;
    }

    let interval = u64::from(step_interval);
    let mut numerators = [
        u64::from(interval_add.unsigned_abs()),
        scale.numerator(),
        u64::from(clock_freq),
        u64::from(gain_permille),
    ];
    let mut denominators = [scale.denominator(), interval, interval, interval, 1_000];
    let current_limit = u64::from(torque_limit.min(i16::MAX as u32));
    let magnitude = round_ratio_to_cap(&mut numerators, &mut denominators, current_limit) as i16;
    if magnitude == 0 {
        return 0;
    }

    let accel_sign = if interval_add < 0 { 1 } else { -1 };
    let direction_sign = if dir_high { -1 } else { 1 };
    if accel_sign * direction_sign < 0 {
        -magnitude
    } else {
        magnitude
    }
}

/// Compute acceleration feedforward with independent accel/decel gains.
pub fn compute_planner_accel_torque_offset_split(
    scale: crate::motion_scale::PlannerVelocityScale,
    clock_freq: u32,
    step_interval: u32,
    interval_add: i16,
    dir_high: bool,
    gains: AccelFeedforwardGains,
    torque_limit: u32,
) -> i16 {
    let gain_permille = if interval_add < 0 {
        gains.accel_permille
    } else if interval_add > 0 {
        gains.decel_permille
    } else {
        0
    };
    compute_planner_accel_torque_offset(
        scale,
        clock_freq,
        step_interval,
        interval_add,
        dir_high,
        gain_permille,
        torque_limit,
    )
}

/// Pack signed torque and flux targets for PID_TORQUE_FLUX_TARGET.
pub fn pack_torque_flux_target(torque: i16, flux: i16) -> u32 {
    ((torque as u16 as u32) << 16) | flux as u16 as u32
}

/// Decode the signed upper 16 bits of a TMC4671 packed register.
pub fn decode_signed_upper_half(value: u32) -> i16 {
    ((value >> 16) & 0xffff) as u16 as i16
}

/// Decode the signed lower 16 bits of a TMC4671 packed register.
pub fn decode_signed_lower_half(value: u32) -> i16 {
    (value & 0xffff) as u16 as i16
}

/// Build a `Write` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_write_register(channel: u8, addr: u8, value: u32) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Write { addr, value })
}

/// Build a `Read` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_read_register(channel: u8, oid: u8, addr: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Read { oid, addr })
}

/// Build an `Init` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_init(channel: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::Init)
}

/// Build a `TestSpin` request (dev-only).
#[cfg(feature = "dev")]
pub fn tmc_test_spin(channel: u8, revolutions: u8) -> TmcRequest {
    TmcRequest::new(channel, TmcRequestKind::TestSpin { revolutions })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn autotune_action_admission_is_frozen() {
        for code in [0u8, 3, 4, 5, 6] {
            assert!(
                AutotuneAction::from_u8(code).is_none(),
                "reserved {code} decoded"
            );
        }
        for code in [1u8, 2, 7, 8, 9, 10] {
            assert!(
                AutotuneAction::from_u8(code).is_some(),
                "live {code} rejected"
            );
        }
    }

    #[test]
    fn integral_resume_decodes_and_carries_no_amplitude_order() {
        assert_eq!(
            AutotuneAction::from_u8(8),
            Some(AutotuneAction::IntegralResume)
        );
        assert_eq!(
            AutotuneAction::from_u8(9),
            Some(AutotuneAction::RobustnessReversal)
        );
        assert_eq!(
            AutotuneAction::from_u8(10),
            Some(AutotuneAction::PositionTune)
        );
        assert_eq!(AutotuneAction::from_u8(11), None);
        assert_eq!(AutotuneAction::IntegralResume as u8, 8);
        assert_eq!(AutotuneAction::IntegralResume.amplitude_order(), None);
        assert_eq!(AutotuneAction::IntegralResume.amplitude_slot_order(), None);
        assert_eq!(
            AutotuneAction::IntegralResume.slot_order(),
            crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder::ForwardFirst
        );
    }

    #[test]
    fn tmc_set_current_includes_channel() {
        let req = tmc_set_current(1, 800);

        assert_eq!(req.channel, 1);
        assert!(matches!(
            req.kind,
            TmcRequestKind::SetCurrent { run_ma: 800 }
        ));
    }

    #[test]
    fn default_request_uses_breakaway_seeded_action() {
        let req = TmcRequest::new(0, TmcRequestKind::SetEncoderDir { invert: true });
        assert_eq!(req.autotune_action, AutotuneAction::BreakawaySeeded);
    }

    #[test]
    fn pack_torque_target_places_signed_torque_in_upper_half() {
        assert_eq!(pack_torque_flux_target(250, 0), 0x00fa_0000);
        assert_eq!(pack_torque_flux_target(-250, 0), 0xff06_0000);
        assert_eq!(pack_torque_flux_target(0, -3), 0x0000_fffd);
    }

    #[test]
    fn signed_half_decoders_extract_torque_and_flux() {
        let value = 0xff06_0005;

        assert_eq!(decode_signed_upper_half(value), -250);
        assert_eq!(decode_signed_lower_half(value), 5);
    }

    #[test]
    fn auto_calibrate_on_enable_policy_can_be_toggled() {
        set_auto_calibrate_on_enable_allowed(0, false);
        assert!(!auto_calibrate_on_enable_allowed(0));

        set_auto_calibrate_on_enable_allowed(0, true);
        assert!(auto_calibrate_on_enable_allowed(0));

        set_auto_calibrate_on_enable_allowed(0, false);
        assert!(!auto_calibrate_on_enable_allowed(0));
    }

    #[test]
    fn rational_planner_velocity_helpers_round_and_apply_tmc_sign() {
        let scale = crate::motion_scale::MotionScale::new(1_000, 3_200)
            .unwrap()
            .planner_velocity_scale(84_000_000, 25_000)
            .unwrap();

        assert_eq!(
            compute_planner_command_velocity_reference(scale, 2_800, false),
            25
        );
        assert_eq!(
            compute_planner_command_velocity_reference(scale, 2_800, true),
            -25
        );
    }

    #[test]
    fn rational_planner_velocity_rounds_ties_away_from_zero() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(1, 2);

        assert_eq!(
            compute_planner_command_velocity_reference(scale, 1, false),
            1
        );
        assert_eq!(
            compute_planner_command_velocity_reference(scale, 1, true),
            -1
        );
    }

    #[test]
    fn rational_planner_velocity_overflow_is_mathematical_zero() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(1, u64::MAX);

        assert_eq!(
            compute_planner_command_velocity_reference(scale, 2, false),
            0
        );
    }

    #[test]
    fn rational_planner_velocity_rounds_exact_ratio_when_denominator_product_overflows() {
        let scale = crate::motion_scale::MotionScale::new(1, 3)
            .unwrap()
            .planner_velocity_scale(4_294_967_294, 4_294_967_295)
            .unwrap();

        assert_eq!(
            compute_planner_velocity_offset_scaled(scale, 1_431_655_766, false, 65_535),
            1
        );
        assert_eq!(
            compute_planner_velocity_offset_scaled(scale, 1_431_655_766, true, 65_535),
            -1
        );
    }

    #[test]
    fn rational_planner_velocity_multiplier_saturates_symmetrically() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(i32::MAX as u64, 1);

        assert_eq!(
            compute_planner_velocity_offset_scaled(scale, 1, false, u16::MAX),
            i32::MAX
        );
        assert_eq!(
            compute_planner_velocity_offset_scaled(scale, 1, true, u16::MAX),
            i32::MIN + 1
        );
    }

    fn planner_scale(encoder_ppr: u32) -> crate::motion_scale::PlannerVelocityScale {
        crate::motion_scale::MotionScale::new(encoder_ppr, 32_768)
            .unwrap()
            .planner_velocity_scale(1, 1)
            .unwrap()
    }

    #[test]
    fn command_velocity_uses_exact_planner_scale_independent_of_encoder_ppr() {
        let low_ppr = compute_planner_command_velocity_reference(planner_scale(500), 3, false);
        let high_ppr = compute_planner_command_velocity_reference(planner_scale(4_000), 3, false);

        assert_eq!(low_ppr, 1);
        assert_eq!(high_ppr, low_ppr);
    }

    #[test]
    fn velocity_feedforward_uses_exact_planner_scale_independent_of_encoder_ppr() {
        let low_ppr = compute_planner_velocity_offset_scaled(planner_scale(500), 3, true, 2);
        let high_ppr = compute_planner_velocity_offset_scaled(planner_scale(4_000), 3, true, 2);

        assert_eq!(low_ppr, -1);
        assert_eq!(high_ppr, low_ppr);
    }

    #[test]
    fn acceleration_feedforward_uses_exact_planner_scale_independent_of_encoder_ppr() {
        let offset = |scale| compute_planner_accel_torque_offset(scale, 1, 1, -1, false, 250, 10);

        assert_eq!(offset(planner_scale(500)), 1);
        assert_eq!(offset(planner_scale(4_000)), 1);
    }

    #[test]
    fn velocity_offset_dir_high_matches_tmc_negative_velocity() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_command_velocity_reference(scale, 420, true);
        assert_eq!(offset, -648);
    }

    #[test]
    fn velocity_offset_dir_low_matches_tmc_positive_velocity() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_command_velocity_reference(scale, 420, false);
        assert_eq!(offset, 648);
    }

    #[test]
    fn velocity_offset_zero_interval() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_command_velocity_reference(scale, 0, true);
        assert_eq!(offset, 0);
    }

    #[test]
    fn velocity_offset_high_speed() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_command_velocity_reference(scale, 84, true);
        assert_eq!(offset, -3240);
    }

    #[test]
    fn velocity_offset_low_speed() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_command_velocity_reference(scale, 84_000, true);
        assert_eq!(offset, -3);
    }

    #[test]
    fn velocity_offset_applies_multiplier() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_velocity_offset_scaled(scale, 84_000, false, 8);
        assert_eq!(offset, 26);
    }

    #[test]
    fn velocity_offset_multiplier_zero_disables_offset() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(272_160, 1);
        let offset = compute_planner_velocity_offset_scaled(scale, 84_000, false, 0);
        assert_eq!(offset, 0);
    }

    #[test]
    fn command_velocity_reference_uses_step_direction_sign() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);

        let forward = compute_planner_command_velocity_reference(scale, 2800, false);
        let reversed = compute_planner_command_velocity_reference(scale, 2800, true);

        assert_eq!(forward, 19);
        assert_eq!(reversed, -19);
    }

    #[test]
    fn command_velocity_reference_zero_cases_disable_reference() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(19, 1);
        assert_eq!(
            compute_planner_command_velocity_reference(scale, 0, false),
            0
        );
    }

    #[test]
    fn command_velocity_reference_is_unmultiplied_velocity_offset() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);
        let reference = compute_planner_command_velocity_reference(scale, 2800, false);
        let offset = compute_planner_velocity_offset_scaled(scale, 2800, false, 40);

        assert_eq!(reference, 19);
        assert_eq!(offset, 768);
    }

    #[test]
    fn velocity_offset_rpm_matches_command_rpm_at_unity_gain() {
        let scale = crate::motion_scale::MotionScale::new(1_000, 3_200)
            .unwrap()
            .planner_rpm_scale(84_000_000)
            .unwrap();
        assert_eq!(
            compute_planner_velocity_offset_rpm(scale, 2_800, false, 1_000),
            563
        );
        assert_eq!(
            compute_planner_command_rpm_reference(scale, 2_800, false),
            563
        );
    }

    #[test]
    fn velocity_offset_rpm_signs_gain_and_edges() {
        let scale = crate::motion_scale::MotionScale::new(1_000, 3_200)
            .unwrap()
            .planner_rpm_scale(84_000_000)
            .unwrap();
        assert_eq!(
            compute_planner_velocity_offset_rpm(scale, 2_800, true, 1_000),
            -563
        );
        assert_eq!(
            compute_planner_velocity_offset_rpm(scale, 2_800, false, 0),
            0
        );
        assert_eq!(
            compute_planner_velocity_offset_rpm(scale, 0, false, 1_000),
            0
        );
        assert_eq!(
            compute_planner_velocity_offset_rpm(scale, 2_800, false, 2_000),
            1_125
        );
    }

    #[test]
    fn clamp_bounds_offset_to_k_times_command() {
        assert_eq!(clamp_velocity_offset_rpm(20_000, 500, 3), (1_500, true));
        assert_eq!(clamp_velocity_offset_rpm(-20_000, 500, 3), (-1_500, true));
        assert_eq!(clamp_velocity_offset_rpm(1_000, 500, 3), (1_000, false));
        assert_eq!(clamp_velocity_offset_rpm(1_500, -500, 3), (1_500, false));
    }

    #[test]
    fn clamp_zero_command_forces_zero_offset() {
        assert_eq!(clamp_velocity_offset_rpm(4_000, 0, 3), (0, true));
        assert_eq!(clamp_velocity_offset_rpm(0, 0, 3), (0, false));
    }

    #[test]
    fn acceleration_feedforward_zero_cases_disable_offset() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);
        assert_eq!(
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, 0, false, 1000, 2816),
            0
        );
        assert_eq!(
            compute_planner_accel_torque_offset(scale, 84_000_000, 0, -9, false, 1000, 2816),
            0
        );
        assert_eq!(
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 0, 2816),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_uses_add_direction_and_gain() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);

        let forward =
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 1000, 2816);
        assert_eq!(forward, 1851);

        let half =
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 500, 2816);
        assert_eq!(half, 926);

        let reversed_dir =
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -9, true, 1000, 2816);
        assert_eq!(reversed_dir, -1851);

        let decel =
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, 9, false, 1000, 2816);
        assert_eq!(decel, -1851);
    }

    #[test]
    fn acceleration_feedforward_split_uses_accel_gain_for_shrinking_interval() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);
        assert_eq!(
            compute_planner_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                -9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 1000,
                    decel_permille: 0,
                },
                2816
            ),
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -9, false, 1000, 2816)
        );
        assert_eq!(
            compute_planner_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                -9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 0,
                    decel_permille: 500,
                },
                2816
            ),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_split_uses_decel_gain_for_growing_interval() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);
        assert_eq!(
            compute_planner_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 0,
                    decel_permille: 500,
                },
                2816
            ),
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, 9, false, 500, 2816)
        );
        assert_eq!(
            compute_planner_accel_torque_offset_split(
                scale,
                84_000_000,
                2800,
                9,
                false,
                AccelFeedforwardGains {
                    accel_permille: 1000,
                    decel_permille: 0,
                },
                2816
            ),
            0
        );
    }

    #[test]
    fn acceleration_feedforward_clamps_to_the_supplied_offset_cap() {
        let scale = crate::motion_scale::PlannerVelocityScale::from_reduced(53_760, 1);
        let offset =
            compute_planner_accel_torque_offset(scale, 84_000_000, 2800, -90, false, 1000, 1200);
        assert_eq!(offset, 1200);
    }
}
