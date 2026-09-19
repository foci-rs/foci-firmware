//! Commissioning restore sequences used by terminal dispatch outcomes.
//!
//! Every error kind in this module (`PositionModeRestoreErrorKind`,
//! `GainsRestoreErrorKind`, `EntrySnapshotRestoreErrorKind`) erases its
//! backend's `B::Error` payload rather than carrying it generically: a
//! generic error would make the owning step type generic over `B`, and a
//! step type held inside `RestoreCursor` must stay `Copy` and
//! backend-independent for `RestoreCursor` itself to derive `Copy`.

#[cfg(test)]
use crate::commissioning::backend::CommissionTiming;
use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;
use foci_tmc4671::pid::{PidIGain, PidPGain};

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionModeRestoreErrorKind {
    PiRepresentation,
    InvalidGains,
    Spi,
}

#[derive(Clone, Copy)]
pub(crate) enum PositionModeRestoreStep {
    VerifyPiRepresentation {
        step: crate::tmc_control::pi_representation::PiRepresentationVerifyStep,
    },
    WriteVelocityGains {
        gains: crate::pid::OuterLoopGains,
    },
    WritePositionGains {
        gains: crate::pid::OuterLoopGains,
    },
    WriteVelocityLimit {
        gains: crate::pid::OuterLoopGains,
    },
    WriteCurrentLimit {
        gains: crate::pid::OuterLoopGains,
    },
    WriteVoltageCap {
        gains: crate::pid::OuterLoopGains,
    },
    ClearPositionActual {
        gains: crate::pid::OuterLoopGains,
    },
    ReadEncoderCount {
        gains: crate::pid::OuterLoopGains,
    },
    WritePositionTarget {
        gains: crate::pid::OuterLoopGains,
        target: u32,
    },
    EnterPositionMode {
        gains: crate::pid::OuterLoopGains,
    },
}

impl PositionModeRestoreStep {
    pub(crate) fn start() -> Self {
        Self::VerifyPiRepresentation {
            step: crate::tmc_control::pi_representation::PiRepresentationVerifyStep::start(),
        }
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        tmc_state: &TmcChannelState,
        outer: &crate::commissioning::types::OuterResult,
    ) -> Option<Result<crate::pid::OuterLoopGains, PositionModeRestoreErrorKind>> {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;

        match self {
            Self::VerifyPiRepresentation { step } => match step.advance(backend, tmc_state).await {
                None => None,
                Some(Err(_)) => Some(Err(PositionModeRestoreErrorKind::PiRepresentation)),
                Some(Ok(())) => {
                    let position_p = match PidPGain::new(outer.position_p) {
                        Ok(v) => v,
                        Err(_) => return Some(Err(PositionModeRestoreErrorKind::InvalidGains)),
                    };
                    let position_i = match PidIGain::new(outer.position_i) {
                        Ok(v) => v,
                        Err(_) => return Some(Err(PositionModeRestoreErrorKind::InvalidGains)),
                    };
                    let velocity_p = match PidPGain::new(outer.velocity_p) {
                        Ok(v) => v,
                        Err(_) => return Some(Err(PositionModeRestoreErrorKind::InvalidGains)),
                    };
                    let velocity_i = match PidIGain::new(outer.velocity_i) {
                        Ok(v) => v,
                        Err(_) => return Some(Err(PositionModeRestoreErrorKind::InvalidGains)),
                    };
                    let gains = crate::pid::OuterLoopGains {
                        position_p,
                        position_i,
                        velocity_p,
                        velocity_i,
                    };
                    *self = Self::WriteVelocityGains { gains };
                    None
                }
            },
            Self::WriteVelocityGains { gains } => {
                let gains = *gains;
                let (velocity_reg, _) = gains.to_registers();
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::WritePositionGains { gains };
                None
            }
            Self::WritePositionGains { gains } => {
                let gains = *gains;
                let (_, position_reg) = gains.to_registers();
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, position_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::WriteVelocityLimit { gains };
                None
            }
            Self::WriteVelocityLimit { gains } => {
                let gains = *gains;
                if backend
                    .write_register(PID_VELOCITY_LIMIT, outer.velocity_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::WriteCurrentLimit { gains };
                None
            }
            Self::WriteCurrentLimit { gains } => {
                let gains = *gains;
                let current_limit = tmc_state.load_current_limit(Relaxed);
                if backend
                    .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::WriteVoltageCap { gains };
                None
            }
            Self::WriteVoltageCap { gains } => {
                let gains = *gains;
                if backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, 1024)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::ClearPositionActual { gains };
                None
            }
            Self::ClearPositionActual { gains } => {
                let gains = *gains;
                if backend
                    .write_register(PID_POSITION_ACTUAL, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::ReadEncoderCount { gains };
                None
            }
            Self::ReadEncoderCount { gains } => {
                let gains = *gains;
                let encoder_count = match backend.read_register(ABN_DECODER_COUNT).await {
                    Ok(v) => v,
                    Err(_) => return Some(Err(PositionModeRestoreErrorKind::Spi)),
                };
                let target = encoder_count_to_position_target(
                    encoder_count,
                    backend.mechanical_encoder_quadrature_counts_per_rev(),
                );
                *self = Self::WritePositionTarget { gains, target };
                None
            }
            Self::WritePositionTarget { gains, target } => {
                let gains = *gains;
                let target = *target;
                if backend
                    .write_register(PID_POSITION_TARGET, target)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                *self = Self::EnterPositionMode { gains };
                None
            }
            Self::EnterPositionMode { gains } => {
                let gains = *gains;
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                    .await
                    .is_err()
                {
                    return Some(Err(PositionModeRestoreErrorKind::Spi));
                }
                Some(Ok(gains))
            }
        }
    }
}

fn encoder_count_to_position_target(encoder_count: u32, encoder_counts_per_rev: u32) -> u32 {
    assert!(
        encoder_counts_per_rev != 0,
        "encoder scale must be configured"
    );

    let scaled = i64::from(encoder_count as i32) * 65_536;
    let denominator = i64::from(encoder_counts_per_rev);
    let rounding = denominator / 2;
    let rounded = if scaled < 0 {
        (scaled - rounding) / denominator
    } else {
        (scaled + rounding) / denominator
    };
    rounded as u32
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionStabilityErrorKind {
    Spi,
    Stale,
}

/// Poll-driven replacement for `outer_restore_position_stable`, used only by
/// `RestoreCursor::OuterFailedStability`. Each `advance` call performs at
/// most one backend register read; between reads it returns `None` without
/// touching the backend until at least `OUTER_RESTORE_STABILITY_INTERVAL_US`
/// has elapsed since the previous sample (or since `start`, for the first
/// sample), and reports `Stale` if more than
/// `OUTER_RESTORE_STABILITY_MAX_SAMPLE_GAP_US` elapses past that deadline
/// before the sample can be taken.
#[derive(Clone, Copy)]
pub(crate) struct PositionStabilityStep {
    index: u8,
    next_sample_due_us: u64,
    first: i32,
    min_delta: i32,
    max_delta: i32,
}

impl PositionStabilityStep {
    pub(crate) fn start(now_us: u64) -> Self {
        Self {
            index: 0,
            next_sample_due_us: now_us
                .saturating_add(u64::from(super::OUTER_RESTORE_STABILITY_INTERVAL_US)),
            first: 0,
            min_delta: 0,
            max_delta: 0,
        }
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> Option<Result<super::RestoreStability, PositionStabilityErrorKind>> {
        let now = backend.elapsed_us();
        if now < self.next_sample_due_us {
            return None;
        }
        let stale_at = self
            .next_sample_due_us
            .saturating_add(u64::from(super::OUTER_RESTORE_STABILITY_MAX_SAMPLE_GAP_US));
        if now > stale_at {
            return Some(Err(PositionStabilityErrorKind::Stale));
        }
        let position = match backend
            .read_register(foci_tmc4671::registers::PID_POSITION_ACTUAL)
            .await
        {
            Ok(v) => v as i32,
            Err(_) => return Some(Err(PositionStabilityErrorKind::Spi)),
        };
        if self.index == 0 {
            self.first = position;
        }
        let delta = position.wrapping_sub(self.first);
        self.min_delta = self.min_delta.min(delta);
        self.max_delta = self.max_delta.max(delta);
        self.index += 1;
        if self.index < super::OUTER_RESTORE_STABILITY_SAMPLES as u8 {
            self.next_sample_due_us = backend
                .elapsed_us()
                .saturating_add(u64::from(super::OUTER_RESTORE_STABILITY_INTERVAL_US));
            return None;
        }
        Some(Ok(super::stability_from_span(
            self.min_delta,
            self.max_delta,
            backend.mechanical_encoder_quadrature_counts_per_rev(),
        )))
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum GainsRestoreErrorKind {
    PiRepresentation,
    Spi,
    InvalidGains,
}

#[derive(Clone, Copy)]
pub(crate) enum CalibrateGainsRestoreStep {
    VerifyPiRepresentation {
        stored: Option<crate::pid::InnerLoopGains>,
        step: crate::tmc_control::pi_representation::PiRepresentationVerifyStep,
    },
    WriteFlux {
        gains: crate::pid::InnerLoopGains,
    },
    WriteTorque {
        torque_reg: u32,
    },
    WriteVelocity,
    WritePosition {
        position_reg: u32,
    },
    WriteVelocityLimit,
    WriteCurrentLimit,
}

impl CalibrateGainsRestoreStep {
    pub(crate) fn start(stored: Option<crate::pid::InnerLoopGains>) -> Self {
        Self::VerifyPiRepresentation {
            stored,
            step: crate::tmc_control::pi_representation::PiRepresentationVerifyStep::start(),
        }
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        tmc_state: &TmcChannelState,
    ) -> Option<Result<(), GainsRestoreErrorKind>> {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;

        match self {
            Self::VerifyPiRepresentation { stored, step } => {
                let stored = *stored;
                match step.advance(backend, tmc_state).await {
                    None => None,
                    Some(Err(_)) => Some(Err(GainsRestoreErrorKind::PiRepresentation)),
                    Some(Ok(())) => {
                        *self = match stored {
                            Some(gains) => Self::WriteFlux { gains },
                            None => Self::WriteVelocity,
                        };
                        None
                    }
                }
            }
            Self::WriteFlux { gains } => {
                let gains = *gains;
                let (flux_reg, torque_reg) = gains.to_registers();
                if backend
                    .write_register(PID_FLUX_P_FLUX_I, flux_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                *self = Self::WriteTorque { torque_reg };
                None
            }
            Self::WriteTorque { torque_reg } => {
                let torque_reg = *torque_reg;
                if backend
                    .write_register(PID_TORQUE_P_TORQUE_I, torque_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                *self = Self::WriteVelocity;
                None
            }
            Self::WriteVelocity => {
                let Ok(gains) = tmc_state.load_outer_gains(Relaxed) else {
                    return Some(Err(GainsRestoreErrorKind::InvalidGains));
                };
                let (velocity_reg, position_reg) = gains.to_registers();
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                *self = Self::WritePosition { position_reg };
                None
            }
            Self::WritePosition { position_reg } => {
                let position_reg = *position_reg;
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, position_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                *self = Self::WriteVelocityLimit;
                None
            }
            Self::WriteVelocityLimit => {
                let velocity_limit = tmc_state.velocity_limit_cfg.load(Relaxed);
                if backend
                    .write_register(PID_VELOCITY_LIMIT, velocity_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                *self = Self::WriteCurrentLimit;
                None
            }
            Self::WriteCurrentLimit => {
                let current_limit = tmc_state.load_current_limit(Relaxed);
                if backend
                    .write_register(PID_TORQUE_FLUX_LIMITS, current_limit.register_value())
                    .await
                    .is_err()
                {
                    return Some(Err(GainsRestoreErrorKind::Spi));
                }
                Some(Ok(()))
            }
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum EntrySnapshotRestoreErrorKind {
    Spi,
    Readback,
}

/// Does **not** touch `PID_TORQUE_FLUX_LIMITS`: the operational current
/// limit lives in `tmc_state`, not the snapshot, and is restored separately
/// by the velocity-disarm step's `ClampRestore` phase, which must run after
/// this one.
#[derive(Clone, Copy)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum EntrySnapshotRestoreStep {
    WriteVelocityGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    },
    WritePositionGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    },
    WriteVelocityLimit {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    },
    WriteVoltageCap {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    },
    Filters {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        step: crate::tmc_control::filters::FilterCutoffsStep,
    },
    ReadbackVelocityGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ReadbackPositionGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ReadbackVelocityLimit {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ReadbackVoltageCap {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
}

impl EntrySnapshotRestoreStep {
    pub(crate) fn start(
        snapshot: &crate::commissioning::OuterGainRollbackSnapshot,
        pwm_maxcnt: u16,
    ) -> Self {
        Self::WriteVelocityGains {
            snapshot: *snapshot,
            pwm_maxcnt,
        }
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
    ) -> Option<Result<(), EntrySnapshotRestoreErrorKind>> {
        use foci_tmc4671::registers::*;

        fn gains(
            snapshot: &crate::commissioning::OuterGainRollbackSnapshot,
        ) -> crate::pid::OuterLoopGains {
            crate::pid::OuterLoopGains {
                position_p: snapshot.position_p,
                position_i: snapshot.position_i,
                velocity_p: snapshot.velocity_p,
                velocity_i: snapshot.velocity_i,
            }
        }

        match self {
            Self::WriteVelocityGains {
                snapshot,
                pwm_maxcnt,
            } => {
                let (snapshot, pwm_maxcnt) = (*snapshot, *pwm_maxcnt);
                let (velocity_reg, _) = gains(&snapshot).to_registers();
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Spi));
                }
                *self = Self::WritePositionGains {
                    snapshot,
                    pwm_maxcnt,
                };
                None
            }
            Self::WritePositionGains {
                snapshot,
                pwm_maxcnt,
            } => {
                let (snapshot, pwm_maxcnt) = (*snapshot, *pwm_maxcnt);
                let (_, position_reg) = gains(&snapshot).to_registers();
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, position_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Spi));
                }
                *self = Self::WriteVelocityLimit {
                    snapshot,
                    pwm_maxcnt,
                };
                None
            }
            Self::WriteVelocityLimit {
                snapshot,
                pwm_maxcnt,
            } => {
                let (snapshot, pwm_maxcnt) = (*snapshot, *pwm_maxcnt);
                if backend
                    .write_register(PID_VELOCITY_LIMIT, snapshot.velocity_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Spi));
                }
                *self = Self::WriteVoltageCap {
                    snapshot,
                    pwm_maxcnt,
                };
                None
            }
            Self::WriteVoltageCap {
                snapshot,
                pwm_maxcnt,
            } => {
                let (snapshot, pwm_maxcnt) = (*snapshot, *pwm_maxcnt);
                if backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, snapshot.voltage_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Spi));
                }
                *self = Self::Filters {
                    snapshot,
                    step: crate::tmc_control::filters::FilterCutoffsStep::from_entry_snapshot(
                        &snapshot, pwm_maxcnt,
                    ),
                };
                None
            }
            Self::Filters { snapshot, step } => {
                let snapshot = *snapshot;
                match step.advance(backend).await {
                    None => None,
                    Some(Ok(())) => {
                        *self = Self::ReadbackVelocityGains { snapshot };
                        None
                    }
                    Some(Err(())) => Some(Err(EntrySnapshotRestoreErrorKind::Spi)),
                }
            }
            Self::ReadbackVelocityGains { snapshot } => {
                let snapshot = *snapshot;
                let (velocity_reg, _) = gains(&snapshot).to_registers();
                let readback = match backend.read_register(PID_VELOCITY_P_VELOCITY_I).await {
                    Ok(v) => v,
                    Err(_) => return Some(Err(EntrySnapshotRestoreErrorKind::Spi)),
                };
                if readback != velocity_reg {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Readback));
                }
                *self = Self::ReadbackPositionGains { snapshot };
                None
            }
            Self::ReadbackPositionGains { snapshot } => {
                let snapshot = *snapshot;
                let (_, position_reg) = gains(&snapshot).to_registers();
                let readback = match backend.read_register(PID_POSITION_P_POSITION_I).await {
                    Ok(v) => v,
                    Err(_) => return Some(Err(EntrySnapshotRestoreErrorKind::Spi)),
                };
                if readback != position_reg {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Readback));
                }
                *self = Self::ReadbackVelocityLimit { snapshot };
                None
            }
            Self::ReadbackVelocityLimit { snapshot } => {
                let snapshot = *snapshot;
                let readback = match backend.read_register(PID_VELOCITY_LIMIT).await {
                    Ok(v) => v,
                    Err(_) => return Some(Err(EntrySnapshotRestoreErrorKind::Spi)),
                };
                if readback != snapshot.velocity_limit {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Readback));
                }
                *self = Self::ReadbackVoltageCap { snapshot };
                None
            }
            Self::ReadbackVoltageCap { snapshot } => {
                let readback = match backend.read_register(PIDOUT_UQ_UD_LIMITS).await {
                    Ok(v) => v,
                    Err(_) => return Some(Err(EntrySnapshotRestoreErrorKind::Spi)),
                };
                if readback != snapshot.voltage_limit {
                    return Some(Err(EntrySnapshotRestoreErrorKind::Readback));
                }
                Some(Ok(()))
            }
        }
    }
}

/// Reloads the four per-axis biquad filters from an `OuterGainRollbackSnapshot`,
/// one raw backend write per `advance()` call.
#[derive(Clone, Copy)]
pub(crate) struct EntryGainFiltersStep {
    // (register, value, index to jump to if this write fails). The disable
    // pair's on_fail_next is always the next index (both writes always run,
    // matching write_sub_register's unconditional double-write); an enable
    // sequence's on_fail_next is that filter's end boundary for every op
    // (matching write_biquad_*_checked's `?`-chained abort-the-rest-of-this-
    // filter behavior, which the original outer wrapper then swallowed).
    ops: [(foci_tmc4671::registers::Register, u32, u8); 48],
    len: u8,
    next: u8,
}

impl EntryGainFiltersStep {
    fn start(snapshot: &crate::commissioning::OuterGainRollbackSnapshot, pwm_maxcnt: u16) -> Self {
        use foci_tmc4671::registers::*;

        fn push_disable_pair(
            ops: &mut [(Register, u32, u8); 48],
            sub: SubRegister,
            len: &mut usize,
        ) {
            ops[*len] = (sub.addr, u32::from(sub.sub_addr), (*len + 1) as u8);
            *len += 1;
            ops[*len] = (sub.data, 0, (*len + 1) as u8);
            *len += 1;
        }

        let placeholder = Register::new(0);
        let mut ops = [(placeholder, 0u32, 0u8); 48];
        let mut len = 0usize;

        for (filter_hz, coeff_regs, enable_reg, max_hz) in [
            (
                snapshot.velocity_filter_hz,
                [
                    CONFIG_BIQUAD_V_A_1,
                    CONFIG_BIQUAD_V_A_2,
                    CONFIG_BIQUAD_V_B_0,
                    CONFIG_BIQUAD_V_B_1,
                    CONFIG_BIQUAD_V_B_2,
                ],
                CONFIG_BIQUAD_V_ENABLE,
                crate::biquad::MOTION_CUTOFF_MAX_HZ,
            ),
            (
                snapshot.torque_filter_hz,
                [
                    CONFIG_BIQUAD_T_A_1,
                    CONFIG_BIQUAD_T_A_2,
                    CONFIG_BIQUAD_T_B_0,
                    CONFIG_BIQUAD_T_B_1,
                    CONFIG_BIQUAD_T_B_2,
                ],
                CONFIG_BIQUAD_T_ENABLE,
                crate::biquad::CURRENT_CUTOFF_MAX_HZ,
            ),
            (
                snapshot.position_filter_hz,
                [
                    CONFIG_BIQUAD_X_A_1,
                    CONFIG_BIQUAD_X_A_2,
                    CONFIG_BIQUAD_X_B_0,
                    CONFIG_BIQUAD_X_B_1,
                    CONFIG_BIQUAD_X_B_2,
                ],
                CONFIG_BIQUAD_X_ENABLE,
                crate::biquad::MOTION_CUTOFF_MAX_HZ,
            ),
            (
                snapshot.flux_filter_hz,
                [
                    CONFIG_BIQUAD_F_A_1,
                    CONFIG_BIQUAD_F_A_2,
                    CONFIG_BIQUAD_F_B_0,
                    CONFIG_BIQUAD_F_B_1,
                    CONFIG_BIQUAD_F_B_2,
                ],
                CONFIG_BIQUAD_F_ENABLE,
                crate::biquad::CURRENT_CUTOFF_MAX_HZ,
            ),
        ] {
            if filter_hz == 0 {
                push_disable_pair(&mut ops, enable_reg, &mut len);
                continue;
            }
            let Some(coeffs) = crate::biquad::butterworth_lowpass(filter_hz, pwm_maxcnt, max_hz)
            else {
                continue;
            };
            let start = len;
            for (sub, value) in [
                (coeff_regs[0], coeffs.a_1 as u32),
                (coeff_regs[1], coeffs.a_2 as u32),
                (coeff_regs[2], coeffs.b_0 as u32),
                (coeff_regs[3], coeffs.b_1 as u32),
                (coeff_regs[4], coeffs.b_2 as u32),
                (enable_reg, 1),
            ] {
                ops[len] = (sub.addr, u32::from(sub.sub_addr), 0);
                len += 1;
                ops[len] = (sub.data, value, 0);
                len += 1;
            }
            // Every op in this filter's enable sequence jumps straight to
            // the filter's end on failure, whether it fails at a selector
            // or a data write -- matching write_biquad_*_checked's `?`
            // chain, which aborts every remaining sub-register pair in the
            // same filter (not just the one in progress).
            for op in &mut ops[start..len] {
                op.2 = len as u8;
            }
        }

        Self {
            ops,
            len: len as u8,
            next: 0,
        }
    }

    /// Returns `true` once every queued op has been attempted (successfully
    /// or not -- failures are swallowed, never reported).
    async fn advance<B: CommissioningBackend>(&mut self, backend: &mut B) -> bool {
        if self.next >= self.len {
            return true;
        }
        let (reg, value, on_fail_next) = self.ops[self.next as usize];
        self.next = if backend.write_register(reg, value).await.is_ok() {
            self.next + 1
        } else {
            on_fail_next
        };
        self.next >= self.len
    }
}

/// Failure reason for [`RestoreEntryGainsStep`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum RestoreEntryGainsErrorKind {
    Spi,
    PiRepresentation,
    FaultInterlock,
}

impl From<RestoreEntryGainsErrorKind> for crate::autotune::CommissionError {
    fn from(kind: RestoreEntryGainsErrorKind) -> Self {
        match kind {
            RestoreEntryGainsErrorKind::Spi => Self::Spi,
            RestoreEntryGainsErrorKind::PiRepresentation => Self::ChipResetDetected,
            RestoreEntryGainsErrorKind::FaultInterlock => Self::FaultInterlock,
        }
    }
}

/// Restores outer gains from an entry snapshot after a Stage 2 soft failure
/// or a safety-fault gain restore, one raw backend operation per
/// `advance()` call.
#[derive(Clone, Copy)]
#[allow(clippy::large_enum_variant)]
pub(crate) enum RestoreEntryGainsStep {
    ZeroTorqueFluxTarget {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ZeroVelocityTarget {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ZeroVelocityOffset {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    DisableMotionMode {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ClearIntegrators {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        ops: [(foci_tmc4671::registers::Register, u32); 8],
        next: u8,
    },
    ClearStatusFlags {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    VerifyPiRepresentation {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        step: crate::tmc_control::pi_representation::PiRepresentationVerifyStep,
    },
    WriteVelocityGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    WritePositionGains {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    WriteVelocityLimit {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    Filters {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
        step: EntryGainFiltersStep,
    },
    WriteVoltageLimit {
        snapshot: crate::commissioning::OuterGainRollbackSnapshot,
    },
    ZeroPosition,
    EnterPositionMode,
}

impl RestoreEntryGainsStep {
    pub(crate) fn start(snapshot: &crate::commissioning::OuterGainRollbackSnapshot) -> Self {
        Self::ZeroTorqueFluxTarget {
            snapshot: *snapshot,
        }
    }

    pub(crate) async fn advance<B: CommissioningBackend>(
        &mut self,
        backend: &mut B,
        tmc_state: &crate::state::tmc::TmcChannelState,
    ) -> Option<Result<(), RestoreEntryGainsErrorKind>> {
        use foci_tmc4671::pid::PidGainPair;
        use foci_tmc4671::registers::*;

        match self {
            Self::ZeroTorqueFluxTarget { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(PID_TORQUE_FLUX_TARGET, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::ZeroVelocityTarget { snapshot };
                None
            }
            Self::ZeroVelocityTarget { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(PID_VELOCITY_TARGET, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::ZeroVelocityOffset { snapshot };
                None
            }
            Self::ZeroVelocityOffset { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(PID_VELOCITY_OFFSET, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::DisableMotionMode { snapshot };
                None
            }
            Self::DisableMotionMode { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                let mut ops = [(Register::new(0), 0u32); 8];
                for (i, sub) in [
                    PID_ERROR_PID_TORQUE_ERROR_SUM,
                    PID_ERROR_PID_FLUX_ERROR_SUM,
                    PID_ERROR_PID_VELOCITY_ERROR_SUM,
                    PID_ERROR_PID_POSITION_ERROR_SUM,
                ]
                .into_iter()
                .enumerate()
                {
                    ops[i * 2] = (sub.addr, u32::from(sub.sub_addr));
                    ops[i * 2 + 1] = (sub.data, 0);
                }
                *self = Self::ClearIntegrators {
                    snapshot,
                    ops,
                    next: 0,
                };
                None
            }
            Self::ClearIntegrators {
                snapshot,
                ops,
                next,
            } => {
                let snapshot = *snapshot;
                let (reg, value) = ops[*next as usize];
                if backend.write_register(reg, value).await.is_err() {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *next += 1;
                if *next >= 8 {
                    *self = Self::ClearStatusFlags { snapshot };
                }
                None
            }
            Self::ClearStatusFlags { snapshot } => {
                let snapshot = *snapshot;
                match crate::tmc_control::interlock::clear_status_flags_via_backend(backend).await {
                    Ok(()) => {
                        *self = Self::VerifyPiRepresentation {
                            snapshot,
                            step:
                                crate::tmc_control::pi_representation::PiRepresentationVerifyStep::start(),
                        };
                        None
                    }
                    Err(crate::autotune::CommissionError::FaultInterlock) => {
                        Some(Err(RestoreEntryGainsErrorKind::FaultInterlock))
                    }
                    Err(_) => Some(Err(RestoreEntryGainsErrorKind::Spi)),
                }
            }
            Self::VerifyPiRepresentation { snapshot, step } => {
                let snapshot = *snapshot;
                match step.advance(backend, tmc_state).await {
                    None => None,
                    Some(Ok(())) => {
                        *self = Self::WriteVelocityGains { snapshot };
                        None
                    }
                    Some(Err(_)) => Some(Err(RestoreEntryGainsErrorKind::PiRepresentation)),
                }
            }
            Self::WriteVelocityGains { snapshot } => {
                let snapshot = *snapshot;
                let velocity_reg =
                    PidGainPair::new(snapshot.velocity_p, snapshot.velocity_i).register_value();
                if backend
                    .write_register(PID_VELOCITY_P_VELOCITY_I, velocity_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::WritePositionGains { snapshot };
                None
            }
            Self::WritePositionGains { snapshot } => {
                let snapshot = *snapshot;
                let position_reg =
                    PidGainPair::new(snapshot.position_p, snapshot.position_i).register_value();
                if backend
                    .write_register(PID_POSITION_P_POSITION_I, position_reg)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::WriteVelocityLimit { snapshot };
                None
            }
            Self::WriteVelocityLimit { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(PID_VELOCITY_LIMIT, snapshot.velocity_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::Filters {
                    snapshot,
                    step: EntryGainFiltersStep::start(&snapshot, backend.pwm_maxcnt()),
                };
                None
            }
            Self::Filters { snapshot, step } => {
                let snapshot = *snapshot;
                if step.advance(backend).await {
                    *self = Self::WriteVoltageLimit { snapshot };
                }
                None
            }
            Self::WriteVoltageLimit { snapshot } => {
                let snapshot = *snapshot;
                if backend
                    .write_register(PIDOUT_UQ_UD_LIMITS, snapshot.voltage_limit)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::ZeroPosition;
                None
            }
            Self::ZeroPosition => {
                if backend
                    .write_register(PID_POSITION_ACTUAL, 0)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                *self = Self::EnterPositionMode;
                None
            }
            Self::EnterPositionMode => {
                if backend
                    .write_register(MODE_RAMP_MODE_MOTION, 0x8000_0003)
                    .await
                    .is_err()
                {
                    return Some(Err(RestoreEntryGainsErrorKind::Spi));
                }
                Some(Ok(()))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
    use foci_firmware_macros::fixture_test;

    fn tmc_state() -> &'static crate::state::tmc::TmcChannelState {
        let state = crate::state::tmc::tmc_channel0();
        state.mark_pi_representation_established();
        state
    }

    fn write_log(backend: &MockBackend) -> heapless::Vec<(u8, u32), 128> {
        let write_count = backend
            .spi_log
            .iter()
            .filter(|op| matches!(op, SpiOp::Write(_, _)))
            .count();
        let writes = backend
            .spi_log
            .iter()
            .filter_map(|op| {
                if let SpiOp::Write(addr, val) = op {
                    Some((*addr, *val))
                } else {
                    None
                }
            })
            .collect::<heapless::Vec<_, 128>>();
        assert_eq!(
            writes.len(),
            write_count,
            "write_log helper capacity truncated the SPI log"
        );
        writes
    }

    #[fixture_test]
    async fn position_mode_restore_step_writes_exact_sequence() {
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        tmc_state().store_current_limit(
            crate::commission_dispatch::test_support::current_limit(0x10),
            core::sync::atomic::Ordering::Relaxed,
        );
        // Pre-seed ABN_DECODER_COUNT for the position-target compute.
        let _ = backend.write_register(ABN_DECODER_COUNT, 1234).await;
        backend.spi_log.clear();

        let outer = sample_outer_result();
        let mut step = PositionModeRestoreStep::start();
        let restored = loop {
            match step.advance(&mut backend, tmc_state(), &outer).await {
                None => continue,
                Some(Ok(gains)) => break gains,
                Some(Err(kind)) => panic!("stepped restore failed: {:?}", kind),
            }
        };

        let (velocity_reg, position_reg) = restored.to_registers();
        assert_eq!(
            write_log(&backend).as_slice(),
            &[
                (
                    CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                    u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                ),
                (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                (PID_POSITION_P_POSITION_I.addr(), position_reg),
                (PID_VELOCITY_LIMIT.addr(), outer.velocity_limit),
                (PID_TORQUE_FLUX_LIMITS.addr(), 0x10),
                (PIDOUT_UQ_UD_LIMITS.addr(), 1024),
                (PID_POSITION_ACTUAL.addr(), 0),
                (PID_POSITION_TARGET.addr(), 20_218),
                (MODE_RAMP_MODE_MOTION.addr(), 0x8000_0003),
            ]
        );

        // Cleanup atomics.
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            core::sync::atomic::Ordering::Relaxed,
        );
    }

    #[fixture_test]
    async fn position_mode_restore_step_propagates_spi_failure() {
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(MODE_RAMP_MODE_MOTION.addr());
        let outer = sample_outer_result();
        let mut step = PositionModeRestoreStep::start();
        let result = loop {
            match step.advance(&mut backend, tmc_state(), &outer).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_err());
    }

    #[fixture_test]
    async fn position_mode_restore_step_advances_one_op_per_call() {
        let outer = sample_outer_result();
        let mut backend = MockBackend::new();
        let mut step = PositionModeRestoreStep::start();
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend, tmc_state(), &outer).await;
            let after = backend.spi_log.len();
            assert!(
                after - before <= 1,
                "tick {}: advance() issued {} ops, expected at most 1",
                ticks,
                after - before
            );
            ticks += 1;
            if !matches!(result, None) {
                break;
            }
        }
        assert_eq!(ticks, 11);
    }

    fn sample_outer_result() -> crate::commissioning::types::OuterResult {
        crate::commissioning::types::OuterResult {
            status: crate::autotune::CommissionStatus::Accepted,
            velocity_p: 1000,
            velocity_i: 100,
            velocity_limit: 500_000,
            position_p: 640,
            position_i: 0,
            velocity_filter_hz: 0,
            position_filter_hz: 0,
            flux_filter_hz: 0,
            torque_filter_hz: 0,
            warning_code: 0,
            provenance: crate::commissioning::types::TuneProvenance::default(),
        }
    }

    #[fixture_test]
    async fn calibrate_gains_restore_step_writes_all_gains_when_stored_present() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        let outer_gains = crate::pid::test_outer_gains(333, 44, 111, 22);
        tmc_state().store_outer_gains(outer_gains, Relaxed);
        tmc_state().velocity_limit_cfg.store(555_000, Relaxed);
        tmc_state().store_current_limit(
            crate::commission_dispatch::test_support::current_limit(0x30),
            Relaxed,
        );
        let stored_gains = crate::pid::InnerLoopGains {
            flux_p: crate::pid::p_gain(100),
            flux_i: crate::pid::i_gain(10),
            torque_p: crate::pid::p_gain(200),
            torque_i: crate::pid::i_gain(20),
        };
        let (flux_reg, torque_reg) = stored_gains.to_registers();
        let mut step = CalibrateGainsRestoreStep::start(Some(stored_gains));
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());
        let (velocity_reg, position_reg) = outer_gains.to_registers();
        assert_eq!(
            write_log(&backend).as_slice(),
            &[
                (
                    CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                    u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                ),
                (PID_FLUX_P_FLUX_I.addr(), flux_reg),
                (PID_TORQUE_P_TORQUE_I.addr(), torque_reg),
                (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                (PID_POSITION_P_POSITION_I.addr(), position_reg),
                (PID_VELOCITY_LIMIT.addr(), 555_000),
                (PID_TORQUE_FLUX_LIMITS.addr(), 0x30),
            ]
        );
        tmc_state().store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
        tmc_state().velocity_limit_cfg.store(0, Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Relaxed,
        );
    }

    #[fixture_test]
    async fn calibrate_gains_restore_step_skips_inner_when_stored_none() {
        use core::sync::atomic::Ordering::Relaxed;
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        let outer_gains = crate::pid::test_outer_gains(377, 55, 211, 33);
        tmc_state().store_outer_gains(outer_gains, Relaxed);
        tmc_state().velocity_limit_cfg.store(666_000, Relaxed);
        tmc_state().store_current_limit(
            crate::commission_dispatch::test_support::current_limit(0x50),
            Relaxed,
        );
        let mut step = CalibrateGainsRestoreStep::start(None);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());
        let (velocity_reg, position_reg) = outer_gains.to_registers();
        assert_eq!(
            write_log(&backend).as_slice(),
            &[
                (
                    CONFIG_ADVANCED_PI_REPRESENT.addr.addr(),
                    u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr),
                ),
                (PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg),
                (PID_POSITION_P_POSITION_I.addr(), position_reg),
                (PID_VELOCITY_LIMIT.addr(), 666_000),
                (PID_TORQUE_FLUX_LIMITS.addr(), 0x50),
            ]
        );
        tmc_state().store_outer_gains(crate::pid::DEFAULT_OUTER_GAINS, Relaxed);
        tmc_state().velocity_limit_cfg.store(0, Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::ZERO,
            Relaxed,
        );
    }

    #[fixture_test]
    async fn calibrate_gains_restore_step_rejects_corrupt_outer_storage_before_pid_write() {
        let mut backend = MockBackend::new();
        tmc_state().corrupt_outer_gain_for_test(0x8000);

        let mut step = CalibrateGainsRestoreStep::start(None);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(GainsRestoreErrorKind::InvalidGains));
        assert!(!backend.spi_log.iter().any(|op| {
            matches!(
                op,
                SpiOp::Write(addr, _)
                    if *addr
                        == foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
                        || *addr
                            == foci_tmc4671::registers::PID_POSITION_P_POSITION_I.addr()
            )
        }));
        tmc_state().store_outer_gains(
            crate::pid::DEFAULT_OUTER_GAINS,
            core::sync::atomic::Ordering::Relaxed,
        );
    }

    #[fixture_test]
    async fn calibrate_gains_restore_step_rejects_pi_representation_mismatch_before_pid_write() {
        let mut backend = MockBackend::new();
        tmc_state().mark_pi_representation_established();
        backend.read_override = Some((
            foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .data
                .addr(),
            0,
        ));

        let mut step = CalibrateGainsRestoreStep::start(None);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(GainsRestoreErrorKind::PiRepresentation));
        assert!(!backend.spi_log.iter().any(|op| {
            matches!(
                op,
                SpiOp::Write(addr, _)
                    if *addr
                        == foci_tmc4671::registers::PID_VELOCITY_P_VELOCITY_I.addr()
                        || *addr
                            == foci_tmc4671::registers::PID_POSITION_P_POSITION_I.addr()
            )
        }));
        tmc_state().clear_pi_representation_established();
    }

    #[fixture_test]
    async fn calibrate_gains_restore_step_advances_one_op_per_call() {
        use core::sync::atomic::Ordering::Relaxed;
        crate::state::tmc::tmc_channel0().reset();
        tmc_state().store_outer_gains(crate::pid::test_outer_gains(333, 44, 111, 22), Relaxed);
        tmc_state().velocity_limit_cfg.store(555_000, Relaxed);
        tmc_state().store_current_limit(
            foci_tmc4671::current_limit::PidTorqueFluxLimit::new(1234).unwrap(),
            Relaxed,
        );
        tmc_state().mark_pi_representation_established();
        let stored = crate::pid::test_inner_gains(100, 10, 200, 20);

        let mut backend = MockBackend::new();
        let mut step = CalibrateGainsRestoreStep::start(Some(stored));
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend, tmc_state()).await;
            let after = backend.spi_log.len();
            assert!(
                after - before <= 1,
                "tick {}: advance() issued {} ops, expected at most 1",
                ticks,
                after - before
            );
            ticks += 1;
            if !matches!(result, None) {
                break;
            }
        }

        crate::state::tmc::tmc_channel0().reset();
    }

    fn sample_entry_snapshot() -> crate::commissioning::OuterGainRollbackSnapshot {
        crate::commission_dispatch::test_support::sample_entry_snapshot()
    }

    fn snapshot_outer_gains(
        snap: &crate::commissioning::OuterGainRollbackSnapshot,
    ) -> crate::pid::OuterLoopGains {
        crate::pid::OuterLoopGains {
            position_p: snap.position_p,
            position_i: snap.position_i,
            velocity_p: snap.velocity_p,
            velocity_i: snap.velocity_i,
        }
    }

    #[fixture_test]
    async fn entry_snapshot_restore_step_writes_gains_limits_and_validates_readback() {
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        let snap = sample_entry_snapshot();

        let mut step = EntrySnapshotRestoreStep::start(&snap, 4095);
        let result = loop {
            match step.advance(&mut backend).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());

        let (velocity_reg, position_reg) = snapshot_outer_gains(&snap).to_registers();
        let w = write_log(&backend);
        assert!(w.contains(&(PID_VELOCITY_P_VELOCITY_I.addr(), velocity_reg)));
        assert!(w.contains(&(PID_POSITION_P_POSITION_I.addr(), position_reg)));
        assert!(w.contains(&(PID_VELOCITY_LIMIT.addr(), snap.velocity_limit)));
        assert!(w.contains(&(PIDOUT_UQ_UD_LIMITS.addr(), snap.voltage_limit)));
        assert!(
            !w.iter()
                .any(|(addr, _)| *addr == PID_TORQUE_FLUX_LIMITS.addr()),
            "restore must not touch the current limit -- it is not snapshot-owned"
        );
    }

    #[fixture_test]
    async fn entry_snapshot_restore_step_writes_filters_when_present() {
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 200;

        let mut step = EntrySnapshotRestoreStep::start(&snap, 4095);
        let result = loop {
            match step.advance(&mut backend).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());
        assert!(
            backend
                .spi_log
                .iter()
                .any(|op| matches!(op, SpiOp::Write(addr, _) if *addr == CONFIG_BIQUAD_V_ENABLE.data.addr())),
            "a non-zero velocity filter cutoff must enable the velocity biquad filter"
        );
    }

    #[fixture_test]
    async fn entry_snapshot_restore_step_propagates_spi_write_failure() {
        use foci_tmc4671::registers::*;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(PIDOUT_UQ_UD_LIMITS.addr());
        let snap = sample_entry_snapshot();

        let mut step = EntrySnapshotRestoreStep::start(&snap, 4095);
        let result = loop {
            match step.advance(&mut backend).await {
                None => continue,
                Some(result) => break result,
            }
        };

        assert_eq!(result, Err(EntrySnapshotRestoreErrorKind::Spi));
    }

    #[fixture_test]
    async fn entry_snapshot_restore_step_advances_one_op_per_call() {
        let snap = sample_entry_snapshot();
        let mut backend = MockBackend::new();
        let mut step = EntrySnapshotRestoreStep::start(&snap, 4095);
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend).await;
            let after = backend.spi_log.len();
            assert!(
                after - before <= 1,
                "tick {}: advance() issued {} ops, expected at most 1",
                ticks,
                after - before
            );
            ticks += 1;
            if !matches!(result, None) {
                break;
            }
        }
    }

    #[fixture_test]
    async fn entry_snapshot_restore_step_readback_mismatch_is_an_error() {
        use foci_tmc4671::registers::*;
        let snap = sample_entry_snapshot();
        let mut backend = MockBackend::new();
        backend.read_override = Some((PID_VELOCITY_LIMIT.addr(), 0xDEAD_BEEF));
        let mut step = EntrySnapshotRestoreStep::start(&snap, 4095);
        let result = loop {
            match step.advance(&mut backend).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(result, Err(EntrySnapshotRestoreErrorKind::Readback));
    }

    const TEST_INTERVAL_US: u64 =
        crate::commission_dispatch::OUTER_RESTORE_STABILITY_INTERVAL_US as u64;
    const TEST_MAX_GAP_US: u64 =
        crate::commission_dispatch::OUTER_RESTORE_STABILITY_MAX_SAMPLE_GAP_US as u64;

    #[fixture_test]
    async fn position_stability_step_waits_for_deadline_before_sampling() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 0;
        let mut step = PositionStabilityStep::start(backend.elapsed_us());

        assert!(step.advance(&mut backend).await.is_none());
        assert!(backend.spi_log.is_empty());

        backend.elapsed_us_value = TEST_INTERVAL_US;
        backend.read_position_actual_sequence.push(100).unwrap();
        assert!(step.advance(&mut backend).await.is_none());
        assert_eq!(
            backend
                .spi_log
                .iter()
                .filter(|op| matches!(op, SpiOp::Read))
                .count(),
            1
        );
    }

    #[fixture_test]
    async fn position_stability_step_reports_stable_within_threshold() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 0;
        for position in [1000u32, 1000, 1031, 1001, 1000] {
            backend
                .read_position_actual_sequence
                .push(position)
                .unwrap();
        }
        let mut step = PositionStabilityStep::start(backend.elapsed_us());
        let stability = loop {
            backend.elapsed_us_value += TEST_INTERVAL_US;
            if let Some(result) = step.advance(&mut backend).await {
                break result.expect("no SPI failure injected");
            }
        };
        assert!(stability.stable);
    }

    #[fixture_test]
    async fn position_stability_step_reports_unstable_over_threshold() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 0;
        for position in [1000u32, 1000, 1200, 1001, 1000] {
            backend
                .read_position_actual_sequence
                .push(position)
                .unwrap();
        }
        let mut step = PositionStabilityStep::start(backend.elapsed_us());
        let stability = loop {
            backend.elapsed_us_value += TEST_INTERVAL_US;
            if let Some(result) = step.advance(&mut backend).await {
                break result.expect("no SPI failure injected");
            }
        };
        assert!(!stability.stable);
    }

    #[fixture_test]
    async fn position_stability_step_propagates_spi_error() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = TEST_INTERVAL_US;
        backend.fail_on_read = Some(foci_tmc4671::registers::PID_POSITION_ACTUAL.addr());
        let mut step = PositionStabilityStep::start(0);
        let outcome = step.advance(&mut backend).await;
        assert!(matches!(
            outcome,
            Some(Err(PositionStabilityErrorKind::Spi))
        ));
    }

    #[fixture_test]
    async fn position_stability_step_tolerates_a_gap_within_the_max_slack() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 0;
        backend.read_position_actual_sequence.push(1000).unwrap();
        let mut step = PositionStabilityStep::start(backend.elapsed_us());
        backend.elapsed_us_value = TEST_INTERVAL_US + TEST_MAX_GAP_US;
        let outcome = step.advance(&mut backend).await;
        assert!(
            outcome.is_none(),
            "a sample exactly at the slack boundary must still be taken"
        );
        assert_eq!(
            backend
                .spi_log
                .iter()
                .filter(|op| matches!(op, SpiOp::Read))
                .count(),
            1
        );
    }

    #[fixture_test]
    async fn position_stability_step_reports_stale_past_the_max_slack() {
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 0;
        let mut step = PositionStabilityStep::start(backend.elapsed_us());
        backend.elapsed_us_value = TEST_INTERVAL_US + TEST_MAX_GAP_US + 1;
        let outcome = step.advance(&mut backend).await;
        assert!(matches!(
            outcome,
            Some(Err(PositionStabilityErrorKind::Stale))
        ));
        assert!(
            backend.spi_log.is_empty(),
            "a stale gap must not attempt a register read"
        );
    }

    #[fixture_test]
    async fn restore_entry_gains_step_matches_flat_function_all_filters_zero() {
        let snap = sample_entry_snapshot();
        assert_eq!(snap.velocity_filter_hz, 0);
        assert_eq!(snap.torque_filter_hz, 0);
        assert_eq!(snap.position_filter_hz, 0);
        assert_eq!(snap.flux_filter_hz, 0);

        let mut flat_backend = MockBackend::new();
        crate::commissioning::inner::restore_entry_gains(&snap, &mut flat_backend, tmc_state())
            .await
            .expect("flat restore_entry_gains must succeed against a healthy mock backend");

        crate::state::tmc::tmc_channel0().reset();
        tmc_state().mark_pi_representation_established();

        let mut stepped_backend = MockBackend::new();
        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut stepped_backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());

        assert_eq!(
            write_log(&flat_backend),
            write_log(&stepped_backend),
            "stepped restore must write the identical register sequence as the flat function"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_matches_flat_function_all_filters_enabled() {
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 200;
        snap.torque_filter_hz = 300;
        snap.position_filter_hz = 400;
        snap.flux_filter_hz = 500;

        let mut flat_backend = MockBackend::new();
        crate::commissioning::inner::restore_entry_gains(&snap, &mut flat_backend, tmc_state())
            .await
            .expect("flat restore_entry_gains must succeed with all filters enabled");

        crate::state::tmc::tmc_channel0().reset();
        tmc_state().mark_pi_representation_established();

        let mut stepped_backend = MockBackend::new();
        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut stepped_backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());
        assert_eq!(write_log(&flat_backend), write_log(&stepped_backend));
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_matches_flat_function_filter_hz_below_cutoff_range() {
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 5;

        let mut flat_backend = MockBackend::new();
        crate::commissioning::inner::restore_entry_gains(&snap, &mut flat_backend, tmc_state())
            .await
            .expect("flat restore_entry_gains must succeed with an out-of-range filter cutoff");

        crate::state::tmc::tmc_channel0().reset();
        tmc_state().mark_pi_representation_established();

        let mut stepped_backend = MockBackend::new();
        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut stepped_backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert!(result.is_ok());
        assert_eq!(write_log(&flat_backend), write_log(&stepped_backend));

        use foci_tmc4671::registers::*;
        // `.data.addr()` (0x4D) is shared by every biquad sub-register, so it
        // cannot identify velocity specifically -- torque/position/flux are
        // hz=0 here and legitimately write 0 to that same shared address via
        // their own disable sequences. The ADDR-selector write
        // (addr.addr()=0x4E, value=sub_addr=15) is what uniquely identifies
        // an access to CONFIG_BIQUAD_V_ENABLE.
        assert!(
            !write_log(&stepped_backend).iter().any(|(addr, val)| {
                *addr == CONFIG_BIQUAD_V_ENABLE.addr.addr()
                    && *val == u32::from(CONFIG_BIQUAD_V_ENABLE.sub_addr)
            }),
            "an out-of-range cutoff must write neither the enable nor the disable sequence"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_advances_one_op_per_call() {
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 200;
        snap.torque_filter_hz = 300;
        snap.position_filter_hz = 400;
        snap.flux_filter_hz = 500;
        let mut backend = MockBackend::new();
        let mut step = RestoreEntryGainsStep::start(&snap);
        let mut ticks = 0usize;
        loop {
            let before = backend.spi_log.len();
            let result = step.advance(&mut backend, tmc_state()).await;
            let after = backend.spi_log.len();
            assert!(
                after - before <= 1,
                "tick {}: advance() issued {} ops, expected at most 1",
                ticks,
                after - before
            );
            ticks += 1;
            if !matches!(result, None) {
                break;
            }
        }
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_fail_fast_phase_aborts_on_first_error() {
        use foci_tmc4671::registers::*;
        let snap = sample_entry_snapshot();
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(PID_VELOCITY_LIMIT.addr());

        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(result, Err(RestoreEntryGainsErrorKind::Spi));
        assert!(
            !write_log(&backend)
                .iter()
                .any(|(addr, _)| *addr == PIDOUT_UQ_UD_LIMITS.addr()),
            "a fail-fast abort before the filter phase must never reach the voltage-limit \
             write that follows the filter phase"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_best_effort_filter_failure_does_not_abort() {
        use foci_tmc4671::registers::*;
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 200;
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(CONFIG_BIQUAD_V_A_1.data.addr());

        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(
            result,
            Ok(()),
            "a filter write failure must not abort the restore"
        );
        assert!(
            write_log(&backend)
                .iter()
                .any(|(addr, _)| *addr == PIDOUT_UQ_UD_LIMITS.addr()),
            "the restore must still reach the voltage-limit write after a swallowed filter error"
        );
        assert!(
            write_log(&backend)
                .iter()
                .any(|(addr, val)| *addr == MODE_RAMP_MODE_MOTION.addr() && *val == 0x8000_0003),
            "the restore must still re-enter position mode after a swallowed filter error"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_filter_failure_aborts_only_that_filters_remaining_writes() {
        use foci_tmc4671::registers::*;
        let mut snap = sample_entry_snapshot();
        snap.velocity_filter_hz = 200;
        let mut backend = MockBackend::new();
        // Fail only the first write to the shared biquad data register
        // (velocity's A_1) -- every later write to that same address must
        // succeed, isolating whether the abort is scoped to one filter.
        backend.fail_on_write_occurrence = Some((CONFIG_BIQUAD_V_A_1.data.addr(), 0));

        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(result, Ok(()));

        let log = write_log(&backend);
        for reg in [
            CONFIG_BIQUAD_V_A_2,
            CONFIG_BIQUAD_V_B_0,
            CONFIG_BIQUAD_V_B_1,
            CONFIG_BIQUAD_V_B_2,
            CONFIG_BIQUAD_V_ENABLE,
        ] {
            assert!(
                !log.contains(&(reg.addr.addr(), u32::from(reg.sub_addr))),
                "velocity's A_1 write failed, so the rest of velocity's enable \
                 sequence must be skipped -- found a selector write for sub_addr {}",
                reg.sub_addr
            );
        }
        assert!(
            log.contains(&(
                CONFIG_BIQUAD_X_ENABLE.addr.addr(),
                u32::from(CONFIG_BIQUAD_X_ENABLE.sub_addr)
            )),
            "position's disable write (an unrelated, unaffected filter) must still happen"
        );
        assert!(
            log.iter()
                .any(|(addr, _)| *addr == PIDOUT_UQ_UD_LIMITS.addr()),
            "the restore must still reach the voltage-limit write"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_interlock_tripped_maps_to_fault_interlock_kind() {
        use foci_tmc4671::registers::*;
        let snap = sample_entry_snapshot();
        let mut backend = MockBackend::new();
        backend.interlock_tripped = true;

        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step.advance(&mut backend, tmc_state()).await {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(result, Err(RestoreEntryGainsErrorKind::FaultInterlock));
        assert!(
            !write_log(&backend)
                .iter()
                .any(|(addr, _)| *addr == STATUS_FLAGS.addr()),
            "an interlock-tripped clear-status-flags step must not write STATUS_FLAGS"
        );
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn restore_entry_gains_step_pi_representation_failure_maps_to_pi_representation_kind() {
        let snap = sample_entry_snapshot();
        let mut backend = MockBackend::new();
        // tmc_state() unconditionally establishes PI representation on every
        // call, so it can never exercise this path -- use tmc_channel0()
        // directly instead, after reset() clears the flag.
        crate::state::tmc::tmc_channel0().reset();

        let mut step = RestoreEntryGainsStep::start(&snap);
        let result = loop {
            match step
                .advance(&mut backend, crate::state::tmc::tmc_channel0())
                .await
            {
                None => continue,
                Some(result) => break result,
            }
        };
        assert_eq!(result, Err(RestoreEntryGainsErrorKind::PiRepresentation));
        crate::state::tmc::tmc_channel0().reset();
    }

    #[fixture_test]
    async fn entry_gain_filters_step_skips_velocity_above_motion_ceiling() {
        // A velocity cutoff above the 1000 Hz motion ceiling but below the
        // 10 kHz current ceiling must still be skipped on restore -- the
        // per-target ceiling threaded through this loop is the only thing
        // stopping it from reusing the wider torque/flux limit.
        let snapshot = crate::commissioning::OuterGainRollbackSnapshot {
            velocity_filter_hz: 2000,
            ..crate::commission_dispatch::test_support::sample_entry_snapshot()
        };

        let mut backend = crate::commission_dispatch::test_support::MockBackend::new();
        let mut step = EntryGainFiltersStep::start(&snapshot, 3999);
        while !step.advance(&mut backend).await {}

        use foci_tmc4671::registers::CONFIG_BIQUAD_V_ENABLE;
        let has_velocity_enable_one = backend.spi_log.iter().any(|op| {
            matches!(
                op,
                crate::commission_dispatch::test_support::SpiOp::Write(addr, 1)
                    if *addr == CONFIG_BIQUAD_V_ENABLE.data.addr()
            )
        });
        assert!(
            !has_velocity_enable_one,
            "a 2000 Hz velocity cutoff must not be enabled on restore -- it exceeds \
             the motion-filter ceiling even though it is a valid current-filter value"
        );
    }
}
