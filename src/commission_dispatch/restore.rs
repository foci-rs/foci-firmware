//! Commissioning restore sequences used by terminal dispatch outcomes.

use crate::commissioning::types::CommissioningBackend;
use crate::state::tmc::TmcChannelState;
use foci_tmc4671::pid::{PidIGain, PidPGain};

/// Error kind for `PositionModeRestoreStep`, for the same reason
/// `GainsRestoreErrorKind` erases `Spi(B::Error)` -- keeps `RestoreCursor:
/// Copy` independent of the backend's error type.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum PositionModeRestoreErrorKind {
    PiRepresentation,
    InvalidGains,
    Spi,
}

/// The OuterDone success-path register-restore sequence, advanced by exactly
/// one backend register operation per `advance()` call. Writes tuned velocity
/// and position gains, the operational velocity limit, current limit from
/// channel state, the safety-net `PIDOUT_UQ_UD_LIMITS` cap (1024 counts),
/// clears the position-actual integrator, snaps the position target to the
/// current encoder count scaled by the encoder resolution, and enters
/// position mode via `MODE_RAMP_MODE_MOTION = 0x8000_0003`. Every variant
/// carries the already-validated `gains` rather than individually-derived
/// register values, since re-deriving `gains.to_registers()` is pure/non-IO
/// and cheaper than threading extra fields through every variant.
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

/// Error kind for `CalibrateGainsRestoreStep`, with the generic
/// `Spi(B::Error)` payload erased -- every call site already discards it
/// (matches with `_`), so carrying it would force `CalibrateGainsRestoreStep`
/// (and transitively `RestoreCursor`) to become generic over the backend's
/// error type, which would break `RestoreCursor: Copy` across different `B`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum GainsRestoreErrorKind {
    PiRepresentation,
    Spi,
    InvalidGains,
}

/// The CalibrateDone gain-restore sequence, advanced by exactly one backend
/// register operation per `advance()` call. Reapplies host-stored inner gains
/// (if any) to `PID_FLUX_P_FLUX_I` and `PID_TORQUE_P_TORQUE_I`, then reads
/// outer gain values from the [`crate::state::tmc`] atomics and writes
/// velocity gains, position gains, velocity limit, and current limit back to
/// the TMC4671. SPI failures abort immediately; reporting success with wrong
/// gains active is worse than failing.
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

/// Error kind for `EntrySnapshotRestoreStep` (it never validates gains, so no
/// `InvalidGains`, and never verifies PI representation, so no
/// `PiRepresentation`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum EntrySnapshotRestoreErrorKind {
    Spi,
    Readback,
}

/// The non-deploying candidate/resume-terminal restore sequence, advanced by
/// exactly one backend register operation per `advance()` call: writes back
/// only the Stage 2 entry snapshot's own registers -- outer velocity and
/// position gains, the velocity limit, the voltage limit
/// (`PIDOUT_UQ_UD_LIMITS`), and the four biquad filters (`FilterCutoffsStep`)
/// -- then reads the gain and limit registers back to confirm the writes
/// landed. Does **not** touch `PID_TORQUE_FLUX_LIMITS`: the operational
/// current limit lives in `tmc_state`, not the snapshot, and is restored
/// separately by the velocity-disarm step's `ClampRestore` phase, which must
/// run after this one. 4 direct writes, then the filter-cutoffs
/// sub-sequence, then 4 readback reads.
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
        // 2 PI-representation ops + 9 direct register ops == 11.
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
        // Corrupt the velocity-limit readback so the snapshot's written value
        // and the value read back disagree -- exercises
        // EntrySnapshotRestoreErrorKind::Readback.
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
}
