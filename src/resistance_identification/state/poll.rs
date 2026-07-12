use foci_tmc4671::registers::{
    ABN_DECODER_COUNT, ADC_IWY_IUX, DSADC_MDEC_B_MDEC_A, MODE_RAMP_MODE_MOTION,
    MOTOR_TYPE_N_POLE_PAIRS, MotionMode, MotorType, MotorTypeNPolePairs, PHI_E_EXT,
    PHI_E_SELECTION, PIDOUT_UQ_UD_LIMITS, PWM_BBM_H_BBM_L, PWM_MAXCNT, PWM_SV_CHOP, STATUS_FLAGS,
    UQ_UD_EXT,
};

use super::{
    MAX_OPPORTUNITIES_PER_WINDOW, ResistancePoll, ResistanceRunner, ResistanceState,
    SAMPLES_PER_WINDOW, WindowCapture, WindowPlan, WindowPurpose, pack_ud, phase_current_for_axis,
    wrapped_encoder_delta, write,
};
use crate::autotune::current_loop_actionable_status_mask;
use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope};
use crate::resistance_identification::runner::{ResistanceBackend, ResistanceRunMode};
use crate::resistance_identification::types::{ResistanceError, ResistanceWarningFlags};

impl ResistanceRunner {
    pub(super) async fn read_saved_limit<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        if !backend.adc_offsets_ready() {
            return self.complete_without_restore(ResistanceError::NotAdcReady);
        }
        if self.mode == ResistanceRunMode::ExternallyArmed
            && !backend.is_armed_for_open_loop_voltage()
        {
            return self.complete_without_restore(ResistanceError::NotArmed);
        }
        match backend.read_register(PIDOUT_UQ_UD_LIMITS.addr()).await {
            Ok(value) => {
                self.saved_voltage_limit = value;
                if self.mode == ResistanceRunMode::ExternallyArmed
                    && value < u32::from(self.profile.max_abs_ud().unsigned_abs())
                {
                    return self.complete_without_restore(ResistanceError::OutOfBoundsParameter);
                }
                self.state = ResistanceState::ReadMotorType;
                ResistancePoll::ReadyNow
            }
            Err(_) => self.complete_without_restore(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn read_motor_type<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        match backend.read_register(MOTOR_TYPE_N_POLE_PAIRS.addr()).await {
            Ok(raw) if MotorTypeNPolePairs::from(raw).motor_type() == MotorType::TwoPhase as u8 => {
                self.state = ResistanceState::Setup(0);
                ResistancePoll::ReadyNow
            }
            Ok(_) => self.complete_without_restore(ResistanceError::InvalidMotorType),
            Err(_) => self.complete_without_restore(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn poll_setup<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let result = match operation {
            0 => {
                write(
                    backend,
                    MODE_RAMP_MODE_MOTION.addr(),
                    MotionMode::Stopped as u32,
                )
                .await
            }
            1 => write(backend, UQ_UD_EXT.addr(), 0).await,
            2 => {
                write(
                    backend,
                    PIDOUT_UQ_UD_LIMITS.addr(),
                    u32::from(self.profile.max_abs_ud().unsigned_abs()),
                )
                .await
            }
            3 => write(backend, PWM_SV_CHOP.addr(), self.profile.pwm_sv_chop).await,
            4 => {
                self.verify_read(backend, PWM_SV_CHOP.addr(), self.profile.pwm_sv_chop)
                    .await
            }
            5 => {
                self.verify_read(
                    backend,
                    PWM_MAXCNT.addr(),
                    u32::from(self.profile.pwm_maxcnt),
                )
                .await
            }
            6 => {
                self.verify_read(
                    backend,
                    PWM_BBM_H_BBM_L.addr(),
                    (u32::from(self.profile.bbm_h) << 8) | u32::from(self.profile.bbm_l),
                )
                .await
            }
            7 => {
                self.verify_read(
                    backend,
                    DSADC_MDEC_B_MDEC_A.addr(),
                    (u32::from(self.profile.dsadc_mdec_b) << 16)
                        | u32::from(self.profile.dsadc_mdec_a),
                )
                .await
            }
            8 => match backend.read_register(STATUS_FLAGS.addr()).await {
                Ok(flags) => {
                    self.status_flags_or |= flags;
                    Ok(())
                }
                Err(_) => Err(ResistanceError::BackendIo),
            },
            _ => Ok(()),
        };
        if let Err(error) = result {
            return self.fail(backend, error);
        }
        if operation == 0 {
            self.energized = true;
        }
        if operation < 8 {
            self.state = ResistanceState::Setup(operation + 1);
        } else {
            if self.mode == ResistanceRunMode::CommissioningManagedEnable {
                backend.set_enable_pin(true);
            }
            self.energized = true;
            self.state = ResistanceState::SelectAxis(0);
        }
        ResistancePoll::ReadyNow
    }

    pub(super) async fn verify_read<B: ResistanceBackend>(
        &self,
        backend: &mut B,
        addr: u8,
        expected: u32,
    ) -> Result<(), ResistanceError> {
        match backend.read_register(addr).await {
            Ok(value) if value == expected => Ok(()),
            Ok(_) => Err(ResistanceError::WrongPowerStage),
            Err(_) => Err(ResistanceError::BackendIo),
        }
    }

    pub(super) async fn select_axis<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let axis = self.profile.axes[self.axis_index as usize];
        let result = if operation == 0 {
            write(backend, PHI_E_SELECTION.addr(), 1).await
        } else {
            write(backend, PHI_E_EXT.addr(), u32::from(axis.phi_e_ext)).await
        };
        if let Err(error) = result {
            return self.fail(backend, error);
        }
        if operation == 0 {
            self.state = ResistanceState::SelectAxis(1);
        } else {
            self.begin_window(WindowPurpose::InitialZero, 0, self.profile.zero_settle_ms);
        }
        ResistancePoll::ReadyNow
    }

    pub(super) fn begin_window(&mut self, purpose: WindowPurpose, ud: i16, settle_ms: u16) {
        self.state = ResistanceState::Window(WindowCapture {
            plan: WindowPlan {
                purpose,
                ud,
                settle_ms,
            },
            operation: 0,
            encoder_before: 0,
            current_sum: 0,
            status_or: 0,
            valid_samples: 0,
            opportunities: 0,
            next_sample_us: 0,
            timing: CaptureTiming::new(
                self.profile.sample_interval_ms.saturating_mul(1_000),
                TimingEnvelope::RESISTANCE,
            ),
        });
    }

    pub(super) async fn poll_window<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        mut capture: WindowCapture,
        now_us: u64,
    ) -> ResistancePoll {
        match capture.operation {
            0 => {
                let value = if capture.plan.ud == 0 {
                    0
                } else {
                    pack_ud(capture.plan.ud)
                };
                if let Err(error) = write(backend, UQ_UD_EXT.addr(), value).await {
                    return self.fail(backend, error);
                }
                capture.operation = 1;
            }
            1 => {
                let mode = if capture.plan.ud == 0 {
                    MotionMode::Stopped
                } else {
                    MotionMode::UqUdExt
                };
                if let Err(error) = write(backend, MODE_RAMP_MODE_MOTION.addr(), mode as u32).await
                {
                    return self.fail(backend, error);
                }
                capture.operation = 2;
            }
            2 => match backend.read_register(ABN_DECODER_COUNT.addr()).await {
                Ok(value) => {
                    capture.encoder_before = value;
                    let settle_us = u64::from(capture.plan.settle_ms) * 1_000;
                    let Some(deadline) = now_us.checked_add(settle_us) else {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    };
                    if deadline > self.deadline_us {
                        return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
                    }
                    capture.next_sample_us = deadline;
                    capture.operation = 3;
                    self.state = ResistanceState::Window(capture);
                    return ResistancePoll::At(deadline);
                }
                Err(_) => return self.fail(backend, ResistanceError::BackendIo),
            },
            _ => return self.poll_sample(backend, capture, now_us).await,
        }
        self.state = ResistanceState::Window(capture);
        ResistancePoll::ReadyNow
    }

    pub(super) async fn poll_sample<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        mut capture: WindowCapture,
        now_us: u64,
    ) -> ResistancePoll {
        if now_us < capture.next_sample_us {
            return ResistancePoll::At(capture.next_sample_us);
        }
        let io_start = backend.elapsed_us();
        let packed = match backend.read_register(ADC_IWY_IUX.addr()).await {
            Ok(value) => value,
            Err(_) => return self.fail(backend, ResistanceError::BackendIo),
        };
        let first_done = backend.elapsed_us();
        let flags = match backend.read_register(STATUS_FLAGS.addr()).await {
            Ok(value) => value,
            Err(_) => return self.fail(backend, ResistanceError::BackendIo),
        };
        let io_done = backend.elapsed_us();
        let spi_wall = first_done
            .saturating_sub(io_start)
            .saturating_add(io_done.saturating_sub(first_done));
        let Ok(spi_wall_u32) = u32::try_from(spi_wall) else {
            return self.fail(backend, ResistanceError::TimingRejected);
        };
        let Some(completion_us) = now_us.checked_add(spi_wall) else {
            return self.fail(backend, ResistanceError::TimingRejected);
        };
        if completion_us > self.deadline_us {
            return self.fail(backend, ResistanceError::PhaseDeadlineExceeded);
        }
        let slot = match capture.timing.record_sample(
            capture.next_sample_us,
            now_us,
            completion_us,
            completion_us,
            spi_wall_u32,
        ) {
            Ok(slot) => slot,
            Err(_) => return self.fail(backend, ResistanceError::TimingRejected),
        };
        capture.opportunities = capture
            .opportunities
            .saturating_add(u8::try_from(slot.missed).unwrap_or(u8::MAX))
            .saturating_add(1);
        if capture.opportunities > MAX_OPPORTUNITIES_PER_WINDOW || capture.timing.finish().is_err()
        {
            return self.fail(backend, ResistanceError::TimingRejected);
        }
        let axis = self.profile.axes[self.axis_index as usize];
        capture.current_sum += i32::from(phase_current_for_axis(axis, packed));
        capture.status_or |= flags;
        capture.valid_samples += 1;
        capture.next_sample_us = slot.next_deadline_us;
        self.status_flags_or |= flags;
        if flags & current_loop_actionable_status_mask() != 0 {
            return self.fail(backend, ResistanceError::CriticalStatus);
        }
        if flags & self.profile.status_flags_warn_mask != 0 {
            self.warning_flags
                .insert(ResistanceWarningFlags::STATUS_FLAGS);
        }
        if capture.valid_samples == SAMPLES_PER_WINDOW {
            let evidence = match capture.timing.finish() {
                Ok(evidence) => evidence,
                Err(_) => return self.fail(backend, ResistanceError::TimingRejected),
            };
            self.timing.completed_windows = self.timing.completed_windows.saturating_add(1);
            self.timing.missed_samples = self
                .timing
                .missed_samples
                .saturating_add(evidence.missed_samples);
            self.timing.max_poll_wall_us =
                self.timing.max_poll_wall_us.max(evidence.max_poll_wall_us);
            self.timing.max_spi_wall_us = self.timing.max_spi_wall_us.max(evidence.max_spi_wall_us);
            self.state = ResistanceState::ReadEncoderAfter(capture);
            ResistancePoll::ReadyNow
        } else {
            self.state = ResistanceState::Window(capture);
            ResistancePoll::At(capture.next_sample_us)
        }
    }

    pub(super) async fn read_encoder_after<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        capture: WindowCapture,
    ) -> ResistancePoll {
        let after = match backend.read_register(ABN_DECODER_COUNT.addr()).await {
            Ok(value) => value,
            Err(_) => return self.fail(backend, ResistanceError::BackendIo),
        };
        let delta = wrapped_encoder_delta(
            capture.encoder_before,
            after,
            backend.encoder_counts_per_rev(),
        );
        if capture.plan.ud != 0
            && delta.unsigned_abs() > u32::from(self.profile.encoder_move_warn_counts)
        {
            self.warning_flags
                .insert(ResistanceWarningFlags::ENCODER_MOVED);
        }
        let mean = capture.current_sum / i32::from(SAMPLES_PER_WINDOW);
        self.finish_window(backend, capture.plan, mean)
    }
}
