use foci_tmc4671::registers::{
    MODE_RAMP_MODE_MOTION, MotionMode, PIDOUT_UQ_UD_LIMITS, PWM_SV_CHOP, UQ_UD_EXT,
};

use super::{
    AxisProgress, ResistancePoll, ResistanceRunner, ResistanceState, WindowPlan, WindowPurpose,
    build_signed_anchor, next_point_predicted_unsafe, update_safe_anchor,
};
use crate::resistance_identification::fit::{evaluate_resistance_result, fit_resistance_axis};
use crate::resistance_identification::runner::{ResistanceBackend, ResistanceRunMode};
use crate::resistance_identification::types::{ResistanceError, ResistancePoint};

impl ResistanceRunner {
    pub(super) fn finish_window<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        plan: WindowPlan,
        mean: i32,
    ) -> ResistancePoll {
        match plan.purpose {
            WindowPurpose::InitialZero => {
                self.axis.leading_zero = mean;
                self.begin_next_point(backend)
            }
            WindowPurpose::PointEnergized(index) => {
                self.current.record_completed_energized_mean(mean);
                self.axis.energized_current = mean;
                self.axis.last_safe_point = Some((plan.ud.unsigned_abs(), mean.unsigned_abs()));
                self.begin_window(
                    WindowPurpose::PointTrailing(index),
                    0,
                    self.profile.zero_settle_ms,
                );
                ResistancePoll::ReadyNow
            }
            WindowPurpose::PointTrailing(index) => {
                let zero = (self.axis.leading_zero + mean) / 2;
                let current = self.axis.energized_current - zero;
                let ud = self.profile.ud_points[index as usize];
                if ud == self.profile.signed_anchor_ud {
                    self.axis.first_anchor_current = Some(current);
                }
                update_safe_anchor(&self.profile, &mut self.axis.safe_anchor, ud, current);
                self.axis.points[index as usize] = ResistancePoint {
                    requested_ud: ud,
                    voltage_delta_count: i32::from(ud),
                    current_delta_count: current,
                    selected: false,
                };
                self.axis.leading_zero = mean;
                self.axis.point_index = index + 1;
                self.begin_next_point(backend)
            }
            WindowPurpose::RepeatedAnchor => {
                self.current.record_completed_energized_mean(mean);
                self.axis.energized_current = mean;
                self.begin_window(WindowPurpose::RepeatedZero, 0, self.profile.zero_settle_ms);
                ResistancePoll::ReadyNow
            }
            WindowPurpose::RepeatedZero => {
                self.axis.repeated_positive =
                    self.axis.energized_current - (self.axis.leading_zero + mean) / 2;
                self.axis.leading_zero = mean;
                let anchor = self
                    .axis
                    .safe_anchor
                    .expect("safe anchor checked before window");
                self.begin_window(
                    WindowPurpose::NegativeAnchor,
                    -anchor.ud,
                    self.profile.settle_ms,
                );
                ResistancePoll::ReadyNow
            }
            WindowPurpose::NegativeAnchor => {
                self.current.record_completed_energized_mean(mean);
                self.axis.energized_current = mean;
                self.begin_window(WindowPurpose::NegativeZero, 0, self.profile.zero_settle_ms);
                ResistancePoll::ReadyNow
            }
            WindowPurpose::NegativeZero => {
                self.axis.negative_current =
                    self.axis.energized_current - (self.axis.leading_zero + mean) / 2;
                self.state = ResistanceState::EvaluateAxis;
                ResistancePoll::ReadyNow
            }
        }
    }

    pub(super) fn begin_next_point<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        if (self.axis.point_index as usize) < self.profile.ud_points.len() {
            let ud = self.profile.ud_points[self.axis.point_index as usize];
            if !next_point_predicted_unsafe(&self.profile, self.axis.last_safe_point, ud) {
                self.begin_window(
                    WindowPurpose::PointEnergized(self.axis.point_index),
                    ud,
                    self.profile.settle_ms,
                );
                return ResistancePoll::ReadyNow;
            }
            let axis = self.profile.axes[self.axis_index as usize];
            if fit_resistance_axis(axis, &self.profile, &self.axis.points, None).is_err() {
                return self.fail(backend, ResistanceError::MeasurementEnvelopeUnsupported);
            }
        }
        let Some(anchor) = self.axis.safe_anchor else {
            return self.fail(backend, ResistanceError::MeasurementEnvelopeUnsupported);
        };
        self.begin_window(
            WindowPurpose::RepeatedAnchor,
            anchor.ud,
            self.profile.settle_ms,
        );
        ResistancePoll::ReadyNow
    }

    pub(super) fn evaluate_axis<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
    ) -> ResistancePoll {
        let anchor = match self.axis.safe_anchor {
            Some(anchor) => anchor,
            None => return self.fail(backend, ResistanceError::MeasurementEnvelopeUnsupported),
        };
        let signed = build_signed_anchor(
            self.axis
                .first_anchor_current
                .unwrap_or(anchor.positive_current_count),
            self.axis.negative_current,
            self.axis.repeated_positive,
            anchor.ud,
        );
        let axis = self.profile.axes[self.axis_index as usize];
        let result = match fit_resistance_axis(axis, &self.profile, &self.axis.points, Some(signed))
        {
            Ok(result) => result,
            Err(error) => return self.fail(backend, error),
        };
        self.axis_results[self.axis_index as usize] = Some(result);
        if self.axis_index == 0 {
            self.axis_index = 1;
            self.axis = AxisProgress::new();
            self.state = ResistanceState::SelectAxis(0);
        } else {
            self.state = ResistanceState::EvaluateRun;
        }
        ResistancePoll::ReadyNow
    }

    pub(super) fn evaluate_run<B: ResistanceBackend>(&mut self, backend: &mut B) -> ResistancePoll {
        let result = evaluate_resistance_result(
            &self.profile,
            self.axis_results[0].expect("axis zero evaluated"),
            self.axis_results[1].expect("axis one evaluated"),
            self.status_flags_or,
        );
        match result {
            Ok(mut result) => {
                result.warning_flags.insert(self.warning_flags);
                result.current = self.current;
                self.primary_error = None;
                self.result = Some(result);
                self.start_restore(backend);
                ResistancePoll::ReadyNow
            }
            Err(error) => self.fail(backend, error),
        }
    }

    pub(super) fn fail<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        error: ResistanceError,
    ) -> ResistancePoll {
        if self.energized {
            if self.primary_error.is_none() {
                self.primary_error = Some(error);
            }
            self.start_restore(backend);
            ResistancePoll::ReadyNow
        } else {
            self.complete_without_restore(error)
        }
    }

    pub(super) fn start_restore<B: ResistanceBackend>(&mut self, backend: &mut B) {
        if self.mode == ResistanceRunMode::CommissioningManagedEnable {
            backend.set_enable_pin(false);
        }
        self.energized = false;
        self.state = ResistanceState::Restore(0);
    }

    pub(super) async fn poll_restore<B: ResistanceBackend>(
        &mut self,
        backend: &mut B,
        operation: u8,
    ) -> ResistancePoll {
        let (addr, value) = match operation {
            0 => (MODE_RAMP_MODE_MOTION.addr(), MotionMode::Stopped as u32),
            1 => (UQ_UD_EXT.addr(), 0),
            2 => (PWM_SV_CHOP.addr(), 0),
            _ => (PIDOUT_UQ_UD_LIMITS.addr(), self.saved_voltage_limit),
        };
        if backend.write_register(addr, value).await.is_err() {
            self.primary_error = Some(ResistanceError::BackendIo);
        }
        if operation < 3 {
            self.state = ResistanceState::Restore(operation + 1);
            return ResistancePoll::ReadyNow;
        }
        self.state = ResistanceState::Complete;
        if let Some(error) = self.primary_error {
            ResistancePoll::Complete(Err(error))
        } else {
            ResistancePoll::Complete(Ok(self.result.expect("successful evaluated result")))
        }
    }

    pub(super) fn complete_without_restore(&mut self, error: ResistanceError) -> ResistancePoll {
        self.primary_error = Some(error);
        self.state = ResistanceState::Complete;
        ResistancePoll::Complete(Err(error))
    }
}
