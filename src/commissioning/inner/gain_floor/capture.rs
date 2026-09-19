//! Per-slot capture primitive for the commissioning fallback gain-floor
//! measurement: two register reads per 30us slot (PID_TORQUE_FLUX_ACTUAL,
//! PID_POSITION_ACTUAL), with actual-current containment and motion-abort
//! safety checks evaluated on every slot, and an immediate
//! de-energize-and-fail on any missed slot deadline.

use foci_tmc4671::registers::{PID_POSITION_ACTUAL, PID_TORQUE_FLUX_ACTUAL, PidTorqueFluxActual};

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;

const SAMPLE_PERIOD_US: u32 = 30;
const CAPTURE_DURATION_US: u32 = 5_000;
const MAX_SAMPLES: usize = 200;
const MOTION_ABORT_THRESHOLD: u32 = 32_760;

/// One capture slot's timestamp and register readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CaptureSample {
    pub t_us: u32,
    pub torque_actual: i16,
    pub flux_actual: i16,
    pub position: i32,
}

pub(crate) type CaptureBuffer = heapless::Vec<CaptureSample, MAX_SAMPLES>;

/// Run the fixed-grid capture window, de-energizing on every exit path.
///
/// Reads `PID_TORQUE_FLUX_ACTUAL` and `PID_POSITION_ACTUAL` once per 30us
/// slot anchored to the capture start, containing actual current on both
/// axes against `resistance_current_ceiling_count` and aborting on
/// displacement beyond `MOTION_ABORT_THRESHOLD` counts from the first
/// sample. A slot whose scheduled start has already passed -- before or
/// after its wait -- trips immediately rather than recording a late sample
/// or continuing to the duration bound.
pub(crate) async fn run_capture<B>(
    backend: &mut B,
    _i_target: i16,
    resistance_current_ceiling_count: u16,
) -> Result<CaptureBuffer, CommissionError>
where
    B: CommissioningBackend,
{
    let mut buffer = CaptureBuffer::new();
    // Fixed grid anchored at capture start: slot n's deadline is
    // capture_start + n * 30 us, not "30 us after the previous pair of reads
    // finished" -- that formulation lets the loop drift and derives t_us
    // from a sample count instead of real elapsed time.
    let capture_start = backend.elapsed_us();
    let capture_deadline = capture_start + u64::from(CAPTURE_DURATION_US);
    let mut position_0: Option<i32> = None;

    let result = loop {
        let slot_index = buffer.len() as u64;
        let slot_start = capture_start + slot_index * u64::from(SAMPLE_PERIOD_US);
        if slot_start >= capture_deadline || buffer.len() >= MAX_SAMPLES {
            break Ok(());
        }
        let now = backend.elapsed_us();
        if now > slot_start {
            // The scheduler is already late arriving at this slot's start --
            // an immediate trip, not a value recorded with a stale timestamp.
            break Err(CommissionError::GainFloorInsufficientCoverage);
        }
        // Actually wait until this slot's scheduled start -- the same
        // cooperative delay primitive `current_loop.rs`'s
        // `read_axis_directional_evidence_for_gate` already awaits between
        // settled-flux samples, not a busy loop. Without this call, both
        // register reads below would issue immediately after the previous
        // slot's reads complete, off the fixed grid.
        let wait_us = (slot_start - now).min(u64::from(u32::MAX)) as u32;
        if wait_us > 0 {
            backend.delay_us(wait_us).await;
        }
        // Re-check after the delay: RTIC can resume this task later than
        // requested (preemption, a longer-running higher-priority task).
        // Without re-reading elapsed_us() here, a late wakeup could still
        // issue both reads and have them finish inside slot_start + 30us,
        // passing the post-read deadline check below despite having missed
        // the slot's actual scheduled start.
        if backend.elapsed_us() > slot_start {
            break Err(CommissionError::GainFloorInsufficientCoverage);
        }

        let torque_flux_raw = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
            Ok(v) => v,
            Err(_) => break Err(CommissionError::GainFloorInsufficientCoverage),
        };
        let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
            Ok(v) => v,
            Err(_) => break Err(CommissionError::GainFloorInsufficientCoverage),
        };

        let slot_deadline = slot_start + u64::from(SAMPLE_PERIOD_US);
        if backend.elapsed_us() > slot_deadline {
            // Immediate trip: this slot's two reads did not both complete
            // within their scheduled 30us window while energized -- a safety
            // event, not a deferred data-quality one.
            break Err(CommissionError::GainFloorInsufficientCoverage);
        }

        let packed = PidTorqueFluxActual::from(torque_flux_raw);
        let torque_actual = packed.pid_torque_actual();
        let flux_actual = packed.pid_flux_actual();
        if i32::from(torque_actual).unsigned_abs() > u32::from(resistance_current_ceiling_count)
            || i32::from(flux_actual).unsigned_abs() > u32::from(resistance_current_ceiling_count)
        {
            break Err(CommissionError::GainFloorPeakCurrentExceeded);
        }

        let position = position_raw as i32;
        let position_0 = *position_0.get_or_insert(position);
        let displacement = position.wrapping_sub(position_0).unsigned_abs();
        if displacement > MOTION_ABORT_THRESHOLD {
            break Err(CommissionError::GainFloorMotionEnvelopeExceeded);
        }

        // Timestamp from the slot's scheduled time, not a re-derived sample
        // count -- the two are equal only when every slot has been on-grid,
        // which the immediate-trip checks above already guarantee for any
        // sample that reaches this point.
        let t_us = (slot_start - capture_start) as u32;
        let _ = buffer.push(CaptureSample {
            t_us,
            torque_actual,
            flux_actual,
            position,
        });
    };

    backend.set_enable_pin(false);

    result.map(|()| buffer)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::MockBackend;

    fn pack_torque_flux(flux: i16, torque: i16) -> u32 {
        PidTorqueFluxActual::new()
            .with_pid_flux_actual(flux)
            .with_pid_torque_actual(torque)
            .into()
    }

    #[test]
    fn capture_completes_and_produces_expected_slot_count() {
        let mut backend = MockBackend::new();
        // Defaults (zero torque/flux/position) satisfy every safety bound
        // for the whole window, so the happy path needs no per-slot
        // scripting.
        let result = pollster::block_on(run_capture(&mut backend, 2000, 4000));
        let buffer = result.expect("capture should complete cleanly");
        assert_eq!(buffer.len(), 167);
    }

    #[test]
    fn actual_current_containment_trips_on_torque_axis_overcurrent() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(0, 200));
        let result = pollster::block_on(run_capture(&mut backend, 2000, 100));
        assert_eq!(result, Err(CommissionError::GainFloorPeakCurrentExceeded));
        assert!(!backend.enable_pin, "must de-energize on trip");
    }

    #[test]
    fn actual_current_containment_trips_on_flux_axis_overcurrent() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(200, 0));
        let result = pollster::block_on(run_capture(&mut backend, 2000, 100));
        assert_eq!(result, Err(CommissionError::GainFloorPeakCurrentExceeded));
    }

    #[test]
    fn motion_abort_trips_on_excessive_displacement() {
        let mut backend = MockBackend::new();
        backend.read_position_actual_sequence.push(0).unwrap();
        backend.read_position_actual_sequence.push(40_000).unwrap();
        let result = pollster::block_on(run_capture(&mut backend, 2000, 4000));
        assert_eq!(
            result,
            Err(CommissionError::GainFloorMotionEnvelopeExceeded)
        );
    }

    #[test]
    fn de_energizes_on_every_exit_path() {
        let mut backend = MockBackend::new();
        let _ = pollster::block_on(run_capture(&mut backend, 2000, 4000));
        assert!(!backend.enable_pin);
    }

    #[test]
    fn samples_are_timestamped_from_the_fixed_grid_not_a_sample_counter() {
        let mut backend = MockBackend::new();
        // `elapsed_us()` only advances via `delay_us` (matching real
        // hardware -- time does not pass just because a register read was
        // issued); `spi_elapsed_step_us` stays at its default of 0, so each
        // register read reports zero additional elapsed time on its own.
        // This makes the test fail if the implementation fabricates t_us
        // from a sample counter without actually calling delay_us: with no
        // delay awaited, elapsed_us() would never reach each slot's real
        // deadline, and the immediate-trip re-checks would falsely accept
        // every slot while `delay_us_log` stayed empty.
        backend.advance_elapsed_on_delay = true;
        let result =
            pollster::block_on(run_capture(&mut backend, 2000, 4000)).expect("valid capture");
        for (i, sample) in result.iter().enumerate() {
            assert_eq!(sample.t_us, i as u32 * 30);
        }
        // `MockBackend::delay_us_log` caps at 32 entries; the capture window
        // issues one delay per slot after the first (166 here), so the log
        // fills to capacity rather than recording every delay. What matters
        // is that delay_us was genuinely called, every logged wait is
        // exactly one slot period, and the log is not empty -- an
        // implementation that fabricated t_us without awaiting would leave
        // it empty.
        assert!(!backend.delay_us_log.is_empty());
        assert_eq!(backend.delay_us_log.len(), backend.delay_us_log.capacity());
        assert!(backend.delay_us_log.iter().all(|&delay| delay == 30));
    }

    #[test]
    fn late_slot_start_trips_immediately_instead_of_finishing_the_capture() {
        let mut backend = MockBackend::new();
        // First `elapsed_us()` call establishes capture_start = 0; the
        // second call (the slot-0 `now` read) reports the clock already
        // past slot 0's scheduled start, simulating RTIC scheduling
        // contention that delayed entry into the capture loop.
        backend.set_elapsed_us_sequence(&[0, 5]);
        let result = pollster::block_on(run_capture(&mut backend, 2000, 4000));
        assert_eq!(result, Err(CommissionError::GainFloorInsufficientCoverage));
    }
}
