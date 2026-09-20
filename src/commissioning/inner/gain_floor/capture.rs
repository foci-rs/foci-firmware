//! Per-slot capture primitive for the commissioning fallback gain-floor
//! measurement: two register reads per 60us slot (PID_TORQUE_FLUX_ACTUAL,
//! PID_POSITION_ACTUAL), with actual-current containment and motion-abort
//! safety checks evaluated on every slot. A slot whose reads overrun its
//! budget is dropped from the sample set; the capture fails only when more
//! slots overrun than the coverage bound allows.

use foci_tmc4671::registers::{PID_POSITION_ACTUAL, PID_TORQUE_FLUX_ACTUAL, PidTorqueFluxActual};

use crate::autotune::CommissionError;
use crate::commissioning::backend::CommissioningBackend;

/// Two TMC4671 register reads per slot take about 44 us on the real SPI
/// path (SPI2 already runs at the chip's 8 MHz maximum; the rest is
/// DMA/async dispatch), so the design's 30 us budget tripped the slot
/// deadline on every slot. 60 us leaves about 35% margin.
const SAMPLE_PERIOD_US: u32 = 60;
/// Fixed capture window per torque step; the current-loop bandwidth floor
/// for running the measurement at all is derived from it.
pub(crate) const CAPTURE_DURATION_US: u32 = 5_000;
const MAX_SAMPLES: usize = 200;
const MOTION_ABORT_THRESHOLD: u32 = 32_760;
/// Slots allowed to overrun their budget before the capture fails. A stray
/// interrupt landing inside a slot costs that slot's sample, not the
/// commission; a tenth of the grid overrunning means the dispatch path
/// itself is over budget.
const MAX_OVERRUN_SLOTS: u32 = 8;
const STEP_OVERSHOOT_MAX_PERMILLE: i32 = 1150;

/// One capture slot's timestamp and register readings.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct CaptureSample {
    pub t_us: u32,
    pub torque_actual: i16,
    pub flux_actual: i16,
    pub position: i32,
}

pub(crate) type CaptureBuffer = heapless::Vec<CaptureSample, MAX_SAMPLES>;

/// Run the fixed-grid capture window into `buffer`, de-energizing on every
/// exit path.
///
/// Reads `PID_TORQUE_FLUX_ACTUAL` and `PID_POSITION_ACTUAL` once per 60us
/// slot anchored to the capture start, containing actual current on both
/// axes against `resistance_current_ceiling_count`, failing a torque read
/// that overshoots `i_target` by more than `STEP_OVERSHOOT_MAX_PERMILLE`,
/// and aborting on displacement beyond `MOTION_ABORT_THRESHOLD` counts from
/// the first sample. The two current checks run on the torque read before
/// the position read is issued; all three trip before the slot is judged
/// for overrun. A slot whose reads do not finish inside its budget records no
/// sample, so timestamps stay on the grid; more than `MAX_OVERRUN_SLOTS`
/// such slots fail the capture as insufficient coverage.
///
/// `buffer` is cleared first. It belongs to the phase state rather than to
/// this future so the samples are neither carried across every slot's await
/// nor copied on the way back to the fits.
pub(crate) async fn run_capture<B>(
    backend: &mut B,
    buffer: &mut CaptureBuffer,
    i_target: i16,
    resistance_current_ceiling_count: u16,
) -> Result<(), CommissionError>
where
    B: CommissioningBackend,
{
    buffer.clear();
    // Fixed grid anchored at capture start: slot n's deadline is
    // capture_start + n * 30 us, not "30 us after the previous pair of reads
    // finished" -- that formulation lets the loop drift and derives t_us
    // from a sample count instead of real elapsed time.
    let capture_start = backend.elapsed_us();
    let capture_deadline = capture_start + u64::from(CAPTURE_DURATION_US);
    let mut position_0: Option<i32> = None;
    let mut overrun_slots: u32 = 0;

    let result = loop {
        // Dropped slots still advance the grid, so the slot index counts
        // them alongside the recorded samples.
        let slot_index = buffer.len() as u64 + u64::from(overrun_slots);
        let slot_start = capture_start + slot_index * u64::from(SAMPLE_PERIOD_US);
        if slot_start >= capture_deadline || buffer.len() >= MAX_SAMPLES {
            break Ok(());
        }
        // Wait until this slot's scheduled start -- the same cooperative
        // delay primitive `current_loop.rs`'s
        // `read_axis_directional_evidence_for_gate` already awaits between
        // settled-flux samples, not a busy loop. Without this call, both
        // register reads below would issue immediately after the previous
        // slot's reads complete, off the fixed grid. `saturating_sub`
        // handles arriving at or after `slot_start` (unavoidable for slot 0,
        // whose start equals `capture_start` exactly, and possible for any
        // later slot under scheduling contention): arrival lateness by
        // itself is not a safety violation, only failing to complete this
        // slot's reads within its budget is (checked below).
        let now = backend.elapsed_us();
        let wait_us = slot_start.saturating_sub(now).min(u64::from(u32::MAX)) as u32;
        if wait_us > 0 {
            backend.delay_us(wait_us).await;
        }

        let torque_flux_raw = match backend.read_register(PID_TORQUE_FLUX_ACTUAL).await {
            Ok(v) => v,
            Err(_) => break Err(CommissionError::GainFloorInsufficientCoverage),
        };
        let packed = PidTorqueFluxActual::from(torque_flux_raw);
        let torque_actual = packed.pid_torque_actual();
        let flux_actual = packed.pid_flux_actual();
        if i32::from(torque_actual).unsigned_abs() > u32::from(resistance_current_ceiling_count)
            || i32::from(flux_actual).unsigned_abs() > u32::from(resistance_current_ceiling_count)
        {
            break Err(CommissionError::GainFloorPeakCurrentExceeded);
        }
        if let Err(e) = check_step_overshoot(
            i32::from(torque_actual) * i32::from(i_target.signum()),
            i32::from(i_target.abs()),
        ) {
            break Err(e);
        }

        let position_raw = match backend.read_register(PID_POSITION_ACTUAL).await {
            Ok(v) => v,
            Err(_) => break Err(CommissionError::GainFloorInsufficientCoverage),
        };

        let slot_deadline = slot_start + u64::from(SAMPLE_PERIOD_US);
        let overran = backend.elapsed_us() > slot_deadline;

        let position = position_raw as i32;
        let position_0 = *position_0.get_or_insert(position);
        let displacement = position.wrapping_sub(position_0).unsigned_abs();
        if displacement > MOTION_ABORT_THRESHOLD {
            break Err(CommissionError::GainFloorMotionEnvelopeExceeded);
        }
        if overran {
            overrun_slots += 1;
            if overrun_slots > MAX_OVERRUN_SLOTS {
                break Err(CommissionError::GainFloorInsufficientCoverage);
            }
            continue;
        }

        // Timestamp from the slot's scheduled time, not from the sample
        // count, which falls behind the grid by every dropped slot.
        let t_us = (slot_start - capture_start) as u32;
        let _ = buffer.push(CaptureSample {
            t_us,
            torque_actual,
            flux_actual,
            position,
        });
    };

    backend.set_enable_pin(false);

    result
}

fn check_step_overshoot(
    peak_along_command: i32,
    target_magnitude: i32,
) -> Result<(), CommissionError> {
    if peak_along_command * 1000 > target_magnitude * STEP_OVERSHOOT_MAX_PERMILLE {
        defmt::warn!(
            "gain-floor step overshoot: torque_actual={} i_target={}",
            peak_along_command,
            target_magnitude
        );
        return Err(CommissionError::GainFloorStepOvershoot);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::commission_dispatch::test_support::MockBackend;

    #[test]
    fn step_overshoot_gate_separates_the_ringing_ladder_runs_from_the_usable_ones() {
        assert_eq!(
            check_step_overshoot(1562, 1280),
            Err(CommissionError::GainFloorStepOvershoot)
        );
        assert_eq!(
            check_step_overshoot(864, 640),
            Err(CommissionError::GainFloorStepOvershoot)
        );
        assert_eq!(check_step_overshoot(1223, 1280), Ok(()));
        assert_eq!(check_step_overshoot(1310, 1280), Ok(()));
    }

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
        let mut buffer = CaptureBuffer::new();
        pollster::block_on(run_capture(&mut backend, &mut buffer, 2000, 4000))
            .expect("capture should complete cleanly");
        // CAPTURE_DURATION_US=5000 / SAMPLE_PERIOD_US=60: slots 0..83 (83*60=4980
        // < 5000; 84*60=5040 does not), so 84 samples.
        assert_eq!(buffer.len(), 84);
    }

    #[test]
    fn capture_future_does_not_carry_the_sample_buffer() {
        let mut backend = MockBackend::new();
        let mut buffer = CaptureBuffer::new();
        let future = run_capture(&mut backend, &mut buffer, 2000, 4000);
        assert!(
            core::mem::size_of_val(&future) < core::mem::size_of::<CaptureBuffer>(),
            "capture future is {} bytes; the buffer must live in the phase state, not the future",
            core::mem::size_of_val(&future)
        );
    }

    #[test]
    fn reused_buffer_is_cleared_before_the_capture_starts() {
        let mut backend = MockBackend::new();
        let mut buffer = CaptureBuffer::new();
        buffer
            .push(CaptureSample {
                t_us: 999,
                torque_actual: 0,
                flux_actual: 0,
                position: 0,
            })
            .unwrap();
        pollster::block_on(run_capture(&mut backend, &mut buffer, 2000, 4000))
            .expect("capture should complete cleanly");
        assert_eq!(buffer.len(), 84);
        assert_eq!(buffer[0].t_us, 0);
    }

    #[test]
    fn actual_current_containment_trips_on_torque_axis_overcurrent() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(0, 200));
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            100,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorPeakCurrentExceeded));
        assert!(!backend.enable_pin, "must de-energize on trip");
    }

    #[test]
    fn actual_current_containment_trips_on_flux_axis_overcurrent() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(200, 0));
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            100,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorPeakCurrentExceeded));
    }

    #[test]
    fn motion_abort_trips_on_excessive_displacement() {
        let mut backend = MockBackend::new();
        backend.read_position_actual_sequence.push(0).unwrap();
        backend.read_position_actual_sequence.push(40_000).unwrap();
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            4000,
        ));
        assert_eq!(
            result,
            Err(CommissionError::GainFloorMotionEnvelopeExceeded)
        );
    }

    #[test]
    fn de_energizes_on_every_exit_path() {
        let mut backend = MockBackend::new();
        let _ = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            4000,
        ));
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
        let mut buffer = CaptureBuffer::new();
        pollster::block_on(run_capture(&mut backend, &mut buffer, 2000, 4000))
            .expect("valid capture");
        for (i, sample) in buffer.iter().enumerate() {
            assert_eq!(sample.t_us, i as u32 * 60);
        }
        // `MockBackend::delay_us_log` caps at 32 entries; the capture window
        // issues one delay per slot after the first (83 here), so the log
        // fills to capacity rather than recording every delay. What matters
        // is that delay_us was genuinely called, every logged wait is
        // exactly one slot period, and the log is not empty -- an
        // implementation that fabricated t_us without awaiting would leave
        // it empty.
        assert!(!backend.delay_us_log.is_empty());
        assert_eq!(backend.delay_us_log.len(), backend.delay_us_log.capacity());
        assert!(backend.delay_us_log.iter().all(|&delay| delay == 60));
    }

    #[test]
    fn late_arrival_that_still_completes_within_budget_does_not_trip() {
        let mut backend = MockBackend::new();
        // First `elapsed_us()` call establishes capture_start = 0; the
        // second call (slot 0's `now`, used to compute the wait) reports the
        // clock already past slot 0's scheduled start -- unavoidable
        // measurement/scheduling overhead, not a safety violation by itself.
        // The third call (post-read) shows both reads still completed well
        // within the 30us slot budget. Arrival lateness alone must not
        // trip: only failing to complete the reads in time does (see the
        // post-read deadline check below).
        backend.set_elapsed_us_sequence(&[0, 25, 28]);
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            4000,
        ));
        assert!(
            result.is_ok(),
            "late arrival alone must not trip when reads still complete in time: {result:?}"
        );
    }

    #[test]
    fn slot_overrun_drops_that_sample_and_keeps_the_grid() {
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        // capture_start = 0, slot 0 arrives on time, but its post-read check
        // reports 70us: the two reads did not finish inside slot 0's 60us
        // budget. That slot's sample is dropped and the capture continues on
        // the fixed grid, so slot 1 still lands at t_us = 60.
        backend.set_elapsed_us_sequence(&[0, 0, 70]);
        let mut buffer = CaptureBuffer::new();
        pollster::block_on(run_capture(&mut backend, &mut buffer, 2000, 4000))
            .expect("one overrun slot must not fail the capture");
        assert_eq!(buffer.len(), 83);
        assert_eq!(buffer[0].t_us, 60);
        assert!(!backend.enable_pin);
    }

    #[test]
    fn overruns_beyond_the_coverage_bound_fail_the_capture() {
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        // Every register read costs 35us, so each slot's pair of reads takes
        // 70us against a 60us budget: every slot overruns. This is a
        // systematic dispatch overrun, not a stray interrupt, and must still
        // fail as insufficient coverage.
        backend.spi_elapsed_step_us = 35;
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            2000,
            4000,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorInsufficientCoverage));
        assert!(!backend.enable_pin, "must de-energize on trip");
    }

    #[test]
    fn overshoot_trips_on_the_slot_that_peaks() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(0, 1562));
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            1280,
            4000,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorStepOvershoot));
        assert!(!backend.enable_pin, "must de-energize on trip");
    }

    #[test]
    fn overshoot_in_a_dropped_overrun_slot_still_trips() {
        let mut backend = MockBackend::new();
        backend.advance_elapsed_on_delay = true;
        backend.set_elapsed_us_sequence(&[0, 0, 70]);
        backend
            .pid_torque_flux_actual_sequence
            .push(pack_torque_flux(0, 1562))
            .unwrap();
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            1280,
            4000,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorStepOvershoot));
    }

    #[test]
    fn overshooting_torque_read_trips_before_the_position_read_can_fail() {
        let mut backend = MockBackend::new();
        let _ = backend
            .registers
            .insert(PID_TORQUE_FLUX_ACTUAL.addr(), pack_torque_flux(0, 1562));
        backend.fail_on_read = Some(PID_POSITION_ACTUAL.addr());
        let result = pollster::block_on(run_capture(
            &mut backend,
            &mut CaptureBuffer::new(),
            1280,
            4000,
        ));
        assert_eq!(result, Err(CommissionError::GainFloorStepOvershoot));
    }
}
