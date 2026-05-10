//! Per-tick TMC4671 trace capture buffer.
//!
//! Ring buffer stored in CCM RAM, captures register snapshots during
//! commissioning phases. See `docs/specs/2026-04-13-trace-capture-design.md`.

use embedded_hal_async::spi::SpiDevice;
use foci_tmc4671::spi::Tmc4671Spi;
use static_assertions::const_assert;

/// Fast preset: 28 bytes/sample, 1 kHz, 5 reg + 1 sub-reg read.
pub const FAST_CAPACITY: usize = 512;
/// Full preset: 48 bytes/sample, 500 Hz (decimated), 7 reg + 4 sub-reg.
pub const FULL_CAPACITY: usize = 256;

const_assert!(FAST_CAPACITY.is_power_of_two());
const_assert!(FULL_CAPACITY.is_power_of_two());

/// Magic value written by reset(). Defense-in-depth against skipped init.
pub const TRACE_MAGIC: u32 = 0xF0C1_78CE;

/// Trace preset selector.
pub const PRESET_FAST: u8 = 0;
/// Trace preset selector.
pub const PRESET_FULL: u8 = 1;

/// Trace buffer state.
pub const STATE_IDLE: u8 = 0;
/// Trace buffer state.
pub const STATE_CAPTURING: u8 = 1;
/// Trace buffer state.
pub const STATE_DONE: u8 = 2;

/// Sample flags.
pub const FLAG_PREV_DROPPED: u8 = 1 << 0;

/// Fast preset sample: 28 bytes. All multi-byte fields little-endian.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct TraceSampleFast {
    pub engine_tick: u16,
    pub phase_id: u8,
    pub flags: u8,
    pub position_target: i32,
    pub position_actual: i32,
    pub torque_flux_actual: u32,
    pub velocity_actual: i32,
    pub status_flags: u32,
    pub abn_count: i32,
}

const_assert!(core::mem::size_of::<TraceSampleFast>() == 28);

/// Full preset sample: 48 bytes.
#[repr(C)]
#[derive(Clone, Copy, Default)]
pub struct TraceSampleFull {
    pub fast: TraceSampleFast,
    pub torque_flux_target: u32,
    pub velocity_offset: i32,
    pub error_sum_position: i32,
    pub error_sum_velocity: i32,
    pub error_sum_torque: i32,
}

const_assert!(core::mem::size_of::<TraceSampleFull>() == 48);

/// Storage union: sized to the larger variant (`fast[512]` = 14336 bytes).
#[repr(C)]
pub union TraceStorage {
    pub fast: [TraceSampleFast; FAST_CAPACITY],
    pub full: [TraceSampleFull; FULL_CAPACITY],
}

/// Trace ring buffer. All fields are integer types for MaybeUninit safety.
/// Must be initialized via `reset()` before any read.
#[repr(C)]
pub struct TraceBuffer {
    magic: u32,
    generation: u8,
    owner_oid: u8,
    preset: u8,
    state: u8,
    head: u16,
    count: u16,
    engine_tick: u16,
    sample_period_us: u16,
    phase_id: u8,
    dropped_count: u16,
    prev_dropped: u8,
    storage: TraceStorage,
}

impl TraceBuffer {
    /// Initialize to a valid idle state. Must be called before any read.
    pub fn reset(&mut self) {
        self.magic = TRACE_MAGIC;
        self.generation = 0;
        self.owner_oid = 0;
        self.preset = PRESET_FAST;
        self.state = STATE_IDLE;
        self.head = 0;
        self.count = 0;
        self.engine_tick = 0;
        self.sample_period_us = 1000;
        self.phase_id = 0;
        self.dropped_count = 0;
        self.prev_dropped = 0;
    }

    /// Begin a new capture session. Increments generation, resets ring.
    pub fn start(&mut self, oid: u8, preset: u8) {
        self.generation = self.generation.wrapping_add(1);
        self.owner_oid = oid;
        self.preset = preset;
        self.state = STATE_CAPTURING;
        self.head = 0;
        self.count = 0;
        self.engine_tick = 0;
        self.sample_period_us = 1000;
        self.phase_id = 0;
        self.dropped_count = 0;
        self.prev_dropped = 0;
    }

    /// Stop capture. Buffer holds data until next start().
    pub fn stop(&mut self) {
        self.state = STATE_DONE;
    }

    /// Update phase_id for subsequent samples. Does NOT reset the ring.
    pub fn set_phase(&mut self, phase_id: u8) {
        self.phase_id = phase_id;
    }

    /// Check if capture is active.
    pub fn is_capturing(&self) -> bool {
        self.magic == TRACE_MAGIC && self.state == STATE_CAPTURING
    }

    /// Get current preset.
    pub fn preset(&self) -> u8 {
        self.preset
    }

    /// Get current engine tick counter.
    pub fn engine_tick(&self) -> u16 {
        self.engine_tick
    }

    fn capacity(&self) -> u16 {
        if self.preset == PRESET_FULL {
            FULL_CAPACITY as u16
        } else {
            FAST_CAPACITY as u16
        }
    }

    /// Advance engine_tick without storing a sample (decimation or budget drop).
    /// If `dropped` is true, marks the next stored sample with FLAG_PREV_DROPPED
    /// and increments dropped_count.
    pub fn tick_no_sample(&mut self, dropped: bool) {
        self.engine_tick = self.engine_tick.wrapping_add(1);
        if dropped {
            self.prev_dropped = 1;
            self.dropped_count = self.dropped_count.saturating_add(1);
        }
    }

    /// Store a fast-preset sample and advance engine_tick.
    pub fn push_fast(&mut self, mut sample: TraceSampleFast) {
        sample.engine_tick = self.engine_tick;
        sample.phase_id = self.phase_id;
        sample.flags = if self.prev_dropped != 0 {
            FLAG_PREV_DROPPED
        } else {
            0
        };
        self.prev_dropped = 0;

        let cap = FAST_CAPACITY as u16;
        let idx = self.head % cap;
        // SAFETY: idx < FAST_CAPACITY, fast variant is active when preset == PRESET_FAST
        unsafe {
            self.storage.fast[idx as usize] = sample;
        }
        self.head = self.head.wrapping_add(1) % cap;
        if self.count < cap {
            self.count += 1;
        }
        self.engine_tick = self.engine_tick.wrapping_add(1);
    }

    /// Store a full-preset sample and advance engine_tick.
    pub fn push_full(&mut self, mut sample: TraceSampleFull) {
        sample.fast.engine_tick = self.engine_tick;
        sample.fast.phase_id = self.phase_id;
        sample.fast.flags = if self.prev_dropped != 0 {
            FLAG_PREV_DROPPED
        } else {
            0
        };
        self.prev_dropped = 0;

        let cap = FULL_CAPACITY as u16;
        let idx = self.head % cap;
        // SAFETY: idx < FULL_CAPACITY, full variant is active when preset == PRESET_FULL
        unsafe {
            self.storage.full[idx as usize] = sample;
        }
        self.head = self.head.wrapping_add(1) % cap;
        if self.count < cap {
            self.count += 1;
        }
        self.engine_tick = self.engine_tick.wrapping_add(1);
    }

    /// Query buffer metadata. Returns all-zero defaults if invalid.
    pub fn info(&self, requesting_oid: u8) -> TraceInfo {
        if self.magic != TRACE_MAGIC || self.owner_oid != requesting_oid {
            return TraceInfo::default();
        }
        TraceInfo {
            state: self.state,
            generation: self.generation,
            preset: self.preset,
            count: self.count,
            phase_id: self.phase_id,
            sample_period_us: self.sample_period_us,
            dropped: self.dropped_count,
        }
    }

    /// Fetch a sample by linearized offset. Returns None on error.
    /// `expected_generation` must match current generation.
    pub fn fetch(&self, requesting_oid: u8, offset: u16, expected_generation: u8) -> FetchResult {
        // Check order: magic/OID -> generation -> capturing -> offset -> OK
        if self.magic != TRACE_MAGIC || self.owner_oid != requesting_oid {
            return FetchResult::Invalid;
        }
        if self.generation != expected_generation {
            return FetchResult::Invalid;
        }
        if self.state == STATE_CAPTURING {
            return FetchResult::Busy;
        }
        if offset >= self.count {
            return FetchResult::Invalid;
        }
        let cap = self.capacity();
        let ring_idx = self.head.wrapping_sub(self.count).wrapping_add(offset) % cap;
        FetchResult::Ok(ring_idx)
    }

    /// Get raw bytes for a sample at ring index. Caller must ensure
    /// ring_idx is valid and preset matches.
    pub fn sample_bytes_fast(&self, ring_idx: u16) -> &[u8] {
        let sample = unsafe { &self.storage.fast[ring_idx as usize] };
        let ptr = sample as *const TraceSampleFast as *const u8;
        unsafe { core::slice::from_raw_parts(ptr, 28) }
    }

    /// Get raw bytes for a full sample at ring index.
    pub fn sample_bytes_full(&self, ring_idx: u16) -> &[u8] {
        let sample = unsafe { &self.storage.full[ring_idx as usize] };
        let ptr = sample as *const TraceSampleFull as *const u8;
        unsafe { core::slice::from_raw_parts(ptr, 48) }
    }
}

/// Metadata returned by info().
#[derive(Clone, Default)]
pub struct TraceInfo {
    pub state: u8,
    pub generation: u8,
    pub preset: u8,
    pub count: u16,
    pub phase_id: u8,
    pub sample_period_us: u16,
    pub dropped: u16,
}

/// Result of a fetch operation.
pub enum FetchResult {
    /// Sample available at this ring index.
    Ok(u16),
    /// Capture still active.
    Busy,
    /// OID/generation mismatch, offset out of range, or uninitialized.
    Invalid,
}

/// Maximum payload bytes in a single anchor protocol message.
/// anchor: MESSAGE_LENGTH_MAX (64) - MESSAGE_HEADER_SIZE (2) - MESSAGE_TRAILER_SIZE (3) = 59.
pub const MESSAGE_PAYLOAD_MAX: usize = 59;

/// Worst-case encoded size for foci_trace_data response.
/// Fields: reply_id VLQ (1) + oid u8 (1) + offset u16 VLQ (2) + status u8 (1) +
/// data length VLQ (1) + data bytes (48 for full preset) = 54 bytes.
/// Conservative estimate uses 2-byte VLQ for data length: 55 bytes.
pub const TRACE_DATA_RESPONSE_MAX: usize = 1 + 1 + 2 + 1 + 2 + 48;

const_assert!(TRACE_DATA_RESPONSE_MAX <= MESSAGE_PAYLOAD_MAX);

/// SPI time budget for the fast trace preset (microseconds).
///
/// DWT-measured duration on OpenFFBoard hardware is approximately 227 us.
/// The budget includes margin for SPI scheduling jitter.
pub const SPI_BUDGET_FAST_US: u64 = 300;

/// SPI time budget for the full trace preset (microseconds).
///
/// Estimated from the fast-preset ratio. Full preset reads 7 registers and
/// 4 sub-registers versus 5 + 1 for fast.
pub const SPI_BUDGET_FULL_US: u64 = 550;

/// Read trace registers from the TMC4671 and push one sample into `buf`.
///
/// Called once per 1 kHz tick from the TMC control task. Handles decimation
/// (full preset fires at 500 Hz), pre- and post-read deadline checks, and
/// SPI failure by recording a budget drop instead of storing partial data.
///
/// # Parameters
///
/// - `tmc` — mutable reference to the TMC4671 SPI driver.
/// - `buf` — mutable reference to the trace ring buffer (caller owns the unsafe
///   static accessor).
/// - `remaining_budget_us` — closure returning `Some(us)` of time left before
///   `tick_deadline`, or `None` if the deadline has already passed.
/// - `past_deadline` — closure returning `true` if the current time is strictly
///   past `tick_deadline` (used for the post-read overrun check).
/// - `cpu_freq_mhz` — CPU frequency in MHz used to convert DWT cycle counts to
///   microseconds for the first-sample SPI-duration log message.
pub async fn sample_tick<B, F1, F2>(
    tmc: &mut Tmc4671Spi<B>,
    buf: &mut TraceBuffer,
    remaining_budget_us: F1,
    past_deadline: F2,
    cpu_freq_mhz: u32,
) where
    B: SpiDevice<u8>,
    F1: Fn() -> Option<u64>,
    F2: Fn() -> bool,
{
    use foci_tmc4671::registers::*;

    if !buf.is_capturing() {
        return;
    }

    let preset = buf.preset();
    let engine_tick = buf.engine_tick();

    // Full preset decimation: skip odd ticks (500 Hz effective)
    if preset == PRESET_FULL && !engine_tick.is_multiple_of(2) {
        buf.tick_no_sample(false); // intentional decimation, not a drop
        return;
    }

    // Pre-read deadline check
    let budget_us = if preset == PRESET_FULL {
        SPI_BUDGET_FULL_US
    } else {
        SPI_BUDGET_FAST_US
    };
    if remaining_budget_us().is_none_or(|d| d < budget_us) {
        buf.tick_no_sample(true); // budget drop
        return;
    }

    // First sample: measure actual SPI duration via DWT CYCCNT
    let is_first_sample = engine_tick == 0;
    let cyccnt_before = if is_first_sample {
        cortex_m::peripheral::DWT::cycle_count()
    } else {
        0
    };

    // Read fast-preset registers. Any SPI failure discards the whole
    // sample — storing zeros would be indistinguishable from real data.
    macro_rules! read_or_drop {
        ($expr:expr) => {
            match $expr.await {
                Ok(v) => v,
                Err(_) => {
                    buf.tick_no_sample(true);
                    return;
                }
            }
        };
    }

    let position_target = read_or_drop!(tmc.read_register(PID_POSITION_TARGET)) as i32;
    let position_actual = read_or_drop!(tmc.read_register(PID_POSITION_ACTUAL)) as i32;
    let torque_flux_actual = read_or_drop!(tmc.read_register(PID_TORQUE_FLUX_ACTUAL));
    let abn_count = read_or_drop!(tmc.read_register(ABN_DECODER_COUNT)) as i32;
    let status_flags = read_or_drop!(tmc.read_register(STATUS_FLAGS));
    let velocity_actual =
        read_or_drop!(tmc.read_sub_register(INTERIM_PIDOUT_TARGET_VELOCITY)) as i32;

    if preset == PRESET_FULL {
        // Read additional full-preset registers
        let torque_flux_target = read_or_drop!(tmc.read_register(PID_TORQUE_FLUX_TARGET));
        let velocity_offset = read_or_drop!(tmc.read_register(PID_VELOCITY_OFFSET)) as i32;
        let error_sum_position =
            read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_POSITION_ERROR_SUM)) as i32;
        let error_sum_velocity =
            read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_VELOCITY_ERROR_SUM)) as i32;
        let error_sum_torque =
            read_or_drop!(tmc.read_sub_register(PID_ERROR_PID_TORQUE_ERROR_SUM)) as i32;

        // First sample: log measured SPI duration
        if is_first_sample {
            let elapsed_cycles =
                cortex_m::peripheral::DWT::cycle_count().wrapping_sub(cyccnt_before);
            let elapsed_us = elapsed_cycles / cpu_freq_mhz;
            defmt::info!(
                "trace: full SPI read {} us ({} cycles)",
                elapsed_us,
                elapsed_cycles
            );
            if elapsed_us as u64 > SPI_BUDGET_FULL_US {
                defmt::warn!(
                    "trace: full SPI read {} us exceeds budget {} us",
                    elapsed_us,
                    SPI_BUDGET_FULL_US
                );
            }
        }

        // Post-read deadline check
        if past_deadline() {
            buf.tick_no_sample(true);
            defmt::warn!("trace: tick overrun, full sample discarded");
            return;
        }

        buf.push_full(TraceSampleFull {
            fast: TraceSampleFast {
                engine_tick: 0, // set by push
                phase_id: 0,    // set by push
                flags: 0,       // set by push
                position_target,
                position_actual,
                torque_flux_actual,
                velocity_actual,
                status_flags,
                abn_count,
            },
            torque_flux_target,
            velocity_offset,
            error_sum_position,
            error_sum_velocity,
            error_sum_torque,
        });
    } else {
        // First sample: log measured SPI duration
        if is_first_sample {
            let elapsed_cycles =
                cortex_m::peripheral::DWT::cycle_count().wrapping_sub(cyccnt_before);
            let elapsed_us = elapsed_cycles / cpu_freq_mhz;
            defmt::info!(
                "trace: fast SPI read {} us ({} cycles)",
                elapsed_us,
                elapsed_cycles
            );
            if elapsed_us as u64 > SPI_BUDGET_FAST_US {
                defmt::warn!(
                    "trace: fast SPI read {} us exceeds budget {} us",
                    elapsed_us,
                    SPI_BUDGET_FAST_US
                );
            }
        }

        // Post-read deadline check
        if past_deadline() {
            buf.tick_no_sample(true);
            defmt::warn!("trace: tick overrun, fast sample discarded");
            return;
        }

        buf.push_fast(TraceSampleFast {
            engine_tick: 0,
            phase_id: 0,
            flags: 0,
            position_target,
            position_actual,
            torque_flux_actual,
            velocity_actual,
            status_flags,
            abn_count,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_buffer() -> TraceBuffer {
        // SAFETY: all fields are integer types, we immediately reset()
        let mut buf: TraceBuffer = unsafe { core::mem::zeroed() };
        buf.reset();
        buf
    }

    #[test]
    fn sample_sizes() {
        assert_eq!(core::mem::size_of::<TraceSampleFast>(), 28);
        assert_eq!(core::mem::size_of::<TraceSampleFull>(), 48);
    }

    #[test]
    fn reset_sets_magic_and_idle() {
        let buf = make_buffer();
        assert_eq!(buf.magic, TRACE_MAGIC);
        assert_eq!(buf.state, STATE_IDLE);
        assert_eq!(buf.count, 0);
    }

    #[test]
    fn start_increments_generation() {
        let mut buf = make_buffer();
        assert_eq!(buf.generation, 0);
        buf.start(1, PRESET_FAST);
        assert_eq!(buf.generation, 1);
        buf.stop();
        buf.start(1, PRESET_FAST);
        assert_eq!(buf.generation, 2);
    }

    #[test]
    fn generation_wraps() {
        let mut buf = make_buffer();
        buf.generation = 255;
        buf.start(1, PRESET_FAST);
        assert_eq!(buf.generation, 0);
    }

    #[test]
    fn push_fast_basic() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        assert_eq!(buf.count, 1);
        assert_eq!(buf.engine_tick, 1);
    }

    #[test]
    fn ring_wrap_fast() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        // Fill buffer + 1 extra
        for i in 0..=(FAST_CAPACITY as u16) {
            let s = TraceSampleFast {
                position_actual: i as i32,
                ..TraceSampleFast::default()
            };
            buf.push_fast(s);
        }
        assert_eq!(buf.count, FAST_CAPACITY as u16);
        // Offset 0 should be the second sample (first was overwritten)
        buf.stop();
        match buf.fetch(1, 0, buf.generation) {
            FetchResult::Ok(idx) => {
                let sample = unsafe { buf.storage.fast[idx as usize] };
                assert_eq!(sample.position_actual, 1);
            }
            _ => panic!("expected Ok"),
        }
    }

    #[test]
    fn fetch_wrong_oid_returns_invalid() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        assert!(matches!(
            buf.fetch(2, 0, buf.generation),
            FetchResult::Invalid
        ));
    }

    #[test]
    fn fetch_stale_generation_returns_invalid() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        let old_gen = buf.generation;
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        assert!(matches!(buf.fetch(1, 0, old_gen), FetchResult::Invalid));
    }

    #[test]
    fn fetch_during_capture_returns_busy() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        assert!(matches!(buf.fetch(1, 0, buf.generation), FetchResult::Busy));
    }

    #[test]
    fn fetch_invalid_offset_returns_invalid() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        assert!(matches!(
            buf.fetch(1, 1, buf.generation),
            FetchResult::Invalid
        ));
    }

    #[test]
    fn info_wrong_oid_returns_defaults() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        let info = buf.info(2);
        assert_eq!(info.state, 0);
        assert_eq!(info.count, 0);
        assert_eq!(info.generation, 0);
        assert_eq!(info.preset, 0);
    }

    #[test]
    fn info_wrong_magic_returns_defaults() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        buf.magic = 0xDEAD_BEEF; // deliberate wrong magic
        let info = buf.info(1);
        assert_eq!(info.state, 0);
        assert_eq!(info.count, 0);
    }

    #[test]
    fn dropped_count_and_flags() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.push_fast(TraceSampleFast::default()); // tick 0, no flag
        buf.tick_no_sample(true); // tick 1, dropped
        buf.push_fast(TraceSampleFast::default()); // tick 2, should have flag
        buf.stop();
        assert_eq!(buf.dropped_count, 1);
        // Sample at offset 1 (the second stored sample) should have prev_dropped flag
        match buf.fetch(1, 1, buf.generation) {
            FetchResult::Ok(idx) => {
                let sample = unsafe { buf.storage.fast[idx as usize] };
                assert_eq!(sample.flags & FLAG_PREV_DROPPED, FLAG_PREV_DROPPED);
                assert_eq!(sample.engine_tick, 2);
            }
            _ => panic!("expected Ok"),
        }
    }

    #[test]
    fn decimation_not_counted_as_drop() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.tick_no_sample(false); // decimation, not a drop
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        assert_eq!(buf.dropped_count, 0);
        match buf.fetch(1, 0, buf.generation) {
            FetchResult::Ok(idx) => {
                let sample = unsafe { buf.storage.fast[idx as usize] };
                assert_eq!(sample.flags & FLAG_PREV_DROPPED, 0);
            }
            _ => panic!("expected Ok"),
        }
    }

    #[test]
    fn set_phase_changes_subsequent_samples() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FAST);
        buf.set_phase(9);
        buf.push_fast(TraceSampleFast::default());
        buf.set_phase(10);
        buf.push_fast(TraceSampleFast::default());
        buf.stop();
        match buf.fetch(1, 0, buf.generation) {
            FetchResult::Ok(idx) => {
                let s = unsafe { buf.storage.fast[idx as usize] };
                assert_eq!(s.phase_id, 9);
            }
            _ => panic!("expected Ok"),
        }
        match buf.fetch(1, 1, buf.generation) {
            FetchResult::Ok(idx) => {
                let s = unsafe { buf.storage.fast[idx as usize] };
                assert_eq!(s.phase_id, 10);
            }
            _ => panic!("expected Ok"),
        }
    }

    #[test]
    fn full_preset_push_and_fetch() {
        let mut buf = make_buffer();
        buf.start(1, PRESET_FULL);
        let s = TraceSampleFull {
            error_sum_position: 42,
            ..TraceSampleFull::default()
        };
        buf.push_full(s);
        buf.stop();
        assert_eq!(buf.count, 1);
        match buf.fetch(1, 0, buf.generation) {
            FetchResult::Ok(idx) => {
                let fetched = unsafe { buf.storage.full[idx as usize] };
                assert_eq!(fetched.error_sum_position, 42);
            }
            _ => panic!("expected Ok"),
        }
    }
}
