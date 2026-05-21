//! USB trace stream protocol encoding.
//!
//! This module owns the version-1 FTRC wire format. It performs no USB I/O and
//! no TMC register reads; board crates provide samples and transport.

#![cfg(feature = "trace")]

/// FTRC little-endian magic.
pub const FRAME_MAGIC: u32 = 0x4352_5446;
/// Version-1 firmware trace protocol.
pub const PROTOCOL_VERSION: u8 = 1;
/// Fixed envelope length in bytes.
pub const HEADER_LEN: usize = 32;
/// Maximum critical frame transport budget in bytes.
pub const CRITICAL_FRAME_BUDGET: usize = 128;
/// Maximum control-detail frame transport budget in bytes.
pub const CONTROL_DETAIL_FRAME_BUDGET: usize = 96;

/// Critical motion/saturation frame type.
pub const FRAME_TYPE_CRITICAL: u8 = 1;
/// Control-detail frame type.
pub const FRAME_TYPE_CONTROL_DETAIL: u8 = 2;
/// Status/counter frame type.
pub const FRAME_TYPE_STATUS: u8 = 3;
/// Structured event frame type.
pub const FRAME_TYPE_EVENT: u8 = 4;
/// Stream metadata frame type.
pub const FRAME_TYPE_METADATA: u8 = 5;

/// Board-level source kind.
pub const SOURCE_KIND_BOARD: u8 = 0;
/// FOCI/TMC4671 channel source kind.
pub const SOURCE_KIND_FOCI: u8 = 1;
/// USB trace transport source kind.
pub const SOURCE_KIND_USB: u8 = 2;
/// Klipper/protocol source kind.
pub const SOURCE_KIND_KLIPPER: u8 = 3;
/// Source index for not-applicable board or metadata frames.
pub const SOURCE_INDEX_NA: u8 = 0xff;

/// First good frame after drops in the same producer queue.
pub const FLAG_DROP_GAP_BEFORE_THIS_FRAME: u8 = 1 << 0;
/// Payload was shortened.
pub const FLAG_PAYLOAD_TRUNCATED: u8 = 1 << 1;
/// Critical sample contains at least one invalid field due to SPI failure.
pub const SAMPLE_FLAG_SPI_ERROR: u8 = 1 << 0;

/// Version-1 critical payload length.
pub const CRITICAL_PAYLOAD_LEN: usize = 84;
/// Version-1 control-detail payload length.
pub const CONTROL_DETAIL_PAYLOAD_LEN: usize = 28;
/// Version-1 status payload length.
pub const STATUS_PAYLOAD_LEN: usize = 40;
/// Version-1 event payload length.
pub const EVENT_PAYLOAD_LEN: usize = 24;
/// Maximum metadata string bytes in firmware frames.
pub const METADATA_STRING_MAX: usize = 31;
/// Maximum source labels in a metadata frame.
pub const METADATA_SOURCE_MAX: usize = 16;
/// Maximum metadata payload bytes accepted by firmware.
pub const METADATA_PAYLOAD_MAX: usize = 192;
/// Maximum frame bytes stored in firmware queues.
///
/// Critical frames still obey [`CRITICAL_FRAME_BUDGET`]. This larger storage
/// exists for low-rate metadata frames and for 64-byte USB packet chunking.
pub const FRAME_BUF_CAPACITY: usize = 256;

/// Version-1 trace frame header fields excluding fixed magic/version values.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    /// Frame type.
    pub frame_type: u8,
    /// Envelope flags.
    pub flags: u8,
    /// Payload byte count.
    pub payload_len: u16,
    /// Source kind.
    pub source_kind: u8,
    /// Source index.
    pub source_index: u8,
    /// Global per-device sequence.
    pub sequence: u32,
    /// MCU timestamp ticks.
    pub timestamp_ticks: u64,
    /// Best-effort boot/session id.
    pub boot_id: u32,
}

/// Fixed trace frame buffer.
#[derive(Clone, Copy)]
pub struct FrameBuf {
    bytes: [u8; FRAME_BUF_CAPACITY],
    len: usize,
}

impl FrameBuf {
    /// Create an empty frame buffer.
    pub const fn new() -> Self {
        Self {
            bytes: [0; FRAME_BUF_CAPACITY],
            len: HEADER_LEN,
        }
    }

    /// Reset length and clear bytes.
    pub fn clear(&mut self) {
        self.bytes = [0; FRAME_BUF_CAPACITY];
        self.len = HEADER_LEN;
    }

    /// Write the fixed header.
    pub fn write_header(&mut self, header: FrameHeader) {
        self.bytes[0..4].copy_from_slice(&FRAME_MAGIC.to_le_bytes());
        self.bytes[4] = PROTOCOL_VERSION;
        self.bytes[5] = HEADER_LEN as u8;
        self.bytes[6] = header.frame_type;
        self.bytes[7] = header.flags;
        self.bytes[8..10].copy_from_slice(&header.payload_len.to_le_bytes());
        self.bytes[10] = header.source_kind;
        self.bytes[11] = header.source_index;
        self.bytes[12..16].copy_from_slice(&header.sequence.to_le_bytes());
        self.bytes[16..24].copy_from_slice(&header.timestamp_ticks.to_le_bytes());
        self.bytes[24..28].copy_from_slice(&header.boot_id.to_le_bytes());
        self.bytes[28..32].copy_from_slice(&0u32.to_le_bytes());
        self.len = HEADER_LEN + header.payload_len as usize;
    }

    /// Return the initialized bytes.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes[..self.len]
    }

    /// Return the mutable payload area.
    pub fn payload_mut(&mut self, len: usize) -> &mut [u8] {
        assert!(HEADER_LEN + len <= FRAME_BUF_CAPACITY);
        self.len = HEADER_LEN + len;
        &mut self.bytes[HEADER_LEN..HEADER_LEN + len]
    }
}

impl Default for FrameBuf {
    fn default() -> Self {
        Self::new()
    }
}

/// Return the global per-device sequence from a frame envelope.
pub fn frame_sequence(frame: &FrameBuf) -> u32 {
    u32::from_le_bytes([
        frame.bytes[12],
        frame.bytes[13],
        frame.bytes[14],
        frame.bytes[15],
    ])
}

/// Return true when `candidate` should be emitted no later than `other`.
///
/// Sequence numbers are compared in the RFC 1982 style half-range window. The
/// trace queues are tiny relative to `u32::MAX`, so values exactly half a range
/// apart are treated as not before to avoid guessing.
pub fn sequence_before_or_equal(candidate: u32, other: u32) -> bool {
    candidate == other || other.wrapping_sub(candidate) < (1u32 << 31)
}

/// Critical 1 kHz sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CriticalSample {
    /// FOCI step counter or commanded position.
    pub step_position: i32,
    /// TMC position actual.
    pub tmc_position_actual: i32,
    /// Position error.
    pub position_error: i32,
    /// TMC PID velocity target.
    pub target_velocity: i32,
    /// TMC velocity actual.
    pub pid_velocity_actual: i32,
    /// Cached software velocity feedforward offset.
    pub cached_pid_velocity_offset: i32,
    /// Velocity error.
    pub velocity_error: i32,
    /// Enable and direction flags.
    pub enable_and_direction: u8,
    /// Sample/drop flags.
    pub sample_flags: u8,
    /// Timer queue depth.
    pub timer_queue_depth: u16,
    /// Max absolute step-timer jitter since prior sample.
    pub step_timer_jitter: u32,
    /// Minimum deadline slack since prior sample.
    pub min_deadline_slack: i32,
    /// Last served step tick in frame timestamp domain.
    pub last_served_step_tick: u64,
    /// Last Klipper command-service tick in frame timestamp domain.
    pub last_klipper_command_service_tick: u64,
    /// Wrapping missed/deferred step counter.
    pub missed_or_deferred_step_counter: u32,
    /// TMC torque/flux target register.
    pub pid_torque_flux_target: u32,
    /// TMC torque/flux actual register.
    pub pid_torque_flux_actual: u32,
    /// TMC interim FOC voltage request.
    pub interim_foc_uq_ud: u32,
    /// TMC interim limited FOC voltage output.
    pub interim_foc_uq_ud_limited: u32,
}

/// Control-detail sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ControlDetailSample {
    /// INTERIM_PIDOUT_TARGET_TORQUE.
    pub interim_pidout_target_torque: u32,
    /// INTERIM_PIDOUT_TARGET_FLUX.
    pub interim_pidout_target_flux: u32,
    /// INTERIM_PIDIN_TARGET_VELOCITY.
    pub interim_pidin_target_velocity: u32,
    /// INTERIM_PIDOUT_TARGET_VELOCITY.
    pub interim_pidout_target_velocity: u32,
    /// PIDOUT_UQ_UD_LIMITS.
    pub pidout_uq_ud_limits: u32,
    /// PID_TORQUE_FLUX_LIMITS.
    pub pid_torque_flux_limits: u32,
    /// PID_VELOCITY_LIMIT.
    pub pid_velocity_limit: u32,
}

/// Status sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StatusSample {
    /// Status flags.
    pub status_flags: u32,
    /// Latched STATUS interrupt count.
    pub status_interrupt_count: u32,
    /// ADC/current clipping flags.
    pub clipping_flags: u32,
    /// SPI error count.
    pub spi_error_count: u32,
    /// Watchdog or step-timer overrun count.
    pub watchdog_or_step_overrun_count: u32,
    /// MCU receive-side Klipper command queue depth.
    pub klipper_rx_queue_depth: u16,
    /// USB trace queue high-water mark.
    pub usb_trace_queue_high_water: u16,
    /// Dropped motion sample count.
    pub dropped_motion_sample_count: u32,
    /// Dropped event/log count.
    pub dropped_event_log_count: u32,
    /// Host-read underrun or endpoint-not-ready count.
    pub endpoint_not_ready_count: u32,
    /// Suppressed change-triggered status notifications.
    pub suppressed_status_change_count: u32,
}

/// Structured event sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EventSample {
    /// Event severity level.
    pub level: u8,
    /// Subsystem id.
    pub subsystem_id: u8,
    /// Event id.
    pub event_id: u16,
    /// Argument count, 0..4.
    pub arg_count: u8,
    /// Integer event args.
    pub args: [i32; 4],
}

fn put_i32(out: &mut [u8], offset: usize, value: i32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put_u32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_u64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

/// Encode a critical payload into an exact 84-byte buffer.
pub fn encode_critical_payload(sample: &CriticalSample, out: &mut [u8; CRITICAL_PAYLOAD_LEN]) {
    *out = [0; CRITICAL_PAYLOAD_LEN];
    put_i32(out, 0, sample.step_position);
    put_i32(out, 4, sample.tmc_position_actual);
    put_i32(out, 8, sample.position_error);
    put_i32(out, 12, sample.target_velocity);
    put_i32(out, 16, sample.pid_velocity_actual);
    put_i32(out, 20, sample.cached_pid_velocity_offset);
    put_i32(out, 24, sample.velocity_error);
    out[28] = sample.enable_and_direction;
    out[29] = sample.sample_flags;
    put_u16(out, 30, sample.timer_queue_depth);
    put_u32(out, 32, sample.step_timer_jitter);
    put_i32(out, 36, sample.min_deadline_slack);
    put_u64(out, 40, sample.last_served_step_tick);
    put_u64(out, 48, sample.last_klipper_command_service_tick);
    put_u32(out, 56, sample.missed_or_deferred_step_counter);
    put_u32(out, 60, sample.pid_torque_flux_target);
    put_u32(out, 64, sample.pid_torque_flux_actual);
    put_u32(out, 68, sample.interim_foc_uq_ud);
    put_u32(out, 72, sample.interim_foc_uq_ud_limited);
}

/// Encode a control-detail payload.
pub fn encode_control_detail_payload(
    sample: &ControlDetailSample,
    out: &mut [u8; CONTROL_DETAIL_PAYLOAD_LEN],
) {
    *out = [0; CONTROL_DETAIL_PAYLOAD_LEN];
    put_u32(out, 0, sample.interim_pidout_target_torque);
    put_u32(out, 4, sample.interim_pidout_target_flux);
    put_u32(out, 8, sample.interim_pidin_target_velocity);
    put_u32(out, 12, sample.interim_pidout_target_velocity);
    put_u32(out, 16, sample.pidout_uq_ud_limits);
    put_u32(out, 20, sample.pid_torque_flux_limits);
    put_u32(out, 24, sample.pid_velocity_limit);
}

/// Encode a status payload.
pub fn encode_status_payload(sample: &StatusSample, out: &mut [u8; STATUS_PAYLOAD_LEN]) {
    *out = [0; STATUS_PAYLOAD_LEN];
    put_u32(out, 0, sample.status_flags);
    put_u32(out, 4, sample.status_interrupt_count);
    put_u32(out, 8, sample.clipping_flags);
    put_u32(out, 12, sample.spi_error_count);
    put_u32(out, 16, sample.watchdog_or_step_overrun_count);
    put_u16(out, 20, sample.klipper_rx_queue_depth);
    put_u16(out, 22, sample.usb_trace_queue_high_water);
    put_u32(out, 24, sample.dropped_motion_sample_count);
    put_u32(out, 28, sample.dropped_event_log_count);
    put_u32(out, 32, sample.endpoint_not_ready_count);
    put_u32(out, 36, sample.suppressed_status_change_count);
}

/// Encode a structured event payload.
pub fn encode_event_payload(sample: &EventSample, out: &mut [u8; EVENT_PAYLOAD_LEN]) {
    *out = [0; EVENT_PAYLOAD_LEN];
    out[0] = sample.level;
    out[1] = sample.subsystem_id;
    put_u16(out, 2, sample.event_id);
    out[4] = sample.arg_count.min(4);
    put_i32(out, 8, sample.args[0]);
    put_i32(out, 12, sample.args[1]);
    put_i32(out, 16, sample.args[2]);
    put_i32(out, 20, sample.args[3]);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn envelope_is_32_bytes() {
        let mut frame = FrameBuf::new();
        let header = FrameHeader {
            frame_type: FRAME_TYPE_CRITICAL,
            flags: FLAG_DROP_GAP_BEFORE_THIS_FRAME,
            payload_len: 84,
            source_kind: SOURCE_KIND_FOCI,
            source_index: 0,
            sequence: 0x0102_0304,
            timestamp_ticks: 0x1112_1314_1516_1718,
            boot_id: 0x2122_2324,
        };

        frame.write_header(header);

        assert_eq!(&frame.bytes()[0..4], b"FTRC");
        assert_eq!(frame.bytes()[4], PROTOCOL_VERSION);
        assert_eq!(frame.bytes()[5], HEADER_LEN as u8);
        assert_eq!(frame.bytes()[6], FRAME_TYPE_CRITICAL);
        assert_eq!(frame.bytes()[7], FLAG_DROP_GAP_BEFORE_THIS_FRAME);
        assert_eq!(&frame.bytes()[8..10], &84u16.to_le_bytes());
        assert_eq!(frame.bytes()[10], SOURCE_KIND_FOCI);
        assert_eq!(frame.bytes()[11], 0);
        assert_eq!(&frame.bytes()[12..16], &0x0102_0304u32.to_le_bytes());
        assert_eq!(
            &frame.bytes()[16..24],
            &0x1112_1314_1516_1718u64.to_le_bytes()
        );
        assert_eq!(&frame.bytes()[24..28], &0x2122_2324u32.to_le_bytes());
        assert_eq!(&frame.bytes()[28..32], &[0, 0, 0, 0]);
    }

    #[test]
    fn critical_payload_has_exact_offsets() {
        let sample = CriticalSample {
            step_position: -1,
            tmc_position_actual: 2,
            position_error: -3,
            target_velocity: 4,
            pid_velocity_actual: -5,
            cached_pid_velocity_offset: 6,
            velocity_error: -7,
            enable_and_direction: 0x81,
            sample_flags: 0x02,
            timer_queue_depth: 9,
            step_timer_jitter: 10,
            min_deadline_slack: -11,
            last_served_step_tick: 12,
            last_klipper_command_service_tick: 13,
            missed_or_deferred_step_counter: 14,
            pid_torque_flux_target: 15,
            pid_torque_flux_actual: 16,
            interim_foc_uq_ud: 17,
            interim_foc_uq_ud_limited: 18,
        };

        let mut payload = [0xAA; CRITICAL_PAYLOAD_LEN];
        encode_critical_payload(&sample, &mut payload);

        assert_eq!(&payload[0..4], &(-1i32).to_le_bytes());
        assert_eq!(&payload[28..29], &[0x81]);
        assert_eq!(&payload[29..30], &[0x02]);
        assert_eq!(&payload[30..32], &9u16.to_le_bytes());
        assert_eq!(&payload[40..48], &12u64.to_le_bytes());
        assert_eq!(&payload[48..56], &13u64.to_le_bytes());
        assert_eq!(&payload[76..84], &[0; 8]);
    }

    #[test]
    fn fixed_payload_lengths_match_spec() {
        assert_eq!(CRITICAL_PAYLOAD_LEN, 84);
        assert_eq!(CONTROL_DETAIL_PAYLOAD_LEN, 28);
        assert_eq!(STATUS_PAYLOAD_LEN, 40);
        assert_eq!(EVENT_PAYLOAD_LEN, 24);
        assert!(HEADER_LEN + CRITICAL_PAYLOAD_LEN <= CRITICAL_FRAME_BUDGET);
        assert!(HEADER_LEN + METADATA_PAYLOAD_MAX <= FRAME_BUF_CAPACITY);
    }

    #[test]
    fn frame_sequence_reads_envelope_sequence() {
        let mut frame = FrameBuf::new();
        frame.write_header(FrameHeader {
            frame_type: FRAME_TYPE_EVENT,
            flags: 0,
            payload_len: EVENT_PAYLOAD_LEN as u16,
            source_kind: SOURCE_KIND_BOARD,
            source_index: SOURCE_INDEX_NA,
            sequence: 0xA1B2_C3D4,
            timestamp_ticks: 0,
            boot_id: 0,
        });

        assert_eq!(frame_sequence(&frame), 0xA1B2_C3D4);
    }

    #[test]
    fn sequence_before_or_equal_orders_with_wraparound() {
        assert!(sequence_before_or_equal(41, 42));
        assert!(sequence_before_or_equal(42, 42));
        assert!(!sequence_before_or_equal(43, 42));
        assert!(sequence_before_or_equal(u32::MAX, 0));
        assert!(!sequence_before_or_equal(0, u32::MAX));
    }
}
