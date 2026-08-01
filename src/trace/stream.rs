//! USB trace stream protocol encoding.
//!
//! This module owns the FTRC wire format. It performs no USB I/O and
//! no TMC register reads; board crates provide samples and transport.

#![cfg(feature = "trace")]

/// FTRC little-endian magic.
pub const FRAME_MAGIC: u32 = 0x4352_5446;
/// Firmware trace protocol version. Bumped to 10 for the independent-replay
/// raw-observation record expansion (51 -> 99 bytes), a breaking wire change: an
/// old decoder must reject a version-10 stream at the version gate rather than
/// misdecode the wider record.
pub const PROTOCOL_VERSION: u8 = 10;
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
/// Low-rate current-path diagnostic frame type.
pub const FRAME_TYPE_CURRENT_PATH: u8 = 6;
/// Runtime phase-advance diagnostic frame type.
pub const FRAME_TYPE_PHASE_ADVANCE: u8 = 7;
/// Characterization-only raw velocity observation frame type.
pub const FRAME_TYPE_VELOCITY_OBSERVATION: u8 = 8;
/// Canonical velocity-sweep logical evidence frame type.
pub const FRAME_TYPE_VELOCITY_SWEEP_EVIDENCE: u8 = 9;

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
/// Acceleration feedforward was enabled and ungated for this critical sample.
pub const ACCEL_FF_FLAG_ENABLED: u8 = 1 << 0;
/// Acceleration feedforward wrote a non-zero torque offset for this sample.
pub const ACCEL_FF_FLAG_NONZERO: u8 = 1 << 1;
/// Acceleration feedforward torque offset reached the configured torque limit.
pub const ACCEL_FF_FLAG_CLAMPED: u8 = 1 << 2;
/// Phase advance was enabled and ungated for this sample.
pub const PHASE_ADVANCE_FLAG_ENABLED: u8 = 1 << 0;
/// Phase advance requested a non-zero electrical-angle offset.
pub const PHASE_ADVANCE_FLAG_NONZERO: u8 = 1 << 1;
/// Phase advance reached the configured cap.
pub const PHASE_ADVANCE_FLAG_CLAMPED: u8 = 1 << 2;
/// Phase advance output was zeroed by velocity deadband.
pub const PHASE_ADVANCE_FLAG_DEADBAND: u8 = 1 << 3;
/// Phase advance was gated by homing, reset, or disabled state.
pub const PHASE_ADVANCE_FLAG_GATED: u8 = 1 << 4;
/// Last phase-advance register write failed; hardware state may be stale.
pub const PHASE_ADVANCE_FLAG_WRITE_FAILED: u8 = 1 << 5;

/// Version-6 critical payload length.
pub const CRITICAL_PAYLOAD_LEN: usize = 88;
/// Version-5 current-path payload length.
pub const CURRENT_PATH_PAYLOAD_LEN: usize = 16;
/// Version-1 phase-advance payload length.
pub const PHASE_ADVANCE_PAYLOAD_LEN: usize = 12;
/// Version-1 control-detail payload length.
pub const CONTROL_DETAIL_PAYLOAD_LEN: usize = 28;
/// Version-1 status payload length.
pub const STATUS_PAYLOAD_LEN: usize = 40;
/// Version-1 event payload length.
pub const EVENT_PAYLOAD_LEN: usize = 24;
/// Characterization velocity-observation payload length.
pub const VELOCITY_OBSERVATION_PAYLOAD_LEN: usize = 22;
/// Velocity observation met the existing torque-saturation threshold.
pub const VELOCITY_OBSERVATION_FLAG_SATURATED: u8 = 1 << 0;
/// Velocity observation includes a sampled `PID_VELOCITY_ACTUAL` value.
pub const VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID: u8 = 1 << 1;
/// `velocity_diagnostic_value` carries `ACTUAL_VELOCITY_PPTM`.
pub const VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID: u8 = 1 << 2;
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

/// Trace frame header fields excluding fixed magic/version values.
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

/// Return true when the control-detail sample is due for this tick.
///
/// The normal motion-tuning cadence is 100 Hz (`tick % 10 == 0`). Queue
/// pressure decimates deterministically to 50 Hz above half depth and 20 Hz
/// above three-quarter depth. A deadline-pressure skip suppresses exactly this
/// opportunity.
pub fn control_detail_due(
    control_tick: u32,
    queue_len: usize,
    queue_depth: usize,
    deadline_skip: bool,
) -> bool {
    if deadline_skip {
        return false;
    }

    let divisor = if queue_depth > 0 && queue_len >= (queue_depth * 3) / 4 {
        50
    } else if queue_depth > 0 && queue_len >= queue_depth / 2 {
        20
    } else {
        10
    };
    control_tick.is_multiple_of(divisor)
}

/// Return true when the low-rate current-path sample is due for this tick.
///
/// Current-path sampling is disabled by default in the motion-tuning profile.
/// When a diagnostic profile enables it, the normal cadence is 100 Hz. Samples
/// are emitted on `tick % 10 == 5` so the extra INTERIM subregister reads do
/// not coincide with the control-detail cadence. Queue pressure decimates
/// deterministically.
pub fn current_path_due(
    enabled: bool,
    control_tick: u32,
    queue_len: usize,
    queue_depth: usize,
) -> bool {
    if !enabled {
        return false;
    }

    let divisor = if queue_depth > 0 && queue_len >= (queue_depth * 3) / 4 {
        50
    } else if queue_depth > 0 && queue_len >= queue_depth / 2 {
        20
    } else {
        10
    };
    control_tick % divisor == 5
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
    /// Software command rate in TMC position counts per PWM cycle.
    ///
    /// This is derived from Klipper STEP/DIR timing. It is not a TMC register read.
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
    /// TMC clipped current-loop input target torque from INTERIM_ADDR 0.
    pub interim_pidin_target_torque: u32,
    /// TMC packed torque target/actual current monitor from INTERIM_ADDR 21.
    pub interim_pid_torque_target_actual: u32,
    /// TMC interim FOC voltage request.
    pub interim_foc_uq_ud: u32,
    /// TMC interim limited FOC voltage output.
    pub interim_foc_uq_ud_limited: u32,
    /// Cached software acceleration feedforward torque offset.
    pub cached_accel_torque_offset: i32,
    /// Runtime acceleration feedforward gain in permille.
    pub accel_feedforward_gain_permille: u16,
    /// Acceleration feedforward diagnostic flags.
    pub accel_feedforward_flags: u8,
    /// Cached velocity-surface transient feedforward component.
    pub cached_velocity_transient_offset: i32,
}

/// Low-rate current-path diagnostic sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct CurrentPathSample {
    /// Sample/drop flags.
    pub sample_flags: u8,
    /// TMC interim FOC dq current monitor.
    pub interim_foc_iq_id: u32,
    /// TMC interim FOC phase-current monitor.
    pub interim_foc_iwy_iux: u32,
    /// TMC interim raw ADC current monitor.
    pub interim_adc_i1_i0: u32,
}

/// Low-rate runtime phase-advance diagnostic sample.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct PhaseAdvanceSample {
    /// Requested signed electrical-angle advance in PHI_E counts.
    pub phase_advance_counts: i16,
    /// Phase-advance diagnostic flags.
    pub phase_advance_flags: u8,
    /// Reserved alignment byte.
    pub reserved: u8,
    /// Configured signed gain in ppm.
    pub gain_ppm: i32,
    /// Configured maximum absolute advance in PHI_E counts.
    pub max_counts: u16,
    /// Configured target-velocity deadband.
    pub deadband: u16,
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

/// Raw signed velocity observation emitted only by characterization firmware.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct VelocityTraceObservation {
    /// Signed commanded encoder rate in counts per millisecond.
    pub target_counts_per_ms: i32,
    /// Wrap-aware signed encoder displacement.
    pub delta_counts: i32,
    /// Actual interval between encoder observations.
    pub elapsed_us: u32,
    /// Signed-stroke identity within the velocity-validation run.
    pub stroke_index: u16,
    /// Commanded direction, encoded as `1` or `-1`.
    pub direction: i8,
    /// Observation evidence flags.
    pub flags: u8,
    /// Peak absolute velocity-controller torque target during this stroke.
    pub peak_torque_target_abs: u16,
    /// Decimated signed velocity diagnostic selected by the mutually exclusive flags.
    pub velocity_diagnostic_value: i32,
}

fn put_i32(out: &mut [u8], offset: usize, value: i32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put_i16(out: &mut [u8], offset: usize, value: i16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
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
    put_u32(out, 60, sample.interim_pidin_target_torque);
    put_u32(out, 64, sample.interim_pid_torque_target_actual);
    put_u32(out, 68, sample.interim_foc_uq_ud);
    put_u32(out, 72, sample.interim_foc_uq_ud_limited);
    put_i32(out, 76, sample.cached_accel_torque_offset);
    put_u16(out, 80, sample.accel_feedforward_gain_permille);
    out[82] = sample.accel_feedforward_flags;
    put_i32(out, 84, sample.cached_velocity_transient_offset);
}

/// Encode a current-path payload.
pub fn encode_current_path_payload(
    sample: &CurrentPathSample,
    out: &mut [u8; CURRENT_PATH_PAYLOAD_LEN],
) {
    *out = [0; CURRENT_PATH_PAYLOAD_LEN];
    out[0] = sample.sample_flags;
    put_u32(out, 4, sample.interim_foc_iq_id);
    put_u32(out, 8, sample.interim_foc_iwy_iux);
    put_u32(out, 12, sample.interim_adc_i1_i0);
}

/// Encode a phase-advance payload.
pub fn encode_phase_advance_payload(
    sample: &PhaseAdvanceSample,
    out: &mut [u8; PHASE_ADVANCE_PAYLOAD_LEN],
) {
    *out = [0; PHASE_ADVANCE_PAYLOAD_LEN];
    put_i16(out, 0, sample.phase_advance_counts);
    out[2] = sample.phase_advance_flags;
    out[3] = sample.reserved;
    put_i32(out, 4, sample.gain_ppm);
    put_u16(out, 8, sample.max_counts);
    put_u16(out, 10, sample.deadband);
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

/// Encode a characterization velocity observation into its exact wire payload.
pub fn encode_velocity_observation_payload(
    sample: &VelocityTraceObservation,
    out: &mut [u8; VELOCITY_OBSERVATION_PAYLOAD_LEN],
) {
    *out = [0; VELOCITY_OBSERVATION_PAYLOAD_LEN];
    put_i32(out, 0, sample.target_counts_per_ms);
    put_i32(out, 4, sample.delta_counts);
    put_u32(out, 8, sample.elapsed_us);
    put_u16(out, 12, sample.stroke_index);
    out[14] = sample.direction as u8;
    out[15] = sample.flags;
    put_u16(out, 16, sample.peak_torque_target_abs);
    put_i32(out, 18, sample.velocity_diagnostic_value);
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
            interim_pidin_target_torque: 15,
            interim_pid_torque_target_actual: 16,
            interim_foc_uq_ud: 17,
            interim_foc_uq_ud_limited: 18,
            cached_accel_torque_offset: -19,
            accel_feedforward_gain_permille: 2000,
            accel_feedforward_flags: 0x03,
            cached_velocity_transient_offset: 21,
        };

        let mut payload = [0xAA; CRITICAL_PAYLOAD_LEN];
        encode_critical_payload(&sample, &mut payload);

        assert_eq!(&payload[0..4], &(-1i32).to_le_bytes());
        assert_eq!(&payload[28..29], &[0x81]);
        assert_eq!(&payload[29..30], &[0x02]);
        assert_eq!(&payload[30..32], &9u16.to_le_bytes());
        assert_eq!(&payload[40..48], &12u64.to_le_bytes());
        assert_eq!(&payload[48..56], &13u64.to_le_bytes());
        assert_eq!(&payload[60..64], &15u32.to_le_bytes());
        assert_eq!(&payload[64..68], &16u32.to_le_bytes());
        assert_eq!(&payload[76..80], &(-19i32).to_le_bytes());
        assert_eq!(&payload[80..82], &2000u16.to_le_bytes());
        assert_eq!(&payload[82..83], &[0x03]);
        assert_eq!(&payload[83..84], &[0]);
        assert_eq!(&payload[84..88], &21i32.to_le_bytes());
    }

    #[test]
    fn fixed_payload_lengths_match_spec() {
        assert_eq!(PROTOCOL_VERSION, 10);
        assert_eq!(CRITICAL_PAYLOAD_LEN, 88);
        assert_eq!(CURRENT_PATH_PAYLOAD_LEN, 16);
        assert_eq!(PHASE_ADVANCE_PAYLOAD_LEN, 12);
        assert_eq!(CONTROL_DETAIL_PAYLOAD_LEN, 28);
        assert_eq!(STATUS_PAYLOAD_LEN, 40);
        assert_eq!(EVENT_PAYLOAD_LEN, 24);
        const {
            assert!(HEADER_LEN + CRITICAL_PAYLOAD_LEN <= CRITICAL_FRAME_BUDGET);
            assert!(HEADER_LEN + CURRENT_PATH_PAYLOAD_LEN <= CONTROL_DETAIL_FRAME_BUDGET);
            assert!(HEADER_LEN + PHASE_ADVANCE_PAYLOAD_LEN <= 64);
            assert!(HEADER_LEN + VELOCITY_OBSERVATION_PAYLOAD_LEN <= 64);
            assert!(HEADER_LEN + METADATA_PAYLOAD_MAX <= FRAME_BUF_CAPACITY);
        }
    }

    #[test]
    fn current_path_payload_has_exact_offsets() {
        let sample = CurrentPathSample {
            sample_flags: 0x02,
            interim_foc_iq_id: 20,
            interim_foc_iwy_iux: 21,
            interim_adc_i1_i0: 22,
        };

        let mut payload = [0xAA; CURRENT_PATH_PAYLOAD_LEN];
        encode_current_path_payload(&sample, &mut payload);

        assert_eq!(&payload[0..1], &[0x02]);
        assert_eq!(&payload[1..4], &[0; 3]);
        assert_eq!(&payload[4..8], &20u32.to_le_bytes());
        assert_eq!(&payload[8..12], &21u32.to_le_bytes());
        assert_eq!(&payload[12..16], &22u32.to_le_bytes());
    }

    #[test]
    fn velocity_observation_payload_has_exact_signed_layout() {
        let sample = VelocityTraceObservation {
            target_counts_per_ms: -4,
            delta_counts: -7,
            elapsed_us: 1_037,
            stroke_index: 513,
            direction: -1,
            flags: VELOCITY_OBSERVATION_FLAG_SATURATED
                | VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID,
            peak_torque_target_abs: 900,
            velocity_diagnostic_value: -8,
        };
        let mut payload = [0xAA; VELOCITY_OBSERVATION_PAYLOAD_LEN];

        encode_velocity_observation_payload(&sample, &mut payload);

        assert_eq!(FRAME_TYPE_VELOCITY_OBSERVATION, 8);
        assert_eq!(VELOCITY_OBSERVATION_PAYLOAD_LEN, 22);
        assert_eq!(&payload[0..4], &(-4i32).to_le_bytes());
        assert_eq!(&payload[4..8], &(-7i32).to_le_bytes());
        assert_eq!(&payload[8..12], &1_037u32.to_le_bytes());
        assert_eq!(&payload[12..14], &513u16.to_le_bytes());
        assert_eq!(payload[14], 0xFF);
        assert_eq!(
            payload[15],
            VELOCITY_OBSERVATION_FLAG_SATURATED | VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID
        );
        assert_eq!(&payload[16..18], &900u16.to_le_bytes());
        assert_eq!(&payload[18..22], &(-8i32).to_le_bytes());
    }

    #[test]
    fn velocity_diagnostic_kinds_are_mutually_exclusive() {
        assert_ne!(
            VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID,
            VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID
        );
        assert_eq!(
            VELOCITY_OBSERVATION_FLAG_PID_VELOCITY_ACTUAL_VALID
                & VELOCITY_OBSERVATION_FLAG_PPTM_ACTUAL_VALID,
            0
        );
    }

    #[test]
    fn phase_advance_payload_layout_is_fixed() {
        let sample = PhaseAdvanceSample {
            phase_advance_counts: -64,
            phase_advance_flags: PHASE_ADVANCE_FLAG_ENABLED
                | PHASE_ADVANCE_FLAG_NONZERO
                | PHASE_ADVANCE_FLAG_CLAMPED,
            reserved: 0,
            gain_ppm: -60_000,
            max_counts: 64,
            deadband: 16,
        };
        let mut payload = [0xAA; PHASE_ADVANCE_PAYLOAD_LEN];
        encode_phase_advance_payload(&sample, &mut payload);

        assert_eq!(PHASE_ADVANCE_PAYLOAD_LEN, 12);
        assert_eq!(i16::from_le_bytes([payload[0], payload[1]]), -64);
        assert_eq!(payload[2], 0b0000_0111);
        assert_eq!(payload[3], 0);
        assert_eq!(
            i32::from_le_bytes(payload[4..8].try_into().unwrap()),
            -60_000
        );
        assert_eq!(u16::from_le_bytes([payload[8], payload[9]]), 64);
        assert_eq!(u16::from_le_bytes([payload[10], payload[11]]), 16);
        const {
            assert!(HEADER_LEN + PHASE_ADVANCE_PAYLOAD_LEN <= 64);
        }
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

    #[test]
    fn control_detail_due_uses_motion_tuning_rate_without_pressure() {
        assert!(!control_detail_due(5, 0, 32, false));
        assert!(control_detail_due(10, 0, 32, false));
        assert!(!control_detail_due(10, 0, 32, true));
    }

    #[test]
    fn control_detail_due_decimates_under_queue_pressure() {
        assert!(!control_detail_due(10, 16, 32, false));
        assert!(control_detail_due(20, 16, 32, false));
        assert!(!control_detail_due(20, 24, 32, false));
        assert!(control_detail_due(50, 24, 32, false));
    }

    #[test]
    fn current_path_due_is_disabled_in_motion_tuning_profile() {
        assert!(!current_path_due(false, 5, 0, 32));
        assert!(!current_path_due(false, 25, 16, 32));
        assert!(!current_path_due(false, 55, 24, 32));
    }

    #[test]
    fn current_path_due_uses_odd_ticks_when_enabled() {
        assert!(!current_path_due(true, 4, 0, 32));
        assert!(current_path_due(true, 5, 0, 32));
        assert!(!current_path_due(true, 10, 0, 32));
        assert!(current_path_due(true, 15, 0, 32));
        assert!(!current_path_due(true, 15, 16, 32));
        assert!(current_path_due(true, 25, 16, 32));
        assert!(!current_path_due(true, 25, 24, 32));
        assert!(current_path_due(true, 55, 24, 32));
    }
}
