//! USB trace transport helpers.

use core::sync::atomic::{AtomicU16, Ordering};

use crate::trace::stream::{
    EVENT_PAYLOAD_LEN, EventSample, FRAME_TYPE_EVENT, FRAME_TYPE_STATUS, FrameBuf, FrameHeader,
    METADATA_PAYLOAD_MAX, METADATA_SOURCE_MAX, METADATA_STRING_MAX, SOURCE_INDEX_NA,
    SOURCE_KIND_BOARD, SOURCE_KIND_FOCI, SOURCE_KIND_KLIPPER, SOURCE_KIND_USB, STATUS_PAYLOAD_LEN,
    StatusSample, encode_event_payload, encode_status_payload, sequence_before_or_equal,
};

/// One trace source advertised by metadata and used by source-scoped frames.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub struct TraceSourceDescriptor {
    /// Trace source kind.
    pub source_kind: u8,
    /// Trace source index, or [`SOURCE_INDEX_NA`] for global sources.
    pub source_index: u8,
    /// Stable human-readable source label.
    pub label: &'static str,
}

/// Board-global trace source.
pub const BOARD_SOURCE: TraceSourceDescriptor = TraceSourceDescriptor {
    source_kind: SOURCE_KIND_BOARD,
    source_index: SOURCE_INDEX_NA,
    label: "board",
};

/// USB trace transport source.
pub const USB_SOURCE: TraceSourceDescriptor = TraceSourceDescriptor {
    source_kind: SOURCE_KIND_USB,
    source_index: SOURCE_INDEX_NA,
    label: "usb",
};

/// Klipper/protocol source.
pub const KLIPPER_SOURCE: TraceSourceDescriptor = TraceSourceDescriptor {
    source_kind: SOURCE_KIND_KLIPPER,
    source_index: SOURCE_INDEX_NA,
    label: "klipper",
};

/// Return the canonical FOCI/TMC source descriptor for `channel`.
pub const fn foci_source(channel: u8) -> Option<TraceSourceDescriptor> {
    match channel {
        0 => Some(TraceSourceDescriptor {
            source_kind: SOURCE_KIND_FOCI,
            source_index: 0,
            label: "channel0",
        }),
        1 => Some(TraceSourceDescriptor {
            source_kind: SOURCE_KIND_FOCI,
            source_index: 1,
            label: "channel1",
        }),
        _ => None,
    }
}

/// Metadata fields supplied by board trace integrations.
pub struct TraceMetadataInputs<'a> {
    /// Number of skipped-sample counters encoded before configuration.
    pub skipped_sample_counter_count: u8,
    /// Firmware clock rate in trace ticks per second.
    pub clock_freq: u32,
    /// Advertised critical sample rate in Hz, or zero when variable.
    pub critical_hz: u32,
    /// Advertised control-detail sample rate in Hz, or zero when variable.
    pub control_detail_hz: u32,
    /// Advertised status sample rate in Hz, or zero when variable.
    pub status_hz: u32,
    /// Elapsed milliseconds before trace configuration.
    pub pre_configuration_elapsed_ms: u32,
    /// Critical samples skipped before trace configuration.
    pub pre_configuration_skipped_critical: u32,
    /// Control-detail samples skipped before trace configuration.
    pub pre_configuration_skipped_control_detail: u32,
    /// Status samples skipped before trace configuration.
    pub pre_configuration_skipped_status: u32,
    /// Firmware identifier string.
    pub firmware_id: &'a str,
    /// Product identifier string.
    pub product: &'a str,
    /// USB serial string, when available.
    pub serial: &'a str,
    /// Canonical source descriptors.
    pub sources: &'a [TraceSourceDescriptor],
}

/// Error while building a trace metadata payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TraceMetadataError {
    /// The source list is longer than the protocol metadata budget.
    TooManySources,
    /// A FOCI channel has no shared canonical descriptor.
    UnsupportedFociChannel,
    /// A source label exceeds the metadata string limit.
    SourceLabelTooLong,
    /// The complete metadata payload does not fit in one frame buffer.
    PayloadTooLarge,
}

/// Return canonical sources for a board with `channel_count` FOCI channels.
pub fn canonical_sources(
    channel_count: u8,
) -> Result<heapless::Vec<TraceSourceDescriptor, METADATA_SOURCE_MAX>, TraceMetadataError> {
    if usize::from(channel_count) + 3 > METADATA_SOURCE_MAX {
        return Err(TraceMetadataError::TooManySources);
    }

    let mut sources = heapless::Vec::new();
    sources
        .push(BOARD_SOURCE)
        .map_err(|_| TraceMetadataError::TooManySources)?;
    sources
        .push(USB_SOURCE)
        .map_err(|_| TraceMetadataError::TooManySources)?;
    sources
        .push(KLIPPER_SOURCE)
        .map_err(|_| TraceMetadataError::TooManySources)?;

    let mut channel = 0;
    while channel < channel_count {
        let source = foci_source(channel).ok_or(TraceMetadataError::UnsupportedFociChannel)?;
        sources
            .push(source)
            .map_err(|_| TraceMetadataError::TooManySources)?;
        channel += 1;
    }

    Ok(sources)
}

/// Canonical board status cache slot.
pub const fn board_status_slot() -> usize {
    0
}

/// Canonical USB status cache slot.
pub const fn usb_status_slot() -> usize {
    1
}

/// Canonical Klipper status cache slot.
pub const fn klipper_status_slot() -> usize {
    2
}

/// Canonical FOCI/TMC status cache slot for `channel`.
pub const fn foci_status_slot(channel: u8) -> usize {
    3 + channel as usize
}

/// Total canonical status cache slots for `channel_count` FOCI channels.
pub const fn status_slot_count(channel_count: usize) -> usize {
    3 + channel_count
}

fn metadata_push(
    out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>,
    bytes: &[u8],
) -> Result<(), TraceMetadataError> {
    out.extend_from_slice(bytes)
        .map_err(|_| TraceMetadataError::PayloadTooLarge)
}

fn metadata_push_capped_str(
    out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>,
    value: &str,
) -> Result<(), TraceMetadataError> {
    let bytes = value.as_bytes();
    let len = bytes.len().min(METADATA_STRING_MAX);
    out.push(len as u8)
        .map_err(|_| TraceMetadataError::PayloadTooLarge)?;
    metadata_push(out, &bytes[..len])
}

fn metadata_push_source_label(
    out: &mut heapless::Vec<u8, METADATA_PAYLOAD_MAX>,
    label: &str,
) -> Result<(), TraceMetadataError> {
    let bytes = label.as_bytes();
    if bytes.len() > METADATA_STRING_MAX {
        return Err(TraceMetadataError::SourceLabelTooLong);
    }
    out.push(bytes.len() as u8)
        .map_err(|_| TraceMetadataError::PayloadTooLarge)?;
    metadata_push(out, bytes)
}

/// Build a schema-1 metadata payload from board-supplied values.
pub fn build_metadata_payload(
    inputs: TraceMetadataInputs<'_>,
) -> Result<heapless::Vec<u8, METADATA_PAYLOAD_MAX>, TraceMetadataError> {
    if inputs.sources.len() > METADATA_SOURCE_MAX {
        return Err(TraceMetadataError::TooManySources);
    }

    let mut payload = heapless::Vec::<u8, METADATA_PAYLOAD_MAX>::new();
    metadata_push(
        &mut payload,
        &[
            1,
            1,
            inputs.sources.len() as u8,
            inputs.skipped_sample_counter_count,
        ],
    )?;
    metadata_push(&mut payload, &inputs.clock_freq.to_le_bytes())?;
    metadata_push(&mut payload, &inputs.critical_hz.to_le_bytes())?;
    metadata_push(&mut payload, &inputs.control_detail_hz.to_le_bytes())?;
    metadata_push(&mut payload, &inputs.status_hz.to_le_bytes())?;
    metadata_push(
        &mut payload,
        &inputs.pre_configuration_elapsed_ms.to_le_bytes(),
    )?;
    metadata_push(
        &mut payload,
        &inputs.pre_configuration_skipped_critical.to_le_bytes(),
    )?;
    metadata_push(
        &mut payload,
        &inputs
            .pre_configuration_skipped_control_detail
            .to_le_bytes(),
    )?;
    metadata_push(
        &mut payload,
        &inputs.pre_configuration_skipped_status.to_le_bytes(),
    )?;
    metadata_push_capped_str(&mut payload, inputs.firmware_id)?;
    metadata_push_capped_str(&mut payload, inputs.product)?;
    metadata_push_capped_str(&mut payload, inputs.serial)?;

    for source in inputs.sources {
        metadata_push(&mut payload, &[source.source_kind, source.source_index])?;
        metadata_push_source_label(&mut payload, source.label)?;
    }

    Ok(payload)
}

/// Last emitted status sample for one source slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StatusSnapshot {
    /// Last emitted source kind.
    pub source_kind: u8,
    /// Last emitted source index.
    pub source_index: u8,
    /// Last emitted sample payload.
    pub sample: StatusSample,
}

impl StatusSnapshot {
    /// Create an empty status snapshot.
    pub const fn new() -> Self {
        Self {
            source_kind: 0,
            source_index: 0,
            sample: StatusSample {
                status_flags: 0,
                status_interrupt_count: 0,
                clipping_flags: 0,
                spi_error_count: 0,
                watchdog_or_step_overrun_count: 0,
                klipper_rx_queue_depth: 0,
                usb_trace_queue_high_water: 0,
                dropped_motion_sample_count: 0,
                dropped_event_log_count: 0,
                endpoint_not_ready_count: 0,
                suppressed_status_change_count: 0,
                trace_deadline_pressure_count: 0,
                max_spi_sample_cycles: 0,
            },
        }
    }
}

impl Default for StatusSnapshot {
    fn default() -> Self {
        Self::new()
    }
}

/// Return true when a status sample has any trigger field set.
pub fn status_nonzero(sample: &StatusSample) -> bool {
    sample.status_flags != 0
        || sample.status_interrupt_count != 0
        || sample.clipping_flags != 0
        || sample.spi_error_count != 0
        || sample.watchdog_or_step_overrun_count != 0
        || sample.dropped_motion_sample_count != 0
        || sample.dropped_event_log_count != 0
        || sample.endpoint_not_ready_count != 0
        || sample.suppressed_status_change_count != 0
        || sample.trace_deadline_pressure_count != 0
}

/// Return true when status trigger fields changed.
pub fn status_changed(previous: &StatusSample, next: &StatusSample) -> bool {
    previous.status_flags != next.status_flags
        || previous.status_interrupt_count != next.status_interrupt_count
        || previous.clipping_flags != next.clipping_flags
        || previous.spi_error_count != next.spi_error_count
        || previous.watchdog_or_step_overrun_count != next.watchdog_or_step_overrun_count
        || previous.dropped_motion_sample_count != next.dropped_motion_sample_count
        || previous.dropped_event_log_count != next.dropped_event_log_count
        || previous.endpoint_not_ready_count != next.endpoint_not_ready_count
        || previous.suppressed_status_change_count != next.suppressed_status_change_count
        || previous.trace_deadline_pressure_count != next.trace_deadline_pressure_count
}

/// Decide whether a status frame should be emitted for a source slot.
pub fn should_emit_status(
    previous: &StatusSnapshot,
    source_kind: u8,
    source_index: u8,
    sample: &StatusSample,
    force: bool,
) -> bool {
    let changed = previous.source_kind != source_kind
        || previous.source_index != source_index
        || status_changed(&previous.sample, sample);
    force || status_nonzero(sample) || changed
}

/// Build a structured event frame using a board-supplied header.
pub fn build_event_frame(header: FrameHeader, sample: EventSample) -> FrameBuf {
    assert!(header.frame_type == FRAME_TYPE_EVENT);
    assert!(header.payload_len == EVENT_PAYLOAD_LEN as u16);

    let mut payload = [0u8; EVENT_PAYLOAD_LEN];
    encode_event_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header);
    frame
        .payload_mut(EVENT_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    frame
}

/// Build a status frame using a board-supplied header.
pub fn build_status_frame(header: FrameHeader, sample: StatusSample) -> FrameBuf {
    assert!(header.frame_type == FRAME_TYPE_STATUS);
    assert!(header.payload_len == STATUS_PAYLOAD_LEN as u16);

    let mut payload = [0u8; STATUS_PAYLOAD_LEN];
    encode_status_payload(&sample, &mut payload);
    let mut frame = FrameBuf::new();
    frame.write_header(header);
    frame
        .payload_mut(STATUS_PAYLOAD_LEN)
        .copy_from_slice(&payload);
    frame
}

/// Raise the high-water counter to at least `len`.
pub fn update_high_water(counter: &AtomicU16, len: u16) {
    let mut current = counter.load(Ordering::Relaxed);
    while len > current {
        match counter.compare_exchange_weak(current, len, Ordering::Relaxed, Ordering::Relaxed) {
            Ok(_) => break,
            Err(next) => current = next,
        }
    }
}

/// Queue selected for the next trace frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TraceQueueSelection {
    /// Dequeue from the motion/counter queue.
    Motion,
    /// Dequeue from the structured-event queue.
    Event,
    /// Both queues are empty.
    None,
}

/// Select the next queue by global frame sequence.
pub fn select_next_queue(
    motion_sequence: Option<u32>,
    event_sequence: Option<u32>,
) -> TraceQueueSelection {
    match (motion_sequence, event_sequence) {
        (Some(motion_sequence), Some(event_sequence)) => {
            if sequence_before_or_equal(motion_sequence, event_sequence) {
                TraceQueueSelection::Motion
            } else {
                TraceQueueSelection::Event
            }
        }
        (Some(_), None) => TraceQueueSelection::Motion,
        (None, Some(_)) => TraceQueueSelection::Event,
        (None, None) => TraceQueueSelection::None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::trace::stream::{
        EVENT_PAYLOAD_LEN, FRAME_TYPE_EVENT, FRAME_TYPE_STATUS, HEADER_LEN, METADATA_PAYLOAD_MAX,
        METADATA_STRING_MAX, SOURCE_INDEX_NA, SOURCE_KIND_BOARD, SOURCE_KIND_FOCI,
        SOURCE_KIND_KLIPPER, SOURCE_KIND_USB, STATUS_PAYLOAD_LEN,
    };

    fn header(frame_type: u8, payload_len: usize, flags: u8) -> FrameHeader {
        FrameHeader {
            frame_type,
            flags,
            payload_len: payload_len as u16,
            source_kind: SOURCE_KIND_BOARD,
            source_index: SOURCE_INDEX_NA,
            sequence: 0x0102_0304,
            timestamp_ticks: 0x1112_1314_1516_1718,
            boot_id: 0x2122_2324,
        }
    }

    fn status_with_all_trigger_fields() -> StatusSample {
        StatusSample {
            status_flags: 1,
            status_interrupt_count: 2,
            clipping_flags: 4,
            spi_error_count: 8,
            watchdog_or_step_overrun_count: 16,
            klipper_rx_queue_depth: 32,
            usb_trace_queue_high_water: 64,
            dropped_motion_sample_count: 128,
            dropped_event_log_count: 256,
            endpoint_not_ready_count: 512,
            suppressed_status_change_count: 1024,
            trace_deadline_pressure_count: 2048,
            max_spi_sample_cycles: 4096,
        }
    }

    #[test]
    fn status_nonzero_ignores_queue_pressure_only_fields() {
        let queue_only = StatusSample {
            klipper_rx_queue_depth: 9,
            usb_trace_queue_high_water: 7,
            ..StatusSample::default()
        };
        assert!(!status_nonzero(&queue_only));

        let trigger_fields = status_with_all_trigger_fields();
        assert!(status_nonzero(&trigger_fields));
    }

    #[test]
    fn status_changed_ignores_queue_depth_and_high_water() {
        let previous = StatusSample {
            klipper_rx_queue_depth: 1,
            usb_trace_queue_high_water: 2,
            ..StatusSample::default()
        };
        let next = StatusSample {
            klipper_rx_queue_depth: 7,
            usb_trace_queue_high_water: 9,
            ..StatusSample::default()
        };
        assert!(!status_changed(&previous, &next));

        let changed = StatusSample {
            endpoint_not_ready_count: 1,
            ..next
        };
        assert!(status_changed(&previous, &changed));
    }

    #[test]
    fn should_emit_status_covers_force_source_nonzero_and_changed() {
        let previous = StatusSnapshot::new();
        assert!(!should_emit_status(
            &previous,
            previous.source_kind,
            previous.source_index,
            &StatusSample::default(),
            false,
        ));
        assert!(should_emit_status(
            &previous,
            previous.source_kind,
            previous.source_index,
            &StatusSample::default(),
            true,
        ));
        assert!(should_emit_status(
            &previous,
            SOURCE_KIND_FOCI,
            1,
            &StatusSample::default(),
            false,
        ));
        assert!(should_emit_status(
            &previous,
            SOURCE_KIND_BOARD,
            SOURCE_INDEX_NA,
            &StatusSample {
                spi_error_count: 1,
                ..StatusSample::default()
            },
            false,
        ));

        let previous = StatusSnapshot {
            source_kind: SOURCE_KIND_USB,
            source_index: SOURCE_INDEX_NA,
            sample: StatusSample {
                endpoint_not_ready_count: 1,
                ..StatusSample::default()
            },
        };
        assert!(should_emit_status(
            &previous,
            SOURCE_KIND_USB,
            SOURCE_INDEX_NA,
            &StatusSample {
                endpoint_not_ready_count: 2,
                ..StatusSample::default()
            },
            false,
        ));
    }

    #[test]
    fn canonical_sources_are_global_sources_then_foci_channels() {
        let sources = canonical_sources(2).expect("two-channel source layout fits");

        assert_eq!(
            sources.as_slice(),
            &[
                TraceSourceDescriptor {
                    source_kind: SOURCE_KIND_BOARD,
                    source_index: SOURCE_INDEX_NA,
                    label: "board",
                },
                TraceSourceDescriptor {
                    source_kind: SOURCE_KIND_USB,
                    source_index: SOURCE_INDEX_NA,
                    label: "usb",
                },
                TraceSourceDescriptor {
                    source_kind: SOURCE_KIND_KLIPPER,
                    source_index: SOURCE_INDEX_NA,
                    label: "klipper",
                },
                TraceSourceDescriptor {
                    source_kind: SOURCE_KIND_FOCI,
                    source_index: 0,
                    label: "channel0",
                },
                TraceSourceDescriptor {
                    source_kind: SOURCE_KIND_FOCI,
                    source_index: 1,
                    label: "channel1",
                },
            ]
        );
    }

    #[test]
    fn canonical_sources_reject_unsupported_foci_channel() {
        assert_eq!(
            canonical_sources(3),
            Err(TraceMetadataError::UnsupportedFociChannel)
        );
    }

    #[test]
    fn status_slots_match_canonical_source_order() {
        assert_eq!(board_status_slot(), 0);
        assert_eq!(usb_status_slot(), 1);
        assert_eq!(klipper_status_slot(), 2);
        assert_eq!(foci_status_slot(0), 3);
        assert_eq!(foci_status_slot(1), 4);
        assert_eq!(status_slot_count(0), 3);
        assert_eq!(status_slot_count(1), 4);
        assert_eq!(status_slot_count(2), 5);
    }

    fn expected_metadata_payload_for_two_channels() -> heapless::Vec<u8, METADATA_PAYLOAD_MAX> {
        let mut expected = heapless::Vec::<u8, METADATA_PAYLOAD_MAX>::new();
        expected.extend_from_slice(&[1, 1, 5, 3]).unwrap();
        expected
            .extend_from_slice(&260_000_000u32.to_le_bytes())
            .unwrap();
        expected.extend_from_slice(&1000u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&100u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&10u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&7u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&70u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&700u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(&7000u32.to_le_bytes()).unwrap();
        expected.extend_from_slice(b"\x0couroboros-fw").unwrap();
        expected
            .extend_from_slice(b"\x16FOCI Ouroboros (trace)")
            .unwrap();
        expected.extend_from_slice(b"\x00").unwrap();
        expected
            .extend_from_slice(&[SOURCE_KIND_BOARD, SOURCE_INDEX_NA, 5])
            .unwrap();
        expected.extend_from_slice(b"board").unwrap();
        expected
            .extend_from_slice(&[SOURCE_KIND_USB, SOURCE_INDEX_NA, 3])
            .unwrap();
        expected.extend_from_slice(b"usb").unwrap();
        expected
            .extend_from_slice(&[SOURCE_KIND_KLIPPER, SOURCE_INDEX_NA, 7])
            .unwrap();
        expected.extend_from_slice(b"klipper").unwrap();
        expected
            .extend_from_slice(&[SOURCE_KIND_FOCI, 0, 8])
            .unwrap();
        expected.extend_from_slice(b"channel0").unwrap();
        expected
            .extend_from_slice(&[SOURCE_KIND_FOCI, 1, 8])
            .unwrap();
        expected.extend_from_slice(b"channel1").unwrap();
        expected
    }

    #[test]
    fn metadata_builder_matches_schema_one_wire_order() {
        let sources = canonical_sources(2).expect("two-channel source layout fits");

        let payload = build_metadata_payload(TraceMetadataInputs {
            skipped_sample_counter_count: 3,
            clock_freq: 260_000_000,
            critical_hz: 1000,
            control_detail_hz: 100,
            status_hz: 10,
            pre_configuration_elapsed_ms: 7,
            pre_configuration_skipped_critical: 70,
            pre_configuration_skipped_control_detail: 700,
            pre_configuration_skipped_status: 7000,
            firmware_id: "ouroboros-fw",
            product: "FOCI Ouroboros (trace)",
            serial: "",
            sources: sources.as_slice(),
        })
        .expect("metadata payload fits");

        assert_eq!(payload, expected_metadata_payload_for_two_channels());
    }

    #[test]
    fn metadata_builder_truncates_board_strings_but_not_source_labels() {
        let mut long_string = heapless::String::<64>::new();
        for _ in 0..64 {
            long_string.push('x').unwrap();
        }
        let source = TraceSourceDescriptor {
            source_kind: SOURCE_KIND_BOARD,
            source_index: SOURCE_INDEX_NA,
            label: "board",
        };

        let payload = build_metadata_payload(TraceMetadataInputs {
            skipped_sample_counter_count: 0,
            clock_freq: 1,
            critical_hz: 0,
            control_detail_hz: 0,
            status_hz: 0,
            pre_configuration_elapsed_ms: 0,
            pre_configuration_skipped_critical: 0,
            pre_configuration_skipped_control_detail: 0,
            pre_configuration_skipped_status: 0,
            firmware_id: long_string.as_str(),
            product: long_string.as_str(),
            serial: long_string.as_str(),
            sources: &[source],
        })
        .expect("truncated strings fit");

        let strings_offset = 4 + (8 * 4);
        assert_eq!(payload[strings_offset], METADATA_STRING_MAX as u8);
    }

    #[test]
    fn metadata_builder_rejects_too_many_sources() {
        let sources = [BOARD_SOURCE; 17];

        assert_eq!(
            build_metadata_payload(TraceMetadataInputs {
                skipped_sample_counter_count: 0,
                clock_freq: 1,
                critical_hz: 0,
                control_detail_hz: 0,
                status_hz: 0,
                pre_configuration_elapsed_ms: 0,
                pre_configuration_skipped_critical: 0,
                pre_configuration_skipped_control_detail: 0,
                pre_configuration_skipped_status: 0,
                firmware_id: "fw",
                product: "product",
                serial: "",
                sources: &sources,
            }),
            Err(TraceMetadataError::TooManySources)
        );
    }

    #[test]
    fn metadata_builder_rejects_overlong_source_labels() {
        let overlong_label = "abcdefghijklmnopqrstuvwxyz012345";
        let source = TraceSourceDescriptor {
            source_kind: SOURCE_KIND_BOARD,
            source_index: SOURCE_INDEX_NA,
            label: overlong_label,
        };
        assert!(overlong_label.len() > METADATA_STRING_MAX);

        assert_eq!(
            build_metadata_payload(TraceMetadataInputs {
                skipped_sample_counter_count: 0,
                clock_freq: 1,
                critical_hz: 0,
                control_detail_hz: 0,
                status_hz: 0,
                pre_configuration_elapsed_ms: 0,
                pre_configuration_skipped_critical: 0,
                pre_configuration_skipped_control_detail: 0,
                pre_configuration_skipped_status: 0,
                firmware_id: "fw",
                product: "product",
                serial: "",
                sources: &[source],
            }),
            Err(TraceMetadataError::SourceLabelTooLong)
        );
    }

    #[test]
    fn build_event_frame_preserves_header_and_encodes_payload() {
        let mut event_header = header(FRAME_TYPE_EVENT, EVENT_PAYLOAD_LEN, 0x5a);
        event_header.source_kind = SOURCE_KIND_BOARD;
        event_header.source_index = SOURCE_INDEX_NA;
        let sample = EventSample {
            level: 2,
            subsystem_id: 3,
            event_id: 0x1234,
            arg_count: 5,
            args: [11, -22, 33, -44],
        };

        let frame = build_event_frame(event_header, sample);
        let bytes = frame.bytes();

        assert_eq!(bytes[6], FRAME_TYPE_EVENT);
        assert_eq!(bytes[7], 0x5a);
        assert_eq!(&bytes[8..10], &(EVENT_PAYLOAD_LEN as u16).to_le_bytes());
        assert_eq!(bytes[10], SOURCE_KIND_BOARD);
        assert_eq!(bytes[11], SOURCE_INDEX_NA);
        assert_eq!(&bytes[12..16], &0x0102_0304u32.to_le_bytes());
        assert_eq!(&bytes[16..24], &0x1112_1314_1516_1718u64.to_le_bytes());
        assert_eq!(&bytes[24..28], &0x2122_2324u32.to_le_bytes());

        let payload = &bytes[HEADER_LEN..HEADER_LEN + EVENT_PAYLOAD_LEN];
        assert_eq!(payload[0], 2);
        assert_eq!(payload[1], 3);
        assert_eq!(&payload[2..4], &0x1234u16.to_le_bytes());
        assert_eq!(payload[4], 4);
        assert_eq!(&payload[8..12], &11i32.to_le_bytes());
        assert_eq!(&payload[12..16], &(-22i32).to_le_bytes());
        assert_eq!(&payload[16..20], &33i32.to_le_bytes());
        assert_eq!(&payload[20..24], &(-44i32).to_le_bytes());
    }

    #[test]
    #[should_panic]
    fn build_event_frame_rejects_wrong_payload_len() {
        let bad_header = header(FRAME_TYPE_EVENT, EVENT_PAYLOAD_LEN - 1, 0);
        let _ = build_event_frame(bad_header, EventSample::default());
    }

    #[test]
    fn build_status_frame_preserves_header_and_encodes_payload() {
        let mut status_header = header(FRAME_TYPE_STATUS, STATUS_PAYLOAD_LEN, 0xa5);
        status_header.source_kind = SOURCE_KIND_FOCI;
        status_header.source_index = 1;
        let sample = status_with_all_trigger_fields();

        let frame = build_status_frame(status_header, sample);
        let bytes = frame.bytes();

        assert_eq!(bytes[6], FRAME_TYPE_STATUS);
        assert_eq!(bytes[7], 0xa5);
        assert_eq!(&bytes[8..10], &(STATUS_PAYLOAD_LEN as u16).to_le_bytes());
        assert_eq!(bytes[10], SOURCE_KIND_FOCI);
        assert_eq!(bytes[11], 1);

        let payload = &bytes[HEADER_LEN..HEADER_LEN + STATUS_PAYLOAD_LEN];
        assert_eq!(&payload[0..4], &1u32.to_le_bytes());
        assert_eq!(&payload[4..8], &2u32.to_le_bytes());
        assert_eq!(&payload[8..12], &4u32.to_le_bytes());
        assert_eq!(&payload[12..16], &8u32.to_le_bytes());
        assert_eq!(&payload[16..20], &16u32.to_le_bytes());
        assert_eq!(&payload[20..22], &32u16.to_le_bytes());
        assert_eq!(&payload[22..24], &64u16.to_le_bytes());
        assert_eq!(&payload[24..28], &128u32.to_le_bytes());
        assert_eq!(&payload[28..32], &256u32.to_le_bytes());
        assert_eq!(&payload[32..36], &512u32.to_le_bytes());
        assert_eq!(&payload[36..40], &1024u32.to_le_bytes());
    }

    #[test]
    #[should_panic]
    fn build_status_frame_rejects_wrong_payload_len() {
        let bad_header = header(FRAME_TYPE_STATUS, STATUS_PAYLOAD_LEN + 1, 0);
        let _ = build_status_frame(bad_header, StatusSample::default());
    }

    #[test]
    fn update_high_water_only_raises_counter() {
        let counter = AtomicU16::new(5);
        update_high_water(&counter, 3);
        assert_eq!(counter.load(Ordering::Relaxed), 5);
        update_high_water(&counter, 9);
        assert_eq!(counter.load(Ordering::Relaxed), 9);
    }

    #[test]
    fn select_next_queue_handles_empty_single_and_ordered_cases() {
        assert_eq!(select_next_queue(None, None), TraceQueueSelection::None);
        assert_eq!(
            select_next_queue(Some(10), None),
            TraceQueueSelection::Motion
        );
        assert_eq!(
            select_next_queue(None, Some(10)),
            TraceQueueSelection::Event
        );
        assert_eq!(
            select_next_queue(Some(10), Some(11)),
            TraceQueueSelection::Motion
        );
        assert_eq!(
            select_next_queue(Some(12), Some(11)),
            TraceQueueSelection::Event
        );
    }

    #[test]
    fn select_next_queue_preserves_wraparound_ordering() {
        assert_eq!(
            select_next_queue(Some(u32::MAX), Some(0)),
            TraceQueueSelection::Motion
        );
        assert_eq!(
            select_next_queue(Some(0), Some(u32::MAX)),
            TraceQueueSelection::Event
        );
    }
}
