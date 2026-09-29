use crate::commissioning::backend::{
    CommissionBoardConfig, CommissionEncoderRead, CommissionMotorControl, CommissionPendingConfig,
    CommissionRegisterIo, CommissionStepStimulus, CommissionTiming, CommissionTrace, StimulusError,
};
use crate::config::TmcConfig;
use crate::tmc_control::lifecycle::LifecycleFlag;
use crate::tmc_control::pending_config::{PendingConfigItem, PendingConfigSink};
use core::sync::atomic::Ordering::Relaxed;
use foci_klipper::stepper::StepMove;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::Register;
use heapless::Vec;

/// Synthetic SPI error for test injection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MockSpiError;

/// Parameters for `MockBackend::script_gain_floor_capture`'s synthesized
/// quadratic position / exponential current response.
#[derive(Debug, Clone, Copy)]
pub struct GainFloorCaptureScript {
    pub accel_raw_per_us2_per_count: f64,
    pub accel_raw_offset_per_us2: f64,
    pub tau_current_us: f64,
    pub overshoot_ratio: f64,
    pub creep_below_target: i16,
    pub creep_excess_accel_per_us2: f64,
}

const GAIN_FLOOR_RING_ONSET_US: f64 = 60.0;
const GAIN_FLOOR_RING_DECAY_US: f64 = 300.0;

/// Construct a validated current limit for test fixtures.
pub fn current_limit(raw: u32) -> PidTorqueFluxLimit {
    PidTorqueFluxLimit::new(raw).expect("test current limit must be valid")
}

/// Construct the shared outer-entry snapshot used by dispatcher fixtures.
pub fn sample_entry_snapshot() -> crate::commissioning::OuterGainRollbackSnapshot {
    crate::commissioning::OuterGainRollbackSnapshot {
        velocity_p: crate::pid::p_gain(1152),
        velocity_i: crate::pid::i_gain(0),
        position_p: crate::pid::p_gain(640),
        position_i: crate::pid::i_gain(0),
        velocity_limit: 500_000,
        voltage_limit: 16000,
        velocity_filter_hz: 0,
        torque_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
    }
}

/// Construct the shared outer-tune parameters used by dispatcher fixtures.
pub fn sample_outer_params() -> crate::commissioning::OuterAutotuneInputs {
    crate::commissioning::OuterAutotuneInputs {
        slot_order:
            crate::commissioning::outer::velocity::integral_plan::IntegralSlotOrder::ForwardFirst,
        profile: crate::autotune::CommissionProfile::Balanced,
        mode: crate::autotune::OperatingMode::Nominal,
        inner_lambda_us: 100,
        theta_e_us: 50,
        current_bandwidth_hz: 1000,
        inner_warning_flags: 0,
        requested_velocity_mrev_s: 6000,
        machine_velocity_ceiling_mrev_s: 10_000,
        requested_velocity_source:
            crate::commissioning::outer::velocity::sweep_plan::RequestedVelocitySource::Defaulted,
        max_stroke_travel_mrev: 1000,
        settle_travel_reserve_mrev: 100,
        negative_position_headroom_mrev: 2000,
        positive_position_headroom_mrev: 2000,
        max_duration_ms: 3000,
        homing_speed_mrev_s: 4_375,
        max_accel_mrev_s2: 200_000,
    }
}

/// A genuine breakaway confirmation, minted through the same
/// `plan_band_confirmation` -> `evaluate_band_confirmation` ->
/// `bind_breakaway_integral_authority` chain a live campaign uses.
pub struct BreakawayConfirmationForTest {
    pub token: crate::commissioning::outer::velocity::integral_plan::ConfirmedAcquisitionDigest,
    pub confirmation_plan:
        crate::commissioning::outer::velocity::band_confirmation::BandConfirmationPlan,
    pub held_out_response_intervals_q: [[i32; 2]; 2],
    pub confidence_q: u16,
    pub band_position_q: u16,
}

pub fn breakaway_confirmation_for_test() -> BreakawayConfirmationForTest {
    use crate::autotune::velocity_measurement::VelocityDirection;
    use crate::commissioning::outer::velocity::band_confirmation::{
        ConfirmationDirectionalInput, DiscoveryObservationAtom, evaluate_band_confirmation,
        plan_band_confirmation,
    };
    use crate::commissioning::outer::velocity::integral_plan::bind_breakaway_integral_authority;
    use crate::commissioning::outer::velocity::sweep_consensus::{
        CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    };
    use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
    use foci_tmc4671::pid::PidPGain;

    let nominated_p = PidPGain::new(512).expect("raw gain within PidPGain range");
    let confirmation_plan = plan_band_confirmation(0x1111_2222_3333_4444, nominated_p)
        .expect("valid confirmation plan");

    let consensus = || DirectionalConsensus {
        classification: DirectionalConsensusClass::EligibleConsensus,
        components: [[0, 0]; 2],
        component_count: 1,
        collected_mask: 0b1111,
        eligible_mask: 0b1111,
        included_mask: 0b1111,
    };
    let mean_rate_q: i64 = 750_000;
    let variance_of_mean_q2: u128 = 58_752_225;
    let target_rate_q: i64 = 1_000_000;
    let forward = ConfirmationDirectionalInput {
        consensus: consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Forward,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let reverse = ConfirmationDirectionalInput {
        consensus: consensus(),
        classes: [VelocityObservationClass::Eligible; CONSENSUS_OBSERVATIONS_PER_DIRECTION],
        observations: [Some(DiscoveryObservationAtom {
            mean_rate_q: -mean_rate_q,
            variance_of_mean_q2,
            target_rate_q,
            direction: VelocityDirection::Reverse,
        }); CONSENSUS_OBSERVATIONS_PER_DIRECTION],
    };
    let acceptance =
        evaluate_band_confirmation(nominated_p, forward, reverse).expect("accepted confirmation");
    let token = bind_breakaway_integral_authority(
        confirmation_plan.plan_digest,
        confirmation_plan,
        Ok(acceptance),
    )
    .expect("bound breakaway authority");
    BreakawayConfirmationForTest {
        token,
        confirmation_plan,
        held_out_response_intervals_q: acceptance.held_out_response_intervals_q,
        confidence_q: acceptance.confidence_q,
        band_position_q: acceptance.band_position_q,
    }
}

/// Mint one genuine breakaway confirmation token; see
/// [`breakaway_confirmation_for_test`] for the fuller bundle an Integral plan
/// builder needs.
pub fn breakaway_acquisition_for_test()
-> crate::commissioning::outer::velocity::integral_plan::ConfirmedAcquisitionDigest {
    breakaway_confirmation_for_test().token
}

/// Build a real admitted breakaway engine for cross-module lifecycle tests.
pub fn breakaway_campaign_engine() -> crate::commissioning::CommissioningEngine {
    use crate::commissioning::outer::velocity::sweep_plan::VelocitySweepRequest;

    let params = sample_outer_params();
    let request = VelocitySweepRequest {
        requested_velocity_mrev_s: params.requested_velocity_mrev_s,
        machine_velocity_ceiling_mrev_s: params.machine_velocity_ceiling_mrev_s,
        requested_velocity_source: params.requested_velocity_source,
        max_stroke_travel_mrev: params.max_stroke_travel_mrev,
        settle_travel_reserve_mrev: params.settle_travel_reserve_mrev,
        negative_position_headroom_mrev: params.negative_position_headroom_mrev,
        positive_position_headroom_mrev: params.positive_position_headroom_mrev,
        max_duration_ms: params.max_duration_ms,
    };
    let (campaign, _workflow) =
        crate::commissioning::outer::velocity::breakaway_campaign::plan_breakaway_campaign_command(
            request, 4_000, 2_816, 0, 25_000, 0, 7,
        )
        .expect("a valid host request must admit a breakaway campaign");
    crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_breakaway_campaign(
        0,
        crate::state::tmc::tmc_channel0(),
        params,
        sample_entry_snapshot(),
        campaign,
    )
    .expect("valid breakaway safety configuration")
}

/// Build a real admitted reversal-standstill robustness engine for
/// cross-module lifecycle tests.
pub fn robustness_reversal_engine() -> crate::commissioning::CommissioningEngine {
    use crate::commission_dispatch::robustness::RobustnessPreflight;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptancePoint, FixedGainAmplitudeProgress, IntegralPlanIdentity,
    };

    let point = AcceptancePoint {
        identity: IntegralPlanIdentity {
            channel: 0,
            build_features: 0x1234,
            schema_revision: 1,
            plan_digest: 0xfeed_beef,
        },
        selected_p: 1_024,
        selected_i: 512,
        target_velocity_rpm: 176,
        selected_response_intervals_q: [[10, 20], [-20, -10]],
        confidence_q: 0,
        band_position_q: 0,
        qualifier_bits: 0,
        token: None,
        progress: FixedGainAmplitudeProgress::ReadyAscending,
    };
    crate::commissioning::CommissioningEngine::new_outer_for_tmc_with_robustness(
        0,
        crate::state::tmc::tmc_channel0(),
        sample_outer_params(),
        sample_entry_snapshot(),
        RobustnessPreflight {
            point,
            run_sequence: 0,
        },
        4_000,
    )
    .expect("valid robustness safety configuration")
}

/// A single SPI operation recorded by `MockBackend`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpiOp {
    Read,
    Write(u8, u32),
    EnablePin(bool),
    Lifecycle { flag: LifecycleFlag, value: bool },
    DrainPendingConfig,
}

/// Mock implementation of `CommissioningBackend` for host-side unit tests.
pub struct MockBackend {
    pub registers: heapless::LinearMap<u8, u32, 64>,
    config_selected_subregister: u8,
    advanced_pi_representation: u32,
    pub spi_log: std::vec::Vec<SpiOp>,
    pub register_pair_reads: usize,
    pub capture_register_reads: usize,
    pub velocity_controller_output_selects: usize,
    pub pid_velocity_error_sum_clears: usize,
    pub pid_velocity_actual_reads: usize,
    pub pid_velocity_actual_update_interim_on_read: Option<(usize, u32)>,
    pub trace_events: Vec<crate::trace::events::CommissionTraceEvent, 64>,
    #[cfg(feature = "trace")]
    pub velocity_trace_observations: Vec<crate::trace::stream::VelocityTraceObservation, 2_048>,
    #[cfg(feature = "trace")]
    pub velocity_sweep_trace_records:
        Vec<crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord, 128>,
    #[cfg(feature = "trace")]
    pub trace_emission_order: Vec<u16, 256>,
    pub enable_pin: bool,
    pub interlock_tripped: bool,
    pub trip_interlock_on_write: Option<u8>,
    pub delay_us_log: Vec<u32, 32>,
    pub fail_on_write: Option<u8>,
    pub fail_on_write_occurrence: Option<(u8, usize)>,
    pub fail_on_read: Option<u8>,
    pub pend_next_op: bool,
    pub fail_writes_remaining: Option<(u8, u8)>,
    pub fail_reads_remaining: Option<(u8, u8)>,
    pub last_motor_sentinel_stored: core::cell::Cell<Option<u32>>,
    pub read_override: Option<(u8, u32)>,
    pub read_override_remaining: Option<(u8, u32, u8)>,
    pub elapsed_us_value: u64,
    elapsed_us_sequence: Vec<u64, 16>,
    elapsed_us_pos: core::cell::Cell<usize>,
    pub advance_elapsed_on_delay: bool,
    pub spi_elapsed_step_us: u64,
    pub shutdown_requested: bool,
    pub current_limit_value: PidTorqueFluxLimit,
    pub pwm_maxcnt_value: u16,
    pub bbm_register_value: u32,
    pub dsadc_mdec_register_value: u32,
    pub excitation_voltage_value: u16,
    pub configured_physical_width: Option<u32>,
    pub mechanical_encoder_quadrature_counts_per_rev_value: u32,
    pub adc_offsets_ready_value: bool,
    pub operational_velocity_limit_value: u32,
    pub operational_voltage_limit_value: u32,
    pub tmc_config: TmcConfig,
    pub feedback_config: crate::config::FeedbackConfig,
    pub configured_torque_filter_hz: Option<u16>,
    pub configured_flux_filter_hz: Option<u16>,
    pub mechanical_encoder_position_counts_value: u32,
    pub read_position_actual_sequence: Vec<u32, 8>,
    pub read_position_actual_pos: usize,
    pub abn_decoder_count_sequence: Vec<u32, 16>,
    pub abn_decoder_count_pos: usize,
    pub pid_torque_flux_actual_sequence: Vec<u32, 128>,
    pub pid_torque_flux_actual_pos: usize,
    pub openloop_phi_sequence: Vec<u32, 128>,
    pub openloop_phi_pos: usize,
    pub openloop_phi_elapsed_step_us: u64,
    pub assert_motor_armed_false_on_write: bool,
    pub script_resistance_current_proportional_to_ud: bool,
    last_uq_ud_ext_write: u32,
    pub script_gain_floor_capture: Option<GainFloorCaptureScript>,
    pub gain_floor_step_start_us: u64,
    pub gain_floor_commanded_target: i16,
    pub safe_state_incomplete: bool,
    pub safe_state_last_pass_failed: bool,
    pub safe_state_report: Option<crate::tmc_safe_state::SafeStateRestoreReport>,
    pub safe_state_report_timestamp_ms: Option<u32>,
    pub safe_state_pass_count: u16,
    pub tmc_state: &'static crate::state::tmc::TmcChannelState,
    pub pending_config_result: Result<(), PendingConfigItem>,
    pub pending_config_set_during_drain: u8,
    pub pending_config_set_at_interlock_check: u8,
    pub verified_write_calls: usize,
    pub raw_ops_attributed_to_verified_calls: usize,
    pub stimulus_log: heapless::Vec<StimulusOp, 512>,
    pub stimulus_trigger_blocked: bool,
    pub stimulus_refuse_after: Option<usize>,
    pub stimulus_queue_free: usize,
    pub stimulus_step_timer_hz: u32,
    pub stimulus_clock_now: u32,
    pub stimulus_energized: bool,
    pub stall_threshold_units_value: u32,
    pub velocity_feedforward_gain_value: u32,
    pub feedforward_path_flags_value: u8,
    pub event_log: heapless::Vec<MockEvent, 2_048>,
}

impl MockBackend {
    /// Construct a `MockBackend` with sensible defaults for unit tests.
    pub fn new() -> Self {
        Self {
            registers: heapless::LinearMap::new(),
            config_selected_subregister: 0,
            advanced_pi_representation:
                foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12,
            spi_log: std::vec::Vec::new(),
            register_pair_reads: 0,
            capture_register_reads: 0,
            velocity_controller_output_selects: 0,
            pid_velocity_error_sum_clears: 0,
            pid_velocity_actual_reads: 0,
            pid_velocity_actual_update_interim_on_read: None,
            trace_events: Vec::new(),
            #[cfg(feature = "trace")]
            velocity_trace_observations: Vec::new(),
            #[cfg(feature = "trace")]
            velocity_sweep_trace_records: Vec::new(),
            #[cfg(feature = "trace")]
            trace_emission_order: Vec::new(),
            enable_pin: false,
            interlock_tripped: false,
            trip_interlock_on_write: None,
            delay_us_log: Vec::new(),
            fail_on_write: None,
            fail_on_write_occurrence: None,
            fail_on_read: None,
            pend_next_op: false,
            fail_writes_remaining: None,
            fail_reads_remaining: None,
            last_motor_sentinel_stored: core::cell::Cell::new(None),
            read_override: None,
            read_override_remaining: None,
            elapsed_us_value: 0,
            elapsed_us_sequence: Vec::new(),
            elapsed_us_pos: core::cell::Cell::new(0),
            advance_elapsed_on_delay: false,
            spi_elapsed_step_us: 0,
            shutdown_requested: false,
            current_limit_value: PidTorqueFluxLimit::ZERO,
            pwm_maxcnt_value: 4095,
            bbm_register_value: 0,
            dsadc_mdec_register_value: 0,
            excitation_voltage_value: 1024,
            configured_physical_width: None,
            mechanical_encoder_quadrature_counts_per_rev_value: 4000,
            adc_offsets_ready_value: true,
            operational_velocity_limit_value: 500_000,
            operational_voltage_limit_value:
                crate::state::tmc::DEFAULT_OPERATIONAL_PIDOUT_UQ_UD_LIMIT,
            tmc_config: TmcConfig::default_for_test(),
            feedback_config: crate::config::FeedbackConfig::abn_abn(),
            configured_torque_filter_hz: None,
            configured_flux_filter_hz: None,
            mechanical_encoder_position_counts_value: 0,
            read_position_actual_sequence: Vec::new(),
            read_position_actual_pos: 0,
            abn_decoder_count_sequence: Vec::new(),
            abn_decoder_count_pos: 0,
            pid_torque_flux_actual_sequence: Vec::new(),
            pid_torque_flux_actual_pos: 0,
            openloop_phi_sequence: Vec::new(),
            openloop_phi_pos: 0,
            openloop_phi_elapsed_step_us: 0,
            assert_motor_armed_false_on_write: false,
            script_resistance_current_proportional_to_ud: false,
            last_uq_ud_ext_write: 0,
            script_gain_floor_capture: None,
            gain_floor_step_start_us: 0,
            gain_floor_commanded_target: 0,
            safe_state_incomplete: false,
            safe_state_last_pass_failed: false,
            safe_state_report: None,
            safe_state_report_timestamp_ms: None,
            safe_state_pass_count: 0,
            tmc_state: crate::state::tmc::tmc_channel0(),
            pending_config_result: Ok(()),
            pending_config_set_during_drain: 0,
            pending_config_set_at_interlock_check: 0,
            verified_write_calls: 0,
            raw_ops_attributed_to_verified_calls: 0,
            stimulus_log: heapless::Vec::new(),
            stimulus_trigger_blocked: false,
            stimulus_refuse_after: None,
            stimulus_queue_free: 1_023,
            stimulus_step_timer_hz: 260_000_000,
            stimulus_clock_now: 0,
            stimulus_energized: true,
            stall_threshold_units_value: 819,
            velocity_feedforward_gain_value: 1_750,
            feedforward_path_flags_value: 0b0_0001,
            event_log: heapless::Vec::new(),
        }
    }

    /// Set sequential `ABN_DECODER_COUNT` values for read-register tests.
    pub fn set_abn_decoder_count_sequence(&mut self, values: &[u32]) {
        self.abn_decoder_count_sequence.clear();
        for &value in values {
            let _ = self.abn_decoder_count_sequence.push(value);
        }
        self.abn_decoder_count_pos = 0;
    }

    /// Seed `ABN_DECODER_COUNT` with a signed encoder-count bit pattern.
    pub fn set_abn_decoder_count(&mut self, count: i32) {
        let _ = self.registers.insert(
            foci_tmc4671::registers::ABN_DECODER_COUNT.addr(),
            count as u32,
        );
    }

    /// Set sequential timestamps for elapsed-time observations.
    pub fn set_elapsed_us_sequence(&mut self, values: &[u64]) {
        self.elapsed_us_sequence.clear();
        for &value in values {
            let _ = self.elapsed_us_sequence.push(value);
        }
        self.elapsed_us_pos.set(0);
    }

    /// Set sequential `PID_TORQUE_FLUX_ACTUAL` values for read-register tests.
    pub fn set_pid_torque_flux_actual_sequence(&mut self, values: &[u32]) {
        self.pid_torque_flux_actual_sequence.clear();
        for &value in values {
            let _ = self.pid_torque_flux_actual_sequence.push(value);
        }
        self.pid_torque_flux_actual_pos = 0;
    }

    /// Set sequential `OPENLOOP_PHI` values for read-register tests.
    pub fn set_openloop_phi_sequence(&mut self, values: &[u32], elapsed_step_us: u64) {
        self.openloop_phi_sequence.clear();
        for &value in values {
            let _ = self.openloop_phi_sequence.push(value);
        }
        self.openloop_phi_pos = 0;
        self.openloop_phi_elapsed_step_us = elapsed_step_us;
    }

    /// Write a register through the mock's register-I/O trait implementation.
    pub async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), MockSpiError> {
        <Self as CommissionRegisterIo>::write_register(self, reg, val).await
    }

    async fn write_register_verified_inner(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), MockSpiError> {
        let max_attempts = foci_tmc4671::spi::RetryPolicy::production()
            .max_attempts
            .max(1);
        for attempt in 0..max_attempts {
            let result = async {
                self.write_register(reg, val).await?;
                if core::mem::take(&mut self.pend_next_op) {
                    YieldOnce::default().await;
                }
                self.read_register(reg).await
            }
            .await;
            match result {
                Ok(actual) if actual == val => return Ok(()),
                Ok(_) => {}
                Err(error) if attempt + 1 == max_attempts => return Err(error),
                Err(_) => continue,
            }
        }
        Err(MockSpiError)
    }
}

#[derive(Default)]
struct YieldOnce {
    yielded: bool,
}

impl core::future::Future for YieldOnce {
    type Output = ();

    fn poll(
        mut self: core::pin::Pin<&mut Self>,
        cx: &mut core::task::Context<'_>,
    ) -> core::task::Poll<()> {
        if self.yielded {
            return core::task::Poll::Ready(());
        }
        self.yielded = true;
        cx.waker().wake_by_ref();
        core::task::Poll::Pending
    }
}

impl CommissionRegisterIo for MockBackend {
    type Error = MockSpiError;

    async fn read_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        let addr = reg.addr();
        self.elapsed_us_value = self
            .elapsed_us_value
            .saturating_add(self.spi_elapsed_step_us);
        self.spi_log.push(SpiOp::Read);
        if let Some((failure_addr, remaining)) = self.fail_reads_remaining.as_mut()
            && *failure_addr == addr
            && *remaining > 0
        {
            *remaining -= 1;
            return Err(MockSpiError);
        }
        if self.fail_on_read == Some(addr) {
            return Err(MockSpiError);
        }
        if let Some((override_addr, val, remaining)) = self.read_override_remaining.as_mut()
            && *override_addr == addr
            && *remaining > 0
        {
            *remaining -= 1;
            return Ok(*val);
        }
        if let Some((addr_override, val)) = self.read_override
            && addr_override == addr
        {
            return Ok(val);
        }
        if addr
            == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .data
                .addr()
            && self.config_selected_subregister
                == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr
        {
            return Ok(self.advanced_pi_representation);
        }
        if addr == foci_tmc4671::registers::PID_POSITION_ACTUAL.addr()
            && self.read_position_actual_pos < self.read_position_actual_sequence.len()
        {
            let val = self.read_position_actual_sequence[self.read_position_actual_pos];
            self.read_position_actual_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::PID_VELOCITY_ACTUAL.addr() {
            self.pid_velocity_actual_reads += 1;
            if let Some((selected_read, interim_value)) =
                self.pid_velocity_actual_update_interim_on_read
                && self.pid_velocity_actual_reads == selected_read
            {
                let _ = self.registers.insert(
                    foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
                        .data
                        .addr(),
                    interim_value,
                );
            }
        }
        if addr == foci_tmc4671::registers::ABN_DECODER_COUNT.addr()
            && self.abn_decoder_count_pos < self.abn_decoder_count_sequence.len()
        {
            let val = self.abn_decoder_count_sequence[self.abn_decoder_count_pos];
            self.abn_decoder_count_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::PID_TORQUE_FLUX_ACTUAL.addr()
            && self.pid_torque_flux_actual_pos < self.pid_torque_flux_actual_sequence.len()
        {
            let val = self.pid_torque_flux_actual_sequence[self.pid_torque_flux_actual_pos];
            self.pid_torque_flux_actual_pos += 1;
            return Ok(val);
        }
        if addr == foci_tmc4671::registers::OPENLOOP_PHI.addr()
            && self.openloop_phi_pos < self.openloop_phi_sequence.len()
        {
            let val = self.openloop_phi_sequence[self.openloop_phi_pos];
            self.openloop_phi_pos += 1;
            self.elapsed_us_value = self
                .elapsed_us_value
                .saturating_add(self.openloop_phi_elapsed_step_us);
            return Ok(val);
        }
        if let Some(script) = self.script_gain_floor_capture {
            let t = self
                .elapsed_us_value
                .saturating_sub(self.gain_floor_step_start_us) as f64;
            let target = f64::from(self.gain_floor_commanded_target);
            if addr == foci_tmc4671::registers::PID_POSITION_ACTUAL.addr() {
                let creep = if self.gain_floor_commanded_target <= script.creep_below_target {
                    script.creep_excess_accel_per_us2
                } else {
                    0.0
                };
                let accel = script.accel_raw_per_us2_per_count * target
                    + script.accel_raw_offset_per_us2
                    + creep;
                let position = (0.5 * accel * t * t) as i32;
                return Ok(position as u32);
            }
            if addr == foci_tmc4671::registers::PID_TORQUE_FLUX_ACTUAL.addr() {
                let rise = target * (1.0 - (-t / script.tau_current_us).exp());
                let ring = if t >= GAIN_FLOOR_RING_ONSET_US {
                    target
                        * (script.overshoot_ratio - 1.0)
                        * (-(t - GAIN_FLOOR_RING_ONSET_US) / GAIN_FLOOR_RING_DECAY_US).exp()
                } else {
                    0.0
                };
                let torque_actual = (rise + ring) as i16;
                let packed = foci_tmc4671::registers::PidTorqueFluxActual::new()
                    .with_pid_flux_actual(0)
                    .with_pid_torque_actual(torque_actual);
                return Ok(packed.into());
            }
        }
        if self.script_resistance_current_proportional_to_ud {
            if addr == foci_tmc4671::registers::ADC_IWY_IUX.addr() {
                let half = (self.last_uq_ud_ext_write & 0xffff) as u16 as u32;
                return Ok((half << 16) | half);
            }
            if addr == foci_tmc4671::registers::STATUS_FLAGS.addr() {
                return Ok(0);
            }
        }
        Ok(*self.registers.get(&addr).unwrap_or(&0))
    }

    async fn read_register_pair(
        &mut self,
        first: Register,
        second: Register,
    ) -> Result<(u32, u32), Self::Error> {
        self.register_pair_reads += 1;
        let first = self.read_register(first).await?;
        let second = self.read_register(second).await?;
        Ok((first, second))
    }

    async fn read_capture_register(&mut self, reg: Register) -> Result<u32, Self::Error> {
        self.capture_register_reads += 1;
        self.read_register(reg).await
    }

    async fn write_register(&mut self, reg: Register, val: u32) -> Result<(), Self::Error> {
        let addr = reg.addr();
        if addr
            == foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE
                .addr
                .addr()
            && val == u32::from(foci_tmc4671::registers::INTERIM_PIDOUT_TARGET_TORQUE.sub_addr)
        {
            self.velocity_controller_output_selects += 1;
        }
        if addr
            == foci_tmc4671::registers::PID_ERROR_PID_VELOCITY_ERROR_SUM
                .data
                .addr()
            && val == 0
        {
            self.pid_velocity_error_sum_clears += 1;
        }
        if addr == foci_tmc4671::registers::PID_TORQUE_FLUX_TARGET.addr() {
            let target = (val >> 16) as u16 as i16;
            if target != 0 {
                self.gain_floor_step_start_us = self.elapsed_us_value;
                self.gain_floor_commanded_target = target;
            }
        }
        self.elapsed_us_value = self
            .elapsed_us_value
            .saturating_add(self.spi_elapsed_step_us);
        if self.assert_motor_armed_false_on_write {
            assert!(
                !crate::state::tmc::tmc_channel0()
                    .motor_armed
                    .load(core::sync::atomic::Ordering::Relaxed),
                "dispatch SPI write ran while step processing was armed"
            );
        }
        if self.script_resistance_current_proportional_to_ud
            && addr == foci_tmc4671::registers::UQ_UD_EXT.addr()
        {
            self.last_uq_ud_ext_write = val;
        }
        let address_occurrence = self
            .spi_log
            .iter()
            .filter(|operation| matches!(operation, SpiOp::Write(logged, _) if *logged == addr))
            .count();
        self.spi_log.push(SpiOp::Write(addr, val));
        self.event_log
            .push(MockEvent::Spi(SpiOp::Write(addr, val)))
            .ok();
        if self.fail_on_write_occurrence == Some((addr, address_occurrence)) {
            return Err(MockSpiError);
        }
        if let Some((failure_addr, remaining)) = self.fail_writes_remaining.as_mut()
            && *failure_addr == addr
            && *remaining > 0
        {
            *remaining -= 1;
            return Err(MockSpiError);
        }
        if self.fail_on_write == Some(addr) {
            return Err(MockSpiError);
        }
        if addr
            == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .addr
                .addr()
        {
            self.config_selected_subregister = val as u8;
        } else if addr
            == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                .data
                .addr()
            && self.config_selected_subregister
                == foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr
        {
            self.advanced_pi_representation = val;
        }
        let _ = self.registers.insert(addr, val);
        if self.trip_interlock_on_write == Some(addr) {
            self.interlock_tripped = true;
        }
        Ok(())
    }

    async fn write_register_verified(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        self.verified_write_calls += 1;
        let before = self.spi_log.len();
        let result = self.write_register_verified_inner(reg, val).await;
        self.raw_ops_attributed_to_verified_calls += self.spi_log.len() - before;
        result
    }

    async fn read_register_verified(
        &mut self,
        reg: Register,
        expected: u32,
    ) -> Result<u32, Self::Error> {
        let max_attempts = foci_tmc4671::spi::RetryPolicy::production()
            .max_attempts
            .max(1);
        for attempt in 0..max_attempts {
            match self.read_register(reg).await {
                Ok(actual) if actual == expected => return Ok(actual),
                Ok(_) => {}
                Err(error) if attempt + 1 == max_attempts => return Err(error),
                Err(_) => continue,
            }
        }
        Err(MockSpiError)
    }
}

impl CommissionMotorControl for MockBackend {
    fn store_motor_sentinel(&self, value: u32) {
        self.last_motor_sentinel_stored.set(Some(value));
    }

    fn set_enable_pin(&mut self, state: bool) {
        self.enable_pin = state;
        self.spi_log.push(SpiOp::EnablePin(state));
    }

    fn interlock_tripped(&self) -> bool {
        self.tmc_state.config_apply_pending.fetch_or(
            self.pending_config_set_at_interlock_check,
            core::sync::atomic::Ordering::Relaxed,
        );
        self.interlock_tripped
    }

    fn record_lifecycle(&mut self, flag: LifecycleFlag, value: bool) {
        self.spi_log.push(SpiOp::Lifecycle { flag, value });
    }

    fn record_safe_state_report(
        &mut self,
        report: crate::tmc_safe_state::SafeStateRestoreReport,
        timestamp_ms: u32,
    ) {
        self.safe_state_incomplete = true;
        self.safe_state_last_pass_failed = report.failed_writes > 0;
        self.safe_state_pass_count = self.safe_state_pass_count.saturating_add(1);
        self.safe_state_report = Some(report);
        self.safe_state_report_timestamp_ms = Some(timestamp_ms);
    }
}

impl CommissionPendingConfig for MockBackend {
    async fn drain_pending_config(
        &mut self,
        _sink: PendingConfigSink<'_>,
    ) -> Result<(), PendingConfigItem> {
        use core::sync::atomic::Ordering::Relaxed;

        self.spi_log.push(SpiOp::DrainPendingConfig);
        let observed = self.tmc_state.config_apply_pending.load(Relaxed);
        self.pending_config_result?;
        self.tmc_state
            .config_apply_pending
            .fetch_and(!observed, Relaxed);
        self.tmc_state
            .config_apply_pending
            .fetch_or(self.pending_config_set_during_drain, Relaxed);
        Ok(())
    }
}

impl CommissionEncoderRead for MockBackend {
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32 {
        self.mechanical_encoder_position_counts_value
    }

    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        let addr = foci_tmc4671::registers::ABN_DECODER_COUNT.addr();
        self.spi_log.push(SpiOp::Read);
        if self.fail_on_read == Some(addr) {
            return Err(MockSpiError);
        }
        Ok(self.mechanical_encoder_position_counts_value)
    }
}

impl CommissionTiming for MockBackend {
    fn elapsed_us(&self) -> u64 {
        let position = self.elapsed_us_pos.get();
        if position < self.elapsed_us_sequence.len() {
            self.elapsed_us_pos.set(position + 1);
            return self.elapsed_us_sequence[position];
        }
        self.elapsed_us_value
    }

    fn is_shutdown_requested(&self) -> bool {
        self.shutdown_requested
    }

    async fn delay_us(&mut self, us: u32) {
        self.delay_us_log.push(us).ok();
        if self.advance_elapsed_on_delay {
            self.elapsed_us_value = self.elapsed_us_value.saturating_add(u64::from(us));
        }
    }
}

impl CommissionBoardConfig for MockBackend {
    fn current_limit(&self) -> PidTorqueFluxLimit {
        self.current_limit_value
    }

    fn pwm_maxcnt(&self) -> u16 {
        self.pwm_maxcnt_value
    }

    fn bbm_register(&self) -> u32 {
        self.bbm_register_value
    }

    fn dsadc_mdec_register(&self) -> u32 {
        self.dsadc_mdec_register_value
    }

    fn excitation_voltage(&self) -> u16 {
        self.excitation_voltage_value
    }

    fn tmc_config(&self) -> &TmcConfig {
        &self.tmc_config
    }

    fn feedback_config(&self) -> crate::config::FeedbackConfig {
        self.feedback_config
    }

    fn configured_torque_filter_hz(&self) -> Option<u16> {
        self.configured_torque_filter_hz
    }

    fn configured_flux_filter_hz(&self) -> Option<u16> {
        self.configured_flux_filter_hz
    }

    fn configured_physical_step_width(&self) -> Option<u32> {
        self.configured_physical_width
    }

    fn mechanical_encoder_quadrature_counts_per_rev(&self) -> u32 {
        self.mechanical_encoder_quadrature_counts_per_rev_value
    }

    fn motor_pole_pairs(&self) -> u16 {
        foci_tmc4671::registers::MotorTypeNPolePairs::from(
            self.last_motor_sentinel_stored.get().unwrap_or(0),
        )
        .n_pole_pairs()
    }

    fn adc_offsets_ready(&self) -> bool {
        self.adc_offsets_ready_value
    }

    fn operational_velocity_limit(&self) -> u32 {
        self.operational_velocity_limit_value
    }

    fn operational_voltage_limit(&self) -> u32 {
        self.operational_voltage_limit_value
    }

    fn stall_threshold_units(&self) -> u32 {
        self.stall_threshold_units_value
    }

    fn velocity_feedforward_gain_permille(&self) -> u32 {
        self.velocity_feedforward_gain_value
    }

    fn feedforward_path_flags(&self) -> u8 {
        self.feedforward_path_flags_value
    }
}

impl CommissionTrace for MockBackend {
    fn record_commission_trace_event(&mut self, event: crate::trace::events::CommissionTraceEvent) {
        #[cfg(feature = "trace")]
        self.trace_emission_order.push(event.event_id).ok();
        self.trace_events.push(event).ok();
    }

    #[cfg(feature = "trace")]
    fn record_velocity_trace_observation(
        &mut self,
        observation: crate::trace::stream::VelocityTraceObservation,
    ) {
        self.velocity_trace_observations.push(observation).ok();
    }

    #[cfg(feature = "trace")]
    fn record_velocity_sweep_trace_record(
        &mut self,
        record: crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord,
    ) {
        self.trace_emission_order
            .push(u16::from(record.as_bytes()[0]))
            .ok();
        self.velocity_sweep_trace_records.push(record).ok();
    }
}

/// One step-stimulus operation recorded by `MockBackend`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StimulusOp {
    SetDirection(bool),
    ResetClock(u32),
    Queue(StepMove),
    Arm,
    Stop,
    Feedforward(bool),
}

/// One event in `MockBackend::event_log`, the single ordered log
/// interleaving every SPI write and every step-stimulus operation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MockEvent {
    Spi(SpiOp),
    Stimulus(StimulusOp),
}

impl CommissionStepStimulus for MockBackend {
    fn stimulus_step_timer_hz(&self) -> u32 {
        self.stimulus_step_timer_hz
    }

    fn stimulus_step_clock_now(&self) -> u32 {
        self.stimulus_clock_now
    }

    fn stimulus_queue_free(&self) -> usize {
        self.stimulus_queue_free
    }

    fn stimulus_is_trigger_blocked(&self) -> bool {
        self.stimulus_trigger_blocked
    }

    fn stimulus_set_direction(&mut self, forward: bool) {
        let _ = self.stimulus_log.push(StimulusOp::SetDirection(forward));
        let _ = self
            .event_log
            .push(MockEvent::Stimulus(StimulusOp::SetDirection(forward)));
    }

    fn stimulus_reset_step_clock(&mut self, clock: u32) -> Result<(), StimulusError> {
        let _ = self.stimulus_log.push(StimulusOp::ResetClock(clock));
        let _ = self
            .event_log
            .push(MockEvent::Stimulus(StimulusOp::ResetClock(clock)));
        Ok(())
    }

    fn stimulus_queue_segment(&mut self, segment: StepMove) -> Result<(), StimulusError> {
        let queued = self
            .stimulus_log
            .iter()
            .filter(|op| matches!(op, StimulusOp::Queue(_)))
            .count();
        if let Some(limit) = self.stimulus_refuse_after
            && queued >= limit
        {
            return Err(StimulusError::QueueRefused {
                queue_len: queued as u32,
            });
        }
        let _ = self.stimulus_log.push(StimulusOp::Queue(segment));
        let _ = self
            .event_log
            .push(MockEvent::Stimulus(StimulusOp::Queue(segment)));
        Ok(())
    }

    fn stimulus_arm(&mut self) -> Result<(), StimulusError> {
        if !self.stimulus_energized {
            return Err(StimulusError::NotEnergized);
        }
        let _ = self.stimulus_log.push(StimulusOp::Arm);
        let _ = self.event_log.push(MockEvent::Stimulus(StimulusOp::Arm));
        Ok(())
    }

    fn stimulus_stop(&mut self) {
        let _ = self.stimulus_log.push(StimulusOp::Stop);
        let _ = self.event_log.push(MockEvent::Stimulus(StimulusOp::Stop));
    }

    fn set_stimulus_feedforward(&mut self, enabled: bool) {
        let _ = self.stimulus_log.push(StimulusOp::Feedforward(enabled));
        let _ = self
            .event_log
            .push(MockEvent::Stimulus(StimulusOp::Feedforward(enabled)));
        self.tmc_state
            .commission_stimulus_active
            .store(enabled, Relaxed);
    }

    fn stimulus_feedforward_idle(&self) -> bool {
        self.tmc_state.feedforward_surfaces_idle.load(Relaxed)
    }
}

/// Mutex for tests that mutate firmware-shared statics
/// (channel motor-armed / in-flight flags, and the gain/filter atomics in
/// `state::tmc`).
pub static SHARED_STATE_MUTEX: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

/// Build locals for a fresh dispatch state.
pub fn fresh_state_locals() -> (bool, bool, Option<crate::pid::InnerLoopGains>) {
    (false, false, None)
}

/// Build a `DispatchState` for channel 0 from the three locals a test owns.
pub fn dispatch_state<'a>(
    calibrated: &'a mut bool,
    motor_enabled: &'a mut bool,
    stored_gains: &'a mut Option<crate::pid::InnerLoopGains>,
) -> crate::commission_dispatch::DispatchState<'a> {
    crate::state::tmc::tmc_channel0().mark_pi_representation_established();
    crate::commission_dispatch::DispatchState::new(
        0,
        crate::state::tmc::tmc_channel0(),
        crate::state::stepper::stepper_channel0(),
        calibrated,
        motor_enabled,
        stored_gains,
    )
}

/// Build an accepted commissioning result with distinct gains and evidence.
pub fn accepted_commission_result() -> crate::commissioning::types::CommissionResult {
    use crate::autotune::CommissionStatus;
    use crate::commissioning::types::{CommissionResult, InnerResult};

    CommissionResult {
        inner: InnerResult {
            status: CommissionStatus::Accepted,
            r_count_milli: 1706,
            l_count_micro: 1245,
            flux_p: 100,
            flux_i: 10,
            torque_p: 200,
            torque_i: 20,
            candidate_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            candidate_axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            candidate_gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            candidate_measured_axis_split_permille: 1000,
            candidate_applied_axis_split_permille: 1000,
            candidate_axis_split_clamped: 0,
            candidate_flux_p: 100,
            candidate_flux_i: 10,
            candidate_torque_p: 200,
            candidate_torque_i: 20,
            candidate_attempt: 0,
            lambda_us: 100,
            theta_e_us: 50,
            theta_source: 0,
            bandwidth_hz: 1000,
            inductance: crate::commissioning::types::InductanceEvidence::EMPTY,
            inner_warning_flags: 0,
            timing: [crate::commissioning::types::CommissionTimingSummary::NOT_RUN; 3],
            timing_summary: 0,
            resistance_selected_count_slope_milli: 1706,
            resistance_gain_path_count_slope_milli: 1706,
            resistance_axis0_count_slope_milli: 1700,
            resistance_axis1_count_slope_milli: 1712,
            resistance_axis0_intercept_count: 0,
            resistance_axis1_intercept_count: 0,
            resistance_axis0_rmse_permille: 0,
            resistance_axis1_rmse_permille: 0,
            resistance_selected_mask_axis0: 0b1111_1000,
            resistance_selected_mask_axis1: 0b1111_1000,
            resistance_axis0_signed_count_slope_milli: 1702,
            resistance_axis1_signed_count_slope_milli: 1710,
            resistance_axis0_signed_asymmetry_permille: 0,
            resistance_axis1_signed_asymmetry_permille: 0,
            resistance_axis0_drift_permille: 0,
            resistance_axis1_drift_permille: 0,
            resistance_status_flags_or: 0,
            resistance_warning_flags: 0,
            resistance_peak_abs_current_count: 1_440,
            resistance_max_abs_steady_mean_current_count: 1_280,
            resistance_current_ceiling_count: 1_600,
            resistance_power_stage_tripped: false,
            encoder_alignment_count: 0,
            encoder_alignment_electrical_residual_counts: 0,
            encoder_alignment_stability_counts: 0,
            encoder_alignment_movement_counts: 0,
            encoder_alignment_min_movement_counts: 0,
            encoder_alignment_counts_per_electrical_rev: 0,
            adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
            current_gains_source: crate::autotune::CURRENT_GAINS_SOURCE_MEASURED,
            axis_split_source: crate::autotune::AXIS_SPLIT_SOURCE_NONE,
            gains_tier: crate::autotune::CURRENT_GAINS_TIER_MEASURED_SYMMETRIC,
            measured_axis_split_permille: 1000,
            applied_axis_split_permille: 1000,
            axis_split_clamped: 0,
            current_validation_axes: crate::autotune::CURRENT_VALIDATION_AXIS_TORQUE,
            retry_budget_exhausted: 0,
            current_loop_failure_reason: crate::autotune::CURRENT_LOOP_FAILURE_NONE,
            closed_loop_activation_status: crate::autotune::CLOSED_LOOP_ACTIVATION_STATUS_NOT_RUN,
            closed_loop_activation_position_1: 0,
            closed_loop_activation_position_2: 0,
            closed_loop_activation_drift_count: 0,
            closed_loop_activation_threshold_count: 0,
            closed_loop_activation_runaway: 0,
            current_hold_status: crate::autotune::HOLD_GATE_STATUS_NOT_RUN,
            current_hold_warning_flags: 0,
            current_hold_elapsed_us: 0,
            current_hold_requested_sample_period_us: crate::autotune::HOLD_GATE_SAMPLE_PERIOD_US
                as u16,
            current_hold_sample_count: 0,
            current_hold_position_span_count: 0,
            current_hold_position_drift_count: 0,
            current_hold_torque_mean_count: 0,
            current_hold_torque_rms_count: 0,
            current_hold_torque_peak_to_peak_count: 0,
            current_hold_torque_crossing_count: 0,
            current_hold_flux_mean_count: 0,
            current_hold_flux_rms_count: 0,
            current_hold_flux_peak_to_peak_count: 0,
            current_hold_flux_crossing_count: 0,
            current_hold_status_flags_or: 0,
            current_hold_actionable_status_count: 0,
            current_validation_step_amplitude: 128,
            current_validation_flux_step_amplitude: 128,
            current_validation_torque_step_amplitude: 128,
            current_validation_pidout_limit: 2400,
            current_validation_current_limited: 422,
            current_validation_voltage_limited: 128,
            current_validation_max_p: 3474,
            current_settled_validation_count: 0,
            current_settled_validation:
                [crate::commissioning::types::CurrentValidationSettledSampleEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_SETTLED_MAX_SAMPLES],
            current_flux_validation_count: 0,
            current_flux_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
            current_torque_validation_count: 0,
            current_torque_validation:
                [crate::commissioning::types::CurrentValidationAxisEvidence::EMPTY;
                    crate::autotune::CURRENT_VALIDATION_MAX_SAMPLES_PER_AXIS],
        },
        fallback_velocity_p: 1152,
        fallback_velocity_i: 0,
        fallback_position_p: 640,
        fallback_position_i: 0,
        fallback_velocity_limit: 500_000,
        velocity_filter_hz: 0,
        torque_filter_hz: 0,
        position_filter_hz: 0,
        flux_filter_hz: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mock_backend_satisfies_split_backend_traits() {
        fn assert_register_io<T: crate::commissioning::backend::CommissionRegisterIo>() {}
        fn assert_motor_control<T: crate::commissioning::backend::CommissionMotorControl>() {}
        fn assert_encoder_read<T: crate::commissioning::backend::CommissionEncoderRead>() {}
        fn assert_timing<T: crate::commissioning::backend::CommissionTiming>() {}
        fn assert_board_config<T: crate::commissioning::backend::CommissionBoardConfig>() {}
        fn assert_trace<T: crate::commissioning::backend::CommissionTrace>() {}
        fn assert_backend<T: crate::commissioning::backend::CommissioningBackend>() {}

        assert_register_io::<MockBackend>();
        assert_motor_control::<MockBackend>();
        assert_encoder_read::<MockBackend>();
        assert_timing::<MockBackend>();
        assert_board_config::<MockBackend>();
        assert_trace::<MockBackend>();
        assert_backend::<MockBackend>();
    }

    #[test]
    fn timed_register_helpers_measure_existing_awaits() {
        pollster::block_on(async {
            use crate::commissioning::backend::{timed_read_register, timed_write_register};
            use foci_tmc4671::registers::STATUS_FLAGS;

            let mut backend = MockBackend::new();
            backend.elapsed_us_value = 100;
            backend.spi_elapsed_step_us = 7;
            let _ = backend.registers.insert(STATUS_FLAGS.addr(), 0x1234_5678);

            let read = timed_read_register(&mut backend, STATUS_FLAGS).await;
            assert_eq!(read.result, Ok(0x1234_5678));
            assert_eq!(read.wall_us, 7);
            assert!(!read.overflowed);

            let write = timed_write_register(&mut backend, STATUS_FLAGS, 0).await;
            assert_eq!(write.result, Ok(()));
            assert_eq!(write.wall_us, 7);
            assert!(!write.overflowed);
        });
    }

    #[test]
    fn timed_register_helpers_preserve_backend_errors_without_retry() {
        pollster::block_on(async {
            use crate::commissioning::backend::{timed_read_register, timed_write_register};
            use foci_tmc4671::registers::STATUS_FLAGS;

            let mut read_backend = MockBackend::new();
            read_backend.elapsed_us_value = 100;
            read_backend.spi_elapsed_step_us = 7;
            read_backend.fail_on_read = Some(STATUS_FLAGS.addr());

            let read = timed_read_register(&mut read_backend, STATUS_FLAGS).await;
            assert_eq!(read.result, Err(MockSpiError));
            assert_eq!(read.wall_us, 7);
            assert!(!read.overflowed);
            assert_eq!(read_backend.spi_log.len(), 1);
            assert!(matches!(read_backend.spi_log[0], SpiOp::Read));

            let mut write_backend = MockBackend::new();
            write_backend.elapsed_us_value = 200;
            write_backend.spi_elapsed_step_us = 11;
            write_backend.fail_on_write = Some(STATUS_FLAGS.addr());

            let write = timed_write_register(&mut write_backend, STATUS_FLAGS, 0x1234_5678).await;
            assert_eq!(write.result, Err(MockSpiError));
            assert_eq!(write.wall_us, 11);
            assert!(!write.overflowed);
            assert_eq!(write_backend.spi_log.len(), 1);
            assert!(matches!(
                write_backend.spi_log[0],
                SpiOp::Write(addr, 0x1234_5678) if addr == STATUS_FLAGS.addr()
            ));
        });
    }

    #[test]
    fn mock_store_motor_sentinel_writes_field() {
        let backend = MockBackend::new();
        backend.store_motor_sentinel(0x0002_0032);
        assert_eq!(backend.last_motor_sentinel_stored.get(), Some(0x0002_0032));
    }

    #[test]
    fn mock_write_register_verified_succeeds_on_match() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;
            assert!(result.is_ok());
            assert_eq!(
                backend
                    .registers
                    .get(&foci_tmc4671::registers::PID_POSITION_TARGET.addr())
                    .copied(),
                Some(42),
            );
        });
    }

    #[test]
    fn mock_write_register_verified_returns_error_on_mismatch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override =
                Some((foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 999));
            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;
            assert!(result.is_err(), "expected verify mismatch");
            assert_eq!(backend.spi_log.len(), 6);
        });
    }

    #[test]
    fn mock_write_register_verified_recovers_from_transient_write_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_writes_remaining =
                Some((foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 1));

            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert_eq!(result, Ok(()));
            assert_eq!(backend.spi_log.len(), 3);
        });
    }

    #[test]
    fn mock_write_register_verified_recovers_from_transient_mismatch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.read_override_remaining =
                Some((foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 999, 1));

            let result = backend
                .write_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert_eq!(result, Ok(()));
            assert_eq!(backend.spi_log.len(), 4);
        });
    }

    #[test]
    fn mock_read_register_verified_succeeds_on_match() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 42);

            let result = backend
                .read_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert_eq!(result, Ok(42));
        });
    }

    #[test]
    fn mock_read_register_verified_returns_error_on_mismatch() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let _ = backend
                .registers
                .insert(foci_tmc4671::registers::PID_POSITION_TARGET.addr(), 999);

            let result = backend
                .read_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert!(result.is_err(), "expected verify mismatch");
            assert_eq!(backend.spi_log.len(), 3);
        });
    }

    #[test]
    fn mock_read_register_verified_recovers_from_transient_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            let addr = foci_tmc4671::registers::PID_POSITION_TARGET.addr();
            let _ = backend.registers.insert(addr, 42);
            backend.fail_reads_remaining = Some((addr, 1));

            let result = backend
                .read_register_verified(foci_tmc4671::registers::PID_POSITION_TARGET, 42)
                .await;

            assert_eq!(result, Ok(42));
            assert_eq!(backend.spi_log.len(), 2);
        });
    }

    #[test]
    fn mock_try_read_mechanical_encoder_position_counts_propagates_read_failure() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.fail_on_read = Some(foci_tmc4671::registers::ABN_DECODER_COUNT.addr());

            let result = backend.try_read_mechanical_encoder_position_counts().await;

            assert_eq!(result, Err(MockSpiError));
        });
    }

    #[test]
    fn mock_mechanical_encoder_accessors_report_explicit_units() {
        pollster::block_on(async {
            let mut backend = MockBackend::new();
            backend.mechanical_encoder_position_counts_value = 1_234;
            backend.mechanical_encoder_quadrature_counts_per_rev_value = 8_000;

            assert_eq!(
                backend.read_mechanical_encoder_position_counts().await,
                1_234
            );
            assert_eq!(
                backend.mechanical_encoder_quadrature_counts_per_rev(),
                8_000
            );
        });
    }

    #[test]
    fn mock_records_the_stimulus_sequence_and_refuses_after_the_configured_count() {
        let mut backend = MockBackend::new();
        backend.stimulus_refuse_after = Some(2);

        backend.stimulus_set_direction(true);
        backend.stimulus_reset_step_clock(100).unwrap();
        backend
            .stimulus_queue_segment(StepMove::new(10, 1, 0))
            .unwrap();
        backend
            .stimulus_queue_segment(StepMove::new(10, 2, 0))
            .unwrap();
        let refused = backend.stimulus_queue_segment(StepMove::new(10, 3, 0));
        backend.stimulus_stop();

        assert_eq!(refused, Err(StimulusError::QueueRefused { queue_len: 2 }));
        assert_eq!(
            backend.stimulus_log.as_slice(),
            &[
                StimulusOp::SetDirection(true),
                StimulusOp::ResetClock(100),
                StimulusOp::Queue(StepMove::new(10, 1, 0)),
                StimulusOp::Queue(StepMove::new(10, 2, 0)),
                StimulusOp::Stop,
            ]
        );
    }

    #[test]
    fn mock_reports_the_configured_free_capacity() {
        let mut backend = MockBackend::new();
        assert_eq!(backend.stimulus_queue_free(), 1_023);
        backend.stimulus_queue_free = 3;
        assert_eq!(backend.stimulus_queue_free(), 3);
    }

    #[test]
    fn mock_arm_requires_energized_and_reports_trigger_block() {
        let mut backend = MockBackend::new();
        backend.stimulus_energized = false;
        assert_eq!(backend.stimulus_arm(), Err(StimulusError::NotEnergized));
        backend.stimulus_energized = true;
        backend.stimulus_trigger_blocked = true;
        assert!(backend.stimulus_is_trigger_blocked());
        assert_eq!(backend.stimulus_arm(), Ok(()));
        assert_eq!(backend.stimulus_log.last(), Some(&StimulusOp::Arm));
    }
}
