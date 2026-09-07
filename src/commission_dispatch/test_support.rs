//! Test infrastructure for commission_dispatch: mock CommissioningBackend
//! with injectable SPI failures, register-write log, delay log, and
//! enable-pin tracking.

use crate::commissioning::backend::{
    CommissionBoardConfig, CommissionEncoderRead, CommissionMotorControl, CommissionRegisterIo,
    CommissionTiming, CommissionTrace,
};
use crate::config::TmcConfig;
use crate::tmc_control::lifecycle::LifecycleFlag;
use foci_tmc4671::current_limit::PidTorqueFluxLimit;
use foci_tmc4671::registers::Register;
use heapless::Vec;

/// Synthetic SPI error for test injection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct MockSpiError;

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
        current_ringing: false,
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
    }
}

/// A genuine breakaway confirmation, minted through the real
/// `plan_band_confirmation` -> `evaluate_band_confirmation` ->
/// `bind_breakaway_stage_c_authority` chain a live campaign uses, plus the
/// intermediate values a Stage-C plan builder needs alongside the token.
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
    use crate::commissioning::outer::velocity::integral_plan::bind_breakaway_stage_c_authority;
    use crate::commissioning::outer::velocity::sweep_consensus::{
        CONSENSUS_OBSERVATIONS_PER_DIRECTION, DirectionalConsensus, DirectionalConsensusClass,
    };
    use crate::commissioning::outer::velocity::sweep_evidence::VelocityObservationClass;
    use foci_tmc4671::pid::PidPGain;

    let candidate_p = PidPGain::new(512).expect("raw gain within PidPGain range");
    let confirmation_plan = plan_band_confirmation(0x1111_2222_3333_4444, candidate_p)
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
        evaluate_band_confirmation(candidate_p, forward, reverse).expect("accepted confirmation");
    let token = bind_breakaway_stage_c_authority(
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
/// [`breakaway_confirmation_for_test`] for the fuller bundle a Stage-C plan
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
/// cross-module lifecycle tests. Unlike a breakaway-campaign or Stage-C
/// engine, this phase reports no active velocity workflow (see
/// `CommissioningEngine::velocity_operational_torque_limit`), so it is the
/// live-path witness for dispatch behavior gated on that distinction.
pub fn robustness_reversal_engine() -> crate::commissioning::CommissioningEngine {
    use crate::commission_dispatch::robustness::RobustnessPreflight;
    use crate::commissioning::outer::velocity::integral_reproduction::{
        AcceptanceMatrixProgress, AcceptancePoint, StageCPlanIdentity,
    };

    let point = AcceptancePoint {
        identity: StageCPlanIdentity {
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
        progress: AcceptanceMatrixProgress::ReadyAscending,
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
    /// `set_enable_pin` call, recorded in the same ordered log as register
    /// writes so tests can assert relative ordering (e.g. a register must be
    /// written before the enable pin is asserted).
    EnablePin(bool),
    /// A motor lifecycle flag transition, recorded at its exact mutation
    /// site (see `CommissionMotorControl::record_lifecycle`) so tests can
    /// assert its relative order against SPI writes and `EnablePin` calls
    /// in the same interleaved log.
    Lifecycle {
        flag: LifecycleFlag,
        value: bool,
    },
}

/// Mock implementation of `CommissioningBackend` for host-side unit tests.
///
/// Stores register state in a linear map keyed by register address, records
/// every SPI operation, and supports per-address failure injection for both
/// reads and writes.
///
/// Do not implement the `Commission*Access` helper traits for this type.
/// `MockBackend` intentionally keeps explicit capability impls so tests can
/// inject register-map mismatches, SPI failures, and delay observations without
/// colliding with production blanket impls.
pub struct MockBackend {
    /// Current register values, keyed by TMC4671 address byte.
    pub registers: heapless::LinearMap<u8, u32, 64>,
    /// Last CONFIG subregister selected through address register 0x4E.
    config_selected_subregister: u8,
    /// Value retained specifically for CONFIG subregister 62.
    advanced_pi_representation: u32,
    /// Ordered log of every SPI read/write issued during the test.
    pub spi_log: Vec<SpiOp, 8_192>,
    /// Number of paired register-read operations issued during the test.
    pub register_pair_reads: usize,
    /// Number of single-register capture reads issued during the test.
    pub capture_register_reads: usize,
    /// Number of velocity-controller torque-output selector writes.
    pub velocity_controller_output_selects: usize,
    /// Number of writes clearing the velocity-integrator error sum.
    pub pid_velocity_error_sum_clears: usize,
    /// Number of direct velocity-feedback reads.
    pub pid_velocity_actual_reads: usize,
    /// Update the interim torque output on a selected velocity-feedback read.
    pub pid_velocity_actual_update_interim_on_read: Option<(usize, u32)>,
    /// Ordered trace events recorded by the commissioning engine.
    pub trace_events: Vec<crate::trace::events::CommissionTraceEvent, 64>,
    /// Characterization-only raw velocity observations.
    #[cfg(feature = "trace")]
    pub velocity_trace_observations: Vec<crate::trace::stream::VelocityTraceObservation, 2_048>,
    /// Canonical Stage B records emitted into the trace stream.
    #[cfg(feature = "trace")]
    pub velocity_sweep_trace_records:
        Vec<crate::ankyra::velocity_sweep_encoding::VelocitySweepCanonicalRecord, 128>,
    /// Interleaved canonical-record kinds and event IDs for emission-order assertions.
    #[cfg(feature = "trace")]
    pub trace_emission_order: Vec<u16, 256>,
    /// Current state of the TMC4671 enable pin.
    pub enable_pin: bool,
    /// Log of every `delay_us` call, in order.
    pub delay_us_log: Vec<u32, 32>,
    /// If `Some(addr)`, `write_register` returns `Err(MockSpiError)` for that address.
    pub fail_on_write: Option<u8>,
    /// Fail the zero-based write occurrence for one register address.
    pub fail_on_write_occurrence: Option<(u8, usize)>,
    /// If `Some(addr)`, `read_register` returns `Err(MockSpiError)` for that address.
    pub fail_on_read: Option<u8>,
    /// Suspend the next verified register write exactly once, after its write
    /// has landed and before its verifying read-back, so a manually polled test
    /// can drop a caller while a real backend operation is genuinely half done.
    pub pend_next_op: bool,
    /// Transient write failures as `(address, remaining failures)`.
    pub fail_writes_remaining: Option<(u8, u8)>,
    /// Transient read failures as `(address, remaining failures)`.
    pub fail_reads_remaining: Option<(u8, u8)>,
    /// Last value passed to `store_motor_sentinel`. None until first call.
    pub last_motor_sentinel_stored: core::cell::Cell<Option<u32>>,
    /// When Some((addr, value)), `read_register(reg)` for that addr returns
    /// `value` instead of consulting `registers`. Used by verify tests to
    /// inject a mismatch.
    pub read_override: Option<(u8, u32)>,
    /// Transient read mismatches as `(address, value, remaining reads)`.
    pub read_override_remaining: Option<(u8, u32, u8)>,
    /// Value returned by `elapsed_us()`.
    pub elapsed_us_value: u64,
    /// Sequential values returned by `elapsed_us()` before the fallback value.
    elapsed_us_sequence: Vec<u64, 16>,
    /// Current index into `elapsed_us_sequence`.
    elapsed_us_pos: core::cell::Cell<usize>,
    /// Whether cooperative test delays advance the mock monotonic.
    pub advance_elapsed_on_delay: bool,
    /// Elapsed-time increment applied by each mock register read or write.
    pub spi_elapsed_step_us: u64,
    /// Value returned by `is_shutdown_requested()`.
    pub shutdown_requested: bool,
    /// Value returned by `current_limit()`.
    pub current_limit_value: PidTorqueFluxLimit,
    /// Value returned by `pwm_maxcnt()`.
    pub pwm_maxcnt_value: u16,
    /// Value returned by `bbm_register()`.
    pub bbm_register_value: u32,
    /// Value returned by `dsadc_mdec_register()`.
    pub dsadc_mdec_register_value: u32,
    /// Value returned by `excitation_voltage()`.
    pub excitation_voltage_value: u16,
    /// Value returned by `configured_physical_step_width()`.
    pub configured_physical_width: Option<u32>,
    /// Encoder quadrature counts per mechanical revolution.
    pub mechanical_encoder_quadrature_counts_per_rev_value: u32,
    /// Value returned by `adc_offsets_ready()`.
    pub adc_offsets_ready_value: bool,
    /// Value returned by `operational_velocity_limit()`.
    pub operational_velocity_limit_value: u32,
    /// Value returned by `operational_voltage_limit()`.
    pub operational_voltage_limit_value: u32,
    /// Value returned by `tmc_config()`.
    pub tmc_config: TmcConfig,
    /// Value returned by `feedback_config()`.
    pub feedback_config: crate::config::FeedbackConfig,
    /// Optional torque-filter override returned by `configured_torque_filter_hz()`.
    pub configured_torque_filter_hz: Option<u16>,
    /// Optional flux-filter override returned by `configured_flux_filter_hz()`.
    pub configured_flux_filter_hz: Option<u16>,
    /// Encoder position returned by the sampled-count accessor.
    pub mechanical_encoder_position_counts_value: u32,
    /// Sequential values to return for consecutive `PID_POSITION_ACTUAL` reads.
    ///
    /// When non-empty, each `read_register(PID_POSITION_ACTUAL)` call pops and
    /// returns the front value instead of consulting `registers`. Used by
    /// stability-check tests that need two reads to return different values.
    pub read_position_actual_sequence: Vec<u32, 8>,
    /// Current index into `read_position_actual_sequence`.
    pub read_position_actual_pos: usize,
    /// Sequential values to return for consecutive `ABN_DECODER_COUNT` reads.
    pub abn_decoder_count_sequence: Vec<u32, 16>,
    /// Current index into `abn_decoder_count_sequence`.
    pub abn_decoder_count_pos: usize,
    /// Sequential values to return for consecutive `PID_TORQUE_FLUX_ACTUAL` reads.
    pub pid_torque_flux_actual_sequence: Vec<u32, 128>,
    /// Current index into `pid_torque_flux_actual_sequence`.
    pub pid_torque_flux_actual_pos: usize,
    /// Sequential values to return for consecutive `OPENLOOP_PHI` reads.
    pub openloop_phi_sequence: Vec<u32, 128>,
    /// Current index into `openloop_phi_sequence`.
    pub openloop_phi_pos: usize,
    /// Elapsed-time increment applied for each scripted `OPENLOOP_PHI` read.
    pub openloop_phi_elapsed_step_us: u64,
    /// If true, every SPI write asserts that step processing has already been gated off.
    pub assert_motor_armed_false_on_write: bool,
    /// If true, `read_register(ADC_IWY_IUX)` reports a phase current
    /// proportional 1:1 to the most recent `UQ_UD_EXT` write (packed into
    /// both axis halves), and `read_register(STATUS_FLAGS)` reports `0`.
    /// Used by the shared resistance-identification runner's integration
    /// tests to script a clean linear current response without needing a
    /// dedicated fake backend.
    pub script_resistance_current_proportional_to_ud: bool,
    /// Last value written to `UQ_UD_EXT`, tracked only when
    /// `script_resistance_current_proportional_to_ud` is set.
    last_uq_ud_ext_write: u32,
}

impl MockBackend {
    /// Construct a `MockBackend` with sensible defaults for unit tests.
    pub fn new() -> Self {
        Self {
            registers: heapless::LinearMap::new(),
            config_selected_subregister: 0,
            advanced_pi_representation:
                foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12,
            spi_log: Vec::new(),
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

    /// True once the outer-tune deploy path has entered position mode.
    /// `MODE_RAMP_MODE_MOTION` is written `0x8000_0003` only by the
    /// position-mode restore that publishes an accepted tune; the disarm
    /// safe-state path writes the same register but always with `0`, and a
    /// non-passing robustness/candidate terminal's baseline restore never
    /// touches this register at all, so the paired address+value is an
    /// unambiguous deploy marker.
    pub fn deployed_outer_gains(&self) -> bool {
        let addr = foci_tmc4671::registers::MODE_RAMP_MODE_MOTION.addr();
        self.spi_log
            .iter()
            .any(|op| matches!(op, SpiOp::Write(written, val) if *written == addr && *val == 0x8000_0003))
    }
}

/// Returns `Pending` once, then `Ready`. Lets a test drive a caller's future to
/// a real suspension point instead of running it straight to completion.
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
        self.spi_log.push(SpiOp::Read).ok();
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
        if self.script_resistance_current_proportional_to_ud {
            if addr == foci_tmc4671::registers::ADC_IWY_IUX.addr() {
                // Pack the same proportional current into both axis halves so
                // either electrical axis channel reads the intended value.
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
        self.spi_log.push(SpiOp::Write(addr, val)).ok();
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
        Ok(())
    }

    async fn write_register_verified(
        &mut self,
        reg: Register,
        val: u32,
    ) -> Result<(), Self::Error> {
        let max_attempts = foci_tmc4671::spi::RetryPolicy::production()
            .max_attempts
            .max(1);
        for attempt in 0..max_attempts {
            let result = async {
                self.write_register(reg, val).await?;
                // Suspends *between* the register write and its verifying
                // read-back, so a test that drops here interrupts a
                // half-completed verified write rather than gating entry to
                // one. Deliberately not on `write_register`/`read_register`
                // themselves: the engine-step future contains those, and the
                // extra suspension state would inflate the F407 stack-budget
                // gates in `commissioning::tests::size_regression_gates`.
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
        self.spi_log.push(SpiOp::EnablePin(state)).ok();
    }

    fn record_lifecycle(&mut self, flag: LifecycleFlag, value: bool) {
        self.spi_log.push(SpiOp::Lifecycle { flag, value }).ok();
    }
}

impl CommissionEncoderRead for MockBackend {
    async fn read_mechanical_encoder_position_counts(&mut self) -> u32 {
        self.mechanical_encoder_position_counts_value
    }

    async fn try_read_mechanical_encoder_position_counts(&mut self) -> Result<u32, Self::Error> {
        let addr = foci_tmc4671::registers::ABN_DECODER_COUNT.addr();
        self.spi_log.push(SpiOp::Read).ok();
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

/// Mutex for tests that mutate firmware-shared statics
/// (channel motor-armed / in-flight flags, and the gain/filter atomics in
/// `state::tmc`).
///
/// Tests acquire this guard before any read-modify-write sequence on a shared
/// static, and hold it for the duration of the assertion phase. Cargo runs
/// tests in parallel by default; without serialization, dirty static state
/// from one test would flake another running concurrently.
///
/// Usage:
/// ```ignore
/// let _guard = SHARED_STATE_MUTEX.lock().await;
/// crate::state::tmc::tmc_channel0().motor_armed.store(false, Ordering::Relaxed);
/// // ... test body ...
/// ```
///
/// Use Tokio's async mutex so async tests can intentionally hold the guard
/// across awaited firmware calls without blocking an executor thread.
pub static SHARED_STATE_MUTEX: tokio::sync::Mutex<()> = tokio::sync::Mutex::const_new(());

/// Build locals for a fresh dispatch state.
///
/// Returns `(calibrated, motor_enabled, stored_gains)` with zero/false/None
/// defaults. Tests borrow these as `&mut` to construct a `DispatchState`
/// (added in Task 11+) per call site.
pub fn fresh_state_locals() -> (bool, bool, Option<crate::pid::InnerLoopGains>) {
    (false, false, None)
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
            ringing_count: 0,
            bandwidth_hz: 1000,
            tau_e_us: 730,
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
}
