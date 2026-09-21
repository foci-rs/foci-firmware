use super::*;
use crate::commission_dispatch::test_support::breakaway_campaign_engine;
use crate::commission_dispatch::test_support::{
    MockBackend, SpiOp, accepted_commission_result, current_limit,
};
use crate::commission_dispatch::{PostActions, ReplyPayload, StepperPostAction};
use crate::commissioning::CommissioningEngine;
use crate::config::TmcConfig;
use crate::pid::InnerLoopGains;
use crate::tmc::{TmcRequest, TmcRequestKind};
use crate::tmc_control::dispatch::{
    RegisterRequestError, SharedTmcDispatch, apply_shared_tmc_request,
};
use crate::tmc_control::helpers::{
    LiveGainWriteStatus, apply_live_inner_gains, apply_live_outer_gains, apply_motion_scale,
    refresh_velocity_scale_from_state,
};
use crate::tmc_control::request::live_gain_failure_outcome;
use crate::tmc_control::state::{
    HOMING_CLAMP_MAX_RETRIES, HomingClockWindow, homing_clamp_due, homing_clamp_lead_ticks,
};
use core::sync::atomic::AtomicBool;
use core::sync::atomic::Ordering::Relaxed;

use crate::tmc_control::test_spi::RuntimeMockSpi;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use foci_tmc4671::registers::{
    ABN_DECODER_MODE, ABN_DECODER_PHI_E_PHI_M_OFFSET, ABN_DECODER_PPR, CONFIG_BIQUAD_T_ENABLE,
    PID_ERROR_PID_FLUX_ERROR_SUM, PID_ERROR_PID_POSITION_ERROR_SUM, PID_ERROR_PID_TORQUE_ERROR_SUM,
    PID_ERROR_PID_VELOCITY_ERROR_SUM, PID_FLUX_P_FLUX_I, PID_POSITION_ACTUAL,
    PID_POSITION_P_POSITION_I, PID_POSITION_TARGET, PID_TORQUE_FLUX_LIMITS, PID_TORQUE_FLUX_OFFSET,
    PID_TORQUE_P_TORQUE_I, PID_VELOCITY_OFFSET, PID_VELOCITY_P_VELOCITY_I, PIDOUT_UQ_UD_LIMITS,
    STATUS_FLAGS, STEP_WIDTH,
};
use foci_tmc4671::spi::Tmc4671Spi;

fn assert_no_actions(actions: &BoardActions) {
    assert_eq!(actions.len(), 0, "expected no board actions");
}

fn assert_single_stop_now(actions: &BoardActions) {
    assert_eq!(actions.len(), 1, "expected one StopNow action");
    assert_stop_now_at(actions, 0);
}

fn assert_stop_now_at(actions: &BoardActions, index: usize) {
    match &actions[index] {
        BoardAction::Stepper(StepperAction::StopNow) => {}
        BoardAction::Stepper(_) => panic!("expected StopNow stepper action at index {index}"),
        BoardAction::Shutdown(_) => panic!("expected StopNow, got Shutdown at index {index}"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected StopNow, got SignalEmergencyDisable at index {index}")
        }
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected StopNow, got DelegatedDiagnostic at index {index}")
        }
    }
}

fn assert_shutdown_at(actions: &BoardActions, index: usize, reason: ShutdownReason) {
    match &actions[index] {
        BoardAction::Shutdown(actual) => assert_eq!(*actual, reason),
        BoardAction::Stepper(_) => panic!("expected Shutdown at index {index}, got Stepper"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected Shutdown at index {index}, got SignalEmergencyDisable")
        }
        BoardAction::DelegatedDiagnostic(_) => {
            panic!("expected Shutdown at index {index}, got DelegatedDiagnostic")
        }
    }
}

fn assert_single_shutdown(actions: &BoardActions, reason: ShutdownReason) {
    assert_eq!(actions.len(), 1, "expected one Shutdown action");
    assert_shutdown_at(actions, 0, reason);
}

fn assert_stop_then_shutdown(actions: &BoardActions, reason: ShutdownReason) {
    assert_eq!(actions.len(), 2, "expected StopNow then Shutdown actions");
    assert_stop_now_at(actions, 0);
    assert_shutdown_at(actions, 1, reason);
}

fn assert_single_delegated_request(actions: &BoardActions, expected: &TmcRequest) {
    assert_eq!(actions.len(), 1, "expected one DelegatedDiagnostic action");
    match &actions[0] {
        BoardAction::DelegatedDiagnostic(delegated) => {
            assert_eq!(&delegated.request, expected);
        }
        BoardAction::Stepper(_) => panic!("expected DelegatedDiagnostic, got Stepper"),
        BoardAction::Shutdown(_) => panic!("expected DelegatedDiagnostic, got Shutdown"),
        BoardAction::SignalEmergencyDisable => {
            panic!("expected DelegatedDiagnostic, got SignalEmergencyDisable")
        }
    }
}

#[test]
fn runtime_starts_disarmed_and_uncalibrated() {
    let runtime = TmcControlRuntime::new_for_channel(0);

    assert_eq!(runtime.channel(), 0);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert_eq!(runtime.stored_gains(), None);
    assert_eq!(runtime.pending_commission_kind(), None);
}

#[test]
fn channel_runtime_size_stays_bounded() {
    // The runtime embeds the `CommissioningEngine`, so it tracks that engine's
    // size plus the runtime's own fixed overhead.
    // 5_872 -> 5_856: removes the `combined_workflow` bool
    // from the embedded Stage-C execution plan.
    // Both builds grow because the runtime now caches the planner mechanical-RPM
    // ratio alongside the position-per-PWM ratio; the trace build absorbs part of
    // it into existing padding.
    // 5_872 -> 5_888: the trace-only runtime state gains a per-sample velocity
    // feedforward clamp-engaged flag.
    // 5_888 -> 7_552 / 5_856 -> 7_520: the runtime now embeds the poll-driven
    // terminal restore machine. Its owned terminal payload used to live in the
    // `tmc_control` task future for the duration of the awaited restore, so
    // this is a move out of that future's frame, not new state.
    // 7_552 -> 7_568 / 7_520 -> 7_536: the embedded `OuterSafetyLimits` gains
    // a `u32` peak local-window excursion field (see
    // `commissioning::tests::size_regression_gates`).
    // 7_568 -> 7_584 / 7_536 -> 7_552: the runtime now owns the homing
    // `Option<StallDetector>` and its state-1/state-3 retry counters.
    // The three gain-restore cursor states (`SelftestRestoreGains`,
    // `InnerFailedRestoreGains`, `CalibrateRestoreGains`) now carry an
    // `Option<CalibrateGainsRestoreStep>` so each polls one register
    // operation at a time instead of one shot; the runtime grows by that
    // step state's size.
    // 7_600 -> 7_984 / 7_568 -> 7_952: `CalibrateApplyFilters` and
    // `OuterDoneApplyFilters` now carry an `Option<FilterCutoffsStep>` so
    // each applies one filter-cutoff register write per poll instead of one
    // shot; the runtime grows by that step state's size.
    // 7_984 -> 8_000 / 7_952 -> 7_968: `RobustnessRestoreBaseline` and
    // `VelocityStagedRestoreBaseline` now carry an
    // `Option<restore::EntrySnapshotRestoreStep>`, whose `Filters` variant
    // embeds a `FilterCutoffsStep`; it becomes `RestoreCursor`'s largest
    // variant, so the runtime grows by the resulting size delta.
    // 8_000 -> 8_144 / 7_968 -> 8_112: the restore machine's per-terminal
    // state carries an `Option<RobustnessEvidence>` so a robustness pass, which
    // deploys through `OuterDone` and never emits its terminal, can still put
    // the per-direction cycle evidence on the wire ahead of the tune result.
    #[cfg(feature = "trace")]
    assert_eq!(core::mem::size_of::<TmcControlRuntime>(), 8_144);
    // 3_920 -> 5_824: un-gates `OuterPhase::VelocityIntegral`
    // and `OuterPhase::BreakawayCampaign` into the default build, so the
    // embedded `CommissioningEngine` is now the same largest-variant size as
    // the `trace` build (see `commissioning::tests::size_regression_gates`);
    // only the surrounding runtime's own trace-only fields still differ.
    #[cfg(not(feature = "trace"))]
    assert_eq!(core::mem::size_of::<TmcControlRuntime>(), 8_112);
}

#[test]
fn runtime_stores_inner_gains_without_spi_side_effects() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: crate::pid::p_gain(11),
        flux_i: crate::pid::i_gain(12),
        torque_p: crate::pid::p_gain(13),
        torque_i: crate::pid::i_gain(14),
    };

    runtime.set_stored_gains(gains);

    assert_eq!(runtime.stored_gains(), Some(gains));
}

#[test]
fn refresh_velocity_scale_caches_exact_planner_scale() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc_state = crate::state::tmc::tmc_channel0();
    let stepper_state = crate::state::stepper::stepper_channel0();
    tmc_state.reset();
    stepper_state.reset();
    let scale = tmc_state
        .configure_motion_scale(stepper_state, 1000, 3200)
        .unwrap();

    refresh_velocity_scale_from_state(&mut runtime, tmc_state, 84_000_000, 25_000);

    assert_eq!(
        runtime.planner_velocity_scale(),
        scale.planner_velocity_scale(84_000_000, 25_000)
    );
    tmc_state.reset();
    stepper_state.reset();
}

#[test]
fn apply_live_inner_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = InnerLoopGains {
            flux_p: crate::pid::p_gain(1),
            flux_i: crate::pid::i_gain(2),
            torque_p: crate::pid::p_gain(3),
            torque_i: crate::pid::i_gain(4),
        };
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_flux, fail_torque, expected_first_ok, expected_second_ok) in cases {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            runtime.set_stored_gains(previous);
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            tmc_state.mark_pi_representation_established();
            let mut spi = RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            if fail_flux {
                spi = spi.fail_writes_to(PID_FLUX_P_FLUX_I.addr());
            }
            if fail_torque {
                spi = spi.fail_writes_to(PID_TORQUE_P_TORQUE_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status =
                apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                    .await;
            let spi = tmc.into_inner();

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 3);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                            .addr
                            .addr(),
                        u32::from(foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr,),
                    ),
                    (PID_FLUX_P_FLUX_I.addr(), (11u32 << 16) | 12,),
                    (PID_TORQUE_P_TORQUE_I.addr(), (13u32 << 16) | 14,),
                ],
            );
            assert_eq!(
                runtime.stored_gains(),
                if status.all_ok() {
                    Some(requested)
                } else {
                    Some(previous)
                },
            );
        }
    });
}

#[test]
fn apply_live_outer_gains_publishes_only_after_both_writes_succeed() {
    pollster::block_on(async {
        let previous = crate::pid::test_outer_gains(101, 102, 103, 104);
        let requested = crate::pid::test_outer_gains(201, 202, 203, 204);
        let cases = [
            (false, false, true, true),
            (true, false, false, true),
            (false, true, true, false),
            (true, true, false, false),
        ];

        for (fail_velocity, fail_position, expected_first_ok, expected_second_ok) in cases {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            tmc_state.store_outer_gains(previous, Relaxed);
            tmc_state.mark_pi_representation_established();
            let mut spi = RuntimeMockSpi::default()
                .with_read(foci_tmc4671::registers::ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12);
            if fail_velocity {
                spi = spi.fail_writes_to(PID_VELOCITY_P_VELOCITY_I.addr());
            }
            if fail_position {
                spi = spi.fail_writes_to(PID_POSITION_P_POSITION_I.addr());
            }
            let mut tmc = Tmc4671Spi::new(spi);

            let status = apply_live_outer_gains(&tmc_state, &mut tmc, requested).await;
            let spi = tmc.into_inner();
            let published = tmc_state.load_outer_gains(Relaxed).unwrap();

            assert_eq!(status.first_write_ok, expected_first_ok);
            assert_eq!(status.second_write_ok, expected_second_ok);
            assert_eq!(spi.write_attempts.len(), 3);
            assert_eq!(
                spi.write_attempts,
                std::vec![
                    (
                        foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT
                            .addr
                            .addr(),
                        u32::from(foci_tmc4671::registers::CONFIG_ADVANCED_PI_REPRESENT.sub_addr,),
                    ),
                    (PID_VELOCITY_P_VELOCITY_I.addr(), (203u32 << 16) | 204,),
                    (PID_POSITION_P_POSITION_I.addr(), (201u32 << 16) | 202,),
                ],
            );
            assert_eq!(
                published,
                if status.all_ok() { requested } else { previous },
            );
        }
    });
}

#[test]
fn live_gain_write_refuses_before_pi_representation_establishment() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let status =
            apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                .await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn live_gain_write_refuses_mismatching_pi_representation_before_pid_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.mark_pi_representation_established();
        let requested = InnerLoopGains {
            flux_p: crate::pid::p_gain(11),
            flux_i: crate::pid::i_gain(12),
            torque_p: crate::pid::p_gain(13),
            torque_i: crate::pid::i_gain(14),
        };
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(0));

        let status =
            apply_live_inner_gains(runtime.stored_gains_mut(), &tmc_state, &mut tmc, requested)
                .await;
        let spi = tmc.into_inner();

        assert!(!status.all_ok());
        assert!(!spi.attempted_write_to(PID_FLUX_P_FLUX_I));
        assert!(!spi.attempted_write_to(PID_TORQUE_P_TORQUE_I));
        assert!(!tmc_state.pi_representation_established());
    });
}

#[test]
fn live_gain_failure_outcome_maps_incomplete_pairs_to_shutdown() {
    let cases = [
        (true, true, None),
        (
            false,
            true,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            true,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
        (
            false,
            false,
            Some(TmcControlRequestOutcome::shutdown(
                ShutdownReason::LiveGainApplyFailed,
                TmcControlRequestTail::SkipTickTail,
            )),
        ),
    ];

    for (first_write_ok, second_write_ok, expected) in cases {
        let status = LiveGainWriteStatus {
            first_write_ok,
            second_write_ok,
        };

        assert_eq!(live_gain_failure_outcome(status), expected);
    }
}

#[test]
fn shared_dispatch_set_current_writes_current_limit_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let expected_limit = ((1155.0_f32 * core::f32::consts::SQRT_2) / 1.155_f32) as u32;

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetCurrent { run_ma: 1155 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(PID_TORQUE_FLUX_LIMITS.addr(), expected_limit)]
        );
        assert_eq!(
            u32::from(tmc_state.load_current_limit(Relaxed).raw()),
            expected_limit
        );
    });
}

#[test]
fn shared_dispatch_set_current_rejects_invalid_limit_distinctly() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetCurrent { run_ma: u32::MAX },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;

        assert_eq!(result, Err(RegisterRequestError::CurrentLimitInvalid));
    });
}

#[test]
fn apply_motion_scale_writes_encoder_then_physical_width_and_publishes_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

        let result = apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
            Ok::<(), core::convert::Infallible>(())
        })
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(()));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_PPR.addr(), 4000), (STEP_WIDTH.addr(), 16)]
        );
        assert_eq!(tmc_state.encoder_counts_per_rev(), Some(4000));
        assert_eq!(stepper_state.planner_steps_per_rev(&tmc_state), Some(3200));
        assert_eq!(tmc_state.physical_step_width(), Some(16));
    });
}

#[test]
fn apply_motion_scale_spi_failures_leave_scale_unconfigured() {
    pollster::block_on(async {
        for failed_register in [ABN_DECODER_PPR.addr(), STEP_WIDTH.addr()] {
            let tmc_state = crate::state::tmc::TmcChannelState::new();
            let stepper_state = crate::state::stepper::StepperChannelState::new();
            let mut tmc =
                Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(failed_register));
            let scale = crate::motion_scale::MotionScale::new(1000, 3200).unwrap();

            let expected = if failed_register == ABN_DECODER_PPR.addr() {
                crate::motion_scale::MotionScaleRejection::EncoderRegisterWriteFailed
            } else {
                crate::motion_scale::MotionScaleRejection::StepWidthRegisterWriteFailed
            };
            assert_eq!(
                apply_motion_scale(&mut tmc, &tmc_state, &stepper_state, scale, |_| {
                    Ok::<(), core::convert::Infallible>(())
                })
                .await,
                Err(expected)
            );
            assert!(!tmc_state.motion_scale_configured());
            assert_eq!(tmc_state.physical_step_width(), None);
        }
    });
}

#[test]
fn shared_dispatch_set_encoder_dir_writes_mode_and_marks_reset() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetEncoderDir { invert: true },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert_eq!(
            spi.write_attempts,
            std::vec![(ABN_DECODER_MODE.addr(), 1 << 12)]
        );
        assert!(tmc_state.encoder_reversed.load(Relaxed));
        assert!(tmc_state.feedforward_reset.load(Relaxed));
    });
}

#[test]
fn shared_dispatch_set_voltage_limit_propagates_write_failure() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PIDOUT_UQ_UD_LIMITS.addr()));

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetVoltageLimit {
                voltage_limit: 32_000,
            },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Err(RegisterRequestError::Other));
        assert!(spi.attempted_write_to(PIDOUT_UQ_UD_LIMITS));
    });
}

#[test]
fn shared_dispatch_set_torque_filter_writes_biquad_and_updates_state() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 3000 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 1))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(3000));
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_set_torque_filter_zero_disables_filter() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.torque_filter_hz.store(Some(3000), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::SetTorqueFilter { filter_hz: 0 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Handled));
        assert!(spi.write_attempts.contains(&(
            CONFIG_BIQUAD_T_ENABLE.addr.addr(),
            CONFIG_BIQUAD_T_ENABLE.sub_addr as u32,
        )));
        assert!(
            spi.write_attempts
                .contains(&(CONFIG_BIQUAD_T_ENABLE.data.addr(), 0))
        );
        assert_eq!(tmc_state.torque_filter_hz.load(Relaxed), Some(0));
        assert!(spi.read_requests.is_empty());
    });
}

#[test]
fn shared_dispatch_main_shell_request_returns_unhandled_without_spi() {
    pollster::block_on(async {
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = apply_shared_tmc_request(
            &TmcRequestKind::Calibrate { oid: 7 },
            &mut tmc,
            &tmc_state,
            1.155,
            3999,
        )
        .await;
        let spi = tmc.into_inner();

        assert_eq!(result, Ok(SharedTmcDispatch::Unhandled));
        assert!(spi.write_attempts.is_empty());
        assert!(spi.read_requests.is_empty());
    });
}

fn commissioning_runtime_config() -> TmcControlConfig {
    let tmc = TmcConfig::default_for_test();
    TmcControlConfig::new(84_000_000, tmc.pwm_freq_hz(), 0.03125, tmc)
}

#[test]
fn scheduled_step_preserves_explicit_engine_deadline_without_consuming_engine() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 42_000;
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));
        assert!(matches!(
            runtime
                .step_commissioning_scheduled(&mut backend)
                .await
                .expect("engine should be active"),
            crate::commissioning::schedule::CommissionPoll::At {
                timestamp_us: 292_000,
                event: None,
            }
        ));
    });
}

#[test]
fn scheduled_step_keeps_inductance_capture_inside_one_deadline_loop() {
    pollster::block_on(async {
        use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope};
        use crate::commissioning::encoder_domain::EncoderDomain;
        use crate::commissioning::inner::InductanceCaptureState;

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut engine =
            CommissioningEngine::new_commission(7, crate::autotune::CommissionProfile::Balanced);
        engine.set_inductance_capture_for_test(InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4_000).unwrap(),
            encoder_start: 0,
            start_us: 900,
            previous_elapsed_us: 0,
            previous_phi: 0,
            accumulated_phi_delta_counts: 0,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count: 0,
            status_flags_or: 0,
            next_sample_us: 1_000,
            timing: CaptureTiming::new(200, TimingEnvelope::INDUCTANCE),
            current_envelope: 563,
        });
        runtime.set_commission_engine(engine);

        let mut backend = MockBackend::new();
        backend.elapsed_us_value = 1_138;
        backend.advance_elapsed_on_delay = true;
        let mut phi_samples = heapless::Vec::<u32, 128>::new();
        let mut current_samples = heapless::Vec::<u32, 128>::new();
        let actual = ((-84i16 as u16 as u32) << 16) | 20;
        for index in 1..=100u32 {
            phi_samples
                .push((index.wrapping_mul(20 * 65_536) / 100) & 0xffff)
                .unwrap();
            current_samples.push(actual).unwrap();
        }
        backend.set_openloop_phi_sequence(phi_samples.as_slice(), 0);
        backend.set_pid_torque_flux_actual_sequence(current_samples.as_slice());

        let poll = runtime
            .step_commissioning_scheduled(&mut backend)
            .await
            .expect("engine should be active");

        assert!(matches!(
            poll,
            crate::commissioning::schedule::CommissionPoll::ReadyNow { event: None }
        ));
        assert_eq!(backend.register_pair_reads, 100);
    });
}

#[test]
fn scheduled_capture_shutdown_uses_canonical_engine_failure_path() {
    pollster::block_on(async {
        use crate::commissioning::capture_timing::{CaptureTiming, TimingEnvelope};
        use crate::commissioning::encoder_domain::EncoderDomain;
        use crate::commissioning::inner::InductanceCaptureState;

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut engine =
            CommissioningEngine::new_commission(7, crate::autotune::CommissionProfile::Balanced);
        engine.set_inductance_capture_for_test(InductanceCaptureState {
            encoder_domain: EncoderDomain::new(4_000).unwrap(),
            encoder_start: 0,
            start_us: 900,
            previous_elapsed_us: 0,
            previous_phi: 0,
            accumulated_phi_delta_counts: 0,
            accumulator: crate::electrical_capture::AcFrameAccumulator::new(),
            first_half: crate::electrical_capture::AcFrameAccumulator::new(),
            second_half: crate::electrical_capture::AcFrameAccumulator::new(),
            saliency: crate::electrical_capture::AcSaliencyAccumulator::new(),
            sample_count: 0,
            status_flags_or: 0,
            next_sample_us: 1_000,
            timing: CaptureTiming::new(200, TimingEnvelope::INDUCTANCE),
            current_envelope: 563,
        });
        runtime.set_commission_engine(engine);

        let mut backend = MockBackend::new();
        backend.enable_pin = true;
        backend.elapsed_us_value = 900;
        backend.shutdown_requested = true;

        let poll = runtime
            .step_commissioning_scheduled(&mut backend)
            .await
            .expect("engine should be active");

        assert!(matches!(
            poll,
            crate::commissioning::schedule::CommissionPoll::Terminal {
                result: crate::commissioning::schedule::CommissionTerminal::InnerFailed(
                    crate::autotune::CommissionError::Shutdown
                )
            }
        ));
        assert!(!backend.enable_pin);
        assert_eq!(backend.register_pair_reads, 0);
    });
}

#[test]
fn scheduled_dispatch_preserves_wait_action() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::At {
                    timestamp_us: 43_000,
                    event: None,
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("engine should be active");

        assert!(matches!(
            action,
            ScheduledCommissionTickAction::At {
                timestamp_us: 43_000,
                event: None,
            }
        ));
        assert!(runtime.is_commission_active());
    });
}

#[test]
fn scheduled_classification_is_synchronous_for_at_action() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();
    let tmc = crate::state::tmc::tmc_channel0();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

    let action = runtime
        .classify_commission_poll(crate::commissioning::schedule::CommissionPoll::At {
            timestamp_us: 43_000,
            event: None,
        })
        .expect("engine should be active");

    assert!(matches!(
        action,
        ScheduledCommissionTickAction::At {
            timestamp_us: 43_000,
            event: None,
        }
    ));
}

#[test]
fn scheduled_dispatch_preserves_phase_event_context_without_delay() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            37,
            crate::autotune::CommissionProfile::Balanced,
        ));
        let poll = crate::commissioning::schedule::adapt_step_result(
            crate::commissioning::types::StepResult::PhaseChanged(
                crate::autotune::CommissionPhase::ElectricalIdentification,
            ),
            81_250,
            Some((37, crate::commissioning::types::EngineKind::Commission)),
        )
        .expect("phase change should produce a scheduled poll");

        assert!(matches!(
            &poll,
            crate::commissioning::schedule::CommissionPoll::ReadyNow {
                event: Some(
                    crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                        oid: 37,
                        phase_id: crate::autotune::CommissionPhase::ElectricalIdentification,
                        engine_kind: crate::commissioning::types::EngineKind::Commission,
                    }
                ),
            }
        ));

        let action = runtime
            .drive_commissioning_poll(poll, &mut backend, tmc, stepper, &config)
            .await
            .expect("engine should be active");

        assert!(matches!(
            action,
            ScheduledCommissionTickAction::ReadyNow {
                event: Some(
                    crate::commissioning::schedule::CommissionEvent::PhaseChanged {
                        oid: 37,
                        phase_id: crate::autotune::CommissionPhase::ElectricalIdentification,
                        engine_kind: crate::commissioning::types::EngineKind::Commission,
                    }
                ),
            }
        ));
        assert!(runtime.is_commission_active());
    });
}

#[test]
fn scheduled_dispatch_preserves_terminal_reply_and_actions() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_selftest_for_tmc(7, tmc));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::Terminal {
                    result: crate::commissioning::schedule::CommissionTerminal::SelftestDone,
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("terminal dispatch should produce an action");

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply: ReplyPayload::Selftest { oid: 7, .. },
                actions,
            } => {
                assert_eq!(actions.stop_trace, PostActions::NONE.stop_trace);
                assert_eq!(actions.stepper, PostActions::NONE.stepper);
            }
            _ => panic!("expected Selftest terminal action"),
        }
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn scheduled_dispatch_preserves_accepted_commission_reply_and_actions() {
    pollster::block_on(async {
        use core::sync::atomic::Ordering;

        const MOTOR_SENTINEL: u32 = 0x0002_0032;

        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.reset();
        stepper.reset();
        tmc.mark_pi_representation_established();
        tmc.store_current_limit(current_limit(0x10), Ordering::Relaxed);
        tmc.motor_sentinel.store(MOTOR_SENTINEL, Ordering::Relaxed);
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend
            .write_register(
                foci_tmc4671::registers::MOTOR_TYPE_N_POLE_PAIRS,
                MOTOR_SENTINEL,
            )
            .await
            .expect("test motor sentinel write should succeed");
        let config = commissioning_runtime_config();
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            23,
            crate::autotune::CommissionProfile::Balanced,
        ));

        let action = runtime
            .drive_commissioning_poll(
                crate::commissioning::schedule::CommissionPoll::Terminal {
                    result: crate::commissioning::schedule::CommissionTerminal::CommissionDone(
                        accepted_commission_result(),
                    ),
                },
                &mut backend,
                tmc,
                stepper,
                &config,
            )
            .await
            .expect("terminal dispatch should produce an action");

        match action {
            ScheduledCommissionTickAction::Terminal {
                reply:
                    ReplyPayload::CommissionResult {
                        oid,
                        status,
                        result,
                    },
                actions,
            } => {
                assert_eq!(oid, 23);
                assert_eq!(status, 0);
                assert_eq!(result.inner.r_count_milli, 1706);
                assert_eq!(result.inner.l_count_micro, 1245);
                assert!(actions.stop_trace);
                assert_eq!(actions.stepper, StepperPostAction::ArmPreservePosition);
            }
            _ => panic!("expected accepted CommissionResult terminal action"),
        }
        assert!(!runtime.is_commission_active());
        assert!(runtime.is_calibrated());
        assert!(runtime.is_motor_enabled());
        assert!(runtime.stored_gains().is_some());
        assert!(backend.enable_pin);
        assert!(!tmc.motor_armed.load(Ordering::Relaxed));
        assert!(!tmc.commission_in_flight.load(Ordering::Relaxed));

        tmc.reset();
        stepper.reset();
    });
}

#[test]
fn state_refs_carry_channel_state_without_ownership() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let refs = TmcControlStateRefs::new(&tmc, &stepper);

    assert!(core::ptr::eq(refs.tmc, &tmc));
    assert!(core::ptr::eq(refs.stepper, &stepper));
}

#[test]
fn dispatch_state_view_mutates_runtime_owned_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());

    runtime.set_calibrated_for_test(false);
    runtime.set_motor_enabled_for_test(false);
}

#[test]
fn config_reset_runtime_state_clears_owned_flags_and_defaults() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(1234);

    runtime.reset_runtime_state();

    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(runtime.planner_velocity_scale(), None);
    assert_eq!(runtime.planner_rpm_scale(), None);
}

#[test]
fn config_reset_channel_state_restores_known_defaults() {
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();

    tmc.store_current_limit(current_limit(99), Relaxed);
    tmc.velocity_limit_cfg.store(123, Relaxed);
    stepper.position.store(44, Relaxed);

    TmcControlRuntime::reset_channel_state(&tmc, &stepper);

    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 0);
    assert_eq!(tmc.velocity_limit_cfg.load(Relaxed), 500_000);
    assert_eq!(stepper.position.load(Relaxed), 0);
}

#[test]
fn queue_flush_without_signal_does_not_mutate_state() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.store_current_limit(current_limit(88), Relaxed);
    stepper.position.store(55, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert_no_actions(&actions);
    assert!(!queue_flush.load(Relaxed));
    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 88);
    assert_eq!(stepper.position.load(Relaxed), 55);
    assert!(fault_active.load(Relaxed));
}

#[test]
fn queue_flush_signal_resets_runtime_and_channel_state() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.store_current_limit(current_limit(88), Relaxed);
    stepper.position.store(55, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(!queue_flush.load(Relaxed));
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_motor_enabled());
    assert_eq!(tmc.load_current_limit(Relaxed).raw(), 0);
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert!(!fault_active.load(Relaxed));
    assert_single_stop_now(&actions);
}

#[test]
fn queue_flush_keeps_the_fault_latch_while_the_status_pin_is_asserted() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    stepper.position.store(55, Relaxed);
    tmc.status_pin_asserted.store(true, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(
        fault_active.load(Relaxed),
        "a flush must not retire a latch the STATUS pin still reports"
    );
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert_single_stop_now(&actions);
}

#[test]
fn queue_flush_keeps_the_fault_latch_while_the_disable_signal_is_pending() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    disable_signal.signal(());
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    stepper.position.store(55, Relaxed);
    queue_flush.store(true, Relaxed);

    let actions = runtime.poll_queue_flush(&signals, &tmc, &stepper);

    assert!(
        fault_active.load(Relaxed),
        "a flush must not retire a latch while an emergency disable is pending"
    );
    assert!(
        disable_signal.signaled(),
        "the pending disable must survive for the board's containment check"
    );
    assert_eq!(stepper.position.load(Relaxed), 0);
    assert_single_stop_now(&actions);
}

#[test]
fn pid_error_clear_request_loads_hold_settle_countdown() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();

    tmc.pid_error_sums_clear.store(true, Relaxed);
    runtime.poll_pid_error_clear_request(&tmc);

    assert_eq!(
        runtime.pid_error_clear_countdown_for_test(),
        crate::tmc_control::HOLD_SETTLE_DELAY_MS
    );
}

#[test]
fn pid_error_settle_tick_waits_for_inactive_timer_then_clears() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        runtime.set_motor_enabled_for_test(true);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .expect("motion scale should configure");
        tmc_state.pid_error_sums_clear.store(true, Relaxed);
        stepper_state.timer_active_mirror.store(true, Relaxed);

        let mut busy_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_pid_error_settle_tick(
                &mut busy_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert!(!tmc_state.pid_error_sums_clear.load(Relaxed));
        assert_eq!(
            runtime.pid_error_clear_countdown_for_test(),
            crate::tmc_control::HOLD_SETTLE_DELAY_MS
        );
        assert!(busy_tmc.into_inner().writes.is_empty());

        stepper_state.timer_active_mirror.store(false, Relaxed);
        runtime.set_pid_error_clear_countdown_ms(1);
        let mut settle_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000).with_read(1_000));

        runtime
            .apply_pid_error_settle_tick(
                &mut settle_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
            )
            .await;

        assert_eq!(runtime.pid_error_clear_countdown_for_test(), 0);
        let raw_spi = settle_tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![
                (PID_POSITION_ACTUAL.addr(), 1_000),
                (
                    PID_ERROR_PID_TORQUE_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_TORQUE_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_TORQUE_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_FLUX_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_FLUX_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_FLUX_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_VELOCITY_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_VELOCITY_ERROR_SUM.data.addr(), 0),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
                (STATUS_FLAGS.addr(), 0),
            ]
        );
    });
}

#[test]
fn homing_channel_tick_suppresses_position_i_without_global_advance() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 123u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_suppression() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(77, 0, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_P_POSITION_I.addr(), 77u32 << 16)]
        );
    });
}

#[test]
fn homing_channel_tick_restores_position_i_and_clears_integrator() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(12, 34, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (12u32 << 16) | 34),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
    });
}

#[test]
fn homing_channel_tick_preserves_openffboard_ungated_restore() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.store_outer_gains(crate::pid::test_outer_gains(7, 9, 1152, 0), Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![
                (PID_POSITION_P_POSITION_I.addr(), (7u32 << 16) | 9),
                (
                    PID_ERROR_PID_POSITION_ERROR_SUM.addr.addr(),
                    PID_ERROR_PID_POSITION_ERROR_SUM.sub_addr as u32,
                ),
                (PID_ERROR_PID_POSITION_ERROR_SUM.data.addr(), 0),
            ]
        );
    });
}

#[test]
fn homing_channel_tick_restore_failure_blocks_advance() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let raw_spi =
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &commissioning_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::RetryCurrentState);
        assert!(runtime.phase_advance_active_for_test());
        assert!(
            !tmc.into_inner()
                .attempted_write_to(PID_POSITION_P_POSITION_I)
        );
    });
}

#[test]
fn homing_state_1_phase_restore_failure_retries_twice_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
        );

        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
    });
}

#[test]
fn homing_state_1_outer_gain_failure_retries_twice_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        tmc_state.corrupt_outer_gain_for_test(0x8000);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
    });
}

/// Board current scale realistic enough that both the homing and the run
/// current land inside the TMC4671 limit domain. `commissioning_runtime_config`
/// uses 0.03125 mA/LSB, which caps the representable run current near 724 mA.
const HOMING_CURRENT_SCALE_MA_PER_LSB: f32 = 1.155;

fn homing_runtime_config() -> TmcControlConfig {
    let mut config = commissioning_runtime_config();
    config.current_scale_ma_per_lsb = HOMING_CURRENT_SCALE_MA_PER_LSB;
    config
}

fn due_homing_window() -> HomingClockWindow {
    HomingClockWindow { now: 0, start: 0 }
}

fn homing_limit_for(run_ma: u32) -> foci_tmc4671::current_limit::PidTorqueFluxLimit {
    crate::tmc_control::current_limit::current_limit_from_run_ma(
        run_ma,
        HOMING_CURRENT_SCALE_MA_PER_LSB,
    )
    .expect("current must be representable")
}

fn homing_state(runtime: &mut TmcControlRuntime) -> crate::state::tmc::TmcChannelState {
    let tmc_state = crate::state::tmc::TmcChannelState::new();
    tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
    tmc_state.homing_current_ma.store(700, Relaxed);
    tmc_state.stall_threshold_units.store(100, Relaxed);
    tmc_state.stall_persistence.store(2, Relaxed);
    tmc_state.store_current_limit(homing_limit_for(2300), Relaxed);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_calibrated_for_test(true);
    tmc_state
}

async fn homing_tick(
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
    refs: TmcControlStateRefs<'_>,
    state: u8,
) -> HomingMaintenanceResult {
    runtime
        .apply_homing_channel_tick(
            tmc,
            refs,
            &homing_runtime_config(),
            state,
            due_homing_window(),
        )
        .await
}

async fn three_homing_state_1_attempts(
    runtime: &mut TmcControlRuntime,
    tmc: &mut Tmc4671Spi<RuntimeMockSpi>,
    refs: TmcControlStateRefs<'_>,
) -> [HomingMaintenanceResult; 3] {
    [
        homing_tick(runtime, tmc, refs, 1).await,
        homing_tick(runtime, tmc, refs, 1).await,
        homing_tick(runtime, tmc, refs, 1).await,
    ]
}

const fn expected_exhausted_homing_state_1_budget() -> [HomingMaintenanceResult; 3] {
    [
        HomingMaintenanceResult::RetryCurrentState,
        HomingMaintenanceResult::RetryCurrentState,
        HomingMaintenanceResult::FailClosed,
    ]
}

#[test]
fn homing_prepare_clamps_torque_limit_and_arms_detector() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let expected_limit = homing_limit_for(700).register_value();
        let writes = tmc.into_inner().writes;
        assert!(writes.contains(&(PID_TORQUE_FLUX_LIMITS.addr(), expected_limit)));
        assert!(tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
        assert!(runtime.stall_detector_armed_for_test());
        assert_eq!(
            tmc_state.load_current_limit(Relaxed).raw(),
            homing_limit_for(2300).raw(),
            "clamp must not overwrite the stored run current"
        );
    });
}

#[test]
fn homing_clamp_due_opens_a_fixed_lead_before_the_start_clock() {
    let clock_freq = 84_000_000;
    let lead = homing_clamp_lead_ticks(clock_freq);
    assert_eq!(lead, 336_000, "4 ms lead at 84 MHz");
    let start = 10_000_000;
    let window = |now| HomingClockWindow { now, start };
    assert!(!homing_clamp_due(window(start - lead - 1), clock_freq));
    assert!(homing_clamp_due(window(start - lead), clock_freq));
    assert!(homing_clamp_due(window(start + 5), clock_freq));
    let wrapped = HomingClockWindow {
        now: u32::MAX - 100,
        start: lead + 50,
    };
    assert!(!homing_clamp_due(wrapped, clock_freq));
}

#[test]
fn homing_state_1_defers_the_clamp_until_the_start_window_opens() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let config = homing_runtime_config();
        let lead = homing_clamp_lead_ticks(config.clock_freq);
        let start = 50_000_000;

        let early = HomingClockWindow {
            now: start - lead - 1_000,
            start,
        };
        let deferred = runtime
            .apply_homing_channel_tick(&mut tmc, refs, &config, 1, early)
            .await;

        assert_eq!(deferred, HomingMaintenanceResult::Deferred);
        assert!(
            tmc.inner().writes.is_empty(),
            "no register may change while the previous move is still running"
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());

        let due = HomingClockWindow {
            now: start - lead,
            start,
        };
        let applied = runtime
            .apply_homing_channel_tick(&mut tmc, refs, &config, 1, due)
            .await;

        assert_eq!(applied, HomingMaintenanceResult::ReadyToAdvance);
        assert!(tmc_state.homing_clamp_active.load(Relaxed));
        assert!(runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_1_deferral_does_not_consume_the_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let config = homing_runtime_config();
        let start = 50_000_000;
        let early = HomingClockWindow {
            now: start - homing_clamp_lead_ticks(config.clock_freq) - 1,
            start,
        };

        for _ in 0..(HOMING_CLAMP_MAX_RETRIES + 2) {
            let result = runtime
                .apply_homing_channel_tick(&mut tmc, refs, &config, 1, early)
                .await;
            assert_eq!(result, HomingMaintenanceResult::Deferred);
        }
        assert!(runtime.homing_state_1_retries_for_test() == 0);
    });
}

#[test]
fn homing_prepare_without_homing_current_leaves_limit_alone() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        tmc_state.homing_current_ma.store(0, Relaxed);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_prepare_fails_closed_for_stall_endstop_without_current() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        tmc_state.homing_current_ma.store(0, Relaxed);
        tmc_state.stall_endstop_configured.store(true, Relaxed);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::FailClosed);
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
    });
}

#[test]
fn homing_prepare_fails_closed_when_not_armed_for_steps() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        runtime.set_calibrated_for_test(false);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::FailClosed);
        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
    });
}

#[test]
fn homing_prepare_fails_closed_when_homing_current_not_below_run_current() {
    pollster::block_on(async {
        for homing_ma in [2300u32, 3000] {
            let mut runtime = TmcControlRuntime::new_for_channel(0);
            let tmc_state = homing_state(&mut runtime);
            tmc_state.homing_current_ma.store(homing_ma, Relaxed);
            let stepper_state = crate::state::stepper::StepperChannelState::new();
            let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

            let result = runtime
                .apply_homing_channel_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &homing_runtime_config(),
                    1,
                    due_homing_window(),
                )
                .await;

            assert_eq!(result, HomingMaintenanceResult::FailClosed);
            assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_LIMITS));
            assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        }
    });
}

#[test]
fn homing_shutdown_quiesce_drops_detector_and_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        assert!(runtime.stall_detector_armed_for_test());

        let _ = runtime.prepare_shutdown_quiesce();

        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_forced_idle_drops_detector_and_clamp_flag() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        tmc_state.stall_latched.store(true, Relaxed);
        assert!(tmc_state.homing_clamp_active.load(Relaxed));

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                0,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        assert!(!runtime.stall_detector_armed_for_test());
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
    });
}

#[test]
fn homing_state_1_clamp_write_failure_retries_twice_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );

        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_state_1_mixed_failure_causes_share_one_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut phase_restore_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
        );

        let first = homing_tick(&mut runtime, &mut phase_restore_failure, refs, 1).await;

        tmc_state.corrupt_outer_gain_for_test(0x8000);
        let mut outer_gain_failure = Tmc4671Spi::new(RuntimeMockSpi::default());
        let second = homing_tick(&mut runtime, &mut outer_gain_failure, refs, 1).await;

        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        let mut clamp_write_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );
        let third = homing_tick(&mut runtime, &mut clamp_write_failure, refs, 1).await;

        assert_eq!(first, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(second, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(third, HomingMaintenanceResult::FailClosed);
    });
}

#[test]
fn homing_state_1_success_resets_its_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        tmc_state.corrupt_outer_gain_for_test(0x8000);

        for _ in 0..2 {
            assert_eq!(
                homing_tick(&mut runtime, &mut tmc, refs, 1).await,
                HomingMaintenanceResult::RetryCurrentState
            );
        }

        tmc_state.store_outer_gains(crate::pid::test_outer_gains(123, 0, 1152, 0), Relaxed);
        assert_eq!(
            homing_tick(&mut runtime, &mut tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        tmc_state.corrupt_outer_gain_for_test(0x8000);
        assert_eq!(
            three_homing_state_1_attempts(&mut runtime, &mut tmc, refs).await,
            expected_exhausted_homing_state_1_budget()
        );
    });
}

#[test]
fn homing_state_1_attempts_preserve_state_3_retry_budget() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut successful_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            homing_tick(&mut runtime, &mut successful_tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut phase_restore_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
        );
        assert_eq!(
            homing_tick(&mut runtime, &mut phase_restore_failure, refs, 1).await,
            HomingMaintenanceResult::RetryCurrentState
        );

        let mut restore_failure = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );
        assert_eq!(
            homing_tick(&mut runtime, &mut restore_failure, refs, 3).await,
            HomingMaintenanceResult::RetryCurrentState
        );

        let mut successful_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            homing_tick(&mut runtime, &mut successful_tmc, refs, 1).await,
            HomingMaintenanceResult::ReadyToAdvance
        );

        assert_eq!(
            homing_tick(&mut runtime, &mut restore_failure, refs, 3).await,
            HomingMaintenanceResult::FailClosed
        );
    });
}

#[test]
fn homing_detect_sets_latch_after_persistence() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default()
                .with_read(1000)
                .with_read(950)
                .with_read(1000)
                .with_read(850)
                .with_read(1000)
                .with_read(850),
        );
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;

        assert_eq!(
            runtime
                .apply_homing_channel_tick(
                    &mut tmc,
                    refs,
                    &homing_runtime_config(),
                    2,
                    due_homing_window()
                )
                .await,
            HomingMaintenanceResult::ReadyToAdvance
        );
        assert!(!tmc_state.stall_latched.load(Relaxed));
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        assert!(!tmc_state.stall_latched.load(Relaxed));
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        assert!(tmc_state.stall_latched.load(Relaxed));

        let spi = tmc.into_inner();
        assert!(spi.read_requested(PID_POSITION_TARGET));
        assert!(spi.read_requested(PID_POSITION_ACTUAL));
    });
}

#[test]
fn homing_restore_writes_run_current_and_publishes_result() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default()
                .with_read(1000)
                .with_read(0)
                .with_read(1000)
                .with_read(0),
        );
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                2,
                due_homing_window(),
            )
            .await;

        let result = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(result, HomingMaintenanceResult::ReadyToAdvance);
        let run_limit = tmc_state.load_current_limit(Relaxed).register_value();
        assert_eq!(
            tmc.into_inner().writes.last(),
            Some(&(PID_TORQUE_FLUX_LIMITS.addr(), run_limit))
        );
        assert!(!tmc_state.homing_clamp_active.load(Relaxed));
        assert!(!tmc_state.stall_latched.load(Relaxed));
        assert!(tmc_state.stall_result_latched.load(Relaxed));
        assert_eq!(tmc_state.stall_peak_error_units.load(Relaxed), 1000);
        assert_eq!(tmc_state.stall_trigger_tick.load(Relaxed), 2);
        assert!(!runtime.stall_detector_armed_for_test());
    });
}

#[test]
fn homing_restore_failure_retries_then_fails_closed() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let tmc_state = homing_state(&mut runtime);
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                1,
                due_homing_window(),
            )
            .await;
        let mut tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_LIMITS.addr()),
        );

        let first = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;
        let second = runtime
            .apply_homing_channel_tick(
                &mut tmc,
                refs,
                &homing_runtime_config(),
                3,
                due_homing_window(),
            )
            .await;

        assert_eq!(first, HomingMaintenanceResult::RetryCurrentState);
        assert_eq!(second, HomingMaintenanceResult::FailClosed);
        assert!(
            tmc_state.homing_clamp_active.load(Relaxed),
            "clamp stays flagged until a successful restore or the board disable path"
        );
    });
}

#[test]
fn advance_homing_state_after_channels_waits_for_all_channels() {
    let homing_state = core::sync::atomic::AtomicU8::new(1);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(false);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);

    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, false);
    assert_eq!(homing_state.load(Relaxed), 1);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 1, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 2);

    homing_triggered.store(0, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 2, true);
    assert_eq!(homing_state.load(Relaxed), 3);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(homing_state.load(Relaxed), 0);
}

#[test]
fn advance_homing_state_re_enters_preparation_after_a_restore_with_a_queued_pass() {
    let homing_state = core::sync::atomic::AtomicU8::new(3);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(true);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, false);
    assert_eq!(
        homing_state.load(Relaxed),
        3,
        "an unfinished restore keeps waiting"
    );
    assert!(homing_reenter.load(Relaxed));

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        1,
        "the pass that registered during the restore gets its own preparation"
    );
    assert!(!homing_reenter.load(Relaxed));
}

#[test]
fn restore_leaves_state_3_before_consuming_the_re_entry_flag() {
    let homing_state = core::sync::atomic::AtomicU8::new(3);
    let homing_triggered = core::sync::atomic::AtomicU8::new(0xFF);
    let homing_reenter = core::sync::atomic::AtomicBool::new(false);

    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(homing_state.load(Relaxed), 0);

    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    crate::state::trsync::HOMING_STATE.store(0, Relaxed);
    crate::state::trsync::HOMING_REENTER_PENDING.store(false, Relaxed);
    assert!(
        crate::state::trsync::begin_homing_preparation(900),
        "a registration that arrives once state 3 is gone takes the idle path directly"
    );
    assert!(!crate::state::trsync::HOMING_REENTER_PENDING.load(Relaxed));
    crate::state::trsync::HOMING_STATE.store(0, Relaxed);

    homing_state.store(3, Relaxed);
    homing_reenter.store(true, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        1,
        "a registration that arrived while state 3 was still held is promoted after the exchange"
    );
    assert!(!homing_reenter.load(Relaxed));

    homing_state.store(0, Relaxed);
    homing_reenter.store(true, Relaxed);
    let homing = TmcControlHomingRefs::new(&homing_state, &homing_triggered, &homing_reenter);
    TmcControlRuntime::advance_homing_state_after_channels(homing, 3, true);
    assert_eq!(
        homing_state.load(Relaxed),
        0,
        "a stale snapshot never re-enters from idle"
    );
    assert!(
        homing_reenter.load(Relaxed),
        "the flag is left for the path that owns the state"
    );
}

#[test]
fn shutdown_quiesce_clears_safe_state_without_config_reset() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let gains = InnerLoopGains {
        flux_p: crate::pid::p_gain(11),
        flux_i: crate::pid::i_gain(12),
        torque_p: crate::pid::p_gain(13),
        torque_i: crate::pid::i_gain(14),
    };

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(crate::commissioning::CommissioningEngine::new_calibrate(0));
    runtime.set_stored_gains(gains);
    runtime.set_planner_velocity_scale_for_test(123_456);
    runtime.set_feedforward_was_active(true);
    runtime.set_pid_error_clear_countdown_ms(9);

    let actions = runtime.prepare_shutdown_quiesce();

    assert_single_stop_now(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());
    assert!(!runtime.feedforward_was_active());
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    assert_eq!(runtime.stored_gains(), Some(gains));
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
}

#[test]
fn shutdown_quiesce_clears_torque_flux_offset_activity() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let refs = TmcControlStateRefs::new(&tmc_state, &stepper_state);
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(&mut active_tmc, refs, &config, false, false, true)
                .await,
            Ok(())
        );
        assert!(
            active_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );

        assert_single_stop_now(&runtime.prepare_shutdown_quiesce());
        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert!(
            !inactive_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET),
            "shutdown quiesce should clear the offset-active latch so the next inactive tick does not emit a cleanup write"
        );
    });
}

#[test]
fn feedforward_and_pid_settle_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    assert!(!runtime.feedforward_was_active());
    runtime.set_feedforward_was_active(true);
    assert!(runtime.feedforward_was_active());

    assert_eq!(runtime.pid_error_clear_countdown_ms(), 0);
    runtime.set_pid_error_clear_countdown_ms(5);
    assert_eq!(runtime.pid_error_clear_countdown_ms(), 5);
}

#[test]
fn dispatch_state_remains_usable_after_shutdown_quiesce() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc: &'static crate::state::tmc::TmcChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::tmc::TmcChannelState::new()),
    );
    let stepper: &'static crate::state::stepper::StepperChannelState = std::boxed::Box::leak(
        std::boxed::Box::new(crate::state::stepper::StepperChannelState::new()),
    );

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.prepare_shutdown_quiesce();

    {
        let dispatch = runtime.dispatch_state(tmc, stepper);
        *dispatch.calibrated = true;
        *dispatch.motor_enabled = true;
    }

    assert!(runtime.is_calibrated());
    assert!(runtime.is_motor_enabled());
}

#[test]
fn shutdown_signal_emits_one_stop_action_until_reset() {
    let shutdown_active = AtomicBool::new(true);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(false);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(7));

    let first = runtime.poll_shutdown(&signals);

    assert_single_stop_now(&first);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!runtime.is_commission_active());

    let second = runtime.poll_shutdown(&signals);
    assert_no_actions(&second);

    shutdown_active.store(false, Relaxed);
    assert_no_actions(&runtime.poll_shutdown(&signals));

    shutdown_active.store(true, Relaxed);
    let third = runtime.poll_shutdown(&signals);
    assert_single_stop_now(&third);
}

#[test]
fn adc_current_clipped_true_when_bit_set() {
    assert!(is_adc_current_clipped(Ok(1 << 26)));
}

#[test]
fn adc_current_clipped_false_for_clean_read() {
    assert!(!is_adc_current_clipped(Ok(0)));
}

#[test]
fn adc_current_clipped_false_when_status_read_fails() {
    assert!(!is_adc_current_clipped(Err(())));
}

#[test]
fn classify_emergency_disable_fault_prefers_adc_clipped_over_read_failure() {
    assert_eq!(
        classify_emergency_disable_fault(Ok(1 << 26)),
        EmergencyDisableFault::AdcCurrentClipped
    );
}

#[test]
fn classify_emergency_disable_fault_is_read_failed_on_err() {
    assert_eq!(
        classify_emergency_disable_fault(Err(())),
        EmergencyDisableFault::StatusReadFailed
    );
}

#[test]
fn classify_emergency_disable_fault_is_generic_for_other_fault_bits() {
    assert_eq!(
        classify_emergency_disable_fault(Ok(1 << 3)),
        EmergencyDisableFault::Generic
    );
}

#[test]
fn combine_emergency_disable_fault_clipped_beats_read_failed_either_order() {
    use EmergencyDisableFault::{AdcCurrentClipped, StatusReadFailed};
    assert_eq!(
        combine_emergency_disable_fault(AdcCurrentClipped, StatusReadFailed),
        AdcCurrentClipped
    );
    assert_eq!(
        combine_emergency_disable_fault(StatusReadFailed, AdcCurrentClipped),
        AdcCurrentClipped
    );
}

#[test]
fn combine_emergency_disable_fault_read_failed_beats_generic_either_order() {
    use EmergencyDisableFault::{Generic, StatusReadFailed};
    assert_eq!(
        combine_emergency_disable_fault(StatusReadFailed, Generic),
        StatusReadFailed
    );
    assert_eq!(
        combine_emergency_disable_fault(Generic, StatusReadFailed),
        StatusReadFailed
    );
}

#[test]
fn multi_channel_fold_retains_earlier_channel_failure_after_later_channel_succeeds() {
    let mut fault = EmergencyDisableFault::Generic;
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Err(())));
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Ok(0)));
    assert_eq!(fault, EmergencyDisableFault::StatusReadFailed);
}

#[test]
fn multi_channel_fold_retains_earlier_channel_clip_after_later_channel_read_fails() {
    let mut fault = EmergencyDisableFault::Generic;
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Ok(1 << 26)));
    fault = combine_emergency_disable_fault(fault, classify_emergency_disable_fault(Err(())));
    assert_eq!(fault, EmergencyDisableFault::AdcCurrentClipped);
}

#[test]
fn emergency_disable_without_signal_is_empty() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.poll_emergency_disable(&signals, Ok(1 << 26));

    assert_no_actions(&actions);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(fault_active.load(Relaxed));
}

#[test]
fn emergency_disable_consumes_signal_and_orders_stop_before_shutdown() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    disable_signal.signal(());
    let clipped = runtime.poll_emergency_disable(&signals, Ok(1 << 26));

    assert_stop_then_shutdown(&clipped, ShutdownReason::AdcCurrentClipped);
    assert!(disable_signal.try_take().is_none());
    assert!(!fault_active.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());

    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    fault_active.store(true, Relaxed);
    disable_signal.signal(());
    let non_clipping_fault = runtime.poll_emergency_disable(&signals, Ok(0));

    assert_stop_then_shutdown(&non_clipping_fault, ShutdownReason::TmcFault);
    assert!(disable_signal.try_take().is_none());
    assert!(!fault_active.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn emergency_disable_treats_failed_status_read_as_tmc_fault() {
    let shutdown_active = AtomicBool::new(false);
    let queue_flush = AtomicBool::new(false);
    let fault_active = AtomicBool::new(true);
    let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
    let signals = TmcControlSignals::new(
        &shutdown_active,
        &queue_flush,
        &fault_active,
        &disable_signal,
    );
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    disable_signal.signal(());
    let read_failed = runtime.poll_emergency_disable(&signals, Err(()));

    assert_stop_then_shutdown(&read_failed, ShutdownReason::TmcFault);
    assert!(disable_signal.try_take().is_none());
    assert!(!fault_active.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn same_channel_regular_request_is_unhandled_without_mutating_runtime() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(8));
    let req = TmcRequest::new(0, TmcRequestKind::SetCurrent { run_ma: 750 });

    let actions = runtime.classify_request(req);

    assert_no_actions(&actions);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(runtime.is_commission_active());
}

#[test]
fn wrong_channel_request_emits_only_invalid_channel_shutdown_without_mutation() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(9));
    let req = TmcRequest::new(1, TmcRequestKind::SetEnable(false));

    let actions = runtime.classify_request(req);

    assert_single_shutdown(&actions, ShutdownReason::InvalidTmcChannel);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
    assert!(runtime.is_commission_active());
}

#[test]
fn all_active_diagnostic_requests_are_delegated_without_mutation() {
    let requests = [
        crate::tmc::tmc_current_step_test(0, 10, crate::tmc::CurrentStepAxis::Torque, 111, 12, 13),
        crate::tmc::tmc_current_vector_step_test(0, 11, 112, -113, 14, 15),
        crate::tmc::tmc_current_torque_sample_test(0, 12, 114, -115, 16, 17),
        crate::tmc::tmc_position_torque_offset_sample_test(0, 13, 116, 18, 19),
        crate::tmc::tmc_voltage_step_test(0, 14, 120, -121, 20),
        crate::tmc::tmc_resistance_test(0, 15, 0),
        #[cfg(feature = "trace-velocity-limit-latch-test")]
        crate::tmc::tmc_velocity_limit_latch_test(0, 16),
    ];

    for req in requests {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_calibrated_for_test(true);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(CommissioningEngine::new_calibrate(10));
        let expected = req.clone();

        let actions = runtime.classify_request(req);

        assert_single_delegated_request(&actions, &expected);
        assert!(runtime.is_motor_enabled());
        assert!(runtime.is_calibrated());
        assert!(runtime.is_commission_active());
    }
}

#[test]
fn commission_tick_without_engine_is_empty() {
    let runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.pending_commission_kind().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn take_commission_engine_is_none_when_idle() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert!(runtime.take_commission_engine().is_none());
    assert!(!runtime.is_commission_active());
}

#[test]
fn raw_take_preserves_breakaway_cancellation_interrupted_state() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(breakaway_campaign_engine());
    crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
    crate::diagnostic::begin_breakaway_campaign();

    assert!(runtime.take_commission_engine().is_some());

    assert!(!runtime.is_commission_active());
    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        Some(crate::diagnostic::BreakawayBreadcrumb {
            checkpoint: crate::diagnostic::BreakawayCheckpoint::CampaignInstalled,
            interrupted: true,
        })
    );
}

#[test]
fn non_breakaway_handled_stop_preserves_completed_integral_breadcrumb() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(8));
    crate::diagnostic::set_breakaway_breadcrumb_raw_for_test(0);
    crate::diagnostic::complete_breakaway_campaign(
        crate::diagnostic::BreakawayCheckpoint::IntegralHandedOff,
    );

    assert!(
        runtime
            .take_commission_engine_after_handled_stop()
            .is_some()
    );

    assert!(!runtime.is_commission_active());
    assert_eq!(
        crate::diagnostic::current_breakaway_breadcrumb_for_test(),
        Some(crate::diagnostic::BreakawayBreadcrumb {
            checkpoint: crate::diagnostic::BreakawayCheckpoint::IntegralHandedOff,
            interrupted: false,
        })
    );
}

#[test]
fn clear_calibrated_resets_flag() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.clear_calibrated();
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_without_gate_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(false, Relaxed);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_before_calibration_with_auto_calibrate_gate_still_does_not_energize() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    tmc.auto_calibrate_on_enable_allowed.store(true, Relaxed);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn enable_after_calibration_emits_only_arm_preserve_position_check() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, Some(StepperPostAction::ArmPreservePosition));
    assert!(!runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn enable_when_already_enabled_is_noop() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let action = runtime.prepare_enable_request(&tmc);

    assert_eq!(action, None);
    assert!(runtime.is_motor_enabled());
    assert!(runtime.is_calibrated());
}

#[test]
fn disable_request_stops_and_clears_runtime_flags() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);

    let actions = runtime.prepare_disable_request();

    assert_single_stop_now(&actions);
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
}

#[test]
fn diagnostic_trip_clears_runtime_and_step_acceptance_state() {
    let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.blocking_lock();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    tmc.reset();
    stepper.reset();
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    tmc.motor_armed.store(true, Relaxed);
    stepper.timer_active_mirror.store(true, Relaxed);

    runtime.contain_diagnostic_trip();

    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!tmc.motor_armed.load(Relaxed));
    assert!(!stepper.timer_active_mirror.load(Relaxed));
    tmc.reset();
    stepper.reset();
}

#[test]
fn feedforward_tick_writes_velocity_offset_and_marks_active() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let expected = crate::tmc::compute_planner_command_rpm_reference(
            crate::motion_scale::PlannerRpmScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        ) as u32;
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected)]
        );
        assert!(runtime.feedforward_was_active());
        assert!(!tmc_state.velocity_clamp_latched());
    });
}

#[test]
fn feedforward_tick_clamps_velocity_offset_to_command_multiple_and_latches() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(60_000, Relaxed);
        stepper_state.interval.store(8_400_000, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let command = crate::tmc::compute_planner_command_rpm_reference(
            crate::motion_scale::PlannerRpmScale::from_reduced(84_000_000, 1),
            8_400_000,
            false,
        );
        let expected = command * i32::from(crate::tmc::VELOCITY_FF_CLAMP_K);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(command, 10);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), expected as u32)]
        );
        assert!(tmc_state.velocity_clamp_latched());
        assert_eq!(tmc_state.velocity_clamp_count(), 1);
    });
}

#[test]
fn feedforward_tick_clears_velocity_offset_once_after_deactivation() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        runtime.set_feedforward_was_active(true);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );

        assert_eq!(
            first_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(second_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_velocity_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .velocity_transient_feedforward
            .store(true, Relaxed);
        tmc_state
            .velocity_transient_lead_time_us
            .store(1_000, Relaxed);
        tmc_state
            .velocity_transient_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .velocity_transient_max_offset
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 1);
        let active_offset = active_writes[0].1 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        let active_transient_offset = runtime.trace_state().cached_velocity_transient_offset;
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
            assert_ne!(active_transient_offset, 0);
        }

        tmc_state.velocity_feedforward.store(false, Relaxed);
        tmc_state
            .velocity_transient_feedforward
            .store(false, Relaxed);
        let mut failed_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_VELOCITY_OFFSET.addr()));

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                false,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, active_offset);
            assert_eq!(
                runtime.trace_state().cached_velocity_transient_offset,
                active_transient_offset
            );
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![(PID_VELOCITY_OFFSET.addr(), 0)]
        );
        assert!(!runtime.feedforward_was_active());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_velocity_offset, 0);
            assert_eq!(runtime.trace_state().cached_velocity_transient_offset, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );
        assert!(inactive_tmc.into_inner().writes.is_empty());
    });
}

#[test]
fn feedforward_tick_retries_failed_torque_flux_offset_clear() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut active_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut active_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let active_writes = active_tmc.into_inner().writes;
        assert_eq!(active_writes.len(), 1);
        assert_eq!(active_writes[0].0, PID_TORQUE_FLUX_OFFSET.addr());
        let active_offset = ((active_writes[0].1 >> 16) as u16) as i16 as i32;
        assert_ne!(active_offset, 0);
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        let (active_gain, active_flags) = {
            let trace = runtime.trace_state();
            assert_eq!(trace.cached_accel_torque_offset, active_offset);
            assert_ne!(trace.cached_accel_gain_permille, 0);
            assert_ne!(trace.cached_accel_flags, 0);
            (trace.cached_accel_gain_permille, trace.cached_accel_flags)
        };

        tmc_state.accel_feedforward.store(false, Relaxed);
        let mut failed_tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );

        let failed_tick = runtime
            .apply_feedforward_tick(
                &mut failed_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await;

        assert_eq!(failed_tick, Err(()));
        assert!(runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(
                runtime.trace_state().cached_accel_torque_offset,
                active_offset
            );
            assert_eq!(
                runtime.trace_state().cached_accel_gain_permille,
                active_gain
            );
            assert_eq!(runtime.trace_state().cached_accel_flags, active_flags);
        }

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(
            retry_tmc.into_inner().writes,
            std::vec![(PID_TORQUE_FLUX_OFFSET.addr(), 0)]
        );
        assert!(!runtime.torque_flux_offset_was_active_for_test());
        #[cfg(feature = "trace")]
        {
            assert_eq!(runtime.trace_state().cached_accel_torque_offset, 0);
            assert_eq!(runtime.trace_state().cached_accel_gain_permille, 0);
            assert_eq!(runtime.trace_state().cached_accel_flags, 0);
        }

        let mut inactive_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut inactive_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            !inactive_tmc
                .into_inner()
                .attempted_write_to(PID_TORQUE_FLUX_OFFSET)
        );
    });
}

#[test]
fn feedforward_tick_skips_unchanged_velocity_offset_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(first_tmc.into_inner().writes.len(), 1);

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            second_tmc.into_inner().writes.is_empty(),
            "constant cruise must not rewrite an unchanged velocity offset"
        );
    });
}

#[test]
fn feedforward_tick_retries_velocity_offset_after_a_failed_change() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.velocity_feedforward.store(true, Relaxed);
        tmc_state
            .velocity_feedforward_gain_permille
            .store(1_000, Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_feedforward_tick(
                &mut first_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await
            .expect("first tick applies the initial offset");
        let first_offset = first_tmc.into_inner().writes[0].1;

        stepper_state.interval.store(1_400, Relaxed);
        let mut failed_tmc =
            Tmc4671Spi::new(RuntimeMockSpi::default().fail_writes_to(PID_VELOCITY_OFFSET.addr()));
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut failed_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(()),
            "active-tick writes stay best-effort; only inactive-transition clears propagate errors"
        );
        assert_eq!(failed_tmc.into_inner().write_attempts.len(), 1);

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let retry_writes = retry_tmc.into_inner().writes;
        assert_eq!(
            retry_writes.len(),
            1,
            "a failed write must leave the cache invalid so the changed value is retried"
        );
        assert_ne!(retry_writes[0].1, first_offset);
    });
}

#[test]
fn feedforward_tick_skips_unchanged_torque_flux_offset_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut first_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert_eq!(first_tmc.into_inner().writes.len(), 1);

        let mut second_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut second_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        assert!(
            second_tmc.into_inner().writes.is_empty(),
            "steady acceleration must not rewrite an unchanged torque/flux offset"
        );
    });
}

#[test]
fn feedforward_tick_retries_torque_flux_offset_after_a_failed_change() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(200, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut first_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        runtime
            .apply_feedforward_tick(
                &mut first_tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                &config,
                false,
                false,
                true,
            )
            .await
            .expect("first tick applies the initial offset");
        let first_offset = first_tmc.into_inner().writes[0].1;

        stepper_state.interval_add.store(10, Relaxed);
        let mut failed_tmc = Tmc4671Spi::new(
            RuntimeMockSpi::default().fail_writes_to(PID_TORQUE_FLUX_OFFSET.addr()),
        );
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut failed_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(()),
            "active-tick writes stay best-effort; only inactive-transition clears propagate errors"
        );
        assert_eq!(failed_tmc.into_inner().write_attempts.len(), 1);

        let mut retry_tmc = Tmc4671Spi::new(RuntimeMockSpi::default());
        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut retry_tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    true,
                )
                .await,
            Ok(())
        );
        let retry_writes = retry_tmc.into_inner().writes;
        assert_eq!(
            retry_writes.len(),
            1,
            "a failed write must leave the cache invalid so the changed value is retried"
        );
        assert_ne!(retry_writes[0].1, first_offset);
    });
}

#[test]
fn feedforward_fast_tick_skips_torque_flux_offset_slow_work() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        let config = commissioning_runtime_config();
        tmc_state.accel_feedforward.store(true, Relaxed);
        tmc_state
            .accel_feedforward_accel_gain_permille
            .store(1_000, Relaxed);
        tmc_state
            .accel_feedforward_decel_gain_permille
            .store(1_000, Relaxed);
        tmc_state.store_current_limit(current_limit(10_000), Relaxed);
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.interval_add.store(-10, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        assert_eq!(
            runtime
                .apply_feedforward_tick(
                    &mut tmc,
                    TmcControlStateRefs::new(&tmc_state, &stepper_state),
                    &config,
                    false,
                    false,
                    false,
                )
                .await,
            Ok(())
        );

        assert!(!tmc.into_inner().attempted_write_to(PID_TORQUE_FLUX_OFFSET));
    });
}

#[test]
fn command_velocity_reference_zeroes_when_reset_is_requested() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, true);

    assert_eq!(reference, 0);
}

#[test]
fn command_velocity_reference_can_use_single_consumed_reset_latch() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    tmc.feedforward_reset.store(true, Relaxed);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reset = tmc.feedforward_reset.swap(false, Relaxed);
    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, reset);

    assert!(reset);
    assert_eq!(reference, 0);
    assert!(!tmc.feedforward_reset.load(Relaxed));
}

#[test]
fn command_velocity_reference_uses_motion_when_reset_false() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_motor_enabled_for_test(true);
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(
        reference,
        crate::tmc::compute_planner_command_velocity_reference(
            crate::motion_scale::PlannerVelocityScale::from_reduced(84_000_000, 1),
            2_800,
            false,
        )
    );
}

#[test]
fn command_velocity_reference_zeroes_when_runtime_inactive() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let tmc = crate::state::tmc::TmcChannelState::new();
    let stepper = crate::state::stepper::StepperChannelState::new();
    runtime.set_planner_velocity_scale_for_test(84_000_000);
    stepper.interval.store(2_800, Relaxed);
    stepper.direction.store(false, Relaxed);

    let reference =
        runtime.command_velocity_reference(TmcControlStateRefs::new(&tmc, &stepper), false, false);

    assert_eq!(reference, 0);
}

#[test]
fn planner_velocity_scale_accessors_round_trip() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    assert_eq!(runtime.planner_velocity_scale(), None);
    assert_eq!(runtime.planner_rpm_scale(), None);
    runtime.set_planner_velocity_scale_for_test(123_456);
    assert_eq!(
        runtime.planner_velocity_scale(),
        Some(crate::motion_scale::PlannerVelocityScale::from_reduced(
            123_456, 1
        ))
    );
    assert_eq!(
        runtime.planner_rpm_scale(),
        Some(crate::motion_scale::PlannerRpmScale::from_reduced(
            123_456, 1
        ))
    );
}

#[test]
fn phase_advance_state_tracks_restore_to_base_register() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);

    runtime.mark_phase_advance_restored_for_test();

    assert!(!runtime.phase_advance_active_for_test());
    assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
}

#[test]
fn phase_advance_restore_now_writes_base_and_clears_active_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default());

        let restored = runtime.restore_phase_advance_now(&mut tmc).await;

        assert!(restored);
        assert!(!runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), 0x1111_2222)]
        );
    });
}

#[test]
fn phase_advance_restore_now_failure_preserves_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let raw_spi =
            RuntimeMockSpi::default().fail_writes_to(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());
        let mut tmc = Tmc4671Spi::new(raw_spi);

        let restored = runtime.restore_phase_advance_now(&mut tmc).await;

        assert!(!restored);
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
        let raw_spi = tmc.into_inner();
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), 0x1111_2222)]
        );
        assert!(raw_spi.writes.is_empty());
    });
}

#[test]
fn phase_advance_restore_via_backend_writes_base_and_clears_active_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();

        let restored = runtime
            .restore_phase_advance_via_backend(&mut backend)
            .await;

        assert!(restored);
        assert!(!runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x1111_2222);
        assert_eq!(
            backend
                .registers
                .get(&ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            Some(&0x1111_2222)
        );
    });
}

#[test]
fn phase_advance_restore_via_backend_failure_preserves_retry_state() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_phase_advance_for_test(true, 0x1111_2222, 0x3333_4444);
        let mut backend = MockBackend::new();
        backend.fail_on_write = Some(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr());

        let restored = runtime
            .restore_phase_advance_via_backend(&mut backend)
            .await;

        assert!(!restored);
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(runtime.phase_advance_last_register_for_test(), 0x3333_4444);
        assert_eq!(
            backend
                .registers
                .get(&ABN_DECODER_PHI_E_PHI_M_OFFSET.addr()),
            None
        );
    });
}

#[test]
fn phase_advance_tick_snapshots_base_and_writes_adjusted_offset() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.phase_advance_gain_ppm.store(1_000, Relaxed);
        tmc_state.phase_advance_max_counts.store(20, Relaxed);
        tmc_state.phase_advance_deadband.store(0, Relaxed);
        let base_register = 0x0100_2345;
        let command_velocity_reference = 10_000;
        let phase_output = crate::tmc::compute_phase_advance_counts(
            command_velocity_reference,
            crate::tmc::PhaseAdvanceConfig {
                enable: true,
                gain_ppm: 1_000,
                max_counts: 20,
                deadband_velocity: 0,
            },
        );
        let expected_register = crate::tmc::pack_abn_phi_offset(base_register, phase_output.counts);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(base_register));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                command_velocity_reference,
                false,
                false,
            )
            .await;

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.read_requested(ABN_DECODER_PHI_E_PHI_M_OFFSET));
        assert_eq!(
            raw_spi.write_attempts,
            std::vec![(ABN_DECODER_PHI_E_PHI_M_OFFSET.addr(), expected_register)]
        );
        assert!(runtime.phase_advance_active_for_test());
        assert_eq!(
            runtime.phase_advance_last_register_for_test(),
            expected_register
        );
    });
}

#[test]
fn phase_advance_tick_reset_gate_skips_abn_snapshot_and_write() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                123,
                false,
                true,
            )
            .await;

        assert!(!runtime.phase_advance_active_for_test());
        let raw_spi = tmc.into_inner();
        assert!(!raw_spi.read_requested(ABN_DECODER_PHI_E_PHI_M_OFFSET));
        assert!(!raw_spi.attempted_write_to(ABN_DECODER_PHI_E_PHI_M_OFFSET));
    });
}

#[test]
fn phase_and_position_tick_disabled_runtime_writes_nothing() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.phase_advance.store(true, Relaxed);
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.position_lead_gain_permille.store(1000, Relaxed);
        tmc_state.position_lead_max_counts.store(20, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
        stepper_state.interval.store(2_800, Relaxed);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                123,
                false,
                false,
            )
            .await;

        let raw_spi = tmc.into_inner();
        assert!(raw_spi.read_requests.is_empty());
        assert!(raw_spi.write_attempts.is_empty());
        assert_eq!(runtime.position_lead_applied_units_for_test(), 0);
    });
}

#[test]
fn position_lead_tick_uses_planner_position_units() {
    pollster::block_on(async {
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_planner_velocity_scale_for_test(84_000_000);
        let tmc_state = crate::state::tmc::TmcChannelState::new();
        let stepper_state = crate::state::stepper::StepperChannelState::new();
        tmc_state.position_lead.store(true, Relaxed);
        tmc_state.position_lead_gain_permille.store(1000, Relaxed);
        tmc_state.position_lead_max_counts.store(20, Relaxed);
        tmc_state
            .configure_motion_scale(&stepper_state, 1_000, 3_200)
            .unwrap();
        stepper_state.interval.store(2_800, Relaxed);
        stepper_state.direction.store(false, Relaxed);
        let lead_counts =
            crate::tmc::compute_position_lead_counts_from_reference(30_000, false, 1000, 20);
        let (expected_target, expected_applied_units) =
            crate::tmc::compute_position_lead_target(1_000, 0, lead_counts, 3_200);
        let mut tmc = Tmc4671Spi::new(RuntimeMockSpi::default().with_read(1_000));

        runtime
            .apply_phase_and_position_tick(
                &mut tmc,
                TmcControlStateRefs::new(&tmc_state, &stepper_state),
                30_000,
                false,
                false,
            )
            .await;

        assert_eq!(
            runtime.position_lead_applied_units_for_test(),
            expected_applied_units
        );
        assert_eq!(
            tmc.into_inner().writes,
            std::vec![(PID_POSITION_TARGET.addr(), expected_target)]
        );
    });
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_returns_sample_before_incremented_control_tick() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 0);
    assert_eq!(tick.control_tick, 1);
    assert!(!tick.status_detail_due);
    assert!(!tick.metadata_due);
    assert!(!tick.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_status_detail_due_at_100() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(99);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 99);
    assert_eq!(tick.control_tick, 100);
    assert!(tick.status_detail_due);
    assert!(!tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_marks_metadata_due_at_1000() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(999);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.sample_tick, 999);
    assert_eq!(tick.control_tick, 1000);
    assert!(tick.status_detail_due);
    assert!(tick.metadata_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_consumes_phase_advance_one_shot_once() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_phase_advance_for_test(true, 0x01, 7);

    let first = runtime.advance_trace_tick();
    let second = runtime.advance_trace_tick();

    assert!(first.phase_advance_due);
    assert_eq!(first.phase_advance_flags, 0x01);
    assert_eq!(first.phase_advance_counts, 7);
    assert!(!second.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_emits_phase_advance_periodically_when_nonzero() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(9);
    runtime.set_trace_phase_advance_for_test(false, 0x02, 5);

    let due = runtime.advance_trace_tick();
    let not_due = runtime.advance_trace_tick();

    assert_eq!(due.control_tick, 10);
    assert!(due.phase_advance_due);
    assert_eq!(due.phase_advance_flags, 0x02);
    assert_eq!(due.phase_advance_counts, 5);
    assert_eq!(not_due.control_tick, 11);
    assert!(!not_due.phase_advance_due);
}

#[cfg(feature = "trace")]
#[test]
fn trace_tick_does_not_emit_periodic_phase_advance_when_zero() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.set_trace_control_tick_for_test(9);
    runtime.set_trace_phase_advance_for_test(false, 0, 0);

    let tick = runtime.advance_trace_tick();

    assert_eq!(tick.control_tick, 10);
    assert!(!tick.phase_advance_due);
}

// --- terminal-restore cancellation ------------------------------------------
//
// A terminal restore survives across ticks, so a shutdown, queue flush,
// emergency disable, or host-lease expiry arriving mid-restore must drive the
// motor to safe state rather than dropping the machine on a half-armed motor.
// One test per exit source, plus the shared invariant that the slots stay
// populated until the drain has actually run.

fn zero_cal_result() -> crate::commissioning::types::AdcCalResult {
    crate::commissioning::types::AdcCalResult {
        adc_i0: 0,
        adc_i1: 0,
        encoder_count: 0,
        encoder_alignment_electrical_residual_counts: 0,
        encoder_alignment_stability_counts: 0,
        encoder_alignment_movement_counts: 0,
        encoder_alignment_min_movement_counts: 0,
        encoder_alignment_counts_per_electrical_rev: 0,
        adc_residual_chop0: crate::commissioning::types::AdcResidualEvidence::EMPTY,
        adc_residual_chop7: crate::commissioning::types::AdcResidualEvidence::EMPTY,
    }
}

/// Install an armed motor with a terminal restore started but not yet advanced.
fn armed_runtime_with_restore_started(runtime: &mut TmcControlRuntime, backend: &mut MockBackend) {
    use crate::commissioning::backend::CommissionMotorControl;

    let tmc = crate::state::tmc::tmc_channel0();
    tmc.mark_pi_representation_established();
    tmc.motor_armed.store(true, Relaxed);
    tmc.commission_in_flight.store(true, Relaxed);
    backend.set_enable_pin(true);
    runtime.set_calibrated_for_test(true);
    runtime.set_motor_enabled_for_test(true);
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(0));

    runtime.begin_commissioning_terminal(
        crate::commissioning::schedule::CommissionTerminal::CalibrateDone(zero_cal_result()),
        &commissioning_runtime_config(),
    );
    assert!(runtime.is_restore_pending());
    assert!(runtime.is_commission_active());
}

/// Install an armed motor with a terminal restore stopped part-way through.
async fn armed_runtime_mid_restore(
    runtime: &mut TmcControlRuntime,
    backend: &mut MockBackend,
    slot: &mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
) {
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    armed_runtime_with_restore_started(runtime, backend);

    let mut retention = crate::commission_dispatch::CommissionRetention::new(slot);
    assert!(
        runtime
            .poll_commissioning_terminal(backend, &mut retention, tmc, stepper)
            .await
            .is_none(),
        "the calibrate restore must need more than one poll"
    );
    assert!(runtime.is_restore_pending());
    assert!(runtime.is_commission_active());
}

/// Drain a latched cancel and assert the motor reached safe state with both
/// slots retired and no further restore work left to do.
async fn assert_drain_reaches_safe_state(
    runtime: &mut TmcControlRuntime,
    backend: &mut MockBackend,
    slot: &mut crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot,
) {
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();

    assert!(
        runtime.restore_cancel_pending(),
        "the exit must latch a cancel before the drain runs"
    );
    assert!(
        runtime.is_restore_pending() && runtime.is_commission_active(),
        "both slots must survive until the drain has run"
    );

    let _ = runtime
        .drive_restore_cancel(
            backend,
            tmc,
            stepper,
            crate::autotune::CommissionError::Shutdown,
        )
        .await;

    assert!(!backend.enable_pin, "the motor must be de-energized");
    assert!(!tmc.motor_armed.load(Relaxed));
    assert!(!runtime.is_motor_enabled());
    assert!(!runtime.is_calibrated());
    assert!(!tmc.commission_in_flight.load(Relaxed));
    assert!(
        !runtime.is_restore_pending(),
        "restore machine must be gone"
    );
    assert!(!runtime.is_commission_active(), "engine slot must be gone");
    assert!(!runtime.restore_cancel_pending());

    // A later tick must not advance any restore or re-assert the enable pin.
    let before = backend.spi_log.len();
    let mut retention = crate::commission_dispatch::CommissionRetention::new(slot);
    assert!(
        runtime
            .poll_commissioning_terminal(backend, &mut retention, tmc, stepper)
            .await
            .is_none()
    );
    assert_eq!(backend.spi_log.len(), before, "a later tick must be inert");
    assert!(!backend.enable_pin);
}

fn cancel_test_signals<'a>(
    shutdown_active: &'a AtomicBool,
    queue_flush: &'a AtomicBool,
    fault_active: &'a AtomicBool,
    disable_signal: &'a Signal<CriticalSectionRawMutex, ()>,
) -> TmcControlSignals<'a> {
    TmcControlSignals::new(shutdown_active, queue_flush, fault_active, disable_signal)
}

#[test]
fn shutdown_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let shutdown_active = AtomicBool::new(true);
        let queue_flush = AtomicBool::new(false);
        let fault_active = AtomicBool::new(false);
        let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
        let signals = cancel_test_signals(
            &shutdown_active,
            &queue_flush,
            &fault_active,
            &disable_signal,
        );
        let _ = runtime.poll_shutdown(&signals);

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn shutdown_safe_state_drains_a_pending_restore_before_disarming() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        // The board's shutdown chokepoint: it must drain the restore itself,
        // without any board-side cancel request.
        assert!(!runtime.restore_cancel_pending());
        crate::tmc_control::lifecycle::apply_shutdown_safe_state(
            &mut runtime,
            &mut backend,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        )
        .await;

        assert!(!backend.enable_pin);
        assert!(!crate::state::tmc::tmc_channel0().motor_armed.load(Relaxed));
        assert!(!runtime.is_motor_enabled());
        assert!(!runtime.is_restore_pending());
        assert!(!runtime.is_commission_active());
        assert!(!runtime.restore_cancel_pending());
    });
}

#[test]
fn queue_flush_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let shutdown_active = AtomicBool::new(false);
        let queue_flush = AtomicBool::new(true);
        let fault_active = AtomicBool::new(false);
        let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
        let signals = cancel_test_signals(
            &shutdown_active,
            &queue_flush,
            &fault_active,
            &disable_signal,
        );
        let _ = runtime.poll_queue_flush(
            &signals,
            crate::state::tmc::tmc_channel0(),
            crate::state::stepper::stepper_channel0(),
        );

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn emergency_disable_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let shutdown_active = AtomicBool::new(false);
        let queue_flush = AtomicBool::new(false);
        let fault_active = AtomicBool::new(false);
        let disable_signal = Signal::<CriticalSectionRawMutex, ()>::new();
        disable_signal.signal(());
        let signals = cancel_test_signals(
            &shutdown_active,
            &queue_flush,
            &fault_active,
            &disable_signal,
        );
        // `poll_emergency_disable` never owned the engine slot; the board's exit
        // path latches the cancel, exactly as the commissioning loop does.
        let _ = runtime.poll_emergency_disable(&signals, Ok(0));
        runtime.request_restore_cancel();

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn lease_expiry_mid_restore_drains_to_safe_state_before_clearing_slots() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        // Host-lease expiry is a board-side classification with no runtime
        // helper of its own; its only obligation is the latch.
        runtime.request_restore_cancel();

        assert_drain_reaches_safe_state(&mut runtime, &mut backend, &mut slot).await;
    });
}

#[test]
fn request_restore_cancel_without_a_restore_is_inert() {
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    runtime.request_restore_cancel();
    assert!(!runtime.restore_cancel_pending());
    assert!(!runtime.is_restore_pending());
}

#[test]
fn cancelled_mid_restore_reports_cancelled_status_not_shutdown() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();
        let mut slot =
            crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new(
            );
        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        armed_runtime_mid_restore(&mut runtime, &mut backend, &mut slot).await;

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.cancel_requested.store(true, Relaxed);

        let pwm_maxcnt = commissioning_runtime_config().tmc.pwm_maxcnt;
        let outcome = runtime
            .drive_operator_cancel(&mut backend, tmc, stepper, pwm_maxcnt)
            .await
            .expect("a cancel latched mid-restore must produce a terminal outcome");

        // armed_runtime_mid_restore's fixture is a CalibrateDone restore, so
        // the reply is a CalibrateResult whose status is the calibration-side
        // mapping of the status this drain was told to report -- Cancelled,
        // not the InternalError fallback Shutdown maps to (Shutdown has no
        // explicit CalibrationError arm).
        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, .. } => match reply {
                crate::commission_dispatch::ReplyPayload::CalibrateResult { status, .. } => {
                    assert_eq!(
                        status,
                        crate::calibration::CalibrationError::Cancelled.status_code(),
                        "a mid-restore operator cancel must report Cancelled, not Shutdown"
                    );
                }
                _ => panic!("expected a CalibrateResult reply from a CalibrateDone restore"),
            },
            _ => panic!("expected a Terminal outcome"),
        }

        assert!(!backend.enable_pin, "the motor must be de-energized");
        assert!(!tmc.commission_in_flight.load(Relaxed));
        assert!(
            !tmc.cancel_requested.load(Relaxed),
            "the cancel flag must be consumed"
        );
        assert!(!runtime.is_restore_pending());
        assert!(!runtime.is_commission_active());
    });
}

#[test]
fn cancelled_mid_phase_disarms_only_the_targeted_channel_and_reports_cancelled() {
    pollster::block_on(async {
        let _guard = crate::commission_dispatch::test_support::SHARED_STATE_MUTEX
            .lock()
            .await;
        crate::state::tmc::reset_all_tmc_channels();
        crate::state::stepper::reset_all_stepper_channels();

        use crate::commissioning::backend::CommissionMotorControl;

        let tmc = crate::state::tmc::tmc_channel0();
        let stepper = crate::state::stepper::stepper_channel0();
        tmc.mark_pi_representation_established();
        tmc.motor_armed.store(true, Relaxed);
        tmc.commission_in_flight.store(true, Relaxed);
        tmc.cancel_requested.store(true, Relaxed);

        let mut runtime = TmcControlRuntime::new_for_channel(0);
        let mut backend = MockBackend::new();
        backend.set_enable_pin(true);
        runtime.set_calibrated_for_test(true);
        runtime.set_motor_enabled_for_test(true);
        runtime.set_commission_engine(CommissioningEngine::new_commission(
            7,
            crate::autotune::CommissionProfile::Balanced,
        ));
        assert!(runtime.is_commission_active());
        assert!(!runtime.is_restore_pending(), "no restore machine yet");

        let pwm_maxcnt = commissioning_runtime_config().tmc.pwm_maxcnt;
        let outcome = runtime
            .drive_operator_cancel(&mut backend, tmc, stepper, pwm_maxcnt)
            .await
            .expect("a cancel latched mid-phase must produce a terminal outcome");

        match outcome {
            crate::commission_dispatch::CommissionOutcome::Terminal { reply, .. } => match reply {
                crate::commission_dispatch::ReplyPayload::CommissionResult { status, .. } => {
                    assert_eq!(
                        status,
                        crate::autotune::CommissionError::Cancelled.status_code()
                    );
                }
                _ => panic!("expected a CommissionResult reply from a Commission-kind engine"),
            },
            _ => panic!("expected a Terminal outcome"),
        }

        assert!(
            !backend.enable_pin,
            "channel 0's motor must be de-energized"
        );
        assert!(!tmc.motor_armed.load(Relaxed));
        assert!(!tmc.commission_in_flight.load(Relaxed));
        assert!(
            !tmc.cancel_requested.load(Relaxed),
            "the cancel flag must be consumed"
        );
        assert!(!runtime.is_commission_active());
        assert!(!runtime.is_restore_pending());

        let other_channel = crate::state::tmc::tmc_channel(1)
            .expect("a second channel's static state must exist on this build");
        assert!(
            !other_channel.motor_armed.load(Relaxed),
            "the other channel's motor state must be untouched"
        );
        assert!(
            !other_channel.commission_in_flight.load(Relaxed),
            "the other channel's commissioning state must be untouched"
        );
        assert!(
            !other_channel.cancel_requested.load(Relaxed),
            "the other channel's cancel flag must be untouched"
        );
    });
}

/// Assert the drop landed inside a real backend operation: the verified write
/// that was in flight had already pushed its register write, and its verifying
/// read-back had not run yet.
fn assert_dropped_mid_operation(backend: &MockBackend, ops_before: usize) {
    assert!(
        backend.spi_log.len() > ops_before,
        "the suspension must interrupt a real backend operation, not gate entry to one"
    );
    assert!(
        matches!(backend.spi_log.last(), Some(SpiOp::Write(_, _))),
        "the interrupted verified write must have landed its write and not yet read back, \
         got {:?}",
        backend.spi_log.last()
    );
}

/// A waker that does nothing; the tests below poll a future by hand exactly
/// once and never wait for a wake-up.
struct NoopWake;

impl std::task::Wake for NoopWake {
    fn wake(self: std::sync::Arc<Self>) {}
}

fn noop_waker() -> std::task::Waker {
    std::task::Waker::from(std::sync::Arc::new(NoopWake))
}

/// Acquire the shared-state guard and reset the channel atomics without an
/// enclosing `block_on`, so the caller can poll futures by hand afterwards.
fn lock_and_reset_shared_state()
-> impl core::ops::Deref<Target = ()> + core::ops::DerefMut + 'static {
    let guard =
        pollster::block_on(crate::commission_dispatch::test_support::SHARED_STATE_MUTEX.lock());
    crate::state::tmc::reset_all_tmc_channels();
    crate::state::stepper::reset_all_stepper_channels();
    guard
}

#[test]
fn a_torn_down_restore_poll_keeps_both_slots_installed() {
    // The restore machine must live in the runtime slot for every suspension
    // point of a poll, not in a local moved out of it. Dropping an in-flight
    // poll is the observable form of that: with the machine moved into the
    // future, the drop would destroy it and the runtime would report no
    // pending restore while the motor is still mid-sequence.
    let _guard = lock_and_reset_shared_state();
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    let mut slot =
        crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut backend = MockBackend::new();
    armed_runtime_with_restore_started(&mut runtime, &mut backend);
    // Restart as an InnerFailed terminal: its first step is the safe-state
    // sequence, whose verified writes are where the mock can suspend.
    runtime.request_restore_cancel();
    pollster::block_on(runtime.drive_restore_cancel(
        &mut backend,
        tmc,
        stepper,
        crate::autotune::CommissionError::Shutdown,
    ));
    runtime.set_commission_engine(CommissioningEngine::new_calibrate(0));
    runtime.begin_commissioning_terminal(
        crate::commissioning::schedule::CommissionTerminal::InnerFailed(
            crate::autotune::CommissionError::EncoderFault,
        ),
        &commissioning_runtime_config(),
    );

    backend.pend_next_op = true;
    let ops_before = backend.spi_log.len();
    let suspended = {
        let mut retention = crate::commission_dispatch::CommissionRetention::new(&mut slot);
        let mut future = std::boxed::Box::pin(runtime.poll_commissioning_terminal(
            &mut backend,
            &mut retention,
            tmc,
            stepper,
        ));
        let waker = noop_waker();
        let mut context = std::task::Context::from_waker(&waker);
        matches!(future.as_mut().poll(&mut context), std::task::Poll::Pending)
        // The future is dropped here, mid-await.
    };

    assert!(
        suspended,
        "the mock must suspend so the poll is genuinely in flight when dropped"
    );
    assert_dropped_mid_operation(&backend, ops_before);
    assert!(
        runtime.is_restore_pending(),
        "a torn-down poll must not lose the restore machine"
    );
    assert!(
        runtime.is_commission_active(),
        "a torn-down poll must not lose the engine"
    );
}

#[test]
fn a_torn_down_restore_cancel_keeps_the_slots_and_latch_and_stays_resumable() {
    // Same ownership contract for the cancel drain, which runs the mandatory
    // safe-state sequence: neither slot nor the latch may be retired until
    // that sequence has actually run, and a torn-down drain must be resumable.
    let _guard = lock_and_reset_shared_state();
    let tmc = crate::state::tmc::tmc_channel0();
    let stepper = crate::state::stepper::stepper_channel0();
    let mut slot =
        crate::commissioning::outer::velocity::integral_reproduction::IntegralRetentionSlot::new();
    let mut runtime = TmcControlRuntime::new_for_channel(0);
    let mut backend = MockBackend::new();
    pollster::block_on(armed_runtime_mid_restore(
        &mut runtime,
        &mut backend,
        &mut slot,
    ));
    runtime.request_restore_cancel();

    backend.pend_next_op = true;
    let ops_before = backend.spi_log.len();
    let suspended = {
        let mut future = std::boxed::Box::pin(runtime.drive_restore_cancel(
            &mut backend,
            tmc,
            stepper,
            crate::autotune::CommissionError::Shutdown,
        ));
        let waker = noop_waker();
        let mut context = std::task::Context::from_waker(&waker);
        matches!(future.as_mut().poll(&mut context), std::task::Poll::Pending)
    };

    assert!(
        suspended,
        "the mock must suspend so the drain is genuinely in flight when dropped"
    );
    assert_dropped_mid_operation(&backend, ops_before);
    assert!(
        runtime.is_restore_pending(),
        "a torn-down drain must not lose the restore machine"
    );
    assert!(
        runtime.is_commission_active(),
        "a torn-down drain must not lose the engine"
    );
    assert!(
        runtime.restore_cancel_pending(),
        "a torn-down drain must leave the cancel latched so it is retried"
    );
    // The retry the latch guarantees still reaches safe state.
    pollster::block_on(assert_drain_reaches_safe_state(
        &mut runtime,
        &mut backend,
        &mut slot,
    ));
}
