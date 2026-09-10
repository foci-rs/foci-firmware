use super::pi_representation::{
    PiRepresentationError, PiRepresentationVerifyStep, bootstrap_pi_representation,
    verify_pi_representation,
};
use crate::commission_dispatch::test_support::{MockBackend, SpiOp};
use crate::state::tmc::TmcChannelState;
use foci_tmc4671::registers::{ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12, CONFIG_ADVANCED_PI_REPRESENT};

#[test]
fn pi_representation_bootstrap_writes_reads_and_publishes_in_order() {
    pollster::block_on(async {
        let state = TmcChannelState::new();
        let mut backend = MockBackend::new();

        bootstrap_pi_representation(&mut backend, &state)
            .await
            .unwrap();

        assert!(state.pi_representation_established());
        assert_eq!(backend.spi_log.len(), 4);
        assert!(matches!(
            backend.spi_log[0],
            SpiOp::Write(addr, value)
                if addr == CONFIG_ADVANCED_PI_REPRESENT.addr.addr()
                    && value == u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr)
        ));
        assert!(matches!(
            backend.spi_log[1],
            SpiOp::Write(addr, value)
                if addr == CONFIG_ADVANCED_PI_REPRESENT.data.addr()
                    && value == ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12
        ));
        assert!(matches!(
            backend.spi_log[2],
            SpiOp::Write(addr, value)
                if addr == CONFIG_ADVANCED_PI_REPRESENT.addr.addr()
                    && value == u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr)
        ));
        assert!(matches!(backend.spi_log[3], SpiOp::Read));
        assert!(
            !backend
                .spi_log
                .iter()
                .any(|op| matches!(op, SpiOp::EnablePin(_)))
        );
    });
}

#[test]
fn pi_representation_bootstrap_fails_closed_for_each_io_failure_and_mismatch() {
    pollster::block_on(async {
        let cases = [
            (
                Some(CONFIG_ADVANCED_PI_REPRESENT.addr.addr()),
                None,
                None,
                PiRepresentationError::Write,
            ),
            (
                Some(CONFIG_ADVANCED_PI_REPRESENT.data.addr()),
                None,
                None,
                PiRepresentationError::Write,
            ),
            (
                None,
                Some(CONFIG_ADVANCED_PI_REPRESENT.data.addr()),
                None,
                PiRepresentationError::Read,
            ),
            (
                None,
                None,
                Some((CONFIG_ADVANCED_PI_REPRESENT.data.addr(), 0)),
                PiRepresentationError::Mismatch { actual: 0 },
            ),
        ];

        for (fail_write, fail_read, read_override, expected) in cases {
            let state = TmcChannelState::new();
            state.mark_pi_representation_established();
            let mut backend = MockBackend::new();
            backend.fail_on_write = fail_write;
            backend.fail_on_read = fail_read;
            backend.read_override = read_override;

            assert_eq!(
                bootstrap_pi_representation(&mut backend, &state).await,
                Err(expected)
            );
            assert!(!state.pi_representation_established());
            assert!(
                !backend
                    .spi_log
                    .iter()
                    .any(|op| matches!(op, SpiOp::EnablePin(_)))
            );
        }
    });
}

#[test]
fn pi_representation_bootstrap_is_deterministic_when_repeated_disarmed() {
    pollster::block_on(async {
        let state = TmcChannelState::new();
        let mut backend = MockBackend::new();

        bootstrap_pi_representation(&mut backend, &state)
            .await
            .unwrap();
        bootstrap_pi_representation(&mut backend, &state)
            .await
            .unwrap();

        assert!(state.pi_representation_established());
        assert_eq!(backend.spi_log.len(), 8);
        for pair in backend.spi_log.chunks_exact(4) {
            assert!(matches!(
                pair[0],
                SpiOp::Write(addr, value)
                    if addr == CONFIG_ADVANCED_PI_REPRESENT.addr.addr()
                        && value == u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr)
            ));
            assert!(matches!(
                pair[1],
                SpiOp::Write(addr, value)
                    if addr == CONFIG_ADVANCED_PI_REPRESENT.data.addr()
                        && value == ADVANCED_PI_REPRESENT_P_Q8_8_I_Q4_12
            ));
            assert!(matches!(
                pair[2],
                SpiOp::Write(addr, value)
                    if addr == CONFIG_ADVANCED_PI_REPRESENT.addr.addr()
                        && value == u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr)
            ));
            assert!(matches!(pair[3], SpiOp::Read));
        }
    });
}

#[test]
fn pi_representation_verify_requires_established_state_before_spi() {
    pollster::block_on(async {
        let state = TmcChannelState::new();
        let mut backend = MockBackend::new();

        assert_eq!(
            verify_pi_representation(&mut backend, &state).await,
            Err(PiRepresentationError::NotEstablished)
        );
        assert!(backend.spi_log.is_empty());
    });
}

#[test]
fn pi_representation_verify_step_advances_one_raw_op_per_call() {
    pollster::block_on(async {
        let state = TmcChannelState::new();
        let mut backend = MockBackend::new();
        crate::tmc_control::pi_representation::bootstrap_pi_representation(&mut backend, &state)
            .await
            .expect("bootstrap should succeed");

        let mut step = PiRepresentationVerifyStep::start();
        let mut calls = 0u32;
        let result = loop {
            assert!(calls < 4, "step sequence did not terminate");
            let before = backend.spi_log.len();
            let outcome = step.advance(&mut backend, &state).await;
            let delta = &backend.spi_log[before..];
            assert_eq!(
                delta.len(),
                1,
                "call {calls} logged {delta:?}, expected exactly one raw op"
            );
            match calls {
                0 => assert!(
                    matches!(
                        delta[0],
                        SpiOp::Write(addr, value)
                            if addr == CONFIG_ADVANCED_PI_REPRESENT.addr.addr()
                                && value == u32::from(CONFIG_ADVANCED_PI_REPRESENT.sub_addr)
                    ),
                    "call 0 expected the selector write, got {delta:?}"
                ),
                1 => assert!(
                    matches!(delta[0], SpiOp::Read),
                    "call 1 expected the read-back, got {delta:?}"
                ),
                _ => unreachable!("loop bound above caps calls at 4"),
            }
            calls += 1;
            if let Some(result) = outcome {
                break result;
            }
        };
        assert_eq!(calls, 2, "expected exactly 2 steps");
        assert!(result.is_ok());
    });
}
