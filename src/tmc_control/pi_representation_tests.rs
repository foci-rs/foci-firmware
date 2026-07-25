use super::pi_representation::{PiRepresentationError, bootstrap_pi_representation};
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
