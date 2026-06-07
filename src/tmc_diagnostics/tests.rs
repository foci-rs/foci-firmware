use super::*;

#[test]
fn current_step_encoder_delta_wraps_by_counts_per_rev() {
    assert_eq!(current_step_encoder_delta(90, 10, 100), 20);
    assert_eq!(current_step_encoder_delta(10, 90, 100), -20);
    assert_eq!(current_step_encoder_delta(10, 40, 100), 30);
    assert_eq!(current_step_encoder_delta(40, 10, 100), -30);
    assert_eq!(current_step_encoder_delta(10, 90, 0), 80);
}

#[test]
fn pack_voltage_ext_preserves_signed_halves() {
    assert_eq!(pack_voltage_ext(0x1234, -2), 0x1234_fffe);
    assert_eq!(pack_voltage_ext(-1, 0x4567), 0xffff_4567);
}

#[test]
fn empty_reports_preserve_headers_and_zero_samples() {
    let torque =
        empty_current_torque_sample_report(7, CURRENT_STEP_STATUS_NOT_READY, -11, 22, 3, 44);
    assert_eq!(torque.oid, 7);
    assert_eq!(torque.status, CURRENT_STEP_STATUS_NOT_READY);
    assert_eq!(torque.target, -11);
    assert_eq!(torque.flux_target, 22);
    assert_eq!(torque.sample_delay_ms, 3);
    assert_eq!(torque.voltage_limit, 44);
    assert_eq!(torque.torque_sample, 0);
    assert_eq!(torque.status_flags, 0);

    let voltage = empty_voltage_step_report(8, CURRENT_STEP_STATUS_SPI, -33, 44, 5);
    assert_eq!(voltage.oid, 8);
    assert_eq!(voltage.status, CURRENT_STEP_STATUS_SPI);
    assert_eq!(voltage.uq_ext, -33);
    assert_eq!(voltage.ud_ext, 44);
    assert_eq!(voltage.sample_delay_ms, 5);
    assert_eq!(voltage.encoder_delta_after, 0);
}
