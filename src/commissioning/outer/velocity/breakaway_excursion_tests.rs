use super::{ProbeExcursionCapture, ProbeExcursionError, ProbeMotionClass};
use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::encoder_domain::EncoderDomain;

fn domain() -> EncoderDomain {
    EncoderDomain::new(4_000).unwrap()
}

#[test]
fn excursion_capture_initializes_the_zero_baseline() {
    let capture = ProbeExcursionCapture::new(2_000, 100);

    assert_eq!(capture.wire_fields(), Ok((0, 0, 0)));
    assert_eq!(
        capture.finish(VelocityDirection::Forward, 63, 2_000),
        Err(ProbeExcursionError::CaptureInterval)
    );
}

#[test]
fn excursion_capture_folds_forward_and_reverse_encoder_wrap() {
    let mut forward = ProbeExcursionCapture::new(3_998, 0);
    forward.observe(domain(), 2, 1_000).unwrap();
    forward.observe(domain(), 65, 2_000).unwrap();
    let forward = forward
        .finish(VelocityDirection::Forward, 63, 2_000)
        .unwrap();
    assert_eq!(forward.minimum_counts, 0);
    assert_eq!(forward.maximum_counts, 67);
    assert_eq!(forward.motion, ProbeMotionClass::Moved);

    let mut reverse = ProbeExcursionCapture::new(2, 0);
    reverse.observe(domain(), 3_998, 1_000).unwrap();
    reverse.observe(domain(), 3_935, 2_000).unwrap();
    let reverse = reverse
        .finish(VelocityDirection::Reverse, 63, 2_000)
        .unwrap();
    assert_eq!(reverse.minimum_counts, -67);
    assert_eq!(reverse.maximum_counts, 0);
    assert_eq!(reverse.motion, ProbeMotionClass::Moved);
}

#[test]
fn excursion_threshold_is_inclusive_and_direction_aware() {
    for (excursion, expected) in [
        (62, ProbeMotionClass::NoMotion),
        (63, ProbeMotionClass::Moved),
    ] {
        let mut capture = ProbeExcursionCapture::new(0, 0);
        capture.observe(domain(), excursion, 1_000).unwrap();
        assert_eq!(
            capture
                .finish(VelocityDirection::Forward, 63, 2_000)
                .unwrap()
                .motion,
            expected
        );
    }

    let mut opposite = ProbeExcursionCapture::new(0, 0);
    opposite.observe(domain(), 3_930, 1_000).unwrap();
    assert_eq!(
        opposite
            .finish(VelocityDirection::Forward, 63, 2_000)
            .unwrap()
            .motion,
        ProbeMotionClass::WrongDirection
    );
}

#[test]
fn commanded_motion_wins_when_both_excursions_cross_threshold() {
    let mut capture = ProbeExcursionCapture::new(0, 0);
    capture.observe(domain(), 70, 1_000).unwrap();
    capture.observe(domain(), 3_900, 2_000).unwrap();

    let evidence = capture
        .finish(VelocityDirection::Forward, 63, 2_000)
        .unwrap();
    assert_eq!(evidence.minimum_counts, -100);
    assert_eq!(evidence.maximum_counts, 70);
    assert_eq!(evidence.motion, ProbeMotionClass::Moved);
}

#[test]
fn excursion_capture_interval_bound_is_inclusive() {
    let mut accepted = ProbeExcursionCapture::new(0, 0);
    accepted.observe(domain(), 63, 2_000).unwrap();
    assert_eq!(
        accepted
            .finish(VelocityDirection::Forward, 63, 2_000)
            .unwrap()
            .maximum_interval_us,
        2_000
    );

    let mut refused = ProbeExcursionCapture::new(0, 0);
    refused.observe(domain(), 63, 2_001).unwrap();
    assert_eq!(
        refused.finish(VelocityDirection::Forward, 63, 2_000),
        Err(ProbeExcursionError::CaptureInterval)
    );

    let mut zero = ProbeExcursionCapture::new(0, 1_000);
    assert_eq!(
        zero.observe(domain(), 63, 1_000),
        Err(ProbeExcursionError::CaptureInterval)
    );
}

#[test]
fn excursion_capture_rejects_checked_accumulator_overflow() {
    let mut capture = ProbeExcursionCapture::new(0, 0);
    capture.displacement_counts = i64::MAX;

    assert_eq!(
        capture.observe(domain(), 1, 1_000),
        Err(ProbeExcursionError::Arithmetic)
    );
}

#[test]
fn zero_excursion_is_ordinary_no_motion() {
    let mut capture = ProbeExcursionCapture::new(1_234, 0);
    capture.observe(domain(), 1_234, 1_000).unwrap();

    assert_eq!(
        capture
            .finish(VelocityDirection::Forward, 63, 2_000)
            .unwrap()
            .motion,
        ProbeMotionClass::NoMotion
    );
}
