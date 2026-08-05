use super::super::sweep_plan::MAX_SWEEP_RUNGS;
use super::{BreakawaySearchError, ProbeStrokeRole, UpwardBreakawayCursor, UpwardProbeStep};
use crate::autotune::velocity_measurement::VelocityDirection;

fn expect_upward_probe(step: UpwardProbeStep, rung_index: u8, role: ProbeStrokeRole) {
    assert_eq!(step, UpwardProbeStep::Probe { rung_index, role });
}

#[test]
fn upward_probe_roles_pin_direction_and_slot_index() {
    assert_eq!(
        ProbeStrokeRole::InitialForward.direction(),
        VelocityDirection::Forward
    );
    assert_eq!(
        ProbeStrokeRole::InitialReverse.direction(),
        VelocityDirection::Reverse
    );
    assert_eq!(
        ProbeStrokeRole::RepeatForward.direction(),
        VelocityDirection::Forward
    );
    assert_eq!(
        ProbeStrokeRole::RepeatReverse.direction(),
        VelocityDirection::Reverse
    );
    assert_eq!(ProbeStrokeRole::InitialForward.slot_index(), 0);
    assert_eq!(ProbeStrokeRole::InitialReverse.slot_index(), 0);
    assert_eq!(ProbeStrokeRole::RepeatForward.slot_index(), 1);
    assert_eq!(ProbeStrokeRole::RepeatReverse.slot_index(), 1);
}

#[test]
fn upward_probe_all_no_motion_consumes_two_strokes_per_rung() {
    for rung_count in 1..=MAX_SWEEP_RUNGS as u8 {
        let (mut cursor, mut step) = UpwardBreakawayCursor::start(rung_count).unwrap();
        let mut expected_rung = 0;
        loop {
            expect_upward_probe(step, expected_rung, ProbeStrokeRole::InitialForward);
            step = cursor.advance(false).unwrap();
            expect_upward_probe(step, expected_rung, ProbeStrokeRole::InitialReverse);
            step = cursor.advance(false).unwrap();
            expected_rung += 1;
            if expected_rung == rung_count {
                assert_eq!(
                    step,
                    UpwardProbeStep::NoRepeatableMotionAtAuthority {
                        observations: rung_count * 2,
                    }
                );
                assert_eq!(cursor.observations(), rung_count * 2);
                break;
            }
        }
    }
}

#[test]
fn upward_probe_resolves_the_first_four_stroke_pass() {
    let (mut cursor, step) = UpwardBreakawayCursor::start(32).unwrap();
    expect_upward_probe(step, 0, ProbeStrokeRole::InitialForward);
    let step = cursor.advance(true).unwrap();
    expect_upward_probe(step, 0, ProbeStrokeRole::InitialReverse);
    let step = cursor.advance(true).unwrap();
    expect_upward_probe(step, 0, ProbeStrokeRole::RepeatForward);
    let step = cursor.advance(true).unwrap();
    expect_upward_probe(step, 0, ProbeStrokeRole::RepeatReverse);
    assert_eq!(
        cursor.advance(true),
        Ok(UpwardProbeStep::Resolved {
            rung_index: 0,
            observations: 4,
        })
    );
}

#[test]
fn upward_probe_failed_repeat_advances_once_without_a_reroll() {
    let (mut cursor, mut step) = UpwardBreakawayCursor::start(3).unwrap();
    for moved in [true, true, true, false] {
        step = cursor.advance(moved).unwrap();
    }

    expect_upward_probe(step, 1, ProbeStrokeRole::InitialForward);
    for moved in [true, true, true] {
        step = cursor.advance(moved).unwrap();
    }
    expect_upward_probe(step, 1, ProbeStrokeRole::RepeatReverse);
    assert_eq!(
        cursor.advance(true),
        Ok(UpwardProbeStep::Resolved {
            rung_index: 1,
            observations: 8,
        })
    );
}

#[test]
fn upward_probe_worst_case_consumes_exactly_128_strokes() {
    let (mut cursor, mut step) = UpwardBreakawayCursor::start(MAX_SWEEP_RUNGS as u8).unwrap();
    while let UpwardProbeStep::Probe { role, .. } = step {
        let moved = role != ProbeStrokeRole::RepeatReverse;
        step = cursor.advance(moved).unwrap();
    }

    assert_eq!(
        step,
        UpwardProbeStep::NoRepeatableMotionAtAuthority { observations: 128 }
    );
    assert_eq!(cursor.observations(), 128);
}

#[test]
fn upward_probe_rejects_an_empty_or_oversized_grid() {
    assert_eq!(
        UpwardBreakawayCursor::start(0),
        Err(BreakawaySearchError::InvalidRequest)
    );
    assert_eq!(
        UpwardBreakawayCursor::start(MAX_SWEEP_RUNGS as u8 + 1),
        Err(BreakawaySearchError::InvalidRequest)
    );
}
