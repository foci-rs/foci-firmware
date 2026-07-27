use super::acceptance_evidence::{
    AcceptanceAmplitudeAvailability, AcceptanceAmplitudeReport, AcceptanceDirectionInput,
    AcceptanceEvidenceError, AcceptanceRunMasks, acceptance_run_sufficient,
    classify_acceptance_amplitude, robust_rate_location,
};
use super::sweep_consensus::DirectionalConsensusClass;
use super::sweep_evidence::VelocityObservationClass;

#[test]
fn acceptance_reports_keep_measured_sizes() {
    assert_eq!(core::mem::size_of::<AcceptanceAmplitudeReport>(), 184);
    assert_eq!(
        core::mem::size_of::<crate::commissioning::types::AcceptanceMatrixTerminal>(),
        112
    );
}

fn input(intervals: [[i32; 2]; 4]) -> AcceptanceDirectionInput {
    AcceptanceDirectionInput {
        signed_target_rate_q: 100,
        collected_mask: 0b1111,
        classes: [VelocityObservationClass::Eligible; 4],
        intervals: intervals.map(Some),
        means_q: [Some(1), Some(2), Some(3), Some(4)],
        blocking_levels: [2, 3, 4, 2],
        precision_floor_candidate: false,
    }
}

#[test]
fn four_observation_consensus_preserves_depth_three_semantics() {
    let compatible =
        classify_acceptance_amplitude(input([[0, 10], [2, 9], [3, 8], [4, 7]])).unwrap();
    assert_eq!(
        compatible.consensus.classification,
        DirectionalConsensusClass::EligibleConsensus
    );
    assert_eq!(compatible.response_interval_q, Some([3, 8]));
    assert_eq!(compatible.consensus.included_mask, 0b1111);
    assert_eq!(compatible.deficit_interval_q, Some([92, 97]));

    let outlier = classify_acceptance_amplitude(input([[0, 4], [0, 4], [0, 4], [6, 10]])).unwrap();
    assert_eq!(outlier.response_interval_q, Some([0, 4]));
    assert_eq!(outlier.consensus.included_mask, 0b0111);
    assert_eq!(outlier.excluded_mask, 0b1000);

    let ambiguous =
        classify_acceptance_amplitude(input([[0, 4], [0, 10], [0, 10], [6, 10]])).unwrap();
    assert_eq!(
        ambiguous.availability,
        AcceptanceAmplitudeAvailability::Ambiguous
    );
    assert_eq!(ambiguous.consensus.component_count, 2);
    assert_eq!(ambiguous.response_interval_q, None);

    let touching =
        classify_acceptance_amplitude(input([[0, 5], [5, 10], [0, 5], [5, 10]])).unwrap();
    assert_eq!(touching.response_interval_q, Some([5, 5]));
}

#[test]
fn unusable_observations_and_signed_overshoot_remain_evidence() {
    let mut two = input([[0, 4]; 4]);
    two.classes[2] = VelocityObservationClass::Inert;
    two.classes[3] = VelocityObservationClass::DeficitUnresolved;
    two.intervals[2] = None;
    two.intervals[3] = None;
    two.means_q[2] = None;
    two.means_q[3] = None;
    two.precision_floor_candidate = true;
    let two = classify_acceptance_amplitude(two).unwrap();
    assert_eq!(
        two.availability,
        AcceptanceAmplitudeAvailability::BelowPrecisionFloor
    );
    assert_eq!(two.robust_location_q, None);

    let mut overshoot = input([[-10, -5], [-9, -4], [-8, -3], [-7, -2]]);
    overshoot.signed_target_rate_q = -6;
    let overshoot = classify_acceptance_amplitude(overshoot).unwrap();
    assert_eq!(
        overshoot.availability,
        AcceptanceAmplitudeAvailability::Eligible
    );
    assert_eq!(overshoot.response_interval_q, Some([-8, -4]));
    assert_eq!(overshoot.deficit_interval_q, Some([-2, 2]));
}

#[test]
fn robust_location_is_exact_for_three_and_four_signed_means() {
    assert_eq!(
        robust_rate_location([Some(9), Some(1), Some(5), None]),
        Ok(Some(5))
    );
    assert_eq!(
        robust_rate_location([Some(4), Some(1), Some(8), Some(3)]),
        Ok(Some(3))
    );
    assert_eq!(
        robust_rate_location([Some(-1), Some(-4), Some(-8), Some(3)]),
        Ok(Some(-3))
    );
    assert_eq!(
        robust_rate_location([Some(1), Some(2), None, None]),
        Ok(None)
    );
}

#[test]
fn run_sufficiency_requires_three_eligible_amplitudes_per_direction() {
    let sufficient = AcceptanceRunMasks {
        attempted: [0b11111; 2],
        eligible: [0b00111, 0b10101],
        current_terminus: [0, 0b10000],
        unattempted: [0; 2],
    };
    assert_eq!(acceptance_run_sufficient(sufficient), Ok(true));

    let mut insufficient = sufficient;
    insufficient.eligible[1] = 0b00011;
    assert_eq!(acceptance_run_sufficient(insufficient), Ok(false));

    let invalid = AcceptanceRunMasks {
        attempted: [0b00111; 2],
        eligible: [0b00111; 2],
        current_terminus: [0; 2],
        unattempted: [0; 2],
    };
    assert_eq!(
        acceptance_run_sufficient(invalid),
        Err(AcceptanceEvidenceError::InvalidMasks)
    );
}

#[test]
fn current_and_fault_classes_override_statistical_consensus() {
    let mut current = input([[0, 4]; 4]);
    current.classes[3] = VelocityObservationClass::CurrentHeadroom;
    assert_eq!(
        classify_acceptance_amplitude(current).unwrap().availability,
        AcceptanceAmplitudeAvailability::CurrentHeadroom
    );

    let mut fault = input([[0, 4]; 4]);
    fault.classes[3] = VelocityObservationClass::HardClipFault;
    assert_eq!(
        classify_acceptance_amplitude(fault).unwrap().availability,
        AcceptanceAmplitudeAvailability::Fault
    );
}
