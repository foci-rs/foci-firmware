use super::integral_plan::{VelocityIntegralPlanError, transition_capable_integral_gain};

#[test]
fn integral_transition_capable_predicate_is_exact_at_quantum_and_clamp_boundaries() {
    for (inputs, expected) in [
        ((16_384, 264, 100, 1_000, 512_000, 55_377), false),
        ((16_384, 264, 100, 1_000, 512_000, 55_378), true),
        ((835, 374, 4_000, 25_000, 512_000, 2_534), true),
        ((835, 375, 4_000, 25_000, 512_000, 2_534), false),
        ((834, 376, 4_000, 25_000, 512_000, 2_534), false),
        ((724, 264, 4_000, 25_000, 512_001, 2_534), true),
        ((1_024, 132, 4_000, 25_000, 512_000, 2_534), true),
        ((1_448, 132, 4_000, 25_000, 512_000, 2_534), false),
    ] {
        assert_eq!(
            transition_capable_integral_gain(
                inputs.0, inputs.1, inputs.2, inputs.3, inputs.4, inputs.5,
            ),
            Ok(expected)
        );
    }
    for invalid in [
        transition_capable_integral_gain(0, 264, 4_000, 25_000, 512_000, 2_534),
        transition_capable_integral_gain(724, 264, 0, 25_000, 512_000, 2_534),
        transition_capable_integral_gain(724, 264, 4_000, 0, 512_000, 2_534),
        transition_capable_integral_gain(724, 264, 4_000, 25_000, 0, 2_534),
        transition_capable_integral_gain(724, 264, 4_000, 25_000, 512_000, 0),
    ] {
        assert_eq!(invalid, Err(VelocityIntegralPlanError::InvalidRequest));
    }
    assert_eq!(
        transition_capable_integral_gain(1, i32::MIN, 1, 1, 1, 1),
        Err(VelocityIntegralPlanError::Arithmetic)
    );
    assert_eq!(
        transition_capable_integral_gain(
            u16::MAX,
            i32::MAX,
            u32::MAX,
            u32::MAX,
            u32::MAX,
            u16::MAX,
        ),
        Ok(false)
    );
}
