use super::{
    BUILD_FEATURE_FIXED_TIME_REVERSAL_OBSERVATION, BUILD_FEATURES,
    PROPORTIONAL_EVIDENCE_SCHEMA_REVISION,
};

#[test]
fn build_features_and_schema_revision_are_stable() {
    assert_eq!(BUILD_FEATURES, 0b11_1111);
    assert_eq!(PROPORTIONAL_EVIDENCE_SCHEMA_REVISION, 8);
}

#[test]
fn build_features_carries_the_fixed_time_reversal_bit() {
    assert_eq!(BUILD_FEATURE_FIXED_TIME_REVERSAL_OBSERVATION, 1 << 5);
    assert_ne!(
        BUILD_FEATURES & BUILD_FEATURE_FIXED_TIME_REVERSAL_OBSERVATION,
        0
    );
}
