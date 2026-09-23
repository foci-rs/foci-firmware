use super::{BUILD_FEATURES, PROPORTIONAL_EVIDENCE_SCHEMA_REVISION};

#[test]
fn build_features_and_schema_revision_are_stable() {
    assert_eq!(BUILD_FEATURES, 0b1_1111);
    assert_eq!(PROPORTIONAL_EVIDENCE_SCHEMA_REVISION, 8);
}
