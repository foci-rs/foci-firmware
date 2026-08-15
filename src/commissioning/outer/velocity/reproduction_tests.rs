use super::{BUILD_FEATURES, STAGE_B_EVIDENCE_SCHEMA_REVISION};

#[test]
fn build_features_and_schema_revision_are_stable() {
    // The current firmware compiles in all four analysis rules.
    assert_eq!(BUILD_FEATURES, 0b1111);
    assert_eq!(STAGE_B_EVIDENCE_SCHEMA_REVISION, 8);
}
