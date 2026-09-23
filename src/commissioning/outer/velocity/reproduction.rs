/// Proportional evidence schema revision stamped on every emitted plan.
pub const PROPORTIONAL_EVIDENCE_SCHEMA_REVISION: u16 = 8;

// Injected by build.rs: `BUILD_IDENTITY`, the first four bytes of the
// foci-firmware HEAD commit (or 0 when unresolved). Pure image provenance; it is
// emitted on every plan for attestation but never digested or identity-checked,
// so a rebuild does not spuriously stale retained authority.
include!(concat!(env!("OUT_DIR"), "/build_identity.rs"));

/// Integral gains are encoded Q4.12 rather than the legacy Q8.8.
pub const BUILD_FEATURE_Q4_12_GAINS: u16 = 1 << 0;
/// The moving-stroke gate applies the stationarity drift bound.
pub const BUILD_FEATURE_DRIFT_BOUND_GATE: u16 = 1 << 1;
/// The suffix walk continues past a rejected window instead of ending there.
pub const BUILD_FEATURE_SUFFIX_WALK_CONTINUES: u16 = 1 << 2;
/// The scored-rest chain closes only on evidence against rest.
pub const BUILD_FEATURE_REST_CHAIN_ON_EVIDENCE: u16 = 1 << 3;
/// The Stage-C transition gate models the standard velocity meter's fixed-rate
/// quantum instead of one encoder count per PWM period.
pub const BUILD_FEATURE_STANDARD_METER_QUANTUM: u16 = 1 << 4;

/// Analysis-rule contract this firmware build compiles in, stamped on every
/// plan so a capture states which rules produced it rather than being dated by
/// schema revision. Replaces the crate-version `build_revision`, whose only real
/// consumer selected the gain representation (now bit 0 here). It is digested
/// and identity-checked, so retained authority refuses to resume across a rules
/// change.
pub const BUILD_FEATURES: u16 = BUILD_FEATURE_Q4_12_GAINS
    | BUILD_FEATURE_DRIFT_BOUND_GATE
    | BUILD_FEATURE_SUFFIX_WALK_CONTINUES
    | BUILD_FEATURE_REST_CHAIN_ON_EVIDENCE
    | BUILD_FEATURE_STANDARD_METER_QUANTUM;

/// Cross-run validity of one selected directional model or fragment.
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum ProportionalDirectionalValidity {
    /// Either run has no selected directional evidence.
    NoRegion = 0,
    /// At least one run retains only a one- or two-rung fragment.
    Fragment = 1,
    /// Both runs retain regions, but their constant-disturbance model did not reproduce.
    RegionNotReproduced = 2,
    /// The reproduced directional model contains the final operating P.
    ReproducedRegionCoversFinalP = 3,
    /// The reproduced directional model transfers to the final operating P.
    ReproducedRegionTransferred = 4,
}

#[cfg(test)]
#[path = "reproduction_tests.rs"]
mod tests;
