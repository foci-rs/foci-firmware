//! Command-level velocity-autotune workflow selection and disclosure.
//!
//! [`VelocityAutotuneWorkflowKind`] names which whole command workflow a host
//! requested -- resume, an fixed-gain-amplitude order, the breakaway-seeded
//! campaign, or the robustness-reversal gate -- not a phase or subphase within
//! one of them. It is disclosed, alongside its conservative duration envelope,
//! before any motion so the host can arm the correct wait timeout.

use super::integral_plan::{VelocityIntegralPlanError, fnv1a64_byte};

pub use crate::commissioning::vocabulary_generated::VelocityAutotuneWorkflowKind;

/// Conservative command-level plan used only for disclosure and host timeout.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct CommissioningWorkflowPlan {
    /// Caller-assigned run sequence.
    pub run_sequence: u32,
    /// Motion shape selected from retained firmware state.
    pub shape: VelocityAutotuneWorkflowKind,
    /// Expected duration under the selected fixed schedule.
    pub nominal_workflow_ms: u32,
    /// Conservative host-timeout authority.
    pub maximum_workflow_ms: u32,
}

impl CommissioningWorkflowPlan {
    /// Construct one checked command-level workflow envelope.
    pub fn new(
        run_sequence: u32,
        shape: VelocityAutotuneWorkflowKind,
        stage_b_ms: u32,
        stage_c_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        let valid = match shape {
            VelocityAutotuneWorkflowKind::Resume => stage_b_ms == 0 && stage_c_ms != 0,
            VelocityAutotuneWorkflowKind::FixedGainAmplitudeAscending
            | VelocityAutotuneWorkflowKind::FixedGainAmplitudeDescending
            | VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral
            | VelocityAutotuneWorkflowKind::ReversalStandstillRobustnessGate => false,
        };
        if !valid {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        let maximum_workflow_ms = stage_b_ms
            .checked_add(stage_c_ms)
            .ok_or(VelocityIntegralPlanError::Arithmetic)?;
        Ok(Self {
            run_sequence,
            shape,
            nominal_workflow_ms: maximum_workflow_ms,
            maximum_workflow_ms,
        })
    }

    /// Construct one checked fixed-I fixed-gain-amplitude workflow envelope.
    pub fn new_fixed_gain_amplitude(
        run_sequence: u32,
        order: super::fixed_gain_amplitude_plan::FixedGainAmplitudeOrder,
        nominal_workflow_ms: u32,
        maximum_workflow_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        if nominal_workflow_ms == 0
            || maximum_workflow_ms == 0
            || nominal_workflow_ms > maximum_workflow_ms
        {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        let shape = match order {
            super::fixed_gain_amplitude_plan::FixedGainAmplitudeOrder::Ascending => {
                VelocityAutotuneWorkflowKind::FixedGainAmplitudeAscending
            }
            super::fixed_gain_amplitude_plan::FixedGainAmplitudeOrder::Descending => {
                VelocityAutotuneWorkflowKind::FixedGainAmplitudeDescending
            }
        };
        Ok(Self {
            run_sequence,
            shape,
            nominal_workflow_ms,
            maximum_workflow_ms,
        })
    }

    /// Construct one checked breakaway-campaign workflow envelope.
    ///
    /// Reserved before the probe's first stroke. `nominal_workflow_ms` and
    /// `maximum_workflow_ms` must already be the worst-case sums produced by
    /// [`super::integral_plan::breakaway_workflow_envelope_ms`]; every later
    /// probe, discovery, confirmation, and Stage-C continuation plan must
    /// report an exact duration that fits inside `maximum_workflow_ms`. A
    /// phase plan that would exceed it is rejected before that phase's first
    /// stroke -- see
    /// [`super::breakaway_plan::AdditiveLadderPlanError::WorkflowEnvelopeExceeded`]
    /// for the discovery-phase example of that check.
    pub fn new_breakaway(
        run_sequence: u32,
        nominal_workflow_ms: u32,
        maximum_workflow_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        if nominal_workflow_ms == 0
            || maximum_workflow_ms == 0
            || nominal_workflow_ms > maximum_workflow_ms
        {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        Ok(Self {
            run_sequence,
            shape: VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral,
            nominal_workflow_ms,
            maximum_workflow_ms,
        })
    }

    /// Construct one checked robustness-reversal workflow envelope.
    ///
    /// Reserved before the robustness probe's first motion so the host arms
    /// its extended run timeout: the K-cycle run runs far longer than the
    /// host's 5 s initial plan window. `maximum_workflow_ms` is
    /// `robustness_worst_case_duration_ms`, `nominal_workflow_ms` a clean-run
    /// estimate.
    pub fn new_robustness(
        run_sequence: u32,
        nominal_workflow_ms: u32,
        maximum_workflow_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        if nominal_workflow_ms == 0
            || maximum_workflow_ms == 0
            || nominal_workflow_ms > maximum_workflow_ms
        {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        Ok(Self {
            run_sequence,
            shape: VelocityAutotuneWorkflowKind::ReversalStandstillRobustnessGate,
            nominal_workflow_ms,
            maximum_workflow_ms,
        })
    }

    /// Hash the command-level envelope independently of either stage plan.
    pub fn digest(self) -> u64 {
        let mut hash = 0xcbf2_9ce4_8422_2325_u64;
        for byte in self.run_sequence.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        hash = fnv1a64_byte(hash, self.shape as u8);
        for byte in self.nominal_workflow_ms.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        for byte in self.maximum_workflow_ms.to_le_bytes() {
            hash = fnv1a64_byte(hash, byte);
        }
        hash
    }
}
