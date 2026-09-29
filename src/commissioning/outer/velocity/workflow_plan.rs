//! Command-level velocity-autotune workflow selection and disclosure.
//!
//! [`VelocityAutotuneWorkflowKind`] names which whole command workflow a host
//! requested; it is disclosed with its duration envelope before any motion.

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
        proportional_ms: u32,
        integral_ms: u32,
    ) -> Result<Self, VelocityIntegralPlanError> {
        let valid = match shape {
            VelocityAutotuneWorkflowKind::Resume => proportional_ms == 0 && integral_ms != 0,
            VelocityAutotuneWorkflowKind::FixedGainAmplitudeAscending
            | VelocityAutotuneWorkflowKind::FixedGainAmplitudeDescending
            | VelocityAutotuneWorkflowKind::BreakawaySeededProportionalThenIntegral
            | VelocityAutotuneWorkflowKind::ReversalStandstillRobustnessGate
            | VelocityAutotuneWorkflowKind::PositionTune => false,
        };
        if !valid {
            return Err(VelocityIntegralPlanError::InvalidRequest);
        }
        let maximum_workflow_ms = proportional_ms
            .checked_add(integral_ms)
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
    /// `nominal_workflow_ms` and `maximum_workflow_ms` must be the worst-case sums from
    /// [`super::integral_plan::breakaway_workflow_envelope_ms`].
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
    /// `maximum_workflow_ms` is `robustness_worst_case_duration_ms`.
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

    /// Construct one checked position-P sweep workflow envelope.
    ///
    /// `maximum_workflow_ms` is `position_tune_worst_case_duration_ms`.
    pub fn new_position_tune(
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
            shape: VelocityAutotuneWorkflowKind::PositionTune,
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
