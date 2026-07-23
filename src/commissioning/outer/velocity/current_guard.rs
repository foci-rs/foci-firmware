//! Shared velocity-current prevention and evidence classification.
#![cfg_attr(not(test), allow(dead_code))]

use foci_tmc4671::{current_limit::PidTorqueFluxLimit, registers::StatusFlags};

use super::sweep_plan::{CURRENT_HEADROOM_DEN, CURRENT_HEADROOM_NUM};

/// Firmware's ability to prove whether the velocity-output clamp engaged.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum VelocityLimitEvidenceCapability {
    /// Periodic reads can confirm observed contacts but cannot prove absence.
    BestEffortPolling = 0,
    /// The controller preserves a contact until firmware reads the status flag.
    LatchedFlag = 1,
}

/// Primitive phase in which the velocity-output clamp first made contact.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub(crate) enum VelocityCurrentContact {
    /// No clamp contact was observed.
    None = 0,
    /// Contact occurred while collecting moving evidence.
    Moving = 1,
    /// Contact occurred after commanding zero for braking and settle.
    Braking = 2,
    /// Contact occurred during the optional return-to-origin motion.
    Recovery = 3,
}

/// Zero-target phase represented by the second observation window.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum VelocityCurrentZeroPhase {
    /// The moving primitive's braking and settle phase.
    Braking,
    /// The optional return-to-origin primitive.
    Recovery,
}

/// Current-limit evidence collected across one velocity primitive.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityCurrentPhaseEvidence {
    /// Temporary probe clamp applied to the velocity PID output.
    pub(crate) clamp_limit: u16,
    /// Largest sampled absolute velocity-PID output during moving evidence.
    pub(crate) moving_pid_output_peak: u16,
    /// Largest sampled absolute velocity-PID output after commanding zero.
    pub(crate) zero_pid_output_peak: u16,
    /// Complete `STATUS_FLAGS` word observed during moving evidence.
    pub(crate) moving_status_flags: u32,
    /// Complete `STATUS_FLAGS` word observed after commanding zero.
    pub(crate) zero_status_flags: u32,
    /// Available mechanism for proving absence of an unsampled contact.
    pub(crate) capability: VelocityLimitEvidenceCapability,
    /// First phase in which clamp contact was observed.
    pub(crate) contact: VelocityCurrentContact,
}

impl VelocityCurrentPhaseEvidence {
    /// Return whether the moving phase is proven free of clamp contact.
    pub(crate) fn moving_clip_absence_proven(self) -> bool {
        self.capability == VelocityLimitEvidenceCapability::LatchedFlag
            && self.contact != VelocityCurrentContact::Moving
            && self.moving_pid_output_peak < self.clamp_limit
    }

    /// Return whether an unobserved moving-phase contact remains possible.
    pub(crate) fn moving_clip_uncertain(self) -> bool {
        self.capability == VelocityLimitEvidenceCapability::BestEffortPolling
            && self.contact != VelocityCurrentContact::Moving
            && self.moving_pid_output_peak < self.clamp_limit
    }
}

/// Invalid operational current limit for a nonzero commissioning clamp.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct VelocityCurrentGuardError;

/// Derive the exact nine-tenths probe limit from the operational limit.
pub(crate) fn derive_probe_limit(
    operational_limit: PidTorqueFluxLimit,
) -> Result<PidTorqueFluxLimit, VelocityCurrentGuardError> {
    let probe_raw =
        u32::from(operational_limit.raw()) * CURRENT_HEADROOM_NUM / CURRENT_HEADROOM_DEN;
    if probe_raw == 0 {
        return Err(VelocityCurrentGuardError);
    }

    PidTorqueFluxLimit::new(probe_raw).map_err(|_| VelocityCurrentGuardError)
}

/// Classify current-limit evidence without changing the underlying raw flags.
#[allow(clippy::too_many_arguments)]
pub(crate) fn classify_phase_evidence(
    clamp_limit: PidTorqueFluxLimit,
    moving_pid_output_peak: u16,
    zero_pid_output_peak: u16,
    moving_status_flags: u32,
    zero_status_flags: u32,
    capability: VelocityLimitEvidenceCapability,
    zero_phase: VelocityCurrentZeroPhase,
) -> Result<VelocityCurrentPhaseEvidence, VelocityCurrentGuardError> {
    if clamp_limit == PidTorqueFluxLimit::ZERO {
        return Err(VelocityCurrentGuardError);
    }

    let moving_contact = moving_pid_output_peak >= clamp_limit.raw()
        || StatusFlags::from(moving_status_flags).pid_v_output_limit();
    let zero_contact = zero_pid_output_peak >= clamp_limit.raw()
        || StatusFlags::from(zero_status_flags).pid_v_output_limit();
    let contact = if moving_contact {
        VelocityCurrentContact::Moving
    } else if zero_contact {
        match zero_phase {
            VelocityCurrentZeroPhase::Braking => VelocityCurrentContact::Braking,
            VelocityCurrentZeroPhase::Recovery => VelocityCurrentContact::Recovery,
        }
    } else {
        VelocityCurrentContact::None
    };

    Ok(VelocityCurrentPhaseEvidence {
        clamp_limit: clamp_limit.raw(),
        moving_pid_output_peak,
        zero_pid_output_peak,
        moving_status_flags,
        zero_status_flags,
        capability,
        contact,
    })
}
