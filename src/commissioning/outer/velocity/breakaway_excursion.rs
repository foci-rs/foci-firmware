use crate::autotune::velocity_measurement::VelocityDirection;
use crate::commissioning::encoder_domain::EncoderDomain;

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ProbeMotionClass {
    NoMotion,
    Moved,
    WrongDirection,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) enum ProbeExcursionError {
    CaptureInterval,
    Arithmetic,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ProbeExcursionEvidence {
    pub(crate) minimum_counts: i32,
    pub(crate) maximum_counts: i32,
    pub(crate) maximum_interval_us: u32,
    pub(crate) motion: ProbeMotionClass,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub(crate) struct ProbeExcursionCapture {
    previous_encoder_raw: u32,
    previous_read_us: u64,
    displacement_counts: i64,
    minimum_counts: i64,
    maximum_counts: i64,
    maximum_interval_us: u32,
    measured_intervals: u16,
}

impl ProbeExcursionCapture {
    pub(crate) const fn new(baseline_encoder_raw: u32, baseline_read_us: u64) -> Self {
        Self {
            previous_encoder_raw: baseline_encoder_raw,
            previous_read_us: baseline_read_us,
            displacement_counts: 0,
            minimum_counts: 0,
            maximum_counts: 0,
            maximum_interval_us: 0,
            measured_intervals: 0,
        }
    }

    pub(crate) fn observe(
        &mut self,
        domain: EncoderDomain,
        encoder_raw: u32,
        read_us: u64,
    ) -> Result<(), ProbeExcursionError> {
        let interval_us = read_us
            .checked_sub(self.previous_read_us)
            .and_then(|value| u32::try_from(value).ok())
            .filter(|value| *value != 0)
            .ok_or(ProbeExcursionError::CaptureInterval)?;
        let delta = domain
            .delta(
                domain.count(self.previous_encoder_raw),
                domain.count(encoder_raw),
            )
            .map_err(|_| ProbeExcursionError::Arithmetic)?;
        let displacement_counts = self
            .displacement_counts
            .checked_add(i64::from(delta.signed_counts()))
            .ok_or(ProbeExcursionError::Arithmetic)?;
        let measured_intervals = self
            .measured_intervals
            .checked_add(1)
            .ok_or(ProbeExcursionError::Arithmetic)?;

        self.previous_encoder_raw = encoder_raw;
        self.previous_read_us = read_us;
        self.displacement_counts = displacement_counts;
        self.minimum_counts = self.minimum_counts.min(displacement_counts);
        self.maximum_counts = self.maximum_counts.max(displacement_counts);
        self.maximum_interval_us = self.maximum_interval_us.max(interval_us);
        self.measured_intervals = measured_intervals;
        Ok(())
    }

    pub(crate) fn wire_fields(&self) -> Result<(i32, i32, u32), ProbeExcursionError> {
        let minimum_counts =
            i32::try_from(self.minimum_counts).map_err(|_| ProbeExcursionError::Arithmetic)?;
        let maximum_counts =
            i32::try_from(self.maximum_counts).map_err(|_| ProbeExcursionError::Arithmetic)?;
        Ok((minimum_counts, maximum_counts, self.maximum_interval_us))
    }

    pub(crate) fn finish(
        self,
        direction: VelocityDirection,
        motion_threshold_counts: u32,
        max_capture_interval_us: u32,
    ) -> Result<ProbeExcursionEvidence, ProbeExcursionError> {
        let (minimum_counts, maximum_counts, maximum_interval_us) = self.wire_fields()?;
        if self.measured_intervals == 0
            || maximum_interval_us == 0
            || maximum_interval_us > max_capture_interval_us
        {
            return Err(ProbeExcursionError::CaptureInterval);
        }
        if motion_threshold_counts == 0 {
            return Err(ProbeExcursionError::Arithmetic);
        }

        let positive_excursion =
            u32::try_from(maximum_counts).map_err(|_| ProbeExcursionError::Arithmetic)?;
        let negative_excursion = minimum_counts.unsigned_abs();
        let (commanded_excursion, opposite_excursion) = match direction {
            VelocityDirection::Forward => (positive_excursion, negative_excursion),
            VelocityDirection::Reverse => (negative_excursion, positive_excursion),
        };
        let motion = if commanded_excursion >= motion_threshold_counts {
            ProbeMotionClass::Moved
        } else if opposite_excursion >= motion_threshold_counts {
            ProbeMotionClass::WrongDirection
        } else {
            ProbeMotionClass::NoMotion
        };

        Ok(ProbeExcursionEvidence {
            minimum_counts,
            maximum_counts,
            maximum_interval_us,
            motion,
        })
    }
}

#[cfg(test)]
#[path = "breakaway_excursion_tests.rs"]
mod tests;
