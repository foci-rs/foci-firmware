//! Effective in-loop delay estimate from the inductance run's excitation phase.

use libm::{acosf, atan2f, cosf, hypotf, roundf, sinf, sqrtf};

/// Inputs to the phase-delay estimate, all from an accepted inductance run and
/// the resistance model.
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub struct PhaseDelayInput {
    /// Mean d-axis current over the averaging window, in milli-counts.
    pub id_mean_milli_count: i32,
    /// Mean q-axis current over the averaging window, in milli-counts.
    pub iq_mean_milli_count: i32,
    /// RMS d-axis spread around the mean, in milli-counts.
    pub id_rms_milli_count: u32,
    /// RMS q-axis spread around the mean, in milli-counts.
    pub iq_rms_milli_count: u32,
    /// Number of samples the mean and RMS were computed over.
    pub sample_count: u16,
    /// Fitted resistance drop, in voltage-count millis per current count.
    pub r_count_milli: u32,
    /// Applied d-axis excitation voltage count.
    pub ud_count: u16,
    /// Realized excitation frequency, in millihertz.
    pub realized_frequency_millihz: u32,
}

/// A measured effective in-loop delay with its uncertainty, both in microseconds,
/// rounded to nearest.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct PhaseDelayEstimate {
    /// Effective in-loop delay, in microseconds, rounded to nearest.
    pub theta_e_us: u32,
    /// One-sigma uncertainty on `theta_e_us`, in microseconds, rounded to nearest.
    pub sigma_us: u32,
    /// `delta_meas - delta_rl` in hundredths of a degree, for the log line.
    pub excess_centideg: i32,
}

/// Why no delay could be read from the run.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum PhaseDelayRejection {
    /// `sample_count`, `realized_frequency_millihz`, `ud_count` or the current magnitude is zero.
    NoSamples,
    /// `R * |I| > U`: the resistive drop exceeds the applied voltage.
    ResistiveDropExceedsDrive,
    /// `iq_mean >= 0`: the current leads, so the frame or rotation is not what the design assumes.
    Lead,
    /// `delta_meas <= delta_rl`: no lag beyond the RL phase.
    NoExcess,
}

/// Effective in-loop delay from the lag of the mean rotating-frame current past the RL phase.
///
/// `theta_e_us` is the delay the on-chip control loop adds beyond the motor's own RL phase lag:
/// PWM zero-order hold plus DSADC decimation, both evaluated at the excitation frequency, and
/// excluding SPI register-read latency. Any lag measured beyond `delta_rl` can only be that
/// in-loop delay, because with the excitation on the d axis a passive RL circuit never produces a
/// negative in-phase current, so a positive excess is unambiguously delay rather than a sign
/// artifact of the measurement.
pub fn estimate_phase_delay(
    input: &PhaseDelayInput,
) -> Result<PhaseDelayEstimate, PhaseDelayRejection> {
    let id_mean = input.id_mean_milli_count as f32;
    let iq_mean = input.iq_mean_milli_count as f32;
    let i_milli = hypotf(id_mean, iq_mean);

    if input.sample_count == 0
        || input.realized_frequency_millihz == 0
        || input.ud_count == 0
        || i_milli == 0.0
    {
        return Err(PhaseDelayRejection::NoSamples);
    }

    let f_hz = input.realized_frequency_millihz as f32 / 1000.0;
    let r = input.r_count_milli as f32 / 1000.0;
    let i_mag = i_milli / 1000.0;
    let u = input.ud_count as f32;

    if r * i_mag > u {
        return Err(PhaseDelayRejection::ResistiveDropExceedsDrive);
    }

    if input.iq_mean_milli_count >= 0 {
        return Err(PhaseDelayRejection::Lead);
    }

    let delta_meas = atan2f(iq_mean.abs(), id_mean);
    let delta_rl = acosf((r * i_mag / u).min(1.0));

    if delta_meas <= delta_rl {
        return Err(PhaseDelayRejection::NoExcess);
    }

    let excess_rad = delta_meas - delta_rl;
    let two_pi_f = 2.0 * core::f32::consts::PI * f_hz;
    let theta_e_us = 1_000_000.0 * excess_rad / two_pi_f;

    let psi = atan2f(iq_mean, id_mean);
    let sigma_perp = (sinf(psi).abs() * input.id_rms_milli_count as f32
        + cosf(psi).abs() * input.iq_rms_milli_count as f32)
        / 1000.0;
    let sigma_theta_us =
        1_000_000.0 * sigma_perp / (i_mag * sqrtf(input.sample_count as f32) * two_pi_f);

    let excess_centideg = excess_rad * 180.0 / core::f32::consts::PI * 100.0;

    Ok(PhaseDelayEstimate {
        theta_e_us: roundf(theta_e_us) as u32,
        sigma_us: roundf(sigma_theta_us) as u32,
        excess_centideg: roundf(excess_centideg) as i32,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run(id: i32, iq: i32, id_rms: u32, iq_rms: u32, r: u32) -> PhaseDelayInput {
        PhaseDelayInput {
            id_mean_milli_count: id,
            iq_mean_milli_count: iq,
            id_rms_milli_count: id_rms,
            iq_rms_milli_count: iq_rms,
            sample_count: 100,
            r_count_milli: r,
            ud_count: 768,
            realized_frequency_millihz: 1_000_000,
        }
    }

    // (id_mean, iq_mean, id_rms, iq_rms, r_count_milli, theta_us, sigma_us): seven
    // recorded runs, four at 50 kHz PWM then three at 25 kHz. theta_us is the offline
    // fractional value from the spec's equations, to two decimals; the nearest integer
    // is the uniquely correct rounding of it.
    const RECORDED_RUNS: [(i32, i32, u32, u32, u32, f32, u32); 7] = [
        (-3540, -52671, 21588, 19848, 997, 21.60, 7),
        (-4071, -54311, 10105, 12198, 951, 22.65, 3),
        (-2010, -53785, 13060, 10731, 940, 16.44, 4),
        (-207, -54901, 9161, 11997, 953, 11.45, 3),
        (-10657, -52615, 11909, 11265, 938, 42.25, 4),
        (-10880, -53904, 12157, 12685, 932, 42.33, 4),
        (-10075, -52282, 11739, 12046, 929, 40.56, 4),
    ];

    #[test]
    fn rounds_the_recorded_delays_to_the_nearest_microsecond() {
        for (id, iq, id_rms, iq_rms, r, theta, sigma) in RECORDED_RUNS {
            let estimate = estimate_phase_delay(&run(id, iq, id_rms, iq_rms, r)).unwrap();
            assert!(
                (estimate.theta_e_us as f32 - theta).abs() <= 0.5,
                "id {id}: theta {}",
                estimate.theta_e_us
            );
            assert!(
                estimate.sigma_us.abs_diff(sigma) <= 1,
                "id {id}: sigma {}",
                estimate.sigma_us
            );
            assert!(estimate.excess_centideg > 0);
        }
    }

    #[test]
    fn pwm_period_scaling_shows_in_the_recorded_runs() {
        let at_50k: u32 = RECORDED_RUNS[..4]
            .iter()
            .map(|&(id, iq, a, b, r, ..)| {
                estimate_phase_delay(&run(id, iq, a, b, r))
                    .unwrap()
                    .theta_e_us
            })
            .sum::<u32>()
            / 4;
        let at_25k: u32 = RECORDED_RUNS[4..]
            .iter()
            .map(|&(id, iq, a, b, r, ..)| {
                estimate_phase_delay(&run(id, iq, a, b, r))
                    .unwrap()
                    .theta_e_us
            })
            .sum::<u32>()
            / 3;
        assert!((15..=21).contains(&at_50k), "50 kHz mean {at_50k}");
        assert!((40..=44).contains(&at_25k), "25 kHz mean {at_25k}");
    }

    #[test]
    fn a_leading_current_is_rejected() {
        assert_eq!(
            estimate_phase_delay(&run(-3540, 52671, 21588, 19848, 997)),
            Err(PhaseDelayRejection::Lead)
        );
        assert_eq!(
            estimate_phase_delay(&run(-3540, 0, 21588, 19848, 997)),
            Err(PhaseDelayRejection::Lead)
        );
    }

    #[test]
    fn no_lag_beyond_the_rl_phase_is_rejected() {
        // In-phase component large and positive: the current sits well inside the RL phase.
        assert_eq!(
            estimate_phase_delay(&run(40_000, -30_000, 1_000, 1_000, 997)),
            Err(PhaseDelayRejection::NoExcess)
        );
    }

    #[test]
    fn a_resistive_drop_above_the_drive_is_rejected() {
        // R = 20 V/I counts, |I| = 52.8: R*|I| = 1056 > 768.
        assert_eq!(
            estimate_phase_delay(&run(-3540, -52671, 21588, 19848, 20_000)),
            Err(PhaseDelayRejection::ResistiveDropExceedsDrive)
        );
    }

    #[test]
    fn degenerate_inputs_are_rejected() {
        let mut zero_n = run(-3540, -52671, 21588, 19848, 997);
        zero_n.sample_count = 0;
        assert_eq!(
            estimate_phase_delay(&zero_n),
            Err(PhaseDelayRejection::NoSamples)
        );
        let mut zero_f = run(-3540, -52671, 21588, 19848, 997);
        zero_f.realized_frequency_millihz = 0;
        assert_eq!(
            estimate_phase_delay(&zero_f),
            Err(PhaseDelayRejection::NoSamples)
        );
        let mut zero_u = run(-3540, -52671, 21588, 19848, 997);
        zero_u.ud_count = 0;
        assert_eq!(
            estimate_phase_delay(&zero_u),
            Err(PhaseDelayRejection::NoSamples)
        );
        assert_eq!(
            estimate_phase_delay(&run(0, 0, 1, 1, 997)),
            Err(PhaseDelayRejection::NoSamples)
        );
    }

    #[test]
    fn sigma_scales_with_the_perpendicular_scatter_and_sample_count() {
        let base = run(-10657, -52615, 11909, 11265, 938);
        let mut doubled_id_rms = base;
        doubled_id_rms.id_rms_milli_count *= 2;
        let mut quadrupled_n = base;
        quadrupled_n.sample_count = 400;
        let s0 = estimate_phase_delay(&base).unwrap().sigma_us;
        let s1 = estimate_phase_delay(&doubled_id_rms).unwrap().sigma_us;
        let s2 = estimate_phase_delay(&quadrupled_n).unwrap().sigma_us;
        // psi is near -101 degrees, so id_rms dominates: doubling it nearly doubles sigma.
        assert!(s1 >= 2 * s0 - 1 && s1 <= 2 * s0 + 1, "s0 {s0} s1 {s1}");
        assert_eq!(s2, (s0 + 1) / 2, "s0 {s0} s2 {s2}");
    }
}
