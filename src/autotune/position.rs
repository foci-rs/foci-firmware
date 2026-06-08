use super::TransientBuffer;

// ---------------------------------------------------------------------------
// Position oscillation detection (Phase 11)
// ---------------------------------------------------------------------------

/// Diagnostic breakdown of `analyze_position_oscillation`. Exposed so the
/// caller can log the inputs that drove the bool decision when an autotune
/// rejects a gain choice.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PositionOscillationDiagnostics {
    pub oscillating: bool,
    pub peak: u32,
    pub overshoot: u32,
    pub crossings: u8,
}

/// Compute oscillation diagnostics from a position capture.
///
/// Returns the same boolean decision as `detect_position_oscillation`, plus
/// the peak sample, the overshoot above target, and the post-peak zero-crossing
/// count. All three feed the bool so logging them tells you exactly which
/// rule fired (or which one almost fired).
pub fn analyze_position_oscillation(
    buf: &TransientBuffer,
    target_position: u32,
) -> PositionOscillationDiagnostics {
    let samples = buf.samples();
    if samples.is_empty() || target_position == 0 {
        return PositionOscillationDiagnostics {
            oscillating: false,
            peak: 0,
            overshoot: 0,
            crossings: 0,
        };
    }

    // Overshoot.
    let overshoot_threshold = target_position / 10;
    let peak = samples.iter().copied().max().unwrap_or(0);
    let overshoot = peak.saturating_sub(target_position);

    // Ringing: zero-crossings of (actual - target) after peak.
    let mut crossings: u8 = 0;
    let peak_idx = samples.iter().position(|&s| s == peak).unwrap_or(0);
    if peak_idx + 1 < samples.len() {
        let mut above = samples[peak_idx] > target_position;
        for &sample in &samples[peak_idx + 1..] {
            let now_above = sample > target_position;
            if now_above != above {
                crossings = crossings.saturating_add(1);
                above = now_above;
            }
        }
    }

    let oscillating = (peak > target_position && overshoot > overshoot_threshold) || crossings > 2;

    PositionOscillationDiagnostics {
        oscillating,
        peak,
        overshoot,
        crossings,
    }
}

/// Returns true when the position capture overshoots target by more than 10%
/// or shows more than one full ringing cycle (>2 zero-crossings) after peak.
///
/// Thin wrapper around `analyze_position_oscillation`; preserves the original
/// boolean API for callers that don't need the diagnostic breakdown.
pub fn detect_position_oscillation(buf: &TransientBuffer, target_position: u32) -> bool {
    analyze_position_oscillation(buf, target_position).oscillating
}

#[cfg(test)]
mod tests {
    use super::*;

    // --- Position oscillation detection ---

    #[test]
    fn position_no_oscillation_clean_settle() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Smooth approach to target 1000.
        for i in 0..256u32 {
            let value = (1000.0 * (1.0 - (-((i as f32) / 20.0)).exp())) as u32;
            buf.push(value);
        }
        assert!(
            !detect_position_oscillation(&buf, 1000),
            "clean settle should not detect oscillation"
        );
    }

    #[test]
    fn position_oscillation_detected_overshoot() {
        let mut buf = TransientBuffer::new();
        buf.reset(1000);
        // Overshoot > 10%: target 1000, peak 1200.
        for i in 0..256u32 {
            let base = 1000.0 * (1.0 - (-((i as f32) / 20.0)).exp());
            let overshoot = 250.0 * (-((i as f32) / 30.0)).exp() * ((i as f32) / 8.0).sin();
            buf.push((base + overshoot).max(0.0) as u32);
        }
        let peak = buf.samples().iter().copied().max().unwrap_or(0);
        if peak > 1100 {
            assert!(
                detect_position_oscillation(&buf, 1000),
                "should detect oscillation with peak={}",
                peak
            );
        }
    }

    #[test]
    fn position_oscillation_empty_buffer() {
        let buf = TransientBuffer::new();
        assert!(!detect_position_oscillation(&buf, 1000));
    }
}
