/// Nested scored-rest candidates, anchored at the end of a capture.
pub const VELOCITY_SUFFIX_LENGTHS: [u16; 3] = [64, 128, 256];

/// Structural result for one scored-rest suffix.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RestStructuralCode {
    /// Every structural stationarity gate passed.
    Usable = 0,
    /// No two-consecutive-doubling variance plateau was found.
    NoBlockingPlateau = 1,
    /// The selected scale has fewer than the required decorrelated blocks.
    TooFewDecorrelatedBlocks = 2,
    /// The fitted rate slope remains distinguishable from zero.
    ResolvedSlope = 3,
    /// Detrended residuals retain significant lag-one structure.
    StructuredResidual = 4,
    /// Checked arithmetic or a numeric conversion failed.
    Arithmetic = 5,
}

/// Final scored-rest selector disposition.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
#[repr(u8)]
pub enum RestSelectionDisposition {
    /// A contiguous recent suffix chain established rest statistically.
    StatisticalAcceptance = 0,
    /// The complete window stayed inside the existing one-count band.
    OneCountFallbackAcceptance = 1,
    /// Neither the statistical chain nor the one-count fallback established rest.
    Rejected = 2,
}

/// Compact scored-rest decision over the declared suffix family.
#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct RestSelectionDecision {
    /// Structural result codes in [`VELOCITY_SUFFIX_LENGTHS`] order.
    pub structural_codes: [RestStructuralCode; 3],
    /// Exact simultaneous intervals in [`VELOCITY_SUFFIX_LENGTHS`] order.
    pub intervals_q: [[i64; 2]; 3],
    /// Bit set when the matching interval is present.
    pub interval_present_mask: u8,
    /// Bit set when every structural stationarity gate passed.
    pub usable_mask: u8,
    /// Bit set when a structurally usable closed interval contains zero.
    pub zero_compatible_mask: u8,
    /// Bit naming the selected suffix, or zero when no suffix was selected.
    pub selected_mask: u8,
    /// Selected suffix length, or zero when no suffix was selected.
    pub selected_suffix_len: u16,
    /// Final selector disposition.
    pub disposition: RestSelectionDisposition,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) struct RestSuffixEvaluation {
    pub(super) structural_code: RestStructuralCode,
    pub(super) interval_present: bool,
    pub(super) interval_q: [i64; 2],
}

pub(super) fn select_causal_rest_suffixes(
    suffixes: [RestSuffixEvaluation; 3],
    one_count_fallback: bool,
) -> RestSelectionDecision {
    let mut structural_codes = [RestStructuralCode::TooFewDecorrelatedBlocks; 3];
    let mut intervals_q = [[0; 2]; 3];
    let mut interval_present_mask = 0_u8;
    let mut usable_mask = 0_u8;
    let mut zero_compatible_mask = 0_u8;
    let mut selected_mask = 0_u8;
    let mut selected_suffix_len = 0_u16;
    let mut chain_open = true;

    for (index, suffix) in suffixes.into_iter().enumerate() {
        let bit = 1_u8 << index;
        structural_codes[index] = suffix.structural_code;
        intervals_q[index] = suffix.interval_q;
        if suffix.interval_present {
            interval_present_mask |= bit;
        }
        let usable = suffix.structural_code == RestStructuralCode::Usable;
        if usable {
            usable_mask |= bit;
        }
        let zero_compatible = usable
            && suffix.interval_present
            && suffix.interval_q[0] <= 0
            && 0 <= suffix.interval_q[1];
        if zero_compatible {
            zero_compatible_mask |= bit;
        }
        if chain_open && zero_compatible {
            selected_mask = bit;
            selected_suffix_len = VELOCITY_SUFFIX_LENGTHS[index];
        } else {
            chain_open = false;
        }
    }

    let disposition = if selected_mask != 0 {
        RestSelectionDisposition::StatisticalAcceptance
    } else if one_count_fallback {
        RestSelectionDisposition::OneCountFallbackAcceptance
    } else {
        RestSelectionDisposition::Rejected
    };
    RestSelectionDecision {
        structural_codes,
        intervals_q,
        interval_present_mask,
        usable_mask,
        zero_compatible_mask,
        selected_mask,
        selected_suffix_len,
        disposition,
    }
}
