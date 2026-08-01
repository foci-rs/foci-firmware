use super::*;

/// Assemble the expected canonical-record bytes for one test, independently
/// of the encoder under test: each `push` call appends one field's exact
/// little-endian bytes, in the same order the production `encode_*` function
/// pushes them. Pinning the byte layout this way (rather than comparing two
/// calls to the same encoder) is what makes these true golden tests: a field
/// reorder, width change, or dropped field changes `expected.as_slice()`
/// without touching the encoder under test.
struct Expected(heapless::Vec<u8, 128>);

impl Expected {
    fn new() -> Self {
        Self(heapless::Vec::new())
    }

    fn u8(mut self, value: u8) -> Self {
        self.0.push(value).unwrap();
        self
    }

    fn u16(mut self, value: u16) -> Self {
        self.0.extend_from_slice(&value.to_le_bytes()).unwrap();
        self
    }

    fn u32(mut self, value: u32) -> Self {
        self.0.extend_from_slice(&value.to_le_bytes()).unwrap();
        self
    }

    fn i32(mut self, value: i32) -> Self {
        self.0.extend_from_slice(&value.to_le_bytes()).unwrap();
        self
    }

    fn u64(self, value: u64) -> Self {
        self.u32(value as u32).u32((value >> 32) as u32)
    }

    fn bytes(&self) -> &[u8] {
        self.0.as_slice()
    }
}

#[test]
fn encode_probe_planned_has_stable_frame_layout() {
    let report = BreakawayProbePlanned {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        family_size: 0x0910,
        max_observations: 0x11,
        search_count: 0x12,
        p_start_raw: 0x1314,
        p_top_raw: 0x1516,
    };
    let record = encode_probe_planned(report);
    let expected = Expected::new()
        .u8(0)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u16(report.family_size)
        .u8(report.max_observations)
        .u8(report.search_count)
        .u16(report.p_start_raw)
        .u16(report.p_top_raw);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_directional_breakaway_has_stable_frame_layout() {
    let report = BreakawayDirectionalBreakaway {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        direction: 1,
        inert_present: 1,
        inert_p_raw: 0x0910,
        moving_p_raw: 0x1112,
        observations: 6,
    };
    let record = encode_directional_breakaway(report);
    let expected = Expected::new()
        .u8(1)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.direction)
        .u8(report.inert_present)
        .u16(report.inert_p_raw)
        .u16(report.moving_p_raw)
        .u8(report.observations);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_probe_terminal_has_stable_frame_layout() {
    let report = BreakawayProbeTerminal {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        family_size: 0x0910,
        terminal_cause: 1,
        has_safety_fault: 0,
    };
    let record = encode_probe_terminal(report);
    let expected = Expected::new()
        .u8(2)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u16(report.family_size)
        .u8(report.terminal_cause)
        .u8(report.has_safety_fault);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_discovery_planned_has_stable_frame_layout() {
    let report = AdditiveDiscoveryPlanned {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        prior_plan_digest: 0x99aa_bbcc_ddee_ff00,
        family_size: 0x0910,
        rung_count: 8,
        observations_per_direction: 4,
        floor_p_raw: 0x1112,
        floor_origin: 1,
        breakaway_p_raw: 0x1314,
        ceiling_p_raw: 0x1516,
        first_additive_step_raw: 0x1718_1920,
        maximum_workflow_ms: 0x2122_2324,
    };
    let record = encode_discovery_planned(report);
    let expected = Expected::new()
        .u8(3)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u64(report.prior_plan_digest)
        .u16(report.family_size)
        .u8(report.rung_count)
        .u16(report.observations_per_direction)
        .u16(report.floor_p_raw)
        .u8(report.floor_origin)
        .u16(report.breakaway_p_raw)
        .u16(report.ceiling_p_raw)
        .u32(report.first_additive_step_raw)
        .u32(report.maximum_workflow_ms);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_ceiling_source_has_stable_frame_layout() {
    let report = BreakawayCeilingSource {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        binding_source: 1,
    };
    let record = encode_ceiling_source(report);
    let expected = Expected::new()
        .u8(4)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.binding_source);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_ladder_rung_has_stable_frame_layout() {
    let report = BreakawayLadderRung {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        rung_index: 3,
        p_raw: 0x0910,
    };
    let record = encode_ladder_rung(report);
    let expected = Expected::new()
        .u8(5)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.rung_index)
        .u16(report.p_raw);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_discovery_terminal_has_stable_frame_layout() {
    let report = AdditiveDiscoveryTerminal {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        prior_plan_digest: 0x99aa_bbcc_ddee_ff00,
        family_size: 0x0910,
        terminal_cause: 0,
        collected_count: 2,
        has_safety_fault: 0,
    };
    let record = encode_discovery_terminal(report);
    let expected = Expected::new()
        .u8(6)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u64(report.prior_plan_digest)
        .u16(report.family_size)
        .u8(report.terminal_cause)
        .u8(report.collected_count)
        .u8(report.has_safety_fault);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_discovery_rung_margin_has_stable_frame_layout() {
    let report = BreakawayDiscoveryRungMargin {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        collected_index: 1,
        p_raw: 0x0910,
        lower_percent_milli: 70_500,
        upper_percent_milli: 79_500,
        margin_percent_milli: 500,
        half_width_percent_milli: 4_500,
        adjacent_rate_step_permille: -1_200,
    };
    let record = encode_discovery_rung_margin(report);
    let expected = Expected::new()
        .u8(7)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.collected_index)
        .u16(report.p_raw)
        .i32(report.lower_percent_milli)
        .i32(report.upper_percent_milli)
        .i32(report.margin_percent_milli)
        .i32(report.half_width_percent_milli)
        .i32(report.adjacent_rate_step_permille);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_rung_zero_diagnostic_has_stable_frame_layout() {
    let report = BreakawayRungZeroDiagnostic {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        forward_moved: 1,
        reverse_moved: 0,
    };
    let record = encode_rung_zero_diagnostic(report);
    let expected = Expected::new()
        .u8(8)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.forward_moved)
        .u8(report.reverse_moved);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_confirmation_planned_has_stable_frame_layout() {
    let report = BandConfirmationPlanned {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        prior_plan_digest: 0x99aa_bbcc_ddee_ff00,
        family_size: 8,
        observations_per_direction: 4,
        candidate_p_raw: 0x0910,
        band_lower_percent: 70,
        band_upper_percent: 80,
        capture_profile: 0,
        acceptance_rule: 0,
        nominated_margin_percent_milli: 1_500,
    };
    let record = encode_confirmation_planned(report);
    let expected = Expected::new()
        .u8(9)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u64(report.prior_plan_digest)
        .u16(report.family_size)
        .u16(report.observations_per_direction)
        .u16(report.candidate_p_raw)
        .u8(report.band_lower_percent)
        .u8(report.band_upper_percent)
        .u8(report.capture_profile)
        .u8(report.acceptance_rule)
        .i32(report.nominated_margin_percent_milli);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_confirmation_terminal_has_stable_frame_layout() {
    let report = BandConfirmationTerminal {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        prior_plan_digest: 0x99aa_bbcc_ddee_ff00,
        family_size: 8,
        terminal_cause: 21,
        forward_collected_mask: 0b1111,
        forward_eligible_mask: 0b1111,
        forward_included_mask: 0b1111,
        reverse_collected_mask: 0b1111,
        reverse_eligible_mask: 0b1111,
        reverse_included_mask: 0b1111,
        accepted: 1,
        confirmed_p_raw: 0x0910,
        max_relative_se_permille: 20,
        required_relative_se_permille: 67,
        has_safety_fault: 0,
    };
    let record = encode_confirmation_terminal(report);
    let expected = Expected::new()
        .u8(10)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u64(report.prior_plan_digest)
        .u16(report.family_size)
        .u8(report.terminal_cause)
        .u8(report.forward_collected_mask)
        .u8(report.forward_eligible_mask)
        .u8(report.forward_included_mask)
        .u8(report.reverse_collected_mask)
        .u8(report.reverse_eligible_mask)
        .u8(report.reverse_included_mask)
        .u8(report.accepted)
        .u16(report.confirmed_p_raw)
        .u16(report.max_relative_se_permille)
        .u16(report.required_relative_se_permille)
        .u8(report.has_safety_fault);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_confirmation_observation_has_stable_frame_layout() {
    let report = BandConfirmationObservation {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        plan_digest: 0x1122_3344_5566_7788,
        slot_index: 3,
        direction: 1,
        response_lower_percent_milli: 70_800,
        response_upper_percent_milli: 79_200,
        relative_standard_error_permille: 18,
    };
    let record = encode_confirmation_observation(report);
    let expected = Expected::new()
        .u8(11)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u64(report.plan_digest)
        .u8(report.slot_index)
        .u8(report.direction)
        .i32(report.response_lower_percent_milli)
        .i32(report.response_upper_percent_milli)
        .u16(report.relative_standard_error_permille);
    assert_eq!(record.as_bytes(), expected.bytes());
}

#[test]
fn encode_campaign_terminal_has_stable_frame_layout() {
    let report = BreakawayCampaignTerminal {
        run_sequence: 0x1020_3040,
        evidence_sequence: 0x5060,
        phase: 2,
        terminal_cause: 21,
        accepted: 1,
        stage_c_plan_digest: 0x1122_3344_5566_7788,
    };
    let record = encode_campaign_terminal(report);
    let expected = Expected::new()
        .u8(12)
        .u32(report.run_sequence)
        .u16(report.evidence_sequence)
        .u8(report.phase)
        .u8(report.terminal_cause)
        .u8(report.accepted)
        .u64(report.stage_c_plan_digest);
    assert_eq!(record.as_bytes(), expected.bytes());
}

/// Local tags are per-record-kind discriminants scoped to the breakaway
/// family (see this module's own doc comment); pin them explicitly so an
/// accidental tag collision between two breakaway encoders is caught here,
/// not only downstream in a host decoder.
#[test]
fn every_encoder_uses_a_distinct_local_tag() {
    let tags = [
        encode_probe_planned(BreakawayProbePlanned {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            family_size: 0,
            max_observations: 0,
            search_count: 0,
            p_start_raw: 0,
            p_top_raw: 0,
        })
        .as_bytes()[0],
        encode_directional_breakaway(BreakawayDirectionalBreakaway {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            direction: 0,
            inert_present: 0,
            inert_p_raw: 0,
            moving_p_raw: 0,
            observations: 0,
        })
        .as_bytes()[0],
        encode_probe_terminal(BreakawayProbeTerminal {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            family_size: 0,
            terminal_cause: 0,
            has_safety_fault: 0,
        })
        .as_bytes()[0],
        encode_discovery_planned(AdditiveDiscoveryPlanned {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            prior_plan_digest: 0,
            family_size: 0,
            rung_count: 0,
            observations_per_direction: 0,
            floor_p_raw: 0,
            floor_origin: 0,
            breakaway_p_raw: 0,
            ceiling_p_raw: 0,
            first_additive_step_raw: 0,
            maximum_workflow_ms: 0,
        })
        .as_bytes()[0],
        encode_ceiling_source(BreakawayCeilingSource {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            binding_source: 0,
        })
        .as_bytes()[0],
        encode_ladder_rung(BreakawayLadderRung {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            rung_index: 0,
            p_raw: 0,
        })
        .as_bytes()[0],
        encode_discovery_terminal(AdditiveDiscoveryTerminal {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            prior_plan_digest: 0,
            family_size: 0,
            terminal_cause: 0,
            collected_count: 0,
            has_safety_fault: 0,
        })
        .as_bytes()[0],
        encode_discovery_rung_margin(BreakawayDiscoveryRungMargin {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            collected_index: 0,
            p_raw: 0,
            lower_percent_milli: 0,
            upper_percent_milli: 0,
            margin_percent_milli: 0,
            half_width_percent_milli: 0,
            adjacent_rate_step_permille: 0,
        })
        .as_bytes()[0],
        encode_rung_zero_diagnostic(BreakawayRungZeroDiagnostic {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            forward_moved: 0,
            reverse_moved: 0,
        })
        .as_bytes()[0],
        encode_confirmation_planned(BandConfirmationPlanned {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            prior_plan_digest: 0,
            family_size: 0,
            observations_per_direction: 0,
            candidate_p_raw: 0,
            band_lower_percent: 0,
            band_upper_percent: 0,
            capture_profile: 0,
            acceptance_rule: 0,
            nominated_margin_percent_milli: 0,
        })
        .as_bytes()[0],
        encode_confirmation_terminal(BandConfirmationTerminal {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            prior_plan_digest: 0,
            family_size: 0,
            terminal_cause: 0,
            forward_collected_mask: 0,
            forward_eligible_mask: 0,
            forward_included_mask: 0,
            reverse_collected_mask: 0,
            reverse_eligible_mask: 0,
            reverse_included_mask: 0,
            accepted: 0,
            confirmed_p_raw: 0,
            max_relative_se_permille: 0,
            required_relative_se_permille: 0,
            has_safety_fault: 0,
        })
        .as_bytes()[0],
        encode_confirmation_observation(BandConfirmationObservation {
            run_sequence: 0,
            evidence_sequence: 0,
            plan_digest: 0,
            slot_index: 0,
            direction: 0,
            response_lower_percent_milli: 0,
            response_upper_percent_milli: 0,
            relative_standard_error_permille: 0,
        })
        .as_bytes()[0],
        encode_campaign_terminal(BreakawayCampaignTerminal {
            run_sequence: 0,
            evidence_sequence: 0,
            phase: 0,
            terminal_cause: 0,
            accepted: 0,
            stage_c_plan_digest: 0,
        })
        .as_bytes()[0],
    ];
    for (index, tag) in tags.iter().enumerate() {
        assert_eq!(
            usize::from(*tag),
            index,
            "encoder at position {index} does not use local tag {index}"
        );
        for (other_index, other_tag) in tags.iter().enumerate() {
            if other_index != index {
                assert_ne!(tag, other_tag, "tags {index} and {other_index} collide");
            }
        }
    }
}
