//! Cross-check guard binding `CommissionPhase`/`BreakawayPhase` to the
//! checked-in vocabulary fixture shared with the Python decode tables
//! (`host/klipper-foci`, `host/foci-trace`; spec S7).
//!
//! In-crate rather than an external `tests/` crate: `BreakawayPhase` is
//! `pub(crate)`, so an external crate could not name it without widening its
//! visibility.

use std::collections::BTreeSet;
use std::string::String;
use std::vec::Vec;

use crate::autotune::CommissionPhase;
use crate::commissioning::outer::velocity::breakaway_reporting::BreakawayPhase;

const FIXTURE: &str = include_str!("vocabulary_fixture.json");

struct FixtureEntry {
    group: String,
    ident: String,
    wire_code: u8,
}

/// Parses `vocabulary_fixture.json`'s fixed shape: a JSON array of flat
/// objects with only string and integer fields, no nesting.
///
/// Hand-rolled instead of pulling in a JSON crate: the fixture's shape is
/// fixed and checked in, so a general-purpose parser would be dead weight
/// for a test-only guard.
fn parse_fixture(raw: &str) -> Vec<FixtureEntry> {
    let mut chars = raw.char_indices().peekable();
    for (_, c) in chars.by_ref() {
        if c == '[' {
            break;
        }
    }

    let mut entries = Vec::new();
    loop {
        let mut found_object = false;
        while let Some(&(_, c)) = chars.peek() {
            match c {
                '{' => {
                    chars.next();
                    found_object = true;
                    break;
                }
                ']' => {
                    chars.next();
                    return entries;
                }
                _ => {
                    chars.next();
                }
            }
        }
        if !found_object {
            return entries;
        }

        let mut group = String::new();
        let mut ident = String::new();
        let mut wire_code: Option<u8> = None;
        loop {
            while let Some(&(_, c)) = chars.peek() {
                if c.is_whitespace() || c == ',' {
                    chars.next();
                } else {
                    break;
                }
            }
            match chars.peek().copied() {
                Some((_, '}')) => {
                    chars.next();
                    break;
                }
                Some((_, '"')) => {
                    chars.next();
                    let key = take_string(&mut chars);
                    while let Some(&(_, c)) = chars.peek() {
                        if c.is_whitespace() || c == ':' {
                            chars.next();
                        } else {
                            break;
                        }
                    }
                    match key.as_str() {
                        "group" => {
                            chars.next();
                            group = take_string(&mut chars);
                        }
                        "ident" => {
                            chars.next();
                            ident = take_string(&mut chars);
                        }
                        "label" => {
                            chars.next();
                            let _ = take_string(&mut chars);
                        }
                        "wire_code" => {
                            let digits = take_digits(&mut chars);
                            wire_code = Some(digits.parse().expect("wire_code must parse as u8"));
                        }
                        other => panic!("unexpected fixture key: {other}"),
                    }
                }
                other => panic!("unexpected fixture token: {other:?}"),
            }
        }
        entries.push(FixtureEntry {
            group,
            ident,
            wire_code: wire_code.expect("fixture entry missing wire_code"),
        });
    }
}

fn take_string(chars: &mut core::iter::Peekable<core::str::CharIndices<'_>>) -> String {
    let mut s = String::new();
    for (_, c) in chars.by_ref() {
        if c == '"' {
            break;
        }
        s.push(c);
    }
    s
}

fn take_digits(chars: &mut core::iter::Peekable<core::str::CharIndices<'_>>) -> String {
    while let Some(&(_, c)) = chars.peek() {
        if c.is_whitespace() {
            chars.next();
        } else {
            break;
        }
    }
    let mut s = String::new();
    while let Some(&(_, c)) = chars.peek() {
        if c.is_ascii_digit() {
            s.push(c);
            chars.next();
        } else {
            break;
        }
    }
    s
}

fn fixture_group(group: &str) -> BTreeSet<(String, u8)> {
    parse_fixture(FIXTURE)
        .into_iter()
        .filter(|e| e.group == group)
        .map(|e| (e.ident, e.wire_code))
        .collect()
}

/// The live sets come from each enum's macro-generated `VOCABULARY`, not from
/// a list restated here: a hand-maintained inventory would silently accept a
/// new variant that never reached the fixture or the Python tables, which is
/// the exact single-layer desynchronization these tests exist to catch.
#[test]
fn commission_phase_matches_fixture_exactly() {
    let live: BTreeSet<(String, u8)> = CommissionPhase::VOCABULARY
        .iter()
        .map(|(ident, phase)| (String::from(*ident), phase.to_u8()))
        .collect();

    assert_eq!(
        live,
        fixture_group("phase"),
        "CommissionPhase and vocabulary_fixture.json's `phase` group have diverged"
    );
}

#[test]
fn breakaway_phase_matches_fixture_exactly() {
    let live: BTreeSet<(String, u8)> = BreakawayPhase::VOCABULARY
        .iter()
        .map(|(ident, phase)| (String::from(*ident), *phase as u8))
        .collect();

    assert_eq!(
        live,
        fixture_group("breakaway"),
        "BreakawayPhase and vocabulary_fixture.json's `breakaway` group have diverged"
    );
}
