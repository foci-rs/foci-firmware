//! Completeness and drift regression tests for `CommissionReplySink`. Two
//! properties: (1) the trait must not gain a hand-typed default-bodied
//! method outside the `__commission_reply_manifest!` invocation -- a
//! default-bodied method that nobody wired into the manifest lets every
//! board sink silently skip forwarding that reply; (2) every manifest
//! entry's declared reply type and field list must match the actual
//! `ankyra::replies` struct it names -- a drifted field can compile and
//! forward successfully while silently changing the wire protocol. Both
//! turn a silent failure into an immediate, attributable build failure.

// foci-firmware is #![no_std], but lib.rs already has
// `#[cfg(test)] extern crate std;` crate-wide -- std is linked for every
// test module, this one included. Vec/String/format!/ToString need this
// explicit import regardless: the crate root's `#![no_std]` means every
// module resolves against `core::prelude`, not `std::prelude`, so none of
// these ride in automatically the way they would in an ordinary std crate.
use std::{
    format,
    string::{String, ToString},
    vec::Vec,
};

const TRAIT_SOURCE: &str = include_str!("replies.rs");

/// Every method signature found directly in the trait's hand-written
/// region, paired with its terminator: `;` (required, no body) or `{`
/// (default body). Deliberately not scoped to `fn foci_*` -- the gate must
/// reject *any* default-bodied method someone hand-types into the trait,
/// not only ones following today's naming convention. Anchored on
/// `"\n    fn "` (a method declaration starting at the trait's own
/// indentation level) rather than a bare `"fn "` search, so a `fn` occurring
/// inside a doc comment's prose can't match. The manifest's own
/// `entry { ... }` data blocks never produce that anchor (entries are
/// declared as `name: foci_x`, never as a line starting with `fn `), so
/// nothing needs to be excluded by region -- scanning the whole trait body
/// is sufficient.
fn trait_method_terminators(source: &str) -> Vec<(String, char)> {
    let trait_start = source
        .find("pub trait CommissionReplySink")
        .expect("CommissionReplySink trait definition not found");
    let body = &source[trait_start..];
    let trait_end = body
        .find("\n}\n")
        .expect("could not find CommissionReplySink trait's closing brace");
    let trait_body = &body[..trait_end];

    const ANCHOR: &str = "\n    fn ";
    let mut results = Vec::new();
    let mut search_from = 0usize;
    while let Some(rel_fn) = trait_body[search_from..].find(ANCHOR) {
        let fn_start = search_from + rel_fn + 1; // +1: skip the leading '\n', keep "    fn "
        let after_fn = &trait_body[fn_start + 7..]; // skip "    fn "
        let name_end = after_fn
            .find(|c: char| c == '(' || c.is_whitespace())
            .expect("malformed method signature: no name terminator");
        let name = after_fn[..name_end].to_string();

        let paren_start = trait_body[fn_start..]
            .find('(')
            .map(|i| fn_start + i)
            .expect("malformed method signature: no parameter list");

        let mut depth = 0i32;
        let mut paren_end = None;
        for (offset, ch) in trait_body[paren_start..].char_indices() {
            match ch {
                '(' => depth += 1,
                ')' => {
                    depth -= 1;
                    if depth == 0 {
                        paren_end = Some(paren_start + offset);
                        break;
                    }
                }
                _ => {}
            }
        }
        let paren_end = paren_end.expect("unbalanced parameter list parens");

        let after_params = trait_body[paren_end + 1..].trim_start();
        let terminator = after_params
            .chars()
            .next()
            .expect("nothing follows method parameter list");
        assert!(
            terminator == ';' || terminator == '{',
            "unexpected terminator '{terminator}' after {name}'s parameter list"
        );

        results.push((name, terminator));
        search_from = paren_end + 1;
    }
    results
}

#[test]
fn hand_written_region_has_no_default_bodied_methods() {
    let methods = trait_method_terminators(TRAIT_SOURCE);
    assert_eq!(
        methods.len(),
        20,
        "expected exactly the 20 bespoke required methods in the trait's \
         hand-written region; found {}: {methods:?}. If this changed on \
         purpose, update this count -- otherwise something didn't migrate \
         into __commission_reply_manifest! as expected.",
        methods.len()
    );

    let defaults: Vec<_> = methods
        .iter()
        .filter(|(_, term)| *term == '{')
        .map(|(name, _)| name.clone())
        .collect();
    assert!(
        defaults.is_empty(),
        "found default-bodied CommissionReplySink method(s) outside the \
         manifest, silently droppable by any board sink that doesn't \
         override them: {defaults:?}. Add them to \
         __commission_reply_manifest! in replies.rs instead of hand-typing \
         a default body -- an unmanifested default body is exactly how a \
         reply gets silently dropped by every board sink at once."
    );
}

#[test]
fn manifest_has_the_expected_entry_count() {
    let entry_count = TRAIT_SOURCE.matches("entry {").count();
    assert_eq!(
        entry_count, 26,
        "expected 26 entries in __commission_reply_manifest! (24 plain + \
         foci_robustness_cycle_evidence + foci_commission_timing); found \
         {entry_count}. If you added or removed a manifest entry on \
         purpose, update this count."
    );
}

/// Wire reply structs live in `ankyra/replies.rs`, as
/// `pub struct <Name>[<'a>] { /// doc pub <field>: <type>, ... }`.
const ANKYRA_REPLIES_SOURCE: &str = include_str!("../ankyra/replies.rs");

/// Strips the codebase's one lifetime name so `&'a [u8]` (the struct
/// definition's own field type) and `&[u8]` (the manifest's field type,
/// which never carries an explicit lifetime -- it isn't needed at the
/// macro-input level) compare equal.
fn strip_lifetime(ty: &str) -> String {
    ty.replace("'a ", "").trim().to_string()
}

/// Ordered (name, type) pairs for a struct's fields, found by locating
/// `pub struct <name>` and reading each `pub <field>: <type>,` line in its
/// body up to the closing brace.
fn struct_field_list(source: &str, struct_name: &str) -> Vec<(String, String)> {
    let marker = format!("pub struct {struct_name}");
    let start = source
        .find(&marker)
        .unwrap_or_else(|| panic!("struct {struct_name} not found in ankyra::replies"));
    let body_start = source[start..]
        .find('{')
        .map(|i| start + i + 1)
        .unwrap_or_else(|| panic!("struct {struct_name} has no body"));
    let body_end = source[body_start..]
        .find("\n}")
        .map(|i| body_start + i)
        .unwrap_or_else(|| panic!("struct {struct_name} body not closed"));
    let body = &source[body_start..body_end];

    let mut fields = Vec::new();
    let mut search_from = 0usize;
    while let Some(rel) = body[search_from..].find("pub ") {
        let field_start = search_from + rel + 4;
        let rest = &body[field_start..];
        let colon = rest.find(':').expect("field with no type");
        let name = rest[..colon].trim().to_string();
        let comma = rest[colon + 1..]
            .find(',')
            .expect("field with no trailing comma");
        let ty = strip_lifetime(rest[colon + 1..colon + 1 + comma].trim());
        fields.push((name, ty));
        search_from = field_start + colon + 1 + comma;
    }
    fields
}

/// The wire struct name (stripped of the `foci_firmware::ankyra::replies::`
/// prefix and any lifetime suffix) and ordered (name, type) field pairs
/// declared in one manifest entry's `fields: { ... }` block.
fn manifest_entry_fields(source: &str, method: &str) -> (String, Vec<(String, String)>) {
    let marker = format!("name: {method},");
    let start = source
        .find(&marker)
        .unwrap_or_else(|| panic!("manifest entry {method} not found"));
    let after = &source[start..];
    // Bound to this one entry, same reasoning as the board host-tests'
    // assert_manifest_entry helper: `after` starts at "name: <method>", strictly
    // after this entry's own opening "entry {", so the next occurrence of
    // "entry {" (if any) is the following entry's. Without this bound, an
    // entry missing its own reply:/fields: block would silently read the
    // *next* entry's instead of failing -- a false pass, not caught by
    // adjacent entries that happen to share the same field shape (e.g.
    // foci_fixed_gain_amplitude_terminal and foci_robustness_cycle_evidence
    // are both just { oid: u8, payload: &[u8] }).
    let entry_end = after.find("entry {").unwrap_or(after.len());
    let entry = &after[..entry_end];

    const REPLY_MARKER: &str = "reply: foci_firmware::ankyra::replies::";
    let reply_start = entry
        .find(REPLY_MARKER)
        .map(|i| i + REPLY_MARKER.len())
        .unwrap_or_else(|| panic!("{method} manifest entry has no reply: field"));
    let reply_rest = &entry[reply_start..];
    let reply_end = reply_rest
        .find(|c: char| c == ',' || c == '<')
        .unwrap_or_else(|| panic!("{method} manifest entry's reply: field is malformed"));
    let struct_name = reply_rest[..reply_end].trim().to_string();

    const FIELDS_MARKER: &str = "fields: {";
    let fields_start = entry
        .find(FIELDS_MARKER)
        .map(|i| i + FIELDS_MARKER.len())
        .unwrap_or_else(|| panic!("{method} manifest entry has no fields: block"));
    let fields_rest = &entry[fields_start..];
    let fields_end = fields_rest
        .find('}')
        .unwrap_or_else(|| panic!("{method} manifest entry's fields: block is unclosed"));
    let fields_body = &fields_rest[..fields_end];

    let mut fields = Vec::new();
    for raw in fields_body.split(',') {
        let raw = raw.trim();
        if raw.is_empty() {
            continue;
        }
        let colon = raw.find(':').expect("manifest field with no type");
        let name = raw[..colon].trim().to_string();
        let ty = strip_lifetime(raw[colon + 1..].trim());
        fields.push((name, ty));
    }
    (struct_name, fields)
}

#[test]
fn manifest_entries_match_their_wire_reply_struct_fields() {
    // Every `name: foci_x,` occurrence inside the manifest -- distinct from
    // trait_method_terminators, which scans the hand-written region.
    let mut names = Vec::new();
    let mut search_from = 0usize;
    while let Some(rel) = TRAIT_SOURCE[search_from..].find("name: foci_") {
        let start = search_from + rel + "name: ".len();
        let rest = &TRAIT_SOURCE[start..];
        let end = rest
            .find(',')
            .expect("manifest entry name with no trailing comma");
        names.push(rest[..end].trim().to_string());
        search_from = start + end;
    }
    assert_eq!(
        names.len(),
        26,
        "expected 26 manifest entry names; found {}: {names:?}",
        names.len()
    );

    for method in &names {
        let (struct_name, manifest_fields) = manifest_entry_fields(TRAIT_SOURCE, method);
        let struct_fields = struct_field_list(ANKYRA_REPLIES_SOURCE, &struct_name);
        assert_eq!(
            manifest_fields, struct_fields,
            "{method}'s manifest fields do not match {struct_name}'s actual \
             field list (name, order, or type) in ankyra/replies.rs -- a \
             mismatch here can compile and forward successfully while \
             silently changing the wire protocol, defeating the pure-refactor \
             requirement"
        );
    }
}
