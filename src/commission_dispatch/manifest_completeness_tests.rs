use std::{
    format,
    string::{String, ToString},
    vec::Vec,
};

const TRAIT_SOURCE: &str = include_str!("replies.rs");

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
        let fn_start = search_from + rel_fn + 1;
        let after_fn = &trait_body[fn_start + 7..];
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
        19,
        "expected exactly the 19 bespoke required methods in the trait's \
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
        entry_count, 24,
        "expected 24 entries in __commission_reply_manifest! (22 plain + \
         foci_robustness_cycle_evidence + foci_commission_timing); found \
         {entry_count}. If you added or removed a manifest entry on \
         purpose, update this count."
    );
}

const ANKYRA_REPLIES_SOURCE: &str = include_str!("../ankyra/replies.rs");

fn strip_lifetime(ty: &str) -> String {
    ty.replace("'a ", "").trim().to_string()
}

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

fn manifest_entry_fields(source: &str, method: &str) -> (String, Vec<(String, String)>) {
    let marker = format!("name: {method},");
    let start = source
        .find(&marker)
        .unwrap_or_else(|| panic!("manifest entry {method} not found"));
    let after = &source[start..];
    let entry_end = after.find("entry {").unwrap_or(after.len());
    let entry = &after[..entry_end];

    const REPLY_MARKER: &str = "reply: foci_firmware::ankyra::replies::";
    let reply_start = entry
        .find(REPLY_MARKER)
        .map(|i| i + REPLY_MARKER.len())
        .unwrap_or_else(|| panic!("{method} manifest entry has no reply: field"));
    let reply_rest = &entry[reply_start..];
    let reply_end = reply_rest
        .find([',', '<'])
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
        24,
        "expected 24 manifest entry names; found {}: {names:?}",
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
