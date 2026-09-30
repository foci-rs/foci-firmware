use std::path::{Path, PathBuf};
use std::string::String;
use std::vec::Vec;

const LIMIT_WRITE_EXEMPTIONS: &[(&str, &str, &str)] = &[
    (
        "tmc_control/state.rs",
        "advance_offset_headroom",
        "reservation engage: writes the reduced limit before any non-zero offset",
    ),
    (
        "tmc_control/state.rs",
        "release_offset_headroom_step",
        "reservation release: writes the committed or pending limit after the offset is zero",
    ),
    (
        "tmc_control/state.rs",
        "apply_homing_current_clamp",
        "homing clamp entry: below the committed limit, reservation released for all of homing",
    ),
    (
        "tmc_control/request_writes.rs",
        "apply_set_current",
        "writes the requested limit only while the reservation is clear, deferring it otherwise",
    ),
    (
        "commissioning/inner/encoder.rs",
        "start_encoder_current_drive",
        "commissioning: runs only after the exclusive-activity release, engagement blocked",
    ),
    (
        "commissioning/inner/current_loop.rs",
        "step_current_validate",
        "commissioning: runs only after the exclusive-activity release, engagement blocked",
    ),
    (
        "commissioning/outer/velocity/velocity_clamp.rs",
        "install_probe_limit",
        "commissioning probe: runs only after the exclusive-activity release",
    ),
    (
        "commissioning/outer/velocity/velocity_clamp.rs",
        "restore_operational_limit",
        "commissioning probe restore: runs only after the exclusive-activity release",
    ),
    (
        "tmc_diagnostics/torque_sample.rs",
        "run_position_torque_offset_sample_request",
        "offset diagnostic: lowers the limit by the offset magnitude before writing the offset",
    ),
    (
        "tmc_diagnostics/torque_sample.rs",
        "restore_offset_then_limit",
        "offset diagnostic: restores the committed limit only after the offset clear succeeded",
    ),
];

const LIMIT_REGISTER: &str = "PID_TORQUE_FLUX_LIMITS";
const WRITE_CALLEE: &str = "write_register";
const CHOKEPOINT: &str = "operational_limit_register(";

struct LimitWrite {
    file: String,
    line: usize,
    function: String,
    routed: bool,
}

impl core::fmt::Debug for LimitWrite {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}:{} in {}", self.file, self.line, self.function)
    }
}

fn is_test_only_file(path: &Path) -> bool {
    let name = path
        .file_name()
        .and_then(|name| name.to_str())
        .unwrap_or("");
    matches!(name, "tests.rs" | "test_support.rs" | "test_spi.rs") || name.ends_with("_tests.rs")
}

fn collect_sources(dir: &Path, out: &mut Vec<PathBuf>) {
    let mut entries: Vec<PathBuf> = std::fs::read_dir(dir)
        .unwrap_or_else(|error| panic!("read {}: {error}", dir.display()))
        .map(|entry| entry.expect("directory entry").path())
        .collect();
    entries.sort();
    for path in entries {
        if path.is_dir() {
            collect_sources(&path, out);
        } else if path.extension().is_some_and(|ext| ext == "rs") && !is_test_only_file(&path) {
            out.push(path);
        }
    }
}

/// Brace delta of one line, ignoring braces in string literals, `'{'`/`'}'`, and line comments.
fn brace_delta(line: &str) -> i32 {
    let line = line.replace("'{'", "").replace("'}'", "");
    let mut delta = 0;
    let mut in_string = false;
    let mut chars = line.chars().peekable();
    while let Some(ch) = chars.next() {
        match (in_string, ch) {
            (true, '\\') => {
                chars.next();
            }
            (true, '"') | (false, '"') => in_string = !in_string,
            (false, '/') if chars.peek() == Some(&'/') => break,
            (false, '{') => delta += 1,
            (false, '}') => delta -= 1,
            _ => {}
        }
    }
    delta
}

/// Blank out every `#[cfg(test)]` item, keeping line numbers.
fn strip_cfg_test_items(source: &str) -> String {
    let lines: Vec<&str> = source.lines().collect();
    let mut kept = Vec::with_capacity(lines.len());
    let mut index = 0;
    while index < lines.len() {
        if lines[index].trim() != "#[cfg(test)]" {
            kept.push(lines[index]);
            index += 1;
            continue;
        }
        let mut depth = 0;
        let mut opened = false;
        while index < lines.len() {
            let line = lines[index];
            kept.push("");
            index += 1;
            let trimmed = line.trim();
            if trimmed.starts_with("#[") && !opened {
                continue;
            }
            depth += brace_delta(line);
            opened |= line.contains('{');
            if (opened && depth <= 0)
                || (!opened && (trimmed.ends_with(';') || trimmed.ends_with(',')))
            {
                break;
            }
        }
    }
    kept.join("\n")
}

fn matching_paren(source: &str, open: usize) -> usize {
    let mut depth = 0;
    let mut in_string = false;
    let mut escaped = false;
    for (offset, ch) in source[open..].char_indices() {
        match (in_string, escaped, ch) {
            (true, true, _) => escaped = false,
            (true, false, '\\') => escaped = true,
            (_, false, '"') => in_string = !in_string,
            (false, _, '(') => depth += 1,
            (false, _, ')') => {
                depth -= 1;
                if depth == 0 {
                    return open + offset;
                }
            }
            _ => {}
        }
    }
    panic!("unbalanced call at byte {open}");
}

fn enclosing_function(source: &str, offset: usize) -> String {
    source[..offset]
        .rmatch_indices("fn ")
        .filter(|(at, _)| {
            source[..*at]
                .chars()
                .next_back()
                .is_none_or(char::is_whitespace)
        })
        .find_map(|(at, _)| {
            let name: String = source[at + 3..]
                .chars()
                .take_while(|ch| ch.is_ascii_alphanumeric() || *ch == '_')
                .collect();
            (!name.is_empty()).then_some(name)
        })
        .unwrap_or_default()
}

fn is_identifier_char(ch: char) -> bool {
    ch.is_ascii_alphanumeric() || ch == '_'
}

/// Byte offsets of every call whose callee name contains `write_register`, as
/// `(callee start, opening paren)`.
fn write_calls(source: &str) -> Vec<(usize, usize)> {
    let mut calls = Vec::new();
    let mut resume = 0;
    for (at, _) in source.match_indices(WRITE_CALLEE) {
        if at < resume {
            continue;
        }
        let start = source[..at]
            .char_indices()
            .rev()
            .take_while(|(_, ch)| is_identifier_char(*ch))
            .last()
            .map_or(at, |(index, _)| index);
        let end = at
            + source[at..]
                .find(|ch| !is_identifier_char(ch))
                .unwrap_or(source.len() - at);
        resume = end;
        if source[end..].starts_with('(') && !source[..start].trim_end().ends_with("fn") {
            calls.push((start, end));
        }
    }
    calls
}

/// Textual scan: matches only the literal `PID_TORQUE_FLUX_LIMITS` token in a `write_register`
/// call's arguments, so an alias, a table-driven write, a raw address or a turbofish call is not
/// detected.
fn limit_writes_in(file: &str, source: &str) -> Vec<LimitWrite> {
    let source = strip_cfg_test_items(source);
    write_calls(&source)
        .into_iter()
        .filter_map(|(start, open)| {
            let arguments = &source[open + 1..matching_paren(&source, open)];
            arguments.contains(LIMIT_REGISTER).then(|| LimitWrite {
                file: file.into(),
                line: source[..start].matches('\n').count() + 1,
                function: enclosing_function(&source, start),
                routed: arguments.contains(CHOKEPOINT),
            })
        })
        .collect()
}

fn production_limit_writes() -> Vec<LimitWrite> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("src");
    let mut sources = Vec::new();
    collect_sources(&root, &mut sources);
    sources
        .iter()
        .flat_map(|path| {
            let relative = path
                .strip_prefix(&root)
                .expect("source under src")
                .components()
                .map(|part| part.as_os_str().to_string_lossy().into_owned())
                .collect::<Vec<_>>()
                .join("/");
            let source = std::fs::read_to_string(path)
                .unwrap_or_else(|error| panic!("read {}: {error}", path.display()));
            limit_writes_in(&relative, &source)
        })
        .collect()
}

#[test]
fn every_operational_limit_write_routes_through_the_chokepoint() {
    let writes = production_limit_writes();

    let unrouted: Vec<&LimitWrite> = writes
        .iter()
        .filter(|write| {
            !write.routed
                && !LIMIT_WRITE_EXEMPTIONS
                    .iter()
                    .any(|(file, function, _)| write.file == *file && write.function == *function)
        })
        .collect();
    assert!(
        unrouted.is_empty(),
        "limit writes must take their value from operational_limit_register or be exempted \
         with a reason: {unrouted:#?}"
    );
    let stale: Vec<&(&str, &str, &str)> = LIMIT_WRITE_EXEMPTIONS
        .iter()
        .filter(|(file, function, _)| {
            !writes
                .iter()
                .any(|write| !write.routed && write.file == *file && write.function == *function)
        })
        .collect();
    assert!(
        stale.is_empty(),
        "exemptions without an unrouted limit write: {stale:#?}"
    );
    assert!(
        writes.iter().any(|write| write.routed),
        "no routed limit write found: {writes:#?}"
    );
}

fn fixture_writes(source: &str) -> Vec<LimitWrite> {
    limit_writes_in("fixture.rs", source)
}

#[test]
fn scanner_finds_a_write_with_the_register_first() {
    let source = "fn set(b: &mut B) {\n    b.write_register(PID_TORQUE_FLUX_LIMITS, value);\n}\n";

    let writes = fixture_writes(source);

    assert_eq!(writes.len(), 1);
    assert_eq!(writes[0].function, "set");
    assert!(!writes[0].routed);
}

#[test]
fn scanner_finds_a_wrapper_write_with_the_register_after_the_first_argument() {
    let source = "async fn set(b: &mut B) {\n    write_register(backend, PID_TORQUE_FLUX_LIMITS, \
                  value).await;\n    write_register_verified(backend, PID_TORQUE_FLUX_LIMITS, \
                  value).await;\n}\n";

    let writes = fixture_writes(source);

    assert_eq!(writes.len(), 2);
    assert!(
        writes
            .iter()
            .all(|write| write.function == "set" && !write.routed)
    );
}

#[test]
fn scanner_ignores_reads_of_the_register() {
    let source = "fn get(b: &mut B) {\n    b.read_register(PID_TORQUE_FLUX_LIMITS);\n    \
                  read_sub_register(backend, PID_TORQUE_FLUX_LIMITS);\n}\n";

    assert!(fixture_writes(source).is_empty());
}

#[test]
fn scanner_skips_cfg_test_modules() {
    let source = "#[cfg(test)]\nmod tests {\n    fn t(b: &mut B) {\n        \
                  write_register(b, PID_TORQUE_FLUX_LIMITS, 1);\n    }\n}\n";

    assert!(fixture_writes(source).is_empty());
}

#[test]
fn scanner_classifies_a_chokepoint_call_as_routed() {
    let source = "fn set(b: &mut B) {\n    write_register(backend, PID_TORQUE_FLUX_LIMITS, \
                  operational_limit_register(ch, limit));\n}\n";

    let writes = fixture_writes(source);

    assert_eq!(writes.len(), 1);
    assert!(writes[0].routed);
}
