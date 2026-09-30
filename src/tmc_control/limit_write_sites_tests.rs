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
];

const LIMIT_REGISTER: &str = "PID_TORQUE_FLUX_LIMITS";
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

fn first_argument(arguments: &str) -> &str {
    let mut depth = 0;
    for (offset, ch) in arguments.char_indices() {
        match ch {
            '(' | '[' | '{' => depth += 1,
            ')' | ']' | '}' => depth -= 1,
            ',' if depth == 0 => return &arguments[..offset],
            _ => {}
        }
    }
    arguments
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

fn limit_writes_in(file: &str, source: &str) -> Vec<LimitWrite> {
    let source = strip_cfg_test_items(source);
    let mut writes = Vec::new();
    for needle in ["write_register(", "write_register_verified("] {
        for (at, _) in source.match_indices(needle) {
            if source[..at].trim_end().ends_with("fn") {
                continue;
            }
            let open = at + needle.len() - 1;
            let arguments = &source[open + 1..matching_paren(&source, open)];
            if !first_argument(arguments).contains(LIMIT_REGISTER) {
                continue;
            }
            writes.push(LimitWrite {
                file: file.into(),
                line: source[..at].matches('\n').count() + 1,
                function: enclosing_function(&source, at),
                routed: arguments.contains(CHOKEPOINT),
            });
        }
    }
    writes
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
