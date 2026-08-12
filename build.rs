//! Injects the firmware image identity (`BUILD_IDENTITY`) at compile time.
//!
//! `build_identity` attests which `foci-firmware` commit produced a
//! capture, so two builds of the same crate version are distinguishable -- the
//! discrimination `build_revision` (a hash of the crate version string) could
//! never provide. It is the first four bytes of the `HEAD` commit SHA-1, or a
//! sentinel when no commit can be resolved (a vendored tarball or a build
//! outside a git checkout).
//!
//! A dirty working tree is deliberately not encoded here: a single bit could
//! label an uncommitted build but never distinguish two of them, and the
//! host-side artifact SHA-256 recorded beside a capture is the ground-truth
//! identity for that case. `build_features` carries the analysis-rule contract
//! separately, so this value is pure provenance.

use std::path::Path;
use std::process::Command;

/// Sentinel written when no git commit can be resolved.
const UNKNOWN_BUILD_IDENTITY: u32 = 0;

fn main() {
    let identity = git_head_identity().unwrap_or(UNKNOWN_BUILD_IDENTITY);

    let out_dir = std::env::var("OUT_DIR").expect("cargo sets OUT_DIR for build scripts");
    let dest = Path::new(&out_dir).join("build_identity.rs");
    std::fs::write(
        &dest,
        format!("/// First four bytes of the foci-firmware HEAD commit, or 0 when unresolved.\npub const BUILD_IDENTITY: u32 = {identity:#010x};\n"),
    )
    .expect("write build_identity.rs into OUT_DIR");

    // Re-stamp the identity whenever HEAD or its target ref moves. `git` resolves
    // the git directory correctly for a submodule (whose `.git` is a file), which
    // walking the tree by hand would not.
    if let Ok(git_dir) = run_git(&["rev-parse", "--absolute-git-dir"]) {
        let git_dir = Path::new(git_dir.trim());
        let head = git_dir.join("HEAD");
        if head.exists() {
            println!("cargo:rerun-if-changed={}", head.display());
        }
        if let Ok(symbolic) = run_git(&["symbolic-ref", "-q", "HEAD"]) {
            let reference = git_dir.join(symbolic.trim());
            if reference.exists() {
                println!("cargo:rerun-if-changed={}", reference.display());
            }
        }
    }
}

/// Return the first four bytes of the HEAD commit SHA-1 as a big-endian `u32`.
fn git_head_identity() -> Option<u32> {
    let head = run_git(&["rev-parse", "HEAD"]).ok()?;
    let prefix = head.trim().get(0..8)?;
    u32::from_str_radix(prefix, 16).ok()
}

/// Run `git` in the crate directory, returning stdout on success.
fn run_git(args: &[&str]) -> std::io::Result<String> {
    let manifest_dir = std::env::var("CARGO_MANIFEST_DIR")
        .map_err(|_| std::io::Error::other("CARGO_MANIFEST_DIR unset"))?;
    let output = Command::new("git")
        .current_dir(manifest_dir)
        .args(args)
        .output()?;
    if !output.status.success() {
        return Err(std::io::Error::other("git command failed"));
    }
    Ok(String::from_utf8_lossy(&output.stdout).into_owned())
}
