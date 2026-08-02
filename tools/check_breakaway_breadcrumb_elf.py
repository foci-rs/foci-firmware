#!/usr/bin/env python3
"""Verify that linked firmware retains the breakaway breadcrumb in CCMRAM."""

from __future__ import annotations

import argparse
import re
import shutil
import subprocess
import sys
from pathlib import Path


SYMBOL = "BREAKAWAY_BREADCRUMB"


def symbol_block(elf: Path, readobj: str) -> str | None:
    """Return the retained breadcrumb symbol block from an ELF symbol table."""
    result = subprocess.run(
        [readobj, "--symbols", str(elf)],
        check=True,
        capture_output=True,
        text=True,
    )
    for block in result.stdout.split("  Symbol {")[1:]:
        block = block.split("\n  }", 1)[0]
        name = re.search(r"^\s*Name: (.+)$", block, re.MULTILINE)
        if (
            name is not None
            and SYMBOL in name.group(1)
            and "LAST_BOOT" not in name.group(1)
        ):
            return block
    return None


def check_elf(elf: Path, readobj: str) -> str | None:
    """Return an actionable error when the retained symbol contract is absent."""
    if not elf.is_file():
        return f"{elf}: ELF does not exist"
    block = symbol_block(elf, readobj)
    if block is None:
        return f"{elf}: missing retained {SYMBOL} object"
    requirements = {
        "four-byte allocation": r"^\s*Size: 4$",
        "global binding": r"^\s*Binding: Global ",
        "object type": r"^\s*Type: Object ",
        ".ccmram placement": r"^\s*Section: \.ccmram ",
    }
    missing = [
        description
        for description, pattern in requirements.items()
        if re.search(pattern, block, re.MULTILINE) is None
    ]
    if missing:
        return f"{elf}: {SYMBOL} lacks {', '.join(missing)}"
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "elf", nargs="+", type=Path, help="linked firmware ELF to inspect"
    )
    args = parser.parse_args()

    readobj = shutil.which("rust-readobj")
    if readobj is None:
        parser.error(
            "rust-readobj is required; install cargo-binutils and llvm-tools-preview"
        )

    errors = [
        error for elf in args.elf if (error := check_elf(elf, readobj)) is not None
    ]
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    for elf in args.elf:
        print(f"{elf}: retained {SYMBOL} object in .ccmram")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
