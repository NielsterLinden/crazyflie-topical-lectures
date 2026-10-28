#!/usr/bin/env python3
"""Check the interpreter used to create the native lecture environment."""

from __future__ import annotations

import sys


def check_version(version: tuple[int, int]) -> str | None:
    if version < (3, 12):
        raise ValueError("Python 3.12 or newer is needed; 3.12 is the tested default.")
    if version > (3, 12):
        return (
            f"WARNING: Python {version[0]}.{version[1]} is untested with the pinned "
            "packages, Qt GUI, and USB access. Python 3.12 is the tested default; "
            "installation or runtime compatibility is not guaranteed."
        )
    return None


if __name__ == "__main__":
    try:
        warning = check_version(sys.version_info[:2])
    except ValueError as error:
        print(error, file=sys.stderr)
        sys.exit(1)
    if warning:
        print(warning, file=sys.stderr)
