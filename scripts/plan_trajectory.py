#!/usr/bin/env python3
"""Repository-root entry point for the Day 3 offline course planner."""

from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "code"))

from day3.plan import main

if __name__ == "__main__":
    main()
