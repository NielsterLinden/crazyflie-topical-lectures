#!/usr/bin/env python3
"""Validate Topical Lecture imports and optional Crazyradio access."""

from __future__ import annotations

import argparse
import importlib
import importlib.metadata
import shutil
import subprocess
import sys


PACKAGES = {
    "cfclient": "cfclient",
    "cflib": "cflib",
    "numpy": "numpy",
    "scipy": "scipy",
    "matplotlib": "matplotlib",
    "control": "control",
    "jupyterlab": "jupyterlab",
}


def check_imports() -> bool:
    ok = True
    print(f"Python: {sys.version.split()[0]}")
    for distribution, module in PACKAGES.items():
        try:
            importlib.import_module(module)
            version = importlib.metadata.version(distribution)
            print(f"[OK] {distribution} {version}")
        except Exception as exc:
            ok = False
            print(f"[FAIL] {distribution}: {exc}")
    return ok


def check_radio() -> bool:
    try:
        from cflib.drivers.crazyradio import get_serials

        serials = get_serials()
    except Exception as exc:
        print(f"[FAIL] Crazyradio discovery: {exc}")
        return False
    if not serials:
        print("[FAIL] No Crazyradio detected")
        return False
    print(f"[OK] Crazyradio(s): {', '.join(serials)}")
    return True


def check_docker() -> bool:
    docker = shutil.which("docker")
    if docker is None:
        print("[FAIL] Docker CLI is not installed or not on PATH")
        return False

    commands = (
        ([docker, "--version"], "Docker"),
        ([docker, "compose", "version"], "Docker Compose"),
        (
            [docker, "info", "--format", "Docker server {{.ServerVersion}}"],
            "Docker daemon",
        ),
    )
    ok = True
    for command, name in commands:
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode == 0:
            print(f"[OK] {result.stdout.strip()}")
        else:
            ok = False
            detail = result.stderr.strip() or result.stdout.strip()
            print(f"[FAIL] {name}: {detail}")
    return ok


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--radio", action="store_true", help="also require a connected Crazyradio")
    parser.add_argument(
        "--docker",
        action="store_true",
        help="also require Docker, Compose, and a running Docker daemon",
    )
    args = parser.parse_args()

    ok = check_imports()
    if args.radio:
        ok = check_radio() and ok
    if args.docker:
        ok = check_docker() and ok
    print("Environment ready." if ok else "Environment check failed.")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
