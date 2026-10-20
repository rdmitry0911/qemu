#!/usr/bin/env python3
"""Reject mutable QEMU/QMetal build-pair selection for Apple GFX ML.

This is intentionally a source-only verifier.  It establishes neither a
compiled binary nor a guest, packet, rendering, or visual result.
"""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def require(text: str, needle: str, label: str) -> None:
    if needle not in text:
        raise SystemExit(f"missing {label}: {needle!r}")


def reject(text: str, needle: str, label: str) -> None:
    if needle in text:
        raise SystemExit(f"forbidden {label}: {needle!r}")


def main() -> None:
    options = (ROOT / "meson_options.txt").read_text()
    buildoptions = (ROOT / "scripts/meson-buildoptions.py").read_text()
    meson = (ROOT / "hw/display/meson.build").read_text()

    for option in (
        "apple_virgl_qmetal_source_dir",
        "apple_virgl_qmetal_build_dir",
    ):
        require(options, f"option('{option}'", "QMetal selection option")
        require(buildoptions, f'"{option}"', "configure option exclusion")
        require(meson, f"get_option('{option}')", "explicit QMetal selection")

    for needle, label in (
        ("if config_all_devices.has_key('CONFIG_APPLE_GFX_ML')", "device gate"),
        ("qmetal_source_dir.startswith('/')", "absolute source guard"),
        ("qmetal_build_dir.startswith('/')", "absolute build guard"),
        ("include/qmu/qmetal_unified.h", "unified header guard"),
        ("include/pvg/pvg_fifo.h", "PVG header guard"),
        ("libmlapi.so", "library guard"),
        ("link_args: [mlapi_lib_so, '-Wl,-rpath,' + qmetal_build_dir]", "exact library link"),
    ):
        require(meson, needle, label)

    for needle, label in (
        ("dependency('mlapi'", "system mlapi discovery"),
        ("../../../qmetal", "source-relative QMetal fallback"),
        ("cc.find_library(", "name-based mlapi lookup"),
    ):
        reject(meson, needle, label)

    print(json.dumps({
        "schema": "apple-gfx-qmetal-build-authority-v1",
        "explicit_options": [
            "apple_virgl_qmetal_source_dir",
            "apple_virgl_qmetal_build_dir",
        ],
        "system_or_relative_fallback": False,
        "runtime_admission_forbidden": True,
    }, sort_keys=True))


if __name__ == "__main__":
    main()
