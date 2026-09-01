#!/usr/bin/env python3
"""Patch a cloned ros-conan tree so vcstool retries flaky GitHub clones.

Usage: python3 scripts/patch_ros_kilted_vcs.py <ros-conan-root>
"""
from pathlib import Path
import sys

OLD = "import --input"
NEW = "import --retry 5 --workers 5 --input"


def main() -> int:
    if len(sys.argv) != 2:
        print("usage: patch_ros_kilted_vcs.py <ros-conan-root>", file=sys.stderr)
        return 2
    path = Path(sys.argv[1], "recipes", "ros-kilted", "all", "conanfile.py")
    text = path.read_text(encoding="utf-8")
    if OLD not in text:
        print(f"vcs {OLD} not found in {path}", file=sys.stderr)
        return 1
    if NEW in text:
        return 0
    path.write_text(text.replace(OLD, NEW, 1), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
