#!/usr/bin/env python3
"""Uniform version bump, version-match check, and release tag for mujoco_ros_pkgs.

Commands:
  bump         Bump every package.xml and versioned CMakeLists.txt from pending entries.
  match        Fail unless tip versions equal the version the CHANGELOG implies.
  tag [--dry-run]  Create the annotated release tag for the tip version if missing.

Usage: bump_versions.py [--repo PATH] {bump,match,tag}
"""

import argparse
import os
import re
import sys
from datetime import date
from pathlib import Path

from versioning_common import (
    CHANGELOG_NAME,
    UNRELEASED_HEADING_RE,
    VersioningError,
    bot_env,
    bump_version,
    git,
    load_config,
    pending_impact,
    released_version,
    split_changelog,
)

BOT_NAME = "Version Bot"
BOT_EMAIL = "version-bot@ci"

# Package-root CMakeLists.txt files that declare a project VERSION. Other CMake files are left alone.
VERSIONED_CMAKE_PACKAGES = (
    "mujoco_ros",
    "mujoco_ros_control",
    "mujoco_ros_laser",
    "mujoco_ros_mocap",
    "mujoco_ros_sensors",
    "mujoco_ros_testing_utils",
)
SKIP_DIRS = {"build", "install", "log", ".git", "__pycache__", ".cache"}
XML_VERSION_RE = re.compile(r"<version>([^<]+)</version>")
CMAKE_VERSION_RE = re.compile(r"(project\(\s*\w+\s+VERSION\s+)(\d+\.\d+\.\d+)")


def _package_xmls(repo):
    found = []
    for dirpath, dirnames, filenames in os.walk(repo):
        dirnames[:] = [name for name in dirnames if name not in SKIP_DIRS]
        if "package.xml" in filenames:
            found.append(Path(dirpath) / "package.xml")
    return sorted(found)


def _cmake_path(repo, pkg):
    return Path(repo) / pkg / "CMakeLists.txt"


def _read_xml_version(path):
    match = XML_VERSION_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        raise VersioningError(f"{path}: no <version> element")
    return match.group(1).strip()


def _set_xml_version(path, version):
    text, count = XML_VERSION_RE.subn(
        f"<version>{version}</version>", path.read_text(encoding="utf-8"), count=1
    )
    if count != 1:
        raise VersioningError(f"{path}: no <version> element")
    path.write_text(text, encoding="utf-8")


def _read_cmake_version(path):
    match = CMAKE_VERSION_RE.search(path.read_text(encoding="utf-8"))
    if match is None:
        raise VersioningError(f"{path}: no project(... VERSION x.y.z) declaration")
    return match.group(2)


def _set_cmake_version(path, version):
    text, count = CMAKE_VERSION_RE.subn(
        lambda match: match.group(1) + version,
        path.read_text(encoding="utf-8"),
        count=1,
    )
    if count != 1:
        raise VersioningError(f"{path}: no project(... VERSION x.y.z) declaration")
    path.write_text(text, encoding="utf-8")


def read_tip_version(repo):
    """Return the single version shared by every package.xml and versioned CMakeLists.txt."""
    load_config(repo)
    sources = {
        str(path.relative_to(repo)): _read_xml_version(path) for path in _package_xmls(repo)
    }
    for pkg in VERSIONED_CMAKE_PACKAGES:
        path = _cmake_path(repo, pkg)
        sources[str(path.relative_to(repo))] = _read_cmake_version(path)
    versions = set(sources.values())
    if len(versions) != 1:
        listing = ", ".join(f"{name}={version}" for name, version in sorted(sources.items()))
        raise VersioningError(f"version drift across package files: {listing}")
    return versions.pop()


def _changelog_text(repo):
    return (Path(repo) / CHANGELOG_NAME).read_text(encoding="utf-8")


def infer_next_version(repo_root: Path) -> tuple[str | None, str]:
    """Return (next version or None, pending impact of the Unreleased section)."""
    impact = pending_impact(split_changelog(_changelog_text(repo_root))[0])
    if impact in ("major", "minor", "patch"):
        return bump_version(read_tip_version(repo_root), impact), impact
    return None, impact


def _finalize_changelog(repo, version, today):
    path = Path(repo) / CHANGELOG_NAME
    content = path.read_text(encoding="utf-8")
    heading = UNRELEASED_HEADING_RE.search(content)
    if heading is None:
        raise VersioningError(f"{CHANGELOG_NAME} has no Unreleased heading")
    release = f'\n\n<a name="{version}"></a>\n## [{version}] - {today}'
    end = heading.end()
    path.write_text(content[:end] + release + content[end:], encoding="utf-8")


def bump_workspace(repo_root: Path, *, skip_bump_marker: bool) -> str | None:
    """Bump all versions from pending entries and commit as Version Bot.

    Returns the new version, or None when nothing is released or the bump is skipped.
    """
    if skip_bump_marker:
        return None
    next_version, _ = infer_next_version(repo_root)
    if next_version is None:
        return None

    _finalize_changelog(repo_root, next_version, date.today().isoformat())
    xml_paths = _package_xmls(repo_root)
    for path in xml_paths:
        _set_xml_version(path, next_version)
    cmake_paths = [_cmake_path(repo_root, pkg) for pkg in VERSIONED_CMAKE_PACKAGES]
    for path in cmake_paths:
        _set_cmake_version(path, next_version)

    touched = [str(p.relative_to(repo_root)) for p in [*xml_paths, *cmake_paths]]
    touched.append(CHANGELOG_NAME)
    git(repo_root, "add", "--", *touched)
    git(
        repo_root,
        "commit",
        "-q",
        "-m",
        f"chore(release): bump version to {next_version}",
        env=bot_env(BOT_NAME, BOT_EMAIL),
    )
    return next_version


def assert_versions_match(repo_root: Path) -> None:
    """Fail unless tip versions equal the version the CHANGELOG implies."""
    tip = read_tip_version(repo_root)
    next_version, impact = infer_next_version(repo_root)
    if impact == "chore":
        raise VersioningError(
            "non-release: Unreleased holds only [chore] entries, so there is no version to release"
        )
    if next_version is None:
        expected = released_version(split_changelog(_changelog_text(repo_root))[1])
    else:
        expected = next_version
    if tip != expected:
        raise VersioningError(
            f"tip version {tip} does not match expected {expected} ({impact} pending); "
            "run the version bump first"
        )


def ensure_version_tag(repo_root: Path, *, dry_run: bool = False) -> str | None:
    """Create annotated tag `<tag_prefix><version>` if missing. Returns it, or None if it exists."""
    tag = f"{load_config(repo_root)['tag_prefix']}{read_tip_version(repo_root)}"
    if git(repo_root, "tag", "--list", tag).strip():
        return None
    if not dry_run:
        git(
            repo_root,
            "tag",
            "-a",
            tag,
            "-m",
            f"Version {tag}",
            env=bot_env(BOT_NAME, BOT_EMAIL),
        )
    return tag


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=Path("."))
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("bump")
    sub.add_parser("match")
    tag_parser = sub.add_parser("tag")
    tag_parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    repo = args.repo.resolve()
    try:
        if args.command == "bump":
            head_message = git(repo, "log", "-1", "--format=%B")
            version = bump_workspace(repo, skip_bump_marker="[skip-bump]" in head_message)
            print(version or "no bump")
        elif args.command == "match":
            assert_versions_match(repo)
            print("VERSION MATCH: PASS")
        else:
            tag = ensure_version_tag(repo, dry_run=args.dry_run)
            print(tag or "tag exists")
    except VersioningError as error:
        print(f"VERSIONING: FAIL\n{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
