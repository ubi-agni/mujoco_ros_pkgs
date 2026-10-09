"""Throwaway git repos shaped like mujoco_ros_pkgs, shared by the versioning tests."""

import subprocess
import sys
from pathlib import Path

VERSIONING_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERSIONING_DIR / "scripts"))

BASE_BRANCH = "hybrid-main"
VERSION = "2.0.0"
VERSIONED_PACKAGES = (
    "mujoco_ros",
    "mujoco_ros_control",
    "mujoco_ros_laser",
    "mujoco_ros_mocap",
    "mujoco_ros_sensors",
    "mujoco_ros_testing_utils",
)
# These packages must keep their CMake VERSION line through a bump.
UNVERSIONED_PACKAGES = ("mujoco_ros_msgs", "mujoco_ros_pkgs")
DECOY_VERSION = "1.0.0"

PACKAGE_XML = """<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>{version}</version>
  <description>fixture</description>
  <maintainer email="developer@example.com">developer</maintainer>
  <license>BSD-3-Clause</license>
</package>
"""

CMAKE_LISTS = """cmake_minimum_required(VERSION 3.16)
project({name} VERSION {version} LANGUAGES CXX)
"""

CHANGELOG_TEMPLATE = """# Changelog

<a name="unreleased"></a>
## Unreleased
{unreleased}
<a name="2.0.0"></a>
## [2.0.0] - 2026-09-24

### Added
* [minor] Initial release.
"""

PATCH_ENTRY = "\n### Fixed\n* [patch] Fix plugin.\n"
SHIPPED_EDIT = {"mujoco_ros/src/plugin.cpp": "// v2\n"}


def git(repo, *args):
    return subprocess.run(
        ["git", *args], cwd=repo, check=True, capture_output=True, text=True
    ).stdout


def write(repo, rel, text):
    path = repo / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def commit_all(repo, message):
    git(repo, "add", "-A")
    git(repo, "commit", "-q", "-m", message)


def make_repo(root, *, unreleased=""):
    """Create `root/repo` on BASE_BRANCH with one commit of fixture packages."""
    repo = root / "repo"
    repo.mkdir()
    git(repo, "init", "-q", "-b", BASE_BRANCH)
    git(repo, "config", "user.name", "Developer")
    git(repo, "config", "user.email", "developer@example.com")
    write(repo, ".versioning/config.yaml", (VERSIONING_DIR / "config.yaml").read_text())
    for pkg in VERSIONED_PACKAGES + UNVERSIONED_PACKAGES:
        write(repo, f"{pkg}/package.xml", PACKAGE_XML.format(name=pkg, version=VERSION))
    for pkg in VERSIONED_PACKAGES:
        write(repo, f"{pkg}/CMakeLists.txt", CMAKE_LISTS.format(name=pkg, version=VERSION))
    for pkg in UNVERSIONED_PACKAGES:
        write(repo, f"{pkg}/CMakeLists.txt", CMAKE_LISTS.format(name=pkg, version=DECOY_VERSION))
    write(repo, "CHANGELOG.md", CHANGELOG_TEMPLATE.format(unreleased=unreleased))
    write(repo, "mujoco_ros/src/plugin.cpp", "// v1\n")
    commit_all(repo, "chore: base")
    return repo


def feature_commit(repo, files, changelog):
    """Commit `files` and `changelog` on a new `feature` branch off BASE_BRANCH."""
    git(repo, "checkout", "-q", "-b", "feature")
    for rel, text in files.items():
        write(repo, rel, text)
    write(repo, "CHANGELOG.md", changelog)
    commit_all(repo, "feat: plugin change")
