"""Unit tests for GitHub Release note composition (changelog + generate-notes)."""

import sys
from pathlib import Path

import pytest

VERSIONING_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERSIONING_DIR / "scripts"))

from create_github_release import (  # noqa: E402
    compose_release_body,
    extract_tip_released_section,
    rewrite_generate_notes,
)

CHANGELOG_TWO_RELEASES = """# Changelog

<a name="unreleased"></a>
## Unreleased

<a name="2.1.0"></a>
## [2.1.0] - 2026-11-05

### Changed
* [minor] Pin MuJoCo 3.17.0.

### Fixed
* [patch] Fix equality accessors.

Contributors: @DavidPL1

<a name="2.0.0"></a>
## [2.0.0] - 2026-09-24

### Added
* [minor] Initial release.
"""

GENERATE_NOTES_WITH_CONTRIBUTORS = """## What's Changed
* feat: load descriptions by @Alice in https://github.com/ubi-agni/mujoco_ros_pkgs/pull/52
* fix: pedantic GCC9 by @sugikazu75 in https://github.com/ubi-agni/mujoco_ros_pkgs/pull/51

## New Contributors
* @sugikazu75 made their first contribution in https://github.com/ubi-agni/mujoco_ros_pkgs/pull/51

**Full Changelog**: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0
"""

GENERATE_NOTES_NO_CONTRIBUTORS = """## What's Changed
* ci: hybrid-main release flow by @DavidPL1 in https://github.com/ubi-agni/mujoco_ros_pkgs/pull/56

**Full Changelog**: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0
"""


def test_extract_tip_released_section_drops_heading_and_contributors():
    version, body = extract_tip_released_section(CHANGELOG_TWO_RELEASES)

    assert version == "2.1.0"
    assert body.startswith("### Changed")
    assert "Pin MuJoCo 3.17.0" in body
    assert "Contributors:" not in body
    assert "## [2.1.0]" not in body
    assert "2.0.0" not in body


def test_extract_tip_released_section_requires_a_released_heading():
    with pytest.raises(Exception, match="no released version"):
        extract_tip_released_section("# Changelog\n\n## Unreleased\n\n* [patch] Only pending.\n")


def test_rewrite_generate_notes_renames_and_shortens_pr_links():
    merged, new_contrib, full = rewrite_generate_notes(GENERATE_NOTES_WITH_CONTRIBUTORS)

    assert merged == (
        "## Merged PRs\n"
        "- feat: load descriptions by @Alice in #52\n"
        "- fix: pedantic GCC9 by @sugikazu75 in #51"
    )
    assert new_contrib == (
        "## New Contributors\n" "- @sugikazu75 made their first contribution in #51"
    )
    assert full == "https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0"


def test_rewrite_generate_notes_omits_new_contributors_when_absent():
    merged, new_contrib, full = rewrite_generate_notes(GENERATE_NOTES_NO_CONTRIBUTORS)

    assert merged.startswith("## Merged PRs\n- ci: hybrid-main release flow")
    assert new_contrib is None
    assert "v2.1.0" in full


def test_compose_release_body_orders_merged_changelog_contributors_full():
    version, changelog = extract_tip_released_section(CHANGELOG_TWO_RELEASES)
    merged, new_contrib, full = rewrite_generate_notes(GENERATE_NOTES_WITH_CONTRIBUTORS)

    body = compose_release_body(changelog, merged, new_contrib, full)

    assert version == "2.1.0"
    assert body.index("## Merged PRs") < body.index("### Changed")
    assert body.index("### Changed") < body.index("## New Contributors")
    assert body.index("## New Contributors") < body.index("**Full Changelog**")
    assert "What's Changed" not in body
    assert "https://github.com/" not in body.split("**Full Changelog**")[0]


def test_compose_release_body_skips_new_contributors_block_when_none():
    _, changelog = extract_tip_released_section(CHANGELOG_TWO_RELEASES)
    merged, new_contrib, full = rewrite_generate_notes(GENERATE_NOTES_NO_CONTRIBUTORS)

    body = compose_release_body(changelog, merged, new_contrib, full)

    assert "## New Contributors" not in body
    assert body.rstrip().endswith(
        "**Full Changelog**: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0"
    )
