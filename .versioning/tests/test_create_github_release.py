"""Unit tests for GitHub Release note composition (changelog + hybrid-devel PRs)."""

import sys
from pathlib import Path

import pytest

VERSIONING_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(VERSIONING_DIR / "scripts"))

from create_github_release import (  # noqa: E402
    compose_release_body,
    extract_tip_released_section,
    format_merged_prs,
    format_new_contributors,
    full_changelog_url,
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

DEVEL_PRS = [
    {
        "number": 57,
        "title": "fix: push sim time into ROS clocks without /clock yield-wait",
        "author": "DavidPL1",
        "mergedAt": "2026-10-29T15:35:38Z",
    },
    {
        "number": 58,
        "title": "feat: gate mujoco_ros_control Jazzy APIs with Humble backwards compat",
        "author": "DavidPL1",
        "mergedAt": "2026-10-30T09:42:27Z",
    },
    {
        "number": 59,
        "title": "Bump MuJoCo from 3.3.5 to 3.17.0",
        "author": "DavidPL1",
        "mergedAt": "2026-11-05T12:16:20Z",
    },
]


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


def test_format_merged_prs_lists_devel_prs_only():
    merged = format_merged_prs(DEVEL_PRS)

    assert merged == (
        "## Merged PRs\n"
        "- fix: push sim time into ROS clocks without /clock yield-wait by @DavidPL1 in #57\n"
        "- feat: gate mujoco_ros_control Jazzy APIs with Humble backwards compat by @DavidPL1 in #58\n"
        "- Bump MuJoCo from 3.3.5 to 3.17.0 by @DavidPL1 in #59"
    )
    assert "#56" not in merged
    assert "#60" not in merged


def test_format_merged_prs_omits_section_when_empty():
    assert format_merged_prs([]) is None


def test_format_new_contributors_omits_when_empty():
    assert format_new_contributors([]) is None


def test_format_new_contributors_lists_first_prs():
    text = format_new_contributors([("sugikazu75", 51)])

    assert text == "## New Contributors\n- @sugikazu75 made their first contribution in #51"


def test_full_changelog_url_uses_tag_compare():
    assert (
        full_changelog_url("ubi-agni/mujoco_ros_pkgs", "v2.0.0", "v2.1.0")
        == "https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0"
    )


def test_compose_release_body_orders_merged_changelog_contributors_full():
    version, changelog = extract_tip_released_section(CHANGELOG_TWO_RELEASES)
    merged = format_merged_prs(DEVEL_PRS)
    new_contrib = format_new_contributors([("sugikazu75", 51)])
    full = full_changelog_url("ubi-agni/mujoco_ros_pkgs", "v2.0.0", "v2.1.0")

    body = compose_release_body(changelog, merged, new_contrib, full)

    assert version == "2.1.0"
    assert body.index("## Merged PRs") < body.index("### Changed")
    assert body.index("### Changed") < body.index("## New Contributors")
    assert body.index("## New Contributors") < body.index("**Full Changelog**")
    assert "hybrid-main" not in body
    assert "#60" not in body


def test_compose_release_body_skips_optional_sections_when_none():
    _, changelog = extract_tip_released_section(CHANGELOG_TWO_RELEASES)
    full = full_changelog_url("ubi-agni/mujoco_ros_pkgs", "v2.0.0", "v2.1.0")

    body = compose_release_body(changelog, None, None, full)

    assert "## Merged PRs" not in body
    assert "## New Contributors" not in body
    assert body.rstrip().endswith(
        "**Full Changelog**: https://github.com/ubi-agni/mujoco_ros_pkgs/compare/v2.0.0...v2.1.0"
    )
