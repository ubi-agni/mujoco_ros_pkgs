import re

import pytest

from helpers import (
    CHANGELOG_TEMPLATE,
    DECOY_VERSION,
    PATCH_ENTRY,
    UNVERSIONED_PACKAGES,
    VERSION,
    VERSIONED_PACKAGES,
    VERSIONING_DIR,
    feature_commit,
    git,
    make_repo,
    write,
)
from bump_versions import (
    assert_versions_match,
    bump_workspace,
    ensure_version_tag,
    infer_next_version,
)
from versioning_common import VersioningError

ALL_PACKAGES = VERSIONED_PACKAGES + UNVERSIONED_PACKAGES


def pending(repo, unreleased):
    feature_commit(repo, {}, CHANGELOG_TEMPLATE.format(unreleased=unreleased))


def package_version(repo, pkg):
    return re.search(
        r"<version>([^<]+)</version>", (repo / pkg / "package.xml").read_text()
    ).group(1)


def cmake_version(repo, pkg):
    return re.search(
        r"project\(\w+ VERSION (\S+) ", (repo / pkg / "CMakeLists.txt").read_text()
    ).group(1)


def test_patch_entry_bumps_every_package_xml(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)

    assert bump_workspace(repo, skip_bump_marker=False) == "2.0.1"
    assert {package_version(repo, pkg) for pkg in ALL_PACKAGES} == {"2.0.1"}


def test_bump_updates_only_versioned_cmake_packages(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)

    bump_workspace(repo, skip_bump_marker=False)

    for pkg in VERSIONED_PACKAGES:
        assert cmake_version(repo, pkg) == "2.0.1"
    for pkg in UNVERSIONED_PACKAGES:
        assert cmake_version(repo, pkg) == DECOY_VERSION


@pytest.mark.parametrize(("impact", "expected"), [("minor", "2.1.0"), ("major", "3.0.0")])
def test_bump_applies_impact_level(tmp_path, impact, expected):
    repo = make_repo(tmp_path)
    pending(repo, f"\n### Added\n* [{impact}] Add thing.\n")

    assert bump_workspace(repo, skip_bump_marker=False) == expected


def test_bump_finalizes_unreleased_entries_under_new_version(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)

    bump_workspace(repo, skip_bump_marker=False)

    text = (repo / "CHANGELOG.md").read_text()
    assert re.search(r"## \[2\.0\.1\] - \d{4}-\d{2}-\d{2}", text)
    assert (
        text.index("## [2.0.1]") < text.index("* [patch] Fix plugin.") < text.index("## [2.0.0]")
    )


def test_bump_commits_as_version_bot(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)

    bump_workspace(repo, skip_bump_marker=False)

    identity = git(repo, "log", "-1", "--format=%an|%ae|%cn|%ce").strip()
    assert identity == "Version Bot|version-bot@ci|Version Bot|version-bot@ci"


def test_chore_only_pending_does_not_bump(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, "\n### Changed\n* [chore] Tidy docs.\n")
    head = git(repo, "rev-parse", "HEAD")

    assert bump_workspace(repo, skip_bump_marker=False) is None
    assert git(repo, "rev-parse", "HEAD") == head
    assert package_version(repo, "mujoco_ros") == VERSION


def test_skip_bump_marker_is_noop(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)
    head = git(repo, "rev-parse", "HEAD")

    assert bump_workspace(repo, skip_bump_marker=True) is None
    assert git(repo, "rev-parse", "HEAD") == head
    assert package_version(repo, "mujoco_ros") == VERSION


def test_infer_reports_pending_impact(tmp_path):
    repo = make_repo(tmp_path)
    assert infer_next_version(repo) == (None, "none")

    pending(repo, PATCH_ENTRY)
    assert infer_next_version(repo) == ("2.0.1", "patch")


def test_package_xml_drift_fails_bump(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)
    write(
        repo,
        "mujoco_ros_msgs/package.xml",
        (repo / "mujoco_ros_msgs/package.xml").read_text().replace(VERSION, "2.0.5"),
    )

    with pytest.raises(VersioningError, match="version drift"):
        bump_workspace(repo, skip_bump_marker=False)


def test_cmake_drift_fails_version_match(tmp_path):
    repo = make_repo(tmp_path)
    write(repo, "mujoco_ros/CMakeLists.txt", "project(mujoco_ros VERSION 1.9.9 LANGUAGES CXX)\n")

    with pytest.raises(VersioningError, match="version drift"):
        assert_versions_match(repo)


def test_match_fails_while_bump_is_pending(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)

    with pytest.raises(VersioningError, match="2.0.1"):
        assert_versions_match(repo)


def test_match_passes_after_bump(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, PATCH_ENTRY)
    bump_workspace(repo, skip_bump_marker=False)

    assert_versions_match(repo)


def test_match_rejects_chore_only_as_non_release(tmp_path):
    repo = make_repo(tmp_path)
    pending(repo, "\n### Changed\n* [chore] Tidy docs.\n")

    with pytest.raises(VersioningError, match="non-release"):
        assert_versions_match(repo)


def test_tag_is_annotated_v_version(tmp_path):
    repo = make_repo(tmp_path)

    assert ensure_version_tag(repo) == f"v{VERSION}"
    assert git(repo, "cat-file", "-t", f"v{VERSION}").strip() == "tag"


def test_existing_tag_is_noop(tmp_path):
    repo = make_repo(tmp_path)
    ensure_version_tag(repo)

    assert ensure_version_tag(repo) is None


def test_dry_run_tag_writes_nothing(tmp_path):
    repo = make_repo(tmp_path)

    assert ensure_version_tag(repo, dry_run=True) == f"v{VERSION}"
    assert git(repo, "tag", "--list").strip() == ""
    assert git(repo, "status", "--porcelain").strip() == ""


def test_non_uniform_mode_is_rejected(tmp_path):
    repo = make_repo(tmp_path)
    config = (repo / ".versioning/config.yaml").read_text()
    write(repo, ".versioning/config.yaml", config.replace("mode: uniform", "mode: single-package"))

    with pytest.raises(VersioningError, match="uniform"):
        ensure_version_tag(repo)


FORBIDDEN_IDENTIFIER = re.compile(r"neura|sdd-version|neura_spec", re.IGNORECASE)


def test_shipped_versioning_sources_are_neutral():
    shipped = [VERSIONING_DIR / "config.yaml", *sorted((VERSIONING_DIR / "scripts").glob("*.py"))]
    offenders = [
        str(p) for p in shipped if FORBIDDEN_IDENTIFIER.search(p.read_text(encoding="utf-8"))
    ]

    assert offenders == []
