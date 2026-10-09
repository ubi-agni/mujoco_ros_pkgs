import pytest

from helpers import (
    BASE_BRANCH,
    CHANGELOG_TEMPLATE,
    PATCH_ENTRY,
    SHIPPED_EDIT,
    feature_commit,
    make_repo,
)
from bump_versions import bump_workspace
from changelog_gate import run_changelog_gate
from versioning_common import VersioningError

CHORE_ENTRY = "\n### Changed\n* [chore] Tidy docs.\n"


def test_valid_unreleased_entry_passes(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY))

    run_changelog_gate(repo, BASE_BRANCH)


def test_shipped_change_without_entry_fails(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=""))

    with pytest.raises(VersioningError, match="no non-chore"):
        run_changelog_gate(repo, BASE_BRANCH)


def test_chore_entry_does_not_satisfy_shipped_change(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=CHORE_ENTRY))

    with pytest.raises(VersioningError, match="no non-chore"):
        run_changelog_gate(repo, BASE_BRANCH)


def test_entry_without_shipped_change_fails(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(
        repo, {"docs/notes.md": "notes\n"}, CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY)
    )

    with pytest.raises(VersioningError, match="no shipped file"):
        run_changelog_gate(repo, BASE_BRANCH)


def test_malformed_entry_fails(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(
        repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased="\n* Fix plugin without a tag.\n")
    )

    with pytest.raises(VersioningError, match="malformed"):
        run_changelog_gate(repo, BASE_BRANCH)


def test_edit_to_released_section_fails(tmp_path):
    repo = make_repo(tmp_path)
    released_edit = CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY).replace(
        "Initial release.", "Rewritten history."
    )
    feature_commit(repo, SHIPPED_EDIT, released_edit)

    with pytest.raises(VersioningError, match="released section was mutated"):
        run_changelog_gate(repo, BASE_BRANCH)


def test_bump_commit_at_head_gates_the_pre_bump_tree(tmp_path):
    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY))
    assert bump_workspace(repo, skip_bump_marker=False) == "2.0.1"

    run_changelog_gate(repo, BASE_BRANCH)


def test_human_commit_after_bump_still_passes(tmp_path):
    """Reviewer fix on top of Version Bot must not brick the gate."""
    from helpers import commit_all, write

    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY))
    assert bump_workspace(repo, skip_bump_marker=False) == "2.0.1"
    write(repo, "docs/notes.md", "reviewer nits\n")
    commit_all(repo, "docs: reviewer nits")

    run_changelog_gate(repo, BASE_BRANCH)


def test_shipped_edit_after_bump_still_needs_unreleased(tmp_path):
    from helpers import commit_all, write

    repo = make_repo(tmp_path)
    feature_commit(repo, SHIPPED_EDIT, CHANGELOG_TEMPLATE.format(unreleased=PATCH_ENTRY))
    assert bump_workspace(repo, skip_bump_marker=False) == "2.0.1"
    write(repo, "mujoco_ros/src/plugin.cpp", "// v3 after bump\n")
    commit_all(repo, "fix: more shipped work")

    with pytest.raises(VersioningError, match="no non-chore"):
        run_changelog_gate(repo, BASE_BRANCH)
