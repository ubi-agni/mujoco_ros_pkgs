import os
import subprocess
import sys

import pytest

from helpers import VERSIONING_DIR
from bump_versions import BOT_EMAIL, BOT_NAME
from ci_push_skip import is_version_bot_author, should_skip_full_ci_push

SCRIPT = VERSIONING_DIR / "scripts" / "ci_push_skip.py"


def test_bot_identity_is_recognised():
    assert is_version_bot_author(BOT_NAME, BOT_EMAIL) is True


@pytest.mark.parametrize(
    ("name", "email"),
    [
        ("David Leins", "david@example.com"),
        (BOT_NAME, "someone@example.com"),
        ("Someone", BOT_EMAIL),
        ("version bot", BOT_EMAIL),
    ],
)
def test_non_bot_identity_is_rejected(name, email):
    assert is_version_bot_author(name, email) is False


def test_bot_author_skips_even_without_merged_pr():
    assert (
        should_skip_full_ci_push(
            author_name=BOT_NAME, author_email=BOT_EMAIL, associated_merged_pr_bases=[]
        )
        is True
    )


def test_human_push_without_merged_pr_runs():
    assert (
        should_skip_full_ci_push(
            author_name="David Leins",
            author_email="david@example.com",
            associated_merged_pr_bases=[],
        )
        is False
    )


def test_human_commit_of_merged_devel_pr_skips():
    assert (
        should_skip_full_ci_push(
            author_name="David Leins",
            author_email="david@example.com",
            associated_merged_pr_bases=["hybrid-devel"],
        )
        is True
    )


def test_merged_pr_into_main_does_not_skip():
    assert (
        should_skip_full_ci_push(
            author_name="David Leins",
            author_email="david@example.com",
            associated_merged_pr_bases=["hybrid-main"],
        )
        is False
    )


def test_any_devel_base_in_mixed_list_skips():
    assert (
        should_skip_full_ci_push(
            author_name="David Leins",
            author_email="david@example.com",
            associated_merged_pr_bases=["hybrid-main", "hybrid-devel"],
        )
        is True
    )


def test_custom_devel_branch_is_honoured():
    assert (
        should_skip_full_ci_push(
            author_name="David Leins",
            author_email="david@example.com",
            associated_merged_pr_bases=["next"],
            devel_branch="next",
        )
        is True
    )


def test_empty_author_fails_loud():
    with pytest.raises(ValueError):
        should_skip_full_ci_push(
            author_name="", author_email="david@example.com", associated_merged_pr_bases=[]
        )


def _run_cli(args, env_extra=None):
    env = dict(os.environ)
    env.pop("GITHUB_OUTPUT", None)
    env.update(env_extra or {})
    return subprocess.run(
        [sys.executable, str(SCRIPT), *args],
        capture_output=True,
        text=True,
        env=env,
    )


def test_cli_prints_skip_true_for_bot(tmp_path):
    result = _run_cli(["--author-name", BOT_NAME, "--author-email", BOT_EMAIL])

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "skip=true"


def test_cli_prints_skip_false_for_human_push(tmp_path):
    result = _run_cli(["--author-name", "David Leins", "--author-email", "david@example.com"])

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "skip=false"


def test_cli_passes_merged_pr_bases(tmp_path):
    result = _run_cli(
        [
            "--author-name",
            "David Leins",
            "--author-email",
            "david@example.com",
            "--pr-base",
            "hybrid-devel",
        ]
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "skip=true"


def test_cli_appends_to_github_output(tmp_path):
    output = tmp_path / "github_output"
    output.write_text("existing=1\n", encoding="utf-8")

    result = _run_cli(
        ["--author-name", BOT_NAME, "--author-email", BOT_EMAIL],
        env_extra={"GITHUB_OUTPUT": str(output)},
    )

    assert result.returncode == 0, result.stderr
    assert output.read_text(encoding="utf-8") == "existing=1\nskip=true\n"


def test_cli_fails_on_empty_author(tmp_path):
    result = _run_cli(["--author-name", "", "--author-email", "david@example.com"])

    assert result.returncode != 0
    assert "author" in result.stderr.lower()
