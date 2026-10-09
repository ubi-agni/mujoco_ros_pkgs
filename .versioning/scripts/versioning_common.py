"""Shared CHANGELOG, config, git, and bump helpers for the release versioning scripts."""

import os
import re
import subprocess
from pathlib import Path

import yaml

CONFIG_RELPATH = Path(".versioning") / "config.yaml"
CHANGELOG_NAME = "CHANGELOG.md"

# Each entry is a `*` or `-` bullet tagged with its impact, e.g. `* [patch] Fix x.`
ENTRY_RE = re.compile(r"^[-*]\s+\[(major|minor|patch|chore)\]\s+\S")
UNRELEASED_HEADING_RE = re.compile(r"^## \[?Unreleased\]?[ \t]*$", re.IGNORECASE | re.MULTILINE)
RELEASED_HEADING_RE = re.compile(r"^## \[(\d+\.\d+\.\d+)\]", re.MULTILINE)
# A release section starts at its heading or its `<a name=...>` anchor, whichever comes first.
SECTION_START_RE = re.compile(r"^(?:## |<a name=)", re.MULTILINE)

# Changes to these paths do not ship code and need no changelog entry.
NON_SHIPPED_NAMES = ("CHANGELOG.md", "README.md", "CONTRIBUTING.md", "AGENTS.md", "LICENSE")
NON_SHIPPED_PREFIXES = (
    "docs/",
    "specs/",
    ".versioning/",
    ".github/",
    ".scratch/",
    ".superpowers/",
    ".housekeeping/",
    "build/",
    "install/",
    "log/",
)


class VersioningError(RuntimeError):
    """A release precondition is not met. The message says what to fix."""


def git(repo, *args, env=None):
    result = subprocess.run(["git", *args], cwd=repo, capture_output=True, text=True, env=env)
    if result.returncode != 0:
        raise VersioningError(f"git {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout


def load_config(repo):
    cfg = yaml.safe_load((Path(repo) / CONFIG_RELPATH).read_text(encoding="utf-8")) or {}
    if cfg.get("mode") != "uniform":
        raise VersioningError(f"{CONFIG_RELPATH}: mode must be 'uniform', got {cfg.get('mode')!r}")
    return cfg


def bump_version(version, impact):
    major, minor, patch = (int(part) for part in version.split("."))
    if impact == "major":
        return f"{major + 1}.0.0"
    if impact == "minor":
        return f"{major}.{minor + 1}.0"
    if impact == "patch":
        return f"{major}.{minor}.{patch + 1}"
    raise VersioningError(f"no version bump for impact {impact!r}")


def split_changelog(content):
    """Return (unreleased_body, released_tail) of a CHANGELOG.md text.

    The released tail is everything from the first section boundary after the Unreleased
    heading to the end of the file. Raises if the Unreleased heading is missing.
    """
    heading = UNRELEASED_HEADING_RE.search(content)
    if heading is None:
        raise VersioningError(f"{CHANGELOG_NAME} has no Unreleased heading")
    start = heading.end()
    boundary = SECTION_START_RE.search(content, start)
    if boundary is None:
        return content[start:], ""
    cut = boundary.start()
    return content[start:cut], content[cut:]


def pending_impact(unreleased_body):
    """Highest impact of Unreleased entries: major, minor, patch, chore, or none."""
    impacts = set()
    for line in unreleased_body.splitlines():
        match = ENTRY_RE.match(line)
        if match:
            impacts.add(match.group(1))
    for level in ("major", "minor", "patch"):
        if level in impacts:
            return level
    return "chore" if impacts else "none"


def released_version(released_tail):
    match = RELEASED_HEADING_RE.search(released_tail)
    if match is None:
        raise VersioningError(f"{CHANGELOG_NAME} has no released version section")
    return match.group(1)


def is_shipped_file(path):
    if Path(path).name in NON_SHIPPED_NAMES:
        return False
    return not path.startswith(NON_SHIPPED_PREFIXES)


def bot_env(bot_name, bot_email):
    """Environment that makes git author and committer the given bot identity."""
    env = dict(os.environ)
    env.update(
        GIT_AUTHOR_NAME=bot_name,
        GIT_AUTHOR_EMAIL=bot_email,
        GIT_COMMITTER_NAME=bot_name,
        GIT_COMMITTER_EMAIL=bot_email,
    )
    return env
