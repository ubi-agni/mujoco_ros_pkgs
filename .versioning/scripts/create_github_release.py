#!/usr/bin/env python3
"""Compose and publish a GitHub Release for the tip version tag.

Body shape (agreed for hybrid-main tag runs):

  ## Merged PRs
  - <PR title> by @user in #N
  ...

  <tip CHANGELOG section body>

  ## New Contributors          # omitted when none
  - @user made their first contribution in #N
  ...

  **Full Changelog**: <compare URL>

Merged PRs are pull requests with base ``hybrid-devel`` merged after the
previous release tag and through the new tag time. GitHub generate-notes is
not used for that list: tags live on ``hybrid-main``, so generate-notes only
sees PRs into ``hybrid-main`` (release / cutover PRs).

Usage:
  create_github_release.py [--repo PATH] --tag vX.Y.Z [--dry-run]
  create_github_release.py [--repo PATH] --compose-only --tag vX.Y.Z
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from versioning_common import (
    CHANGELOG_NAME,
    RELEASED_HEADING_RE,
    SECTION_START_RE,
    VersioningError,
    load_config,
    split_changelog,
)

DEVEL_BASE = "hybrid-devel"
CONTRIBUTORS_FOOTER_RE = re.compile(r"(?:\n---)?\nContributors:[^\n]*(?:\n|$)", re.IGNORECASE)


def extract_tip_released_section(changelog_text: str) -> tuple[str, str]:
    """Return (version, section body) for the newest released CHANGELOG section.

    Drops the `## [X.Y.Z] - date` heading and a trailing `Contributors:` line.
    """
    _, released_tail = split_changelog(changelog_text)
    match = RELEASED_HEADING_RE.search(released_tail)
    if match is None:
        raise VersioningError(f"{CHANGELOG_NAME} has no released version section")
    version = match.group(1)
    heading_end = match.end()
    after_heading = released_tail[heading_end:]
    if after_heading.startswith("\n"):
        after_heading = after_heading[1:]
    # Drop an optional date remainder on the same line as the heading match end.
    # RELEASED_HEADING_RE only matches through the version bracket; consume rest of line.
    if after_heading and not after_heading.startswith("#") and not after_heading.startswith("<"):
        # Heading was `## [ver] - date\n...` — match.end() is after `]`, so strip ` - date`.
        nl = after_heading.find("\n")
        if nl == -1:
            after_heading = ""
        else:
            body_start = nl + 1
            after_heading = after_heading[body_start:]

    next_section = SECTION_START_RE.search(after_heading)
    if next_section is None:
        body = after_heading
    else:
        section_start = next_section.start()
        body = after_heading[:section_start]
    body = CONTRIBUTORS_FOOTER_RE.sub("\n", body).strip()
    if not body:
        raise VersioningError(f"{CHANGELOG_NAME} tip released section {version} is empty")
    return version, body


def format_merged_prs(prs: list[dict]) -> str | None:
    """Format Merged PRs section; ``prs`` oldest-first. None when empty."""
    if not prs:
        return None
    lines = [f"- {pr['title']} by @{pr['author']} in #{pr['number']}" for pr in prs]
    return "## Merged PRs\n" + "\n".join(lines)


def format_new_contributors(entries: list[tuple[str, int]]) -> str | None:
    """Format New Contributors; entries are (login, first_pr_number). None when empty."""
    if not entries:
        return None
    lines = [f"- @{login} made their first contribution in #{number}" for login, number in entries]
    return "## New Contributors\n" + "\n".join(lines)


def compose_release_body(
    changelog_section: str,
    merged_prs: str | None,
    new_contributors: str | None,
    full_changelog_url: str,
) -> str:
    """Assemble the agreed GitHub Release body."""
    parts: list[str] = []
    if merged_prs:
        parts.extend([merged_prs.strip(), ""])
    parts.extend([changelog_section.strip(), ""])
    if new_contributors:
        parts.extend([new_contributors.strip(), ""])
    parts.append(f"**Full Changelog**: {full_changelog_url}")
    return "\n".join(parts).rstrip() + "\n"


def _gh(*args: str, input_text: str | None = None) -> str:
    env = dict(os.environ)
    result = subprocess.run(
        ["gh", *args],
        capture_output=True,
        text=True,
        input=input_text,
        env=env,
    )
    if result.returncode != 0:
        raise VersioningError(f"gh {' '.join(args)} failed: {result.stderr.strip()}")
    return result.stdout


def _repo_slug(repo: Path) -> str:
    if slug := os.environ.get("GITHUB_REPOSITORY"):
        return slug
    url = _gh("repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner").strip()
    if not url:
        raise VersioningError("could not resolve GitHub repository slug")
    return url


def _parse_iso(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00"))


def tag_creatordate(repo: Path, tag: str) -> datetime:
    """Return the tag creation time (tagger date for annotated tags)."""
    raw = subprocess.run(
        ["git", "for-each-ref", "--format=%(creatordate:iso-strict)", f"refs/tags/{tag}"],
        cwd=repo,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    if not raw:
        raise VersioningError(f"no creatordate for tag {tag}")
    return _parse_iso(raw)


def previous_tag_name(repo: Path, tag: str) -> str | None:
    """Return the previous version tag, or None when this is the first."""
    listed = subprocess.run(
        ["git", "tag", "--list", "--sort=-version:refname"],
        cwd=repo,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.splitlines()
    version_tags = [name for name in listed if re.fullmatch(r"v?\d+\.\d+\.\d+", name)]
    if tag not in version_tags:
        version_tags = [tag, *version_tags]
    try:
        idx = version_tags.index(tag)
    except ValueError:
        return None
    if idx + 1 >= len(version_tags):
        return None
    return version_tags[idx + 1]


def full_changelog_url(repo_slug: str, previous_tag: str | None, tag: str) -> str:
    if previous_tag:
        return f"https://github.com/{repo_slug}/compare/{previous_tag}...{tag}"
    return f"https://github.com/{repo_slug}/commits/{tag}"


def fetch_merged_devel_prs(repo_slug: str, after: datetime | None, until: datetime) -> list[dict]:
    """Merged PRs with base hybrid-devel in (after, until], oldest first."""
    raw = _gh(
        "pr",
        "list",
        "--repo",
        repo_slug,
        "--base",
        DEVEL_BASE,
        "--state",
        "merged",
        "--limit",
        "100",
        "--json",
        "number,title,author,mergedAt",
    )
    selected: list[dict] = []
    for item in json.loads(raw):
        merged_at = _parse_iso(item["mergedAt"])
        if after is not None and not (after < merged_at):
            continue
        if merged_at > until:
            continue
        selected.append(
            {
                "number": item["number"],
                "title": item["title"],
                "author": item["author"]["login"],
                "mergedAt": item["mergedAt"],
            }
        )
    selected.sort(key=lambda pr: pr["mergedAt"])
    return selected


def first_merged_pr_number(repo_slug: str, login: str) -> int | None:
    """Return the author's earliest merged PR number in this repo, or None."""
    raw = _gh(
        "pr",
        "list",
        "--repo",
        repo_slug,
        "--author",
        login,
        "--state",
        "merged",
        "--limit",
        "100",
        "--json",
        "number,mergedAt",
    )
    items = json.loads(raw)
    if not items:
        return None
    items.sort(key=lambda pr: pr["mergedAt"])
    return items[0]["number"]


def new_contributor_entries(repo_slug: str, prs: list[dict]) -> list[tuple[str, int]]:
    """Authors whose first-ever merged PR in the repo is one of ``prs``."""
    window_numbers = {pr["number"] for pr in prs}
    seen: set[str] = set()
    entries: list[tuple[str, int]] = []
    for pr in prs:
        login = pr["author"]
        if login in seen:
            continue
        seen.add(login)
        first = first_merged_pr_number(repo_slug, login)
        if first is not None and first in window_numbers:
            entries.append((login, first))
    return entries


def release_exists(tag: str) -> bool:
    result = subprocess.run(
        ["gh", "release", "view", tag],
        capture_output=True,
        text=True,
    )
    return result.returncode == 0


def create_release(tag: str, title: str, body: str, *, dry_run: bool) -> None:
    if release_exists(tag):
        print(f"release exists: {tag}")
        return
    if dry_run:
        print(f"dry-run: would create release {tag!r} title={title!r}")
        print(body)
        return
    _gh(
        "release",
        "create",
        tag,
        "--title",
        title,
        "--notes",
        body,
    )
    print(f"created release: {tag}")


def build_body(repo: Path, tag: str) -> tuple[str, str]:
    changelog = (repo / CHANGELOG_NAME).read_text(encoding="utf-8")
    version, section = extract_tip_released_section(changelog)
    expected_tag = f"{load_config(repo)['tag_prefix']}{version}"
    if tag != expected_tag:
        raise VersioningError(f"tag {tag} does not match tip CHANGELOG version tag {expected_tag}")

    slug = _repo_slug(repo)
    prev = previous_tag_name(repo, tag)
    until = tag_creatordate(repo, tag)
    after = tag_creatordate(repo, prev) if prev else None
    prs = fetch_merged_devel_prs(slug, after, until)
    merged = format_merged_prs(prs)
    new_contrib = format_new_contributors(new_contributor_entries(slug, prs))
    full = full_changelog_url(slug, prev, tag)
    return version, compose_release_body(section, merged, new_contrib, full)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--tag", required=True, help="Release tag, e.g. v2.1.0")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument(
        "--compose-only",
        action="store_true",
        help="Print composed body to stdout; do not create the release",
    )
    args = parser.parse_args(argv)
    repo = args.repo.resolve()

    try:
        version, body = build_body(repo, args.tag)
        if args.compose_only:
            sys.stdout.write(body)
            return 0
        create_release(args.tag, version, body, dry_run=args.dry_run)
    except VersioningError as error:
        print(f"GITHUB_RELEASE: FAIL\n{error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
