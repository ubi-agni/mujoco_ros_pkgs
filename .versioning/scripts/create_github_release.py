#!/usr/bin/env python3
"""Compose and publish a GitHub Release for the tip version tag.

Body shape (agreed for hybrid-main tag runs):

  ## Merged PRs
  - <PR title> by @user in #N
  ...

  <tip CHANGELOG section body>

  ## New Contributors          # omitted when generate-notes has none
  - @user made their first contribution in #N
  ...

  **Full Changelog**: <compare URL>

Usage:
  create_github_release.py [--repo PATH] --tag vX.Y.Z [--dry-run]
  create_github_release.py [--repo PATH] --compose-only --tag vX.Y.Z
      [--generate-notes-file PATH]

`--compose-only` prints the body to stdout (no API create). Without
`--generate-notes-file`, generate-notes is fetched via `gh api`.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
from pathlib import Path

from versioning_common import (
    CHANGELOG_NAME,
    RELEASED_HEADING_RE,
    SECTION_START_RE,
    VersioningError,
    load_config,
    split_changelog,
)

PR_LINE_RE = re.compile(r"^\* (.+) by (@\S+) in https://github\.com/[^/]+/[^/]+/pull/(\d+)\s*$")
CONTRIB_LINE_RE = re.compile(
    r"^\* (@\S+) made their first contribution in https://github\.com/[^/]+/[^/]+/pull/(\d+)\s*$"
)
FULL_CHANGELOG_RE = re.compile(r"^\*\*Full Changelog\*\*: (\S+)\s*$", re.MULTILINE)
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
    # Body starts after the heading line.
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


def rewrite_generate_notes(auto_body: str) -> tuple[str, str | None, str]:
    """Parse GitHub generate-notes body into Merged PRs, optional New Contributors, compare URL."""
    full_match = FULL_CHANGELOG_RE.search(auto_body)
    if full_match is None:
        raise VersioningError("generate-notes body has no Full Changelog line")
    full_url = full_match.group(1)

    whats = re.search(
        r"## What's Changed\n(.*?)(?=\n## |\n\*\*Full Changelog\*\*|\Z)",
        auto_body,
        re.S,
    )
    if whats is None:
        raise VersioningError("generate-notes body has no What's Changed section")

    merged_lines = []
    for line in whats.group(1).strip().splitlines():
        line = line.strip()
        if not line:
            continue
        match = PR_LINE_RE.match(line)
        if match is None:
            raise VersioningError(f"unrecognized What's Changed line: {line!r}")
        title, user, number = match.groups()
        merged_lines.append(f"- {title} by {user} in #{number}")
    if not merged_lines:
        raise VersioningError("generate-notes What's Changed section is empty")
    merged = "## Merged PRs\n" + "\n".join(merged_lines)

    new_contrib = None
    contrib = re.search(
        r"## New Contributors\n(.*?)(?=\n\*\*Full Changelog\*\*|\Z)",
        auto_body,
        re.S,
    )
    if contrib is not None:
        contrib_lines = []
        for line in contrib.group(1).strip().splitlines():
            line = line.strip()
            if not line:
                continue
            match = CONTRIB_LINE_RE.match(line)
            if match is None:
                raise VersioningError(f"unrecognized New Contributors line: {line!r}")
            user, number = match.groups()
            contrib_lines.append(f"- {user} made their first contribution in #{number}")
        if contrib_lines:
            new_contrib = "## New Contributors\n" + "\n".join(contrib_lines)

    return merged, new_contrib, full_url


def compose_release_body(
    changelog_section: str,
    merged_prs: str,
    new_contributors: str | None,
    full_changelog_url: str,
) -> str:
    """Assemble the agreed GitHub Release body."""
    parts = [merged_prs.strip(), "", changelog_section.strip(), ""]
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


def fetch_generate_notes(repo_slug: str, tag: str, previous_tag: str | None) -> str:
    args = [
        "api",
        f"repos/{repo_slug}/releases/generate-notes",
        "-f",
        f"tag_name={tag}",
    ]
    if previous_tag:
        args.extend(["-f", f"previous_tag_name={previous_tag}"])
    payload = json.loads(_gh(*args))
    body = payload.get("body")
    if not body:
        raise VersioningError("generate-notes returned an empty body")
    return body


def previous_tag_name(repo: Path, tag: str) -> str | None:
    """Return the previous version tag, or None when this is the first."""
    listed = subprocess.run(
        ["git", "tag", "--list", "--sort=-version:refname"],
        cwd=repo,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.splitlines()
    # Prefer tags that look like releases (optional v + semver).
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


def build_body(repo: Path, tag: str, generate_notes_text: str | None) -> tuple[str, str]:
    changelog = (repo / CHANGELOG_NAME).read_text(encoding="utf-8")
    version, section = extract_tip_released_section(changelog)
    expected_tag = f"{load_config(repo)['tag_prefix']}{version}"
    if tag != expected_tag:
        raise VersioningError(f"tag {tag} does not match tip CHANGELOG version tag {expected_tag}")
    if generate_notes_text is None:
        prev = previous_tag_name(repo, tag)
        generate_notes_text = fetch_generate_notes(_repo_slug(repo), tag, prev)
    merged, new_contrib, full = rewrite_generate_notes(generate_notes_text)
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
    parser.add_argument(
        "--generate-notes-file",
        type=Path,
        help="Optional path to a generate-notes body (skips gh api)",
    )
    args = parser.parse_args(argv)
    repo = args.repo.resolve()

    try:
        notes_text = None
        if args.generate_notes_file is not None:
            notes_text = args.generate_notes_file.read_text(encoding="utf-8")
        version, body = build_body(repo, args.tag, notes_text)
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
