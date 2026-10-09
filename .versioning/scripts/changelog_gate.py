#!/usr/bin/env python3
"""CI gate: changes to shipped code need a non-chore CHANGELOG [Unreleased] entry.

Checks the diff between BASE_REF and the branch head: shipped files without an entry,
an entry without shipped files, malformed entry lines, and edits to released sections.

Version Bot bumps move Unreleased entries into a new released section. After that bump,
reviewer commits may sit on top. The gate then:
  - allows the released section to grow by prepending (old released text stays a suffix);
  - evaluates Unreleased/shipped against the tree before the newest Version Bot bump when
    HEAD itself has no non-chore Unreleased entry, unless new shipped files landed after
    that bump (those still need a fresh Unreleased entry on HEAD).

Usage: changelog_gate.py [--repo PATH] [--base-ref REF]
"""

import argparse
import sys
from pathlib import Path

from bump_versions import BOT_EMAIL
from versioning_common import (
    CHANGELOG_NAME,
    ENTRY_RE,
    VersioningError,
    git,
    is_shipped_file,
    load_config,
    split_changelog,
)


def _file_at(repo, ref, path):
    if not git(repo, "ls-tree", "--name-only", ref, "--", path).strip():
        return ""
    return git(repo, "show", f"{ref}:{path}")


def _newest_bot_bump(repo, base_ref):
    """Return (bot_sha, parent_sha) for the newest Version Bot commit in base..HEAD, or None."""
    log = git(repo, "log", "--format=%H %ae", f"{base_ref}..HEAD")
    for line in log.splitlines():
        sha, email = line.split(" ", 1)
        if email.strip() == BOT_EMAIL:
            parent = git(repo, "rev-parse", f"{sha}^").strip()
            return sha, parent
    return None


def _added_unreleased_lines(before_body, after_body):
    before_lines = {line.rstrip() for line in before_body.splitlines()}
    return [
        line.rstrip()
        for line in after_body.splitlines()
        if line.strip() and not line.startswith("#") and line.rstrip() not in before_lines
    ]


def _classify_entries(added):
    entries = [line for line in added if ENTRY_RE.match(line)]
    malformed = [line for line in added if not ENTRY_RE.match(line)]
    non_chore = [line for line in entries if ENTRY_RE.match(line).group(1) != "chore"]
    return entries, malformed, non_chore


def _released_only_grew(before_tail, after_tail):
    if before_tail == after_tail:
        return True
    if not before_tail:
        return True
    return after_tail.endswith(before_tail)


def run_changelog_gate(repo_root: Path, base_ref: str) -> None:
    head = "HEAD"
    bot = _newest_bot_bump(repo_root, base_ref)

    before = _file_at(repo_root, base_ref, CHANGELOG_NAME)
    after = _file_at(repo_root, head, CHANGELOG_NAME)
    before_body, before_tail = split_changelog(before)
    after_body, after_tail = split_changelog(after)

    failures = []
    if not _released_only_grew(before_tail, after_tail):
        failures.append(f"released section was mutated in {CHANGELOG_NAME}")

    changed_to_head = git(repo_root, "diff", "--name-only", f"{base_ref}...{head}").splitlines()
    shipped_to_head = [path for path in changed_to_head if is_shipped_file(path)]

    added_head = _added_unreleased_lines(before_body, after_body)
    _, malformed_head, non_chore_head = _classify_entries(added_head)

    content_ref = head
    if bot is not None and not non_chore_head:
        bot_sha, bot_parent = bot
        post_bump_shipped = [
            path
            for path in git(repo_root, "diff", "--name-only", f"{bot_sha}...{head}").splitlines()
            if is_shipped_file(path)
        ]
        if not post_bump_shipped:
            # No new shipped work after the bump: gate the pre-bump feature tip.
            content_ref = bot_parent

    if content_ref == head:
        shipped = shipped_to_head
        added = added_head
        malformed = malformed_head
        non_chore = non_chore_head
    else:
        content_changed = git(
            repo_root, "diff", "--name-only", f"{base_ref}...{content_ref}"
        ).splitlines()
        shipped = [path for path in content_changed if is_shipped_file(path)]
        content_changelog = _file_at(repo_root, content_ref, CHANGELOG_NAME)
        content_body, _ = split_changelog(content_changelog)
        added = _added_unreleased_lines(before_body, content_body)
        _, malformed, non_chore = _classify_entries(added)

    if shipped and not non_chore:
        files = "\n".join(f"  - {path}" for path in shipped)
        failures.append(
            f"shipped file(s) changed with no non-chore [Unreleased] entry:\n{files}\n"
            "Add a [patch], [minor], or [major] entry under ## Unreleased in CHANGELOG.md."
        )
    if non_chore and not shipped:
        lines = "\n".join(f"  - {line}" for line in non_chore)
        failures.append(f"[Unreleased] entry added but no shipped file changed:\n{lines}")
    if malformed:
        lines = "\n".join(f"  - {line}" for line in malformed)
        failures.append(f"malformed [Unreleased] line(s):\n{lines}")

    if failures:
        raise VersioningError("\n\n".join(failures))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repo", type=Path, default=Path("."))
    parser.add_argument("--base-ref", help="defaults to origin/<merge_target> from config")
    args = parser.parse_args(argv)

    repo = args.repo.resolve()
    try:
        base_ref = args.base_ref or f"origin/{load_config(repo)['merge_target']}"
        run_changelog_gate(repo, base_ref)
    except VersioningError as error:
        print(f"CHANGELOG GATE: FAIL\n{error}", file=sys.stderr)
        return 1
    print("CHANGELOG GATE: PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
