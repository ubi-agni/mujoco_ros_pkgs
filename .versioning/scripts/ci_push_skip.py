#!/usr/bin/env python3
"""Decide whether a push should skip the full CI matrix.

A push is skipped when its head commit is written by the Version Bot, or when the commit
belongs to a merged PR into the devel branch (rebase-merge after a green PR run).

ponytail: only the Version Bot author check is reliable. The merged-PR skip depends on
`commits/{sha}/pulls` associating rebase-merged SHAs, which is not yet verified: the devel tip
returned no PRs because no PR has merged on that repo. A miss runs full CI, never skips it.

Usage: ci_push_skip.py --author-name NAME --author-email EMAIL [--pr-base BASE ...]
                       [--devel-branch BRANCH]
Prints `skip=true` or `skip=false` to stdout and appends it to $GITHUB_OUTPUT when set.
"""

import argparse
import os
import sys

from bump_versions import BOT_EMAIL, BOT_NAME


def is_version_bot_author(name, email):
    return name == BOT_NAME and email == BOT_EMAIL


def should_skip_full_ci_push(
    *, author_name, author_email, associated_merged_pr_bases, devel_branch="hybrid-devel"
):
    # An empty author means the event payload is missing; running CI on it would be a guess.
    if not author_name or not author_email:
        raise ValueError("push author name and email are required to decide the CI skip")
    if is_version_bot_author(author_name, author_email):
        return True
    return devel_branch in associated_merged_pr_bases


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--author-name", required=True)
    parser.add_argument("--author-email", required=True)
    parser.add_argument("--pr-base", action="append", default=[], dest="pr_bases")
    parser.add_argument("--devel-branch", default="hybrid-devel")
    args = parser.parse_args(argv)

    try:
        skip = should_skip_full_ci_push(
            author_name=args.author_name,
            author_email=args.author_email,
            associated_merged_pr_bases=args.pr_bases,
            devel_branch=args.devel_branch,
        )
    except ValueError as exc:
        print(f"ci_push_skip: {exc}", file=sys.stderr)
        return 1

    line = f"skip={'true' if skip else 'false'}"
    print(line)
    output_path = os.environ.get("GITHUB_OUTPUT")
    if output_path:
        with open(output_path, "a", encoding="utf-8") as handle:
            handle.write(line + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
