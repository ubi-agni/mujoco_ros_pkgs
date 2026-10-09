# Contributing to `mujoco_ros_pkgs`

This is an open source project welcoming contributions. In this document we list a few requirements you should adhere to.

## Branches

| Branch | Role | How changes land |
|--------|------|------------------|
| `hybrid-devel` | Integration line. Feature work targets it. | Pull requests, or a direct push for maintainer hotfixes |
| `hybrid-main` | Stable releases. Default branch. | Release pull request from `hybrid-devel`, merged by rebase |

Release tags use the form `vX.Y.Z` (for example `v2.0.0`) and point at `hybrid-main`.
CI creates them after a release merge. Do not create release tags by hand.

Older history used `noetic-devel` as the integration branch. New work does not target it.

## Workflow
1. Browse Issues and Pull-Requests on Github to see if the bug or feature you want to fix/add has already been reported/requested.
If not, please create a new issue.

2. Fork the repository, if you havn't done it already. Then clone the forked project and add the upstream repository:
    ```sh
    git clone https://github.com/<your-username>/mujoco_ros_pkgs.git
    cd mujoco_ros_pkgs
    git remote add upstream https://github.com/ubi-agni/mujoco_ros_pkgs.git
    ```

3. Setup pre-commit:
    * Install pre-commit with `pip install pre-commit`
    * In the root directory of the repository run the following commands
    ```sh
    # sets up clang-tidy, clang-format and other utils on commit
    pre-commit install-hooks
    # sets up commit message check
    pre-commit install --hook-type commit-msg
    ```
    * Optionally set our commit template to be shown when you commit: `git config commit.template .housekeeping/git-commit-template.txt`

4. Develop your contribution

    * Make sure your fork is up to date with the `hybrid-devel` branch of the upstream repository:
    ```sh
    git checkout hybrid-devel
    git pull upstream hybrid-devel
    ```
    * Create a branch for your contribution with a sensible name:
    ```sh
    git checkout -b add-featureX
    ```
    or
    ```sh
    git checkout -b fix-bugX
    ```

    * As you code, commit your changes (following conventional commits style, the commit template will help you with that).
    pre-commit will automatically run clang-format and clang-tidy and make sure your commit message conforms with our template.

    * Write tests using Google Tests to test your code.
    In case your contribution is a bugfix, add or improve a test to catch the bug before the fix is applied. This ensures resurfacing of the bug due to later code changes is caught.

    * Add an entry under `## Unreleased` in `CHANGELOG.md` for user-visible changes. Each entry starts with its impact tag: `[major]`, `[minor]`, `[patch]`, or `[chore]`.

    * Do not edit version numbers. Package versions and `CMakeLists.txt` version lines are set by the release process, not by feature PRs.

5. Propose changes via Pull Requests

   * When your contribution is ready and all tests are passing, make sure all your changes are pushed to your feature branch
   * Submit a pull request into `ubi-agni/mujoco_ros_pkgs:hybrid-devel` with an informative title and a detailed description.
   * Please link all relevant issues to your PR.
   * Full CI runs on pull requests into `hybrid-devel`. Keep it green.
   * The team will review your contribution and provide feedback. To incorporate changes recommended by the reviewers, commit edits to your branch, and push to the branch again (there is no need to re-create the pull request, it will automatically track modifications to your branch).
   * Once your pull request is approved by the reviewers, it will be merged into the main codebase.

## Releases

A release is a pull request from `hybrid-devel` into `hybrid-main`.

1. Make sure every change for the release is on `hybrid-devel` and listed under `## Unreleased` in `CHANGELOG.md`.
2. Open a pull request with base `hybrid-main` and head `hybrid-devel`.
3. The **Changelog gate** check validates the Unreleased entries. It rejects edits to released sections. Fix failures in the entries, not in released sections.
4. The **Version bump** job computes the next version from the highest impact tag. It then pushes one commit to the pull request branch as `Version Bot <version-bot@ci>`. That commit:
    * sets the version in every package `package.xml` and in each `CMakeLists.txt` version line,
    * moves `## Unreleased` into `## [X.Y.Z] - <date>`.
5. The **Version match** and **pre-commit** checks run on the bumped commit in the same workflow run. Wait for them to pass.
6. Merge with **rebase merge** only. Do not squash or create a merge commit.
7. After the merge, CI pushes the tag `vX.Y.Z` to `hybrid-main`. The tag run builds the released documentation.

### Version rules

* **No hand version bumps.** Do not edit version numbers on `hybrid-devel`. The release pull request makes the bump.
* **`[skip-bump]`** in a commit message turns off the bump job for that commit. The version-match check still runs, so the versions must already be correct.
* **Chore-only changes.** If all Unreleased entries are `[chore]`, no version is bumped and no tag is created. Do not open a versioned release for them.
* **Version Bot commits** do not start a full CI run. The Version Bot's own bump commit is checked by the release workflow.

### Documentation

* **Unstable docs** track `hybrid-devel`. They rebuild on pushes to `hybrid-devel` that touch the docs, and on each release-pull-request bump.
* **Released docs** are built from `v*` tags. Each tag adds one release tree to the version selector.
* See [docs/README.md](docs/README.md) for the full list of triggers and the local preview commands.

### CI at a glance

| Event | What runs |
|-------|-----------|
| Pull request into `hybrid-devel` | Full CI (ROS 2, ROS 1, format, docker) |
| Push to `hybrid-devel` | Full CI, except Version Bot pushes and commits that came in through a merged pull request into `hybrid-devel` |
| Pull request into `hybrid-main` | Release checks only: changelog gate, version bump, version match, docs, pre-commit |
| Push to `hybrid-main` | Tag job only |

> [!NOTE]
> ponytail: The skip for commits that came in through a merged pull request is best effort. It depends on GitHub linking rebase-merged commits to their pull request, which is not yet confirmed on this fork. If the link is missing, full CI runs again. It assumes the pull request was green before merge, so keep the full CI check required on `hybrid-devel`. The Version Bot skip is based on the commit author and is reliable.

## Maintainer notes

### GitHub settings (cutover)

Do these once, in this order.

1. Create `hybrid-main` from the current stable tip (the commit tagged with the latest release, for example `v2.0.0`). Push it.
2. Set the default branch to `hybrid-main`.
3. Protect `hybrid-main`:
    * require a pull request before merging,
    * forbid force-pushes and branch deletion,
    * allow only rebase merge,
    * require the release checks only: **Changelog gate**, **Version match**, **pre-commit**, and **Unstable documentation** (recommended; drop it only if the environment cannot allow it).
4. `hybrid-devel`: keep direct pushes allowed for maintainers. Optionally require the full CI checks on pull requests into `hybrid-devel`. The merged-pull-request skip assumes those checks were green.
5. Actions: under repository workflow permissions, set the `GITHUB_TOKEN` to read and write. Only the release **Version bump** job requests `contents: write`.
6. Allow `github-actions[bot]` to push to `hybrid-devel`. Do this if branch rules or rulesets would block the Version Bot push. Without it the bump job fails loudly and the version-match check stays red.
7. Do not add a personal access token. The default design uses only `GITHUB_TOKEN`. Pushes made with it do not start new workflow runs, so the release workflow runs its own checks after the bump.
8. `github-pages` environment: allow deployments from `hybrid-devel` docs pushes, `hybrid-main` (tag-release), and `v*` tag pushes. Release PRs build docs but do not deploy.

### Acceptance after cutover

Check these on the first real release. Record the result in the release notes or the PR.

* A release pull request into `hybrid-main` runs only the release workflow. Full CI does not run on it.
* The bump and the version match happen in one release workflow run. The version match passes on the bumped commit.
* The release pull request is merged with rebase merge, and the commit history on `hybrid-main` matches the `hybrid-devel` commits.
* The tag `vX.Y.Z` is created on `hybrid-main` by the tag job, without an extra commit on `hybrid-main`.
* The unstable docs on the Pages site follow `hybrid-devel` (including the Version Bot CHANGELOG after it lands on devel). Release PRs build docs without deploying them.
* The released docs for `vX.Y.Z` appear in the version selector after the tag run.
* A direct push to `hybrid-devel` by a maintainer runs full CI.
* A Version Bot push to `hybrid-devel` does not run full CI or docker builds.
* The rebase-merge link to its pull request is confirmed once on this repository, or the merged-pull-request skip is recorded as unverified.
