# MuJoCo ROS Documentation

This directory contains the Sphinx documentation for MuJoCo ROS.

Build locally with:

```bash
python3 -m pip install -r docs/requirements.txt
make -C docs html
```

Open `docs/build/html/index.html` after a successful build.

To reproduce the GitHub Pages layout locally, including the version selector,
run:

```bash
make -C docs versioned
```

Open `docs/build/html/index.html`; it redirects to `devel/`.

The `versioned` target always builds `docs/build/html/devel/` from the current
checkout. It also builds one release tree per local tag matching `v[0-9]*`.
For example, tags `v1.0.0`, `v1.0.1`, and `v1.1.0` produce
`docs/build/html/1.0.0/`, `docs/build/html/1.0.1/`, and
`docs/build/html/1.1.0/`.

To test a specific set of release refs locally:

```bash
make -C docs versioned RELEASE_REFS="v1.0.0 v1.0.1"
```

## When GitHub Pages updates

The `Documentation` workflow (`.github/workflows/sphinxdoc.yaml`) publishes the
site. It runs in three cases:

- A push to `hybrid-devel` that touches `docs/`, `CHANGELOG.md`, `README.md`, or
  the workflow file rebuilds the unstable `devel/` tree from that branch tip.
  The workflow does not skip Version Bot commits. Bumps pushed with
  `GITHUB_TOKEN` do not start it, so the release workflow calls it after the
  bump is verified. That rebuilds unstable from the release PR head.
- A release PR into `hybrid-main` rebuilds unstable from the verified bump
  commit at the PR head (`release.yaml`, after `version-match`).
- A new `vX.Y.Z` tag created by the tag-release workflow after a release merge
  to `hybrid-main` builds the released trees in the same run. Tags pushed by
  GitHub Actions with `GITHUB_TOKEN` do not start other workflows, so this call
  is the only automatic path for those tags.
- A `v*` tag pushed by a person starts the workflow directly. Tags that already
  exist do not rebuild the site from the tag-release run.

Pages deploys are serialized, so a late release build cannot overlap another
docs deploy.
