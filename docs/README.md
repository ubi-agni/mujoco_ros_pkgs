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
