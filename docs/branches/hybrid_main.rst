``hybrid-main``
===============

``hybrid-main`` is the stable release branch and the repository default.
Release tags (``vX.Y.Z``) point at commits on this branch.

How releases reach ``hybrid-main``
---------------------------------

Feature work lands on ``hybrid-devel`` first.
A release is a pull request from ``hybrid-devel`` into ``hybrid-main``,
merged with rebase merge so history stays linear.

That pull request runs the slim release workflow only (changelog gate,
version bump, version match, docs build, pre-commit). Full ROS CI does
not run on it.

After the merge, a tag-only workflow on ``hybrid-main`` creates the
annotated ``vX.Y.Z`` tag when the tip version is not tagged yet, and
builds the released documentation trees.

See :doc:`hybrid_devel` for the integration line, and the root
``CONTRIBUTING.md`` for the full release checklist and GitHub settings.
