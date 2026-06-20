# Makefile for MuJoCo ROS documentation.

SPHINXBUILD ?= python3 -m sphinx
SPHINXOPTS  ?=
SOURCEDIR   := .
BUILDDIR    := build
HTMLDIR     := $(BUILDDIR)/html
RELEASE_TAG_PATTERN ?= v[0-9]*
RELEASE_REFS ?=

.PHONY: help html versioned clean

help:
	@$(SPHINXBUILD) -M help "$(SOURCEDIR)" "$(BUILDDIR)" -c "$(SOURCEDIR)" $(SPHINXOPTS)
	@echo
	@echo "Project targets:"
	@echo "  html       build one local documentation tree"
	@echo "  versioned  build devel/ and one tree per release tag"

html:
	@MUJOCO_ROS_DOCS_VERSION=devel \
	 MUJOCO_ROS_DOCS_RELEASE=devel \
	 MUJOCO_ROS_DOCS_SOURCE_REF=hybrid-devel \
	 $(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(HTMLDIR)" -c "$(SOURCEDIR)" $(SPHINXOPTS)

versioned:
	@rm -rf "$(HTMLDIR)"
	@mkdir -p "$(HTMLDIR)"
	@release_refs="$(RELEASE_REFS)"; \
	if [ -z "$${release_refs}" ]; then \
	  release_refs="$$(git -C .. tag --list '$(RELEASE_TAG_PATTERN)' --sort=v:refname)"; \
	fi; \
	versions='devel:../devel/'; \
	for ref in $${release_refs}; do \
	  version="$${ref#v}"; \
	  versions="$${versions},$${version}:../$${version}/"; \
	done; \
	if [ -n "$${release_refs}" ]; then \
	  echo "Building release documentation for: $${release_refs}"; \
	else \
	  echo "No release tags matching '$(RELEASE_TAG_PATTERN)' found; building only devel documentation."; \
	fi; \
	MUJOCO_ROS_DOCS_VERSION=devel \
	MUJOCO_ROS_DOCS_RELEASE=devel \
	MUJOCO_ROS_DOCS_SOURCE_REF=hybrid-devel \
	MUJOCO_ROS_DOCS_VERSIONS="$${versions}" \
	$(SPHINXBUILD) -b html "$(SOURCEDIR)" "$(HTMLDIR)/devel" -c "$(SOURCEDIR)" $(SPHINXOPTS)
	@release_refs="$(RELEASE_REFS)"; \
	if [ -z "$${release_refs}" ]; then \
	  release_refs="$$(git -C .. tag --list '$(RELEASE_TAG_PATTERN)' --sort=v:refname)"; \
	fi; \
	versions='devel:../devel/'; \
	for ref in $${release_refs}; do \
	  version="$${ref#v}"; \
	  versions="$${versions},$${version}:../$${version}/"; \
	done; \
	for ref in $${release_refs}; do \
	  version="$${ref#v}"; \
	  worktree="$${TMPDIR:-/tmp}/mujoco_ros_docs_$${version}"; \
	  rm -rf "$${worktree}"; \
	  git -C .. worktree add --detach "$${worktree}" "$${ref}"; \
	  MUJOCO_ROS_DOCS_VERSION="$${version}" \
	  MUJOCO_ROS_DOCS_RELEASE="$${version}" \
	  MUJOCO_ROS_DOCS_SOURCE_REF="$${ref}" \
	  MUJOCO_ROS_DOCS_VERSIONS="$${versions}" \
	  $(SPHINXBUILD) -b html "$${worktree}/docs" "$(HTMLDIR)/$${version}" -c "$${worktree}/docs" $(SPHINXOPTS); \
	  git -C .. worktree remove --force "$${worktree}"; \
	done
	@printf '%s\n' \
	  '<!doctype html>' \
	  '<html><head><meta charset="utf-8">' \
	  '<meta http-equiv="refresh" content="0; url=devel/">' \
	  '<title>MuJoCo ROS documentation</title></head>' \
	  '<body><p><a href="devel/">MuJoCo ROS documentation</a></p></body></html>' \
	  > "$(HTMLDIR)/index.html"
	@touch "$(HTMLDIR)/.nojekyll"

clean:
	@$(SPHINXBUILD) -M clean "$(SOURCEDIR)" "$(BUILDDIR)" -c "$(SOURCEDIR)" $(SPHINXOPTS)
