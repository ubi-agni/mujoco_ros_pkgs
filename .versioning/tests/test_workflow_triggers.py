import json
import re

import pytest
import yaml

from helpers import VERSIONING_DIR

WORKFLOWS = VERSIONING_DIR.parent / ".github" / "workflows"
DEVEL = ["hybrid-devel"]
PUSH_GATE = "push-gate"
# The first job of each workflow; it waits on the push gate, later jobs chain off it.
ENTRYPOINT = {
    "format.yaml": "pre-commit",
    "ci.ros2.yaml": "ci-policy",
    "ci.ros1.yaml": "ci-policy",
    "docker.ros2.yaml": "ci_ros2",
    "docker.ros1.yaml": "ci_ros1",
}


def _load(name):
    doc = yaml.safe_load((WORKFLOWS / name).read_text(encoding="utf-8"))
    # PyYAML parses the bare `on` key as boolean True (YAML 1.1).
    return doc, doc.get("on", doc.get(True))


@pytest.mark.parametrize("name", list(ENTRYPOINT))
def test_triggers_target_devel_only(name):
    _, triggers = _load(name)

    assert triggers["pull_request"]["branches"] == DEVEL
    assert triggers["push"]["branches"] == DEVEL
    assert "workflow_dispatch" in triggers


@pytest.mark.parametrize("name", list(ENTRYPOINT))
def test_push_gate_guards_entrypoint(name):
    doc, _ = _load(name)
    gate = doc["jobs"][PUSH_GATE]
    entry = doc["jobs"][ENTRYPOINT[name]]

    assert gate["if"] == "github.event_name == 'push'"
    assert PUSH_GATE in entry["needs"]
    assert "push-gate.outputs.skip == 'false'" in entry["if"]
    # always() would run superseded runs cancelled by cancel-in-progress.
    assert "always()" not in entry["if"]
    assert "!cancelled()" in entry["if"]


RELEASE = "release.yaml"
RELEASE_JOBS = ["changelog-gate", "version-bump", "version-match", "format"]
FORBIDDEN_IDENTIFIER = re.compile(r"neura|sdd|neura_spec", re.IGNORECASE)


def test_release_triggers_target_hybrid_main_prs_only():
    doc, triggers = _load(RELEASE)

    assert triggers["pull_request"]["branches"] == ["hybrid-main"]
    assert "push" not in triggers
    assert doc["permissions"] == {"contents": "read"}
    assert doc["jobs"]["version-bump"]["permissions"] == {"contents": "write"}


def test_release_format_does_not_upload_patches_with_write_token():
    doc, _ = _load(RELEASE)
    format_steps = "\n".join(step.get("uses", "") for step in doc["jobs"]["format"]["steps"])

    assert "pre-commit/action" in format_steps
    assert "upload-git-patch" not in format_steps


def test_release_jobs_run_in_order_on_one_run():
    doc, _ = _load(RELEASE)

    assert list(doc["jobs"]) == [
        "changelog-gate",
        "version-bump",
        "version-match",
        "docs",
        "format",
    ]
    assert "needs" not in doc["jobs"]["changelog-gate"]
    assert doc["jobs"]["version-bump"]["needs"] == "changelog-gate"
    assert doc["jobs"]["version-match"]["needs"] == "version-bump"
    # format and docs both read the verified tip from version-match.
    assert doc["jobs"]["format"]["needs"] == "version-match"
    assert doc["jobs"]["docs"]["needs"] == "version-match"


def test_release_calls_sphinxdoc_after_verified_bump():
    doc, _ = _load(RELEASE)
    docs = doc["jobs"]["docs"]

    assert docs["uses"] == "./.github/workflows/sphinxdoc.yaml"
    assert docs["needs"] == "version-match"
    assert docs["with"]["ref"] == "${{ needs.version-match.outputs.sha }}"
    assert docs["permissions"] == {"contents": "read", "pages": "write", "id-token": "write"}


def test_release_docs_job_runs_after_bump_is_verified():
    doc, _ = _load(RELEASE)

    # sphinxdoc reads the post-bump tip, so it must wait for the version check.
    assert "version-match" in doc["jobs"]
    assert (
        doc["jobs"]["version-match"]["outputs"]["sha"] == "${{ needs.version-bump.outputs.sha }}"
    )


def test_release_has_no_ros_matrix_jobs():
    doc, _ = _load(RELEASE)

    for job_id, job in doc["jobs"].items():
        assert not job_id.startswith(("ros", "ci_ros")), job_id
        assert "strategy" not in job


def test_release_job_outputs_reference_direct_needs_only():
    # A job's `needs` context exposes only its direct dependencies; a transitive
    # reference resolves to an empty string at runtime without failing YAML parsing.
    doc, _ = _load(RELEASE)

    for job_id, job in doc["jobs"].items():
        needs = job.get("needs", [])
        direct = {needs} if isinstance(needs, str) else set(needs)
        referenced = set(re.findall(r"needs\.([\w-]+)\.", json.dumps(job)))
        assert referenced <= direct, f"{job_id} reads needs outputs of {referenced - direct}"


def test_release_uses_neutral_naming():
    text = (WORKFLOWS / RELEASE).read_text(encoding="utf-8")

    assert not FORBIDDEN_IDENTIFIER.search(text)


TAG_RELEASE = "tag-release.yaml"


def test_tag_release_triggers_on_hybrid_main_push_only():
    doc, triggers = _load(TAG_RELEASE)

    assert triggers["push"]["branches"] == ["hybrid-main"]
    assert "pull_request" not in triggers
    assert "workflow_dispatch" not in triggers
    assert doc["permissions"] == {"contents": "write"}
    assert doc["concurrency"] == {"group": "tag-release", "cancel-in-progress": False}


def test_tag_release_does_not_write_package_files():
    text = (WORKFLOWS / TAG_RELEASE).read_text(encoding="utf-8")

    assert "package.xml" not in text
    assert "CMakeLists" not in text
    assert not re.search(r"bump_versions\.py\s+--repo\s+\S+\s+bump\b", text)
    assert "git commit" not in text


def test_tag_release_creates_then_pushes_tag_only():
    doc, _ = _load(TAG_RELEASE)
    run = "\n".join(
        step.get("run", "") for job in doc["jobs"].values() for step in job.get("steps", [])
    )

    assert "bump_versions.py --repo . tag" in run
    assert "git push origin" in run
    assert "tag exists" in run


def test_tag_release_uses_neutral_naming():
    text = (WORKFLOWS / TAG_RELEASE).read_text(encoding="utf-8")

    assert not FORBIDDEN_IDENTIFIER.search(text)


SPHINX = "sphinxdoc.yaml"
SPHINX_REF = "${{ inputs.ref || 'hybrid-devel' }}"


def _sphinx_checkout_steps(doc):
    return [
        step
        for job in doc["jobs"].values()
        if "steps" in job
        for step in job["steps"]
        if str(step.get("uses", "")).startswith("actions/checkout")
    ]


def test_sphinx_push_triggers_devel_branch_and_release_tags():
    doc, triggers = _load(SPHINX)

    assert triggers["push"]["branches"] == DEVEL
    assert triggers["push"]["tags"] == ["v*"]
    assert "docs/**" in triggers["push"]["paths"]
    assert "workflow_dispatch" in triggers


def test_sphinx_is_reusable_with_ref_input():
    doc, triggers = _load(SPHINX)

    call = triggers["workflow_call"]
    assert call["inputs"]["ref"]["required"] is False
    assert call["inputs"]["ref"]["type"] == "string"
    assert doc["permissions"] == {"contents": "read", "pages": "write", "id-token": "write"}


def test_sphinx_checkout_ref_follows_event():
    doc, _ = _load(SPHINX)

    checkouts = _sphinx_checkout_steps(doc)
    assert len(checkouts) == 1
    assert checkouts[0]["with"]["ref"] == SPHINX_REF
    assert checkouts[0]["with"]["fetch-tags"] is True
    assert checkouts[0]["with"]["fetch-depth"] == 0


def test_sphinx_checkout_never_uses_the_pushed_ref():
    # A tag push must not check out the tag tip: `make versioned` builds devel/ from
    # the checkout, so the checkout has to stay on hybrid-devel.
    doc, _ = _load(SPHINX)

    checkout_ref = _sphinx_checkout_steps(doc)[0]["with"]["ref"]
    assert "github.ref" not in checkout_ref
    assert "github.event_name" not in checkout_ref
    assert "github.ref_name" not in (WORKFLOWS / SPHINX).read_text(encoding="utf-8")


def test_sphinx_ref_input_documents_it_is_not_a_release_tag():
    doc, triggers = _load(SPHINX)

    description = triggers["workflow_call"]["inputs"]["ref"]["description"]
    assert "Never pass a release tag" in description


def test_sphinx_does_not_skip_version_bot_pushes():
    text = (WORKFLOWS / SPHINX).read_text(encoding="utf-8")

    assert "Version Bot" not in text
    assert "github.actor" not in text


def test_sphinx_skips_pages_deploy_on_pull_request():
    doc, _ = _load(SPHINX)
    deploy = doc["jobs"]["deploy"]
    build_steps = doc["jobs"]["build"]["steps"]

    assert deploy["if"] == "github.event_name != 'pull_request'"
    upload = next(s for s in build_steps if "upload-pages-artifact" in str(s.get("uses", "")))
    assert upload["if"] == "github.event_name != 'pull_request'"


def test_tag_release_builds_released_docs_in_same_run():
    doc, _ = _load(TAG_RELEASE)
    docs = doc["jobs"]["docs"]

    assert docs["uses"] == "./.github/workflows/sphinxdoc.yaml"
    assert docs["needs"] == "tag"
    assert docs["if"] == "needs.tag.outputs.tag != 'tag exists'"
    assert docs["with"]["ref"] == "hybrid-devel"
    assert docs["permissions"] == {"contents": "read", "pages": "write", "id-token": "write"}


def test_tag_release_docs_ref_is_never_the_new_tag():
    doc, _ = _load(TAG_RELEASE)
    docs = doc["jobs"]["docs"]

    assert "needs.tag.outputs.tag" not in docs["with"]["ref"]
    assert docs["with"]["ref"] == "hybrid-devel"
    # The tag job still fetches tags so the new tag reaches `make versioned`.
    assert doc["jobs"]["tag"]["steps"][0]["with"]["fetch-tags"] is True


def test_tag_release_exposes_tag_output_for_docs_job():
    doc, _ = _load(TAG_RELEASE)

    assert doc["jobs"]["tag"]["outputs"]["tag"] == "${{ steps.tag.outputs.tag }}"


PUSH_GATE_WORKFLOWS = ["format.yaml", "ci.ros1.yaml", "ci.ros2.yaml"]


@pytest.mark.parametrize("name", PUSH_GATE_WORKFLOWS)
def test_push_gate_base_loop_avoids_short_circuit_append(name):
    # `[[ -n x ]] && cmd` as a loop body is a hygiene hazard under `set -e`; use `if`.
    text = (WORKFLOWS / name).read_text(encoding="utf-8")

    assert not re.search(r"\[\[ -n .*\]\] && args\+=", text)
    assert 'if [[ -n "${base}" ]]; then args+=(--pr-base "${base}"); fi' in text


def test_docs_readme_states_tags_never_become_the_checkout():
    text = (VERSIONING_DIR.parent / "docs" / "README.md").read_text(encoding="utf-8")

    assert "is never the checkout" in text
    assert "ref: hybrid-devel" in text


def test_contributing_warns_against_pushing_after_version_bot_bump():
    text = (VERSIONING_DIR.parent / "CONTRIBUTING.md").read_text(encoding="utf-8")

    assert "After the Version Bot bump" in text
    assert "shipped code again" in text
