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
