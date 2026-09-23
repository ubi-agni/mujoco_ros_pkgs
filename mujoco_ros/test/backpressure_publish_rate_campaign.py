#!/usr/bin/env python3
"""Helpers and receipt validation for step-limited backpressure publish campaigns."""

from __future__ import annotations

import json
import pathlib
from typing import Any

REQUIRED_KEYS = {
    "measurement",
    "backend",
    "render_backpressure_policy",
    "camera_count",
    "steps",
    "timestep",
    "camera_frequency_hz",
    "expected_publish_count",
    "real_time_index",
    "unbound",
    "aggregate_publish_rate_hz",
    "per_camera_publish_rate_hz",
    "publish_count_delta",
}


def discrete_expected_publish_count_per_camera(
    steps: int, timestep: float, frequency_hz: float
) -> int:
    period_steps = 1.0 / (frequency_hz * timestep)
    count = 0
    last_step = -1
    for step in range(steps):
        if count == 0 or (step - last_step) >= period_steps:
            count += 1
            last_step = step
    return count


def discrete_expected_publish_count(
    steps: int, timestep: float, frequency_hz: float, camera_count: int
) -> int:
    return discrete_expected_publish_count_per_camera(steps, timestep, frequency_hz) * camera_count


def load_receipt(path: pathlib.Path | str) -> dict[str, Any]:
    with open(path, encoding="utf-8") as handle:
        payload = json.load(handle)
    if not isinstance(payload, dict):
        raise ValueError("receipt must be a JSON object")
    return payload


def validate_receipt(payload: dict[str, Any]) -> None:
    missing = REQUIRED_KEYS - payload.keys()
    if missing:
        raise ValueError(f"missing receipt keys: {sorted(missing)}")
    if payload["measurement"] != "step_limited_camera_image_publish_rate":
        raise ValueError("unexpected measurement name")
    if payload["render_backpressure_policy"] not in {"drop", "wait_for_slot"}:
        raise ValueError("invalid render_backpressure_policy")
    if not payload["unbound"]:
        raise ValueError("campaign receipt must record unbound realtime")
    if payload["real_time_index"] != 0:
        raise ValueError("campaign receipt must record real_time_index 0")
    if int(payload["steps"]) < 1:
        raise ValueError("steps must be positive")
    if float(payload["timestep"]) <= 0.0:
        raise ValueError("timestep must be positive")
    expected = int(payload["expected_publish_count"])
    if expected < 1:
        raise ValueError("expected_publish_count must be positive")
    if int(payload["publish_count_delta"]) < 0:
        raise ValueError("publish_count_delta must be non-negative")
    if float(payload["aggregate_publish_rate_hz"]) < 0.0:
        raise ValueError("aggregate_publish_rate_hz must be non-negative")


def summarize_campaign(
    drop_receipt: dict[str, Any], wait_receipt: dict[str, Any]
) -> dict[str, float]:
    validate_receipt(drop_receipt)
    validate_receipt(wait_receipt)
    expected = float(drop_receipt["expected_publish_count"])
    drop_count = float(drop_receipt["publish_count_delta"])
    wait_count = float(wait_receipt["publish_count_delta"])
    drop_hz = float(drop_receipt["aggregate_publish_rate_hz"])
    wait_hz = float(wait_receipt["aggregate_publish_rate_hz"])
    # With paused Step(N), expected_publish_count mirrors ShouldPublishAtTimeLocked
    # discrete gating (inclusive period comparison), not the naive steps*cameras budget.
    if drop_count > expected:
        raise ValueError(
            "drop publish_count_delta must not exceed expected_publish_count "
            f"(got {drop_count} > {expected})"
        )
    return {
        "drop_hz": drop_hz,
        "wait_hz": wait_hz,
        "drop_count": drop_count,
        "wait_count": wait_count,
        "expected_publish_count": expected,
    }
