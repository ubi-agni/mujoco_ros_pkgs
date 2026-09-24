#!/usr/bin/env python3
"""Merge single-trial render_performance_test JSON receipts into one gate receipt."""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path
from typing import Any


def median(values: list[float]) -> float:
    if not values:
        raise ValueError("cannot compute median without samples")
    return statistics.median(values)


def load_receipt(path: Path) -> dict[str, Any]:
    with path.open(encoding="utf-8") as handle:
        return json.load(handle)


def merge_receipts(paths: list[Path]) -> dict[str, Any]:
    if not paths:
        raise ValueError("at least one trial receipt is required")

    trials: list[dict[str, Any]] = []
    total_renders = 0.0
    total_elapsed_ms = 0.0

    base = load_receipt(paths[0])
    for index, path in enumerate(paths, start=1):
        receipt = load_receipt(path)
        if receipt.get("iterations_per_trial") != base.get("iterations_per_trial"):
            raise ValueError(f"{path} iterations_per_trial mismatch")
        if receipt.get("camera_count") != base.get("camera_count"):
            raise ValueError(f"{path} camera_count mismatch")
        if len(receipt.get("trials", [])) != 1:
            raise ValueError(f"{path} must contain exactly one trial")

        trial = dict(receipt["trials"][0])
        trial["index"] = index
        if "run_metadata" in receipt:
            trial["run_metadata"] = receipt["run_metadata"]
        if "gpu_state" in receipt:
            trial["gpu_state"] = receipt["gpu_state"]
        trials.append(trial)

        total_renders += float(trial["render_count"])
        total_elapsed_ms += float(trial["elapsed_wall_time_ms"])

    span_fields = [
        "enqueue_to_worker_ms",
        "worker_to_publish_entry_ms",
        "publish_entry_to_return_ms",
        "publish_return_to_receipt_ms",
        "enqueue_to_receipt_ms",
    ]
    trial_throughputs = [float(trial["renders_per_second"]) for trial in trials]
    trial_frame_latencies = [float(trial["median_frame_latency_ms"]) for trial in trials]
    trial_physics_steps = [float(trial["median_physics_step_duration_ms"]) for trial in trials]
    trial_wrapped_steps = [float(trial["median_wrapped_step_ms"]) for trial in trials]
    trial_post_step = [float(trial["median_post_step_to_receipt_ms"]) for trial in trials]

    attribution = dict(base.get("attribution", {}))
    attribution["median_wrapped_step_ms"] = median(trial_wrapped_steps)
    attribution["median_post_step_to_receipt_ms"] = median(trial_post_step)
    for field in span_fields:
        values = [
            float(trial[f"median_{field}"])
            for trial in trials
            if trial.get(f"median_{field}") is not None
        ]
        attribution[f"median_{field}"] = median(values) if values else None

    merged = {
        key: value
        for key, value in base.items()
        if key
        not in {
            "trials",
            "trial_count",
            "render_count",
            "elapsed_wall_time_ms",
            "renders_per_second",
            "median_trial_renders_per_second",
            "median_frame_latency_ms",
            "median_physics_step_duration_ms",
            "frame_latency_samples",
            "physics_step_samples",
            "attribution",
            "run_metadata",
        }
    }
    merged["trial_count"] = len(trials)
    merged["render_count"] = total_renders
    merged["elapsed_wall_time_ms"] = total_elapsed_ms
    merged["renders_per_second"] = total_renders / (total_elapsed_ms / 1000.0)
    merged["median_trial_renders_per_second"] = median(trial_throughputs)
    merged["median_frame_latency_ms"] = median(trial_frame_latencies)
    merged["median_physics_step_duration_ms"] = median(trial_physics_steps)
    merged["frame_latency_samples"] = int(base["iterations_per_trial"]) * len(trials)
    merged["physics_step_samples"] = int(base["iterations_per_trial"]) * len(trials)
    merged["attribution"] = attribution
    merged["trials"] = trials
    return merged


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    merged = merge_receipts(args.inputs)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", encoding="utf-8") as handle:
        json.dump(merged, handle, indent=2)
        handle.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
