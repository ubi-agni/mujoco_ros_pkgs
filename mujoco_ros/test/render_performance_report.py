#!/usr/bin/env python3
"""Normalize merged render performance receipts into advisory regression reports."""

from __future__ import annotations

import argparse
import json
import math
import pathlib
import sys
from typing import Any


class ReceiptValidationError(Exception):
    """Raised when a receipt or comparison violates reporting preconditions."""


REQUIRED_METRICS = (
    "median_trial_renders_per_second",
    "median_frame_latency_ms",
    "median_physics_step_duration_ms",
)


def _reject_non_finite_values(value: object, path_label: str) -> None:
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ReceiptValidationError(f"{path_label}: non-finite numeric value {value!r}")
    elif isinstance(value, dict):
        for key, item in value.items():
            _reject_non_finite_values(item, f"{path_label}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _reject_non_finite_values(item, f"{path_label}[{index}]")


def load_receipt(path: pathlib.Path) -> dict[str, object]:
    with path.open(encoding="utf-8") as handle:
        data = json.load(handle)
    if not isinstance(data, dict):
        raise ReceiptValidationError(f"{path}: receipt root must be a JSON object")
    _reject_non_finite_values(data, str(path))
    return data


def _require_int_field(receipt: dict[str, object], field: str, label: str) -> int:
    if field not in receipt:
        raise ReceiptValidationError(f"{label}: missing required field {field!r}")
    value = receipt[field]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ReceiptValidationError(f"{label}: field {field!r} must be an integer")
    return value


def _require_numeric_metric(receipt: dict[str, object], field: str, label: str) -> float:
    if field not in receipt:
        raise ReceiptValidationError(f"{label}: missing required metric {field!r}")
    value = receipt[field]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ReceiptValidationError(f"{label}: metric {field!r} must be numeric")
    numeric = float(value)
    if not math.isfinite(numeric):
        raise ReceiptValidationError(f"{label}: metric {field!r} must be finite")
    return numeric


def normalized_regressions(
    baseline: dict[str, object],
    candidate: dict[str, object],
) -> dict[str, float]:
    baseline_camera_count = _require_int_field(baseline, "camera_count", "baseline")
    candidate_camera_count = _require_int_field(candidate, "camera_count", "candidate")
    if baseline_camera_count != candidate_camera_count:
        raise ReceiptValidationError(
            "camera_count mismatch: "
            f"baseline={baseline_camera_count}, candidate={candidate_camera_count}"
        )

    baseline_iterations = _require_int_field(baseline, "iterations_per_trial", "baseline")
    candidate_iterations = _require_int_field(candidate, "iterations_per_trial", "candidate")
    if baseline_iterations != candidate_iterations:
        raise ReceiptValidationError(
            "iterations_per_trial mismatch: "
            f"baseline={baseline_iterations}, candidate={candidate_iterations}"
        )

    baseline_throughput = _require_numeric_metric(
        baseline, "median_trial_renders_per_second", "baseline"
    )
    candidate_throughput = _require_numeric_metric(
        candidate, "median_trial_renders_per_second", "candidate"
    )
    baseline_frame_latency = _require_numeric_metric(
        baseline, "median_frame_latency_ms", "baseline"
    )
    candidate_frame_latency = _require_numeric_metric(
        candidate, "median_frame_latency_ms", "candidate"
    )
    baseline_physics = _require_numeric_metric(
        baseline, "median_physics_step_duration_ms", "baseline"
    )
    candidate_physics = _require_numeric_metric(
        candidate, "median_physics_step_duration_ms", "candidate"
    )

    if baseline_throughput == 0.0:
        raise ReceiptValidationError("baseline median_trial_renders_per_second must be non-zero")
    if baseline_frame_latency == 0.0:
        raise ReceiptValidationError("baseline median_frame_latency_ms must be non-zero")
    if baseline_physics == 0.0:
        raise ReceiptValidationError("baseline median_physics_step_duration_ms must be non-zero")

    return {
        "median_trial_renders_per_second_regression_pct": (
            (baseline_throughput - candidate_throughput) / baseline_throughput * 100.0
        ),
        "median_frame_latency_ms_regression_pct": (
            (candidate_frame_latency - baseline_frame_latency) / baseline_frame_latency * 100.0
        ),
        "median_physics_step_duration_ms_regression_pct": (
            (candidate_physics - baseline_physics) / baseline_physics * 100.0
        ),
    }


def render_markdown_report(report: dict[str, Any]) -> str:
    baseline_metrics = report["baseline_metrics"]
    candidate_metrics = report["candidate_metrics"]
    regressions = report["regressions"]
    lines = [
        "# Render performance advisory report",
        "",
        f"- Baseline: `{report['baseline_ref']}`",
        f"- Candidate: `{report['candidate_ref']}`",
        f"- Cameras: {report['camera_count']}",
        f"- Iterations/trial: {report['iterations_per_trial']}",
        "",
        "## Raw medians",
        "",
        "| Metric | Baseline (B) | Candidate (C) |",
        "| --- | ---: | ---: |",
        (
            f"| Throughput (renders/sec) | "
            f"{baseline_metrics['median_trial_renders_per_second']:.3f} | "
            f"{candidate_metrics['median_trial_renders_per_second']:.3f} |"
        ),
        (
            f"| Frame latency (ms) | "
            f"{baseline_metrics['median_frame_latency_ms']:.3f} | "
            f"{candidate_metrics['median_frame_latency_ms']:.3f} |"
        ),
        (
            f"| Physics step duration (ms) | "
            f"{baseline_metrics['median_physics_step_duration_ms']:.3f} | "
            f"{candidate_metrics['median_physics_step_duration_ms']:.3f} |"
        ),
        "",
        "## Percentage offsets (C vs B)",
        "",
        "| Metric | Offset % |",
        "| --- | ---: |",
        (
            f"| Throughput (renders/sec) | "
            f"{regressions['median_trial_renders_per_second_regression_pct']:.3f} |"
        ),
        (
            f"| Frame latency (ms) | "
            f"{regressions['median_frame_latency_ms_regression_pct']:.3f} |"
        ),
        (
            f"| Physics step duration (ms) | "
            f"{regressions['median_physics_step_duration_ms_regression_pct']:.3f} |"
        ),
        "",
        "Positive offset % = advisory regression "
        "(lower throughput or higher latency/physics duration). "
        "CI stays green unless execution or receipt validation fails.",
    ]
    return "\n".join(lines) + "\n"


def build_comparison_report(
    baseline: dict[str, object],
    candidate: dict[str, object],
    baseline_ref: str,
    candidate_ref: str,
) -> dict[str, Any]:
    regressions = normalized_regressions(baseline, candidate)
    baseline_metrics = {
        field: _require_numeric_metric(baseline, field, "baseline") for field in REQUIRED_METRICS
    }
    candidate_metrics = {
        field: _require_numeric_metric(candidate, field, "candidate") for field in REQUIRED_METRICS
    }
    return {
        "baseline_ref": baseline_ref,
        "candidate_ref": candidate_ref,
        "camera_count": _require_int_field(baseline, "camera_count", "baseline"),
        "iterations_per_trial": _require_int_field(baseline, "iterations_per_trial", "baseline"),
        "baseline_metrics": baseline_metrics,
        "candidate_metrics": candidate_metrics,
        "regressions": regressions,
    }


def emit_github_warnings(report: dict[str, Any]) -> None:
    if report.get("mode") == "candidate_only":
        return
    camera_count = report["camera_count"]
    regressions = report["regressions"]
    for metric, value in regressions.items():
        if value > 0.0:
            print(
                f"::warning title=Render performance advisory (cam{camera_count})::"
                f"{metric}={value:.3f}% (baseline={report['baseline_ref']}, "
                f"candidate={report['candidate_ref']})"
            )


def build_candidate_only_report(
    candidate: dict[str, object],
    *,
    candidate_ref: str,
    baseline_ref: str,
    reason: str,
) -> dict[str, Any]:
    metrics = {
        field: _require_numeric_metric(candidate, field, "candidate") for field in REQUIRED_METRICS
    }
    return {
        "mode": "candidate_only",
        "baseline_ref": baseline_ref,
        "candidate_ref": candidate_ref,
        "camera_count": _require_int_field(candidate, "camera_count", "candidate"),
        "iterations_per_trial": _require_int_field(candidate, "iterations_per_trial", "candidate"),
        "reason": reason,
        "metrics": metrics,
        "regressions": {},
    }


def render_candidate_only_markdown_report(report: dict[str, Any]) -> str:
    metrics = report["metrics"]
    lines = [
        "# Render performance advisory report (candidate only)",
        "",
        f"- Baseline: `{report['baseline_ref']}` (no harness — comparison skipped)",
        f"- Candidate: `{report['candidate_ref']}`",
        f"- Reason: {report['reason']}",
        f"- Cameras: {report['camera_count']}",
        f"- Iterations/trial: {report['iterations_per_trial']}",
        "",
        "| Metric | Candidate median |",
        "| --- | ---: |",
        f"| Throughput (renders/sec) | {metrics['median_trial_renders_per_second']:.3f} |",
        f"| Frame latency (ms) | {metrics['median_frame_latency_ms']:.3f} |",
        f"| Physics step duration (ms) | {metrics['median_physics_step_duration_ms']:.3f} |",
        "",
        "No normalized regressions: baseline revision does not build `render_performance_test`.",
    ]
    return "\n".join(lines) + "\n"


def compare_command(args: argparse.Namespace) -> int:
    try:
        baseline = load_receipt(args.baseline)
        candidate = load_receipt(args.candidate)
        report = build_comparison_report(
            baseline,
            candidate,
            baseline_ref=args.baseline_ref,
            candidate_ref=args.candidate_ref,
        )
    except json.JSONDecodeError as exc:
        print(f"malformed JSON receipt: {exc}", file=sys.stderr)
        return 1
    except ReceiptValidationError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    markdown = render_markdown_report(report)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with args.json_out.open("w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")

    if args.markdown_out is not None:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(markdown, encoding="utf-8")

    if args.emit_github_warnings:
        emit_github_warnings(report)

    return 0


def candidate_only_command(args: argparse.Namespace) -> int:
    try:
        candidate = load_receipt(args.candidate)
        report = build_candidate_only_report(
            candidate,
            candidate_ref=args.candidate_ref,
            baseline_ref=args.baseline_ref,
            reason=args.reason,
        )
    except json.JSONDecodeError as exc:
        print(f"malformed JSON receipt: {exc}", file=sys.stderr)
        return 1
    except ReceiptValidationError as exc:
        print(str(exc), file=sys.stderr)
        return 1

    markdown = render_candidate_only_markdown_report(report)

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with args.json_out.open("w", encoding="utf-8") as handle:
            json.dump(report, handle, indent=2)
            handle.write("\n")

    if args.markdown_out is not None:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(markdown, encoding="utf-8")

    return 0


def aggregate_command(args: argparse.Namespace) -> int:
    reports: list[dict[str, Any]] = []
    for report_path in args.reports:
        try:
            with report_path.open(encoding="utf-8") as handle:
                payload = json.load(handle)
        except json.JSONDecodeError as exc:
            print(f"malformed JSON report {report_path}: {exc}", file=sys.stderr)
            return 1
        if not isinstance(payload, dict):
            print(f"report root must be a JSON object: {report_path}", file=sys.stderr)
            return 1
        reports.append(payload)

    if not reports:
        print("at least one report is required", file=sys.stderr)
        return 1

    baseline_ref = reports[0]["baseline_ref"]
    candidate_ref = reports[0]["candidate_ref"]
    candidate_only = all(report.get("mode") == "candidate_only" for report in reports)
    if candidate_only != any(report.get("mode") == "candidate_only" for report in reports):
        print("cannot mix candidate-only and comparison reports in one aggregate", file=sys.stderr)
        return 1

    if candidate_only:
        reason = str(reports[0].get("reason", "baseline missing harness"))
        lines = [
            "# Render performance advisory summary (candidate only)",
            "",
            f"- Baseline: `{baseline_ref}` (no harness — comparison skipped)",
            f"- Candidate: `{candidate_ref}`",
            f"- Reason: {reason}",
            "",
            "| Cameras | Throughput | Frame latency (ms) | Physics step (ms) |",
            "| ---: | ---: | ---: | ---: |",
        ]
        for report in sorted(reports, key=lambda item: int(item["camera_count"])):
            metrics = report["metrics"]
            lines.append(
                "| {camera_count} | {throughput:.3f} | {latency:.3f} | {physics:.3f} |".format(
                    camera_count=report["camera_count"],
                    throughput=metrics["median_trial_renders_per_second"],
                    latency=metrics["median_frame_latency_ms"],
                    physics=metrics["median_physics_step_duration_ms"],
                )
            )
        lines.extend(
            [
                "",
                "Candidate-only run: no normalized regressions until the PR base builds "
                "`render_performance_test`.",
            ]
        )
    else:
        lines = [
            "# Render performance advisory summary",
            "",
            f"- Baseline: `{baseline_ref}`",
            f"- Candidate: `{candidate_ref}`",
            "",
            "## Raw medians",
            "",
            (
                "| Cameras | B throughput | C throughput | "
                "B latency (ms) | C latency (ms) | "
                "B physics (ms) | C physics (ms) |"
            ),
            "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
        ]
        for report in sorted(reports, key=lambda item: int(item["camera_count"])):
            baseline_metrics = report["baseline_metrics"]
            candidate_metrics = report["candidate_metrics"]
            lines.append(
                "| {camera_count} | {b_tp:.3f} | {c_tp:.3f} | "
                "{b_lat:.3f} | {c_lat:.3f} | {b_phys:.3f} | {c_phys:.3f} |".format(
                    camera_count=report["camera_count"],
                    b_tp=baseline_metrics["median_trial_renders_per_second"],
                    c_tp=candidate_metrics["median_trial_renders_per_second"],
                    b_lat=baseline_metrics["median_frame_latency_ms"],
                    c_lat=candidate_metrics["median_frame_latency_ms"],
                    b_phys=baseline_metrics["median_physics_step_duration_ms"],
                    c_phys=candidate_metrics["median_physics_step_duration_ms"],
                )
            )
        lines.extend(
            [
                "",
                "## Percentage offsets (C vs B)",
                "",
                "| Cameras | Throughput % | Frame latency % | Physics step % |",
                "| ---: | ---: | ---: | ---: |",
            ]
        )
        for report in sorted(reports, key=lambda item: int(item["camera_count"])):
            regressions = report["regressions"]
            lines.append(
                "| {camera_count} | {throughput:.3f} | {latency:.3f} | {physics:.3f} |".format(
                    camera_count=report["camera_count"],
                    throughput=regressions["median_trial_renders_per_second_regression_pct"],
                    latency=regressions["median_frame_latency_ms_regression_pct"],
                    physics=regressions["median_physics_step_duration_ms_regression_pct"],
                )
            )
        lines.extend(
            [
                "",
                "Positive offset % = advisory regression only; "
                "execution and receipt failures fail CI.",
            ]
        )
    markdown = "\n".join(lines) + "\n"

    if args.markdown_out is not None:
        args.markdown_out.parent.mkdir(parents=True, exist_ok=True)
        args.markdown_out.write_text(markdown, encoding="utf-8")

    if args.json_out is not None:
        args.json_out.parent.mkdir(parents=True, exist_ok=True)
        with args.json_out.open("w", encoding="utf-8") as handle:
            json.dump(
                {
                    "mode": "candidate_only" if candidate_only else "comparison",
                    "baseline_ref": baseline_ref,
                    "candidate_ref": candidate_ref,
                    "camera_reports": reports,
                },
                handle,
                indent=2,
            )
            handle.write("\n")

    if args.emit_github_warnings:
        for report in reports:
            emit_github_warnings(report)

    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    compare = subparsers.add_parser("compare", help="Compare one baseline/candidate receipt pair")
    compare.add_argument("--baseline", required=True, type=pathlib.Path)
    compare.add_argument("--candidate", required=True, type=pathlib.Path)
    compare.add_argument("--baseline-ref", required=True)
    compare.add_argument("--candidate-ref", required=True)
    compare.add_argument("--json-out", type=pathlib.Path)
    compare.add_argument("--markdown-out", type=pathlib.Path)
    compare.add_argument("--emit-github-warnings", action="store_true")
    compare.set_defaults(func=compare_command)

    candidate_only = subparsers.add_parser(
        "candidate-only",
        help="Report candidate metrics when baseline has no harness",
    )
    candidate_only.add_argument("--candidate", required=True, type=pathlib.Path)
    candidate_only.add_argument("--candidate-ref", required=True)
    candidate_only.add_argument("--baseline-ref", required=True)
    candidate_only.add_argument(
        "--reason",
        default="baseline revision does not build render_performance_test",
    )
    candidate_only.add_argument("--json-out", type=pathlib.Path)
    candidate_only.add_argument("--markdown-out", type=pathlib.Path)
    candidate_only.set_defaults(func=candidate_only_command)

    aggregate = subparsers.add_parser("aggregate", help="Aggregate per-camera comparison reports")
    aggregate.add_argument("reports", nargs="+", type=pathlib.Path)
    aggregate.add_argument("--json-out", type=pathlib.Path)
    aggregate.add_argument("--markdown-out", type=pathlib.Path)
    aggregate.add_argument("--emit-github-warnings", action="store_true")
    aggregate.set_defaults(func=aggregate_command)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
