#!/usr/bin/env python3
"""Tests for normalized render performance receipt reporting."""

from __future__ import annotations

import json
import pathlib
import subprocess
import sys
import unittest

FIXTURES = pathlib.Path(__file__).with_name("render_performance_report_fixtures")
REPORT_SCRIPT = pathlib.Path(__file__).with_name("render_performance_report.py")


class RenderPerformanceReportTest(unittest.TestCase):
    def setUp(self) -> None:
        self.report = __import__("render_performance_report")

    def test_zero_regression(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_zero_regression_cam1.json")
        regressions = self.report.normalized_regressions(baseline, candidate)
        self.assertEqual(regressions["median_trial_renders_per_second_regression_pct"], 0.0)
        self.assertEqual(regressions["median_frame_latency_ms_regression_pct"], 0.0)
        self.assertEqual(regressions["median_physics_step_duration_ms_regression_pct"], 0.0)

    def test_improvement_has_negative_regression(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_improvement_cam1.json")
        regressions = self.report.normalized_regressions(baseline, candidate)
        self.assertLess(regressions["median_trial_renders_per_second_regression_pct"], 0.0)
        self.assertLess(regressions["median_frame_latency_ms_regression_pct"], 0.0)
        self.assertLess(regressions["median_physics_step_duration_ms_regression_pct"], 0.0)

    def test_throughput_regression(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(
            FIXTURES / "candidate_throughput_regression_cam1.json"
        )
        regressions = self.report.normalized_regressions(baseline, candidate)
        self.assertAlmostEqual(
            regressions["median_trial_renders_per_second_regression_pct"],
            10.0,
        )
        self.assertEqual(regressions["median_frame_latency_ms_regression_pct"], 0.0)
        self.assertEqual(regressions["median_physics_step_duration_ms_regression_pct"], 0.0)

    def test_latency_regression(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_latency_regression_cam1.json")
        regressions = self.report.normalized_regressions(baseline, candidate)
        self.assertAlmostEqual(regressions["median_frame_latency_ms_regression_pct"], 20.0)
        self.assertEqual(regressions["median_trial_renders_per_second_regression_pct"], 0.0)

    def test_physics_regression(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_physics_regression_cam1.json")
        regressions = self.report.normalized_regressions(baseline, candidate)
        self.assertAlmostEqual(regressions["median_physics_step_duration_ms_regression_pct"], 25.0)

    def test_malformed_receipt(self) -> None:
        path = FIXTURES / "malformed_receipt.json"
        with self.assertRaises(json.JSONDecodeError):
            self.report.load_receipt(path)

    def test_missing_metric(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_missing_metric_cam1.json")
        with self.assertRaises(self.report.ReceiptValidationError):
            self.report.normalized_regressions(baseline, candidate)

    def test_non_finite_metric_rejected(self) -> None:
        with self.assertRaises(self.report.ReceiptValidationError):
            self.report.load_receipt(FIXTURES / "candidate_non_finite_throughput_cam1.json")

    def test_zero_baseline(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_zero_throughput_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_zero_regression_cam1.json")
        with self.assertRaises(self.report.ReceiptValidationError):
            self.report.normalized_regressions(baseline, candidate)

    def test_mismatched_camera_metadata(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(FIXTURES / "candidate_mismatched_camera_cam2.json")
        with self.assertRaises(self.report.ReceiptValidationError):
            self.report.normalized_regressions(baseline, candidate)

    def test_mismatched_iteration_metadata(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(
            FIXTURES / "candidate_mismatched_iterations_cam1.json"
        )
        with self.assertRaises(self.report.ReceiptValidationError):
            self.report.normalized_regressions(baseline, candidate)

    def test_candidate_only_report_lists_metrics_without_regressions(self) -> None:
        candidate = self.report.load_receipt(FIXTURES / "candidate_zero_regression_cam1.json")
        payload = self.report.build_candidate_only_report(
            candidate,
            candidate_ref="cand",
            baseline_ref="base",
            reason="baseline lacks harness",
        )
        self.assertEqual(payload["mode"], "candidate_only")
        self.assertEqual(payload["regressions"], {})
        self.assertEqual(payload["metrics"]["median_trial_renders_per_second"], 100.0)
        markdown = self.report.render_candidate_only_markdown_report(payload)
        self.assertIn("candidate only", markdown.lower())
        self.assertIn("comparison skipped", markdown.lower())
        self.assertIn("100.000", markdown)

    def test_cli_candidate_only_exits_zero(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(REPORT_SCRIPT),
                "candidate-only",
                "--candidate",
                str(FIXTURES / "candidate_zero_regression_cam1.json"),
                "--baseline-ref",
                "base",
                "--candidate-ref",
                "cand",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0)

    def test_json_report_preserves_comparison_metadata(self) -> None:
        baseline = self.report.load_receipt(FIXTURES / "baseline_cam1.json")
        candidate = self.report.load_receipt(
            FIXTURES / "candidate_throughput_regression_cam1.json"
        )
        payload = self.report.build_comparison_report(
            baseline,
            candidate,
            baseline_ref="823eadfb",
            candidate_ref="a6894a8b",
        )
        self.assertEqual(payload["baseline_ref"], "823eadfb")
        self.assertEqual(payload["candidate_ref"], "a6894a8b")
        self.assertEqual(payload["camera_count"], 1)
        self.assertEqual(payload["iterations_per_trial"], 1000)
        self.assertIn("regressions", payload)
        self.assertEqual(payload["baseline_metrics"]["median_trial_renders_per_second"], 100.0)
        self.assertEqual(payload["candidate_metrics"]["median_trial_renders_per_second"], 90.0)

    def test_markdown_report_lists_raw_values_then_percentage_offsets(self) -> None:
        report = {
            "baseline_ref": "base-ref",
            "candidate_ref": "cand-ref",
            "camera_count": 1,
            "iterations_per_trial": 1000,
            "baseline_metrics": {
                "median_trial_renders_per_second": 100.0,
                "median_frame_latency_ms": 10.0,
                "median_physics_step_duration_ms": 2.0,
            },
            "candidate_metrics": {
                "median_trial_renders_per_second": 90.0,
                "median_frame_latency_ms": 12.0,
                "median_physics_step_duration_ms": 2.0,
            },
            "regressions": {
                "median_trial_renders_per_second_regression_pct": 10.0,
                "median_frame_latency_ms_regression_pct": 20.0,
                "median_physics_step_duration_ms_regression_pct": 0.0,
            },
        }
        markdown = self.report.render_markdown_report(report)
        self.assertIn("base-ref", markdown)
        self.assertIn("cand-ref", markdown)
        self.assertIn("## Raw medians", markdown)
        self.assertIn("| Baseline (B) | Candidate (C) |", markdown)
        self.assertIn("100.000", markdown)
        self.assertIn("90.000", markdown)
        self.assertIn("## Percentage offsets", markdown)
        self.assertIn("10.000", markdown)
        self.assertIn("20.000", markdown)

    def test_aggregate_comparison_markdown_includes_raw_and_offsets(self) -> None:
        import tempfile

        cam1 = {
            "baseline_ref": "base",
            "candidate_ref": "cand",
            "camera_count": 1,
            "iterations_per_trial": 1000,
            "baseline_metrics": {
                "median_trial_renders_per_second": 100.0,
                "median_frame_latency_ms": 10.0,
                "median_physics_step_duration_ms": 2.0,
            },
            "candidate_metrics": {
                "median_trial_renders_per_second": 90.0,
                "median_frame_latency_ms": 12.0,
                "median_physics_step_duration_ms": 2.5,
            },
            "regressions": {
                "median_trial_renders_per_second_regression_pct": 10.0,
                "median_frame_latency_ms_regression_pct": 20.0,
                "median_physics_step_duration_ms_regression_pct": 25.0,
            },
        }
        with tempfile.TemporaryDirectory() as tmp:
            report_path = pathlib.Path(tmp) / "cam1.json"
            md_out = pathlib.Path(tmp) / "summary.md"
            report_path.write_text(json.dumps(cam1), encoding="utf-8")
            result = subprocess.run(
                [
                    sys.executable,
                    str(REPORT_SCRIPT),
                    "aggregate",
                    str(report_path),
                    "--markdown-out",
                    str(md_out),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(result.returncode, 0, result.stderr)
            markdown = md_out.read_text(encoding="utf-8")
            self.assertIn("## Raw medians", markdown)
            self.assertIn("## Percentage offsets", markdown)
            self.assertIn("100.000", markdown)
            self.assertIn("90.000", markdown)
            self.assertIn("10.000", markdown)

    def test_cli_compare_exits_nonzero_on_receipt_failure(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(REPORT_SCRIPT),
                "compare",
                "--baseline",
                str(FIXTURES / "baseline_cam1.json"),
                "--candidate",
                str(FIXTURES / "candidate_missing_metric_cam1.json"),
                "--baseline-ref",
                "base",
                "--candidate-ref",
                "cand",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("missing", result.stderr.lower())

    def test_cli_compare_exits_nonzero_on_non_finite_receipt(self) -> None:
        result = subprocess.run(
            [
                sys.executable,
                str(REPORT_SCRIPT),
                "compare",
                "--baseline",
                str(FIXTURES / "baseline_cam1.json"),
                "--candidate",
                str(FIXTURES / "candidate_non_finite_throughput_cam1.json"),
                "--baseline-ref",
                "base",
                "--candidate-ref",
                "cand",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("finite", result.stderr.lower())


if __name__ == "__main__":
    unittest.main()
