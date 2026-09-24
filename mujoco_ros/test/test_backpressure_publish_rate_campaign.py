#!/usr/bin/env python3
"""Unit tests for step-limited backpressure publish-rate campaign receipts."""

from __future__ import annotations

import pathlib
import unittest

import backpressure_publish_rate_campaign as campaign

FIXTURES = pathlib.Path(__file__).with_name("backpressure_publish_rate_fixtures")


class BackpressurePublishRateCampaignTest(unittest.TestCase):
    def test_discrete_expected_matches_inclusive_period_gate(self) -> None:
        self.assertEqual(
            campaign.discrete_expected_publish_count_per_camera(1000, 0.001, 1000.0),
            1000,
        )
        self.assertEqual(
            campaign.discrete_expected_publish_count_per_camera(1000, 0.001, 100.0),
            100,
        )

    def test_validate_drop_receipt(self) -> None:
        receipt = campaign.load_receipt(FIXTURES / "drop_cam1.json")
        campaign.validate_receipt(receipt)
        self.assertEqual(receipt["render_backpressure_policy"], "drop")
        self.assertLessEqual(receipt["publish_count_delta"], receipt["expected_publish_count"])

    def test_validate_wait_receipt(self) -> None:
        receipt = campaign.load_receipt(FIXTURES / "wait_for_slot_cam1.json")
        campaign.validate_receipt(receipt)
        self.assertEqual(receipt["render_backpressure_policy"], "wait_for_slot")

    def test_summarize_campaign_requires_drop_below_expected(self) -> None:
        drop = campaign.load_receipt(FIXTURES / "drop_cam1.json")
        wait = campaign.load_receipt(FIXTURES / "wait_for_slot_cam1.json")
        summary = campaign.summarize_campaign(drop, wait)
        self.assertEqual(summary["expected_publish_count"], 1000.0)
        self.assertLessEqual(summary["drop_count"], summary["expected_publish_count"])
        self.assertGreaterEqual(summary["drop_hz"], 0.0)
        self.assertGreaterEqual(summary["wait_hz"], 0.0)

    def test_summarize_rejects_drop_exceeding_expected(self) -> None:
        drop = campaign.load_receipt(FIXTURES / "drop_cam1.json")
        wait = campaign.load_receipt(FIXTURES / "wait_for_slot_cam1.json")
        drop = dict(drop)
        drop["publish_count_delta"] = drop["expected_publish_count"] + 1
        with self.assertRaises(ValueError):
            campaign.summarize_campaign(drop, wait)


if __name__ == "__main__":
    unittest.main()
