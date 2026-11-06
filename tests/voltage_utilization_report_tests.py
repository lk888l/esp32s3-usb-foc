"""Offline checks for latch association and normalized voltage evidence."""
import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import voltage_utilization_report as report


def snapshot():
    return {"diagnostic": {"stage": 2}, "current_valid": True, "adc_sample_valid": True,
            "pwm_compare_valid": True, "pwm_half_period_ticks": 2000,
            "pwm_resolution_hz": 80_000_000, "bus_voltage": 12, "pwm_duty_ceiling": .65,
            "adc_completed_mask": 7, "adc_done_mask": 7, "adc_window_after_mask": 7,
            "adc_pwm_compare_before": [[900, 2000, 1800], [700, 2000, 1350]],
            "adc_pwm_compare_after": [[700, 2000, 1350], [700, 2000, 1350]],
            "adc_current_words": [3 << 13, 4 << 13], "adc_trigger_ticks": [-180, -180],
            "adc_armed_words": [3 << 13, 4 << 13], "adc_done_words": [4 << 13, 5 << 13],
            "adc_trigger_elapsed_us": [0, 50], "adc_done_elapsed_us": [9, 59],
            "adc_done_ticks": [560, 560], "voltage_d": .65 * 12 / math.sqrt(3), "voltage_q": 0}


class VoltageEvidenceTests(unittest.TestCase):
    def test_normalization_and_common_mode(self):
        self.assertAlmostEqual(report.utilization([0, .5, 1]), 1)
        self.assertAlmostEqual(report.utilization([0, .325, .65]), .65)
        self.assertAlmostEqual(report.utilization([.1, .425, .75]), .65)
        self.assertEqual(report.utilization([.6, .6, .6]), 0)

    def test_uses_coherent_carrier_not_previous(self):
        sample = snapshot()
        item = report.observation(sample)
        self.assertAlmostEqual(item["latched_utilization"], .65)
        sample["adc_pwm_compare_before"][0] = [2000, 2000, 2000]
        self.assertAlmostEqual(report.observation(sample)["latched_utilization"], .65)

    def test_actual_falling_and_rising_phase_bounds(self):
        item = report.observation(snapshot())
        self.assertEqual(item["falling_margin_us"], [3, 16.75])
        self.assertEqual(item["rising_margin_us"], [.75, 17])
        self.assertEqual(item["start_to_done_observed_us"], [9.25, 9.25])
        sample = snapshot()
        sample["adc_current_words"] = [5 << 13, 3 << 13]
        item = report.observation(sample)
        self.assertEqual(item["falling_margin_us"], [14.25, .5])
        self.assertEqual(item["rising_margin_us"], [8.875, .75])

    def test_rejects_invalid_join_and_missing_evidence(self):
        for mutate in (
                lambda s: s.update(current_valid=False),
                lambda s: s.update(diagnostic=None),
                lambda s: s.update(adc_done_mask=1),
                lambda s: s.update(adc_current_words=[3 << 13, 3 << 13]),
                lambda s: s.update(adc_current_words=[2 << 13, 4 << 13]),
                lambda s: s.update(pwm_resolution_hz=40_000_000),
                lambda s: s.update(bus_voltage=float("nan")),
                lambda s: s["adc_pwm_compare_before"][1].__setitem__(0, 701),
                lambda s: s["adc_done_ticks"].__setitem__(0, 620),
                lambda s: s["adc_trigger_ticks"].__setitem__(0, -421)):
            sample = snapshot()
            mutate(sample)
            self.assertIsNone(report.observation(sample))

    def test_empty_evidence_is_null(self):
        result = report.cohort([{"diagnostic": {"stage": 2}}])
        self.assertEqual(result["valid_latch_snapshots"], 0)
        self.assertIsNone(result["latched_utilization"]["max"])
        self.assertIsNone(result["latched_utilization_at_least_98pct_ceiling"]["fraction"])

    def test_rejected_out_of_window_done_is_retained_only_in_timing(self):
        good, rejected = snapshot(), snapshot()
        rejected.update(adc_sample_valid=False, current_valid=False)
        rejected["adc_done_ticks"][0] = 700
        rejected["adc_done_elapsed_us"][0] = 11
        result = report.timing_summary([good, rejected])
        self.assertEqual(result["all_adc_snapshots"]["available_snapshots"], 2)
        self.assertEqual(result["all_adc_snapshots"]["done_tick"]["max"], 700)
        self.assertEqual(result["accepted_adc_snapshots"]["done_tick"]["max"], 560)
        failed = result["rejected_adc_snapshots"]
        self.assertEqual(failed["available_snapshots"], 1)
        self.assertEqual(failed["rising_margin_us"]["min"], -1)
        self.assertEqual(failed["out_of_window_snapshots"], 1)
        self.assertEqual(failed["rising_window_violation_slots"], 1)
        self.assertEqual(report.cohort([good, rejected])["valid_latch_snapshots"], 1)
        self.assertNotIn("latched_utilization", failed)

    def test_missing_or_mismatched_done_is_unavailable_not_silently_dropped(self):
        for mutate in (lambda s: s.update(adc_done_mask=1),
                       lambda s: s.pop("adc_done_words"),
                       lambda s: s["adc_done_words"].__setitem__(0, 17),
                       lambda s: s["adc_armed_words"].__setitem__(0, 5 << 13),
                       lambda s: s["adc_done_elapsed_us"].__setitem__(0, 76),
                       lambda s: s.update(pwm_compare_valid=False)):
            sample = snapshot()
            sample.update(adc_sample_valid=False, current_valid=False)
            mutate(sample)
            result = report.timing_summary([sample])["rejected_adc_snapshots"]
            self.assertEqual(result["snapshots"], 1)
            self.assertEqual(result["unavailable_snapshots"], 1)
            self.assertEqual(sum(result["unavailable_reasons"].values()), 1)
            self.assertIsNone(result["done_tick"]["max"])

    def test_timing_retains_early_start_and_strict_rising_boundary(self):
        sample = snapshot()
        sample.update(adc_sample_valid=False, current_valid=False)
        sample["adc_trigger_ticks"][0] = -421
        sample["adc_done_ticks"][0] = 620
        sample["adc_done_elapsed_us"][0] = 13
        result = report.timing_summary([sample])["rejected_adc_snapshots"]
        self.assertEqual(result["falling_margin_us"]["min"], -.0125)
        self.assertEqual(result["rising_margin_us"]["min"], 0)
        self.assertEqual(result["falling_window_violation_slots"], 1)
        self.assertEqual(result["rising_window_violation_slots"], 1)

    def test_control_rejection_does_not_hide_adc_timing(self):
        sample = snapshot()
        sample["current_valid"] = False
        result = report.timing_summary([sample])
        self.assertEqual(result["accepted_adc_snapshots"]["available_snapshots"], 1)
        self.assertEqual(result["rejected_adc_snapshots"]["snapshots"], 0)

    def test_tail_and_failed_source_preserved(self):
        samples = []
        for t in (0, 300, 600, 900):
            sample = snapshot()
            sample["uptime_ms"] = t
            samples.append(sample)
        result = report.summarize_report({"passed": False, "error": "stopped", "runs": [
            {"drive_ms": 1000, "samples": samples}]})
        self.assertFalse(result["source_passed"])
        self.assertEqual(result["source_error"], "stopped")
        self.assertEqual(result["runs"][0]["tail_half_drive"]["valid_latch_snapshots"], 2)

    def test_counter_reset_not_a_successful_rate(self):
        first, last = {}, {}
        for k in ("control_cycles", "rejected_samples", "sampling_recoveries", "missed_samples",
                  "reused_samples", "encoder_timing_skips", "pwm_deferred"):
            first[k] = last[k] = 0
        first.update(uptime_ms=0, control_cycles=100)
        last.update(uptime_ms=1000, control_cycles=99)
        self.assertFalse(report.counter_summary([first, last])["valid"])
        last["control_cycles"] = 5100
        self.assertEqual(report.counter_summary([first, last])["effective_control_hz"], 5000)

    def test_bad_input(self):
        for values in ([0, 1], [0, 0, float("nan")], [0, 0, 1.1]):
            with self.assertRaises(ValueError):
                report.utilization(values)
        with self.assertRaises(ValueError):
            report.summarize_report({})


if __name__ == "__main__":
    unittest.main()
