"""The sampled carrier, rather than a previous command, must prove an endpoint."""
import copy
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from stall_window_check import latched_peak_duty


class LatchEvidenceTests(unittest.TestCase):
    def snapshot(self):
        return {"diagnostic": {"stage": 2}, "current_valid": True,
                "adc_sample_valid": True, "pwm_half_period_ticks": 2666,
                "adc_completed_mask": 7, "adc_done_mask": 7, "adc_window_after_mask": 7,
                "adc_pwm_compare_before": [[569, 2666, 1800], [900, 2666, 1800]],
                "adc_pwm_compare_after": [[900, 2666, 1800], [900, 2666, 1800]]}

    def test_old_high_command_is_not_endpoint_evidence(self):
        self.assertAlmostEqual(latched_peak_duty(self.snapshot()), 1 - 900 / 2666)

    def test_actual_endpoint_requires_coherent_carrier(self):
        snapshot = copy.deepcopy(self.snapshot())
        for matrix, row in (("adc_pwm_compare_after", 0), ("adc_pwm_compare_before", 1),
                            ("adc_pwm_compare_after", 1)):
            snapshot[matrix][row][0] = 569
        self.assertAlmostEqual(latched_peak_duty(snapshot), 1 - 569 / 2666)
        snapshot["adc_pwm_compare_before"][1][0] = 570
        self.assertIsNone(latched_peak_duty(snapshot))

    def test_rejected_current_is_not_voltage_evidence(self):
        snapshot = self.snapshot()
        snapshot["adc_sample_valid"] = False
        self.assertIsNone(latched_peak_duty(snapshot))


if __name__ == "__main__":
    unittest.main()
