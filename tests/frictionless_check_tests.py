"""Offline acceptance/cleanup tests; never open a serial port."""
import argparse
import contextlib
import copy
import io
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

try:
    import serial  # noqa: F401
except ModuleNotFoundError:
    sys.modules["serial"] = types.ModuleType("serial")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import frictionless_check as check


class Clock:
    now = 0.

    @classmethod
    def monotonic(cls):
        return cls.now

    @classmethod
    def sleep(cls, seconds):
        cls.now += seconds


def trace(direction=1, stop=False, phase=2):
    rows = []
    angle = 0.
    for us in range(0, 3000001, 10000):
        velocity = direction * (0. if stop and us > 1500000 else 10.)
        angle += velocity * .01
        rows.append([us, angle, velocity, direction * .02, direction * .019, .001,
                     1., .1, .03, 2 if us < 1000000 else 3, velocity, .01,
                     2048, 2048, 1, 2, phase, direction * 10., .001, direction * .019])
    return rows


class FakeKnob:
    last = None
    timeout_on_config = False
    fault_during_run = False
    stop_after_launch = False
    self_start = False
    recovery = False
    boundary_notifications = False
    active_miss = False
    active_duplicate = False

    def __init__(self, port):
        FakeKnob.last = self
        self.serial = argparse.Namespace(timeout=None)
        self.commands = []
        self.config = {"mode": 1, "current_limit": .8, "damping": .003,
                       "frictionless": {"friction": .022, "viscous": .000304,
                                        "scale": 1., "hold_kp": .016, "hold_ki": .024}}
        self.calibration = {"valid": True, "direction": 1, "electrical_zero": .29, "pole_pairs": 7}
        self.state, self.fault = "idle", "none"
        self.busy = False
        self.stage = self.poll = 0
        self.time = 100000
        self.diag_start = None
        self.run_done = False
        self.timeout_sent = False
        self.closed = False
        self.unparsed_lines = []

    def snapshot(self):
        s = {"ok": True, "config": copy.deepcopy(self.config),
             "calibration": self.calibration.copy(), "calibration_quality": {"verified": True},
             "state": self.state, "fault": self.fault, "duty": [0, 0, 0] if self.state != "active" else [.01, 0, .01],
             "app_elf_sha256": "same", "uptime_ms": self.time,
             "diagnostic": {"busy": self.busy, "stage": self.stage, "count": 301 if self.run_done else 0},
             "angle": Clock.now if self.self_start and self.state == "active" else 1.,
             "velocity": 1. if self.self_start and self.state == "active" else 0.,
             "target": .02 if self.self_start and self.state == "active" else 0., "quiet_output": False,
             "adc_reject_counts": {key: 0 for key in check.REASONS}}
        s.update({key: 0 for key in check.COUNTERS})
        s["control_cycles"] = (self.time - 100000) * 5
        s["sampling_recoveries"] = 1 if self.recovery and self.run_done else 0
        if self.boundary_notifications and self.run_done:
            s["reused_samples"], s["missed_samples"] = 4, 5
        if self.diag_start is not None and Clock.now - self.diag_start >= 1.5:
            s["missed_samples"] += int(self.active_miss)
            s["reused_samples"] += int(self.active_duplicate)
        return s

    def request(self, op, config=None, **kwargs):
        self.commands.append(op)
        self.time = max(self.time + 1, 100000 + int(Clock.now * 1000))
        if op == "configure":
            self.config = copy.deepcopy(config)
            if self.timeout_on_config and not self.timeout_sent:
                self.timeout_sent = True
                raise TimeoutError("reply lost after configure applied")
        elif op == "diagnose":
            self.busy, self.stage, self.state = True, 2, "active"
            self.diag_start = Clock.now
        elif op == "status" and self.busy:
            self.poll += 1
            if self.fault_during_run:
                self.fault, self.state, self.stage, self.busy = "overcurrent", "fault", 5, False
            elif Clock.now - self.diag_start >= 3:
                self.busy, self.stage, self.state, self.run_done = False, 4, "idle", True
            elif Clock.now - self.diag_start >= 1:
                self.stage = 3
        elif op == "enable":
            self.state = "active"
        elif op == "disable":
            self.state = "fault" if self.fault != "none" else "idle"
            self.busy = False
        result = self.snapshot()
        if op == "diagnostic_read":
            offset = kwargs["offset"]
            result["diagnostic"].update(offset=offset, samples=trace(stop=self.stop_after_launch)[offset:offset+32])
        return result

    def wait_active(self):
        return self.snapshot()

    def close(self):
        self.request("disable")
        self.closed = True


class TraceTests(unittest.TestCase):
    def args(self, **changes):
        args = check.parse_args(["--output", "unused.json", "--drive-ms", "1000", "--observe-ms", "2000"])
        args.direction = 1
        for key, value in changes.items():
            setattr(args, key, value)
        return args

    def test_signed_hold_in_both_directions(self):
        for direction in (-1, 1):
            result = check.summarize_trace(trace(direction), self.args(velocity=10 * direction))
            self.assertTrue(result["hold_passed"], result)
            self.assertTrue(all(result["criteria"].values()))
            self.assertAlmostEqual(result["tail_speed"], 10 * direction)

    def test_stopped_rotor_does_not_pass_hold(self):
        result = check.summarize_trace(trace(stop=True), self.args())
        self.assertTrue(result["initial_motion_passed"])
        self.assertFalse(result["hold_passed"])
        self.assertFalse(result["hold_criteria"]["no_stop_or_reversal"])

    def test_coasting_control_does_not_require_hold_state(self):
        result = check.summarize_trace(trace(phase=0), self.args(mode=0))
        self.assertTrue(result["hold_passed"])
        result = check.summarize_trace(trace(phase=0), self.args(mode=16))
        self.assertFalse(result["hold_passed"])

    def test_truncated_release_is_not_accepted(self):
        result = check.summarize_trace(trace()[:160], self.args())
        self.assertFalse(result["criteria"]["release_captured"])
        self.assertFalse(result["hold_passed"])

    def test_duplicate_timestamp_is_rejected(self):
        rows = trace()
        rows[4][0] = rows[3][0]
        result = check.summarize_trace(rows, self.args())
        self.assertFalse(result["criteria"]["ordered_trace"])

    def test_nonfinite_samples_are_rejected(self):
        rows = trace()
        rows[0][2] = float("nan")
        self.assertFalse(check.summarize_trace(rows, self.args())["criteria"]["finite_trace"])

    def test_time_mean_uses_real_intervals(self):
        self.assertAlmostEqual(check.time_mean([[0, 0], [1, 2], [4, 2]], 1), 1.75)

    def test_cli_rejects_unbounded_input(self):
        for extra in (["--velocity", "nan"], ["--current", ".8"], ["--drive-ms", "3001"],
                      ["--runtime-current", ".81"], ["--runtime-current", ".05"],
                      ["--rest-seconds", "31"], ["--require-hold", "--rest-seconds", "1"]):
            with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
                check.parse_args(["--output", "unused.json", *extra])


class ExecutionTests(unittest.TestCase):
    def setUp(self):
        Clock.now = 0
        for name in ("timeout_on_config", "fault_during_run", "stop_after_launch", "self_start", "recovery",
                     "boundary_notifications", "active_miss", "active_duplicate"):
            setattr(FakeKnob, name, False)
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)

    def run_fake(self, extra=()):
        args = check.parse_args(["--output", str(Path(self.folder.name) / "report.json"),
                                "--drive-ms", "1000", "--observe-ms", "2000", *extra])
        with patch.object(check, "Knob", FakeKnob), patch.object(check.time, "monotonic", Clock.monotonic), \
                patch.object(check.time, "sleep", Clock.sleep), contextlib.redirect_stdout(io.StringIO()):
            report = check.run_check(args)
        self.assertTrue(FakeKnob.last.closed)
        self.assertNotIn("save", FakeKnob.last.commands)
        self.assertNotIn("clear", FakeKnob.last.commands)
        self.assertEqual(report["final"]["duty"], [0, 0, 0])
        self.assertEqual(report["final"]["config"], report["initial"]["config"])
        self.assertEqual(json.loads(args.output.read_text())["passed"], report["passed"])
        return report

    def test_success_and_restore(self):
        report = self.run_fake(["--require-hold"])
        self.assertTrue(report["passed"], report)
        self.assertTrue(report["hold_passed"])
        commands = FakeKnob.last.commands
        self.assertEqual(commands[commands.index("configure", 3) - 1], "disable")

    def test_hold_failure_is_explicit_even_when_execution_passes(self):
        FakeKnob.stop_after_launch = True
        report = self.run_fake()
        self.assertTrue(report["passed"], report)
        self.assertFalse(report["hold_passed"])

    def test_require_hold_fails_on_stall(self):
        FakeKnob.stop_after_launch = True
        self.assertFalse(self.run_fake(["--require-hold"])["passed"])

    def test_config_reply_loss_still_restores(self):
        FakeKnob.timeout_on_config = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertTrue(report["cleanup_passed"])
        self.assertIn("reply lost", report["error"])

    def test_fault_is_preserved_and_never_cleared(self):
        FakeKnob.fault_during_run = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(report["final"]["fault"], "overcurrent")

    def test_sampling_recovery_prevents_pass(self):
        FakeKnob.recovery = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertTrue(report["hold_passed"])
        self.assertFalse(report["sampling"]["full_interval_guards"]["feedback_uninterrupted"])

    def test_stop_notification_counts_remain_in_full_evidence(self):
        FakeKnob.boundary_notifications = True
        report = self.run_fake()
        self.assertTrue(report["passed"], report)
        self.assertFalse(report["sampling_full"]["passed"])
        self.assertEqual(report["sampling_full"]["counter_delta"]["missed_samples"], 5)
        self.assertEqual(report["sampling"]["counter_delta"]["missed_samples"], 0)
        self.assertEqual(report["sampling"]["outside_observed_window_counter_delta"]["reused_samples"], 4)
        self.assertGreaterEqual(report["sampling"]["observed_fraction"], .85)
        self.assertGreater(len(report["status_samples"]), 5)

    def test_real_active_missed_sample_still_fails(self):
        FakeKnob.boundary_notifications = FakeKnob.active_miss = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(report["sampling"]["counter_delta"]["missed_samples"], 1)
        self.assertFalse(report["sampling"]["criteria"]["no_missing_feedback"])

    def test_real_active_duplicate_notification_still_fails(self):
        FakeKnob.active_duplicate = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertFalse(report["sampling"]["criteria"]["no_duplicate_notifications"])

    def test_runtime_and_driver_budgets_are_separate(self):
        report = self.run_fake(["--runtime-current", ".8"])
        self.assertTrue(report["passed"], report)
        self.assertEqual(report["configured"]["config"]["current_limit"], .8)
        self.assertEqual(report["request"]["current"], .1)
        self.assertEqual(report["request"]["runtime_current"], .8)

    def test_rest_passes_without_drive(self):
        report = self.run_fake(["--rest-seconds", "1"])
        self.assertTrue(report["passed"], report)
        self.assertNotIn("diagnose", FakeKnob.last.commands)
        self.assertFalse(report["hold_passed"])

    def test_rest_rejects_existing_motion(self):
        FakeKnob.self_start = True
        report = self.run_fake(["--rest-seconds", "1"])
        self.assertFalse(report["passed"])
        self.assertIn("stationary", report["error"])


if __name__ == "__main__":
    unittest.main()
