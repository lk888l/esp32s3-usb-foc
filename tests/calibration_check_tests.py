"""Offline checks for calibration evidence, cancellation, and failure handling."""
import argparse
import contextlib
import copy
import io
import json
import math
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

# These tests never open a port, so pyserial is not an offline test dependency.
try:
    import serial  # noqa: F401
except ModuleNotFoundError:
    sys.modules["serial"] = types.ModuleType("serial")
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import calibration_check as check
import calibration_guard_check as guard
from knobctl import Knob


def calibration(zero, direction=1):
    return {"valid": True, "direction": direction, "pole_pairs": 7, "electrical_zero": zero}


class Clock:
    now = 0.0

    @classmethod
    def monotonic(cls):
        return cls.now

    @classmethod
    def sleep(cls, seconds):
        cls.now += seconds


class FakeKnob:
    fault_on_run = False
    change_config = False
    change_identity = False
    stall = False
    reject = False
    change_quality = False
    change_quality_on_cancel = False
    bad_completed_quality = False
    fail_transport = False
    reset_during_cleanup = False
    zeros = [math.tau - .01, .01, .02]
    last = None

    def __init__(self, port):
        self.serial = argparse.Namespace(timeout=None)
        self.identified = False
        self.run = self.poll = 0
        self.state = "idle"
        self.fault = "none"
        self.calibration = calibration(.5)
        self.config = {"current_limit": .8}
        self.quality = {"method": check.METHOD, "stage": "idle", "passed": False, "cancelled": False}
        self.accepted_quality = {"method": "legacy_or_unverified", "verified": False, "current": 0,
                                 "forward_points": 0, "reverse_points": 0, "verification_points": 0,
                                 "phase_check_passed": False}
        self.reset_done = False
        self.unparsed_lines = []
        self.commands = []
        FakeKnob.last = self

    def snapshot(self):
        return {"ok": True, "state": self.state, "fault": self.fault,
                "duty": [0, 0, 0] if self.state in ("idle", "fault") else [.01, .02, 0],
                "app_elf_sha256": "other" if self.reset_done or self.change_identity and self.run else "firmware",
                "encoder": "AS5600" if self.reset_done else "MT6701",
                "reset_reason": 4 if self.reset_done else 1,
                "uptime_ms": int(Clock.now * 1000) + (0 if self.reset_done else 100000),
                "config": copy.deepcopy(self.config), "calibration": copy.deepcopy(self.calibration),
                "calibration_report": copy.deepcopy(self.quality),
                "calibration_quality": copy.deepcopy(self.accepted_quality), "sampling_restart_required": False}

    def request(self, op, **parameters):
        self.identified = True
        self.commands.append(op)
        if op == "calibrate":
            if self.reject:
                return {**self.snapshot(), "ok": False, "error": "calibration_current_range"}
            self.run += 1
            self.poll = 0
            self.state = "offsets"
            self.quality = {"method": check.METHOD, "stage": "offsets", "passed": False, "cancelled": False}
        elif op == "status" and self.state in check.BUSY_STATES:
            self.poll += 1
            if self.fail_transport and self.poll == 2:
                raise TimeoutError("Simulated response timeout")
            self.state = "sweep"
            self.quality["stage"] = "forward"
            if self.change_config:
                self.config["current_limit"] = .7
            if self.change_quality:
                self.accepted_quality["verification_points"] = 1
            if self.fault_on_run:
                self.fault = "calibration_current_tracking"
                self.state = "fault"
            elif self.poll >= 3 and not self.stall:
                self.state = "idle"
                self.calibration = calibration(self.zeros[self.run - 1])
                self.quality.update(stage="complete", passed=True, forward_points=29, reverse_points=29,
                                    verification_points=6, phase_check_passed=True)
                self.accepted_quality = {"method": check.METHOD, "verified": not self.bad_completed_quality,
                                         "current": .08, "forward_points": 29, "reverse_points": 29,
                                         "verification_points": 6, "phase_check_passed": True}
        elif op == "disable":
            if self.state in check.BUSY_STATES:
                self.quality.update(stage="cancelled", passed=False, cancelled=True)
                if self.change_quality_on_cancel:
                    self.accepted_quality["verification_points"] = 2
            self.state = "fault" if self.fault != "none" else "idle"
            if self.reset_during_cleanup and not self.reset_done:
                self.reset_done = True
                self.state, self.fault = "idle", "none"
                self.calibration = calibration(.9)
                self.config["current_limit"] = .7
                self.accepted_quality["current"] = .01
                self.unparsed_lines.append("rst:0x4 (TG0WDT_SYS_RESET)")
        return self.snapshot()

    def close(self):
        self.request("disable")


class CalibrationCheckTests(unittest.TestCase):
    def setUp(self):
        Clock.now = 0
        FakeKnob.fault_on_run = FakeKnob.change_config = FakeKnob.change_identity = False
        FakeKnob.stall = FakeKnob.reject = False
        FakeKnob.change_quality = FakeKnob.change_quality_on_cancel = FakeKnob.bad_completed_quality = False
        FakeKnob.fail_transport = FakeKnob.reset_during_cleanup = False
        FakeKnob.zeros = [math.tau - .01, .01, .02]
        FakeKnob.last = None
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)

    def run_fake(self, **kwargs):
        args = argparse.Namespace(port="fake", output=Path(self.folder.name) / "result.json",
                                  current=.08, max_current=.16, voltage=2., cycles=3, timeout=120.,
                                  period=.1, max_zero_spread=.05, interrupt_after=None)
        for key, value in kwargs.items():
            setattr(args, key, value)
        with patch.object(check, "Knob", FakeKnob), patch.object(check.time, "monotonic", Clock.monotonic), \
                patch.object(check.time, "sleep", Clock.sleep), contextlib.redirect_stdout(io.StringIO()):
            result = check.run_check(args)
        self.assertEqual(json.loads(args.output.read_text(encoding="utf-8"))["passed"], result["passed"])
        self.assertFalse({"configure", "save", "clear"}.intersection(FakeKnob.last.commands))
        return result

    def test_circular_comparison(self):
        result = check.compare_calibrations([calibration(math.tau-.01), calibration(.01), calibration(.02)])
        self.assertAlmostEqual(result["max_pairwise_difference_rad"], .03)
        self.assertEqual(len(result["pairwise_differences"]), 3)
        pair = check.compare_calibrations([calibration(math.tau-.01), calibration(.01)])
        self.assertGreaterEqual(pair["zero_mean_rad"], 0)
        self.assertLess(pair["zero_mean_rad"], math.tau)

    def test_invalid_result_rejected(self):
        cases = [[], [calibration(.1), calibration(.2, -1)], [calibration(float("nan"))],
                 [{**calibration(.1), "pole_pairs": 6}], [{**calibration(.1), "valid": False}]]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(RuntimeError):
                check.compare_calibrations(values)

    def test_complete_runs_keep_latest_result_without_saving(self):
        report = self.run_fake()
        self.assertTrue(report["passed"])
        self.assertEqual(len(report["runs"]), 3)
        self.assertEqual(report["final"]["calibration"], calibration(.02))
        self.assertTrue(all(run["samples"] for run in report["runs"]))

    def test_cancel_keeps_previous_result(self):
        report = self.run_fake(interrupt_after=.15)
        self.assertTrue(report["passed"])
        self.assertTrue(report["interruption_verified"])
        self.assertEqual(report["final"]["calibration"], calibration(.5))
        self.assertEqual(report["final"]["calibration_quality"], report["initial"]["calibration_quality"])
        self.assertEqual(report["final"]["duty"], [0, 0, 0])

    def test_fault_is_preserved_and_does_not_replace_result(self):
        FakeKnob.fault_on_run = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(report["final"]["state"], "fault")
        self.assertEqual(report["final"]["calibration"], calibration(.5))
        self.assertEqual(report["final"]["calibration_quality"], report["initial"]["calibration_quality"])
        self.assertEqual(len(report["runs"]), 1)

    def test_timeout_stops_and_preserves_previous_result(self):
        FakeKnob.stall = True
        report = self.run_fake(timeout=.25)
        self.assertFalse(report["passed"])
        self.assertIn("deadline", report["error"])
        self.assertEqual(report["final"]["calibration"], calibration(.5))
        self.assertEqual(report["final"]["duty"], [0, 0, 0])

    def test_changed_config_rejected(self):
        FakeKnob.change_config = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertIn("configuration changed", report["error"])

    def test_changed_firmware_rejected(self):
        FakeKnob.change_identity = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertIn("Firmware identity changed", report["error"])

    def test_changed_accepted_quality_rejected_during_attempt(self):
        FakeKnob.change_quality = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertIn("Accepted calibration quality changed", report["error"])
        self.assertTrue(any("quality changed" in error for error in report["cleanup_errors"]))

    def test_changed_accepted_quality_rejected_on_cancel(self):
        FakeKnob.change_quality_on_cancel = True
        report = self.run_fake(interrupt_after=.15)
        self.assertFalse(report["passed"])
        self.assertIn("Accepted calibration quality changed", report["error"])

    def test_completed_result_requires_verified_quality(self):
        FakeKnob.bad_completed_quality = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertIn("quality is not verified", report["error"])

    def test_completed_result_requires_all_six_independent_points(self):
        good = {"calibration_quality": {"method": check.METHOD, "verified": True, "forward_points": 29,
                                        "reverse_points": 29, "verification_points": 6,
                                        "phase_check_passed": True}}
        self.assertTrue(check.require_verified_quality(good)["verified"])
        good["calibration_quality"]["verification_points"] = 5
        with self.assertRaisesRegex(RuntimeError, "six independent"):
            check.require_verified_quality(good)

    def test_cleanup_reboot_records_all_damage_without_overwriting_original_timeout(self):
        FakeKnob.fail_transport = FakeKnob.reset_during_cleanup = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(report["error"], "Simulated response timeout")
        for reason in ("Runtime configuration changed", "Firmware identity changed", "Encoder identity changed",
                       "Device reset reason changed", "Device restarted", "Calibration differs",
                       "Accepted calibration quality changed"):
            self.assertTrue(any(reason in error for error in report["cleanup_errors"]), reason)
        self.assertEqual(report["unparsed_lines"], ["rst:0x4 (TG0WDT_SYS_RESET)"])

    def test_repeatability_failure_keeps_last_firmware_accepted_result(self):
        FakeKnob.zeros = [.1, .3, .31]
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(len(report["runs"]), 2)
        self.assertIn("repeatability", report["error"])
        self.assertEqual(report["final"]["calibration"], calibration(.3))

    def test_rejected_command_is_recorded_without_retry(self):
        FakeKnob.reject = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertEqual(len(report["command_errors"]), 1)
        self.assertEqual(FakeKnob.last.commands.count("calibrate"), 1)

    def test_large_status_frame_is_accepted(self):
        class Transport:
            def __init__(self):
                reply = {"id": 1, "device": "smartknob-s3", "protocol": 1, "ok": True, "data": "x"*10000}
                self.remaining = json.dumps(reply).encode() + b"\n"

            def read(self, size):
                part, self.remaining = self.remaining[:size], self.remaining[size:]
                return part

            def write(self, data):
                self.sent = data

        client = object.__new__(Knob)
        client.serial = Transport()
        client.sequence = 0
        client.identified = False
        self.assertEqual(len(client.request("status")["data"]), 10000)
        self.assertTrue(client.identified)

    def test_enable_wait_accepts_delayed_offsets_without_reenabling(self):
        client = object.__new__(Knob)
        requests = []

        def request(op):
            requests.append(op)
            return {"state": "active" if Clock.now >= .35 else "offsets", "fault": "none"}

        client.request = request
        with patch.object(check.time, "monotonic", Clock.monotonic), patch.object(check.time, "sleep", Clock.sleep):
            self.assertEqual(client.wait_active()["state"], "active")
        self.assertGreaterEqual(Clock.now, .35)
        self.assertEqual(set(requests), {"status"})

    def test_enable_wait_fails_immediately_on_fault_or_unexpected_state(self):
        for snapshot in ({"state": "fault", "fault": "offset_current_noise"},
                         {"state": "idle", "fault": "none"}):
            client = object.__new__(Knob)
            requests = []

            def request(op):
                requests.append(op)
                return snapshot

            client.request = request
            with patch.object(check.time, "monotonic", Clock.monotonic), \
                    patch.object(check.time, "sleep", Clock.sleep), self.assertRaises(RuntimeError):
                client.wait_active()
            self.assertEqual(requests, ["status"])

    def test_enable_wait_has_a_deadline(self):
        client = object.__new__(Knob)
        client.request = lambda op: {"state": "offsets", "fault": "none"}
        with patch.object(check.time, "monotonic", Clock.monotonic), \
                patch.object(check.time, "sleep", Clock.sleep), self.assertRaises(TimeoutError):
            client.wait_active(timeout=.1)
        self.assertLessEqual(Clock.now, .100001)


class GuardFakeKnob:
    unexpected_fault = False
    transport_failure = False
    last = None

    def __init__(self, port):
        self.serial = argparse.Namespace(timeout=None, close=lambda: None)
        self.identified = False
        self.state, self.fault, self.operation = "idle", "none", "idle"
        self.run = 0
        self.started = 0
        self.commands = []
        self.report = {"method": check.METHOD, "stage": "idle",
                       "passed": False, "cancelled": False}
        GuardFakeKnob.last = self

    def snapshot(self, error=None):
        result = {"ok": error is None, "state": self.state, "fault": self.fault,
                  "duty": [0, 0, 0] if self.state in ("idle", "fault", "offsets") else [.01, 0, 0],
                  "config": {"current_limit": .8}, "calibration": calibration(.5),
                  "calibration_quality": {"method": check.METHOD, "verified": True, "forward_points": 29,
                                          "reverse_points": 29, "verification_points": 6, "phase_check_passed": True},
                  "calibration_report": copy.deepcopy(self.report), "app_elf_sha256": "firmware",
                  "encoder": "MT6701", "reset_reason": 1, "uptime_ms": 100000 + int(Clock.now*1000),
                  "angle": .3,
                  "bus_voltage_valid": True, "adc_sample_valid": True, "sampling_restart_required": False,
                  "encoder_error": 0}
        if error:
            result["error"] = error
        return result

    def request(self, op, **parameters):
        self.identified = True
        self.commands.append(op)
        if op == "calibrate":
            current = parameters.get("current", .08)
            maximum = parameters.get("max_current", .16)
            voltage = parameters.get("voltage", 2)
            if not isinstance(current, (int, float)):
                return self.snapshot("calibration_parameters")
            if current > .25 or maximum > .25 or maximum < current:
                return self.snapshot("calibration_current_range")
            if voltage > 3:
                return self.snapshot("calibration_voltage_range")
            if self.state != "idle":
                return self.snapshot("idle_required")
            self.run += 1
            self.operation = "calibration"
            self.started = Clock.now
            self.state = "offsets"
            self.report = {"method": check.METHOD, "stage": "offsets", "passed": False, "cancelled": False}
        elif op == "enable":
            self.operation = "enable"
            self.started = Clock.now
            self.state = "offsets"
        elif op == "status" and self.state in guard.BUSY_STATES:
            elapsed = Clock.now - self.started
            if self.operation == "enable" and elapsed >= .2:
                self.state = "active"
            elif self.operation == "calibration":
                if self.run == 2 and self.transport_failure:
                    raise TimeoutError("Simulated guard transport failure")
                if self.run == 2 and self.unexpected_fault:
                    self.state, self.fault = "fault", "calibration_phase_polarity"
                    self.report["stage"] = "failed"
                elif self.run == 3 and elapsed >= .25:
                    self.state, self.fault = "fault", guard.EXPECTED_FAULT
                    self.report["stage"] = "failed"
                elif self.run == 2 and elapsed >= .35:
                    self.state = "settle"
                    self.report["stage"] = "verify"
                elif elapsed >= .12:
                    self.state = "sweep"
                    self.report["stage"] = "forward"
        elif op == "disable":
            if self.operation == "calibration" and self.state in guard.BUSY_STATES:
                self.report.update(stage="cancelled", cancelled=True, passed=False)
            self.state = "fault" if self.fault != "none" else "idle"
        elif op == "clear":
            if self.fault != guard.EXPECTED_FAULT:
                raise AssertionError("Unexpected fault was cleared")
            self.state, self.fault = "idle", "none"
        return self.snapshot()


class CalibrationGuardTests(unittest.TestCase):
    def setUp(self):
        Clock.now = 0
        GuardFakeKnob.unexpected_fault = GuardFakeKnob.transport_failure = False
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)

    def run_fake(self):
        args = argparse.Namespace(port="fake", output=Path(self.folder.name)/"guard.json", timeout=120, period=.05)
        with patch.object(guard, "Knob", GuardFakeKnob), patch.object(guard.time, "monotonic", Clock.monotonic), \
                patch.object(guard.time, "sleep", Clock.sleep), contextlib.redirect_stdout(io.StringIO()):
            report = guard.run_check(args)
        self.assertFalse({"configure", "save"}.intersection(GuardFakeKnob.last.commands))
        self.assertEqual(len(report["responses"]), len(GuardFakeKnob.last.commands))
        self.assertEqual(json.loads(args.output.read_text(encoding="utf-8"))["passed"], report["passed"])
        return report

    def test_complete_guard_sequence_clears_only_witnessed_expected_fault(self):
        report = self.run_fake()
        self.assertTrue(report["passed"], report.get("error"))
        self.assertEqual(len(report["checks"]), 10)
        self.assertEqual(GuardFakeKnob.last.commands.count("clear"), 1)
        self.assertTrue(report["expected_fault_cleared"])
        self.assertEqual(report["final"]["fault"], "none")
        self.assertEqual(report["final"]["calibration_quality"], report["initial"]["calibration_quality"])

    def test_unexpected_fault_stops_without_clear(self):
        GuardFakeKnob.unexpected_fault = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertNotIn("clear", GuardFakeKnob.last.commands)
        self.assertEqual(report["final"]["fault"], "calibration_phase_polarity")
        self.assertEqual(report["final"]["duty"], [0, 0, 0])

    def test_transport_failure_stops_without_clear(self):
        GuardFakeKnob.transport_failure = True
        report = self.run_fake()
        self.assertFalse(report["passed"])
        self.assertNotIn("clear", GuardFakeKnob.last.commands)
        self.assertEqual(report["error"], "Simulated guard transport failure")
        self.assertTrue(report["transport_errors"])
        self.assertEqual(report["final"]["duty"], [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
