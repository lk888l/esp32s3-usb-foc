"""Exercise the real JSON client offline with a captured v21 status response.

fixtures/knob-v21-status.json is the complete hardware-first-v21.json response
captured on 2026-11-07. Its twelve-mode bank previously made all three tools
exceed the firmware request limit. Only serial transport is replaced here;
Knob.request still serializes requests, checks their size and parses replies.
"""
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
import knobctl
import frictionless_check
import reference_modes_check
import exercise
import usb_smoke


FIXTURE = json.loads((Path(__file__).parent / "fixtures" / "knob-v21-status.json").read_text())


class Clock:
    now = 0.

    @classmethod
    def monotonic(cls):
        return cls.now

    @classmethod
    def sleep(cls, seconds):
        cls.now += seconds


class SerialFixture:
    """In-memory wire peer; never opens or enumerates a device."""
    last = None
    reject_configures = ()
    lose_first_configure_reply = False

    def __init__(self, *args, **kwargs):
        SerialFixture.last = self
        self.config = copy.deepcopy(FIXTURE["config"])
        # Nondefault values expose accidental reset of an omitted bank and
        # prove that the mode checker honors configurable initial positions.
        self.config["reference_modes"]["8"]["strength"] = 17.25
        self.config["reference_modes"]["10"]["position"] = 123
        self.config["reference_modes"]["11"]["position"] = 130
        self.original = copy.deepcopy(self.config)
        self.state = "idle"
        self.writes = []
        self.pending = bytearray()
        self.errors = list(self.reject_configures)
        self.lost_reply = False
        self.closed = False

    def open(self):
        pass

    def set_buffer_size(self, **kwargs):
        pass

    def close(self):
        self.closed = True

    def read(self, size):
        result = self.pending[:size]
        del self.pending[:size]
        if not result:
            Clock.sleep(.05)
        return bytes(result)

    def write(self, encoded):
        request = json.loads(encoded)
        self.writes.append((request, bytes(encoded)))
        op = request["op"]
        error = None
        if op == "configure":
            if self.errors:
                # The mode checker has fifteen known invalid-config probes;
                # replay their expected refusal codes without emulating the
                # firmware validator (which has its own C++/hardware tests).
                error = self.errors.pop(0)
            else:
                update = copy.deepcopy(request["config"])
                bank_patch = update.pop("reference_modes", {})
                self.config.update(update)
                self.config["reference_modes"].update(bank_patch)
                if self.lose_first_configure_reply and not self.lost_reply:
                    self.lost_reply = True
                    return len(encoded)
        elif op == "enable":
            self.state = "active"
        elif op == "disable":
            self.state = "idle"
        elif op != "status":
            raise AssertionError("Unexpected test command: " + op)
        reply = copy.deepcopy(FIXTURE)
        mode = self.config["mode"]
        reply.update(id=request["id"], ok=error is None, state=self.state,
                     config=copy.deepcopy(self.config), mode_name=reply["modes"][mode],
                     reference_preset=mode - 4 if mode >= 4 else -1,
                     position=self.config["reference_modes"].get(str(mode), {}).get("position", 0),
                     uptime_ms=FIXTURE["uptime_ms"] + round(Clock.now * 1000),
                     control_cycles=FIXTURE["control_cycles"] + round(Clock.now * 5000),
                     velocity=0., target=0., iq=0., duty=[0, 0, 0])
        if error:
            reply["error"] = error
        self.pending.extend(json.dumps(reply, allow_nan=False).encode() + b"\n")
        return len(encoded)


class ClientCompatibilityTests(unittest.TestCase):
    def setUp(self):
        Clock.now = 0.
        SerialFixture.reject_configures = ()
        SerialFixture.lose_first_configure_reply = False
        self.folder = tempfile.TemporaryDirectory()
        self.addCleanup(self.folder.cleanup)
        for patcher in (patch.object(knobctl.serial, "Serial", SerialFixture, create=True),
                        patch.object(knobctl.time, "monotonic", Clock.monotonic),
                        patch.object(knobctl.time, "sleep", Clock.sleep)):
            patcher.start()
            self.addCleanup(patcher.stop)

    def run_cli(self, *arguments):
        with patch.object(sys, "argv", ["knobctl", "--port", "OFFLINE", *arguments]), \
                contextlib.redirect_stdout(io.StringIO()):
            knobctl.main()

    def assert_closed_and_stopped(self):
        peer = SerialFixture.last
        self.assertTrue(peer.closed)
        self.assertEqual(peer.state, "idle")
        self.assertEqual(peer.writes[-1][0]["op"], "disable")
        self.assertTrue(all(len(encoded) <= 2048 for _, encoded in peer.writes))
        self.assertFalse({"save", "clear"} & {request["op"] for request, _ in peer.writes})
        return peer

    def test_common_configuration_omits_only_bank_and_does_not_alias(self):
        original = copy.deepcopy(FIXTURE["config"])
        copied = knobctl.common_configuration(original)
        self.assertEqual(set(original) - set(copied), {"reference_modes"})
        self.assertEqual(copied, {key: value for key, value in original.items() if key != "reference_modes"})
        copied["frictionless"]["scale"] = .73
        self.assertEqual(original, FIXTURE["config"])
        oversized = json.dumps({"id": 1, "op": "configure", "config": original}).encode() + b"\n"
        self.assertGreater(len(oversized), 2048)  # Real v21 data reproduces the regression.

    def test_common_configuration_accepts_older_firmware_without_bank(self):
        original = copy.deepcopy(FIXTURE["config"])
        del original["reference_modes"]
        self.assertEqual(knobctl.common_configuration(original), original)

    def test_both_reply_readers_accept_32768_and_reject_larger_frames(self):
        class ReplyTransport:
            def __init__(self, body):
                self.pending = body + b"\n"

            def write(self, encoded):
                return len(encoded)

            def read(self, size):
                data, self.pending = self.pending[:size], self.pending[size:]
                if not data:
                    Clock.sleep(.05)
                return data

        for reader in ("knobctl", "usb_smoke"):
            for length in (32768, 32769):
                with self.subTest(reader=reader, length=length):
                    response = copy.deepcopy(FIXTURE)
                    response["extension"] = ""
                    response["extension"] = "x" * (length - len(json.dumps(response).encode()))
                    body = json.dumps(response).encode()
                    self.assertEqual(len(body), length)
                    client = object.__new__(knobctl.Knob)
                    client.serial = ReplyTransport(body)
                    client.sequence = 0
                    client.identified = False
                    client.unparsed_lines = []
                    read = (lambda: client.request("status")) if reader == "knobctl" else \
                        (lambda: usb_smoke.read_replies(client, [1])[1])
                    if length == 32768:
                        self.assertEqual(read(), response)
                    else:
                        with self.assertRaises(TimeoutError if reader == "knobctl" else RuntimeError):
                            read()

    def test_cli_enable_mode_preserves_bank_and_stops_on_exit(self):
        self.run_cli("enable", "--mode", "16", "--seconds", ".1")
        peer = self.assert_closed_and_stopped()
        configures = [request for request, _ in peer.writes if request["op"] == "configure"]
        self.assertEqual(len(configures), 1)
        self.assertNotIn("reference_modes", configures[0]["config"])
        self.assertEqual(peer.config["mode"], 16)
        self.assertEqual(peer.config["reference_modes"], peer.original["reference_modes"])
        self.assertIn("enable", [request["op"] for request, _ in peer.writes])

    def test_cli_explicit_single_mode_patch_is_not_stripped(self):
        config = knobctl.common_configuration(FIXTURE["config"])
        config["reference_modes"] = {"8": copy.deepcopy(FIXTURE["config"]["reference_modes"]["8"])}
        config["reference_modes"]["8"]["strength"] = 19.5
        path = Path(self.folder.name) / "patch.json"
        path.write_text(json.dumps(config))
        self.run_cli("configure", "--mode", "8", "--config", str(path))
        peer = self.assert_closed_and_stopped()
        sent = next(request["config"] for request, _ in peer.writes if request["op"] == "configure")
        self.assertEqual(sent["reference_modes"], config["reference_modes"])
        self.assertEqual(peer.config["reference_modes"]["8"]["strength"], 19.5)
        for mode in set(peer.config["reference_modes"]) - {"8"}:
            self.assertEqual(peer.config["reference_modes"][mode], peer.original["reference_modes"][mode])

    def test_cli_oversized_explicit_bank_still_rejected_and_stopped(self):
        path = Path(self.folder.name) / "oversized.json"
        path.write_text(json.dumps(FIXTURE["config"]))
        with self.assertRaisesRegex(ValueError, "frame size"):
            self.run_cli("configure", "--config", str(path))
        peer = self.assert_closed_and_stopped()
        self.assertEqual(peer.config, peer.original)
        self.assertNotIn("configure", [request["op"] for request, _ in peer.writes])

    def run_frictionless(self):
        args = frictionless_check.parse_args(["--port", "OFFLINE", "--output",
            str(Path(self.folder.name) / "frictionless.json"), "--rest-seconds", "1", "--scale", ".73"])
        with contextlib.redirect_stdout(io.StringIO()):
            report = frictionless_check.run_check(args)
        peer = self.assert_closed_and_stopped()
        self.assertEqual(peer.config, peer.original)
        self.assertEqual(report["final"]["config"], report["initial"]["config"])
        self.assertTrue(report["cleanup_passed"], report)
        for request, _ in peer.writes:
            if request["op"] == "configure":
                self.assertNotIn("reference_modes", request["config"])
        return report

    def test_frictionless_rest_applies_common_values_and_restores_full_config(self):
        report = self.run_frictionless()
        self.assertTrue(report["passed"], report)
        self.assertEqual(report["configured"]["config"]["frictionless"]["scale"], .73)

    def test_frictionless_lost_configure_reply_still_stops_and_restores(self):
        SerialFixture.lose_first_configure_reply = True
        report = self.run_frictionless()
        self.assertFalse(report["passed"])
        self.assertIn("timed out", report["error"])
        self.assertTrue(SerialFixture.last.lost_reply)

    def run_exercise(self):
        arguments = ["exercise", "--port", "OFFLINE", "--seconds", "1", "--cycle-modes",
                     "--output", str(Path(self.folder.name) / "exercise.jsonl")]
        with patch.object(sys, "argv", arguments), contextlib.redirect_stdout(io.StringIO()):
            exercise.main()

    def test_legacy_exercise_cycles_modes_and_restores_custom_bank(self):
        self.run_exercise()
        peer = self.assert_closed_and_stopped()
        self.assertEqual(peer.config, peer.original)
        changes = [request["config"] for request, _ in peer.writes if request["op"] == "configure"]
        self.assertEqual([config["mode"] for config in changes[:4]], [0, 1, 2, 3])
        self.assertTrue(all("reference_modes" not in config for config in changes))

    def test_legacy_exercise_lost_configure_reply_still_stops_and_restores(self):
        SerialFixture.lose_first_configure_reply = True
        with self.assertRaisesRegex(TimeoutError, "timed out"):
            self.run_exercise()
        peer = self.assert_closed_and_stopped()
        self.assertEqual(peer.config, peer.original)
        self.assertTrue(peer.lost_reply)

    def test_mode_checker_runs_all_modes_with_custom_bank_and_restores(self):
        SerialFixture.reject_configures = (
            "mode_invalid", "mode_invalid", "frictionless_friction_range",
            "frictionless_friction_range", "frictionless_viscous_range",
            "frictionless_scale_range", "frictionless_scale_range",
            "frictionless_pi_range", "frictionless_pi_range", "frictionless_config_required",
            *("frictionless_field_missing",) * 5)
        args = reference_modes_check.parse_args(["--port", "OFFLINE", "--output",
                                                str(Path(self.folder.name) / "modes.json")])
        with contextlib.redirect_stdout(io.StringIO()):
            report = reference_modes_check.run_check(args)
        peer = self.assert_closed_and_stopped()
        self.assertTrue(report["passed"], report["errors"] + report["cleanup_errors"])
        self.assertEqual(len(report["modes"]), 17)
        self.assertEqual(len(report["rejections"]), 15)
        self.assertEqual(peer.config, peer.original)
        for request, _ in peer.writes:
            if request["op"] == "configure":
                self.assertNotIn("reference_modes", request["config"])
        for entry in report["modes"]:
            self.assertEqual(entry["configured"]["config"]["reference_modes"], peer.original["reference_modes"])


if __name__ == "__main__":
    unittest.main()
