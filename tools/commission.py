#!/usr/bin/env python3
"""Bounded real-board checks; requires a completed calibration and pyserial.

Exercises every haptic mode, live configuration, command rejection, NVS save,
and host lease expiry. Always disables the bridge before returning. This is
telemetry validation, not an oscilloscope measurement of peak winding current.
"""
import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

from knobctl import Knob, common_configuration


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def calibrate_once(device, previous, attempt):
    """Wait for the complete bidirectional sequence and its quality verdict."""
    require(previous["state"] == "idle" and previous["fault"] == "none",
            "Stop and clear faults before calibration")
    require(previous.get("calibration_report", {}).get("method") == "bidirectional_v1",
            "Bidirectional calibration firmware required")
    attempt["previous"] = previous
    attempt["samples"] = []
    started = time.monotonic()
    accepted = device.request("calibrate")
    attempt["accepted"] = accepted
    require(accepted["state"] in ("offsets", "align", "sweep", "settle") and
            not accepted.get("calibration_report", {}).get("passed", True),
            "Calibration did not start a fresh run")
    while time.monotonic() - started < 130:
        time.sleep(.04)
        snapshot = device.request("status")
        attempt["samples"].append(snapshot)
        require(snapshot["fault"] == "none", snapshot["fault"])
        require(snapshot["config"] == previous["config"], "Calibration changed configuration")
        if snapshot["state"] == "idle":
            quality = snapshot.get("calibration_report", {})
            require(snapshot["calibration"]["valid"], "Calibration ended without a valid result")
            require(quality.get("method") == "bidirectional_v1" and quality.get("passed") is True
                    and not quality.get("cancelled", False), "Calibration quality checks did not pass")
            require(quality.get("forward_points") == 29 and quality.get("reverse_points") == 29,
                    "Calibration did not complete both scans")
            require(snapshot["duty"] == [0, 0, 0], "Calibration left output energized")
            attempt["elapsed_seconds"] = time.monotonic() - started
            return snapshot
        require(snapshot["state"] in ("offsets", "align", "sweep", "settle"),
                "Unexpected calibration state")
        require(snapshot["calibration"] == previous["calibration"],
                "Unfinished calibration replaced the previous result")
    raise TimeoutError("Calibration did not finish within the 130-second host deadline")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/checks.json"))
    parser.add_argument("--calibration-cycles", type=int, default=0, choices=range(9),
                        help="Repeat up to 8 complete bidirectional calibrations (each scans a full mechanical turn both ways)")
    parser.add_argument("--max-current", type=float, default=.8,
                        help="Cap energized checks at this current in A (0.02–4.5; default: 0.8)")
    args = parser.parse_args()
    if not math.isfinite(args.max_current) or not .02 <= args.max_current <= 4.5:
        parser.error("--max-current must be in [0.02, 4.5] A")
    report = {"time_utc": datetime.now(timezone.utc).isoformat(), "checks": [], "samples": []}
    device = Knob(args.port)
    original = None

    def passed(name):
        report["checks"].append(name)
        print(f"PASS: {name}", flush=True)

    def reject(op, expected, config=None):
        if op == "configure" and config is not None:
            config = common_configuration(config)
        reply = device.request(op, config, allow_error=True)
        require(not reply["ok"] and reply.get("error") == expected, f"Unexpected response to {op}")
        passed(expected)
        return reply

    def sample():
        result = device.request("status")
        report["samples"].append(result)
        require(result["state"] == "active", f"Unexpected state: {result['state']}, {result['fault']}")
        require(max(map(abs, result["currents"])) < result.get("current_absolute_trip", result.get("current_trip_limit", 0.9)), "Observed phase current exceeds immediate trip threshold")
        require(abs(result["target"]) <= result["config"]["current_limit"] + 1e-5, "Target exceeds configured current limit")
        require(min(result["duty"]) >= 0 and max(result["duty"]) <= result["pwm_duty_ceiling"] + 0.00001,
                "PWM exceeds the reported modulation range")
        return result

    try:
        initial = device.request("status")
        report["calibration_runs"] = []
        report["calibration_attempts"] = []
        for cycle in range(args.calibration_cycles):
            attempt = {"cycle": cycle + 1}
            report["calibration_attempts"].append(attempt)
            initial = calibrate_once(device, initial, attempt)
            report["calibration_runs"].append(initial)
            passed(f"calibration_sweep_{cycle + 1}")
            time.sleep(.15)
        require(initial["state"] == "idle" and initial["calibration"]["valid"], "Calibrate successfully first")
        original = dict(initial["config"])
        report["initial"] = initial
        invalid_limit = initial.get("current_limit_max", .8) + .01
        test_config = {**original, "current_limit": min(original["current_limit"], args.max_current)}
        report["max_test_current"] = test_config["current_limit"]
        reject("configure", "current_limit_range", {**original, "current_limit": invalid_limit})
        require(device.request("status")["config"] == original, "Rejected config changed device settings")
        for mode in range(4):
            device.request("configure", {**common_configuration(test_config), "mode": mode})
            device.request("enable")
            device.wait_active()
            sample()
            rejected = reject("configure", "current_limit_range", {**test_config, "current_limit": invalid_limit})
            require(rejected["state"] == "active", "Rejected frame did not report real motor state")
            reject("save", "disable_before_save")
            reject("configure", "disable_before_electrical_config", {**test_config, "current_kp": original["current_kp"] + 0.1})
            device.request("zero")
            device.request("pulse")
            stop = time.monotonic() + 2
            while time.monotonic() < stop:
                sample()
                time.sleep(0.04)
            changed = {**test_config, "mode": mode, "strength": 0.6}
            require(device.request("configure", common_configuration(changed))["state"] == "active", "Live haptic change stopped control")
            sample()
            require(device.request("disable")["state"] == "idle", "Disable not acknowledged")
            passed(f"mode_{mode}_enable_pulse_zero_live_config_disable")
        device.request("configure", common_configuration(test_config))
        device.request("enable")
        device.wait_active()
        sample()
        # Firmware may run standalone (lease=0), or enforce a configured lease.
        lease_ms = initial.get("host_lease_ms", 1000)
        observation_s = lease_ms / 1000 + .25 if lease_ms else 1.25
        time.sleep(observation_s)
        timed_out = device.request("status")
        report["lease_expiry"] = timed_out
        if lease_ms:
            require(timed_out["state"] == "fault" and timed_out["fault"] == "host_timeout", "Host lease did not trip")
            require(timed_out["duty"] == [0, 0, 0] and not timed_out["current_valid"], "Host timeout left drive active")
            require(device.request("disable")["fault"] == "host_timeout", "Disable lost latched fault")
            device.request("clear")
            passed("host_timeout_latches_and_disables")
        else:
            require(timed_out["state"] == "active" and timed_out["fault"] == "none", "Standalone session stopped without disable")
            device.request("disable")
            passed("standalone_operation_without_heartbeat")
        device.request("enable")
        device.wait_active()
        sample()
        # Invalid frames must not grant or extend permission to keep torque on.
        end = time.monotonic() + observation_s
        while time.monotonic() < end:
            expired = device.request("invalid_op", allow_error=True)
        if lease_ms:
            require(expired["state"] == "fault" and expired["fault"] == "host_timeout", "Invalid frames renewed the lease")
            device.request("clear")
            passed("invalid_frames_do_not_renew_lease")
        else:
            require(expired["state"] == "active" and expired["fault"] == "none", "Invalid frames changed standalone operation")
            device.request("disable")
            passed("invalid_frames_preserve_standalone_session")
        device.request("configure", common_configuration(original))
        report["saved"] = device.request("save")
        passed("save_while_idle")
        report["passed"] = True
    except Exception as error:
        report["passed"] = False
        report["error"] = str(error)
        raise
    finally:
        try:
            device.request("disable")
            if original is not None:
                device.request("configure", common_configuration(original))
            report["final"] = device.request("status")
        finally:
            device.close()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
