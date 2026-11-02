#!/usr/bin/env python3
"""Bounded real-board checks; requires a completed calibration and pyserial.

Exercises every haptic mode, live configuration, command rejection, NVS save,
and host lease expiry. Always disables the bridge before returning. This is
telemetry validation, not an oscilloscope measurement of peak winding current.
"""
import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path

from knobctl import Knob


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/checks.json"))
    parser.add_argument("--calibration-cycles", type=int, default=0, choices=range(9),
                        help="Repeat up to 8 electrical sweeps; 8 cover a full mechanical turn")
    args = parser.parse_args()
    report = {"time_utc": datetime.now(timezone.utc).isoformat(), "checks": [], "samples": []}
    device = Knob(args.port)
    original = None

    def passed(name):
        report["checks"].append(name)
        print(f"PASS: {name}", flush=True)

    def reject(op, expected, config=None):
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
        require(max(result["duty"]) <= 0.40001, "PWM exceeds sampling window")
        return result

    try:
        initial = device.request("status")
        report["calibration_runs"] = []
        for cycle in range(args.calibration_cycles):
            require(initial["state"] == "idle", "Stop and clear faults before calibration")
            device.request("calibrate")
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline:
                time.sleep(.04)
                initial = device.request("status")
                require(initial["state"] != "fault", initial["fault"])
                if initial["state"] == "idle":
                    require(initial["calibration"]["valid"], "Calibration ended without a valid result")
                    report["calibration_runs"].append(initial)
                    passed(f"calibration_sweep_{cycle + 1}")
                    break
            else:
                raise TimeoutError("Calibration did not finish")
            time.sleep(.15)
        require(initial["state"] == "idle" and initial["calibration"]["valid"], "Calibrate successfully first")
        original = dict(initial["config"])
        report["initial"] = initial
        reject("configure", "current_limit_range", {**original, "current_limit": 1.01})
        require(device.request("status")["config"] == original, "Rejected config changed device settings")
        for mode in range(4):
            device.request("configure", {**original, "mode": mode})
            device.request("enable")
            time.sleep(0.2)
            sample()
            rejected = reject("configure", "current_limit_range", {**original, "current_limit": 1.01})
            require(rejected["state"] == "active", "Rejected frame did not report real motor state")
            reject("save", "disable_before_save")
            reject("configure", "disable_before_electrical_config", {**original, "current_kp": original["current_kp"] + 0.1})
            device.request("zero")
            device.request("pulse")
            stop = time.monotonic() + 2
            while time.monotonic() < stop:
                sample()
                time.sleep(0.04)
            changed = {**original, "mode": mode, "strength": 0.6}
            require(device.request("configure", changed)["state"] == "active", "Live haptic change stopped control")
            sample()
            require(device.request("disable")["state"] == "idle", "Disable not acknowledged")
            passed(f"mode_{mode}_enable_pulse_zero_live_config_disable")
        device.request("configure", original)
        device.request("enable")
        time.sleep(0.2)
        sample()
        # Deliberately withhold every request, including status, beyond the 1 s lease.
        time.sleep(1.25)
        timed_out = device.request("status")
        report["lease_expiry"] = timed_out
        require(timed_out["state"] == "fault" and timed_out["fault"] == "host_timeout", "Host lease did not trip")
        require(timed_out["duty"] == [0, 0, 0] and not timed_out["current_valid"], "Host timeout left drive active")
        require(device.request("disable")["fault"] == "host_timeout", "Disable lost latched fault")
        device.request("clear")
        passed("host_timeout_latches_and_disables")
        device.request("enable")
        time.sleep(0.2)
        sample()
        # Invalid frames must not grant or extend permission to keep torque on.
        end = time.monotonic() + 1.25
        while time.monotonic() < end:
            expired = device.request("invalid_op", allow_error=True)
        require(expired["state"] == "fault" and expired["fault"] == "host_timeout", "Invalid frames renewed the lease")
        device.request("clear")
        passed("invalid_frames_do_not_renew_lease")
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
                device.request("configure", original)
            report["final"] = device.request("status")
        finally:
            device.close()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")


if __name__ == "__main__":
    main()
