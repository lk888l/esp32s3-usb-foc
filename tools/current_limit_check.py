#!/usr/bin/env python3
"""Check 1 A capability and live limit updates; restore config and stop on exit.

This verifies USB acknowledgements and bounded software targets. Rotate the knob
during exercise.py to assess loaded hand feel; these checks do not force a load.
"""
import argparse
import json
import time
from pathlib import Path
from knobctl import Knob


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/current-limits.json"))
    args = parser.parse_args()
    device = Knob(args.port)
    original = None
    report = {"passed": False, "samples": [], "checks": []}

    def passed(name):
        report["checks"].append(name)
        print("PASS:", name, flush=True)

    def sample():
        r = device.request("status")
        report["samples"].append(r)
        require(r["state"] == "active", f"Unexpected state: {r['state']}, {r['fault']}")
        require(abs(r["target"]) <= r["config"]["current_limit"] + 1e-5, "Target exceeded operating limit")
        require(0 <= r["current_target_limit"] <= r["config"]["current_limit"] + 1e-5, "Invalid current budget")
        # Ordinary excess is qualified for 2 ms while torque folds back. The
        # independent absolute ceiling still cuts off on the first valid sample.
        require(max(map(abs, r["currents"])) < r["current_absolute_trip"], "Absolute current ceiling exceeded")
        return r

    try:
        initial = device.request("status")
        require(initial["state"] == "idle" and initial["calibration"]["valid"], "Idle, calibrated board required")
        require(initial["current_limit_max"] == 1, "Expected new 1 A firmware")
        report["initial"] = initial
        original = initial["config"]
        reject = device.request("configure", {**original, "current_limit": 1.01}, allow_error=True)
        require(not reject["ok"] and reject["error"] == "current_limit_range", "1.01 A accepted")
        require(device.request("status")["config"] == original, "Rejected config mutated settings")
        passed("reject_above_1A_atomically")
        for mode in range(4):
            config = {**original, "mode": mode, "current_limit": .8}
            device.request("configure", config)
            device.request("enable")
            time.sleep(.25)
            for limit in (.8, 1, .2, 1, .02, .8):
                config["current_limit"] = limit
                ack = device.request("configure", config)
                require(ack["state"] == "active", "Live update disabled the motor")
                require(abs(ack["config"]["current_limit"] - limit) < 1e-5, "Acknowledged limit differs")
                # A downward change grants the winding 20 ms to settle, without
                # postponing the new software target cap.
                require(abs(ack["target"]) <= limit + 1e-5, "Live target not clamped immediately")
                time.sleep(.035)
                r = sample()
                expected_trip = min(1.25, limit + max(.15, .25 * limit))
                require(abs(r["current_trip_limit"] - expected_trip) < 1e-5, "Runtime trip did not follow configuration")
                device.request("pulse")
                for _ in range(3):
                    sample()
                    time.sleep(.02)
            device.request("disable")
            passed(f"mode_{mode}_live_0.02_to_1A_with_pulse_and_no_disable")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        try:
            if device.identified:
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
