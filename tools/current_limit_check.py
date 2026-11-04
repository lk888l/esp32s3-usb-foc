#!/usr/bin/env python3
"""Check 4.5 A capability and live limit updates; restore config and stop on exit.

This verifies USB acknowledgements and bounded software targets. Rotate the knob
during exercise.py to assess loaded hand feel; these checks do not force a load.
The 4.5 A configuration boundary is tested while idle. Energized checks default
to at most 0.8 A; use --max-current explicitly to exercise a higher limit.
"""
import argparse
import json
import math
import time
from pathlib import Path
from knobctl import Knob


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def enable_when_ready(device, report, mode):
    """Retry only explicit idle/readiness rejections, never uncertain delivery."""
    run = {"mode": mode, "attempts": 0, "retries": 0, "responses": []}
    report["enable_attempts"].append(run)
    deadline = time.monotonic() + .2
    while True:
        if run["attempts"] and time.monotonic() >= deadline:
            raise RuntimeError("Enable readiness retry deadline exceeded")
        run["attempts"] += 1
        if run["attempts"] > 1:
            run["retries"] += 1
            report["enable_retries"] += 1
        # Timeout, malformed framing, or identity errors propagate immediately:
        # an unacknowledged enable may already have been accepted by the device.
        try:
            reply = device.request("enable", allow_error=True)
        except Exception as error:
            run["responses"].append({"request_error": str(error)})
            raise
        if not isinstance(reply, dict):
            run["responses"].append({"request_error": "Unknown enable response"})
            raise RuntimeError("Unknown enable response")
        run["responses"].append({key: reply.get(key) for key in ("ok", "error", "state", "fault")})
        if reply.get("ok") is True:
            require(reply.get("fault") == "none", f"Enable reported fault: {reply.get('fault')}")
            return reply
        retryable = (reply.get("ok") is False and reply.get("state") == "idle" and
                     reply.get("fault") == "none" and
                     reply.get("error") in ("adc_window_invalid", "encoder_not_ready"))
        require(retryable, f"Enable rejected: {reply.get('error')}, state={reply.get('state')}, fault={reply.get('fault')}")
        require(run["attempts"] < 5 and time.monotonic() + .04 < deadline,
                f"Enable readiness retries exhausted: {reply['error']}")
        time.sleep(.04)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/current-limits.json"))
    parser.add_argument("--max-current", type=float, default=.8,
                        help="Highest energized test limit in A (0.02–4.5; default: 0.8)")
    args = parser.parse_args()
    if not math.isfinite(args.max_current) or not .02 <= args.max_current <= 4.5:
        parser.error("--max-current must be in [0.02, 4.5] A")
    device = Knob(args.port)
    original = None
    report = {"passed": False, "samples": [], "checks": [], "max_test_current": args.max_current,
              "enable_retries": 0, "enable_attempts": []}

    def passed(name):
        report["checks"].append(name)
        print("PASS:", name, flush=True)

    def sample():
        r = device.request("status")
        report["samples"].append(r)
        require(r["state"] == "active", f"Unexpected state: {r['state']}, {r['fault']}")
        require(abs(r["target"]) <= r["config"]["current_limit"] + 1e-5, "Target exceeded operating limit")
        require(0 <= r["current_target_limit"] <= r["config"]["current_limit"] + 1e-5, "Invalid current budget")
        # Ordinary excess uses a 20 ms exposure budget while torque folds back;
        # The fixed peak band uses 1 ms and the absolute band cuts off on the
        # first valid sample.
        # These USB snapshots cannot validate sub-millisecond hardware trips.
        require(max(map(abs, r["currents"])) < r["current_absolute_trip"], "Absolute current ceiling exceeded")
        return r

    try:
        initial = device.request("status")
        require(initial["state"] == "idle" and initial["calibration"]["valid"], "Idle, calibrated board required")
        require(initial["current_limit_max"] == 4.5, "Expected 4.5 A firmware capability")
        expected_policy = {"current_absolute_trip": 6, "current_peak_trip": 5,
                           "current_peak_trip_ms": 1, "current_overload_trip_ms": 20,
                           "current_trip_recovery_ratio": 4}
        require(all(initial.get(key) == value for key, value in expected_policy.items()),
                "Expected staged overcurrent policy (20 ms / 1 ms / immediate)")
        passed("staged_overcurrent_policy_telemetry")
        report["initial"] = initial
        original = initial["config"]
        reject = device.request("configure", {**original, "current_limit": 4.51}, allow_error=True)
        require(not reject["ok"] and reject["error"] == "current_limit_range", "4.51 A accepted")
        require(device.request("status")["config"] == original, "Rejected config mutated settings")
        passed("reject_above_4.5A_atomically")
        maximum = device.request("configure", {**original, "current_limit": 4.5})
        require(maximum["state"] == "idle" and maximum["config"]["current_limit"] == 4.5,
                "Maximum capability was not accepted while idle")
        require(maximum["duty"] == [0, 0, 0], "Idle capability check energized the motor")
        passed("accept_4.5A_while_idle")
        for mode in range(4):
            config = {**original, "mode": mode, "current_limit": args.max_current}
            device.request("configure", config)
            enable_when_ready(device, report, mode)
            time.sleep(.25)
            for limit in (args.max_current, min(.2, args.max_current), args.max_current, .02, args.max_current):
                config["current_limit"] = limit
                ack = device.request("configure", config)
                require(ack["state"] == "active", "Live update disabled the motor")
                require(abs(ack["config"]["current_limit"] - limit) < 1e-5, "Acknowledged limit differs")
                # A downward change grants the winding 20 ms to settle, without
                # postponing the new software target cap.
                require(abs(ack["target"]) <= limit + 1e-5, "Live target not clamped immediately")
                time.sleep(.035)
                r = sample()
                expected_trip = min(initial["current_peak_trip"], limit + max(.15, .25 * limit))
                require(abs(r["current_trip_limit"] - expected_trip) < 1e-5, "Runtime trip did not follow configuration")
                device.request("pulse")
                for _ in range(3):
                    sample()
                    time.sleep(.02)
            device.request("disable")
            passed(f"mode_{mode}_live_0.02_to_{args.max_current:g}A_with_pulse_and_no_disable")
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
