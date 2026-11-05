#!/usr/bin/env python3
"""Bounded spring-hold acquisition; disables and restores RAM config on exit.

No calibration or NVS writes. Keep still through offsets, then displace and hold
when the console reports ACTIVE. USB snapshots are not a current-loop trace.
"""
import argparse
import json
import time
from pathlib import Path

from knobctl import Knob


def capture(args):
    report = {"purpose": "PWM / ADC spring-hold comparison", "samples": []}
    original = None
    device = Knob(args.port)
    device.serial.timeout = 0.005
    try:
        initial = device.request("status")
        report["initial"] = initial
        if initial["state"] != "idle" or initial["fault"] != "none":
            raise RuntimeError("Expected idle, fault-free device")
        if not initial["calibration"]["valid"]:
            raise RuntimeError("Valid existing calibration required")
        if initial["pwm_hz"] != args.pwm_hz:
            raise RuntimeError("Unexpected PWM frequency; inspect firmware first")
        original = initial["config"].copy()
        config = {**original, "mode": 3}
        if args.current is not None:
            config["current_limit"] = args.current
        report["configured"] = device.request("configure", config)
        report["enabled"] = device.request("enable")
        started = time.monotonic()
        wait_for_target = getattr(args, "wait_for_target", 0)
        measurement_started = None if wait_for_target else 0
        report["wait_for_target_seconds"] = wait_for_target
        next_log = 0
        announced = False
        while True:
            elapsed = time.monotonic() - started
            if measurement_started is None:
                if elapsed >= wait_for_target:
                    raise TimeoutError("No nonzero spring target within the arming window")
            elif elapsed - measurement_started >= args.seconds:
                break
            s = device.request("status")
            elapsed = time.monotonic() - started
            report["samples"].append({"t": elapsed, **s})
            if s["fault"] != "none":
                raise RuntimeError("Device fault: " + s["fault"])
            if s["state"] == "active" and not announced:
                print("ACTIVE: displace 5-15 degrees, hold 5-10 s, release; repeat both directions", flush=True)
                announced = True
            if measurement_started is None and s["state"] == "active" and abs(s["target"]) > 0.5:
                measurement_started = elapsed
                report["motion_detected_at"] = elapsed
                print("MOTION DETECTED: timed measurement started", flush=True)
            if elapsed >= next_log:
                print(json.dumps({"t": round(elapsed, 2), **{key: s[key] for key in
                    ("state", "target", "iq", "id_current", "voltage_d", "voltage_q",
                     "velocity", "control_hz", "rejected_samples")}}), flush=True)
                next_log += 10
            time.sleep(0.01)
        if not announced:
            raise RuntimeError("Never reached active state")
    except BaseException as error:
        report["error"] = str(error)
        raise
    finally:
        cleanup_errors = []
        try:
            if device.identified:
                stopped = False
                stop_errors = []
                for _ in range(2):
                    try:
                        stopped_reply = device.request("disable")
                        report["disabled"] = stopped_reply
                        stopped = (stopped_reply["state"] in ("idle", "fault") and
                                   stopped_reply["duty"] == [0, 0, 0])
                        if stopped:
                            break
                        stop_errors.append("disable reply still reports output enabled")
                    except Exception as error:
                        stop_errors.append(str(error))
                report["stop_attempt_errors"] = stop_errors
                if not stopped:
                    cleanup_errors.append("disable could not be confirmed: " + "; ".join(stop_errors))
                if original is not None and stopped:
                    try:
                        report["restored"] = device.request("configure", original)
                    except Exception as error:
                        cleanup_errors.append("restore: " + str(error))
                try:
                    report["final"] = device.request("status")
                    final = report["final"]
                    if final["duty"] != [0, 0, 0] or final["state"] not in ("idle", "fault"):
                        cleanup_errors.append("final output is not disabled")
                    if original is not None and final["config"] != original:
                        cleanup_errors.append("final config does not match original")
                except Exception as error:
                    cleanup_errors.append("final status: " + str(error))
        finally:
            device.serial.close()
            report["cleanup_errors"] = cleanup_errors
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            print("Saved " + str(args.output), flush=True)
        if cleanup_errors and "error" not in report:
            raise RuntimeError("; ".join(cleanup_errors))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--pwm-hz", type=int, default=20000)
    parser.add_argument("--current", type=float)
    parser.add_argument("--wait-for-target", type=float, default=0,
                        help="Arm for up to this many seconds; start timing at |target| > 0.5 A")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("seconds must be in (0, 60]")
    if not 0 <= args.wait_for_target <= 90:
        parser.error("wait-for-target must be in [0, 90]")
    if args.output.exists():
        parser.error("output already exists; use a fresh capture path")
    capture(args)


if __name__ == "__main__":
    main()
