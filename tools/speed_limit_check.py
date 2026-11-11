#!/usr/bin/env python3
"""Record hand-driven overspeed fade/recovery; always disable and restore config.

Run after disconnecting the desktop host. A successful capture without a full
cutoff/recovery cycle is a smoke test only; --require-cycle enforces that cycle.
"""
import argparse
import json
import time
from pathlib import Path
from knobctl import Knob, common_configuration


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--mode", type=int, choices=range(4))
    parser.add_argument("--require-cycle", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/speed-limit.jsonl"))
    args = parser.parse_args()
    if not 1 <= args.seconds <= 300:
        parser.error("--seconds must be between 1 and 300")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    device = Knob(args.port)
    # Short reads keep native USB captures fast enough to observe a 10 ms fade.
    device.serial.timeout = .005
    original = None
    report = {"passed": False, "samples": 0, "paused_samples": 0,
              "faded_samples": 0, "muted_samples": 0, "recovery_cycles": 0,
              "peak_speed": 0, "min_velocity": 0, "max_velocity": 0,
              "muted_target_peak": 0, "capture": str(args.output)}
    was_muted = False
    try:
        initial = device.request("status")
        report["initial"] = initial
        require(initial["state"] == "idle" and initial["calibration"]["valid"],
                "An idle, calibrated board is required")
        require("haptic_scale" in initial and "haptic_paused" in initial,
                "Firmware does not report the speed envelope")
        original = initial["config"]
        if args.mode is not None:
            device.request("configure", {**common_configuration(original), "mode": args.mode})
        device.request("enable")
        deadline = time.monotonic() + 2
        while True:
            state = device.request("status")
            if state["state"] == "active":
                break
            require(state["state"] != "fault", state["fault"])
            require(time.monotonic() < deadline, "Enable timed out")
            time.sleep(.005)
        print(f"READY: hand rotation capture, {args.seconds:g} seconds; stops automatically.", flush=True)
        started = time.monotonic()
        next_report = 0
        with args.output.open("w") as output:
            while time.monotonic() - started < args.seconds:
                sample = device.request("status")
                sample["test_elapsed_s"] = time.monotonic() - started
                output.write(json.dumps(sample) + "\n")
                output.flush()
                report["samples"] += 1
                velocity = sample["velocity"]
                report["peak_speed"] = max(report["peak_speed"], abs(velocity))
                report["min_velocity"] = min(report["min_velocity"], velocity)
                report["max_velocity"] = max(report["max_velocity"], velocity)
                require(sample["state"] == "active" and sample["fault"] == "none",
                        f"Unexpected state: {sample['state']}, {sample['fault']}")
                require(0 <= sample["haptic_scale"] <= 1, "Invalid haptic scale")
                require(abs(sample["target"]) <= sample["config"]["current_limit"] + 1e-5,
                        "Target exceeded current limit")
                if sample["speed_limited"]:
                    report["faded_samples"] += 1
                if sample["haptic_paused"]:
                    report["paused_samples"] += 1
                if sample["haptic_paused"] and sample["haptic_scale"] <= 1e-6:
                    report["muted_samples"] += 1
                    report["muted_target_peak"] = max(report["muted_target_peak"], abs(sample["target"]))
                    require(abs(sample["target"]) <= 1e-5, "Torque remained after full fade")
                    if not was_muted:
                        print(f"PAUSED: {velocity:.2f} rad/s, target {sample['target']:.5f} A", flush=True)
                    was_muted = True
                elif was_muted and not sample["haptic_paused"] and not sample["speed_limited"]:
                    report["recovery_cycles"] += 1
                    was_muted = False
                    print(f"RECOVERED: cycle {report['recovery_cycles']}, state active, fault none", flush=True)
                if sample["test_elapsed_s"] >= next_report:
                    print(json.dumps({key: sample[key] for key in
                        ("test_elapsed_s", "velocity", "position", "haptic_scale",
                         "haptic_paused", "target", "state", "fault")}), flush=True)
                    next_report += 10
                time.sleep(.003)
        report["cycle_verified"] = report["recovery_cycles"] > 0
        if args.require_cycle:
            require(report["cycle_verified"], "No complete measured cutoff/recovery cycle; repeat with hand rotation")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        try:
            if device.identified:
                device.request("disable")
                if original is not None:
                    device.request("configure", common_configuration(original))
                report["final"] = device.request("status")
        finally:
            try:
                device.close()
            finally:
                path = args.output.with_suffix(".summary.json")
                path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
                print(json.dumps({key: value for key, value in report.items()
                                  if key not in ("initial", "final")}), flush=True)


if __name__ == "__main__":
    main()
