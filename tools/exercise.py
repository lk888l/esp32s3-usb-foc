#!/usr/bin/env python3
"""Timed haptic exercise with recorded telemetry; always stops and restores config."""
import argparse
import json
import time
from pathlib import Path

from knobctl import Knob


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--mode", type=int, choices=range(4), default=1)
    parser.add_argument("--strength", type=float)
    parser.add_argument("--damping", type=float)
    parser.add_argument("--current-limit", type=float, help="Operating current limit in A, up to 1 A")
    parser.add_argument("--cycle-modes", action="store_true")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/exercise.jsonl"))
    args = parser.parse_args()
    if not 1 <= args.seconds <= 900:
        parser.error("--seconds must be between 1 and 900")
    device = Knob(args.port)
    original = None
    try:
        initial = device.request("status")
        if initial["state"] != "idle" or not initial["calibration"]["valid"]:
            raise RuntimeError("Clear faults and complete calibration before exercising")
        original = initial["config"]
        config = {**original, "mode": 0 if args.cycle_modes else args.mode}
        for key in ("strength", "damping", "current_limit"):
            value = getattr(args, key)
            if value is not None:
                config[key] = value
        device.request("configure", config)
        device.request("enable")
        time.sleep(0.25)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        started = time.monotonic()
        next_report = 0
        print(f"READY: mode {config['mode']}, strength {config['strength']}, {args.seconds} seconds", flush=True)
        with args.output.open("w") as output:
            while time.monotonic() - started < args.seconds:
                elapsed = time.monotonic() - started
                mode = min(3, int(elapsed * 4 / args.seconds)) if args.cycle_modes else args.mode
                if mode != config["mode"]:
                    config["mode"] = mode
                    device.request("configure", config)
                    print(f"MODE: {mode}", flush=True)
                sample = device.request("status")
                sample["test_elapsed_s"] = time.monotonic() - started
                output.write(json.dumps(sample) + "\n")
                output.flush()
                if sample["state"] != "active":
                    raise RuntimeError(f"Unexpected state: {sample['state']}, {sample['fault']}")
                if sample["test_elapsed_s"] >= next_report:
                    print(json.dumps({key: sample[key] for key in ("test_elapsed_s", "angle", "position", "iq", "target", "loop_us", "rejected_samples")}), flush=True)
                    next_report += 10
                time.sleep(0.04)
    finally:
        try:
            if device.identified:
                stopped = device.request("disable")
                print(f"STOP: {stopped['state']}, {stopped['fault']}", flush=True)
                if original is not None:
                    device.request("configure", original)
        finally:
            device.close()


if __name__ == "__main__":
    main()
