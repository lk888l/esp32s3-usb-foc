#!/usr/bin/env python3
"""Run one bounded on-board FreeRTOS drive/release test (diagnostic build only)."""
import argparse
import json
import time
from pathlib import Path
from knobctl import Knob


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--mode", type=int, choices=range(4), default=1)
    parser.add_argument("--velocity", type=float, default=50)
    parser.add_argument("--current", type=float, default=.15)
    parser.add_argument("--drive-ms", type=int, default=300)
    parser.add_argument("--observe-ms", type=int, default=2500)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-cycle", action="store_true")
    args = parser.parse_args()
    request = dict(mode=args.mode, velocity=args.velocity, current=args.current,
                   drive_ms=args.drive_ms, observe_ms=args.observe_ms)
    device = Knob(args.port)
    device.serial.timeout = .005
    report = {"passed": False, "request": request, "samples": [],
              "columns": ["elapsed_us", "angle", "velocity", "target", "iq",
                          "id_current", "haptic_scale", "voltage_q", "phase_peak", "flags", "velocity_fast"]}
    try:
        initial = device.request("status")
        report["initial"] = initial
        if initial["state"] != "idle" or not initial["calibration"]["valid"]:
            raise RuntimeError("Idle calibrated board required")
        if "diagnostic" not in initial:
            raise RuntimeError("CONFIG_KNOB_DIAGNOSTICS build required")
        device.request("diagnose", **request)
        deadline = time.monotonic() + 8
        previous_stage = -1
        while True:
            snapshot = device.request("status")
            diagnostic = snapshot["diagnostic"]
            if diagnostic["stage"] != previous_stage:
                print(json.dumps({"state": snapshot["state"], "fault": snapshot["fault"], **diagnostic}), flush=True)
                previous_stage = diagnostic["stage"]
            if not diagnostic["busy"]:
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("On-board test did not stop within deadline")
            time.sleep(.025)
        report["completion"] = snapshot
        count = diagnostic["count"]
        while len(report["samples"]) < count:
            page = device.request("diagnostic_read", offset=len(report["samples"]))["diagnostic"]
            if page.get("offset") != len(report["samples"]) or not page.get("samples"):
                raise RuntimeError("Invalid diagnostic sample page")
            report["samples"].extend(page["samples"])
        if snapshot["state"] != "idle" or snapshot["fault"] != "none" or diagnostic["stage"] != 4:
            raise RuntimeError("Test aborted: " + snapshot["fault"])
        if snapshot["config"] != initial["config"]:
            raise RuntimeError("Original configuration was not restored")
        if args.require_cycle and not (diagnostic["muted_samples"] > 0 and diagnostic["recovered"]):
            raise RuntimeError("No complete on-board mute/recovery cycle")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        try:
            device.close()
        finally:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n")
            print(json.dumps({"passed": report["passed"], "samples": len(report["samples"]),
                "request": request, "result": report.get("completion", {}).get("diagnostic"),
                "error": report.get("error")}), flush=True)


if __name__ == "__main__":
    main()
