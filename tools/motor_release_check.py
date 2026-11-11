#!/usr/bin/env python3
"""Run one bounded on-board FreeRTOS drive/release test (diagnostic build only)."""
import argparse
import json
import math
import time
from pathlib import Path
from knobctl import Knob


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--mode", type=int, choices=range(17), default=1)
    parser.add_argument("--velocity", type=float, default=50)
    parser.add_argument("--current", type=float, default=.15)
    parser.add_argument("--drive-ms", type=int, default=300)
    parser.add_argument("--observe-ms", type=int, default=2500)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--require-cycle", action="store_true")
    parser.add_argument("--max-tail-speed", type=float,
                        help="Fail if peak speed in the last 500 ms exceeds this rad/s (unloaded settling check)")
    args = parser.parse_args()
    if args.max_tail_speed is not None and (not math.isfinite(args.max_tail_speed) or args.max_tail_speed <= 0):
        parser.error("--max-tail-speed must be finite and positive")
    request = dict(mode=args.mode, velocity=args.velocity, current=args.current,
                   drive_ms=args.drive_ms, observe_ms=args.observe_ms)
    device = Knob(args.port)
    device.serial.timeout = .005
    report = {"passed": False, "request": request, "samples": [],
              "columns": ["elapsed_us", "angle", "velocity", "target", "iq",
                          "id_current", "haptic_scale", "voltage_q", "phase_peak", "flags", "velocity_fast", "voltage_d",
                          "adc_raw_a", "adc_raw_b", "sample_start_tick", "sample_end_tick",
                          "hold_phase", "hold_reference", "hold_correction", "hold_feedforward"]}
    try:
        initial = device.request("status")
        report["initial"] = initial
        if initial["state"] != "idle" or not initial["calibration"]["valid"]:
            raise RuntimeError("Idle calibrated board required")
        if "diagnostic" not in initial:
            raise RuntimeError("CONFIG_KNOB_DIAGNOSTICS build required")
        device.request("diagnose", **request)
        deadline = time.monotonic() + (args.drive_ms + args.observe_ms) / 1000 + 5
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
        samples = report["samples"]
        if samples:
            tail = [row for row in samples if row[0] >= samples[-1][0] - 500000]
            report["tail_peak_speed"] = max(abs(row[2]) for row in tail)
            report["tail_angle_span"] = max(row[1] for row in tail) - min(row[1] for row in tail)
        if args.max_tail_speed is not None:
            if not samples or args.observe_ms < 1000:
                raise RuntimeError("Settling check needs samples and at least 1000 ms observation")
            if report["tail_peak_speed"] > args.max_tail_speed:
                raise RuntimeError(f"Did not settle: tail peak {report['tail_peak_speed']:.3f} rad/s")
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
