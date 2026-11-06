#!/usr/bin/env python3
"""Check bidirectional calibration repeatability or explicit cancellation.

Records complete snapshots and keeps the latest successful calibration in RAM.
Always stops output, preserves runtime configuration, and never saves NVS or
clears a fault. This checks firmware telemetry, not absolute current accuracy.
"""
import argparse
import copy
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

from knobctl import Knob


METHOD = "bidirectional_v1"
BUSY_STATES = ("offsets", "align", "sweep", "settle")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def wrap_signed(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def compare_calibrations(calibrations, reference=None):
    """Compare every run pair, including zero points straddling 0/2pi."""
    require(bool(calibrations), "No completed calibrations to compare")
    for calibration in calibrations:
        require(calibration.get("valid") is True and calibration.get("direction") in (-1, 1)
                and calibration.get("pole_pairs") == 7, "Invalid calibration result")
        zero = calibration.get("electrical_zero")
        require(isinstance(zero, (int, float)) and math.isfinite(zero)
                and 0 <= zero < math.tau, "Invalid electrical zero")
    first = calibrations[0]
    require(all(c["direction"] == first["direction"] and c["pole_pairs"] == first["pole_pairs"]
                for c in calibrations), "Direction or pole-pair count changed between runs")
    zeros = [c["electrical_zero"] for c in calibrations]
    pairs = [{"a": i + 1, "b": j + 1, "difference_rad": wrap_signed(zeros[j] - zeros[i])}
             for i in range(len(zeros)) for j in range(i + 1, len(zeros))]
    mean = math.atan2(sum(map(math.sin, zeros)), sum(map(math.cos, zeros))) % math.tau
    if mean >= math.tau:  # A tiny negative angle can round to exactly 2pi.
        mean = 0.0
    result = {"runs": len(zeros), "direction": first["direction"], "pole_pairs": first["pole_pairs"],
              "zero_mean_rad": mean, "zero_rms_rad": math.sqrt(sum(wrap_signed(z - mean) ** 2
                                                                               for z in zeros) / len(zeros)),
              "max_pairwise_difference_rad": max((abs(p["difference_rad"]) for p in pairs), default=0),
              "pairwise_differences": pairs,
              "differences_from_first_rad": [wrap_signed(z - zeros[0]) for z in zeros]}
    if reference and reference.get("valid"):
        result["initial_direction_matches"] = reference["direction"] == first["direction"]
        result["initial_pole_pairs_match"] = reference["pole_pairs"] == first["pole_pairs"]
        # An old zero is informational: recalibration may intentionally correct it.
        result["differences_from_initial_rad"] = [wrap_signed(z - reference["electrical_zero"])
                                                  for z in zeros]
    return result


def stopped(snapshot):
    return snapshot.get("state") in ("idle", "fault") and snapshot.get("duty") == [0, 0, 0]


def snapshot_integrity_errors(snapshot, initial, minimum_uptime=None, expected_calibration=None,
                              expected_quality=None, require_stopped=False):
    """Return every violation so cleanup cannot hide a reboot behind another error."""
    errors = []
    if require_stopped and not stopped(snapshot):
        errors.append("Output is not stopped")
    if initial is not None:
        if snapshot.get("config") != initial.get("config"):
            errors.append("Runtime configuration changed")
        if snapshot.get("app_elf_sha256") != initial.get("app_elf_sha256"):
            errors.append("Firmware identity changed")
        if snapshot.get("encoder") != initial.get("encoder"):
            errors.append("Encoder identity changed")
        if "reset_reason" in initial and snapshot.get("reset_reason") != initial["reset_reason"]:
            errors.append("Device reset reason changed")
    uptime = snapshot.get("uptime_ms")
    if not isinstance(uptime, (int, float)) or not math.isfinite(uptime) or uptime < 0:
        errors.append("Device uptime is invalid")
    elif minimum_uptime is not None and uptime < minimum_uptime:
        errors.append(f"Device restarted: uptime decreased from {minimum_uptime} to {uptime} ms")
    if expected_calibration is not None and snapshot.get("calibration") != expected_calibration:
        errors.append("Calibration differs from the last accepted result")
    if expected_quality is not None and snapshot.get("calibration_quality") != expected_quality:
        errors.append("Accepted calibration quality changed")
    return errors


def require_verified_quality(snapshot):
    quality = snapshot.get("calibration_quality", {})
    require(quality.get("verified") is True and quality.get("method") == METHOD,
            "Accepted calibration quality is not verified")
    require(quality.get("forward_points") == 29 and quality.get("reverse_points") == 29
            and quality.get("verification_points") == 6,
            "Accepted calibration quality lacks complete scans or six independent verification points")
    require(quality.get("phase_check_passed") is True, "Accepted calibration phase check did not pass")
    return quality


def run_check(args):
    request_parameters = {} if getattr(args, "firmware_defaults", False) else {
        "current": args.current, "max_current": args.max_current, "voltage": args.voltage}
    report = {"passed": False, "time_utc": datetime.now(timezone.utc).isoformat(),
              "request": request_parameters, "requested_cycles": args.cycles,
              "interrupt_after_seconds": args.interrupt_after, "timeout_seconds": args.timeout,
              "max_zero_spread_rad": args.max_zero_spread, "runs": [], "command_errors": [],
              "transport_errors": [], "integrity_errors": [], "cleanup_errors": [], "nvs_saved": False}
    device = original_config = expected_calibration = expected_quality = initial = None
    last_uptime = None
    completed = []
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve a fresh evidence file before sending any motor command.
    with args.output.open("x", encoding="utf-8") as output:
        def request(op, **parameters):
            try:
                snapshot = device.request(op, allow_error=True, **parameters)
            except Exception as error:
                report["transport_errors"].append({"op": op, "error": str(error)})
                raise
            if not snapshot["ok"]:
                report["command_errors"].append({"op": op, "error": snapshot.get("error")})
                raise RuntimeError(f"{op}: {snapshot.get('error', 'Command rejected')}")
            return snapshot

        def integrity_errors(snapshot, preserve_accepted=True, require_stopped=False):
            nonlocal last_uptime
            errors = snapshot_integrity_errors(snapshot, initial, last_uptime,
                expected_calibration if preserve_accepted else None,
                expected_quality if preserve_accepted else None, require_stopped)
            uptime = snapshot.get("uptime_ms")
            if isinstance(uptime, (int, float)) and math.isfinite(uptime) and uptime >= 0:
                last_uptime = uptime if last_uptime is None else max(last_uptime, uptime)
            return errors

        def check(snapshot, preserve_accepted=True):
            errors = integrity_errors(snapshot, preserve_accepted)
            if errors:
                report["integrity_errors"].append({"state": snapshot.get("state"), "errors": errors})
                raise RuntimeError("; ".join(errors))
            require(snapshot["fault"] == "none", "Device fault: " + snapshot["fault"])
            require(not snapshot.get("sampling_restart_required", False), "ADC transport requires restart")
            quality = snapshot.get("calibration_report", {})
            require(quality.get("method") == METHOD, "Bidirectional calibration firmware required")

        def record(run, snapshot, started):
            run["samples"].append({"host_elapsed_seconds": time.monotonic() - started, "snapshot": snapshot})
            completed_now = snapshot["state"] == "idle" and snapshot.get("calibration_report", {}).get("passed") is True
            check(snapshot, preserve_accepted=not completed_now)
            return snapshot["calibration_report"]

        def write_report():
            output.seek(0)
            json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.write("\n")
            output.truncate()
            output.flush()

        try:
            device = Knob(args.port)
            device.serial.timeout = .005
            initial = request("status")
            report["initial"] = initial
            original_config = initial["config"].copy()
            expected_calibration = initial["calibration"].copy()
            require(isinstance(initial.get("calibration_quality"), dict), "Firmware lacks accepted calibration quality")
            expected_quality = copy.deepcopy(initial["calibration_quality"])
            check(initial)
            require(initial["state"] == "idle" and stopped(initial), "Idle device with zero output required")
            require(args.max_current <= original_config["current_limit"],
                    "Calibration maximum exceeds the existing runtime current limit")
            write_report()
            for cycle in range(1 if args.interrupt_after is not None else args.cycles):
                run = {"cycle": cycle + 1, "passed": False, "samples": [],
                       "previous_calibration": expected_calibration.copy(),
                       "previous_calibration_quality": copy.deepcopy(expected_quality)}
                report["runs"].append(run)
                started = time.monotonic()
                accepted = request("calibrate", **request_parameters)
                run["accepted"] = accepted
                quality = record(run, accepted, started)
                require(accepted["state"] in BUSY_STATES and not quality["passed"],
                        "Calibration command did not start a fresh run")
                require(accepted["calibration"] == expected_calibration,
                        "Starting calibration discarded the previous result")
                previous_stage = None
                while True:
                    elapsed = time.monotonic() - started
                    require(elapsed < args.timeout, "Calibration did not finish within the host deadline")
                    if args.interrupt_after is not None and elapsed >= args.interrupt_after:
                        cancelled = request("disable")
                        quality = record(run, cancelled, started)
                        run["completion"] = cancelled
                        require(cancelled["state"] == "idle" and stopped(cancelled),
                                "Cancellation did not stop output")
                        require(quality.get("cancelled") is True and not quality["passed"],
                                "Cancellation was not recorded by firmware")
                        require(cancelled["calibration"] == expected_calibration,
                                "Cancellation changed the previous calibration")
                        run["elapsed_seconds"] = time.monotonic() - started
                        run["passed"] = True
                        report["interruption_verified"] = True
                        break
                    time.sleep(args.period)
                    snapshot = request("status")
                    quality = record(run, snapshot, started)
                    stage = quality["stage"]
                    if stage != previous_stage:
                        print(json.dumps({"cycle": cycle + 1, "elapsed_seconds": round(time.monotonic() - started, 3),
                                          "state": snapshot["state"], "stage": stage,
                                          "forward_points": quality.get("forward_points"),
                                          "reverse_points": quality.get("reverse_points"),
                                          "current": quality.get("current"),
                                          **{key: quality.get(key) for key in (
                                              "attempts", "direction_difference", "hysteresis_max", "residual_rms",
                                              "residual_max", "tracking_error")}}), flush=True)
                        previous_stage = stage
                    if snapshot["state"] == "idle":
                        require(args.interrupt_after is None, "Calibration finished before interruption was exercised")
                        run["completion"] = snapshot
                        require(stopped(snapshot), "Completed calibration left output energized")
                        require(quality["passed"] and not quality.get("cancelled", False),
                                "Calibration ended without passing quality checks")
                        require(snapshot["calibration"]["valid"], "Calibration result is invalid")
                        require(quality.get("forward_points") == 29 and quality.get("reverse_points") == 29,
                                "Calibration did not cover both complete scans")
                        require(quality.get("verification_points") == 6,
                                "Calibration did not complete six independent verification points")
                        accepted_quality = require_verified_quality(snapshot)
                        expected_calibration = snapshot["calibration"].copy()
                        expected_quality = copy.deepcopy(accepted_quality)
                        completed.append(expected_calibration)
                        report["repeatability"] = compare_calibrations(completed, initial["calibration"])
                        require(report["repeatability"]["max_pairwise_difference_rad"] <= args.max_zero_spread,
                                "Electrical-zero repeatability exceeds the configured tolerance")
                        run["elapsed_seconds"] = time.monotonic() - started
                        run["passed"] = True
                        print(json.dumps({"cycle": cycle + 1, "calibration": expected_calibration,
                                          "quality": quality, "repeatability": report["repeatability"]}), flush=True)
                        break
                    require(snapshot["state"] in BUSY_STATES, "Unexpected calibration state")
                    require(snapshot["calibration"] == expected_calibration,
                            "An unfinished calibration changed the accepted result")
                write_report()
            report["measurement_complete"] = True
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
        finally:
            if device is not None:
                report["unparsed_lines"] = getattr(device, "unparsed_lines", [])
            if device is not None:
                if device.identified:
                    stop_errors = []
                    for _ in range(2):
                        try:
                            snapshot = request("disable")
                            report["disabled"] = snapshot
                            for error in integrity_errors(snapshot):
                                report["cleanup_errors"].append("Disable: " + error)
                            require(stopped(snapshot), "Disable left nonzero output")
                            break
                        except Exception as error:
                            stop_errors.append(str(error))
                    else:
                        report["cleanup_errors"].append("Disable could not be confirmed")
                    report["stop_attempt_errors"] = stop_errors
                    try:
                        final = request("status")
                        report["final"] = final
                        for error in integrity_errors(final, require_stopped=True):
                            report["cleanup_errors"].append("Final status: " + error)
                        if final["fault"] != "none":
                            report["cleanup_errors"].append("Latched fault: " + final["fault"])
                    except Exception as error:
                        report["cleanup_errors"].append(str(error))
                try:
                    device.close()
                except Exception as error:
                    report["cleanup_errors"].append("Close: " + str(error))
                report["unparsed_lines"] = getattr(device, "unparsed_lines", [])
            report["passed"] = (report.get("measurement_complete", False) and not report.get("error")
                                and not report["cleanup_errors"] and not report["transport_errors"]
                                and not report["command_errors"])
            write_report()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--cycles", type=int, default=3, choices=range(1, 11))
    parser.add_argument("--current", type=float, default=.16)
    parser.add_argument("--max-current", type=float, default=.25)
    parser.add_argument("--voltage", type=float, default=3.0)
    parser.add_argument("--firmware-defaults", action="store_true",
                        help="Send calibrate without overrides to exercise firmware defaults")
    parser.add_argument("--timeout", type=float, default=130,
                        help="Host deadline in seconds (default: 130; firmware remains bounded to 121)")
    parser.add_argument("--period", type=float, default=.1)
    parser.add_argument("--max-zero-spread", type=float, default=.05,
                        help="Maximum pairwise electrical-zero difference in rad (default: .05)")
    parser.add_argument("--interrupt-after", type=float,
                        help="Instead of full runs, cancel one run after this many seconds and verify preservation")
    args = parser.parse_args()
    for key in ("current", "max_current", "voltage", "timeout", "period", "max_zero_spread"):
        if not math.isfinite(getattr(args, key)):
            parser.error(f"--{key.replace('_', '-')} must be finite")
    if not .02 <= args.current <= args.max_current <= .25:
        parser.error("Require 0.02 <= current <= max-current <= 0.25 A")
    if not .05 <= args.voltage <= 3:
        parser.error("--voltage must be in [0.05, 3] V")
    if not 1 <= args.timeout <= 300 or not .02 <= args.period <= .5:
        parser.error("Require timeout in [1, 300] s and period in [0.02, 0.5] s")
    if not 0 < args.max_zero_spread < math.pi:
        parser.error("--max-zero-spread must be in (0, pi) electrical rad")
    if args.interrupt_after is not None and (not math.isfinite(args.interrupt_after)
                                           or not 0 < args.interrupt_after < args.timeout):
        parser.error("--interrupt-after must be finite, positive, and below --timeout")
    report = run_check(args)
    print(json.dumps({"passed": report["passed"], "error": report.get("error"),
                      "cleanup_errors": report["cleanup_errors"], "repeatability": report.get("repeatability"),
                      "interruption_verified": report.get("interruption_verified", False),
                      "output": str(args.output)}), flush=True)
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
