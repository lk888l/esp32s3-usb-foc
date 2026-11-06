#!/usr/bin/env python3
"""Exercise calibration rejection, cancellation, and expected saturation guards.

Requires an idle device with verified bidirectional calibration. Preserves its
configuration, accepted calibration and quality. The one deliberately provoked
calibration_voltage_saturated fault is explicitly cleared after proving its
latched stop behavior. Unexpected faults and transport errors are never cleared.
No settings are saved; every request/response is retained in a new evidence file.
"""
import argparse
import copy
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path

from knobctl import Knob
from calibration_check import (BUSY_STATES, require, require_verified_quality,
                               snapshot_integrity_errors, stopped)


CALIBRATION = {"current": .25, "max_current": .25, "voltage": 3.0}
EXPECTED_FAULT = "calibration_voltage_saturated"


def run_check(args):
    report = {"passed": False, "time_utc": datetime.now(timezone.utc).isoformat(),
              "calibration_parameters": CALIBRATION.copy(), "checks": [], "responses": [],
              "transport_errors": [], "integrity_errors": [], "cleanup_errors": [],
              "nvs_saved": False, "expected_fault_cleared": False}
    device = baseline = None
    last_uptime = None
    action = "initial"
    opened = time.monotonic()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        def request(op, allow_error=False, **parameters):
            entry = {"action": action, "op": op, "parameters": copy.deepcopy(parameters),
                     "host_elapsed_seconds": time.monotonic() - opened}
            report["responses"].append(entry)
            try:
                snapshot = device.request(op, allow_error=True, **parameters)
            except Exception as error:
                entry["transport_error"] = str(error)
                report["transport_errors"].append({"action": action, "op": op, "error": str(error)})
                raise
            entry["reply"] = snapshot
            require(snapshot.get("ok") is True or allow_error,
                    f"{op}: {snapshot.get('error', 'Command rejected')}")
            return snapshot

        def integrity(snapshot, require_stopped=False):
            nonlocal last_uptime
            errors = snapshot_integrity_errors(snapshot, baseline, last_uptime,
                baseline.get("calibration") if baseline else None,
                baseline.get("calibration_quality") if baseline else None, require_stopped)
            uptime = snapshot.get("uptime_ms")
            if isinstance(uptime, (int, float)) and math.isfinite(uptime) and uptime >= 0:
                last_uptime = uptime if last_uptime is None else max(last_uptime, uptime)
            return errors

        def check(snapshot, allowed_fault=None, require_stopped=False):
            errors = integrity(snapshot, require_stopped)
            if errors:
                report["integrity_errors"].append({"action": action, "errors": errors})
                raise RuntimeError("; ".join(errors))
            fault = snapshot.get("fault")
            require(fault == "none" or fault == allowed_fault, "Unexpected device fault: " + str(fault))
            if fault != "none":
                require(snapshot["state"] == "fault" and stopped(snapshot), "Fault did not stop output")
            return snapshot

        def wait_active():
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                snapshot = check(request("status"))
                if snapshot["state"] == "active":
                    return snapshot
                require(snapshot["state"] == "offsets", "Unexpected enable state: " + snapshot["state"])
                time.sleep(args.period)
            raise TimeoutError("Enable did not become active within 2 seconds")

        def passed(name):
            report["checks"].append(name)
            print("PASS: " + name, flush=True)

        def write_report():
            output.seek(0)
            json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.write("\n")
            output.truncate()
            output.flush()

        def start_calibration(parameters):
            # A verification cancellation can stop during an angle ramp. Let
            # the unloaded rotor settle before the next zero-current capture.
            quiet_since = time.monotonic()
            deadline = quiet_since + 5
            anchor = None
            while time.monotonic() < deadline:
                ready = check(request("status"), require_stopped=True)
                require(ready["state"] == "idle", "Idle state required before calibration")
                angle = ready.get("angle")
                require(isinstance(angle, (int, float)) and math.isfinite(angle), "Invalid rotor angle")
                if anchor is None or abs(angle - anchor) > .0035:
                    anchor, quiet_since = angle, time.monotonic()
                if time.monotonic() - quiet_since >= .3:
                    break
                time.sleep(args.period)
            else:
                raise TimeoutError("Rotor did not settle before calibration within 5 seconds")
            snapshot = check(request("calibrate", **parameters))
            require(snapshot["state"] in BUSY_STATES, "Calibration did not enter a running state")
            quality = snapshot.get("calibration_report", {})
            require(quality.get("method") == "bidirectional_v1" and not quality.get("passed", True),
                    "Calibration did not start a fresh report")
            return snapshot

        def cancel_and_check():
            snapshot = check(request("disable"), require_stopped=True)
            require(snapshot["state"] == "idle", "Cancellation did not return to idle")
            quality = snapshot.get("calibration_report", {})
            require(quality.get("cancelled") is True and quality.get("passed") is False,
                    "Firmware did not record cancellation")
            return snapshot

        try:
            device = Knob(args.port)
            device.serial.timeout = .005
            baseline = copy.deepcopy(request("status"))
            report["initial"] = baseline
            check(baseline, require_stopped=True)
            require(baseline["state"] == "idle", "Idle device required")
            require(baseline.get("calibration", {}).get("valid") is True, "Valid accepted calibration required")
            require_verified_quality(baseline)
            require(baseline["config"]["current_limit"] >= CALIBRATION["max_current"],
                    "Existing runtime current limit is below the 0.25 A guard-test request")
            write_report()

            rejections = (
                ("reject_current_above_0.25A", {"current": .26, "max_current": .26}, "calibration_current_range"),
                ("reject_max_below_current", {"current": .16, "max_current": .08}, "calibration_current_range"),
                ("reject_voltage_above_3V", {"voltage": 3.1}, "calibration_voltage_range"),
                ("reject_non_numeric_current", {"current": "bad"}, "calibration_parameters"),
            )
            for action, parameters, expected_error in rejections:
                snapshot = check(request("calibrate", allow_error=True, **parameters), require_stopped=True)
                require(snapshot["ok"] is False and snapshot.get("error") == expected_error,
                        "Unexpected calibration rejection: " + str(snapshot.get("error")))
                require(snapshot["state"] == "idle", "Rejected request changed idle state")
                passed(action)

            action = "reject_calibration_while_active"
            check(request("enable"))
            wait_active()
            snapshot = check(request("calibrate", allow_error=True, **CALIBRATION))
            require(snapshot["ok"] is False and snapshot.get("error") == "idle_required",
                    "Active calibration was not rejected with idle_required")
            require(snapshot["state"] == "active", "Rejected calibration changed active state")
            snapshot = check(request("disable"), require_stopped=True)
            require(snapshot["state"] == "idle", "Disable did not return to idle")
            passed(action)
            write_report()

            action = "cancel_during_offsets"
            start_calibration(CALIBRATION)
            time.sleep(.05)
            snapshot = check(request("status"))
            require(snapshot["state"] == "offsets" and snapshot["calibration_report"]["stage"] == "offsets",
                    "Offsets cancellation window was missed")
            report["offsets_cancelled"] = cancel_and_check()
            passed(action)
            write_report()

            action = "cancel_during_verification"
            started = time.monotonic()
            start_calibration(CALIBRATION)
            previous_stage = None
            while time.monotonic() - started < args.timeout:
                snapshot = check(request("status"))
                stage = snapshot.get("calibration_report", {}).get("stage")
                if stage != previous_stage:
                    print(json.dumps({"action": action, "stage": stage,
                                      "elapsed_seconds": round(time.monotonic() - started, 3)}), flush=True)
                    previous_stage = stage
                if stage == "verify":
                    report["before_verify_cancel"] = snapshot
                    report["verify_cancelled"] = cancel_and_check()
                    break
                require(snapshot["state"] in BUSY_STATES, "Calibration ended before verification cancellation")
                time.sleep(args.period)
            else:
                raise TimeoutError("Verification stage was not reached within the host deadline")
            passed(action)
            write_report()

            action = "low_voltage_fault_preserves_calibration"
            started = time.monotonic()
            start_calibration({**CALIBRATION, "voltage": .6})
            while time.monotonic() - started < args.timeout:
                snapshot = check(request("status"), allowed_fault=EXPECTED_FAULT)
                if snapshot["fault"] == EXPECTED_FAULT:
                    report["expected_fault"] = snapshot
                    break
                require(snapshot["state"] in BUSY_STATES, "Low-voltage calibration ended without the expected fault")
                time.sleep(args.period)
            else:
                raise TimeoutError("Expected voltage-saturation fault was not observed")
            require(snapshot["state"] == "fault" and stopped(snapshot), "Expected fault did not stop output")
            passed(action)

            action = "disable_preserves_expected_fault"
            latched = check(request("disable"), allowed_fault=EXPECTED_FAULT, require_stopped=True)
            require(latched["state"] == "fault" and latched["fault"] == EXPECTED_FAULT,
                    "Disable unexpectedly cleared the saturation fault")
            report["latched_after_disable"] = latched
            passed(action)

            # This explicit clear is reached only after the exact injected fault
            # and its latched, zero-output state were observed and preserved.
            action = "clear_only_verified_expected_fault"
            cleared = check(request("clear"), require_stopped=True)
            require(cleared["state"] == "idle", "Expected-fault clear did not return to idle")
            report["cleared"] = cleared
            report["expected_fault_cleared"] = True
            deadline = time.monotonic() + 2
            while time.monotonic() < deadline:
                snapshot = check(request("status"), require_stopped=True)
                require(snapshot["state"] == "idle", "Device left idle after clear")
                if (snapshot.get("bus_voltage_valid") is True and snapshot.get("adc_sample_valid") is True
                        and not snapshot.get("sampling_restart_required", True) and snapshot.get("encoder_error") == 0):
                    report["ready_after_clear"] = snapshot
                    break
                time.sleep(args.period)
            else:
                raise TimeoutError("Device did not become ready after the expected-fault clear")
            passed(action)
            report["measurement_complete"] = True
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
        finally:
            action = "cleanup"
            if device is not None:
                if device.identified:
                    stop_errors = []
                    for _ in range(2):
                        try:
                            snapshot = request("disable")
                            report["disabled"] = snapshot
                            for error in integrity(snapshot):
                                report["cleanup_errors"].append("Disable: " + error)
                            require(stopped(snapshot), "Disable left output energized")
                            break
                        except Exception as error:
                            stop_errors.append(str(error))
                    else:
                        report["cleanup_errors"].append("Disable could not be confirmed")
                    report["stop_attempt_errors"] = stop_errors
                    try:
                        final = request("status")
                        report["final"] = final
                        for error in integrity(final, require_stopped=True):
                            report["cleanup_errors"].append("Final status: " + error)
                        if final.get("fault") != "none":
                            report["cleanup_errors"].append("Latched fault retained: " + str(final.get("fault")))
                    except Exception as error:
                        report["cleanup_errors"].append(str(error))
                try:
                    # All stop attempts above are logged; closing the port does
                    # not issue an extra unrecorded motor command.
                    device.serial.close()
                except Exception as error:
                    report["cleanup_errors"].append("Close: " + str(error))
                report["unparsed_lines"] = getattr(device, "unparsed_lines", [])
            report["passed"] = (report.get("measurement_complete", False) and not report.get("error")
                                and not report["transport_errors"] and not report["integrity_errors"]
                                and not report["cleanup_errors"])
            write_report()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=130,
                        help="Host deadline per calibration in seconds (default: 130; firmware remains bounded to 121)")
    parser.add_argument("--period", type=float, default=.05, help="Status poll interval in seconds")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or not 1 <= args.timeout <= 150:
        parser.error("--timeout must be finite and in [1, 150] seconds")
    if not math.isfinite(args.period) or not .02 <= args.period <= .2:
        parser.error("--period must be finite and in [0.02, 0.2] seconds")
    report = run_check(args)
    print(json.dumps({"passed": report["passed"], "checks": report["checks"], "error": report.get("error"),
                      "cleanup_errors": report["cleanup_errors"], "output": str(args.output)}), flush=True)
    raise SystemExit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
