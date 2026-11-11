#!/usr/bin/env python3
"""Exercise all mode IDs at <=80 mA, verify rejected updates, stop and restore.

This checks protocol integration and bounded live operation; it does not measure
hand feel, physical torque, or frictionlessness. No pulses or NVS writes occur.
The output is created exclusively before connecting, preserving earlier evidence.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path

from knobctl import Knob, common_configuration


NAMES = ["Free", "Detents", "Bounded", "Spring", "Custom", "Unbounded smooth",
         "Bounded 0-10", "Multi-rev smooth", "On/off strong", "Return to center",
         "Fine smooth", "Fine detents", "Coarse strong", "Coarse weak",
         "Magnetic detents", "Center detents", "Frictionless"]
FRICTIONLESS_FIELDS = ("friction", "viscous", "scale", "hold_kp", "hold_ki")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def run_check(args):
    report = {"passed": False, "checks": [], "events": [], "modes": [],
              "rejections": [], "errors": [], "cleanup_errors": [],
              "request": {"port": args.port, "dwell_seconds": .2,
                          "minimum_snapshots": 3, "current_cap_a": .08}}
    device = initial = original = None
    restore_needed = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        def checkpoint():
            output.seek(0)
            json.dump(report, output, indent=2, ensure_ascii=False, allow_nan=False)
            output.write("\n")
            output.truncate()
            output.flush()

        def request(op, config=None, **parameters):
            if op == "configure" and config is not None:
                config = common_configuration(config)
            event = {"op": op, "monotonic": time.monotonic()}
            if config is not None:
                event["config"] = copy.deepcopy(config)
            if parameters:
                event["parameters"] = parameters
            report["events"].append(event)
            try:
                snapshot = device.request(op, config, **parameters)
                event["snapshot"] = copy.deepcopy(snapshot)
                return snapshot
            except BaseException as error:
                event["error"] = str(error) or type(error).__name__
                raise

        def unchanged_calibration(snapshot):
            require(snapshot["calibration"] == initial["calibration"], "Calibration changed")
            require(snapshot.get("calibration_quality") == initial.get("calibration_quality"),
                    "Calibration quality changed")
            require(snapshot.get("app_elf_sha256") == initial.get("app_elf_sha256"), "Firmware changed")
            require(snapshot["uptime_ms"] >= initial["uptime_ms"], "Device restarted")

        def healthy(snapshot, expected_state):
            unchanged_calibration(snapshot)
            require(snapshot["fault"] == "none", "Device fault: " + snapshot["fault"])
            require(snapshot["state"] == expected_state, "Unexpected state: " + snapshot["state"])
            require(not snapshot.get("sampling_restart_required", False), "Sampling restart required")

        def metadata(snapshot, mode):
            require(snapshot["config"]["mode"] == mode, "Mode readback mismatch")
            require(snapshot.get("mode_count") == len(NAMES), "Mode count mismatch")
            require(snapshot.get("mode_name") == NAMES[mode], "Mode name mismatch")
            require(snapshot.get("reference_preset") == (mode - 4 if mode >= 4 else -1),
                    "Reference preset mapping mismatch")
            require(snapshot.get("modes") == NAMES, "Mode catalogue mismatch")

        def reject(name, bad_config, expected_error):
            before = request("status")
            healthy(before, "idle")
            rejected = request("configure", bad_config, allow_error=True)
            report["rejections"].append({"name": name, "expected_error": expected_error,
                                          "reply": rejected})
            require(rejected.get("ok") is False and rejected.get("error") == expected_error,
                    name + " was not explicitly rejected")
            for snapshot in (rejected, request("status")):
                healthy(snapshot, "idle")
                require(snapshot["config"] == before["config"], name + " changed config")
                require(snapshot["duty"] == [0, 0, 0], name + " enabled output")

        try:
            checkpoint()
            device = Knob(args.port)
            device.serial.timeout = .005
            initial = request("status")
            original = copy.deepcopy(initial["config"])
            report["initial"] = initial
            report["backup"] = {"config": copy.deepcopy(original),
                                "calibration": copy.deepcopy(initial["calibration"]),
                                "calibration_quality": copy.deepcopy(initial.get("calibration_quality"))}
            checkpoint()  # Backup reaches disk before the first mutation.
            healthy(initial, "idle")
            require(initial["calibration"]["valid"], "Calibrated board required")
            require("frictionless" in original, "Firmware lacks frictionless configuration")
            metadata(initial, original["mode"])
            current = min(.08, original["current_limit"])
            require(math.isfinite(current) and current > 0, "Invalid current budget")
            report["current_limit_a"] = current
            # Rejection checks must also recover if buggy firmware accepts one.
            restore_needed = True

            for mode in (-1, 17):
                invalid = copy.deepcopy(original)
                invalid["mode"] = mode
                reject("mode_" + str(mode), invalid, "mode_invalid")
            invalid_values = (("friction", -.001, "frictionless_friction_range"),
                              ("friction", .8001, "frictionless_friction_range"),
                              ("viscous", .0201, "frictionless_viscous_range"),
                              ("scale", -.001, "frictionless_scale_range"),
                              ("scale", 10.001, "frictionless_scale_range"),
                              ("hold_kp", 1.001, "frictionless_pi_range"),
                              ("hold_ki", 5.001, "frictionless_pi_range"))
            for field, value, error in invalid_values:
                invalid = copy.deepcopy(original)
                invalid["frictionless"][field] = value
                reject(field + "_" + str(value), invalid, error)
            invalid = copy.deepcopy(original)
            invalid["frictionless"] = []
            reject("frictionless_not_object", invalid, "frictionless_config_required")
            for field in FRICTIONLESS_FIELDS:
                invalid = copy.deepcopy(original)
                del invalid["frictionless"][field]
                reject("missing_" + field, invalid, "frictionless_field_missing")
            report["checks"].append("invalid_updates_rejected_without_mutation")

            temporary = copy.deepcopy(original)
            temporary.update(mode=0, damping=0, current_limit=current)
            # Use a distinct valid sentinel: a default-valued config would not
            # detect an older client's omission accidentally restoring defaults.
            sentinel = copy.deepcopy(temporary)
            sentinel["frictionless"]["scale"] = .61 if math.isclose(
                original["frictionless"]["scale"], .73, rel_tol=0, abs_tol=1e-6) else .73
            restore_needed = True  # A timed-out configure may still have applied.
            configured = request("configure", sentinel)
            healthy(configured, "idle")
            legacy = copy.deepcopy(configured["config"])
            del legacy["frictionless"]
            legacy_reply = request("configure", legacy)
            healthy(legacy_reply, "idle")
            require(legacy_reply["config"] == configured["config"], "Legacy omission changed frictionless settings")
            report["legacy_client"] = legacy_reply
            report["checks"].append("legacy_client_preserves_frictionless")
            configured = request("configure", temporary)
            healthy(configured, "idle")
            temporary = copy.deepcopy(configured["config"])
            request("enable")
            active = device.wait_active()
            report["enable_active"] = active
            healthy(active, "active")

            for mode in range(len(NAMES)):
                config = copy.deepcopy(temporary)
                config["mode"] = mode
                entry = {"mode": mode, "name": NAMES[mode], "snapshots": []}
                report["modes"].append(entry)
                configured = request("configure", config)
                entry["configured"] = configured
                healthy(configured, "active")
                metadata(configured, mode)
                if mode in (10, 11):
                    expected = original.get("reference_modes", {}).get(str(mode), {}).get("position", 127)
                    require(configured["position"] == expected, "Fine preset did not start at its configured initial position")
                started = time.monotonic()
                while True:
                    snapshot = request("status")
                    entry["snapshots"].append(snapshot)
                    healthy(snapshot, "active")
                    metadata(snapshot, mode)
                    require(math.isclose(snapshot["config"]["current_limit"], current, rel_tol=1e-6, abs_tol=1e-7),
                            "Temporary current limit changed")
                    require(isinstance(snapshot["target"], (float, int)) and math.isfinite(snapshot["target"])
                            and abs(snapshot["target"]) <= current + 1e-5, "Target exceeds temporary current limit")
                    elapsed = time.monotonic() - started
                    if elapsed >= .2 and len(entry["snapshots"]) >= 3:
                        break
                    time.sleep(.05)
                entry.update(passed=True, elapsed_seconds=elapsed)
                checkpoint()
                print(json.dumps({"mode": mode, "name": NAMES[mode], "passed": True,
                                  "snapshots": len(entry["snapshots"]), "elapsed_seconds": elapsed}), flush=True)
            report["checks"].append("all_17_modes_active_and_current_bounded")
            report["execution_passed"] = True
        except BaseException as error:
            report["errors"].append(str(error) or type(error).__name__)
        finally:
            if device is not None:
                stopped = None
                try:
                    stopped = request("disable")
                    report["stopped"] = stopped
                except BaseException as error:
                    report["cleanup_errors"].append("disable: " + (str(error) or type(error).__name__))
                    try:
                        stopped = request("status")
                    except BaseException as error:
                        report["cleanup_errors"].append("status after disable: " + str(error))
                if restore_needed and original is not None:
                    try:
                        require(stopped is not None and stopped["state"] in ("idle", "fault"),
                                "Cannot restore configuration while output state is unknown or active")
                        report["restored"] = request("configure", original)
                    except BaseException as error:
                        report["cleanup_errors"].append("restore: " + (str(error) or type(error).__name__))
                try:
                    final = request("status")
                    report["final"] = final
                    require(initial is not None, "Initial identity unavailable")
                    healthy(final, "idle")
                    require(final["duty"] == [0, 0, 0], "Final duty is not zero")
                    require(final["config"] == original, "Original configuration was not restored")
                    report["cleanup_passed"] = True
                except BaseException as error:
                    report["cleanup_errors"].append("final: " + (str(error) or type(error).__name__))
                try:
                    device.close()
                except BaseException as error:
                    report["cleanup_errors"].append("close: " + (str(error) or type(error).__name__))
                report["unparsed_lines"] = getattr(device, "unparsed_lines", [])
            report["passed"] = bool(report.get("execution_passed") and report.get("cleanup_passed")
                                    and not report["errors"] and not report["cleanup_errors"])
            checkpoint()
    print(json.dumps({"passed": report["passed"], "modes_checked": len(report["modes"]),
                      "rejections_checked": len(report["rejections"]), "errors": report["errors"],
                      "cleanup_errors": report["cleanup_errors"]}), flush=True)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args(argv)


if __name__ == "__main__":
    raise SystemExit(0 if run_check(parse_args())["passed"] else 1)
