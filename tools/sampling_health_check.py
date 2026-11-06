#!/usr/bin/env python3
"""Measure ADC rejection and effective control rate; stop and restore RAM on exit.

The short pulse command is bounded by firmware. This check never saves NVS and
does not claim an external current calibration or sustained-load verification.
"""
import argparse
import json
import math
import time
from pathlib import Path

from knobctl import Knob


COUNTERS = ("control_cycles", "rejected_samples", "timing_skips", "adc_timing_skips",
            "encoder_timing_skips", "reused_samples", "missed_samples",
            "sampling_recoveries", "pwm_deferred", "pwm_full_duty_cycles")
REASONS = ("transport", "trigger", "pwm_tracking", "burst_timing", "phase_window", "pair_change")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def summarize(report, args):
    first, last = report.get("baseline"), report.get("last")
    result = {"samples": len(report["samples"]), "pulse_attempts": len(report["pulses"]),
              "pulse_accepted": sum(p["ok"] for p in report["pulses"]),
              "pulse_rejected": sum(not p["ok"] for p in report["pulses"]),
              "transport": report.get("initial", {}).get("transport"),
              "transport_errors": report["transport_errors"], "fault": report.get("fault", "none"),
              "thresholds": report.get("thresholds", {"max_reject_ratio": args.max_reject_ratio,
                                                       "min_control_hz": args.min_control_hz})}
    if first is None or last is None:
        result["criteria"] = {"measurement_complete": False}
        return result
    elapsed = (last["uptime_ms"] - first["uptime_ms"]) / 1000
    delta = {key: last[key] - first[key] for key in COUNTERS}
    reasons = {key: last["adc_reject_counts"][key] - first["adc_reject_counts"][key]
               for key in REASONS}
    count = delta["control_cycles"] + delta["rejected_samples"]
    counter_valid = elapsed > 0 and all(v >= 0 for v in (*delta.values(), *reasons.values()))
    hz = delta["control_cycles"] / elapsed if counter_valid else None
    ratio = delta["rejected_samples"] / count if counter_valid and count > 0 else None
    result.update(elapsed_seconds=elapsed, counter_delta=delta, adc_reject_counts=reasons,
                  effective_control_hz=hz, reject_ratio=ratio)
    result["criteria"] = {
        "measurement_complete": report.get("measurement_complete", False) and elapsed >= args.seconds,
        "counters_valid": counter_valid,
        "fault_free": result["fault"] == "none",
        "transport_ok": not report["transport_errors"] and reasons["transport"] == 0,
        "pair_coherent": reasons["pair_change"] == 0,
        "feedback_uninterrupted": delta["sampling_recoveries"] == 0,
        "reject_ratio": ratio is not None and ratio < args.max_reject_ratio,
        "control_hz": hz is not None and hz > result["thresholds"]["min_control_hz"],
        "pulse_commands_accepted": (args.pulse_interval == 0 or
                                    result["pulse_attempts"] > 0 and result["pulse_rejected"] == 0),
    }
    return result


def run_check(args):
    report = {"passed": False, "samples": [], "startup": [], "pulses": [],
              "transport_errors": [], "command_errors": [], "cleanup_errors": [], "fault": "none",
              "requested": {key: getattr(args, key) for key in
                            ("port", "seconds", "mode", "pulse_interval", "max_reject_ratio", "min_control_hz")}}
    device = original = calibration = None
    restore_needed = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Reserve a new report before touching the device; never replace evidence.
    with args.output.open("x", encoding="utf-8") as output:
        def request(op, config=None, **parameters):
            allow_error = parameters.pop("allow_error", False)
            try:
                reply = device.request(op, config, allow_error=True, **parameters)
            except Exception as error:
                report["transport_errors"].append({"op": op, "error": str(error)})
                raise
            if not reply["ok"]:
                report["command_errors"].append({"op": op, "error": reply.get("error", "Command rejected")})
                require(allow_error, reply.get("error", "Command rejected"))
            return reply

        def check(snapshot, active=False):
            if snapshot["fault"] != "none":
                report["fault"] = snapshot["fault"]
                raise RuntimeError("Device fault: " + snapshot["fault"])
            require(snapshot["calibration"] == calibration, "Calibration changed")
            require(not snapshot["sampling_restart_required"], "ADC transport requires restart")
            if active:
                require(snapshot["state"] == "active", "Device left active state")

        def record(snapshot):
            report["samples"].append(snapshot)
            previous = report.get("last", report["baseline"])
            report["last"] = snapshot
            require(snapshot["uptime_ms"] >= previous["uptime_ms"], "Device uptime reset")
            require(snapshot["app_elf_sha256"] == report["baseline"]["app_elf_sha256"], "Firmware changed")
            check(snapshot, active=True)

        try:
            device = Knob(args.port)
            device.serial.timeout = .005
            initial = request("status")
            report["initial"] = initial
            report["fault"] = initial["fault"]
            require(initial["state"] == "idle" and initial["fault"] == "none", "Idle, fault-free device required")
            require(initial["calibration"]["valid"], "Existing valid calibration required")
            nominal_hz = initial.get("control_nominal_hz")
            require(isinstance(nominal_hz, (int, float)) and math.isfinite(nominal_hz) and nominal_hz > 0,
                    "Firmware lacks a valid nominal control rate")
            minimum_hz = args.min_control_hz if args.min_control_hz is not None else .98 * nominal_hz
            report["thresholds"] = {"control_nominal_hz": nominal_hz, "min_control_hz": minimum_hz,
                                    "max_reject_ratio": args.max_reject_ratio}
            original, calibration = initial["config"].copy(), initial["calibration"].copy()
            require(all(key in initial for key in COUNTERS) and
                    all(key in initial.get("adc_reject_counts", {}) for key in REASONS),
                    "Firmware lacks sampling-health counters")
            check(initial)
            if args.mode is not None:
                restore_needed = True  # A timeout may still have applied configure.
                report["configured"] = request("configure", {**original, "mode": args.mode})
                check(report["configured"])
            enabled_at = time.monotonic()
            report["enabled"] = request("enable")
            check(report["enabled"])
            while True:
                snapshot = request("status")
                report["startup"].append(snapshot)
                check(snapshot)
                require(time.monotonic() - enabled_at <= 2, "Enable did not become active within 2 seconds")
                if snapshot["state"] == "active":
                    break
                time.sleep(.01)
            warm_until = time.monotonic() + .5
            while time.monotonic() < warm_until:
                snapshot = request("status")
                report["startup"].append(snapshot)
                check(snapshot, active=True)
                time.sleep(.005)
            baseline = request("status")
            check(baseline, active=True)
            report["baseline"] = report["last"] = baseline
            measured_at = time.monotonic()
            next_pulse, next_log = 0.0, 10.0
            while True:
                snapshot = request("status")
                record(snapshot)
                elapsed = (snapshot["uptime_ms"] - baseline["uptime_ms"]) / 1000
                if elapsed >= args.seconds:
                    report["measurement_complete"] = True
                    break
                require(time.monotonic() - measured_at < args.seconds + 5, "Device measurement clock stalled")
                if args.pulse_interval and elapsed >= next_pulse:
                    pulse = request("pulse", allow_error=True)
                    report["pulses"].append({"elapsed_seconds": elapsed, "ok": bool(pulse["ok"]),
                                             "error": pulse.get("error"), "reply": pulse})
                    record(pulse)
                    if not pulse["ok"]:
                        print(json.dumps({"pulse_rejected": pulse.get("error"), "t": elapsed}), flush=True)
                    next_pulse = elapsed + args.pulse_interval
                if elapsed >= next_log:
                    print(json.dumps(summarize(report, args)), flush=True)
                    next_log = elapsed + 10
                time.sleep(.005)
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
        finally:
            if device is not None:
                if device.identified:
                    stopped = False
                    report["stop_attempt_errors"] = []
                    for _ in range(2):
                        try:
                            stopped_reply = request("disable", allow_error=True)
                            report["disabled"] = stopped_reply
                            stopped = stopped_reply["state"] in ("idle", "fault") and stopped_reply["duty"] == [0, 0, 0]
                            if stopped:
                                break
                            report["stop_attempt_errors"].append("Output not stopped")
                        except Exception as error:
                            report["stop_attempt_errors"].append(str(error))
                    if not stopped:
                        report["cleanup_errors"].append("Disable could not be confirmed")
                    if stopped and restore_needed:
                        try:
                            report["restored"] = request("configure", original)
                        except Exception as error:
                            report["cleanup_errors"].append("Restore: " + str(error))
                    try:
                        final = request("status")
                        report["final"] = final
                        require(final["state"] in ("idle", "fault") and final["duty"] == [0, 0, 0], "Final output is not stopped")
                        if original is not None:
                            require(final["config"] == original and final["calibration"] == calibration,
                                    "Original configuration/calibration not restored")
                        if final["fault"] != "none":
                            report["fault"] = final["fault"]
                    except Exception as error:
                        report["cleanup_errors"].append("Final verification: " + str(error))
                try:
                    device.serial.close()
                except Exception as error:
                    report["cleanup_errors"].append("Close: " + str(error))
            report["summary"] = summarize(report, args)
            report["passed"] = (not report.get("error") and not report["cleanup_errors"] and
                                all(report["summary"]["criteria"].values()))
            json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", type=int, choices=range(4), help="Optional RAM-only mode override; default preserves current mode")
    parser.add_argument("--pulse-interval", type=float, default=.4, help="Seconds between firmware-bounded pulses; 0 disables")
    parser.add_argument("--max-reject-ratio", type=float, default=.01)
    parser.add_argument("--min-control-hz", type=float,
                        help="Minimum effective control rate; defaults to 98%% of device control_nominal_hz")
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 0 < args.seconds <= 120:
        parser.error("--seconds must be in (0, 120]")
    if not math.isfinite(args.pulse_interval) or args.pulse_interval < 0:
        parser.error("--pulse-interval must be finite and nonnegative")
    if not math.isfinite(args.max_reject_ratio) or not 0 < args.max_reject_ratio <= 1:
        parser.error("--max-reject-ratio must be in (0, 1]")
    if args.min_control_hz is not None and (not math.isfinite(args.min_control_hz) or args.min_control_hz <= 0):
        parser.error("--min-control-hz must be finite and positive")
    if args.output.exists():
        parser.error("--output already exists; use a new report path")
    report = run_check(args)
    print(json.dumps({"passed": report["passed"], "summary": report["summary"],
                      "error": report.get("error"), "cleanup_errors": report["cleanup_errors"],
                      "output": str(args.output)}, ensure_ascii=False), flush=True)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
