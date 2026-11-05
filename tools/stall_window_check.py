#!/usr/bin/env python3
"""Bounded positive d-axis current injection; preserve calibration and RAM settings.

This uses a stationary stator vector captured from the initial rotor d axis;
diagnostic Id/Iq refer to that fixed frame. Requested current is not evidence
that the winding reached that current.
Firmware owns the ramp, motion guard and hard deadline, including if USB is lost.
"""
import argparse
import json
import math
import statistics
import time
from pathlib import Path

from knobctl import Knob

COUNTERS = ("control_cycles", "rejected_samples", "sampling_recoveries", "timing_skips",
            "encoder_timing_skips", "missed_samples", "pwm_deferred", "pwm_full_duty_cycles")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def summarize(run):
    first, last = run["before"], run["completion"]
    delta = {k: last[k] - first[k] for k in COUNTERS}
    reasons = {k: last["adc_reject_counts"][k] - first["adc_reject_counts"][k]
               for k in first["adc_reject_counts"]}
    require(all(v >= 0 for v in [*delta.values(), *reasons.values()]), "Counters reset during test")
    active = [s for s in run["samples"] if s["diagnostic"]["stage"] == 2]
    valid = [s for s in active if s["current_valid"]]
    # Keep the latter half of the actual drive period, excluding at least 200ms
    # when possible. USB averages describe observations, not a high-rate trace.
    tail = [s for s in valid if active and s["uptime_ms"] >=
            active[0]["uptime_ms"] + max(200, run["drive_ms"] / 2)]
    elapsed = (last["uptime_ms"] - first["uptime_ms"]) / 1000
    count = delta["control_cycles"] + delta["rejected_samples"]
    mean_id = statistics.mean(s["id_current"] for s in tail) if tail else None
    mean_target = statistics.mean(s["target_d"] for s in tail) if tail else None
    saturated = sum(math.hypot(s["voltage_d"], s["voltage_q"]) >=
                    .98 * s["effective_voltage_limit"] for s in tail)
    near_ceiling = sum(max(s["duty"]) >= .95 * s["pwm_duty_ceiling"] for s in active)
    stimulus_verified = len(tail) >= 2 and mean_target > .025 and mean_id > .02
    drive_delta = {k: active[-1][k] - active[0][k] for k in COUNTERS} if len(active) >= 2 else {}
    drive_elapsed = (active[-1]["uptime_ms"] - active[0]["uptime_ms"]) / 1000 if len(active) >= 2 else 0
    drive_count = drive_delta.get("control_cycles", 0) + drive_delta.get("rejected_samples", 0)
    return {
        "requested_d_current": run["requested_current"], "drive_ms": run["drive_ms"],
        "elapsed_seconds": elapsed, "counter_delta": delta, "rejection_reasons": reasons,
        "effective_control_hz": delta["control_cycles"] / elapsed,
        "reject_ratio": delta["rejected_samples"] / count if count else 1,
        "drive_snapshots": len(active), "valid_drive_snapshots": len(valid),
        "drive_elapsed_seconds": drive_elapsed, "drive_counter_delta": drive_delta,
        "drive_control_hz": drive_delta["control_cycles"] / drive_elapsed if drive_elapsed > 0 else 0,
        "drive_reject_ratio": drive_delta["rejected_samples"] / drive_count if drive_count > 0 else 1,
        "plateau_snapshots": len(tail),
        "plateau_mean_id": mean_id,
        "plateau_min_id": min((s["id_current"] for s in tail), default=None),
        "plateau_max_id": max((s["id_current"] for s in tail), default=None),
        "plateau_mean_target_d": mean_target,
        "plateau_mean_abs_iq": statistics.mean(abs(s["iq"]) for s in tail) if tail else None,
        "voltage_saturated_snapshots": saturated,
        "snapshot_peak_duty": max((max(s["duty"]) for s in active), default=0),
        "near_duty_ceiling_snapshots": near_ceiling,
        "stimulus_verified": stimulus_verified,
        "sampling_window_stressed": stimulus_verified and near_ceiling >= 2,
        "voltage_saturation_reached": stimulus_verified and saturated >= 2,
        "requested_current_reached": stimulus_verified and mean_id >= .9 * run["requested_current"],
        "diagnostic_peak_phase_current": last["diagnostic"]["peak_current"],
        "snapshot_peak_abs_iq": max((abs(s["iq"]) for s in valid), default=0),
        "peak_speed": last["diagnostic"]["peak_speed"],
        "fault": last["fault"], "diagnostic_stage": last["diagnostic"]["stage"],
    }


def run_check(args):
    report = {"passed": False, "runs": [], "cleanup_errors": [],
              "purpose": "Fixed stator-frame d current / sampling window test; not a torque calibration"}
    device = original = calibration = None
    changed = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x", encoding="utf-8") as output:
        try:
            device = Knob(args.port)
            device.serial.timeout = .005
            initial = device.request("status")
            report["initial"] = initial
            require(initial["state"] == "idle" and initial["fault"] == "none", "Idle, fault-free device required")
            require(initial["calibration"]["valid"] and "diagnostic" in initial and "target_d" in initial,
                    "D-axis diagnostic firmware and valid calibration required")
            original, calibration = initial["config"].copy(), initial["calibration"].copy()
            changed = True
            report["configured"] = device.request("configure", {**original, "mode": 0, "damping": 0,
                                                                  "current_limit": max(args.currents)})
            for index, current in enumerate(args.currents):
                seconds = args.final_seconds if index == len(args.currents) - 1 else args.seconds
                drive_ms = round(seconds * 1000)
                run = {"requested_current": current, "drive_ms": drive_ms,
                       "before": device.request("status"), "samples": []}
                report["runs"].append(run)
                run["started"] = device.request("diagnose", axis="d", mode=0, velocity=0,
                                                current=current, drive_ms=drive_ms, observe_ms=500)
                deadline = time.monotonic() + seconds + 5
                next_log = time.monotonic() + 5
                while True:
                    s = device.request("status")
                    run["samples"].append(s)
                    require(s["calibration"] == calibration, "Calibration changed during test")
                    require(s["diagnostic"].get("axis") == "d", "Firmware did not select d-axis injection")
                    require(s["diagnostic"].get("reference") == "fixed_start_angle", "Expected stationary test reference")
                    require(math.isclose(s["effective_voltage_limit"],
                                         s["bus_voltage"] * s["pwm_duty_ceiling"] / math.sqrt(3),
                                         rel_tol=1e-5), "PI voltage capacity does not match PWM ceiling")
                    require(s["fault"] == "none", "Device fault: " + s["fault"])
                    require(not s["sampling_restart_required"], "ADC transport requires restart")
                    if not s["diagnostic"]["busy"]:
                        run["completion"] = s
                        require(s["diagnostic"]["stage"] == 4 and s["state"] == "idle", "Diagnostic did not complete")
                        break
                    require(time.monotonic() < deadline, "Diagnostic exceeded hard host deadline")
                    if time.monotonic() >= next_log:
                        print(json.dumps({"requested": current, "target_d": s["target_d"],
                                          "id": s["id_current"], "iq": s["iq"], "duty": s["duty"],
                                          "phase_window": s["adc_reject_counts"]["phase_window"],
                                          "recoveries": s["sampling_recoveries"]}), flush=True)
                        next_log += 5
                    time.sleep(.01)
                run["summary"] = summarize(run)
                summary = run["summary"]
                run["passed"] = (summary["stimulus_verified"] and
                                 summary["drive_reject_ratio"] < args.max_reject_ratio and
                                 summary["drive_control_hz"] > args.min_control_hz and
                                 summary["rejection_reasons"]["transport"] == 0 and
                                 summary["counter_delta"]["sampling_recoveries"] == 0)
                print(json.dumps({"passed": run["passed"], **summary}), flush=True)
                # Check actual device state during the cooldown rather than
                # leaving a long sleeping host unable to notice a fault.
                cool_until = time.monotonic() + 1
                while time.monotonic() < cool_until:
                    s = device.request("status")
                    require(s["state"] == "idle" and s["fault"] == "none", "Cooldown is not idle")
                    time.sleep(.05)
            report["passed"] = all(run["passed"] for run in report["runs"])
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
        finally:
            if device is not None:
                if device.identified:
                    report["stop_attempt_errors"] = []
                    for _ in range(2):
                        try:
                            report["disabled"] = device.request("disable")
                            break
                        except Exception as error:
                            report["stop_attempt_errors"].append(str(error))
                    stopped = False
                    try:
                        deadline = time.monotonic() + 2
                        while True:
                            s = device.request("status")
                            if not s.get("diagnostic", {}).get("busy", False):
                                break
                            require(time.monotonic() < deadline, "Diagnostic cleanup exceeded deadline")
                            time.sleep(.02)
                        require(s["state"] in ("idle", "fault") and s["duty"] == [0, 0, 0], "Output did not stop")
                        stopped = True
                    except Exception as error:
                        report["cleanup_errors"].append("Confirm stop: " + str(error))
                    if changed and stopped:
                        try:
                            report["restored"] = device.request("configure", original)
                        except Exception as error:
                            report["cleanup_errors"].append("Restore: " + str(error))
                    try:
                        final = device.request("status")
                        report["final"] = final
                        require(final["state"] in ("idle", "fault") and final["duty"] == [0, 0, 0], "Final output did not stop")
                        if original is not None:
                            require(final["config"] == original and final["calibration"] == calibration,
                                    "Original settings/calibration not restored")
                        require(final["state"] == "idle" and final["fault"] == "none",
                                "Final device state is not fault-free idle")
                    except Exception as error:
                        report["cleanup_errors"].append("Final status: " + str(error))
                try:
                    device.serial.close()
                except Exception as error:
                    report["cleanup_errors"].append("Close: " + str(error))
            report["passed"] &= not report.get("error") and not report["cleanup_errors"]
            json.dump(report, output, ensure_ascii=False, indent=2, allow_nan=False)
            output.write("\n")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--currents", default="0.3,0.8,1.5,2,3")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--final-seconds", type=float, default=30)
    parser.add_argument("--max-reject-ratio", type=float, default=.01)
    parser.add_argument("--min-control-hz", type=float, default=4900)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    try:
        args.currents = [float(v) for v in args.currents.split(",")]
    except ValueError:
        parser.error("--currents requires comma-separated amperes")
    if not args.currents or any(not math.isfinite(v) or not .05 <= v <= 3 for v in args.currents):
        parser.error("Currents must be in [0.05, 3] A")
    if any(not math.isfinite(v) or not .05 <= v <= 30 for v in (args.seconds, args.final_seconds)):
        parser.error("Drive durations must be in [0.05, 30] seconds")
    if not math.isfinite(args.max_reject_ratio) or not 0 < args.max_reject_ratio <= 1:
        parser.error("Invalid rejection threshold")
    if not math.isfinite(args.min_control_hz) or args.min_control_hz <= 0:
        parser.error("Invalid control-rate threshold")
    if args.output.exists():
        parser.error("Use a new output path")
    report = run_check(args)
    print(json.dumps({"passed": report["passed"], "error": report.get("error"),
                      "cleanup_errors": report["cleanup_errors"], "output": str(args.output)}), flush=True)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
