#!/usr/bin/env python3
"""Bounded drive/release or no-stimulus check; stop and restore RAM, never save NVS.

Mode 16 is the reference frictionless controller. Mode 0 with zero damping is
the zero-Iq comparison while moving; its existing rest quieting is unchanged.
`passed` describes execution, sampling and initial motion. `hold_passed` is a
separate, explicit speed-retention criterion, never a claim of physical friction
or touch sensing. `--require-hold` also requires that criterion for success.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path

from knobctl import Knob, common_configuration


COLUMNS = ["elapsed_us", "angle", "velocity", "target", "iq", "id_current",
           "haptic_scale", "voltage_q", "phase_peak", "flags", "velocity_fast",
           "voltage_d", "adc_raw_a", "adc_raw_b", "sample_start_tick",
           "sample_end_tick", "hold_phase", "hold_reference", "hold_correction",
           "hold_feedforward"]
COUNTERS = ("control_cycles", "rejected_samples", "timing_skips", "adc_timing_skips",
            "encoder_timing_skips", "reused_samples", "missed_samples",
            "sampling_recoveries", "pwm_deferred")
REASONS = ("transport", "trigger", "pwm_tracking", "burst_timing", "phase_window", "pair_change")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def time_mean(rows, column):
    """Trapezoidal mean uses captured timestamps, not host USB poll cadence."""
    if not rows:
        return None
    if len(rows) == 1:
        return rows[0][column]
    elapsed = rows[-1][0] - rows[0][0]
    if elapsed <= 0:
        return None
    return sum((b[0] - a[0]) * (a[column] + b[column]) * .5
               for a, b in zip(rows, rows[1:])) / elapsed


def summarize_trace(samples, args):
    result = {"hold_passed": False, "initial_motion_passed": False,
              "criteria": {}, "hold_criteria": {}}
    if not samples:
        result["criteria"]["trace_present"] = False
        return result
    valid = all(len(row) >= 16 and all(isinstance(x, (float, int)) and math.isfinite(x)
                                      for x in row) for row in samples)
    ordered = valid and all(b[0] > a[0] for a, b in zip(samples, samples[1:]))
    result["criteria"].update(trace_present=True, finite_trace=valid, ordered_trace=ordered)
    if not valid or not ordered:
        return result
    drive_end = args.drive_ms * 1000
    total_end = (args.drive_ms + args.observe_ms) * 1000
    release = [row for row in samples if row[0] >= drive_end and (int(row[9]) & 255) == 3]
    drive_tail = [row for row in samples if drive_end - min(drive_end // 2, 500000) <= row[0] < drive_end]
    start = [row for row in release if row[0] <= drive_end + min(250000, args.observe_ms * 250)]
    tail = [row for row in release if row[0] >= total_end - min(500000, args.observe_ms * 500)]
    gap = max((b[0] - a[0] for a, b in zip(samples, samples[1:])), default=0)
    covered = bool(release and start and tail and samples[-1][0] >= total_end - max(50000, 2 * gap))
    result["criteria"].update(release_captured=covered,
        target_bounded=all(abs(row[3]) <= args.runtime_current + 1e-5 for row in samples))
    # Runtime compensation and the initial diagnostic driver have separate
    # explicit budgets; the restored user limit bounds both.
    if not covered:
        return result
    start_speed, tail_speed = time_mean(start, 2), time_mean(tail, 2)
    direction = 1 if args.velocity > 0 else -1
    initial_motion = start_speed * direction >= args.min_start_speed
    tolerance = max(.5, abs(start_speed) * args.max_drift_ratio)
    hold_criteria = {
        "initial_motion": initial_motion,
        "tail_same_direction": tail_speed * direction >= args.min_start_speed,
        "speed_retention": abs(tail_speed - start_speed) <= tolerance,
        "no_stop_or_reversal": min(row[2] * direction for row in release) > .2,
        "sufficient_observation": args.observe_ms >= 1000,
    }
    has_hold = all(len(row) >= 20 for row in release)
    if args.mode == 16:
        hold_criteria["hold_telemetry"] = has_hold
        hold_criteria["tracking_at_end"] = has_hold and all(int(row[16]) == 2 for row in tail)
        if has_hold:
            result.update(hold_reference_start=time_mean(start, 17),
                          hold_reference_tail=time_mean(tail, 17),
                          hold_correction_tail_a=time_mean(tail, 18),
                          hold_feedforward_tail_a=time_mean(tail, 19))
    result.update(initial_motion_passed=initial_motion, hold_criteria=hold_criteria,
                  hold_passed=all(hold_criteria.values()), initial_speed=start_speed,
                  tail_speed=tail_speed, velocity_drift=tail_speed - start_speed,
                  velocity_drift_fraction=(tail_speed - start_speed) / abs(start_speed)
                      if start_speed else None,
                  retention_tolerance=tolerance,
                  displacement=release[-1][1] - release[0][1],
                  release_seconds=(release[-1][0] - release[0][0]) * 1e-6,
                  release_peak_speed=max(abs(row[2]) for row in release),
                  release_peak_target_a=max(abs(row[3]) for row in release),
                  release_mean_target_a=time_mean(release, 3),
                  release_mean_iq_a=time_mean(release, 4),
                  release_peak_phase_current_a=max(row[8] for row in release),
                  drive_tail_speed=time_mean(drive_tail, 2),
                  drive_tail_target_a=time_mean(drive_tail, 3),
                  drive_tail_iq_a=time_mean(drive_tail, 4),
                  release_mean_absolute_iq_error_a=time_mean(
                      [[row[0], abs(row[4] - row[3] * args.direction)] for row in release], 1),
                  max_capture_gap_us=gap)
    result["criteria"]["initial_motion"] = initial_motion
    return result


def sampling_delta(first, last):
    if any(key not in first or key not in last for key in COUNTERS):
        return {"passed": False, "error": "Sampling counters missing"}
    if any(key not in first.get("adc_reject_counts", {}) or
           key not in last.get("adc_reject_counts", {}) for key in REASONS):
        return {"passed": False, "error": "Sampling reject reasons missing"}
    delta = {key: last[key] - first[key] for key in COUNTERS}
    reasons = {key: last["adc_reject_counts"][key] - first["adc_reject_counts"][key]
               for key in REASONS}
    elapsed = (last["uptime_ms"] - first["uptime_ms"]) * .001
    valid = elapsed > 0 and all(value >= 0 for value in (*delta.values(), *reasons.values()))
    count = delta["control_cycles"] + delta["rejected_samples"]
    ratio = delta["rejected_samples"] / count if valid and count > 0 else None
    criteria = {"counter_interval_valid": valid and count > 0,
                "rejection_below_one_percent": ratio is not None and ratio < .01,
                "transport_integrity": reasons["transport"] == 0,
                "pair_coherent": reasons["pair_change"] == 0,
                "feedback_uninterrupted": delta["sampling_recoveries"] == 0,
                "no_duplicate_notifications": delta["reused_samples"] == 0,
                "no_missing_feedback": delta["missed_samples"] == 0}
    return {"passed": all(criteria.values()), "criteria": criteria,
            "counter_delta": delta, "adc_reject_delta": reasons,
            "elapsed_seconds": elapsed, "reject_ratio": ratio,
            "effective_control_hz": delta["control_cycles"] / elapsed if valid else None}


def sampling_windows(report, args):
    """Keep full counters and evaluate the USB-observed active window separately.

    Global counters also advance during idle/preparation and supervisor cleanup.
    A repeated notification is skipped by firmware; `reused_samples` does not
    mean the PI integrated a frame twice. Neither that counter nor sequence
    gaps can be assigned to active operation from idle endpoints alone.
    """
    full = sampling_delta(report["baseline"], report["completion"])
    full["scope"] = "including_preparation_and_supervisor_cleanup"
    report["sampling_full"] = full
    statuses = report["rest_samples"] if args.rest_seconds else report["status_samples"]

    def active(snapshot):
        return snapshot["state"] == "active" and (args.rest_seconds > 0 or
            snapshot.get("diagnostic", {}).get("stage") in (2, 3))

    indices = [index for index, snapshot in enumerate(statuses) if active(snapshot)]
    if len(indices) < 2:
        return {"passed": False, "scope": "observed_active_window",
                "error": "Fewer than two active sampling snapshots"}
    first_index, last_index = indices[0], indices[-1]
    first, last = statuses[first_index], statuses[last_index]
    result = sampling_delta(first, last)
    result.update(scope="observed_active_window", first_status_index=first_index,
                  last_status_index=last_index, baseline=first, last=last,
                  boundary_note="Counters before first active snapshot and after last active snapshot remain unassigned; this is not a complete energized-interval measurement.")
    if "criteria" not in result or "criteria" not in full:
        result["passed"] = False
        return result
    expected = args.rest_seconds or (args.drive_ms + args.observe_ms) * .001
    fraction = result["elapsed_seconds"] / expected
    result.update(requested_active_seconds=expected, observed_fraction=fraction,
                  unobserved_seconds=max(0., expected - result["elapsed_seconds"]))
    result["criteria"].update(
        continuous_observed_active=all(active(row) for row in statuses[first_index:last_index + 1]),
        active_coverage_at_least_85_percent=.85 <= fraction <= 1.05)
    if not args.rest_seconds:
        result["criteria"].update(drive_and_release_observed=first["diagnostic"]["stage"] == 2 and
                                  last["diagnostic"]["stage"] == 3)
    # Do not hide a real recovery, transport fault, incoherent pair, or elevated
    # rejection ratio in the unobserved edges merely by trimming the interval.
    full_guards = {key: full["criteria"][key] for key in (
        "counter_interval_valid", "rejection_below_one_percent", "transport_integrity",
        "pair_coherent", "feedback_uninterrupted")}
    result["full_interval_guards"] = full_guards
    result["criteria"]["full_interval_guards"] = all(full_guards.values())
    result["outside_observed_window_counter_delta"] = {
        key: full["counter_delta"][key] - result["counter_delta"][key] for key in COUNTERS}
    result["passed"] = all(result["criteria"].values())
    return result


def summarize_rest(samples, args):
    if not samples:
        return {"passed": False, "error": "No rest samples"}
    angle_span = max(row["angle"] for row in samples) - min(row["angle"] for row in samples)
    peak_speed = max(abs(row["velocity"]) for row in samples)
    peak_target = max(abs(row["target"]) for row in samples)
    criteria = {"no_self_start": angle_span <= .05 and peak_speed <= .3,
                "zero_target": peak_target <= 1e-7,
                "continuous_active_state": all(row["state"] == "active" for row in samples)}
    if args.mode == 16:
        criteria["zero_iq_loop_enabled"] = all(row.get("quiet_output") is False for row in samples)
    return {"passed": all(criteria.values()), "criteria": criteria,
            "angle_span": angle_span, "peak_speed": peak_speed, "peak_target": peak_target}


def run_check(args):
    report = {"passed": False, "hold_passed": False, "samples": [], "rest_samples": [], "status_samples": [],
              "columns": COLUMNS, "cleanup_errors": [], "transport_errors": [],
              "request": {key: getattr(args, key) for key in
                          ("mode", "velocity", "current", "runtime_current", "drive_ms", "observe_ms",
                           "rest_seconds", "min_start_speed", "max_drift_ratio", "require_hold")}}
    device = original = initial = None
    restore_needed = False
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive create preserves every prior failure and avoids touching hardware
    # if the requested evidence path already exists.
    with args.output.open("x", encoding="utf-8") as output:
        def request(op, config=None, **parameters):
            if op == "configure" and config is not None:
                config = common_configuration(config)
            try:
                return device.request(op, config, **parameters)
            except Exception as error:
                report["transport_errors"].append({"op": op, "error": str(error)})
                raise

        def check(snapshot):
            require(snapshot["fault"] == "none", "Device fault: " + snapshot["fault"])
            require(snapshot["calibration"] == initial["calibration"], "Calibration changed")
            require(snapshot.get("calibration_quality") == initial.get("calibration_quality"), "Calibration quality changed")
            require(snapshot.get("app_elf_sha256") == initial.get("app_elf_sha256"), "Firmware changed")
            require(snapshot["uptime_ms"] >= initial["uptime_ms"], "Device reset during test")
            require(not snapshot.get("sampling_restart_required", False), "Sampling restart required")

        try:
            device = Knob(args.port)
            device.serial.timeout = .005
            initial = request("status")
            report["initial"] = initial
            require(initial["state"] == "idle" and initial["fault"] == "none", "Idle fault-free board required")
            require(initial["calibration"]["valid"], "Valid calibration required")
            require(args.rest_seconds > 0 or "diagnostic" in initial, "CONFIG_KNOB_DIAGNOSTICS build required")
            original = copy.deepcopy(initial["config"])
            args.direction = initial["calibration"]["direction"]
            require(args.runtime_current <= original["current_limit"] + 1e-7, "Runtime budget exceeds existing current limit")
            config = copy.deepcopy(original)
            config.update(mode=args.mode, damping=0, current_limit=args.runtime_current)
            for name in ("friction", "viscous", "scale", "hold_kp", "hold_ki"):
                value = getattr(args, name)
                if value is not None:
                    require("frictionless" in config, "Firmware lacks frictionless configuration")
                    config["frictionless"][name] = value
            restore_needed = True  # configure may succeed even if its reply times out.
            configured = request("configure", config)
            report["configured"] = configured
            check(configured)
            baseline = request("status")
            report["baseline"] = baseline
            check(baseline)
            if args.rest_seconds > 0:
                request("enable")
                active = device.wait_active()
                check(active)
                require(abs(active["velocity"]) < .3, "Rest test requires stationary rotor")
                until = time.monotonic() + args.rest_seconds
                while True:
                    snapshot = request("status")
                    report["rest_samples"].append(snapshot)
                    check(snapshot)
                    require(snapshot["state"] == "active", "Rest test left active state")
                    if time.monotonic() >= until:
                        break
                    time.sleep(.025)
                report["rest"] = summarize_rest(report["rest_samples"], args)
                report["completion"] = request("disable")
                report["execution_passed"] = report["rest"]["passed"]
            else:
                request("diagnose", mode=args.mode, velocity=args.velocity, current=args.current,
                        drive_ms=args.drive_ms, observe_ms=args.observe_ms)
                until = time.monotonic() + (args.drive_ms + args.observe_ms) * .001 + 3
                previous_stage = -1
                while True:
                    snapshot = request("status")
                    report["status_samples"].append(snapshot)
                    report["completion"] = snapshot
                    check(snapshot)
                    diagnostic = snapshot["diagnostic"]
                    if diagnostic["stage"] != previous_stage:
                        print(json.dumps({"state": snapshot["state"], "fault": snapshot["fault"],
                                          **diagnostic}), flush=True)
                        previous_stage = diagnostic["stage"]
                    if not diagnostic["busy"]:
                        break
                    require(time.monotonic() < until, "On-board diagnostic deadline exceeded")
                    time.sleep(.025)
                require(snapshot["state"] == "idle" and diagnostic["stage"] == 4, "Diagnostic did not complete")
                count = diagnostic["count"]
                require(isinstance(count, int) and 0 < count <= 2048, "Invalid diagnostic sample count")
                while len(report["samples"]) < count:
                    response = request("diagnostic_read", offset=len(report["samples"]))
                    check(response)
                    page = response["diagnostic"]
                    require(page.get("offset") == len(report["samples"]) and page.get("samples"), "Invalid diagnostic page")
                    report["samples"].extend(page["samples"])
                    require(len(report["samples"]) <= count, "Diagnostic page exceeds frozen count")
                report["trace"] = summarize_trace(report["samples"], args)
                report["hold_passed"] = report["trace"]["hold_passed"]
                report["execution_passed"] = all(report["trace"]["criteria"].values())
            report["sampling"] = sampling_windows(report, args)
            report["measurement_complete"] = True
        except BaseException as error:
            report["error"] = str(error) or type(error).__name__
        finally:
            if device is not None:
                try:
                    stopped = request("disable")
                    # The diagnostic supervisor restores its config after its
                    # motor-task stop. Let it finish before restoring our backup.
                    deadline = time.monotonic() + 2
                    while stopped.get("diagnostic", {}).get("busy", False):
                        require(time.monotonic() < deadline, "Diagnostic cleanup still busy")
                        time.sleep(.025)
                        stopped = request("status")
                    report["stopped"] = stopped
                    if restore_needed:
                        require(stopped["state"] in ("idle", "fault"), "Cannot restore while energized")
                        report["restored"] = request("configure", original)
                    final = request("status")
                    report["final"] = final
                    if initial is not None:
                        check(final)
                        require(final["state"] == "idle" and final["duty"] == [0, 0, 0], "Final output not disabled")
                        if original is not None:
                            require(final["config"] == original, "Original RAM configuration not restored")
                    report["cleanup_passed"] = initial is not None
                except BaseException as error:
                    report["cleanup_errors"].append(str(error) or type(error).__name__)
                try:
                    device.close()
                except BaseException as error:
                    report["cleanup_errors"].append("close: " + str(error))
                report["unparsed_lines"] = getattr(device, "unparsed_lines", [])
            report["passed"] = bool(report.get("measurement_complete") and report.get("execution_passed")
                and report.get("sampling", {}).get("passed") and report.get("cleanup_passed")
                and not report.get("error") and not report["cleanup_errors"]
                and (not args.require_hold or report["hold_passed"]))
            json.dump(report, output, indent=2, allow_nan=False)
            output.write("\n")
    print(json.dumps({"passed": report["passed"], "hold_passed": report["hold_passed"],
                      "trace": report.get("trace"), "rest": report.get("rest"),
                      "error": report.get("error"), "cleanup_errors": report["cleanup_errors"]}), flush=True)
    return report


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", type=int, choices=(0, 16), default=16)
    parser.add_argument("--velocity", type=float, default=10)
    parser.add_argument("--current", type=float, default=.1)
    parser.add_argument("--runtime-current", type=float,
                        help="Runtime current budget .02..0.8 A; defaults to --current and never exceeds the initial device setting")
    parser.add_argument("--drive-ms", type=int, default=2000)
    parser.add_argument("--observe-ms", type=int, default=5000)
    parser.add_argument("--rest-seconds", type=float, default=0, help="Instead of drive/release, observe without a stimulus (0..30 s)")
    parser.add_argument("--min-start-speed", type=float, help="Minimum signed initial release speed; default max(0.6, |requested|/4)")
    parser.add_argument("--max-drift-ratio", type=float, default=.2, help="Speed-retention tolerance, with a 0.5 rad/s floor")
    parser.add_argument("--require-hold", action="store_true")
    for name in ("friction", "viscous", "scale", "hold-kp", "hold-ki"):
        parser.add_argument("--" + name, type=float)
    args = parser.parse_args(argv)
    if not math.isfinite(args.velocity) or not 1 <= abs(args.velocity) <= 30:
        parser.error("--velocity magnitude must be 1..30 rad/s")
    if not math.isfinite(args.current) or not .02 <= args.current <= .15:
        parser.error("--current must be .02..0.15 A")
    if args.runtime_current is None:
        args.runtime_current = args.current
    if not math.isfinite(args.runtime_current) or not .02 <= args.runtime_current <= .8:
        parser.error("--runtime-current must be .02..0.8 A")
    if not 50 <= args.drive_ms <= 3000 or not 500 <= args.observe_ms <= 10000:
        parser.error("--drive-ms must be 50..3000 and --observe-ms 500..10000")
    if not math.isfinite(args.rest_seconds) or not 0 <= args.rest_seconds <= 30:
        parser.error("--rest-seconds must be 0..30")
    if args.require_hold and args.rest_seconds:
        parser.error("--require-hold cannot be used with --rest-seconds")
    if not args.rest_seconds and args.current > args.runtime_current:
        parser.error("--current must not exceed --runtime-current")
    if args.min_start_speed is None:
        args.min_start_speed = max(.6, abs(args.velocity) * .25)
    if not math.isfinite(args.min_start_speed) or not .2 < args.min_start_speed <= 30:
        parser.error("--min-start-speed must be >0.2 and <=30 rad/s")
    if not math.isfinite(args.max_drift_ratio) or not 0 < args.max_drift_ratio <= 1:
        parser.error("--max-drift-ratio must be >0 and <=1")
    for name in ("friction", "viscous", "scale", "hold_kp", "hold_ki"):
        value = getattr(args, name)
        if value is not None and (not math.isfinite(value) or value < 0):
            parser.error("--" + name.replace("_", "-") + " must be finite and nonnegative")
    return args


if __name__ == "__main__":
    raise SystemExit(0 if run_check(parse_args())["passed"] else 1)
