#!/usr/bin/env python3
"""Bounded full-SVPWM checks; stop and restore RAM settings without saving NVS.

Reports actual HAL-latched duty counters, not just requested duty. Reaching full
modulation requires peak duty == 1 and new full-duty cycles during this check.
USB snapshots and software latch counters do not measure driver output voltage.
"""
import argparse
import json
import math
import time
from pathlib import Path

from knobctl import Knob, common_configuration


COUNTERS = ("pwm_full_duty_cycles", "timing_skips", "sampling_recoveries", "pwm_deferred")


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def check_snapshot(snapshot, calibration):
    require(snapshot["fault"] == "none", "Device fault: " + snapshot["fault"])
    require(snapshot["calibration"] == calibration, "Calibration changed during verification")
    require(snapshot.get("voltage_limit_automatic") is True and "voltage_limit" not in snapshot["config"],
            "Expected automatic voltage capacity without a configurable voltage limit")
    require(snapshot["bus_voltage_valid"], "No valid bus measurement")
    expected = snapshot["bus_voltage"] / math.sqrt(3)
    for key in ("max_voltage_limit", "available_voltage_limit", "effective_voltage_limit"):
        require(math.isclose(snapshot[key], expected, rel_tol=1e-5, abs_tol=1e-5),
                f"{key} does not follow Vbus / sqrt(3)")
    require(snapshot["pwm_hz"] == 20000 and snapshot["pwm_duty_ceiling"] == 1 and
            snapshot["pwm_duty_floor"] == 0, "Expected 20 kHz full-range SVPWM")
    require(0 <= snapshot["pwm_peak_duty"] <= 1, "Invalid latched duty telemetry")


def summarize(run):
    first, last = run["before"], run["after"]
    samples = run["samples"]
    valid = [s for s in samples if s.get("current_valid")]
    diagnostic = run.get("diagnostic_completion", {}).get("diagnostic", {})
    result = {
        "configured_current_limit": run["config"]["current_limit"],
        "peak_latched_duty": max(s["pwm_peak_duty"] for s in [first, last, *samples]),
        "counter_delta": {key: last[key] - first[key] for key in COUNTERS},
        "snapshot_peak_phase_current": max((max(map(abs, s["currents"])) for s in valid), default=0),
        "snapshot_peak_speed": max((abs(s["velocity"]) for s in samples), default=0),
        "snapshot_peak_voltage": max((math.hypot(s["voltage_d"], s["voltage_q"]) for s in samples), default=0),
        "diagnostic_peak_phase_current": diagnostic.get("peak_current"),
        "diagnostic_peak_speed": diagnostic.get("peak_speed"),
        "samples": len(samples),
    }
    require(all(delta >= 0 for delta in result["counter_delta"].values()),
            "HAL counters reset during a run; results cannot be compared")
    result["modulation_reached"] = (result["peak_latched_duty"] == 1 and
                                    result["counter_delta"]["pwm_full_duty_cycles"] > 0)
    return result


def run_check(args):
    report = {"passed": False, "modulation_reached": False, "runs": [], "cleanup_errors": [],
              "stress_gain": args.stress_gain, "requested_max_current": args.max_current,
              "evidence": "HAL TEZ-latched duty counters; not an external voltage or waveform measurement"}
    device = None
    original = calibration = None
    restore_needed = False
    started = time.monotonic()
    try:
        device = Knob(args.port)
        device.serial.timeout = .005
        initial = device.request("status")
        report["initial"] = initial
        require(initial["state"] == "idle" and initial["fault"] == "none", "Idle, fault-free device required")
        require(initial["calibration"]["valid"], "Existing valid motor calibration required")
        original, calibration = initial["config"].copy(), initial["calibration"].copy()
        check_snapshot(initial, calibration)
        require(all(key in initial for key in COUNTERS), "Required timing/latch counters are missing")
        maximum = min(args.max_current, original["current_limit"])
        report["effective_max_current"] = maximum
        currents = sorted({round(min(current, maximum), 6) for current in (.08, .25, maximum)})

        refused = device.request("configure", {**common_configuration(original), "voltage_limit": .8}, allow_error=True)
        report["retired_field_rejection"] = refused
        require(not refused["ok"] and refused["error"] == "voltage_limit_automatic",
                "Retired voltage_limit field was not explicitly rejected")
        check_snapshot(refused, calibration)
        require(refused["config"] == original and refused["state"] == "idle" and refused["duty"] == [0, 0, 0],
                "Refused voltage setting changed configuration or motor state")

        for index, current in enumerate(currents):
            config = {**original, "mode": 0, "damping": 0, "current_limit": current}
            if args.stress_gain:
                config.update(current_kp=20, current_ki=60)
            run = {"requested_config": config.copy(), "config": config, "samples": [], "before": device.request("status")}
            report["runs"].append(run)
            check_snapshot(run["before"], calibration)
            require(run["before"]["state"] == "idle", "Previous run did not stop")
            restore_needed = True  # A timeout may mean configure was accepted.
            configured = device.request("configure", common_configuration(config))
            check_snapshot(configured, calibration)
            # Compare subsequent restoration against the firmware's float32
            # roundtrip, rather than Python's original decimal request.
            config = configured["config"].copy()
            run["config"] = config

            def record(snapshot):
                run["samples"].append({"t": time.monotonic() - started, **snapshot})
                check_snapshot(snapshot, calibration)
                return snapshot

            # Mode 0 with zero damping has no persistent torque intent. The
            # pulse itself is bounded to 6 ms by firmware even if USB is lost.
            record(device.request("enable"))
            deadline = time.monotonic() + 2
            while True:
                snapshot = record(device.request("status"))
                if snapshot["state"] == "active":
                    break
                require(time.monotonic() < deadline, "Enable did not become active within 2 seconds")
                time.sleep(.02)
            for pulse in range(3):
                if pulse:
                    record(device.request("pulse"))
                until = time.monotonic() + .15
                while time.monotonic() < until:
                    snapshot = record(device.request("status"))
                    require(snapshot["state"] == "active", "Pulse run unexpectedly stopped")
                    time.sleep(.015)
            stopped = record(device.request("disable"))
            require(stopped["state"] == "idle" and stopped["duty"] == [0, 0, 0], "Pulse output did not stop")

            if "diagnostic" in initial:
                # Firmware's bounded velocity stimulus has its own 0.25 A
                # ceiling. The run's current budget can still match the user's
                # larger setting; report the actual stimulus separately.
                request = {"mode": 0, "velocity": args.velocity * (-1 if index % 2 else 1),
                           "current": min(current, .25), "drive_ms": args.drive_ms, "observe_ms": 500}
                run["diagnostic_request"] = request
                record(device.request("diagnose", **request))
                deadline = time.monotonic() + 6
                while True:
                    snapshot = record(device.request("status"))
                    diagnostic = snapshot["diagnostic"]
                    if not diagnostic["busy"]:
                        require(diagnostic["stage"] == 4 and snapshot["state"] == "idle",
                                "Bounded diagnostic did not complete normally")
                        run["diagnostic_completion"] = snapshot
                        require(snapshot["config"] == config, "Diagnostic did not restore its input configuration")
                        break
                    require(time.monotonic() < deadline, "Diagnostic exceeded its 6 second host deadline")
                    time.sleep(.02)
            else:
                run["diagnostic_skipped"] = "Firmware does not include CONFIG_KNOB_DIAGNOSTICS"
            run["after"] = record(device.request("disable"))
            require(run["after"]["state"] == "idle" and run["after"]["duty"] == [0, 0, 0], "Run did not stop")
            run["summary"] = summarize(run)
            print(json.dumps(run["summary"]), flush=True)
            report["modulation_reached"] |= run["summary"]["modulation_reached"]
        if args.require_full:
            require(report["modulation_reached"], "No new 100% latched PWM cycles; full output was not exercised")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
    finally:
        if device is not None:
            try:
                if device.identified:
                    device.request("disable")
                    # A cancelled on-board test restores its own configuration
                    # asynchronously. Wait for it before restoring our original.
                    deadline = time.monotonic() + 2
                    while True:
                        final = device.request("status")
                        if not final.get("diagnostic", {}).get("busy", False):
                            break
                        require(time.monotonic() < deadline, "Diagnostic cleanup did not finish")
                        time.sleep(.02)
                    if restore_needed:
                        final = device.request("configure", common_configuration(original))
                    final = device.request("status")
                    report["final"] = final
                    require(final["state"] in ("idle", "fault") and final["duty"] == [0, 0, 0],
                            "Final output is not stopped")
                    if original is not None:
                        require(final["config"] == original and final["calibration"] == calibration,
                                "Original configuration/calibration was not restored")
                    if report["passed"]:
                        check_snapshot(final, calibration)
                        require(final["state"] == "idle", "Verification did not finish idle")
            except Exception as error:
                report["cleanup_errors"].append(str(error))
                report["passed"] = False
            finally:
                try:
                    device.close()  # Knob.close makes another explicit stop attempt.
                except Exception as error:
                    report["cleanup_errors"].append("close: " + str(error))
                    report["passed"] = False
        # Full modulation and overall test success are independent findings.
        # An aborted run can reach the endpoint before its per-run summary is
        # written. Preserve that evidence without clearing its fault/error.
        initial, final = report.get("initial", {}), report.get("final", {})
        if ("uptime_ms" in initial and "uptime_ms" in final and
                final["uptime_ms"] >= initial["uptime_ms"] and
                all(key in initial and key in final and final[key] >= initial[key] for key in COUNTERS)):
            report["counter_delta"] = {key: final[key] - initial[key] for key in COUNTERS}
            report["modulation_reached"] = (final.get("pwm_peak_duty") == 1 and
                                            report["counter_delta"]["pwm_full_duty_cycles"] > 0)
        report["elapsed_seconds"] = time.monotonic() - started
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="COM7")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/svpwm-check.json"))
    parser.add_argument("--max-current", type=float, default=.8,
                        help="Maximum configured test current; additionally capped by the initial user setting")
    parser.add_argument("--drive-ms", type=int, default=500, help="On-board bounded drive duration, 50–500 ms")
    parser.add_argument("--velocity", type=float, default=50, help="Diagnostic rad/s; polarity alternates between runs")
    parser.add_argument("--stress-gain", action="store_true", help="Temporarily use Kp=20, Ki=60, restoring original gains on exit")
    parser.add_argument("--require-full", action="store_true", help="Fail unless new full-duty latch cycles were observed")
    args = parser.parse_args()
    if not math.isfinite(args.max_current) or not .02 <= args.max_current <= 4.5:
        parser.error("--max-current must be finite and in [0.02, 4.5] A")
    if not 50 <= args.drive_ms <= 500:
        parser.error("--drive-ms must be in [50, 500]")
    if not math.isfinite(args.velocity) or not 10 <= abs(args.velocity) <= 60:
        parser.error("--velocity magnitude must be finite and in [10, 60] rad/s")
    report = run_check(args)
    print(json.dumps({key: report.get(key) for key in
                      ("passed", "modulation_reached", "effective_max_current", "error", "cleanup_errors")}), flush=True)
    if not report["passed"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
