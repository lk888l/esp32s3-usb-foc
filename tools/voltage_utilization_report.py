#!/usr/bin/env python3
"""Summarize saved stall-window tests without connecting to a device.

Utilization is the dq voltage magnitude divided by Vbus/sqrt(3). The latched
estimate uses the coherent carrier between the two measured ADC slots, never
the preceding carrier or a lifetime duty peak. It is firmware latch evidence,
not an external measurement of motor-terminal voltage.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path
import statistics


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def stats(values):
    values = [value for value in values if finite(value)]
    return {"count": len(values), "min": min(values) if values else None,
            "mean": statistics.mean(values) if values else None,
            "max": max(values) if values else None}


def utilization(duty):
    """Clarke vector magnitude normalized to the full linear SVPWM circle."""
    if (len(duty) != 3 or any(not finite(d) or not 0 <= d <= 1 for d in duty)):
        raise ValueError("Three finite duty values in [0, 1] are required")
    a, b, c = duty
    return math.sqrt(3) * math.hypot((2 * a - b - c) / 3, (b - c) / math.sqrt(3))


def observation(snapshot):
    """Return joined voltage/window evidence or None for incomplete snapshots."""
    diagnostic = snapshot.get("diagnostic")
    if (not isinstance(diagnostic, dict) or diagnostic.get("stage") != 2 or
            snapshot.get("current_valid") is not True or
            snapshot.get("adc_sample_valid") is not True or
            snapshot.get("pwm_compare_valid") is not True):
        return None
    half = snapshot.get("pwm_half_period_ticks")
    resolution = snapshot.get("pwm_resolution_hz")
    bus, ceiling = snapshot.get("bus_voltage"), snapshot.get("pwm_duty_ceiling")
    if (type(half) is not int or half <= 0 or resolution != 80_000_000 or
            not finite(bus) or bus <= 0 or not finite(ceiling) or not 0 < ceiling <= 1):
        return None
    for key in ("adc_completed_mask", "adc_done_mask", "adc_window_after_mask"):
        mask = snapshot.get(key)
        if type(mask) is not int or mask & 3 != 3:
            return None
    before, after = snapshot.get("adc_pwm_compare_before"), snapshot.get("adc_pwm_compare_after")
    for matrix in (before, after):
        if not isinstance(matrix, list) or len(matrix) < 2:
            return None
        for row in matrix[:2]:
            if (not isinstance(row, list) or len(row) != 3 or
                    any(type(value) is not int or not 0 <= value <= half for value in row)):
                return None
    if not after[0] == before[1] == after[1]:
        return None
    words = snapshot.get("adc_current_words")
    starts, dones = snapshot.get("adc_trigger_ticks"), snapshot.get("adc_done_ticks")
    if any(not isinstance(values, list) or len(values) < 2 for values in (words, starts, dones)):
        return None
    phases, falling, rising, conversion = [], [], [], []
    for slot in range(2):
        word, start, done = words[slot], starts[slot], dones[slot]
        if (type(word) is not int or word < 0 or word & ~0x1efff or
                type(start) is not int or type(done) is not int or done < start):
            return None
        phase = ((word >> 13) & 15) - 3
        if not 0 <= phase < 3:
            return None
        phases.append(phase)
        falling.append((start + before[slot][phase] - 480) / 80)
        rising.append((after[slot][phase] - 80 - done) / 80)
        conversion.append((done - start) / 80)
    if phases[0] == phases[1] or min(falling) < 0 or min(rising) <= 0:
        return None
    duty = [1 - value / half for value in after[0]]
    ideal = bus / math.sqrt(3)
    latched = utilization(duty)
    vd, vq = snapshot.get("voltage_d"), snapshot.get("voltage_q")
    commanded = math.hypot(vd, vq) / ideal if finite(vd) and finite(vq) else None
    return {"ideal_dq_voltage_v": ideal, "ceiling": ceiling,
            "commanded_utilization": commanded, "latched_utilization": latched,
            "latched_dq_voltage_estimate_v": ideal * latched,
            "latched_peak_phase_duty": max(duty),
            "falling_margin_us": falling, "rising_margin_us": rising,
            "start_to_done_observed_us": conversion,
            "start_tick": starts[:2], "done_tick": dones[:2]}


def cohort(samples):
    observed = [item for s in samples if (item := observation(s)) is not None]
    result = {"snapshots": len(samples), "valid_latch_snapshots": len(observed),
              "excluded_snapshots": len(samples) - len(observed)}
    for field in ("ideal_dq_voltage_v", "ceiling", "commanded_utilization", "latched_utilization",
                  "latched_dq_voltage_estimate_v", "latched_peak_phase_duty"):
        result[field] = stats(o[field] for o in observed)
    for field in ("falling_margin_us", "rising_margin_us", "start_to_done_observed_us",
                  "start_tick", "done_tick"):
        result[field] = stats(value for o in observed for value in o[field])
    for field in ("commanded_utilization", "latched_utilization"):
        available = [o for o in observed if finite(o[field])]
        reached = sum(o[field] >= .98 * o["ceiling"] for o in available)
        result[field + "_at_least_98pct_ceiling"] = {
            "count": reached, "fraction": reached / len(available) if available else None}
    return result


def timing_observation(snapshot):
    """Validate a DONE association without filtering on firmware acceptance.

    Rejected phase windows are essential timing evidence. In contrast, a
    missing DONE, mismatched channel/value, or untracked PWM carrier cannot
    supply a trustworthy edge margin and is explicitly unavailable.
    """
    half = snapshot.get("pwm_half_period_ticks")
    if type(half) is not int or half <= 0 or snapshot.get("pwm_resolution_hz") != 80_000_000:
        return None, "timer_metadata"
    if snapshot.get("pwm_compare_valid") is not True:
        return None, "pwm_carrier_not_tracked"
    for key in ("adc_completed_mask", "adc_done_mask", "adc_window_after_mask"):
        mask = snapshot.get(key)
        if type(mask) is not int or mask & 3 != 3:
            return None, "missing_" + key
    before, after = snapshot.get("adc_pwm_compare_before"), snapshot.get("adc_pwm_compare_after")
    for matrix in (before, after):
        if not isinstance(matrix, list) or len(matrix) < 2:
            return None, "compare_metadata"
        for row in matrix[:2]:
            if (not isinstance(row, list) or len(row) != 3 or
                    any(type(value) is not int or not 0 <= value <= half for value in row)):
                return None, "compare_metadata"
    keys = ("adc_current_words", "adc_armed_words", "adc_done_words", "adc_trigger_ticks",
            "adc_done_ticks", "adc_trigger_elapsed_us", "adc_done_elapsed_us")
    for key in keys:
        values = snapshot.get(key)
        if not isinstance(values, list) or len(values) < 2 or any(type(v) is not int for v in values[:2]):
            return None, "missing_" + key
    phases, falling, rising, conversion = [], [], [], []
    for slot in range(2):
        word = snapshot["adc_current_words"][slot]
        armed, done_word = snapshot["adc_armed_words"][slot], snapshot["adc_done_words"][slot]
        phase = ((word >> 13) & 15) - 3
        if (word < 0 or word & ~0x1efff or not 0 <= phase < 3 or armed < 0 or done_word < 0 or
                ((armed >> 13) & 15) != phase + 3 or (word & 0xfff) != (done_word & 0xfff)):
            return None, "done_channel_or_value_join"
        start, done = snapshot["adc_trigger_ticks"][slot], snapshot["adc_done_ticks"][slot]
        start_us, done_us = snapshot["adc_trigger_elapsed_us"][slot], snapshot["adc_done_elapsed_us"][slot]
        elapsed = done_us - start_us
        # Mirror the independent hardware-counter/wall-clock association. Do
        # not require the planned START bound or either physical edge margin:
        # violating those is precisely what this diagnostic must retain.
        if (not -half <= start < 0 or not start <= done <= half or start_us < 0 or
                not 0 <= elapsed < 40 or abs(done - start - elapsed * 80) > 400):
            return None, "done_carrier_time_join"
        phases.append(phase)
        falling.append((start + before[slot][phase] - 480) / 80)
        rising.append((after[slot][phase] - 80 - done) / 80)
        conversion.append((done - start) / 80)
    if phases[0] == phases[1]:
        return None, "duplicate_current_phase"
    return {"start_tick": snapshot["adc_trigger_ticks"][:2], "done_tick": snapshot["adc_done_ticks"][:2],
            "start_to_done_observed_us": conversion, "falling_margin_us": falling,
            "rising_margin_us": rising}, None


def timing_cohort(samples):
    observations = [timing_observation(s) for s in samples]
    available = [item for item, _ in observations if item is not None]
    reasons = Counter(reason for item, reason in observations if item is None)
    result = {"snapshots": len(samples), "available_snapshots": len(available),
              "unavailable_snapshots": sum(reasons.values()), "unavailable_reasons": dict(reasons),
              "out_of_window_snapshots": sum(any(f < 0 or r <= 0 for f, r in
                                                   zip(o["falling_margin_us"], o["rising_margin_us"]))
                                               for o in available),
              "falling_window_violation_slots": sum(v < 0 for o in available for v in o["falling_margin_us"]),
              "rising_window_violation_slots": sum(v <= 0 for o in available for v in o["rising_margin_us"])}
    for field in ("start_tick", "done_tick", "start_to_done_observed_us", "falling_margin_us", "rising_margin_us"):
        result[field] = stats(v for o in available for v in o[field])
    return result


def timing_summary(samples):
    return {"all_adc_snapshots": timing_cohort(samples),
            "accepted_adc_snapshots": timing_cohort([s for s in samples if s.get("adc_sample_valid") is True]),
            "rejected_adc_snapshots": timing_cohort([s for s in samples if s.get("adc_sample_valid") is False]),
            "unknown_adc_state_snapshots": timing_cohort([s for s in samples if type(s.get("adc_sample_valid")) is not bool])}


def counter_summary(samples):
    if len(samples) < 2:
        return {"valid": False, "reason": "Fewer than two drive snapshots"}
    first, last = samples[0], samples[-1]
    start, end = first.get("uptime_ms"), last.get("uptime_ms")
    keys = ("control_cycles", "rejected_samples", "sampling_recoveries", "missed_samples",
            "reused_samples", "encoder_timing_skips", "pwm_deferred")
    if (not finite(start) or not finite(end) or end <= start or
            any(type(s.get(k)) is not int for s in (first, last) for k in keys)):
        return {"valid": False, "reason": "Missing counters or invalid device clock"}
    delta = {k: last[k] - first[k] for k in keys}
    reasons = {}
    first_reasons, last_reasons = first.get("adc_reject_counts", {}), last.get("adc_reject_counts", {})
    if not isinstance(first_reasons, dict) or not isinstance(last_reasons, dict):
        return {"valid": False, "reason": "Invalid ADC rejection counters"}
    for key, value in first_reasons.items():
        final = last_reasons.get(key)
        if type(value) is not int or type(final) is not int:
            return {"valid": False, "reason": "Invalid ADC rejection counters"}
        reasons[key] = final - value
    if any(value < 0 for value in [*delta.values(), *reasons.values()]):
        return {"valid": False, "reason": "Counters reset during observation"}
    elapsed = (end - start) / 1000
    count = delta["control_cycles"] + delta["rejected_samples"]
    return {"valid": True, "elapsed_seconds": elapsed, "delta": delta,
            "adc_reject_counts": reasons, "effective_control_hz": delta["control_cycles"] / elapsed,
            "reject_ratio": delta["rejected_samples"] / count if count else None}


def summarize_run(run):
    samples = run.get("samples", [])
    if not isinstance(samples, list) or any(not isinstance(s, dict) for s in samples):
        raise ValueError("run.samples must contain JSON objects")
    active = [s for s in samples if isinstance(s.get("diagnostic"), dict) and
              s["diagnostic"].get("stage") == 2]
    drive_ms = run.get("drive_ms")
    start = active[0].get("uptime_ms") if active else None
    tail_start = start + max(200, drive_ms / 2) if finite(start) and finite(drive_ms) else None
    tail = [s for s in active if tail_start is not None and finite(s.get("uptime_ms")) and
            s["uptime_ms"] >= tail_start]
    return {"requested_current_a": run.get("requested_current"), "drive_ms": drive_ms,
            "source_run_passed": run.get("passed"), "source_criteria": run.get("criteria"),
            "all_drive": cohort(active), "tail_half_drive": cohort(tail),
            "all_drive_timing": timing_summary(active), "tail_half_drive_timing": timing_summary(tail),
            "drive_counters": counter_summary(active),
            "source_summary": run.get("summary")}


def summarize_report(report, source=None):
    if not isinstance(report, dict) or not isinstance(report.get("runs"), list):
        raise ValueError("Expected a stall_window_check report with a runs list")
    if any(not isinstance(run, dict) for run in report["runs"]):
        raise ValueError("Each run must be a JSON object")
    initial = report.get("initial", {})
    if not isinstance(initial, dict):
        raise ValueError("initial must be a JSON object")
    half, resolution = initial.get("pwm_half_period_ticks"), initial.get("pwm_resolution_hz")
    exact_pwm = resolution / (2 * half) if type(half) is int and half > 0 and finite(resolution) else None
    return {"source": source, "source_passed": report.get("passed"), "source_error": report.get("error"),
            "firmware": initial.get("firmware"), "app_elf_sha256": initial.get("app_elf_sha256"),
            "pwm_profile": initial.get("pwm_profile"), "pwm_hz": initial.get("pwm_hz"),
            "pwm_hz_from_timer_ticks": exact_pwm,
            "control_nominal_hz": initial.get("control_nominal_hz"),
            "runs": [summarize_run(run) for run in report["runs"]]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = {"voltage_evidence": "PWM TEZ latch mirror and contemporaneous bus ADC; not external motor-terminal voltage measurement",
              "normalization": "dq magnitude / (Vbus / sqrt(3)); ratios are not percentages",
              "window_contract": "80 MHz ticks; falling margin beyond 6 us settling >= 0; rising margin before 1 us guard > 0",
              "sampling": "USB snapshots; means are observation averages, not time-weighted waveform measurements",
              "timing_evidence": "Separate *_drive_timing summaries retain rejected ADC windows after DONE/channel/carrier association; unavailable evidence is counted. They do not supply voltage estimates. USB timing extrema do not bound unobserved conversions.",
              "tail_definition": "At least max(200 ms, drive_ms/2) after the first drive snapshot",
              "reports": []}
    for path in args.inputs:
        try:
            report = json.loads(path.read_text(encoding="utf-8-sig"))
            result["reports"].append(summarize_report(report, str(path)))
        except (OSError, ValueError, TypeError) as error:
            parser.error(f"{path}: {error}")
    encoded = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("x", encoding="utf-8") as output:
            output.write(encoded)
    print(encoded, end="")


if __name__ == "__main__":
    main()
