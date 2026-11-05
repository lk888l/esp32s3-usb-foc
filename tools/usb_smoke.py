#!/usr/bin/env python3
"""Native USB framing, reconnect, and rejected-command lease checks."""
import argparse
import json
import math
import time
from pathlib import Path
from knobctl import Knob

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def read_replies(device, ids):
    pending = set(ids)
    replies = {}
    deadline = time.monotonic() + 2
    line = bytearray()
    while pending and time.monotonic() < deadline:
        for byte in device.serial.read(256):
            if byte == 10:
                try:
                    reply = json.loads(line)
                except (ValueError, UnicodeError):
                    reply = {}
                line.clear()
                if isinstance(reply, dict) and reply.get("id") in pending:
                    require(reply.get("device") == "smartknob-s3" and reply.get("protocol") == 1 and reply.get("ok") is True, "Invalid USB reply")
                    pending.remove(reply["id"])
                    replies[reply["id"]] = reply
            else:
                line.append(byte)
                require(len(line) <= 8192, "Oversize USB reply")
    require(not pending, f"USB replies missing: {pending}")
    return replies

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--output", type=Path, default=Path("build/commissioning/usb-smoke.json"))
    args = parser.parse_args()
    report = {"checks": [], "passed": False}
    device = Knob(args.port)
    def passed(name):
        report["checks"].append(name)
        print("PASS:", name, flush=True)
    try:
        initial = device.request("status")
        require(initial["state"] == "idle" and initial["transport"] == "usb_serial_jtag", "Expected idle native USB firmware")
        report["initial"] = initial
        require(initial.get("voltage_limit_automatic") is True and "voltage_limit" not in initial["config"],
                "Voltage limit is still configurable")
        require(initial["bus_voltage_valid"], "Bus voltage is unavailable")
        require(math.isclose(initial["effective_voltage_limit"], initial["available_voltage_limit"], rel_tol=1e-5),
                "Automatic voltage capacity differs from the effective limit")
        if initial["pwm_duty_ceiling"] == 1:
            require(math.isclose(initial["effective_voltage_limit"], initial["bus_voltage"] / math.sqrt(3), rel_tol=1e-5),
                    "Full SVPWM capacity does not follow the live bus")
        refused = device.request("configure", {**initial["config"], "voltage_limit": .8}, allow_error=True)
        require(not refused["ok"] and refused["error"] == "voltage_limit_automatic",
                "Retired voltage field was not explicitly rejected")
        require(refused["config"] == initial["config"] and refused["calibration"] == initial["calibration"] and
                refused["state"] == "idle" and refused["duty"] == [0, 0, 0],
                "Rejected voltage field changed settings or energized output")
        report["voltage_limit_rejection"] = refused
        passed("automatic_voltage_limit_and_retired_field_rejection")
        device.sequence += 1
        frame = json.dumps({"id": device.sequence, "op": "status"}).encode() + b"\r\n"
        for byte in frame:
            device.serial.write(bytes([byte]))
            time.sleep(.002)
        read_replies(device, [device.sequence])
        passed("one_byte_fragments_and_crlf")
        ids = list(range(device.sequence + 1, device.sequence + 5))
        device.sequence = ids[-1]
        device.serial.write(b"".join(json.dumps({"id": i, "op": "status"}).encode() + b"\n" for i in ids))
        read_replies(device, ids)
        passed("four_pipelined_requests")
        for frame in (b"x" * 2100 + b'{"id":9999,"op":"enable"}\n',
                      b'{"id":9999,"op":"enable"}\x00\n'):
            device.serial.write(frame)
            require(device.request("status")["state"] == "idle", "Discarded frame enabled the motor")
        passed("oversize_and_control_character_frames_discarded")
        times = []
        for _ in range(100):
            start = time.monotonic()
            snapshot = device.request("status")
            times.append((time.monotonic() - start) * 1000)
            require(snapshot["state"] == "idle" and snapshot["fault"] == "none", "Status polling changed motor state")
        report["round_trip_ms"] = {"mean": sum(times) / len(times), "max": max(times)}
        passed("100_status_requests")
        for _ in range(20):
            device.close()
            device = Knob(args.port)
            require(device.request("status")["state"] == "idle", "Reconnect resumed torque")
        passed("20_reconnects_no_automatic_enable")
        device.request("enable")
        time.sleep(.25)
        require(device.request("status")["state"] == "active", "Enable did not complete")
        lease_ms = initial.get("host_lease_ms", 1000)
        deadline = time.monotonic() + max(1.3, lease_ms / 1000 + .3)
        while time.monotonic() < deadline:
            reply = device.request("enable", allow_error=True)
            require(not reply["ok"] and reply["error"] == "idle_required", "Unexpected repeated enable result")
            time.sleep(.04)
        if lease_ms:
            require(reply["state"] == "fault" and reply["fault"] == "host_timeout", "Refused commands renewed the lease")
            require(reply["duty"] == [0, 0, 0] and not reply["current_valid"], "Lease expiry did not stop output")
            report["refused_commands_expiry"] = reply
            device.request("clear")
            passed("well_formed_refused_commands_do_not_renew_lease")
        else:
            time.sleep(2)  # No commands at all: standalone haptics must keep running.
            reply = device.request("status")
            require(reply["state"] == "active" and reply["fault"] == "none", "Standalone haptics stopped without a lease")
            report["standalone_without_polling"] = reply
            stopped = device.request("disable")
            require(stopped["state"] == "idle" and stopped["target"] == 0 and stopped["duty"] == [0, 0, 0], "Explicit disable did not stop output")
            passed("standalone_no_heartbeat_and_explicit_stop")
        report["passed"] = True
    except Exception as error:
        report["error"] = str(error)
        raise
    finally:
        try:
            if device.identified:
                device.request("disable")
                report["final"] = device.request("status")
        finally:
            device.close()
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")

if __name__ == "__main__":
    main()
