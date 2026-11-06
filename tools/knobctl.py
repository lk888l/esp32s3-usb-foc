#!/usr/bin/env python3
"""USB commissioning client. Requires pyserial; running commands always stop on exit."""
import argparse
import json
import math
import sys
import time
from pathlib import Path
import serial


class Knob:
    def __init__(self, port):
        # Set control lines before opening: Windows native USB can otherwise
        # reset the S3 and silently replace unsaved tuning with NVS defaults.
        self.serial = serial.Serial(None, 115200, timeout=0.05, write_timeout=0.3)
        self.serial.dtr = False
        self.serial.rts = False
        self.serial.port = port
        self.serial.open()
        # Extended sampling diagnostics can exceed Windows' default 4 KiB
        # receive queue in a single USB transfer while the host is compiling.
        if sys.platform == "win32":
            self.serial.set_buffer_size(rx_size=65536, tx_size=8192)
        self.sequence = 0
        self.identified = False
        self.unparsed_lines = []

    def request(self, op, config=None, *, allow_error=False, **parameters):
        self.sequence += 1
        message = {"id": self.sequence, "op": op}
        if {"id", "op", "config"} & parameters.keys():
            raise ValueError("Reserved request field")
        message.update(parameters)
        if config is not None:
            message["config"] = config
        encoded = json.dumps(message, allow_nan=False).encode() + b"\n"
        if len(encoded) > 2048:
            raise ValueError("Request exceeds firmware frame size")
        self.serial.write(encoded)
        deadline = time.monotonic() + 0.6
        line = bytearray()
        overflow = False
        while time.monotonic() < deadline:
            chunk = self.serial.read(256)
            for byte in chunk:
                if byte == 10:
                    try:
                        reply = json.loads(line) if not overflow else {}
                    except (ValueError, UnicodeError):
                        reply = {}
                        if line and len(self.unparsed_lines) < 64:
                            self.unparsed_lines.append(bytes(line).decode('utf-8', errors='replace'))
                    line.clear()
                    overflow = False
                    if not isinstance(reply, dict) or reply.get("id") != self.sequence:
                        continue
                    if reply.get("device") != "smartknob-s3" or reply.get("protocol") != 1:
                        raise RuntimeError("Wrong device/protocol")
                    self.identified = True
                    if not reply.get("ok") and not allow_error:
                        raise RuntimeError(reply.get("error", "Command rejected"))
                    return reply
                elif len(line) >= 16384:
                    overflow = True
                elif not overflow:
                    line.append(byte)
        raise TimeoutError("SmartKnob response timed out")

    def wait_active(self, timeout=2.0):
        """Poll an acknowledged enable without resending the motor command."""
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Active-state timeout must be finite and positive")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            snapshot = self.request("status")
            if snapshot["fault"] != "none" or snapshot["state"] == "fault":
                raise RuntimeError("Enable failed: " + snapshot["fault"])
            if time.monotonic() >= deadline:
                break
            if snapshot["state"] == "active":
                return snapshot
            if snapshot["state"] != "offsets":
                raise RuntimeError("Unexpected enable state: " + snapshot["state"])
            time.sleep(min(.02, max(0, deadline - time.monotonic())))
        raise TimeoutError(f"Enable did not become active within {timeout:g} seconds")

    def close(self):
        try:
            if self.identified:
                self.request("disable")
        finally:
            self.serial.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("op", choices=["status", "calibrate", "enable", "disable", "clear", "zero", "pulse", "save", "configure"])
    parser.add_argument("--seconds", type=float, default=5, help="Enable duration; always disables at end")
    parser.add_argument("--calibration-timeout", type=float, default=130,
                        help="Host calibration deadline in seconds (default: 130; firmware remains bounded to 121)")
    parser.add_argument("--calibration-current", type=float, help="Initial calibration current in A")
    parser.add_argument("--calibration-max-current", type=float, help="Maximum calibration current in A")
    parser.add_argument("--calibration-voltage", type=float, help="Calibration voltage limit in V")
    parser.add_argument("--config", type=Path, help="Complete config JSON (for configure)")
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("--seconds must be in (0, 60]")
    if not 0 < args.calibration_timeout <= 300:
        parser.error("--calibration-timeout must be in (0, 300]")
    calibration_parameters = {key: value for key, value in (
        ("current", args.calibration_current), ("max_current", args.calibration_max_current),
        ("voltage", args.calibration_voltage)) if value is not None}
    if calibration_parameters and args.op != "calibrate":
        parser.error("Calibration parameters require the calibrate operation")
    if args.op == "configure" and args.config is None:
        parser.error("configure requires --config")
    device = Knob(args.port)
    try:
        device.request("status")
        config = json.loads(args.config.read_text()) if args.config else None
        response = device.request(args.op, config, **calibration_parameters)
        print(json.dumps(response, ensure_ascii=False, indent=2))
        if args.op in ("calibrate", "enable"):
            end = time.monotonic() + (args.calibration_timeout if args.op == "calibrate" else args.seconds)
            while time.monotonic() < end:
                time.sleep(0.1)
                response = device.request("status")
                progress = {key: response[key] for key in ("state", "fault", "angle", "iq", "currents", "sample_us", "rejected_samples")}
                if args.op == "calibrate" and "calibration_report" in response:
                    progress["calibration_report"] = response["calibration_report"]
                print(json.dumps(progress))
                if response["state"] == "fault":
                    raise RuntimeError(response["fault"])
                if args.op == "calibrate" and response["state"] == "idle":
                    if not response["calibration"]["valid"]:
                        raise RuntimeError("Calibration did not complete")
                    if "calibration_report" in response and not response["calibration_report"]["passed"]:
                        raise RuntimeError("Calibration ended without a passing quality report")
                    print("Calibration complete. Use 'save' to persist it.")
                    break
            else:
                if args.op == "calibrate":
                    raise TimeoutError(f"Calibration did not finish in {args.calibration_timeout:g} seconds")
    finally:
        device.close()


if __name__ == "__main__":
    main()
