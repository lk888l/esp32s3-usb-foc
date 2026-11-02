#!/usr/bin/env python3
"""USB commissioning client. Requires pyserial; running commands always stop on exit."""
import argparse
import json
import time
from pathlib import Path
import serial


class Knob:
    def __init__(self, port):
        self.serial = serial.Serial(port, 115200, timeout=0.05, write_timeout=0.3)
        self.sequence = 0
        self.identified = False

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
                elif len(line) >= 4096:
                    overflow = True
                elif not overflow:
                    line.append(byte)
        raise TimeoutError("SmartKnob response timed out")

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
    parser.add_argument("--config", type=Path, help="Complete config JSON (for configure)")
    args = parser.parse_args()
    if not 0 < args.seconds <= 60:
        parser.error("--seconds must be in (0, 60]")
    if args.op == "configure" and args.config is None:
        parser.error("configure requires --config")
    device = Knob(args.port)
    try:
        device.request("status")
        config = json.loads(args.config.read_text()) if args.config else None
        response = device.request(args.op, config)
        print(json.dumps(response, ensure_ascii=False, indent=2))
        if args.op in ("calibrate", "enable"):
            end = time.monotonic() + (10 if args.op == "calibrate" else args.seconds)
            while time.monotonic() < end:
                time.sleep(0.1)
                response = device.request("status")
                print(json.dumps({key: response[key] for key in ("state", "fault", "angle", "iq", "currents", "sample_us", "rejected_samples")}))
                if response["state"] == "fault":
                    raise RuntimeError(response["fault"])
                if args.op == "calibrate" and response["state"] == "idle":
                    if not response["calibration"]["valid"]:
                        raise RuntimeError("Calibration did not complete")
                    print("Calibration complete. Use 'save' to persist it.")
                    break
            else:
                if args.op == "calibrate":
                    raise TimeoutError("Calibration did not finish in 10 seconds")
    finally:
        device.close()


if __name__ == "__main__":
    main()
