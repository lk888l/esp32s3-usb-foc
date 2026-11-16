#!/usr/bin/env python3
"""Stop SmartKnob and enter the ESP32-S3 ROM loader via a 1200-baud touch.

The application stops the motor before switching the USB PHY. The ROM port
usually has a different COM number. No flash/NVS/eFuse is written by this tool.
"""
import argparse
import time
import serial
from serial.tools.list_ports import comports


def enter(port):
    matches = [p for p in comports() if p.device == port and p.vid == 0x303A and p.pid == 0x4005]
    if len(matches) != 1:
        raise RuntimeError(f"{port} is not a SmartKnob composite USB port (303A:4005)")
    with serial.Serial(port=None, baudrate=115200, timeout=.1) as device:
        device.dtr = False
        device.rts = False
        device.port = port
        device.open()
        device.dtr = True
        time.sleep(.05)
        device.baudrate = 1200
        time.sleep(.05)
        device.dtr = False
    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        ports = [p.device for p in comports() if p.vid == 0x303A and p.pid == 0x1001]
        if ports:
            return ports
        time.sleep(.1)
    raise TimeoutError("ROM USB port did not appear; use BOOT + RESET to enter download mode")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True)
    print("ROM USB port(s):", ", ".join(enter(parser.parse_args().port)))
