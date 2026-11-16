#!/usr/bin/env python3
"""Observe real Windows Raw Input from SmartKnob without injecting input.

Example: python tools/windows_wheel_capture.py --seconds 15 --output wheel.json
Uses only the Python standard library. The hidden window receives background
mouse input and filters by the USB VID/PID; other mice are never recorded.
"""
import argparse
import ctypes as C
from ctypes import wintypes as W
import json
from pathlib import Path
import threading
import time


class WheelCapture:
    def __init__(self):
        self.events = []
        self.error = None
        self.ready = threading.Event()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        if not self.ready.wait(5):
            raise TimeoutError("Raw Input window did not start")
        if self.error:
            raise RuntimeError(self.error)
        return self

    def stop(self):
        self.stop_event.set()
        self.thread.join(3)
        if self.error:
            raise RuntimeError(self.error)
        return self.events

    def _run(self):
        try:
            user = C.WinDLL("user32", use_last_error=True)
            kernel = C.WinDLL("kernel32", use_last_error=True)
            callback = C.WINFUNCTYPE(C.c_ssize_t, W.HWND, W.UINT, W.WPARAM, W.LPARAM)

            class Header(C.Structure):
                _fields_ = [("type", W.DWORD), ("size", W.DWORD), ("device", W.HANDLE), ("wparam", W.WPARAM)]

            class Device(C.Structure):
                _fields_ = [("page", W.USHORT), ("usage", W.USHORT), ("flags", W.DWORD), ("target", W.HWND)]

            class WindowClass(C.Structure):
                _fields_ = [("style", W.UINT), ("proc", callback), ("extra", C.c_int), ("window_extra", C.c_int),
                            ("instance", W.HINSTANCE), ("icon", W.HICON), ("cursor", W.HANDLE),
                            ("background", W.HBRUSH), ("menu", W.LPCWSTR), ("name", W.LPCWSTR)]

            user.DefWindowProcW.argtypes = [W.HWND, W.UINT, W.WPARAM, W.LPARAM]
            user.DefWindowProcW.restype = C.c_ssize_t
            user.GetRawInputData.argtypes = [W.HANDLE, W.UINT, W.LPVOID, C.POINTER(W.UINT), W.UINT]
            user.GetRawInputDeviceInfoW.argtypes = [W.HANDLE, W.UINT, W.LPVOID, C.POINTER(W.UINT)]
            user.CreateWindowExW.argtypes = [W.DWORD, W.LPCWSTR, W.LPCWSTR, W.DWORD, C.c_int, C.c_int, C.c_int, C.c_int, W.HWND, W.HMENU, W.HINSTANCE, W.LPVOID]
            user.CreateWindowExW.restype = W.HWND
            user.DestroyWindow.argtypes = [W.HWND]
            kernel.GetModuleHandleW.restype = W.HMODULE
            names = {}

            @callback
            def proc(hwnd, message, wparam, lparam):
                if message == 0x00FF:  # WM_INPUT
                    length = W.UINT()
                    user.GetRawInputData(lparam, 0x10000003, None, C.byref(length), C.sizeof(Header))
                    data = C.create_string_buffer(length.value)
                    received = user.GetRawInputData(lparam, 0x10000003, data, C.byref(length), C.sizeof(Header))
                    if received == length.value and length.value >= C.sizeof(Header) + 24:
                        header = Header.from_buffer(data)
                        if header.device not in names:
                            capacity = W.UINT(512)
                            name = C.create_unicode_buffer(512)
                            user.GetRawInputDeviceInfoW(header.device, 0x20000007, name, C.byref(capacity))
                            names[header.device] = name.value
                        name = names[header.device]
                        if header.type == 0 and "VID_303A&PID_4005" in name.upper():
                            offset = C.sizeof(Header)
                            flags = C.c_ushort.from_buffer(data, offset + 4).value
                            delta = C.c_short.from_buffer(data, offset + 6).value
                            self.events.append({"time": time.monotonic(), "device": name,
                                                "flags": flags, "wheel": delta if flags & 0x400 else 0,
                                                "x": C.c_long.from_buffer(data, offset + 12).value,
                                                "y": C.c_long.from_buffer(data, offset + 16).value})
                return user.DefWindowProcW(hwnd, message, wparam, lparam)

            name = f"SmartKnobRawInput{time.time_ns()}"
            instance = kernel.GetModuleHandleW(None)
            window_class = WindowClass(0, proc, 0, 0, instance, None, None, None, None, name)
            if not user.RegisterClassW(C.byref(window_class)):
                raise C.WinError(C.get_last_error())
            window = user.CreateWindowExW(0, name, name, 0, 0, 0, 1, 1, None, None, instance, None)
            if not window:
                raise C.WinError(C.get_last_error())
            device = Device(1, 2, 0x100, window)  # RIDEV_INPUTSINK
            if not user.RegisterRawInputDevices(C.byref(device), 1, C.sizeof(Device)):
                raise C.WinError(C.get_last_error())
            self.ready.set()
            message = W.MSG()
            while not self.stop_event.wait(.002):
                while user.PeekMessageW(C.byref(message), None, 0, 0, 1):
                    user.TranslateMessage(C.byref(message))
                    user.DispatchMessageW(C.byref(message))
            user.DestroyWindow(window)
        except Exception as error:
            self.error = repr(error)
            self.ready.set()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=15)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    capture = WheelCapture().start()
    try:
        time.sleep(args.seconds)
    finally:
        events = capture.stop()
        args.output.write_text(json.dumps(events, indent=2), encoding="utf-8")
        print(json.dumps({"events": len(events), "wheel_delta": sum(e["wheel"] for e in events)}))
