from __future__ import annotations

import argparse
import time

import serial
from serial.tools import list_ports


def list_usb_ports() -> list[serial.tools.list_ports_common.ListPortInfo]:
    ports = []
    for port in list_ports.comports():
        hwid = (port.hwid or "").upper()
        device = (port.device or "").lower()
        if "USB" in hwid or "ACM" in hwid or "USB" in device or "ACM" in device:
            ports.append(port)
    return ports


def probe_port(device: str, baud: int, timeout_s: float = 0.3) -> bool:
    try:
        with serial.Serial(device, baudrate=baud, timeout=timeout_s, write_timeout=timeout_s) as ser:
            time.sleep(0.1)
            waiting = ser.in_waiting
            data = ser.read(waiting or 1)
            print(f"[OK] opened {device} @ {baud}, in_waiting={waiting}, read={data.hex() or 'empty'}")
            try:
                ser.write(b"\xFF")
                ser.flush()
                print(f"[OK] wrote FF to {device}")
            except Exception as exc:
                print(f"[WARN] write failed on {device}: {exc}")
            return True
    except Exception as exc:
        print(f"[FAIL] {device}: {exc}")
        return False


def main() -> None:
    parser = argparse.ArgumentParser(description="Detect USB serial ports on Raspberry Pi / Windows")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--device", help="probe a specific device directly")
    args = parser.parse_args()

    if args.device:
        probe_port(args.device, args.baud)
        return

    ports = list_usb_ports()
    if not ports:
        print("No USB serial ports found.")
        print("Common candidates: /dev/serial0, /dev/ttyUSB0, /dev/ttyACM0")
        return

    print("Detected USB serial ports:")
    for port in ports:
        print(
            f"- device={port.device} "
            f"desc={port.description} "
            f"hwid={port.hwid} "
            f"vid={getattr(port, 'vid', None)} "
            f"pid={getattr(port, 'pid', None)}"
        )

    print("")
    print("Probing each port...")
    for port in ports:
        probe_port(port.device, args.baud)


if __name__ == "__main__":
    main()
