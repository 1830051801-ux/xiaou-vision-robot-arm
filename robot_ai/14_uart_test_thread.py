from __future__ import annotations

import argparse
import os
import threading
import time

import serial

from xiaou_runtime import get_logger, get_xiaou_config


LOGGER = get_logger(__name__)


def _resolve_default_port(preferred: str | None = None) -> str:
    candidates: list[str] = []
    if preferred:
        candidates.append(preferred)
    candidates.extend(
        [
            "/dev/serial0",
            "/dev/ttyAMA10",
            "/dev/ttyAMA0",
            "/dev/ttyS0",
        ]
    )
    seen: set[str] = set()
    for candidate in candidates:
        if candidate and candidate not in seen:
            seen.add(candidate)
            if os.path.exists(candidate):
                return candidate
    return preferred or "/dev/serial0"


class UARTTestThread:
    def __init__(
        self,
        port: str,
        baud: int,
        send_interval_s: float = 2.0,
        periodic_tx: int = 0xFF,
        echo_trigger: int = 0x01,
        echo_reply: int = 0x01,
        idle_warn_s: float = 5.0,
    ) -> None:
        self.port = port
        self.baud = baud
        self.send_interval_s = max(0.1, float(send_interval_s))
        self.periodic_tx = periodic_tx & 0xFF
        self.echo_trigger = echo_trigger & 0xFF
        self.echo_reply = echo_reply & 0xFF
        self.idle_warn_s = max(0.5, float(idle_warn_s))
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run, name="uart-test", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        self._thread.join(timeout=3.0)

    def join(self) -> None:
        self._thread.join()

    def is_alive(self) -> bool:
        return self._thread.is_alive()

    def _run(self) -> None:
        try:
            with serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=0.2,
                write_timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            ) as ser:
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                LOGGER.info(
                    "UART test started on %s @ %s, periodic TX=0x%02X every %.1fs, echo RX=0x%02X -> TX=0x%02X",
                    self.port,
                    self.baud,
                    self.periodic_tx,
                    self.send_interval_s,
                    self.echo_trigger,
                    self.echo_reply,
                )
                LOGGER.info(
                    "If TX keeps printing but RX stays silent for %.1fs+, suspect pin mux, wiring, level mismatch, or no shared ground.",
                    self.idle_warn_s,
                )
                next_send = time.monotonic()
                rx_count = 0
                tx_count = 0
                last_rx_at = time.monotonic()
                last_warn_bucket = -1
                while not self._stop_event.is_set():
                    now = time.monotonic()
                    if now >= next_send:
                        ser.write(bytes([self.periodic_tx]))
                        ser.flush()
                        LOGGER.info("TX 0x%02X", self.periodic_tx)
                        tx_count += 1
                        next_send = now + self.send_interval_s

                    data = ser.read(1)
                    if data:
                        for byte in data:
                            rx_count += 1
                            last_rx_at = now
                            LOGGER.info("RX 0x%02X", byte)
                            if byte == self.echo_trigger:
                                ser.write(bytes([self.echo_reply]))
                                ser.flush()
                                tx_count += 1
                                LOGGER.info("TX 0x%02X", self.echo_reply)
                    idle_s = now - last_rx_at
                    if idle_s >= self.idle_warn_s:
                        bucket = int(idle_s // self.idle_warn_s)
                        if bucket != last_warn_bucket:
                            last_warn_bucket = bucket
                            LOGGER.warning(
                                "No RX for %.1fs after %s TX bytes and %s RX bytes. If the line is still silent, check GPIO14->RX, GPIO15->TX, GND, and the UART pin mux.",
                                idle_s,
                                tx_count,
                                rx_count,
                            )
                    if int(now) % 10 == 0 and int((now - 0.2) * 10) != int(now * 10):
                        LOGGER.debug("UART stats tx=%s rx=%s idle=%.1fs", tx_count, rx_count, idle_s)
        except Exception as exc:
            LOGGER.exception("UART test thread failed: %s", exc)
        finally:
            self._stop_event.set()


def main() -> None:
    cfg = get_xiaou_config()
    parser = argparse.ArgumentParser(description="UART send/echo test thread")
    parser.add_argument("--port", default="", help="serial port, default auto-detects /dev/serial0")
    parser.add_argument("--baud", type=int, default=cfg.serial_baud, help="serial baud rate")
    parser.add_argument("--interval", type=float, default=2.0, help="periodic TX interval in seconds")
    parser.add_argument("--tx", default="FF", help="periodic TX byte in hex, default FF")
    parser.add_argument("--rx", default="FF", help="RX trigger byte in hex, default FF")
    parser.add_argument("--reply", default="FF", help="reply byte in hex, default FF")
    parser.add_argument("--duration", type=float, default=0.0, help="optional duration limit in seconds")
    parser.add_argument("--idle-warn", type=float, default=5.0, help="warn if no RX is seen for this many seconds")
    args = parser.parse_args()

    port = _resolve_default_port(args.port or getattr(cfg, "serial_port", None))
    periodic_tx = int(args.tx, 16)
    echo_trigger = int(args.rx, 16)
    echo_reply = int(args.reply, 16)

    worker = UARTTestThread(
        port=port,
        baud=args.baud,
        send_interval_s=args.interval,
        periodic_tx=periodic_tx,
        echo_trigger=echo_trigger,
        echo_reply=echo_reply,
        idle_warn_s=args.idle_warn,
    )
    worker.start()
    LOGGER.info("Press Ctrl+C to stop.")

    started = time.monotonic()
    try:
        while worker.is_alive():
            if args.duration > 0 and (time.monotonic() - started) >= args.duration:
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        LOGGER.info("Stopping UART test...")
    finally:
        worker.stop()


if __name__ == "__main__":
    main()
