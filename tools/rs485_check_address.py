#!/usr/bin/env python3
"""
RS-485 "Check Address" probe tool (no STM32 required).

Uses the same 15-byte probe frame pattern you used in `src/main.c`:
  3F 3F 00 <ADDR> FF FB 0D 7E 77 01 00 01 00 40 <CHK>

Bus settings (defaults): 19200 8O1.
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Optional, Tuple

try:
    import serial  # pyserial
except Exception:
    print("Missing dependency: pyserial. Install with: python -m pip install -r tools/requirements.txt", file=sys.stderr)
    raise


TEMPLATE = bytes(
    [
        0x3F,
        0x3F,
        0x00,
        0x37,  # overwritten with address
        0xFF,
        0xFB,
        0x0D,
        0x7E,
        0x77,
        0x01,
        0x00,
        0x01,
        0x00,
        0x40,
        0x75,  # overwritten with checksum
    ]
)


def checksum_for(addr: int) -> int:
    # Matches firmware: (addr + 0x3E) & 0xFF
    return (addr + 0x3E) & 0xFF


def build_probe(addr: int) -> bytes:
    b = bytearray(TEMPLATE)
    b[3] = addr & 0xFF
    b[14] = checksum_for(addr)
    return bytes(b)


def hexdump(data: bytes) -> str:
    return " ".join(f"{x:02X}" for x in data)


def read_window(
    ser: serial.Serial,
    total_window_s: float,
    silent_break_s: float,
    cap: int = 2048,
) -> bytes:
    buf = bytearray()
    deadline = time.monotonic() + total_window_s
    last_rx: Optional[float] = None

    while time.monotonic() < deadline and len(buf) < cap:
        chunk = ser.read(cap - len(buf))
        now = time.monotonic()
        if chunk:
            buf.extend(chunk)
            last_rx = now
            continue

        # no new data this tick
        if last_rx is not None and (now - last_rx) >= silent_break_s:
            break

    return bytes(buf)


def open_serial(args: argparse.Namespace) -> serial.Serial:
    return serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_ODD if args.parity.upper() == "O" else serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=max(1, args.timeout_ms) / 1000.0,
        rtscts=False,
        dsrdtr=False,
    )


def probe_one(args: argparse.Namespace, addr: int) -> Tuple[bool, bytes]:
    ser = open_serial(args)
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        # Pre-send quiet to avoid gluing prior traffic into our capture
        time.sleep(max(0, args.pre_quiet_ms) / 1000.0)

        tx = build_probe(addr)
        ser.write(tx)
        ser.flush()

        # TX->RX turnaround
        time.sleep(max(0, args.turnaround_ms) / 1000.0)

        rx = read_window(
            ser,
            total_window_s=max(1, args.read_window_ms) / 1000.0,
            silent_break_s=max(1, args.silent_break_ms) / 1000.0,
            cap=args.rx_cap,
        )

        present = len(rx) >= args.min_reply_len
        return present, rx
    finally:
        try:
            ser.close()
        except Exception:
            pass


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Probe a Teletrol controller address over RS-485 (no STM32 required).")
    ap.add_argument("--port", required=True, help="COM port for your USB-RS485 adapter (e.g. COM4)")
    ap.add_argument("--baud", type=int, default=19200, help="Baud rate (default: 19200)")
    ap.add_argument("--parity", choices=("O", "N"), default="O", help="Parity: O=odd, N=none (default: O)")

    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--addr", type=int, help="Probe a single address (1..64)")
    mode.add_argument("--scan", action="store_true", help="Scan addresses 1..64, stop on first hit")

    ap.add_argument("--timeout-ms", type=int, default=10, help="Serial read timeout in ms (default: 10)")
    ap.add_argument("--pre-quiet-ms", type=int, default=5, help="Quiet time before TX in ms (default: 5)")
    ap.add_argument("--turnaround-ms", type=int, default=100, help="Delay after TX before reading (default: 100)")
    ap.add_argument("--read-window-ms", type=int, default=400, help="Total read window in ms (default: 400)")
    ap.add_argument("--silent-break-ms", type=int, default=30, help="End early after this RX silence in ms (default: 30)")
    ap.add_argument("--min-reply-len", type=int, default=12, help="Minimum RX bytes to count as present (default: 12)")
    ap.add_argument("--rx-cap", type=int, default=2048, help="Max RX bytes to collect (default: 2048)")
    ap.add_argument("--inter-addr-ms", type=int, default=250, help="Delay between addresses in scan mode (default: 250)")
    ap.add_argument("--dump", action="store_true", help="Hex dump TX and RX")
    args = ap.parse_args(argv)

    def do_probe(a: int) -> int:
        present, rx = probe_one(args, a)
        if args.dump:
            print(f"TX[{a:02d}]: {hexdump(build_probe(a))}")
            print(f"RX[{a:02d}] ({len(rx)} bytes): {hexdump(rx)}")
        if present:
            print(f"FOUND controller at address {a} (rx={len(rx)} bytes)")
            return 0
        print(f"no reply at address {a} (rx={len(rx)} bytes)")
        return 1

    if args.addr is not None:
        if not (1 <= args.addr <= 64):
            print("--addr must be in range 1..64", file=sys.stderr)
            return 2
        return do_probe(args.addr)

    # scan
    for a in range(1, 65):
        present, rx = probe_one(args, a)
        if args.dump:
            print(f"TX[{a:02d}]: {hexdump(build_probe(a))}")
            print(f"RX[{a:02d}] ({len(rx)} bytes): {hexdump(rx)}")
        if present:
            print(f"FOUND controller at address {a} (rx={len(rx)} bytes)")
            return 0
        print(f"no reply at address {a} (rx={len(rx)} bytes)")
        time.sleep(max(0, args.inter_addr_ms) / 1000.0)

    print("Scan complete. No devices found.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())


