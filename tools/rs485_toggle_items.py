#!/usr/bin/env python3
"""
Toggle binary outputs (items 49 and 50) on the Teletrol controller via RS‑485.

Two command types exist on the wire:
- Set value (ON/OFF): 00 <ADDR> FF FB 10 7D 77 01 00 <ITEM> 00 01 21 00 <VAL> <CHK>
  CHK = (0x21 + ADDR + ITEM + VAL) & 0xFF
- Commit/save:        00 <ADDR> FF FB 10 7D 77 01 00 <ITEM> 00 01 00 00 00 <CHK>
  CHK = (ADDR + ITEM) & 0xFF

Defaults match existing tools/firmware: 19200 baud, 8O1, 5 ms quiet before TX,
100 ms TX→RX turnaround, 400 ms read window, 30 ms silent gap to end early.
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


# ---------------------- Frame builders ---------------------- #
def build_set_frame(addr: int, item: int, val: int) -> bytes:
    """
    00 <ADDR> FF FB 10 7D 77 01 00 <ITEM> 00 01 21 00 <VAL> <CHK>
    """
    if not (0 <= val <= 1):
        raise ValueError("val must be 0 or 1")
    chk = (0x21 + addr + item + val) & 0xFF
    return bytes(
        [
            0x00,
            addr & 0xFF,
            0xFF,
            0xFB,
            0x10,
            0x7D,
            0x77,
            0x01,
            0x00,
            (item >> 8) & 0xFF,
            item & 0xFF,
            0x00,
            0x01,
            0x21,
            0x00,
            val & 0xFF,
            chk,
        ]
    )


def build_commit_frame(addr: int, item: int) -> bytes:
    """
    00 <ADDR> FF FB 10 7D 77 01 00 <ITEM> 00 01 00 00 00 <CHK>
    CHK = (addr + item) & 0xFF
    """
    chk = (addr + item) & 0xFF
    return bytes(
        [
            0x00,
            addr & 0xFF,
            0xFF,
            0xFB,
            0x10,
            0x7D,
            0x77,
            0x01,
            0x00,
            (item >> 8) & 0xFF,
            item & 0xFF,
            0x00,
            0x01,
            0x00,
            0x00,
            0x00,
            chk,
        ]
    )


# ---------------------- Serial helpers ---------------------- #
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


def read_window(
    ser: serial.Serial,
    total_window_s: float,
    silent_break_s: float,
    cap: int,
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

        if last_rx is not None and (now - last_rx) >= silent_break_s:
            break

    return bytes(buf)


def hexdump(data: bytes) -> str:
    return " ".join(f"{x:02X}" for x in data)


# ---------------------- TX/RX helpers ---------------------- #
def send_frame(args: argparse.Namespace, frame: bytes) -> bytes:
    ser = open_serial(args)
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        time.sleep(max(0, args.pre_quiet_ms) / 1000.0)

        ser.write(frame)
        ser.flush()

        time.sleep(max(0, args.turnaround_ms) / 1000.0)

        rx = read_window(
            ser,
            total_window_s=max(1, args.read_window_ms) / 1000.0,
            silent_break_s=max(1, args.silent_break_ms) / 1000.0,
            cap=args.rx_cap,
        )
        return rx
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ---------------------- CLI ---------------------- #
def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Toggle item 49/50 ON/OFF over RS-485 and optionally commit/save.")
    ap.add_argument("--port", required=True, help="COM port for USB-RS485 adapter (e.g. COM4)")
    ap.add_argument("--addr", type=int, default=4, help="Controller address (default: 4)")
    ap.add_argument("--item", type=int, default=49, help="Item number (default: 49; use 50 for the second output)")
    ap.add_argument("--value", type=int, choices=(0, 1), default=1, help="Value to set: 1=ON, 0=OFF (default: 1)")
    ap.add_argument("--baud", type=int, default=19200, help="Baud rate (default: 19200)")
    ap.add_argument("--parity", choices=("O", "N"), default="O", help="Parity: O=odd, N=none (default: O)")
    ap.add_argument("--timeout-ms", type=int, default=10, help="Serial read timeout in ms (default: 10)")
    ap.add_argument("--pre-quiet-ms", type=int, default=5, help="Quiet time before TX in ms (default: 5)")
    ap.add_argument("--turnaround-ms", type=int, default=100, help="Delay after TX before reading (default: 100)")
    ap.add_argument("--read-window-ms", type=int, default=400, help="Total read window in ms (default: 400)")
    ap.add_argument("--silent-break-ms", type=int, default=30, help="End early after this RX silence in ms (default: 30)")
    ap.add_argument("--rx-cap", type=int, default=2048, help="Max RX bytes to collect (default: 2048)")
    ap.add_argument("--no-commit", action="store_true", help="Do not send the commit/save frame after setting the value")
    ap.add_argument("--commit-only", action="store_true", help="Only send commit/save (skip set-value)")
    ap.add_argument("--dump", action="store_true", help="Hex dump TX/RX for each frame")
    args = ap.parse_args(argv)

    addr = args.addr
    item = args.item
    val = args.value

    if not (0 <= addr <= 0xFF):
        print("--addr must be 0..255", file=sys.stderr)
        return 2
    if not (0 <= item <= 0xFFFF):
        print("--item must be 0..65535", file=sys.stderr)
        return 2

    # 1) Set-value frame (optional if commit-only)
    if not args.commit_only:
        tx_set = build_set_frame(addr, item, val)
        rx_set = send_frame(args, tx_set)
        if args.dump:
            print(f"[set] TX ({len(tx_set)}): {hexdump(tx_set)}")
            print(f"[set] RX ({len(rx_set)}): {hexdump(rx_set)}")
        else:
            print(f"Set item {item} to {val} (TX {len(tx_set)} bytes, RX {len(rx_set)} bytes)")

    # 2) Commit/save frame (optional)
    if not args.no_commit:
        tx_commit = build_commit_frame(addr, item)
        rx_commit = send_frame(args, tx_commit)
        if args.dump:
            print(f"[commit] TX ({len(tx_commit)}): {hexdump(tx_commit)}")
            print(f"[commit] RX ({len(rx_commit)}): {hexdump(rx_commit)}")
        else:
            print(f"Commit item {item} (TX {len(tx_commit)} bytes, RX {len(rx_commit)} bytes)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
