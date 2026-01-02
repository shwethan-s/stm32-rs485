#!/usr/bin/env python3
"""
Change the controller address over RS-485 from a PC (no STM32 needed).

Protocol is inferred from bus captures (TSCView "change address"):
Write frame (16 bytes):
    00 <OLD> FF FB 10 7D 77 01 00 41 00 01 21 00 <NEW> <CHK>
with:
    CHK = (0x62 + OLD + NEW) & 0xFF

Defaults match your firmware: 19200 baud, 8 data bits, odd parity, 1 stop (8O1),
100 ms turnaround, 400 ms read window, 30 ms silent-break.
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


# ---------------------- Probe (same as firmware) ---------------------- #
PROBE_TEMPLATE = bytes(
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


def probe_checksum(addr: int) -> int:
    return (addr + 0x3E) & 0xFF


def build_probe(addr: int) -> bytes:
    b = bytearray(PROBE_TEMPLATE)
    b[3] = addr & 0xFF
    b[14] = probe_checksum(addr)
    return bytes(b)


# ---------------------- Change-address write frame ---------------------- #
def build_change_frame(old_addr: int, new_addr: int) -> bytes:
    """
    00 <OLD> FF FB 10 7D 77 01 00 41 00 01 21 00 <NEW> <CHK>
    CHK = (0x62 + OLD + NEW) & 0xFF
    """
    chk = (0x62 + old_addr + new_addr) & 0xFF
    return bytes(
        [
            0x00,
            old_addr & 0xFF,
            0xFF,
            0xFB,
            0x10,
            0x7D,
            0x77,
            0x01,
            0x00,
            0x41,
            0x00,
            0x01,
            0x21,
            0x00,
            new_addr & 0xFF,
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


# ---------------------- Probe logic (scan or single) ---------------------- #
def probe_once(args: argparse.Namespace, addr: int) -> Tuple[bool, bytes]:
    ser = open_serial(args)
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        time.sleep(max(0, args.pre_quiet_ms) / 1000.0)

        tx = build_probe(addr)
        ser.write(tx)
        ser.flush()

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


def scan_for_first(args: argparse.Namespace) -> Optional[int]:
    for a in range(1, args.max_addr + 1):
        present, _ = probe_once(args, a)
        if present:
            return a
        time.sleep(max(0, args.inter_addr_ms) / 1000.0)
    return None


# ---------------------- Change address operation ---------------------- #
def change_address(args: argparse.Namespace, old_addr: int, new_addr: int) -> Tuple[bytes, bytes]:
    ser = open_serial(args)
    try:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass

        time.sleep(max(0, args.pre_quiet_ms) / 1000.0)

        tx = build_change_frame(old_addr, new_addr)
        ser.write(tx)
        ser.flush()

        time.sleep(max(0, args.turnaround_ms) / 1000.0)

        rx = read_window(
            ser,
            total_window_s=max(1, args.read_window_ms) / 1000.0,
            silent_break_s=max(1, args.silent_break_ms) / 1000.0,
            cap=args.rx_cap,
        )
        return tx, rx
    finally:
        try:
            ser.close()
        except Exception:
            pass


# ---------------------- CLI ---------------------- #
def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Change controller address over RS-485 (PC-side, no STM32).")
    ap.add_argument("--port", required=True, help="COM port for your USB-RS485 adapter (e.g. COM4)")
    ap.add_argument("--baud", type=int, default=19200, help="Baud rate (default: 19200)")
    ap.add_argument("--parity", choices=("O", "N"), default="O", help="Parity: O=odd, N=none (default: O)")

    ap.add_argument("--new-addr", type=int, required=True, help="New address to set (1..63)")
    ap.add_argument("--old-addr", type=int, help="Current address (1..63). If omitted, scan 1..63 first.")

    ap.add_argument("--timeout-ms", type=int, default=10, help="Serial read timeout in ms (default: 10)")
    ap.add_argument("--pre-quiet-ms", type=int, default=5, help="Quiet time before TX in ms (default: 5)")
    ap.add_argument("--turnaround-ms", type=int, default=100, help="Delay after TX before reading (default: 100)")
    ap.add_argument("--read-window-ms", type=int, default=400, help="Total read window in ms (default: 400)")
    ap.add_argument("--silent-break-ms", type=int, default=30, help="End early after this RX silence in ms (default: 30)")
    ap.add_argument("--min-reply-len", type=int, default=12, help="Minimum RX bytes to count as present (default: 12)")
    ap.add_argument("--rx-cap", type=int, default=2048, help="Max RX bytes to collect (default: 2048)")
    ap.add_argument("--inter-addr-ms", type=int, default=250, help="Delay between addresses when scanning (default: 250)")
    ap.add_argument("--max-addr", type=int, default=63, help="Highest address to scan (default: 63)")
    ap.add_argument("--dump", action="store_true", help="Hex dump TX/RX for change + final probe")
    ap.add_argument("--no-confirm", action="store_true", help="Skip probing new address after write")
    args = ap.parse_args(argv)

    if not (1 <= args.new_addr <= args.max_addr):
        print(f"--new-addr must be in range 1..{args.max_addr}", file=sys.stderr)
        return 2

    old_addr = args.old_addr
    if old_addr is not None:
        if not (1 <= old_addr <= args.max_addr):
            print(f"--old-addr must be in range 1..{args.max_addr}", file=sys.stderr)
            return 2
    else:
        print(f"[scan] looking for current address 1..{args.max_addr} ...")
        found = scan_for_first(args)
        if found is None:
            print("No device responded during scan.")
            return 1
        old_addr = found
        print(f"[scan] found device at address {old_addr}")

    if old_addr == args.new_addr:
        print("Old and new address are the same; nothing to do.")
        return 0

    # Send change command
    tx, rx = change_address(args, old_addr, args.new_addr)
    if args.dump:
        print(f"[write] TX ({len(tx)}): {hexdump(tx)}")
        print(f"[write] RX ({len(rx)}): {hexdump(rx)}")
    else:
        print(f"Sent change-address {old_addr} -> {args.new_addr}, got {len(rx)} bytes back.")

    if args.no_confirm:
        return 0

    # Confirm by probing new address
    present_new, rx_new = probe_once(args, args.new_addr)
    present_old, rx_old = probe_once(args, old_addr)

    if args.dump:
        print(f"[probe new {args.new_addr}] present={present_new} rx_len={len(rx_new)} {hexdump(rx_new)}")
        print(f"[probe old {old_addr}] present={present_old} rx_len={len(rx_old)} {hexdump(rx_old)}")

    if present_new and not present_old:
        print(f"SUCCESS: now responds at {args.new_addr} (no response at {old_addr}).")
        return 0

    if present_new and present_old:
        print(f"WARN: responds at both {old_addr} and {args.new_addr}.")
        return 0

    if not present_new and present_old:
        print(f"FAILED: still only responds at {old_addr}, not at {args.new_addr}.")
        return 1

    print(f"UNKNOWN: no response at {args.new_addr}, and {old_addr} also silent.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())


