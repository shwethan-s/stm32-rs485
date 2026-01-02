#!/usr/bin/env python3
"""
RS-485 passive sniffer (serial RX).

Goal: capture raw bytes with timestamps and split into "frames" using a silent-gap heuristic.
Note: RS-485 is half-duplex multi-drop; this tool cannot know direction (master->slave vs slave->master)
unless you also capture a DE/RE signal or tap TTL-side UARTs. It will still show the exact bytes on the bus.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import IO, Optional

try:
    import serial  # pyserial
except Exception as e:  # pragma: no cover
    print("Missing dependency: pyserial. Install with: python -m pip install pyserial", file=sys.stderr)
    raise


def iso_utc_now() -> str:
    # e.g. 2026-01-02T12:34:56.789Z 
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def hexdump(data: bytes, group: int = 1) -> str:
    if group <= 1:
        return " ".join(f"{b:02X}" for b in data)
    out = []
    for i in range(0, len(data), group):
        chunk = data[i : i + group]
        out.append("".join(f"{b:02X}" for b in chunk))
    return " ".join(out)


def asciidump(data: bytes) -> str:
    return "".join(chr(b) if 32 <= b <= 126 else "." for b in data)


def parse_parity(p: str) -> str:
    p = p.upper()
    if p in ("N", "NONE"):
        return serial.PARITY_NONE
    if p in ("E", "EVEN"):
        return serial.PARITY_EVEN
    if p in ("O", "ODD"):
        return serial.PARITY_ODD
    if p in ("M", "MARK"):
        return serial.PARITY_MARK
    if p in ("S", "SPACE"):
        return serial.PARITY_SPACE
    raise argparse.ArgumentTypeError("parity must be one of: N,E,O,M,S")


@dataclass
class Frame:
    ts_first_utc: str
    ts_last_utc: str
    t_first_ms: float
    t_last_ms: float
    gap_ms: float
    length: int
    hex: str
    ascii: str


def write_frame_text(out: IO[str], idx: int, f: Frame) -> None:
    out.write(
        f"[{idx:05d}] first={f.ts_first_utc} last={f.ts_last_utc} "
        f"len={f.length} gap={f.gap_ms:.1f}ms\n"
    )
    out.write(f"  HEX  : {f.hex}\n")
    out.write(f"  ASCII: {f.ascii}\n")
    out.flush()


def write_frame_jsonl(out: IO[str], f: Frame) -> None:
    out.write(json.dumps(asdict(f), separators=(",", ":")) + "\n")
    out.flush()


def main(argv: Optional[list[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Passive RS-485 sniffer with timestamped frame splitting.")
    ap.add_argument("--port", required=True, help="Serial port (e.g. COM7)")
    ap.add_argument("--baud", type=int, default=19200, help="Baud rate (default: 19200)")
    ap.add_argument("--bytesize", type=int, default=8, choices=(5, 6, 7, 8), help="Data bits (default: 8)")
    ap.add_argument("--parity", type=parse_parity, default=serial.PARITY_ODD, help="Parity: N,E,O,M,S (default: O)")
    ap.add_argument("--stopbits", type=float, default=1, choices=(1, 1.5, 2), help="Stop bits (default: 1)")
    ap.add_argument("--timeout-ms", type=int, default=10, help="Serial read timeout in ms (default: 10)")
    ap.add_argument("--gap-ms", type=int, default=30, help="Silent gap that ends a frame in ms (default: 30)")
    ap.add_argument("--max-frame", type=int, default=4096, help="Max bytes per frame before forced flush (default: 4096)")
    ap.add_argument(
        "--format",
        choices=("text", "jsonl"),
        default="text",
        help="Output format: text or jsonl (default: text)",
    )
    ap.add_argument("--out", default="-", help="Output file path, or '-' for stdout (default: '-')")
    ap.add_argument("--hex-group", type=int, default=1, help="Group hex bytes (1=bytewise, 2=words, etc.)")
    args = ap.parse_args(argv)

    timeout_s = max(args.timeout_ms, 1) / 1000.0
    gap_s = max(args.gap_ms, 1) / 1000.0

    out_f: IO[str]
    if args.out == "-":
        out_f = sys.stdout
    else:
        out_f = open(args.out, "a", encoding="utf-8", newline="\n")

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=args.bytesize,
        parity=args.parity,
        stopbits=args.stopbits,
        timeout=timeout_s,
        rtscts=False,
        dsrdtr=False,
    )

    # Avoid printing stale bytes from earlier sessions.
    try:
        ser.reset_input_buffer()
    except Exception:
        pass

    print(
        f"Sniffing {args.port} @ {args.baud} {args.bytesize}{ser.parity}{args.stopbits} "
        f"(timeout={args.timeout_ms}ms, gap={args.gap_ms}ms). Ctrl+C to stop.",
        file=sys.stderr,
    )

    buf = bytearray()
    idx = 0

    first_rx_mono: Optional[float] = None
    last_rx_mono: Optional[float] = None
    first_rx_utc: Optional[str] = None

    def flush_frame(gap_ms: float) -> None:
        nonlocal buf, idx, first_rx_mono, last_rx_mono, first_rx_utc
        if not buf or first_rx_mono is None or last_rx_mono is None or first_rx_utc is None:
            return

        idx += 1
        f = Frame(
            ts_first_utc=first_rx_utc,
            ts_last_utc=iso_utc_now(),
            t_first_ms=first_rx_mono * 1000.0,
            t_last_ms=last_rx_mono * 1000.0,
            gap_ms=gap_ms,
            length=len(buf),
            hex=hexdump(bytes(buf), group=max(1, args.hex_group)),
            ascii=asciidump(bytes(buf)),
        )

        if args.format == "jsonl":
            write_frame_jsonl(out_f, f)
        else:
            write_frame_text(out_f, idx, f)

        buf.clear()
        first_rx_mono = None
        last_rx_mono = None
        first_rx_utc = None

    try:
        while True:
            data = ser.read(4096)
            now_mono = time.monotonic()

            if data:
                if not buf:
                    first_rx_mono = now_mono
                    first_rx_utc = iso_utc_now()
                buf.extend(data)
                last_rx_mono = now_mono

                if len(buf) >= args.max_frame:
                    flush_frame(gap_ms=0.0)
                continue

            # no data this tick; if we have a partial frame and we've been silent long enough, flush it
            if buf and last_rx_mono is not None:
                silent_s = now_mono - last_rx_mono
                if silent_s >= gap_s:
                    flush_frame(gap_ms=silent_s * 1000.0)

    except KeyboardInterrupt:
        # flush any tail
        if buf and last_rx_mono is not None:
            flush_frame(gap_ms=0.0)
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass
        if out_f is not sys.stdout:
            out_f.close()


if __name__ == "__main__":
    raise SystemExit(main())


