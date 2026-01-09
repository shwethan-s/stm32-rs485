# Check Address (STM32 RS‑485)

Plain-language guide to how the STM32 looks for a controller’s current address on the RS‑485 bus. This mirrors the logic in `src/main.c`.

## What it does
- Sends a small “probe” frame to an address.
- Waits a short time for any reply.
- If a reply is long enough, that address is considered present.
- A worker scans addresses 1–64 and stops at the first responder.

## Probe frame format
Bytes sent (15 bytes total):
- `3F 3F 00 <ADDR> FF FB 0D 7E 77 01 00 01 00 40 <CHK>`
- `<ADDR>` is the address being tested (1–64).
- `<CHK>` is a 1‑byte checksum: `(ADDR + 0x3E) & 0xFF`.

## Timing used on the STM32
- Quiet before send: `PRE_SEND_QUIET_MS = 5 ms`
- TX→RX delay: `TURNAROUND_DELAY_MS = 100 ms`
- Read window: `READ_WINDOW_MS = 400 ms`
- Early stop after silence: `SILENT_BREAK_MS = 30 ms`
- Minimum bytes to count as “present”: `MIN_VALID_REPLYLEN = 12`
- Delay between addresses when scanning: `INTER_ADDR_DELAY_MS = 250 ms`

## Step-by-step (per address)
1. Clear the UART RX FIFO (avoid stale bytes).
2. Wait 5 ms of bus quiet.
3. Send the 15‑byte probe frame (no manual DE/RE toggle; hardware handles direction).
4. Wait 100 ms for turnaround.
5. Listen up to 400 ms, but stop early if 30 ms of silence occurs after receiving something.
6. If at least 12 bytes were received, the address is considered found; otherwise it is marked absent or “short reply”.

## Scan behavior
- The scan thread walks addresses 1 through 64.
- For each address it updates the UI status text (e.g., “Checking addr 7...”).
- It stops at the first address that produces a valid reply and reports “Controller address is X”.
- If no addresses respond, it reports “Scan complete. No devices.”

## How to interpret outcomes
- Found: reply length ≥ 12 bytes → address is present.
- Short reply (<12 bytes): something replied but not enough to trust; treated as not found.
- No reply: the address is absent or the bus/device is silent.

