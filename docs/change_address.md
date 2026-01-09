# Change Address (STM32 RS‑485)

Plain-language guide to how the STM32 changes a controller’s RS‑485 address and verifies the result. This mirrors the logic in `src/main.c`.

## What it does
- Optionally discovers the current address (if not provided) by probing 1–64.
- Sends a “change address” frame: old → new.
- Waits briefly for any reply and for the device to commit the change.
- Probes both the new and old addresses to confirm the move.

## Change-frame format
Bytes sent (16 bytes total):
- `00 <OLD> FF FB 10 7D 77 01 00 41 00 01 21 00 <NEW> <CHK>`
- `<OLD>` is the current address; `<NEW>` is the desired address.
- `<CHK>` is a 1‑byte checksum: `(0x62 + OLD + NEW) & 0xFF`.

## Timing used on the STM32
- Quiet before send: `PRE_SEND_QUIET_MS = 5 ms`
- TX→RX delay: `TURNAROUND_DELAY_MS = 100 ms`
- Read window after send: `READ_WINDOW_MS = 400 ms`
- Early stop after silence: `SILENT_BREAK_MS = 30 ms`
- Minimum bytes considered a valid reply: `MIN_VALID_REPLYLEN = 12`
- Delay between probes when searching: `INTER_ADDR_DELAY_MS = 250 ms`
- Extra settle time after change write: ~150 ms before verification probes.

## Step-by-step flow
1. **Find current address (if needed):** probe addresses 1–64 using the check logic until one replies; otherwise report failure.
2. **Prepare to send:** flush UART RX, wait 5 ms of quiet.
3. **Send change frame:** transmit the 16‑byte frame with old/new/checksum; hardware handles RS‑485 direction automatically.
4. **Turnaround:** wait 100 ms, then listen up to 400 ms (stop early after 30 ms of silence).
5. **Settle:** wait ~150 ms to let the device commit the new address.
6. **Verify:** probe the new address, then the old address:
   - Success: replies at new address and not at old address.
   - Warning: replies at both (device responds on both addresses).
   - Failed move: replies only at old address.
   - No response: neither address replies after the change attempt.

## How to interpret replies during change
- The change write itself may or may not produce a reply; logs note “no reply” or “RX N bytes”.
- Final status is determined only by the post-change probes of new and old addresses (see above).

