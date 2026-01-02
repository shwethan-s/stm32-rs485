### RS-485 Protocol Notes (inferred from sniffer captures)

This document summarizes **patterns observed** on the RS‑485 bus while using TSCView to **change controller addresses**.

These notes are **reverse-engineered from captures**, not vendor documentation. The goal is to help implement
“Change Address” in `src/main.c` without guessing.

---

### Capture A: change address **6 → 3**

**Write/change command (16 bytes):**

```
00 06 FF FB 10 7D 77 01 00 41 00 01 21 00 03 6B
```

**ACK/status (13 bytes):**

```
FF FB 00 06 0D 26 7D 01 00 41 00 01 F3
```

**Repeated poll/confirm (13 bytes), repeated many times:**

```
00 06 FF FB 0D 7E 77 01 00 41 00 01 45
```

---

### Capture B: change address **3 → 7**

**Write/change command (16 bytes):**

```
00 03 FF FB 10 7D 77 01 00 41 00 01 21 00 07 6C
```

**ACK/status (13 bytes):**

```
FF FB 00 03 0D 26 7D 01 00 41 00 01 F0
```

**Repeated poll/confirm (13 bytes), repeated many times:**

```
00 03 FF FB 0D 7E 77 01 00 41 00 01 42
```

---

### Similarities (what stays constant)

Across both “change address” captures, these bytes appear stable in the same positions:

- **`FF FB`**: looks like a **header/sync marker** used by this protocol family.
- **`77 01 00 41 00 01`**: appears in multiple frames; likely a **command group** and/or **parameter/register selector**.
- **`7D` and `7E`**: appear consistently as “special” marker bytes in the middle of frames.
  - Many serial framing protocols use `0x7E` as a delimiter and `0x7D` as an escape byte.
  - We’re not fully decoding framing here; we’re just noting the pattern.

---

### Differences (what changes and what it likely means)

#### A) “Current address” field

In each frame, the first two bytes include the address in the form:

- `00 <ADDR>`

Examples:
- `00 06` when talking to address 6
- `00 03` when talking to address 3

So `ADDR` is almost certainly the controller’s current address (or a logical address field used by the tool).

#### B) “New address” field (only in the 16‑byte write)

The 16‑byte write frame contains `00 <NEW_ADDR>` near the end:

- 6→3: `... 21 00 03 6B`
- 3→7: `... 21 00 07 6C`

So `<NEW_ADDR>` is very likely encoded as that `00 xx` pair.

---

### Checksum formulas (inferred)

In each frame, the **last byte** changes in a way that strongly matches simple checksum rules.
We infer a checksum as a function of address fields because:
- only address fields change between captures
- the last byte changes predictably with those address changes

All arithmetic below is **mod 256** (keep the lowest 8 bits).

#### 1) Write/change command checksum (16‑byte frame)

Write frame structure (inferred):

```
00 <OLD> FF FB 10 7D 77 01 00 41 00 01 21 00 <NEW> <CHK>
```

Checksum rule that matches both captures: `CHK = (0x62 + OLD + NEW) mod 256`

Validation:
- For 6→3: `0x62 + 0x06 + 0x03 = 0x6B` ✅
- For 3→7: `0x62 + 0x03 + 0x07 = 0x6C` ✅

#### 2) Poll/confirm checksum (13‑byte repeated frame)

Poll frame ends with:
- addr 6: `... 45`
- addr 3: `... 42`

Checksum rule that matches: `CHK = (0x3F + ADDR) mod 256`

Validation:
- `0x3F + 0x06 = 0x45` ✅
- `0x3F + 0x03 = 0x42` ✅

#### 3) ACK/status checksum (13‑byte `FF FB ...` frame)

ACK frame ends with:
- addr 6: `... F3`
- addr 3: `... F0`

Checksum rule that matches: `CHK = (0xED + ADDR) mod 256`

Validation:
- `0xED + 0x06 = 0xF3` ✅
- `0xED + 0x03 = 0xF0` ✅

---

### Practical takeaway for implementing “Change Address”

Based on these captures:
- The “write new address” command is likely the 16‑byte frame above.
- A minimal implementation could:
  - send the 16‑byte write frame to `OLD`
  - wait for a short ACK/status
  - optionally poll/confirm (the repeated 13‑byte command) and/or re-scan addresses

---

### Open questions / what would increase confidence

To be more confident, capture more examples:
- 3→4, 3→5, 7→1, etc.
- Confirm whether the `00` prefix before `ADDR` and `NEW` is always `00` (or sometimes other values).
- Capture what happens when a write fails (wrong checksum, busy bus, etc.).


