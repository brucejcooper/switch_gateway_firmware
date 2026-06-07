# Issue: RAW_CMD_ASYNC — queue fire-and-forget DALI commands

Status: **deferred** (mitigated). Filed 2026-06-07.

## Problem

The TinyModbus gateway exposes a **single-slot** raw DALI channel: `MBREG_RAW_CMD`
(`0x1000`). While a forward frame is in flight, `raw_cmd_result`'s high byte is
`0xFF` (PENDING) and a second write is refused with **DEVICE_BUSY (0x06)**. So when
the controller fires several DALI commands at the *same gateway* back-to-back, only
the first lands.

This bites every **fire-and-forget** command path in `controller/src/lighting_actions.c`:

| path | mechanism | multi-target today |
|---|---|---|
| on — `lighting_set_on_off(on)` (GOTO_LAST_ACTIVE 0x0A) | `dali_raw_arm` → RAW_CMD | only 1st target ❌ |
| fade up/down ballast — `lighting_dali_step` (UP 0x01 / DOWN 0x02) | `dali_raw_arm` → RAW_CMD | only 1st target ❌ |
| fade up/down group — `lighting_group_step` | `dali_raw_arm` → RAW_CMD | only 1st target ❌ |
| off — `lighting_set_on_off(off)` | per-ballast brightness reg `0x0000+addr` (`SYNC_WRITE_LEVEL`) | queued, fine ✅ |

Fades are worse than on/off: the controller re-issues the step every
`dim_repeat_ms` for the whole hold, so the collision recurs on every tick.

**Current mitigation:** `controller@f486d9f` retries the modbus write on
DEVICE_BUSY (~10 ms wait ≈ one DALI frame, up to 6 polls), which serialises the
commands and makes multi-target on + fades work. It's a workaround — it still
issues a RAW_CMD, gets refused, and re-polls each time.

## Proposed fix: `MBREG_RAW_CMD_ASYNC` (`0x1002`)

One new write-only register. A write **enqueues** the 16-bit DALI frame onto a
small ring; the existing `pick_work()` work-selector drains one frame per idle
cycle as a fire-and-forget `DALI_KIND_CMD` — **no backward-frame wait, no
`raw_cmd_result` update, no feedback**. The completion handler just releases
`dali_busy`. New low-priority slot, below the single OP_RAW slot, so it can't
clobber commissioning/DTR0.

- Ring is push (main, FC06/FC10 handler) + pop (main, `pick_work` under its ATOMIC/
  TCB0 fence); the completion ISR never touches it → no race.
- Returns DEVICE_BUSY only when the ring is full; the controller's existing
  DEVICE_BUSY retry is then the backstop (waits for a slot). For a continuous fade
  a dropped step is harmless (next tick continues).
- Controller: point `lighting_set_on_off(on)`, `lighting_dali_step`,
  `lighting_group_step` at `0x1002` instead of `dali_raw_arm`. Keep the retry.

## RAM/flash budget (ATtiny1604: 1024 B SRAM, 16 KB flash)

- Static SRAM now: **863 B** (`g_regs` alone is 672 B) → **161 B left for stack**.
- Flash now: **15624 / 16384 ≈ 95 %** → ~760 B free.
- Ring cost: depth 4 = +10 B (stack → 151), depth 8 = +18 B (→ 143). Drain slot +
  register handler + completion ≈ 200–250 B flash.

**Recommendation:** depth **4** + the DEVICE_BUSY retry as the ring-full backstop —
spends RAM only on the common case while staying correct for any fan-out. Both
margins are thin, so revisit if more static RAM gets added first.
