# State-sync review: why HA doesn't reliably reflect the lights (2026-06-07)

## How it's supposed to work (and mostly does)

```
DALI gateway (source of truth)
  └─ FC04 dirty bitmap 0x0104-0x0107  (a ballast's bit = "its level/status changed")
       └─ controller dirty_poll (FC04 read every 100ms while "active")
            └─ for each dirty ballast: FC03 read BLOCK0 → arc + status
                 └─ lighting_emit_dali_state → state_dali JSON
                      └─ bridge → HA light brightness/state
```

The gateway is the source of truth; the controller keeps a copy and emits deltas;
the bridge moves them. The dirty bits are the change-detection shortcut.

## What's broken (reproduced on the rig)

Test: fixture-0/button-0 bound to `2:0,2:1`, tap to toggle, watch emitted `state_dali`.

| action | ballast actually | state_dali emitted |
|---|---|---|
| set `bri=115` | 115 | ✅ on/115 (actual read back) |
| `on=false` | 0 | ✅ off/0 |
| **button toggle → off** | 0,0 | ✅ both off |
| **button toggle → on** | **254,100** | ❌ **0/12 across 6 taps** (once caught by luck) |

The lamps **do** turn on (254/100) — the commands fire — but **no state event is
emitted**, so HA keeps showing them off. Intermittent: occasionally one slips
through, which is exactly "not consistently seeing updates".

## Root causes

**1. The dirty-poll gives up too soon (primary, controller).**
`DIRTY_ZERO_SETTLE_COUNT = 2` with `DIRTY_POLL_INTERVAL_MS = 100` → after a command
the poller deactivates after **2 consecutive zero reads (~200 ms)**.
- **Off** marks dirty *immediately* (gateway `OP_WRITE_LEVEL` → unconditional
  `mark_dirty`), so it lands inside 200 ms → reliable.
- **On / fade** (GOTO_LAST_ACTIVE / UP / DOWN, all RAW_CMD) only mark dirty *after*
  a gateway re-query (`SYNC_READ_LEVEL` → `OP_LEVEL_READ`). For **multi-target**
  that's two RAW_CMDs + DEVICE_BUSY retries + two re-queries — the dirty bit lands
  **after** the 200 ms window, so the poller has already deactivated and misses it.
- Fallback once deactivated is `BACKGROUND_CHECK_MS = 60000` → up to **60 s** to
  self-heal. (Confirmed: setting fade_time=0 did **not** fix it — it's the poll
  window, not the fade.)

**2. The gateway re-reads the level only once, conditionally (gateway).**
`OP_LEVEL_READ` does `if (old_b != value) mark_dirty(idx)` on a single re-query
fired right after the command — it can sample a **pre/mid-fade** value, see no
change vs its cache, and never re-read the settled level. There's no "re-read
after FADING clears".

**3. The controller emits a *guessed* brightness on "on" (controller).**
`set_dali on=true` emits a predicted `bri=254` ("full on") before any read. With #1
making the real read unreliable, HA is left showing the guess — this is the user's
"HA is just assuming the value". The button path emits *no* prediction, so HA shows
nothing at all. Either way HA ≠ device.

**4. (Separate UX) GOTO_LAST_ACTIVE jumps to the last-active level**, often max —
so "on" after a fade to 45% can jump to 100% regardless of sync. Not a sync bug,
but it compounds the surprise.

## Recommendation (in priority order)

**A. Controller dirty-poll — make it reliable + regularly polled (highest impact,
no gateway reflash).** This is the user's "polled on a regular basis":
- Keep a slave "active" for a guaranteed window after *any* command (e.g. ~2.5 s,
  time-based deadline rather than a 2-zero count) so a delayed on/fade/multi-target
  dirty bit is still caught.
- Add a **regular background dirty sweep** of every configured DALI slave at a short
  cadence (≈1 s, always) — one FC04 read per slave — so any change (button,
  external, post-fade settle) reflects within ~1 s even if the active window missed
  it. Drop the 60 s fallback to this.

**B. Gateway — re-read the settled level after a fade.**
- In `OP_LEVEL_READ`, if QUERY_STATUS shows FADING (0x10), keep `SYNC_READ_LEVEL`
  set and re-read until it clears, then `mark_dirty` with the final value.
- Consider marking dirty against the *last-reported* level rather than the cache so
  a real change is never masked. (Gateway reflash.)

**C. Controller — stop guessing brightness on "on".**
- Emit `on=true` with the last-known brightness (or no brightness) and let the
  now-reliable poll deliver the truth. Aligns with "gateway is the source of truth".

**D. (Optional UX)** Decide what "on" should do when already on / after a fade —
no-op vs GOTO_LAST_ACTIVE — so it doesn't jump to max.

A alone fixes the reported intermittency and needs no gateway reflash; B makes fade
levels accurate; C removes the misleading optimism.

## Update 2026-06-07 — B implemented (TinyModbus 307d130)

B turned out to fix the *intermittency too*, not just accuracy — for ballasts that
fade. By keeping `SYNC_READ_LEVEL` set while FADE_RUNNING, the gateway re-reads for
the whole (multi-second) fade, so the dirty bit stays set long enough that the
controller's 100 ms poll reliably catches it and stays active. As a bonus `on` now
reports the real settled level instead of the optimistic 254.

Measured on the rig (fixture-0/btn-0 → `2:0,2:1`, 4 ON taps):

| | before B | after B |
|---|---|---|
| fade_time = 4 (fades) | ~0–2 / 8 | **8 / 8** |
| fade_time = 0 (instant) | ~0–2 / 8 | 2 / 8 |
| single `on` emitted level | guessed 254 | actual 137 |

So **B covers the realistic (fading) case**. **A is still wanted** as the backstop
for instant ballasts (fade_time=0 still races the 200 ms window) and for changes
that arrive with no command (external/manual). C is now lower priority since the
real level reliably follows for fading lamps.

Rig caveat: slave 2 hosts both a relay board and the DALI gateway, so its FC03/FC04
reads occasionally collide (observed `None`/garbled query results). That adds read
noise here that won't exist on a clean per-slave bus.
