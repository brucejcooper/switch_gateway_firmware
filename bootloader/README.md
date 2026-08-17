# OTA bootloader

A separate pico-sdk executable, `bootloader.elf`/`bootloader.uf2`, that lives
in the first 56 KB of flash (`BOOTLOADER_OFF=0x0 .. 0xE000`,
`src/partition_map.h`) and decides — on every boot, before the main
`light_switch` application ever runs — whether to boot the active app slot
as-is, install a freshly-staged OTA update into it first, or refuse to boot
anything unverifiable at all.

**This is a standalone CMake project**, built entirely separately from the
main firmware:

```sh
cd bootloader
mkdir -p build && cd build
cmake ..
make bootloader          # -> bootloader.elf / .bin / .uf2
```

It is never `add_subdirectory()`'d into `../CMakeLists.txt` and building it
does not touch that build tree (`make -C ../build light_switch` is
unaffected either way) — see "Status: what is / isn't validated" below for
why it isn't wired up any further than this yet.

## Flash layout (src/partition_map.h — frozen contract, don't change here)

```
0x000000 .. 0x00E000   (56 KB)   bootloader        <- this project (incl. boot2)
0x00E000 .. 0x00F000   ( 4 KB)   boot-control A     src/boot_control.c
0x00F000 .. 0x010000   ( 4 KB)   boot-control B     src/boot_control.c
0x010000 .. 0x0A0000   (576 KB)  active app         APP_OFF,  XIP 0x10010000
0x0A0000 .. 0x130000   (576 KB)  staging            STAGING_OFF, XIP 0x100A0000
0x130000 .. 0x1C0000   (576 KB)  backup             BACKUP_OFF, XIP 0x10130000 (true-rollback phase)
0x1C0000 .. 0x200000   (256 KB)  littlefs           FS_OFF (unchanged)
```

The bootloader's own linker region is pinned to exactly `0x0..0xE000` via a
**dedicated linker script**, `memmap_bootloader.ld` — a byte-for-byte copy of
pico-sdk's `memmap_default.ld` except its `FLASH` `MEMORY` region is
hardcoded to `ORIGIN = 0x10000000, LENGTH = 0xE000` instead of
`INCLUDE "pico_flash_region.ld"`, wired in via `pico_set_linker_script(
bootloader memmap_bootloader.ld)` in `CMakeLists.txt`. A build that ever
grows past 56 KB fails loudly with a "region `FLASH' overflowed" link error,
rather than silently overlapping the boot-control sectors. **Keep that
file's `LENGTH` in sync with `BOOTLOADER_SIZE` in `../src/partition_map.h`**
— they express the same fact in two build systems that can't share a symbol.

**This used to be done differently, and it was a real, confirmed-on-hardware-
analysis bug: an earlier revision set `set(PICO_FLASH_SIZE_BYTES "0xE000")`
in `CMakeLists.txt` before `pico_sdk_init()`, intending it as a "linker-only"
setting.** It is not. Setting that CMake variable before the board header is
processed makes pico-sdk's `cmake/generic_board.cmake` treat it as a user
override of the board's own default (`PICO_BOARD_CMAKE_OVERRIDES`), and
`pico_base_headers`' `CMakeLists.txt` then unconditionally promotes every
name in that list into a **global** `target_compile_definitions(
pico_base_headers INTERFACE PICO_FLASH_SIZE_BYTES=0xE000)` —
`pico_base_headers` being an `INTERFACE` dependency of essentially every
pico-sdk library, including `hardware_flash`. `hardware_flash.c`'s
`flash_range_erase()`/`flash_range_program()` both do
`hard_assert(flash_offs + count <= PICO_FLASH_SIZE_BYTES)`, so with that
global define in place this bootloader would have panicked (`hard_assert`
failure) the very first time it erased or programmed anything at or past
`BOOTCTRL_A_OFF`/`APP_OFF` (`0xE000`/`0x10000`) on real hardware — i.e. every
real `try_commit_staging()`/`bc_write()` call, since `0x10000 > 0xE000`. This
was confirmed by inspecting the generated `build/compile_commands.json`
(`hardware_flash/flash.c` was compiled with `-DPICO_FLASH_SIZE_BYTES=0xE000`
despite nothing in `CMakeLists.txt` naming that translation unit) and, after
the fix below, by disassembling `flash_range_erase`'s bounds check in the
built ELF and confirming its immediate is `0x200000` (2 MB, `128 << 14`) —
the chip's true size — not `0xE000`.

The fix is the linker-script approach described above: `PICO_FLASH_SIZE_BYTES`
is now **never set as a CMake variable** in this project, so it keeps
flowing through from the board header's true 2 MB chip size
(`wiznet_w5100s_evb_pico.h`) as the one and only value `hardware_flash.c`
ever sees, while `memmap_bootloader.ld` alone constrains where *this
executable's own* code+data may be linked — a link-time-only concern with no
C-visible macro at all, so it cannot leak into a bounds-check macro the way
a CMake variable override can.

Runs with **only one core active** — `main.c` never calls
`multicore_launch_core1()` — and with **no USB, no network**
(`pico_enable_stdio_usb`/`pico_enable_stdio_uart` are both explicitly off,
and nothing in this program touches `pico_stdio` at all). Because there is
no second core to lock out of concurrent XIP access, flash writes go
straight through `flash_range_erase`/`flash_range_program` with no
`flash_safe_execute()` wrapper — that call exists in the *app* (`ota_stage.c`,
`boot_control.c`) specifically because the app *does* run two cores.

## The decision core: `boot_decide()` (pure, host-tested)

`boot_decide.h`/`boot_decide.c` implement the actual policy as one small,
pure function with no hardware access at all:

```c
typedef enum { BOOT_ACTIVE, BOOT_COMMIT_STAGING, BOOT_ROLLBACK, BOOT_STUCK } boot_action_t;
boot_action_t boot_decide(const struct boot_control *bc, bool bc_valid, unsigned max_boot_attempts);
```

Rules, in priority order (see `boot_decide.h`'s header comment for the full
rationale of each):

1. `!bc_valid` → `BOOT_ACTIVE` — no persisted record at all (first-ever boot,
   or both boot-control sectors corrupt); trust the active image.
2. `bc_valid && (flags & BC_F_PENDING)` → `BOOT_COMMIT_STAGING` — a staged
   image is waiting, regardless of the active image's own state.
3. `bc_valid && (flags & BC_F_CONFIRMED)` (no PENDING) → `BOOT_ACTIVE` —
   already proven good.
4. Unconfirmed, no PENDING, `boot_attempts >= max_boot_attempts` →
   `BOOT_ROLLBACK` — crash loop.
5. Unconfirmed, no PENDING, `boot_attempts < max_boot_attempts` →
   `BOOT_ACTIVE` — still within the retry budget.

`boot_decide()` itself **never returns `BOOT_STUCK`** — it has no access to
flash contents, so it can't know whether an image actually verifies.
`BOOT_STUCK` is produced only by `main.c` reacting to a real `fw_verify()`
failure, or when a `BOOT_ROLLBACK` verdict has nowhere to actually roll back
to (see below).

Exhaustively host-tested in `../test/boot_decide_test.c` across the full
matrix named in the task brief (no valid bc, PENDING with every
confirmed/attempts combination, CONFIRMED at every attempts value, the
attempts-vs-budget boundary at every tested `max_boot_attempts`, and a sweep
asserting `boot_decide()` never emits `BOOT_STUCK`):

```sh
cd test
cc -Wall -Wextra -std=c11 -I ../src -I ../bootloader \
    -o /tmp/boot_decide_test boot_decide_test.c ../bootloader/boot_decide.c \
    && /tmp/boot_decide_test
```

## The hardware layer: `main.c` (thin, NOT host-tested)

`main()` does exactly four things, each mapped straight to the rules above:

1. `bc_read()` the boot-control record, call `boot_decide()`.
2. **`BOOT_COMMIT_STAGING`** (`try_commit_staging()`): `fw_verify()` the
   staged image (`PART_XIP(STAGING_OFF)`, `bc.staged_len`, `FW_PUBKEY`).
   - Fails (bad `staged_len` / bad signature or hash) → **`BOOT_STUCK`**,
     `bc` left **completely untouched**. This does *not* fall back to
     booting the pre-existing active image — see the "no fallback" callout
     below for why that would actually be unsafe, not merely conservative.
   - Succeeds → **commit-time backup (true-rollback phase)**: if
     `bc_has_valid_backup(&bc)` (`boot_decide.c` — `BC_F_BACKUP_VALID` set
     and `backup_len` in `(0, BACKUP_SIZE]`), the CURRENTLY active image
     (about to be overwritten) is copied into `BACKUP` first, XIP-safe
     (`flash_copy_region(APP_OFF, BACKUP_OFF, bc.backup_len)`), and that
     copy is immediately re-verified with `fw_verify()`. If the backup copy
     doesn't verify → **`BOOT_STUCK`**, *without having touched active at
     all* — the backup step runs entirely before the active-slot erase
     below. If `BC_F_BACKUP_VALID` isn't set (no outgoing image was ever
     itself `CONFIRMED` — most notably the very first-ever OTA, over the
     trailer-less factory image), the backup step is skipped entirely:
     there is simply no fallback yet for that image.
   - Then `flash_copy_region(STAGING_OFF, APP_OFF, bc.staged_len)`: sector
     by sector, **XIP-read the source chunk into a RAM buffer first, then
     `flash_range_erase`+`flash_range_program` the destination sector from
     that RAM copy** — the RP2040 cannot serve an XIP read of *any* flash
     region while an erase/program is in flight, so the source bytes must
     already be in RAM before the destination sector is touched. This one
     generic sector-copy helper is shared by all three flash-to-flash
     copies in this file (staging→active, active→backup, backup→active).
     Then **re-verify the active slot** (`fw_verify(PART_XIP(APP_OFF), ...)`).
     - Re-verify fails → leave `bc` completely untouched (`BC_F_PENDING`
       still set, so the next boot retries the same commit from the
       still-intact staging copy) and go straight to `BOOT_STUCK` — refuse
       to jump into a slot that didn't come out right.
     - Re-verify succeeds → clear `BC_F_PENDING` **in RAM only**; the
       `bc_write()` that persists it is deferred to step 4 below, so the
       whole commit-and-boot sequence costs exactly one flash write to
       boot-control (one `seq` bump), not two.

   **No fallback on a rejected candidate, and why:** `ota_stage_finish()`
   (`src/ota_stage.c`, already-landed and frozen) writes a *brand-new*
   boot-control record the instant staging completes — `staged_len` and
   `staged_sha512` become the **candidate's**, unconditionally, with
   whatever length the *previous* active image was installed at simply
   gone (`backup_len` is exactly that preserved length — see "True rollback"
   below). So by the time this bootloader ever observes `BC_F_PENDING`,
   `bc.staged_len` no longer names the active image — it names the
   candidate. If that candidate then fails verification here, there is no
   length recorded anywhere to re-check the (perfectly intact, untouched)
   active image against; guessing one (e.g. reusing the candidate's length)
   would violate "never run an unverified image" exactly as badly as
   skipping the check outright. So every failure in `try_commit_staging()`
   goes straight to `BOOT_STUCK` rather than a guess. In practice this
   should be rare — `ota_stage_finish()` already ran this same `fw_verify()`
   over the staged image before ever writing `BC_F_PENDING` — so a failure
   here means the staging region was corrupted or tampered with *after*
   that check succeeded (or, post-copy, that the flash copy itself went
   wrong). Since this phase, a re-verify failure at this exact point is no
   longer necessarily a dead end on a *later* boot: if this candidate is
   later force-committed again and crash-loops, `BOOT_ROLLBACK` (below) can
   still recover the pre-candidate image from `BACKUP`, if one was made.
3. **`BOOT_ROLLBACK`** (`try_rollback()`, true-rollback phase — see "True
   rollback" below for the full story): attempts a real restore of the
   previous known-good image from `BACKUP` instead of unconditionally
   halting.
4. **`BOOT_ACTIVE`** (whether the original verdict, or steps 2/3's
   fallthrough): if `bc_valid`,
   `fw_verify(PART_XIP(APP_OFF), bc.staged_len, FW_PUBKEY)` — `bc.staged_len`
   doubles as "the currently-active image's length" once a commit (or
   restore) has happened, since nothing after `ota_stage_finish()` /
   `try_rollback()` ever changes it without also updating `bc.staged_len`
   to match. Fails → `BOOT_STUCK`. Succeeds and still unconfirmed → bump
   `boot_attempts` and `bc_write()` **before** jumping (so a crash during
   *this* attempt is visible to the crash-loop check next boot) — this is
   also where step 2's deferred `BC_F_PENDING` clear actually hits flash.
   A `try_rollback()` success writes its own `BC_F_CONFIRMED` record
   directly (see below), so this branch's bump-and-write is skipped for a
   restored image — it already proved itself once and doesn't owe another
   retry budget. If `!bc_valid` (case 1's pre-OTA factory boot), jump
   unconditionally — see the callout below.
5. `BOOT_STUCK` → blink the onboard LED forever (`halt_blink()`, the only
   diagnostic output available with no USB/UART). Otherwise, `jump_to_app()`:
   disable interrupts → `SCB->VTOR = PART_XIP(APP_OFF)` → load the app's
   initial MSP and reset-handler address out of its vector table → `msr msp`
   → branch to the reset handler. Standard bare-metal Cortex-M handoff,
   commented step-by-step in the source.

### True rollback (this phase): the `BACKUP` region + restore

Earlier phases carved flash into a single active app slot with no fallback:
when an update was committed, the previous active image was simply
overwritten, so a crash-looping fresh commit had nothing left to revert to
and the bootloader halted (`BOOT_ROLLBACK` → `BOOT_STUCK`, unconditionally).
This phase adds a third, equal-sized flash-only slot — `BACKUP`
(`BACKUP_OFF/BACKUP_SIZE`, `src/partition_map.h`; re-carved from the old,
larger active/staging slots down to 576 KB each so all three now tile
exactly) — that the bootloader copies into/out of. **The app still links
ONCE, at the unchanged `APP_OFF`/XIP `0x10010000`** — this is a third
flash-only slot the bootloader manages, not A/B dual-linking.

**Only a `CONFIRMED` image is ever backed up.** `src/ota_stage.c`'s
`ota_stage_finish()` (already-landed, frozen by an earlier phase) is the
sole writer of `BC_F_BACKUP_VALID`/`backup_len`: when a new image is staged,
it reads the *current* boot-control record (describing whatever is active
right now) and, only if that outgoing image is itself `BC_F_CONFIRMED`,
records its length as `backup_len` and sets `BC_F_BACKUP_VALID` on the new
(`BC_F_PENDING`) record. An outgoing image that was never confirmed (e.g.
back-to-back OTAs with no successful boot in between) is never backed up —
backing up an unproven image would be no safer than the image replacing it.
`bc_has_valid_backup()` (`boot_decide.c`, pure/host-tested,
`test/boot_decide_test.c`) is the single shared range check
(`BC_F_BACKUP_VALID` set, `backup_len` in `(0, BACKUP_SIZE]`) both hardware
touch points below gate on, so they can never disagree about what counts as
"usable".

**Commit-time backup** (`try_commit_staging()`, step 2 above): after the new
staged image verifies but *before* the active slot is touched, if
`bc_has_valid_backup()` says there's something to preserve, the CURRENTLY
active image (named by `bc.backup_len`, not `bc.staged_len` — see the "no
fallback" callout above for why those differ once `BC_F_PENDING` is
observed) is copied `APP_OFF → BACKUP_OFF`, XIP-safe, and the copy is
re-verified. A failed backup copy aborts straight to `BOOT_STUCK` *without
ever touching active* — the commit itself only proceeds once the safety net
is confirmed in place.

**Rollback restore** (`try_rollback()`, step 3 above, replacing the old
unconditional-halt stub): reached only when the active image is unconfirmed
and has crash-looped past `MAX_BOOT_ATTEMPTS` (`boot_decide()`'s
`BOOT_ROLLBACK` verdict). If `bc_has_valid_backup()` and the backup image
itself passes `fw_verify(PART_XIP(BACKUP_OFF), bc.backup_len, FW_PUBKEY)`,
it is copied `BACKUP_OFF → APP_OFF` (XIP-safe, same generic
`flash_copy_region()` helper as every other flash-to-flash copy in this
file), the restored active slot is re-verified, and — only once that
succeeds end-to-end — a **fresh** boot-control record is written directly:
`staged_len = bc.backup_len`, `staged_sha512` recomputed from the restored
image's own (now-verified) trailer, flags set to exactly `BC_F_CONFIRMED`
(no `BC_F_PENDING`), `boot_attempts` zeroed, and `BC_F_BACKUP_VALID`/
`backup_len` left clear — `BACKUP` now holds the same bytes as `active`, not
a distinct fallback, until the next confirmed-outgoing OTA repopulates it.
The record is marked `CONFIRMED` immediately rather than through the normal
attempt-budget dance: it already proved itself once (that's *why* it was
eligible to be backed up), and a bootloader-verified restore from `BACKUP`
is at least as strong evidence of health as a fresh `ota_stage_finish()`
commit. **Never jump to an unverified image** on any path: no valid backup,
a backup that fails `fw_verify()`, or a restore copy that fails its
post-copy re-verify all fall straight through to `BOOT_STUCK`, exactly the
old stub's behaviour — a rollback attempt can never make things worse than
halting would have.

**Two residual gaps, both `HARDWARE-VALIDATION` items along with the copy
paths themselves (see "Status" below):**
- **The very first-ever OTA** (installed over the trailer-less factory
  image) has no `BC_F_CONFIRMED` outgoing image to back up, so it still has
  no fallback if it crash-loops — `try_rollback()` correctly reports
  `BOOT_STUCK` in this case, same as before this phase, rather than
  fabricating one. Closing this gap (e.g. requiring even the factory image
  to carry a trailer and seeding a `CONFIRMED` record at first boot) is out
  of scope here — see the next section's callout, which documents the same
  underlying limitation from the factory-boot side.
- **"Rollback of the rollback":** immediately after a restore, `BACKUP`
  holds the same image as `active` (not a distinct fallback — see above), so
  if *that* restored image somehow also crash-loops, there is nothing left
  to roll back to and the bootloader halts. This can only be closed by a
  *subsequent* confirmed OTA repopulating `BACKUP` with a new outgoing
  image; it is not a bug so much as the natural boundary of a single-slot
  backup (as opposed to a deeper history of backups, which flash budget
  doesn't accommodate here).

### The one documented exception to "never run an unverified image"

When `bc_valid` is false (no boot-control record has ever been written —
i.e. the factory/pre-OTA state, or theoretically both redundant sectors
corrupted simultaneously), there is no persisted image length to anchor a
trailer search against, so `fw_verify()` cannot be run at all. `main.c`
jumps to the active image unconditionally in this one case. This is the
**only** place in this bootloader that doesn't verify a signature before
jumping, and it's a direct, known consequence of the current
`light_switch.uf2` default build having **no fw_trailer at all** (only
images built via `tools/fw_sign.c` carry one) and no app-slot-length record
existing before the first OTA. It does not make anything *worse* than
today's flashing story (see below) — but it is a real, narrow gap that a
future revision should close, e.g. by requiring even the factory image to
carry a trailer and seeding a `BC_F_CONFIRMED` boot-control record at
first-boot instead of leaving it unwritten.

## Status: what is / isn't validated

**Host-tested and verified in this repo:**
- `boot_decide()` — exhaustively, `test/boot_decide_test.c` (see above).
- Everything it's built from (`bc_read`/`bc_write`, `fw_verify`) — already
  covered by `test/boot_control_test.c` and `test/fw_image_test.c` from
  earlier phases.
- The bootloader **builds cleanly as a real pico-sdk RP2040 ELF** (`make
  bootloader` above), with `-Wall -Wextra` and zero warnings, and links
  within its 56 KB budget (currently ~47 KB of the 56 KB flash region used,
  measured via `arm-none-eabi-size bootloader.elf`).

**NOT validated without real hardware — the first things to bring up
on-device, in this order:**
1. **The flash-copy path** (`flash_copy_region()` / the whole
   `try_commit_staging()` sequence, including the true-rollback phase's
   commit-time active→`BACKUP` backup copy and `try_rollback()`'s
   `BACKUP`→active restore copy — all three now share the one generic
   sector-copy helper): erasing/programming a flash region from a
   RAM-buffered chunk of another flash region, sector by sector, while
   running from flash itself. The XIP-unavailable-during-erase/program
   reasoning is architecturally correct per the RP2040 datasheet and matches
   how `src/boot_control.c`/`src/ota_stage.c` already do the equivalent
   under `flash_safe_execute()`, but it has never actually been exercised
   against a real W25Q flash chip by this bootloader — this now includes
   the backup and restore paths, not just the original staging→active
   commit path.
2. ~~The jump itself~~ **VALIDATED on hardware 2026-07-25** — after fixing a
   real bug found on the bench (see "Hardware bring-up findings" below): the
   original `jump_to_app()` entered the app with **PRIMASK set** (`cpsid i`
   with no matching `cpsie`), which a real reset never does. The app executed
   perfectly but every IRQ was masked: `sleep_ms()` blocked forever on a
   timer event that couldn't arrive and USB never enumerated — an apparent
   "hang" with no USB and no network. The jump now hands the app
   reset-faithful state (VTOR → app table, MSP → app stack, **PRIMASK
   clear**, then branch).
3. ~~The flash-copy paths~~ **ALL VALIDATED on hardware 2026-07-25** on the
   bench device, via a full OTA cycle and a deliberate crash-loop:
   - staging→active commit + active→BACKUP copy: exercised by a real OTA
     (`2.0.0+a9a853a` → `2.0.0+2a2f7c9`, manifest served by the dev HA at
     `/local/`, install triggered over the dev broker's `update/install`).
     The backup copy also proves the app-side `ota_mark_valid()` confirm
     one-shot ran (only a CONFIRMED outgoing image gets backed up).
   - BACKUP→active restore: a signed crash-loop image (blinks ~3 s, then
     `watchdog_reboot`, never confirms) was installed the same way; the
     bootloader exhausted `MAX_BOOT_ATTEMPTS` (3) and restored the previous
     image, which booted, reconnected, and re-polled the manifest.

## Re-running the bench OTA test

1. Dev stack up (`mapper/dev/docker-compose.yml`): mosquitto :1883, HA :8123.
   HA serves `ha-config/www/` at `http://<host>:8123/local/` (no auth) — no
   dedicated file server needed.
2. Device: `cfg fw_url=http://<host>:8123/local/light_switch.json` (CLI,
   persisted; reboot to apply — core 1 reads it from the launch snapshot).
3. Release per `../tools/README.md` (commit first — version must be clean),
   stage `light_switch_ota.signed.bin` as `www/light_switch.bin` +
   `fw_manifest.sh` output as `www/light_switch.json`.
4. Trigger: `mosquitto_pub -t 'lighting/update/check' -m CHECK` then
   `… 'lighting/update/install' -m INSTALL` (or the HA update entity's
   button). Success = retained `lighting/update/state` shows the new
   `installed_version`.
5. Rollback drill: sign a crash-loop image (`sleep ~3 s; watchdog_reboot`,
   no confirm), serve it as the update, install, and wait for
   `installed_version` to return to the pre-update version. RESTORE THE REAL
   MANIFEST AFTERWARD — never leave a crash manifest being served.

## Hardware bring-up findings (bench, 2026-07-25)

Two failed production-conversion attempts and a bench session produced three
hard-won rules, now encoded in `main.c`:

1. **`picotool load -f <file>` auto-reboots back into application mode after
   loading — it does NOT stay in BOOTSEL.** Flashing the bootloader alone
   this way boots it against a stale app slot. ALWAYS park the device first
   (`picotool reboot -f -u`), load BOTH images (`bootloader.uf2`, then the
   app at `-o 0x10010000`), then issue one `picotool reboot`.
2. **Vector-table sanity check** on the factory (`!bc_valid`) path: word 0
   must be an SP inside SRAM, word 1 a thumb reset vector inside the app
   slot. A stale/erased slot fails → BOOT_STUCK. This is what makes rule 1's
   failure mode recoverable instead of a dead hang.
3. **Watchdog-armed factory jump + BOOTSEL landing:** before the factory-path
   jump the bootloader arms an 8 s hardware watchdog and counts attempts in
   `watchdog_hw->scratch[2..3]` (magic `0x0B007D06`; scratch survives watchdog
   resets, and [4..7] are avoided — the SDK's `watchdog_reboot` vector
   protocol owns those). Three consecutive watchdog-reaped boots → BOOT_STUCK.
   Every BOOT_STUCK path blinks the LED 10× then calls `reset_usb_boot()` —
   the device lands in BOOTSEL, **remotely re-flashable**, instead of
   blinking forever behind a closed case. **Contract: the app must clear
   `scratch[2]` and disable the watchdog before it arms its own** — not
   necessarily as the very first thing in `main()`. The clear/disable must
   still land before anything slower than 8 s (bus discovery is), but
   nothing before it may be either: whatever runs between the bootloader's
   jump and the app's own watchdog arm is, for those 8 s, covered by the
   *bootloader's* still-ticking watchdog rather than by nothing at all. This
   matters because the hang classes this hardening exists to catch (see the
   two entries above) live in early init, not in the main loop — a
   clear/disable placed before that code ran would defeat the very coverage
   it's meant to provide. `../src/main.c` hands off immediately before its
   own `watchdog_enable(5000, 1)` call — after USB/LED init and the core 1
   launch, before `fs_init()`/discovery — unconditionally; it is a no-op in
   the standalone boot path (the watchdog isn't enabled yet there either
   way).

## The relocated app build: `light_switch_ota` (opt-in, `../CMakeLists.txt`)

Everything above assumes an app image already sitting at `APP_OFF` with its
vector table at `PART_XIP(APP_OFF)` (0x10010000) and no `.boot2` in front of
it — but the *default* `light_switch` target (`make -C ../build
light_switch`) is deliberately still linked at the standard standalone origin
(0x10000000, with its own `.boot2`), per the hard rule that this phase never
changes the default app's link address. So until now there was literally no
build in this repo producing an image this bootloader could jump to.

`../CMakeLists.txt` now has an **opt-in** second target that closes that gap:

```sh
cmake -S .. -B ../build_ota -DOTA_APP_LAYOUT=ON
make -C ../build_ota light_switch_ota
```

`light_switch_ota` compiles the exact same application sources as
`light_switch` (duplicated into a second `add_executable()`, not shared, so
the option's presence/absence can never perturb the default target — see the
CMakeLists.txt comment block above it) but links them with
`bootloader/app_ota.ld` instead of pico-sdk's stock linker script:

- `FLASH` region is `ORIGIN = 0x10010000, LENGTH = 0x90000` — i.e. exactly
  `PART_XIP(APP_OFF)`/`APP_SIZE` (576 KB, true-rollback phase re-carve) —
  instead of the stock `0x10000000`/2 MB, so the whole image (vector table
  first) is laid out starting at the address this bootloader's
  `jump_to_app()` already hands off to.
- **No `.boot2`.** A normal Pico app's `.boot2` exists so the RP2040 boot ROM
  can configure XIP on a cold reset — but `light_switch_ota` is never
  reached that way; *this bootloader* is the thing the boot ROM boots
  (`bootloader.elf`, with its own real `.boot2` at flash offset 0), and by
  the time `jump_to_app()` branches into the app the QSPI/SSI controller is
  already in XIP mode (that peripheral state persists across the plain
  branch — there's no chip reset in between). `app_ota.ld` drops the
  `.boot2` output section entirely, and `CMakeLists.txt` points
  `PICO_TARGET_BOOT_STAGE2` at an empty `bs2_none_library` so no object
  contributes a `.boot2` input section in the first place. Net effect:
  `light_switch_ota`'s vector table is the very first thing at `APP_OFF`,
  which is exactly what `jump_to_app()` (`main.c`, unchanged) already
  assumes — word 0 = initial MSP, word 1 = reset handler, read straight from
  `PART_XIP(APP_OFF)` with no offset.
- This is the same relocation idea used by `picowota`/`pico-flashloader`:
  app linked at a non-zero flash offset, no boot2, the *bootloader* owns the
  one-time flash/XIP bring-up.

**Verified without hardware** (configure + build both targets in sibling
build dirs and inspect the outputs — see the CMakeLists.txt comment block
and `app_ota.ld`'s header for the mechanics):
- `make -C ../build light_switch` still links at 0x10000000 with a 256-byte
  `.boot2` first and `Entry point 0x100001e9` — byte-for-byte the same as
  before this change existed.
- `light_switch_ota.elf`'s map (`Memory Configuration`) shows
  `FLASH  0x10010000  0x000e0000`, `.text` starts at `0x10010000` (no
  `.boot2` section anywhere in the map), and the first 8 bytes at that
  address disassemble as a plausible ARMv6-M vector table (initial
  SP/`Reset_Handler` pointers into `RAM`/`.text` respectively) — a real
  device would land in `Reset_Handler` exactly like a normal Pico app does,
  just starting from a jump instead of a cold reset.
- `light_switch_ota.bin` is ~337 KB, comfortably inside the 576 KB
  `APP_SIZE` budget (`app_ota.ld`'s `FLASH` `LENGTH` — a build that ever
  grows past it fails the link with a "region `FLASH' overflowed" error,
  same automatic guard as `memmap_bootloader.ld`), leaving `STAGING_OFF`
  (0xA0000) and `FS_OFF`/littlefs (0x1C0000) both completely unencroached.
- What is **still not validated without real hardware**: actually booting
  this image via `jump_to_app()` on a device (see the numbered list below —
  this build only removes the "there is no such image to test with" gap,
  it doesn't itself prove the jump works).

### How the published, signed OTA artifact relates to this target

`tools/fw_sign.c`/`tools/README.md`'s release flow ("build -> sign ->
manifest -> publish") previously named plain `light_switch` as step 1's
input, which was only ever a placeholder — a `light_switch.bin` linked at
0x10000000 would fail `fw_verify()`'s signature check no differently than
any other input, but even if it were signed, copying it to `APP_OFF` and
jumping to `PART_XIP(APP_OFF)` would branch into the *middle* of that image
(its vector table is 0x10000 bytes further down than the bootloader expects,
since the whole image would've been linked assuming it starts at
0x10000000). **`light_switch_ota` is the correct input to that pipeline now**:

```
light_switch_ota.elf  --objcopy--> light_switch_ota.bin  (raw image, vector table at offset 0)
                                          |
                                    tools/fw_sign  (appends struct fw_trailer: img_len, sha512, ed25519 sig)
                                          v
                              light_switch_ota.signed.bin  <-- THIS is what gets published (the .bin
                                                                 the manifest's `url` points at)
```

On the device side nothing about this changes: `src/net/fwupdate.c` downloads
the manifest, `src/net/ota.c` streams `light_switch_ota.signed.bin` into
`STAGING_OFF` unmodified, `ota_stage_finish()` writes `BC_F_PENDING` +
`staged_len`/`staged_sha512`, and on the next boot **this** bootloader's
`try_commit_staging()` (`main.c`) `fw_verify()`s it, copies it into
`APP_OFF`, re-verifies, and `jump_to_app()`s straight into the vector table
`app_ota.ld` placed at `PART_XIP(APP_OFF)` — no part of that flow
needed to change for this phase; it was simply missing a real producer of
the image shape it already assumed. `tools/README.md`'s release flow has
been updated to build `light_switch_ota` (not `light_switch`) as its step 1.

## Files

- `boot_decide.h` / `boot_decide.c` — the pure decision function (see above).
- `main.c` — the bootloader executable's `main()` (hardware layer).
- `CMakeLists.txt` — standalone pico-sdk project, board = `wiznet_w5100s_evb_pico`.
- `memmap_bootloader.ld` — custom linker script constraining the bootloader's
  own code+data to `0x0..0xE000` (see "Flash layout" above for why this is a
  linker script rather than a `PICO_FLASH_SIZE_BYTES` CMake variable).
- `app_ota.ld` — custom linker script for the opt-in `light_switch_ota`
  CMake target (`../CMakeLists.txt`, `-DOTA_APP_LAYOUT=ON`): relinks the same
  app sources to `APP_OFF`/`APP_SIZE` with no `.boot2`, so the bootloader can
  jump straight to their vector table (see "The relocated app build" above).
- Links `../src/boot_control.c`, `../src/fw_image.c`,
  `../lib/monocypher/monocypher{,-ed25519}.c` directly (source-shared with
  the main firmware, not a separate copy) plus `../src/partition_map.h` for
  the offsets.
