/*
 * bootloader/main.c — the OTA bootloader's hardware layer.
 *
 * This file is the thin, heavily-commented, NOT-host-tested half of the
 * bootloader: it reads the persisted boot_control record, calls the PURE
 * boot_decide() (boot_decide.h/.c — exhaustively host-tested under
 * test/boot_decide_test.c) to get a verdict, hands the COMMIT_STAGING/
 * ROLLBACK orchestration off to commit.c's boot_commit_run() (factored out
 * 2026-08-17 so that orchestration can also be replayed as a SEQUENCE across
 * simulated boots by test/boot_commit_test.c — see commit.h), and does the
 * two things left that genuinely require hardware and cannot be exercised on
 * a host either way: the !bc_valid factory-path hardening (vector-table
 * sanity check + watchdog-armed jump) and the bare-metal jump into the app.
 * See bootloader/README.md for the full boot-flow writeup and — this is
 * important — an explicit list of what here is UNVALIDATED WITHOUT HARDWARE.
 *
 * Runs with ONLY ONE core active: core 1 is never launched (no
 * multicore_launch_core1() call anywhere in this program), so there is no
 * flash-XIP lockout victim to coordinate with and flash_range_erase/program
 * (commit.c) are called directly (no flash_safe_execute — see CLAUDE.md's
 * flash-write model note and the task brief this was built from). No USB,
 * no network: pico_enable_stdio_usb/uart are both off (see CMakeLists.txt)
 * and nothing here calls into pico_stdio at all.
 */

#include <stdint.h>
#include <stdbool.h>

#include "partition_map.h"
#include "commit.h"

#include "hardware/gpio.h"
#include "hardware/structs/scb.h"
#include "hardware/watchdog.h"
#include "hardware/structs/watchdog.h"
#include "pico/bootrom.h"

/* A boot is allowed this many *unconfirmed* attempts before the bootloader
 * gives up on it as a crash loop (see boot_decide.h rules 4/5 — compared
 * with `>=`, so this constant names the exact attempt count: 3 really does
 * mean the candidate gets three real boots before a fourth rolls it back,
 * not four). Matches boot_attempts semantics already established by
 * src/ota_stage.c (ota_stage_confirm() zeroes it on success). Three retries
 * is generous enough to ride out one bad boot (e.g. a transient
 * bus-discovery hiccup) without giving a genuinely broken image dozens of
 * chances to keep re-flashing/rebooting. */
#define MAX_BOOT_ATTEMPTS 3u

/* Onboard LED (GPIO 25 on this board, see boards/wiznet_w5100s_evb_pico.h) —
 * the only "output device" this bootloader is allowed: no USB, no UART, no
 * network. Used solely for the BOOT_STUCK halt signal. */
#define HALT_LED_PIN PICO_DEFAULT_LED_PIN

/* Crude busy-wait, calibrated for "roughly human-visible", not for accuracy:
 * this only ever runs on the terminal BOOT_STUCK path, where correctness no
 * longer matters and pulling in pico_time's alarm-pool machinery for a
 * blink loop would be a needless dependency for a bootloader. */
static void busy_delay(void) {
    for (volatile uint32_t i = 0; i < 800000u; i++) {
        __asm volatile("");
    }
}

/* ── BOOT_STUCK terminal state ────────────────────────────────────────────
 * Nothing was safely bootable (see boot_decide.h's BOOT_STUCK doc). Blink
 * the onboard LED a few times as the visible diagnostic, then drop into the
 * ROM's USB (BOOTSEL) mode so the device stays REMOTELY re-flashable — a
 * blink-forever halt needs a physical BOOTSEL press to escape, which cost a
 * case-opening the night of 2026-07-25. reset_usb_boot() never returns; the
 * blink loop after it is unreachable belt-and-braces should it ever. */
static void halt_blink(void) {
    gpio_init(HALT_LED_PIN);
    gpio_set_dir(HALT_LED_PIN, GPIO_OUT);
    for (int i = 0; i < 10; i++) {
        gpio_put(HALT_LED_PIN, 1);
        busy_delay();
        gpio_put(HALT_LED_PIN, 0);
        busy_delay();
    }
    reset_usb_boot(0, 0);
    for (;;) { }
}

/* ── The bare-metal handoff to the app ────────────────────────────────────
 * Standard Cortex-M0+ bootloader-to-application jump. Never returns. */
static void jump_to_app(void) __attribute__((noreturn));
static void jump_to_app(void) {
    /* 1. Disable interrupts. The bootloader never enables any (no NVIC_
     *    EnableIRQ anywhere in this program), so this is belt-and-braces
     *    rather than undoing something specific — but it makes the
     *    invariant explicit right before we hand control to code that will
     *    set up its own vector table and interrupt configuration from
     *    scratch. */
    __asm volatile ("cpsid i" ::: "memory");

    /* 2. Point the Vector Table Offset Register at the app's own vector
     *    table. From this instant on, any exception (including the SysTick/
     *    fault handlers the app's own runtime_init() will shortly
     *    configure) is dispatched through the APP's table, not the
     *    bootloader's. */
    scb_hw->vtor = (uint32_t)PART_XIP(APP_OFF);

    /* 3. Read the app's initial main-stack-pointer and entry point out of
     *    its vector table (standard ARMv6-M layout: word 0 = initial MSP,
     *    word 1 = Reset_Handler address, already Thumb-bit-encoded by the
     *    app's own linker/crt0). */
    const uint32_t *app_vectors = (const uint32_t *)PART_XIP(APP_OFF);
    uint32_t app_sp    = app_vectors[0];
    uint32_t app_reset = app_vectors[1];

    /* 4. Load the app's initial stack pointer into MSP. The bootloader's
     *    own C stack is abandoned right here — there is no returning from
     *    this function, by design (noreturn) and by construction (the app
     *    now owns the only stack pointer this core has). */
    __asm volatile ("msr msp, %0" :: "r" (app_sp) : "memory");

    /* 5. Hand the app RESET-FAITHFUL exception state: PRIMASK clear. Step 1
     *    set PRIMASK (cpsid i), and the app's crt0/runtime never clears it —
     *    a real reset doesn't need to. Found the hard way (bench,
     *    2026-07-25): entering the app with PRIMASK set masks every IRQ, so
     *    sleep_ms() waits forever on a timer event that can't arrive and USB
     *    never enumerates — the app "hangs" while executing perfectly.
     *    Enabling here is safe: this bootloader never enables any interrupt
     *    source, so nothing can actually fire before the app configures its
     *    own handlers via its own vector table (VTOR already points there). */
    __asm volatile ("cpsie i" ::: "memory");

    /* 6. Branch to the app's reset handler. This never returns to the
     *    bootloader. */
    ((void (*)(void))app_reset)();

    /* Unreachable. */
    for (;;) { }
}

int main(void) {
    /* Single-core, no interrupts enabled anywhere in this program — see the
     * file header. Nothing to initialise before reading boot_control. */

    /* boot_decide() + the COMMIT_STAGING/ROLLBACK dispatch + the shared
     * "about to boot ACTIVE" bc_write() step all live in commit.c's
     * boot_commit_run() now (factored out so test/boot_commit_test.c can
     * replay the same sequence host-side — see commit.h). bc_valid is still
     * needed here for the !bc_valid hardware-only hardening below. */
    bool bc_valid;
    boot_action_t action = boot_commit_run(MAX_BOOT_ATTEMPTS, &bc_valid);

    /* action == BOOT_ACTIVE && !bc_valid: no boot_control record has ever
     * been written (both sectors erased/corrupt) — the pre-OTA factory
     * state boot_decide()'s rule 1 exists for. There is no persisted length
     * to anchor a trailer search against, so this is the ONE path in this
     * bootloader that boots without fw_verify()ing anything first. This is
     * an explicit, narrow, documented exception to "never run an
     * unverified image" — see the prominent callout in README.md, which
     * also covers why it doesn't (yet) get worse than the status quo. Two
     * hardenings below, both scoped to this one factory path:
     *
     * HARDENING (2026-07-25): even without a length to verify, the app
     * slot's vector table can be sanity-checked for free: a real app's
     * word 0 is an initial SP inside SRAM and word 1 a thumb reset vector
     * inside the app slot. A stale/half-converted slot (arbitrary code
     * bytes, 0xFF erased flash) essentially never satisfies both — and
     * jumping into it hangs the chip with USB dead, needing a physical
     * BOOTSEL press to recover (learned the hard way the night this was
     * written). Failing here routes to BOOT_STUCK, which now lands in
     * BOOTSEL: remotely re-flashable.
     *
     * HARDENING #2 (2026-07-25): a sane-LOOKING app can still hang in early
     * init (observed on the bench: the relocated app passes the vector check,
     * the jump executes, then nothing — no USB, unrecoverable without the
     * physical BOOTSEL button). Close that gap too: arm the hardware
     * watchdog before jumping and count attempts in a watchdog SCRATCH
     * register (they survive watchdog resets). A hung app → watchdog reset
     * ≤8 s → back here with the marker set → after 3 strikes, BOOT_STUCK →
     * BOOTSEL, remotely re-flashable. Gated on the vector check above still
     * holding (action still BOOT_ACTIVE) — no point arming a retry budget
     * for an app slot hardening #1 already condemned. A healthy app must
     * clear scratch[2] and disable this watchdog before arming its own —
     * not necessarily as the very first thing in main(), just before
     * anything slower than 8 s (see README.md's "Contract" and src/main.c,
     * which hands off immediately before its own watchdog_enable() call so
     * the bootloader's watchdog stays live across USB/core-1 bring-up
     * instead of being disarmed before anything replaces it). scratch[2]/[3]
     * chosen to stay clear of the SDK's watchdog_reboot vector protocol in
     * [4..7]. */
    #define BL_WD_MAGIC     0x0B007D06u
    #define BL_WD_TIMEOUT_MS 8000u
    #define BL_WD_MAX_TRIES  3u
    if (action == BOOT_ACTIVE && !bc_valid) {
        const volatile uint32_t *v = (const volatile uint32_t *)PART_XIP(APP_OFF);
        uint32_t sp = v[0], pc = v[1];
        bool sp_ok = sp >= 0x20000000u && sp <= 0x20042000u;   /* RP2040 SRAM */
        bool pc_ok = (pc & 1u) &&
                     (pc & ~1u) >= PART_XIP(APP_OFF) &&
                     (pc & ~1u) <  PART_XIP(APP_OFF) + APP_SIZE;
        if (!sp_ok || !pc_ok) action = BOOT_STUCK;

        if (action == BOOT_ACTIVE) {
            uint32_t attempts = 0;
            if (watchdog_caused_reboot() &&
                watchdog_hw->scratch[2] == BL_WD_MAGIC)
                attempts = watchdog_hw->scratch[3];
            if (attempts >= BL_WD_MAX_TRIES) {
                watchdog_hw->scratch[2] = 0;
                action = BOOT_STUCK;
            } else {
                watchdog_hw->scratch[2] = BL_WD_MAGIC;
                watchdog_hw->scratch[3] = attempts + 1;
                watchdog_enable(BL_WD_TIMEOUT_MS, true);
            }
        }
    }

    if (action == BOOT_STUCK) {
        halt_blink();   /* never returns */
    }

    jump_to_app();      /* never returns */
}
