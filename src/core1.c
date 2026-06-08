/*
 * core1.c — the second core: network bridge entry point + liveness.
 *
 * For now this is a stub: it registers as the flash lockout victim, beats its
 * heartbeat, and drains (discards) the core0->core1 event queue so it does not
 * back up. The W5100S socket state machine, coreMQTT, and the HA translation
 * layer replace the discard over the following commits.
 *
 * Core 1 owns SPI0 (the W5100S) exclusively; core 0 owns the RS-485 UART/PIO and
 * USB CDC. The only shared state is the two inter-core queues, the heartbeat,
 * and the flash (coordinated via flash_safe_execute / multicore lockout).
 */

#include "core1.h"
#include "intercore.h"
#include "net/netif.h"
#include "log.h"

#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/time.h"

/* Explicit 8 KB core-1 stack (deliberately sized for the upcoming network stack
 * depth rather than taken on faith under the tighter RP2040 SRAM budget). */
#define CORE1_STACK_WORDS 2048u
static uint32_t s_core1_stack[CORE1_STACK_WORDS];

static volatile uint32_t s_heartbeat_ms = 0;
static volatile bool     s_ready        = false;

uint32_t core1_heartbeat_ms(void) {
    return s_heartbeat_ms;
}

static void core1_entry(void) {
    /* Register core 1 as the flash lockout victim. This is the ONLY thing the
     * application must do for multicore flash safety: pico-vfs already wraps its
     * erase/program in flash_safe_execute() (blockdevice/flash.c), which parks
     * this core while core 0 writes. Core 1 never writes flash itself. */
    bool ok = flash_safe_execute_core_init();
    s_heartbeat_ms = to_ms_since_boot(get_absolute_time());
    s_ready = true;
    LOG_INFO("core1", "flash lockout victim init %s", ok ? "ok" : "FAILED");

    /* Bring up the W5100S (DHCP by default; the /config.json net settings are
     * wired in later). netif_tick() pumps the bring-up/DHCP/link/recovery SM. */
    netif_init(NULL);

    uint32_t discarded = 0, last_log_ms = 0;
    ic_event_t ev;
    while (true) {
        s_heartbeat_ms = to_ms_since_boot(get_absolute_time());

        netif_tick();

        /* Drain and discard for now (MQTT publish replaces this later). */
        while (intercore_pop_event(&ev)) discarded++;

        if (s_heartbeat_ms - last_log_ms >= 5000u) {
            last_log_ms = s_heartbeat_ms;
            LOG_INFO("core1", "alive: ready=%d link=%d events_drained=%lu drops=%lu",
                     (int)netif_ready(), (int)netif_link_up(),
                     (unsigned long)discarded,
                     (unsigned long)intercore_event_drops());
        }
    }
}

void core1_launch(void) {
    multicore_launch_core1_with_stack(core1_entry, s_core1_stack,
                                      sizeof(s_core1_stack));

    /* Wait (bounded) for core 1 to finish flash_safe_execute_core_init() before
     * core 0 proceeds to anything that might write flash. */
    absolute_time_t deadline = make_timeout_time_ms(1000);
    while (!s_ready && absolute_time_diff_us(get_absolute_time(), deadline) > 0)
        tight_loop_contents();

    LOG_INFO("core1", "launched (ready=%d)", (int)s_ready);
}
