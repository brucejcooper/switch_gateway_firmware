/*
 * main.c — switch gateway firmware, v2.
 *
 * Retains from original:
 *   - pico_enable_filesystem (pico-vfs / littlefs via fopen())
 *   - modbus_init() + modbus_poll() ISR-driven poll loop
 *   - buttongrid_init() + PIO button scanner
 *   - alarm_pool_init_default()
 *   - watchdog
 *   - LED heartbeat
 *
 * Replaces:
 *   - lua_init() / lua_poll()  → serial_protocol_init() / serial_poll()
 *   - Lua coroutine scheduler  → button_fsm_tick() + callback model
 *   - DALI coroutine bindings  → lighting_actions + dirty_poll_tick()
 */

#include "modbus.h"
#include "button_actions.h"
#include "button_fsm.h"
#include "state_cache.h"
#include "config.h"
#include "mapping.h"
#include "lighting_actions.h"
#include "dirty_poll.h"
#include "serial_protocol.h"
#include "dali_provision.h"
#include "bus_discovery.h"
#include "core1.h"
#include "intercore.h"
#include "log.h"
#include <string.h>

#include <hardware/timer.h>
#include <hardware/watchdog.h>
#include <pico/stdio.h>
#include <pico/stdlib.h>
#include <pico/time.h>
#include <filesystem/vfs.h>

static int pico_led_init(void) {
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}

/* Blocking relay read using the sync_read pattern from dirty_poll.c */
typedef struct { volatile bool done; volatile bool ok;
                 uint8_t *bits; } relay_sync_ctx_t;

static void relay_read_cb(int res, uint8_t *buf, void *ctx) {
    relay_sync_ctx_t *c = (relay_sync_ctx_t *)ctx;
    if (res >= 7) { /* addr+FC+bytecount+4bytes+CRC... actually buf has raw frame */
        /* buf[2] = byte count, buf[3..] = coil bytes, each bit = one coil */
        uint8_t byte_count = buf[2];
        for (int i = 0; i < byte_count && i < 4; i++) {
            c->bits[i] = buf[3 + i];
        }
        c->ok = true;
    }
    c->done = true;
}

/* Grace window end: core 0 feeds the watchdog unconditionally until here. */
static absolute_time_t s_core1_grace_until;

/* Feed the hardware watchdog only while core 1's heartbeat is fresh (or during
 * the post-launch grace window). A network outage keeps core 1 looping and
 * beating, so it never reboots; only a genuine core-1 wedge (stuck SPI, infinite
 * loop) stops the beat and lets the 5 s watchdog fire. */
static void wd_feed(void) {
    if (!time_reached(s_core1_grace_until) ||
        (to_ms_since_boot(get_absolute_time()) - core1_heartbeat_ms()) < CORE1_STALE_MS) {
        watchdog_update();
    }
}

int main(void) {
    stdio_init_all();
    fs_init();
    alarm_pool_init_default();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    busy_wait_ms(2500); /* USB enumeration settling time */

    if (watchdog_enable_caused_reboot()) {
        LOG_INFO("boot", "Rebooted by Watchdog");
    } else {
        LOG_INFO("boot", "Clean boot");
    }

    /* Load persistent config and mappings */
    config_load();
    mapping_load();
    state_cache_init();
    LOG_INFO("boot", "config loaded relay_slaves=%u dali_slaves=%u",
             g_config.relay_slave_count, g_config.dali_slave_count);

    /* Modbus must init first (may be called during scan) */
    modbus_init();
    LOG_INFO("boot", "modbus init baud=%lu", (unsigned long)modbus_get_baud());

    /* Bus discovery: probe each configured slave at the canonical baud,
     * recovering factory-default 9600-baud slaves to 19200 if needed.
     * Runs before watchdog_enable and before any DALI/relay scan. */
    LOG_INFO("boot", "bus discovery: relay_count=%u dali_count=%u",
             g_config.relay_slave_count, g_config.dali_slave_count);
    for (int i = 0; i < g_config.relay_slave_count; i++) {
        bus_discover_slave(g_config.relay_slave_ids[i], SLAVE_KIND_RELAY);
    }
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        bus_discover_slave(g_config.dali_slave_ids[i], SLAVE_KIND_DALI);
    }

    /* Dirty poll init before full scan */
    dirty_poll_init();
    dali_provision_init();

    /* Full scan of each DALI slave — blocking, establishes ground truth */
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        LOG_INFO("boot", "scanning slave %u", g_config.dali_slave_ids[i]);
        dirty_poll_full_scan(g_config.dali_slave_ids[i]);
    }

    /* Read all relay coils to sync cache */
    if (g_config.relay_slave_count > 0) {
        static relay_sync_ctx_t ctx;
        static uint8_t coil_bits[4] = {0};
        ctx = (relay_sync_ctx_t){ false, false, coil_bits };
        modbus_get_coils(g_config.relay_slave_ids[0], 0, 32, relay_read_cb, &ctx);
        absolute_time_t dl = make_timeout_time_ms(1000);
        while (!ctx.done && absolute_time_diff_us(get_absolute_time(), dl) > 0) {
            modbus_poll();
        }
        if (ctx.ok) {
            for (int ch = 0; ch < 32; ch++) {
                bool on = (coil_bits[ch / 8] >> (ch % 8)) & 1;
                state_cache_set_relay((uint8_t)ch, on);
                lighting_emit_relay_state((uint8_t)ch, on);
            }
            LOG_INFO("boot", "relay sync: 32 channels read");
        } else {
            LOG_WARN("boot", "relay sync: read failed");
        }
    }

    /* Button grid — goes last as in original, injects events once loop starts */
    button_fsm_init();
    buttongrid_init();

    serial_protocol_init();   /* inits the inter-core queues */

    /* Snapshot the slave lists for core 1's HA discovery, then launch core 1
     * (the network bridge). core1_launch blocks until core 1 has registered as
     * the flash lockout victim. */
    ic_snapshot_t snap;
    memset(&snap, 0, sizeof snap);
    snap.relay_count = g_config.relay_slave_count;
    memcpy(snap.relay_slaves, g_config.relay_slave_ids, g_config.relay_slave_count);
    snap.dali_count = g_config.dali_slave_count;
    memcpy(snap.dali_slaves, g_config.dali_slave_ids, g_config.dali_slave_count);
    intercore_set_snapshot(&snap);

    core1_launch();

    watchdog_enable(5000, 1);
    s_core1_grace_until = make_timeout_time_ms(CORE1_GRACE_MS);

    /* Announce over USB */
    sleep_ms(100);
    serial_tx_hello();

    LOG_INFO("boot", "main loop entering, watchdog=5000 ms, core1 grace=%u ms",
             (unsigned)CORE1_GRACE_MS);

    /* ── Main loop ────────────────────────────────────────────────────── */
    while (true) {
        modbus_poll();
        wd_feed();

        button_fsm_tick();
        dirty_poll_tick();
        dali_provision_tick();
        serial_poll();

        wd_feed();
    }

    panic_unsupported();
}
