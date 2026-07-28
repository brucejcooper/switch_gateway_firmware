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
#include "dali_queue.h"
#include "dq_soak.h"
#include "dirty_poll.h"
#include "serial_protocol.h"
#include "dali_provision.h"
#include "bus_discovery.h"
#include "core1.h"
#include "intercore.h"
#include "log.h"
#include "cdc_out.h"
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
void wd_feed(void) {
    if (!time_reached(s_core1_grace_until) ||
        (to_ms_since_boot(get_absolute_time()) - core1_heartbeat_ms()) < CORE1_STALE_MS) {
        watchdog_update();
    }
}

int main(void) {
    /* If the OTA bootloader launched us with its watchdog-armed jump
     * (bootloader/main.c "HARDENING #2"), prove we're alive: clear the
     * attempt marker and disable its 8 s watchdog before anything slow
     * (discovery takes far longer than 8 s). Harmless in the standalone
     * build — the watchdog isn't enabled yet and scratch[2] is unused.
     * Our own watchdog is armed at main-loop entry as before. */
    watchdog_hw->scratch[2] = 0;
    hw_clear_bits(&watchdog_hw->ctrl, WATCHDOG_CTRL_ENABLE_BITS);

    cdc_out_init();     /* before any LOG_* / core 1 launch: arms the output ring */
    stdio_init_all();
    alarm_pool_init_default();

    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);

    busy_wait_ms(2500); /* USB enumeration settling time */

    /* Launch core 1 BEFORE fs_init(): core 1 registers as the flash lockout
     * victim (flash_safe_execute_core_init), which the first-boot littlefs
     * format requires — otherwise flash_safe_execute() returns NOT_PERMITTED and
     * the filesystem never mounts (so config/mappings can't persist). core 1
     * then parks waiting for the config snapshot before bringing up the network. */
    intercore_init();
    core1_launch();

    if (!fs_init())
        LOG_ERR("boot", "fs_init failed — persistence unavailable");

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
    dq_init();   /* per-slave gateway command-queue driver state */

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
                bool on = lighting_relay_coil_on(coil_bits, 4, (uint8_t)ch);
                state_cache_set_relay((uint8_t)ch, on);
                lighting_emit_relay_state((uint8_t)ch, on);
            }
            LOG_INFO("boot", "relay sync: 32 channels read (byteswap=%u)",
                     g_config.relay_byteswap ? 1u : 0u);
        } else {
            LOG_WARN("boot", "relay sync: read failed");
        }
    }

    /* Button grid — goes last as in original, injects events once loop starts */
    button_fsm_init();
    buttongrid_init();

    serial_protocol_init();

    /* Fill the config snapshot (slave lists + broker/net settings) for core 1,
     * then release it to bring up the network. */
    ic_snapshot_t snap;
    memset(&snap, 0, sizeof snap);
    snap.relay_count = g_config.relay_slave_count;
    memcpy(snap.relay_slaves, g_config.relay_slave_ids, g_config.relay_slave_count);
    snap.dali_count = g_config.dali_slave_count;
    memcpy(snap.dali_slaves, g_config.dali_slave_ids, g_config.dali_slave_count);
    /* MQTT: parse the broker host (dotted-quad) to an IP; 0.0.0.0 if unset. */
    {
        unsigned a0, a1, a2, a3;
        if (sscanf(g_config.mqtt_host, "%u.%u.%u.%u", &a0, &a1, &a2, &a3) == 4) {
            snap.mqtt_ip[0] = (uint8_t)a0; snap.mqtt_ip[1] = (uint8_t)a1;
            snap.mqtt_ip[2] = (uint8_t)a2; snap.mqtt_ip[3] = (uint8_t)a3;
        }
    }
    snap.mqtt_port = g_config.mqtt_port;
    memcpy(snap.mqtt_user,   g_config.mqtt_user,   sizeof snap.mqtt_user);
    memcpy(snap.mqtt_pass,   g_config.mqtt_pass,   sizeof snap.mqtt_pass);
    memcpy(snap.mqtt_prefix, g_config.mqtt_prefix, sizeof snap.mqtt_prefix);
    snap.mqtt_have_creds = g_config.mqtt_user[0] != '\0';
    snap.net_dhcp = g_config.net_dhcp != 0;
    memcpy(snap.net_ip,   g_config.net_ip,   4);
    memcpy(snap.net_mask, g_config.net_mask, 4);
    memcpy(snap.net_gw,   g_config.net_gw,   4);
    memcpy(snap.net_dns,  g_config.net_dns,  4);
    memcpy(snap.net_hostname, g_config.net_hostname, sizeof snap.net_hostname);
    memcpy(snap.web_user, g_config.web_user, sizeof snap.web_user);
    memcpy(snap.web_pass, g_config.web_pass, sizeof snap.web_pass);
    memcpy(snap.fw_url,   g_config.fw_url,   sizeof snap.fw_url);
    intercore_set_snapshot(&snap);
    core1_config_ready();   /* core 1 may now bring up netif + MQTT */

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

        lighting_any_on_probe_tick();   /* submit probe queries into the gateway queue */
        lighting_cfg_query_tick();      /* submit config queries into the gateway queue */
        dq_tick();                      /* drive the gateway command queue (FC23) */
        dq_soak_tick();                 /* bench: queue soak probe (idle unless started) */
        button_fsm_tick();
        dirty_poll_tick();
        dali_provision_tick();
        serial_poll();

        wd_feed();
    }

    panic_unsupported();
}
