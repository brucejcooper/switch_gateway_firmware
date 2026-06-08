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
#include "net/mqtt_app.h"
#include "net/ha_bridge.h"
#include "log.h"

#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/time.h"
#include <string.h>

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

    /* Bring up the W5100S and the MQTT connection manager from the config
     * snapshot core 0 filled before launch. Both are non-blocking SMs. */
    const ic_snapshot_t *sn = intercore_snapshot();
    netif_config_t nc;
    memset(&nc, 0, sizeof nc);
    nc.dhcp = sn->net_dhcp;
    memcpy(nc.ip, sn->net_ip, 4);
    memcpy(nc.sn, sn->net_mask, 4);
    memcpy(nc.gw, sn->net_gw, 4);
    memcpy(nc.dns, sn->net_dns, 4);
    netif_init(&nc);

    mqtt_config_t mc;
    memset(&mc, 0, sizeof mc);
    memcpy(mc.broker_ip, sn->mqtt_ip, 4);
    mc.broker_port = sn->mqtt_port;
    memcpy(mc.user, sn->mqtt_user, sizeof mc.user);
    memcpy(mc.pass, sn->mqtt_pass, sizeof mc.pass);
    memcpy(mc.prefix, sn->mqtt_prefix, sizeof mc.prefix);
    mc.have_creds = sn->mqtt_have_creds;
    mqtt_app_init(&mc);

    ha_bridge_init();

    uint32_t last_log_ms = 0;
    while (true) {
        s_heartbeat_ms = to_ms_since_boot(get_absolute_time());

        netif_tick();
        mqtt_app_tick();
        ha_bridge_tick();   /* discovery + availability sweep + event->MQTT pump */

        if (s_heartbeat_ms - last_log_ms >= 5000u) {
            last_log_ms = s_heartbeat_ms;
            LOG_INFO("core1", "alive: link=%d ready=%d mqtt=%d evt_drops=%lu",
                     (int)netif_link_up(), (int)netif_ready(),
                     (int)mqtt_app_connected(),
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
