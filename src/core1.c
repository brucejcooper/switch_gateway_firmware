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
#include "net/web_state.h"
#include "net/http_server.h"
#include "net/ping.h"
#include "net/ota.h"
#include "net/fwupdate.h"
#include "log.h"

#include "pico/multicore.h"
#include "pico/flash.h"
#include "pico/time.h"
#include <string.h>

/* Explicit 8 KB core-1 stack (deliberately sized for the upcoming network stack
 * depth rather than taken on faith under the tighter RP2040 SRAM budget). */
#define CORE1_STACK_WORDS 2048u
static uint32_t s_core1_stack[CORE1_STACK_WORDS];

static volatile uint32_t s_heartbeat_ms  = 0;
/* Live (DHCP-leased or static) address, republished each loop for core 0 to
 * read. Same shared-volatile pattern as the heartbeat — no queue involved. It
 * exists because the `net` CLI could only print the STATIC config, which is all
 * zeros under DHCP: a device with a perfectly good lease reported 0.0.0.0, and
 * that is exactly how it reads to anyone diagnosing a network problem. */
static volatile uint8_t  s_live_ip[4]    = {0, 0, 0, 0};
static volatile bool     s_ready         = false;
static volatile bool     s_config_ready  = false;

uint32_t core1_heartbeat_ms(void) {
    return s_heartbeat_ms;
}

void core1_live_ip(uint8_t out[4]) {
    out[0] = s_live_ip[0]; out[1] = s_live_ip[1];
    out[2] = s_live_ip[2]; out[3] = s_live_ip[3];
}

void core1_config_ready(void) {
    s_config_ready = true;
}

static void core1_entry(void) {
    /* Register core 1 as the flash lockout victim. This is the ONLY thing the
     * application must do for multicore flash safety: pico-vfs already wraps its
     * erase/program in flash_safe_execute() (blockdevice/flash.c), which parks
     * this core while core 0 writes. Core 1 never writes flash itself. */
    bool ok = flash_safe_execute_core_init();
    s_heartbeat_ms = to_ms_since_boot(get_absolute_time());
    s_ready = true;   /* core 0's fs_init() flash format may now lock us out */
    LOG_INFO("core1", "flash lockout victim init %s", ok ? "ok" : "FAILED");

    /* Park until core 0 has loaded config + filled the snapshot. Keep beating the
     * heartbeat (core 0 enables the watchdog only after this point, but be safe).
     * While we spin here, core 0's fs_init/config writes can lock us out. */
    while (!s_config_ready) {
        s_heartbeat_ms = to_ms_since_boot(get_absolute_time());
        tight_loop_contents();
    }

    /* Bring up the W5100S and the MQTT connection manager from the config
     * snapshot. Both are non-blocking SMs. */
    const ic_snapshot_t *sn = intercore_snapshot();
    netif_config_t nc;
    memset(&nc, 0, sizeof nc);
    nc.dhcp = sn->net_dhcp;
    memcpy(nc.ip, sn->net_ip, 4);
    memcpy(nc.sn, sn->net_mask, 4);
    memcpy(nc.gw, sn->net_gw, 4);
    memcpy(nc.dns, sn->net_dns, 4);
    memcpy(nc.hostname, sn->net_hostname, sizeof nc.hostname);
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

    web_state_init();
    ha_bridge_init();
    http_server_init();
    ota_init();
    fwupdate_init();

    /* Hydrate the web read model independent of MQTT: ask core 0 to re-emit all
     * cached state now, so the config UI is fully populated even when no broker
     * is reachable. (A second resync also fires after MQTT discovery completes;
     * the re-emit is idempotent.) */
    {
        ic_cmd_t rc; memset(&rc, 0, sizeof rc); rc.type = IC_CMD_RESYNC;
        if (!intercore_push_cmd(&rc))
            LOG_WARN("core1", "boot resync push dropped — web model may start blank");
    }

    uint32_t last_log_ms = 0;
    while (true) {
        s_heartbeat_ms = to_ms_since_boot(get_absolute_time());
        {   /* republish the live address for core 0's `net` command */
            uint8_t ip[4]; netif_get_ip(ip);
            s_live_ip[0] = ip[0]; s_live_ip[1] = ip[1];
            s_live_ip[2] = ip[2]; s_live_ip[3] = ip[3];
        }

        netif_tick();
        mqtt_app_tick();
        ha_bridge_tick();    /* discovery + availability sweep + event->MQTT pump
                              * (also routes IC_EVT_PING_REQ to ping_submit) */
        ping_tick();         /* CLI network ping: borrows socket 0 when idle */
        http_server_tick();  /* config web UI (sockets 2-3) */
        ota_poll();          /* pull-based OTA: borrows socket 0 when running */
        fwupdate_poll();     /* manifest check: borrows socket 0; never overlaps ota_poll() */

        if (s_heartbeat_ms - last_log_ms >= 5000u) {
            last_log_ms = s_heartbeat_ms;
            LOG_DBG("core1", "alive: link=%d ready=%d mqtt=%d evt_drops=%lu",
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
