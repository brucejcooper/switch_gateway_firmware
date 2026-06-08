/*
 * netif.c — W5100S bring-up / DHCP / PHY / recovery state machine (core 1).
 */

#include "netif.h"
#include "wizchip_glue.h"
#include "log.h"

#include "wizchip_conf.h"
#include "socket.h"
#include "dhcp.h"

#include "pico/unique_id.h"
#include "pico/time.h"
#include <string.h>

#define W5100S_VERSION  0x51
#define WIZ_RTR         2000u   /* retransmit timeout, 100 us units -> 200 ms */
#define WIZ_RCR         3u      /* retry count -> ~0.8 s to declare a dead peer */

#define DHCP_SOCKET     0
#define DHCP_BUF_SIZE   1024
static uint8_t s_dhcp_buf[DHCP_BUF_SIZE];

typedef enum { NS_RESET, NS_LINK_WAIT, NS_DHCP, NS_READY, NS_FAULT } netif_state_t;

static netif_state_t s_state = NS_RESET;
static netif_config_t s_cfg;
static wiz_NetInfo    s_netinfo;
static uint32_t       s_last_1s_ms   = 0;
static uint32_t       s_fault_until   = 0;
static uint8_t        s_fault_count   = 0;

/* ── helpers ─────────────────────────────────────────────────────────────── */
static void build_mac(uint8_t mac[6]) {
    /* Stable per-board, locally-administered unicast MAC from the flash uid. */
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    mac[0] = 0x02;
    for (int i = 0; i < 5; i++) mac[i + 1] = id.id[i + 3];
}

static bool link_up(void) {
    uint8_t v = PHY_LINK_OFF;
    ctlwizchip(CW_GET_PHYLINK, &v);
    return v == PHY_LINK_ON;
}

static void enter_fault(uint32_t now) {
    s_fault_count++;
    uint32_t shift = s_fault_count > 5 ? 5 : s_fault_count;
    uint32_t backoff = 500u << shift;   /* 1 s .. 16 s */
    s_fault_until = now + backoff;
    s_state = NS_FAULT;
    LOG_WARN("net", "fault #%u, backoff %lu ms", s_fault_count,
             (unsigned long)backoff);
}

static bool chip_bringup(void) {
    wizchip_glue_reset();
    uint8_t txsz[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    uint8_t rxsz[8] = { 2, 2, 2, 2, 2, 2, 2, 2 };
    if (wizchip_init(txsz, rxsz) != 0) {
        LOG_ERR("net", "wizchip_init failed");
        return false;
    }
    uint8_t ver = getVER();
    if (ver != W5100S_VERSION) {
        LOG_ERR("net", "bad W5100S VER 0x%02x (want 0x%02x) — SPI desync?",
                ver, W5100S_VERSION);
        return false;
    }
    setRTR(WIZ_RTR);
    setRCR(WIZ_RCR);
    wizchip_setnetinfo(&s_netinfo);     /* programs MAC (ip still zero) */
    LOG_INFO("net", "W5100S up VER=0x%02x RTR=%u RCR=%u", ver, WIZ_RTR, WIZ_RCR);
    return true;
}

static void log_ip(const char *what) {
    LOG_INFO("net", "%s ip=%u.%u.%u.%u gw=%u.%u.%u.%u dns=%u.%u.%u.%u", what,
             s_netinfo.ip[0], s_netinfo.ip[1], s_netinfo.ip[2], s_netinfo.ip[3],
             s_netinfo.gw[0], s_netinfo.gw[1], s_netinfo.gw[2], s_netinfo.gw[3],
             s_netinfo.dns[0], s_netinfo.dns[1], s_netinfo.dns[2], s_netinfo.dns[3]);
}

static void dhcp_assigned(void) {
    getIPfromDHCP(s_netinfo.ip);
    getGWfromDHCP(s_netinfo.gw);
    getSNfromDHCP(s_netinfo.sn);
    getDNSfromDHCP(s_netinfo.dns);
    s_netinfo.dhcp = NETINFO_DHCP;
    wizchip_setnetinfo(&s_netinfo);
    log_ip("DHCP lease");
}
static void dhcp_conflict(void) { LOG_WARN("net", "DHCP IP conflict"); }

/* ── public ──────────────────────────────────────────────────────────────── */
void netif_init(const netif_config_t *cfg) {
    if (cfg) {
        s_cfg = *cfg;
    } else {
        memset(&s_cfg, 0, sizeof s_cfg);
        s_cfg.dhcp = true;
    }
    memset(&s_netinfo, 0, sizeof s_netinfo);
    build_mac(s_netinfo.mac);
    s_netinfo.dhcp = s_cfg.dhcp ? NETINFO_DHCP : NETINFO_STATIC;

    wizchip_glue_init();
    s_state = NS_RESET;
    LOG_INFO("net", "init mac=%02x:%02x:%02x:%02x:%02x:%02x mode=%s",
             s_netinfo.mac[0], s_netinfo.mac[1], s_netinfo.mac[2],
             s_netinfo.mac[3], s_netinfo.mac[4], s_netinfo.mac[5],
             s_cfg.dhcp ? "dhcp" : "static");
}

void netif_tick(void) {
    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (s_state) {
    case NS_RESET:
        if (!chip_bringup()) { enter_fault(now); break; }
        s_fault_count = 0;
        s_state = NS_LINK_WAIT;
        break;

    case NS_LINK_WAIT:
        if (!link_up()) break;          /* park until the cable is in */
        LOG_INFO("net", "PHY link up");
        if (s_cfg.dhcp) {
            DHCP_init(DHCP_SOCKET, s_dhcp_buf);
            reg_dhcp_cbfunc(dhcp_assigned, dhcp_assigned, dhcp_conflict);
            s_last_1s_ms = now;
            s_state = NS_DHCP;
        } else {
            memcpy(s_netinfo.ip, s_cfg.ip, 4);
            memcpy(s_netinfo.sn, s_cfg.sn, 4);
            memcpy(s_netinfo.gw, s_cfg.gw, 4);
            memcpy(s_netinfo.dns, s_cfg.dns, 4);
            s_netinfo.dhcp = NETINFO_STATIC;
            wizchip_setnetinfo(&s_netinfo);
            log_ip("static");
            s_state = NS_READY;
        }
        break;

    case NS_DHCP:
        if (!link_up()) { s_state = NS_LINK_WAIT; break; }
        if (now - s_last_1s_ms >= 1000) { s_last_1s_ms += 1000; DHCP_time_handler(); }
        switch (DHCP_run()) {
        case DHCP_IP_LEASED: s_state = NS_READY; break;
        case DHCP_FAILED:    LOG_WARN("net", "DHCP failed"); enter_fault(now); break;
        default: break;
        }
        break;

    case NS_READY:
        if (!link_up()) { LOG_WARN("net", "PHY link down"); s_state = NS_LINK_WAIT; break; }
        if (s_cfg.dhcp) {   /* keep the lease renewed */
            if (now - s_last_1s_ms >= 1000) { s_last_1s_ms += 1000; DHCP_time_handler(); }
            if (DHCP_run() == DHCP_FAILED) enter_fault(now);
        }
        break;

    case NS_FAULT:
        if ((int32_t)(now - s_fault_until) < 0) break;   /* backoff */
        LOG_INFO("net", "recovery: full chip reset + re-init");
        s_state = NS_RESET;
        break;
    }
}

bool netif_ready(void)   { return s_state == NS_READY; }
bool netif_link_up(void) { return link_up(); }
void netif_get_ip(uint8_t ip[4])   { memcpy(ip, s_netinfo.ip, 4); }
void netif_get_dns(uint8_t dns[4]) { memcpy(dns, s_netinfo.dns, 4); }
