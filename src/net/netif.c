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

/* ioLibrary's DHCP client reads its option-12 hostname from this global (defined
 * in Internet/DHCP/dhcp.c; our build-time patch 0002 widens it to 32 bytes and
 * drops the MAC-hex suffix, so the wire hostname is exactly what we copy here). */
extern uint8_t HOST_NAME[];

/* Socket-0 lending: the W5100S has only 4 sockets (0=DHCP, 1=MQTT), leaving 2
 * for the web server. Between DHCP renewals the DHCP socket sits idle, so we
 * lend it to the web LISTEN pool as an opportunistic 3rd socket and reclaim it a
 * guard band before the T1 renewal (lease/2). The invariant that makes this safe
 * is simple: while lent, netif must NOT call DHCP_run() (its socket-0 self-heal,
 * dhcp.c:797, would reopen UDP over the web TCP socket). The web server treats
 * socket 0 as revocable bonus capacity and never depends on it. */
#define DHCP_RENEW_GUARD_S  15u
static volatile bool s_web_sock0 = false;   /* may the web server use socket 0 now? */
static uint32_t      s_lease_elapsed_s = 0; /* seconds since the current lease/renew */
/* The CLI network-ping borrows socket 0 too (IPRAW/ICMP). While it holds it,
 * netif must not touch socket 0 (no DHCP_run, no web lend) — same invariant as
 * the web loan, with ping taking priority for the ~1 s it runs. */
static volatile bool s_ping_sock0 = false;  /* does the ping own socket 0 now? */

typedef enum { NS_RESET, NS_LINK_WAIT, NS_DHCP, NS_READY, NS_FAULT } netif_state_t;

static netif_state_t s_state = NS_RESET;
static netif_config_t s_cfg;
static wiz_NetInfo    s_netinfo;
static uint32_t       s_last_1s_ms   = 0;
static uint32_t       s_fault_until   = 0;
static uint8_t        s_fault_count   = 0;
static volatile bool  s_recovery_req  = false;  /* set by netif_request_recovery() */

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
        /* Diagnostics: re-read VER, and do a register write/read-back test to
         * distinguish a dead SPI link (reads 0x0000/0xFFFF) from an anomalous
         * VER read (RTR reads back 0x1234 -> SPI actually works). */
        uint8_t v2 = getVER(), v3 = getVER();
        setRTR(0x1234);
        uint16_t rtr = getRTR();
        LOG_ERR("net", "bad VER 0x%02x (re-read 0x%02x 0x%02x); RTR wr=0x1234 rd=0x%04x",
                ver, v2, v3, rtr);
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
    s_lease_elapsed_s = 0;              /* fresh lease/renew: restart the T1 clock */
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
    /* Publish the DHCP hostname before any DISCOVER/REQUEST is built. Empty =>
     * leave dhcp.c's compiled-in default ("WIZnet") untouched. */
    if (s_cfg.hostname[0]) {
        size_t i = 0;
        for (; i < sizeof s_cfg.hostname - 1 && s_cfg.hostname[i]; i++)
            HOST_NAME[i] = (uint8_t)s_cfg.hostname[i];
        HOST_NAME[i] = 0;
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

    /* Default each tick: socket 0 belongs to DHCP. The NS_READY arms below grant
     * it to the web server when it is genuinely idle. */
    s_web_sock0 = false;

    /* Honour an externally-requested full recovery (e.g. wedged web sockets):
     * drop to NS_RESET for a clean chip re-init. Not via enter_fault() — this is
     * a deliberate, targeted reset, not a cascading fault, so don't inflate the
     * fault backoff. */
    if (s_recovery_req) {
        s_recovery_req = false;
        LOG_WARN("net", "recovery requested — full W5100S reset");
        s_state = NS_RESET;
        return;
    }

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
        if (s_ping_sock0) {
            /* Ping owns socket 0 for the moment: keep the DHCP lease clock
             * ticking but do NOT call DHCP_run() or re-lend to the web server
             * (the ping has an IPRAW socket open on 0). Released within ~1 s. */
            if (s_cfg.dhcp && now - s_last_1s_ms >= 1000) {
                s_last_1s_ms += 1000;
                DHCP_time_handler();
                s_lease_elapsed_s++;
            }
            break;
        }
        if (s_cfg.dhcp) {   /* keep the lease renewed, lend socket 0 between renews */
            if (now - s_last_1s_ms >= 1000) {
                s_last_1s_ms += 1000;
                DHCP_time_handler();        /* must keep ticking even while lending */
                s_lease_elapsed_s++;
            }
            uint32_t lease = getDHCPLeasetime();        /* seconds */
            bool finite = lease != 0 && lease != 0xFFFFFFFFu;
            bool renewal_due = finite &&
                (s_lease_elapsed_s + DHCP_RENEW_GUARD_S >= lease / 2);
            if (renewal_due) {
                /* Reclaim socket 0 for the renewal. DHCP_run()'s self-heal
                 * reopens it as UDP (stomping any web TCP use); since netif_tick
                 * runs before http_server_tick, the web server sees the flag drop
                 * this same tick and backs off. */
                if (DHCP_run() == DHCP_FAILED) enter_fault(now);
            } else {
                /* Idle mid-lease: lend socket 0. Deliberately skip DHCP_run() so
                 * it can't reopen UDP over the web socket. */
                s_web_sock0 = true;
            }
        } else {
            s_web_sock0 = true;   /* static IP: no DHCP socket, lend it permanently */
        }
        break;

    case NS_FAULT:
        if ((int32_t)(now - s_fault_until) < 0) break;   /* backoff */
        LOG_INFO("net", "recovery: full chip reset + re-init");
        s_state = NS_RESET;
        break;
    }
}

void netif_request_recovery(void) { s_recovery_req = true; }

bool netif_ready(void)   { return s_state == NS_READY; }
bool netif_link_up(void) { return link_up(); }
bool netif_web_sock0_enabled(void) { return s_web_sock0; }

bool netif_ping_acquire_sock0(void) {
    if (s_state != NS_READY || s_ping_sock0) return false;
    /* Only grant when socket 0 is genuinely free: static IP always, or DHCP not
     * inside the pre-renewal guard band (mirrors the web-lend decision). */
    if (s_cfg.dhcp) {
        uint32_t lease = getDHCPLeasetime();
        bool finite = lease != 0 && lease != 0xFFFFFFFFu;
        if (finite && s_lease_elapsed_s + DHCP_RENEW_GUARD_S >= lease / 2)
            return false;
    }
    s_web_sock0  = false;   /* web server backs off socket 0 next tick */
    s_ping_sock0 = true;
    return true;
}

void netif_ping_release_sock0(void) { s_ping_sock0 = false; }
void netif_get_ip(uint8_t ip[4])   { memcpy(ip, s_netinfo.ip, 4); }
void netif_get_dns(uint8_t dns[4]) { memcpy(dns, s_netinfo.dns, 4); }
