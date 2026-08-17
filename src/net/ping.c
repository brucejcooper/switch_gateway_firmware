/*
 * ping.c — CLI network ping (ICMP echo) on core 1.
 *
 * The W5100S has no ping helper, so we open an IPRAW socket (IP protocol ICMP),
 * build echo-request packets by hand, and poll for replies. All four sockets
 * are assigned (0=DHCP, 1=MQTT, 2-3=web), so we borrow the idle DHCP socket 0
 * through the netif arbiter — the same revocable loan the web server uses —
 * held only for the ~1 s a run takes.
 *
 * Non-blocking: ping_tick() advances at most one step per core-1 loop pass.
 * Each echo's outcome is pushed to core 0 as an IC_CMD_PING_REPLY.
 *
 * Hot-path rule (CLAUDE.md): no logging here — core-1 net code must not printf
 * (USB-CDC back-pressure stalls the W5100S).
 */

#include "ping.h"
#include "netif.h"
#include "intercore.h"

#include "socket.h"
#include "wizchip_conf.h"   /* W5100S register macros: setSn_PROTO, Sn_MR_IPRAW, IPPROTO_ICMP */

#include "pico/time.h"
#include <string.h>

#define PING_SOCK        0          /* borrowed DHCP socket */
#define PING_LOCAL_PORT  3000u      /* arbitrary; ignored in IPRAW mode */
#define PING_TIMEOUT_MS  1000u      /* per-echo reply wait */
#define PING_PAYLOAD     32u        /* echo data bytes */
#define PING_MAX_COUNT   16u

#define ICMP_ECHO_REQUEST 8u
#define ICMP_ECHO_REPLY   0u

typedef enum { PH_IDLE, PH_OPEN, PH_SEND, PH_WAIT } ping_phase_t;

static struct {
    bool         active;
    ping_phase_t phase;
    uint8_t      ip[4];
    uint8_t      count;       /* total echoes requested */
    uint8_t      seq;         /* current echo, 0..count-1 */
    uint16_t     id;          /* ICMP identifier for this run */
    bool         have_sock;   /* socket 0 acquired */
    uint32_t     send_ms;     /* when the current echo was sent */
} P;

static uint16_t s_run_id = 0;

static void advance(void);   /* send_echo() reports an unreachable echo via this */

/* 16-bit one's-complement checksum (RFC 1071) over len bytes, big-endian. */
static uint16_t icmp_checksum(const uint8_t *data, uint16_t len) {
    uint32_t sum = 0;
    for (uint16_t i = 0; i + 1 < len; i += 2)
        sum += ((uint32_t)data[i] << 8) | data[i + 1];
    if (len & 1) sum += (uint32_t)data[len - 1] << 8;
    while (sum >> 16) sum = (sum & 0xFFFFu) + (sum >> 16);
    return (uint16_t)~sum;
}

/* Push one echo outcome up to core 0. */
static void emit(uint8_t seq, ic_ping_status_t status, uint16_t rtt_ms) {
    ic_cmd_t c;
    memset(&c, 0, sizeof c);
    c.type = IC_CMD_PING_REPLY;
    memcpy(c.ping_reply.ip, P.ip, 4);
    c.ping_reply.seq    = seq;
    c.ping_reply.count  = P.count;
    c.ping_reply.status = (uint8_t)status;
    c.ping_reply.rtt_ms = rtt_ms;
    intercore_push_cmd(&c);
}

static void finish(void) {
    close(PING_SOCK);
    if (P.have_sock) { netif_sock0_release(); P.have_sock = false; }
    P.active = false;
    P.phase  = PH_IDLE;
}

void ping_submit(const uint8_t ip[4], uint8_t count) {
    if (P.active) {
        /* Busy with another run — report BUSY for the new target only. */
        ic_cmd_t c;
        memset(&c, 0, sizeof c);
        c.type = IC_CMD_PING_REPLY;
        memcpy(c.ping_reply.ip, ip, 4);
        c.ping_reply.count  = count;
        c.ping_reply.status = (uint8_t)IC_PING_BUSY;
        intercore_push_cmd(&c);
        return;
    }
    if (count == 0) count = 1;
    if (count > PING_MAX_COUNT) count = PING_MAX_COUNT;
    memset(&P, 0, sizeof P);
    P.active = true;
    P.phase  = PH_OPEN;
    memcpy(P.ip, ip, 4);
    P.count  = count;
    P.id     = ++s_run_id;
}

static void send_echo(void) {
    uint8_t pkt[8 + PING_PAYLOAD];
    memset(pkt, 0, sizeof pkt);
    pkt[0] = ICMP_ECHO_REQUEST;
    pkt[1] = 0;                                  /* code */
    pkt[4] = (uint8_t)(P.id >> 8);               /* identifier (BE) */
    pkt[5] = (uint8_t)P.id;
    pkt[6] = 0;                                  /* sequence (BE) */
    pkt[7] = P.seq;
    for (uint16_t i = 0; i < PING_PAYLOAD; i++) pkt[8 + i] = (uint8_t)i;
    uint16_t ck = icmp_checksum(pkt, sizeof pkt);
    pkt[2] = (uint8_t)(ck >> 8);
    pkt[3] = (uint8_t)ck;

    /* sendto blocks until SEND_OK or the chip's ARP retry budget expires. For an
     * on-link destination that never ARP-answers it returns a timeout error —
     * that's an unreachable host, not a tool failure, so report this echo as a
     * timeout and move on rather than aborting the whole run. (The block is
     * bounded by RTR*RCR, ~1.6 s at the WIZnet defaults — under the 2 s core-1
     * staleness window, so the watchdog is safe.) */
    int32_t r = sendto(PING_SOCK, pkt, sizeof pkt, P.ip, PING_LOCAL_PORT);
    if (r < 0) { emit(P.seq, IC_PING_TIMEOUT, 0); advance(); return; }
    P.send_ms = to_ms_since_boot(get_absolute_time());
    P.phase   = PH_WAIT;
}

static void advance(void) {
    P.seq++;
    if (P.seq >= P.count) finish();
    else P.phase = PH_SEND;
}

void ping_tick(void) {
    if (!P.active) return;
    uint32_t now = to_ms_since_boot(get_absolute_time());

    switch (P.phase) {
    case PH_OPEN:
        if (!netif_sock0_acquire()) { emit(0, IC_PING_BUSY, 0); finish(); return; }
        P.have_sock = true;
        close(PING_SOCK);
        setSn_PROTO(PING_SOCK, IPPROTO_ICMP);
        if (socket(PING_SOCK, Sn_MR_IPRAW, PING_LOCAL_PORT, 0) != PING_SOCK) {
            emit(0, IC_PING_ERROR, 0); finish(); return;
        }
        setSn_PROTO(PING_SOCK, IPPROTO_ICMP);   /* ensure proto survives OPEN */
        P.phase = PH_SEND;
        return;

    case PH_SEND:
        send_echo();
        return;

    case PH_WAIT: {
        /* Consume a few packets per tick, looking for our echo reply. The IPRAW
         * socket sees every ICMP to us, so filter by type/id/seq/source. Some
         * stacks hand back the full IP datagram, others just the IP payload —
         * detect a leading IPv4 header and skip it either way. */
        for (int n = 0; n < 4 && getSn_RX_RSR(PING_SOCK) > 0; n++) {
            uint8_t  rbuf[8 + PING_PAYLOAD + 40];
            uint8_t  raddr[4];
            uint16_t rport = 0;
            int32_t  got = recvfrom(PING_SOCK, rbuf, sizeof rbuf, raddr, &rport);
            if (got < 8) continue;

            const uint8_t *icmp = rbuf;
            int32_t icmp_len = got;
            if ((rbuf[0] >> 4) == 4) {                 /* leading IPv4 header */
                uint8_t ihl = (uint8_t)((rbuf[0] & 0x0F) * 4);
                if (got <= ihl + 8) continue;
                icmp = rbuf + ihl;
                icmp_len = got - ihl;
            }
            if (icmp_len < 8 || icmp[0] != ICMP_ECHO_REPLY) continue;

            uint16_t rid = ((uint16_t)icmp[4] << 8) | icmp[5];
            if (rid != P.id || icmp[7] != P.seq) continue;
            if (memcmp(raddr, P.ip, 4) != 0) continue;

            uint16_t rtt = (uint16_t)(now - P.send_ms);
            emit(P.seq, IC_PING_REPLY, rtt);
            advance();
            return;
        }
        if (now - P.send_ms >= PING_TIMEOUT_MS) { emit(P.seq, IC_PING_TIMEOUT, 0); advance(); }
        return;
    }

    default:
        finish();
        return;
    }
}
