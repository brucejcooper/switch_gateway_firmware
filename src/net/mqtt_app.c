/*
 * mqtt_app.c — TCP socket state machine + coreMQTT connection manager (core 1).
 */

#include "mqtt_app.h"
#include "netif.h"
#include "log.h"

#include "core_mqtt.h"
#include "socket.h"
#include "wizchip_conf.h"   /* pulls W5100S register macros (getSn_SR, Sn_IR_*) */

#include "pico/time.h"
#include "pico/unique_id.h"
#include <string.h>

#define MQTT_SOCKET             1            /* socket 0 is the DHCP client */
#define MQTT_LOCAL_PORT         50000
#define MQTT_NET_BUF_SIZE       2048
#define MQTT_KEEPALIVE_S        30
#define MQTT_KPALVTR_UNITS      6            /* hardware keepalive: 6 * 5 s = 30 s */
#define MQTT_CONNECT_TIMEOUT_MS 1200         /* < CORE1_STALE_MS (2000) */
#define TCP_CONNECT_TIMEOUT_MS  5000
#define BACKOFF_MIN_MS          1000u
#define BACKOFF_MAX_MS          30000u

/* coreMQTT's opaque transport context. We use a single fixed socket. */
struct NetworkContext { uint8_t sn; };

typedef enum { MA_OFFLINE, MA_TCP_CONNECTING, MA_MQTT_CONNECTING, MA_ONLINE, MA_BACKOFF }
    ma_state_t;

static ma_state_t            s_state = MA_OFFLINE;
static mqtt_config_t         s_cfg;
static MQTTContext_t         s_mqtt;
static struct NetworkContext s_netctx = { MQTT_SOCKET };
static TransportInterface_t  s_transport;
static uint8_t               s_netbuf[MQTT_NET_BUF_SIZE];
static MQTTFixedBuffer_t     s_fixedbuf = { s_netbuf, sizeof s_netbuf };

static char     s_client_id[24];
static char     s_avail_topic[40];
static uint32_t s_backoff_ms = BACKOFF_MIN_MS;
static uint32_t s_deadline   = 0;
static uint32_t s_rng        = 0;

static void (*s_on_connected)(void) = NULL;
static void (*s_on_message)(const char *, uint16_t, const uint8_t *, uint16_t) = NULL;

/* ── time + rng ──────────────────────────────────────────────────────────── */
static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }
static uint32_t rng_next(void) { s_rng = s_rng * 1664525u + 1013904223u; return s_rng; }

/* ── coreMQTT transport (non-blocking; SOCK_BUSY -> 0, error -> -1) ─────────── */
static int32_t transport_recv(NetworkContext_t *ctx, void *buf, size_t len) {
    /* Read Sn_RX_RSR until two reads agree (W5100S volatile-register quirk). */
    uint16_t a, b;
    do { a = getSn_RX_RSR(ctx->sn); b = getSn_RX_RSR(ctx->sn); } while (a != b);
    if (a == 0) return 0;   /* no data yet */

    /* Work around an ioLibrary_Driver bug: on the non-IPv6 path, recv() on a
     * SF_IO_NONBLOCK socket returns SOCK_BUSY *before* checking for available
     * data, so it never delivers a received packet (the IPv6 path has the
     * checks in the correct order). Flip to blocking just for this read — data
     * is confirmed present, so it cannot block — then restore non-block. */
    uint8_t mode = SOCK_IO_BLOCK;
    ctlsocket(ctx->sn, CS_SET_IOMODE, &mode);
    int32_t r = recv(ctx->sn, (uint8_t *)buf, (uint16_t)len);
    mode = SOCK_IO_NONBLOCK;
    ctlsocket(ctx->sn, CS_SET_IOMODE, &mode);
    return r < 0 ? -1 : r;
}
static int32_t transport_send(NetworkContext_t *ctx, const void *buf, size_t len) {
    int32_t r = send(ctx->sn, (uint8_t *)(uintptr_t)buf, (uint16_t)len);
    if (r == SOCK_BUSY) return 0;
    return r < 0 ? -1 : r;
}

/* ── inbound MQTT packets ────────────────────────────────────────────────── */
static void mqtt_event_cb(MQTTContext_t *ctx, MQTTPacketInfo_t *pkt,
                          MQTTDeserializedInfo_t *des) {
    (void)ctx;
    if ((pkt->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH && des->pPublishInfo) {
        const MQTTPublishInfo_t *p = des->pPublishInfo;
        if (s_on_message)
            s_on_message(p->pTopicName, p->topicNameLength,
                         (const uint8_t *)p->pPayload, (uint16_t)p->payloadLength);
    }
}

/* ── TCP socket state machine ────────────────────────────────────────────── */
static void tcp_close(void) {
    disconnect(MQTT_SOCKET);
    close(MQTT_SOCKET);
}

static bool tcp_open(void) {
    close(MQTT_SOCKET);
    int8_t s = socket(MQTT_SOCKET, Sn_MR_TCP, MQTT_LOCAL_PORT, SF_IO_NONBLOCK);
    if (s != MQTT_SOCKET) { LOG_WARN("mqtt", "socket() failed (%d)", s); return false; }
    /* Non-block connect returns SOCK_BUSY immediately; progress is polled. */
    connect(MQTT_SOCKET, s_cfg.broker_ip, s_cfg.broker_port);
    return true;
}

/* Returns 1 = established, 0 = pending, -1 = failed/closed. Acts on Sn_IR. */
static int tcp_poll(void) {
    uint8_t ir = getSn_IR(MQTT_SOCKET);
    if (ir & Sn_IR_TIMEOUT) { setSn_IR(MQTT_SOCKET, Sn_IR_TIMEOUT);
                              LOG_WARN("mqtt", "Sn_IR TIMEOUT (peer gone)"); return -1; }
    if (ir & Sn_IR_DISCON)  { setSn_IR(MQTT_SOCKET, Sn_IR_DISCON);
                              LOG_WARN("mqtt", "Sn_IR DISCON (FIN)"); return -1; }
    if (ir & Sn_IR_CON)       setSn_IR(MQTT_SOCKET, Sn_IR_CON);

    switch (getSn_SR(MQTT_SOCKET)) {
    case SOCK_ESTABLISHED: return 1;
    case SOCK_CLOSE_WAIT:  LOG_WARN("mqtt", "CLOSE_WAIT -> recycle"); tcp_close(); return -1;
    case SOCK_CLOSED:      return -1;
    default:               return 0;   /* INIT / SYNSENT / ... */
    }
}

/* ── coreMQTT connect (with LWT) ─────────────────────────────────────────── */
static bool mqtt_connect(void) {
    s_transport.pNetworkContext = &s_netctx;
    s_transport.recv = transport_recv;
    s_transport.send = transport_send;

    if (MQTT_Init(&s_mqtt, &s_transport, now_ms, mqtt_event_cb, &s_fixedbuf) != MQTTSuccess)
        return false;

    MQTTConnectInfo_t ci;
    memset(&ci, 0, sizeof ci);
    ci.cleanSession          = true;
    ci.keepAliveSeconds      = MQTT_KEEPALIVE_S;
    ci.pClientIdentifier     = s_client_id;
    ci.clientIdentifierLength = (uint16_t)strlen(s_client_id);
    if (s_cfg.have_creds) {
        ci.pUserName     = s_cfg.user; ci.userNameLength = (uint16_t)strlen(s_cfg.user);
        ci.pPassword     = s_cfg.pass; ci.passwordLength = (uint16_t)strlen(s_cfg.pass);
    }

    /* Last Will: HA reflects a dropped Pico immediately. */
    MQTTPublishInfo_t will;
    memset(&will, 0, sizeof will);
    will.qos             = MQTTQoS0;
    will.retain          = true;
    will.pTopicName      = s_avail_topic;
    will.topicNameLength = (uint16_t)strlen(s_avail_topic);
    will.pPayload        = "offline";
    will.payloadLength   = 7;

    bool session = false;
    MQTTStatus_t st = MQTT_Connect(&s_mqtt, &ci, &will, MQTT_CONNECT_TIMEOUT_MS, &session);
    if (st != MQTTSuccess) {
        /* Diagnostics: socket state + unread RX bytes. rxrsr>0 means CONNACK
         * reached the chip but wasn't read (recv path); sr!=ESTABLISHED means
         * the link dropped. */
        LOG_WARN("mqtt", "MQTT_Connect=%d sr=0x%02x rx_rsr=%u tx_fsr=%u",
                 (int)st, getSn_SR(MQTT_SOCKET), getSn_RX_RSR(MQTT_SOCKET),
                 getSn_TX_FSR(MQTT_SOCKET));
        return false;
    }
    return true;
}

static void go_backoff(uint32_t now) {
    /* Abrupt TCP close (no clean MQTT DISCONNECT) so the broker fires our LWT and
     * HA sees us offline immediately. */
    tcp_close();
    uint32_t base = s_backoff_ms;
    uint32_t wait = base / 2 + (rng_next() % (base / 2 + 1));   /* +/- jitter */
    s_deadline   = now + wait;
    s_backoff_ms = base * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : base * 2;
    s_state      = MA_BACKOFF;
    LOG_INFO("mqtt", "backoff ~%lu ms", (unsigned long)wait);
}

/* ── public ──────────────────────────────────────────────────────────────── */
void mqtt_app_init(const mqtt_config_t *cfg) {
    if (cfg) {
        s_cfg = *cfg;
    } else {
        memset(&s_cfg, 0, sizeof s_cfg);
        s_cfg.broker_port = 1883;
        strncpy(s_cfg.prefix, "lighting", sizeof s_cfg.prefix - 1);
    }
    if (s_cfg.prefix[0] == '\0') strncpy(s_cfg.prefix, "lighting", sizeof s_cfg.prefix - 1);

    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    snprintf(s_client_id, sizeof s_client_id, "swgw-%02x%02x%02x%02x",
             id.id[4], id.id[5], id.id[6], id.id[7]);
    snprintf(s_avail_topic, sizeof s_avail_topic, "%s/availability", s_cfg.prefix);
    s_rng = ((uint32_t)id.id[4] << 24) | ((uint32_t)id.id[5] << 16) |
            ((uint32_t)id.id[6] << 8)  |  (uint32_t)id.id[7] | 1u;

    s_state      = MA_OFFLINE;
    s_backoff_ms = BACKOFF_MIN_MS;
    LOG_INFO("mqtt", "init broker=%u.%u.%u.%u:%u prefix=%s id=%s",
             s_cfg.broker_ip[0], s_cfg.broker_ip[1], s_cfg.broker_ip[2], s_cfg.broker_ip[3],
             s_cfg.broker_port, s_cfg.prefix, s_client_id);
}

void mqtt_app_tick(void) {
    uint32_t now = now_ms();

    switch (s_state) {
    case MA_OFFLINE:
        if (!netif_ready()) return;
        if (!tcp_open()) { go_backoff(now); return; }
        s_deadline = now + TCP_CONNECT_TIMEOUT_MS;
        s_state    = MA_TCP_CONNECTING;
        break;

    case MA_TCP_CONNECTING: {
        if (!netif_ready()) { go_backoff(now); break; }
        int r = tcp_poll();
        if (r == 1) {
            setSn_KPALVTR(MQTT_SOCKET, MQTT_KPALVTR_UNITS);   /* HW keepalive */
            s_state = MA_MQTT_CONNECTING;
        } else if (r < 0 || (int32_t)(now - s_deadline) >= 0) {
            if (r >= 0) LOG_WARN("mqtt", "TCP connect timeout");
            go_backoff(now);
        }
        break;
    }

    case MA_MQTT_CONNECTING:
        if (mqtt_connect()) {
            LOG_INFO("mqtt", "connected to broker");
            s_backoff_ms = BACKOFF_MIN_MS;
            mqtt_app_publish(s_avail_topic, "online", 6, true);
            if (s_on_connected) s_on_connected();
            s_state = MA_ONLINE;
        } else {
            go_backoff(now);
        }
        break;

    case MA_ONLINE: {
        if (!netif_ready()) { go_backoff(now); break; }
        if (tcp_poll() < 0) { go_backoff(now); break; }
        MQTTStatus_t st = MQTT_ProcessLoop(&s_mqtt);
        if (st != MQTTSuccess && st != MQTTNeedMoreBytes) {
            LOG_WARN("mqtt", "ProcessLoop=%d -> reconnect", (int)st);
            go_backoff(now);
        }
        break;
    }

    case MA_BACKOFF:
        if ((int32_t)(now - s_deadline) >= 0) s_state = MA_OFFLINE;
        break;
    }
}

bool mqtt_app_connected(void) { return s_state == MA_ONLINE; }

bool mqtt_app_publish(const char *topic, const void *payload, size_t len, bool retain) {
    if (s_state != MA_ONLINE && s_state != MA_MQTT_CONNECTING) return false;
    MQTTPublishInfo_t pi;
    memset(&pi, 0, sizeof pi);
    pi.qos             = MQTTQoS0;
    pi.retain          = retain;
    pi.pTopicName      = topic;
    pi.topicNameLength = (uint16_t)strlen(topic);
    pi.pPayload        = payload;
    pi.payloadLength   = len;
    MQTTStatus_t st = MQTT_Publish(&s_mqtt, &pi, 0);
    if (st != MQTTSuccess) {   /* DEBUG */
        static uint32_t fails = 0;
        if ((fails++ % 40) == 0)
            LOG_WARN("mqtt", "publish st=%d (#%lu) %s", (int)st, (unsigned long)fails, topic);
    }
    return st == MQTTSuccess;
}

bool mqtt_app_subscribe(const char *filter) {
    MQTTSubscribeInfo_t si;
    memset(&si, 0, sizeof si);
    si.qos               = MQTTQoS0;
    si.pTopicFilter      = filter;
    si.topicFilterLength = (uint16_t)strlen(filter);
    return MQTT_Subscribe(&s_mqtt, &si, 1, MQTT_GetPacketId(&s_mqtt)) == MQTTSuccess;
}

uint16_t mqtt_app_tx_free(void) {
    if (s_state != MA_ONLINE && s_state != MA_MQTT_CONNECTING) return 0;
    return getSn_TX_FSR(MQTT_SOCKET);
}

const char *mqtt_app_prefix(void) { return s_cfg.prefix; }

void mqtt_app_set_connected_cb(void (*cb)(void)) { s_on_connected = cb; }
void mqtt_app_set_message_cb(void (*cb)(const char *, uint16_t, const uint8_t *, uint16_t)) {
    s_on_message = cb;
}
