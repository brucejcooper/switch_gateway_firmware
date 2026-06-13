#pragma once
/*
 * intercore.h — the core0 <-> core1 contract.
 *
 * Core 0 runs all domain logic (Modbus master, button FSM, dirty-poll, state
 * cache, persistence) and the USB text CLI. Core 1 runs the network bridge
 * (W5100S + MQTT to Home Assistant). They exchange *typed C structs* over two
 * pico_util SPSC queues — NOT serialised JSON. JSON exists only at core 1's MQTT
 * edge. This avoids a format-on-core0 / re-parse-on-core1 round trip and shrinks
 * the largest message (scan_complete) from ~4.5 KB of JSON to 9 bytes.
 *
 * Both queues live in SRAM behind pico_util spinlocks, leaving the SIO inter-core
 * FIFO free for the flash_safe_execute() multicore-lockout handshake.
 *
 *   core0 -> core1 : ic_event_t  (domain state changes -> MQTT publishes)
 *   core1 -> core0 : ic_cmd_t    (MQTT commands -> domain actions)
 */

#include <stdint.h>
#include <stdbool.h>
#include "mapping.h"   /* mapping_target_t, MAX_TARGETS_PER_BTN */

/* Longest provisioning status message carried to core 1 (matches the gateway's
 * PROV_STATUS_MSG_MAX; an independent constant to avoid header coupling). */
#define IC_PROV_MSG_MAX 80

/* Tri-state for on/off/toggle commands. */
typedef enum { IC_TRI_OFF = 0, IC_TRI_ON = 1, IC_TRI_TOGGLE = 2 } ic_tri_t;

/* Button action — mirrors the FSM's action strings. */
typedef enum {
    IC_BTN_PRESS = 0,         /* physical button down (every interaction) */
    IC_BTN_RELEASE,           /* physical button up (every interaction) */
    IC_BTN_CLICK,             /* completed short tap (down->up before long-press) */
    IC_BTN_LONG_PRESS,        /* held past the long-press threshold */
    IC_BTN_LONG_PRESS_REPEAT, /* repeats while held */
} ic_btn_action_t;

/* ── core0 -> core1 : events ─────────────────────────────────────────────── */
typedef enum {
    IC_EVT_RELAY = 0,
    IC_EVT_DALI,
    IC_EVT_GROUP,
    IC_EVT_GATEWAY,
    IC_EVT_PRESENCE,
    IC_EVT_FAULT,
    IC_EVT_SCAN_COMPLETE,
    IC_EVT_BUTTON,
    IC_EVT_MAPPING,
    IC_EVT_PROVISION,
    IC_EVT_CFG,
    IC_EVT_PING_REQ,   /* CLI 'net ping' request: run an ICMP echo on core 1 */
} ic_evt_type_t;

typedef struct {
    ic_evt_type_t type;
    union {
        struct { uint8_t ch; bool on; } relay;
        /* DALI ballast state + bundled config (filled from state_cache at push
         * time so core 1 needs no cache of its own). status is the raw DALI
         * status byte (bit1 lamp_fail, bit4 fading); arc 0 = off. */
        struct { uint8_t slave, addr, arc, status,
                         min, max, power_on, sys_fail, fade_rate, fade_time; } dali;
        /* Group derived state + present-member mask (intersect with present on
         * the consumer if needed). */
        struct { uint8_t slave, group, bri; bool on; uint64_t members; } group;
        struct { uint8_t slave; bool online; } gateway;
        struct { uint8_t slave; uint64_t appeared, disappeared; } presence;
        struct { uint8_t slave, addr; bool lamp_fail, power_cycle; } fault;
        struct { uint8_t slave; uint64_t present_mask; } scan_complete;
        struct { uint8_t fixture, button; ic_btn_action_t action; } button;
        struct { uint8_t fixture, button, count;
                 mapping_target_t targets[MAX_TARGETS_PER_BTN]; } mapping;
        /* state = raw prov_run_state_t value (consumer maps it to the short state
         * string); msg is the gateway's free-form status text, carried verbatim
         * so HA's "Commission Status" sensor keeps its detail. */
        struct { uint8_t slave, state, count; char msg[IC_PROV_MSG_MAX]; } provision;
        struct { uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
                 uint32_t baud; uint8_t de_mode; } cfg;
        /* CLI network-ping request: send `count` ICMP echoes to ip on core 1. */
        struct { uint8_t ip[4]; uint8_t count; } ping_req;
    };
} ic_event_t;

/* ── core1 -> core0 : commands ───────────────────────────────────────────── */

/* set_dali field-present bits */
#define IC_DALI_F_ON        (1u << 0)
#define IC_DALI_F_BRI       (1u << 1)
#define IC_DALI_F_MIN       (1u << 2)
#define IC_DALI_F_MAX       (1u << 3)
#define IC_DALI_F_POWER_ON  (1u << 4)
#define IC_DALI_F_SYS_FAIL  (1u << 5)
#define IC_DALI_F_FADE_RATE (1u << 6)
#define IC_DALI_F_FADE_TIME (1u << 7)

/* set_group / set_cfg field-present bits */
#define IC_GRP_F_ON         (1u << 0)
#define IC_GRP_F_BRI        (1u << 1)
#define IC_GRP_F_MEMBERS    (1u << 2)

#define IC_CFG_F_LONG_PRESS (1u << 0)
#define IC_CFG_F_DIM_REPEAT (1u << 1)
#define IC_CFG_F_REVERSAL   (1u << 2)
#define IC_CFG_F_BAUD       (1u << 3)
#define IC_CFG_F_DE_MODE    (1u << 4)

/* set_netcfg field-present bits */
#define IC_NET_F_DHCP       (1u << 0)
#define IC_NET_F_IP         (1u << 1)
#define IC_NET_F_MASK       (1u << 2)
#define IC_NET_F_GW         (1u << 3)
#define IC_NET_F_DNS        (1u << 4)

/* set_mqttcfg field-present bits */
#define IC_MQTT_F_HOST      (1u << 0)
#define IC_MQTT_F_PORT      (1u << 1)
#define IC_MQTT_F_USER      (1u << 2)
#define IC_MQTT_F_PASS      (1u << 3)
#define IC_MQTT_F_PREFIX    (1u << 4)

/* provision mode — mirrors prov_mode_t */
typedef enum { IC_PROV_ALL = 0, IC_PROV_UNADDRESSED = 1, IC_PROV_ADDRESS = 2 } ic_prov_mode_t;

/* network-ping per-echo outcome, carried back to core 0 for CLI printing. */
typedef enum {
    IC_PING_REPLY = 0,   /* echo reply received; rtt_ms valid */
    IC_PING_TIMEOUT,     /* no reply within the per-echo deadline */
    IC_PING_BUSY,        /* couldn't borrow a socket (e.g. DHCP renewing); run aborted */
    IC_PING_ERROR,       /* socket/send failure; run aborted */
} ic_ping_status_t;

typedef enum {
    IC_CMD_SET_RELAY = 0,
    IC_CMD_SET_DALI,
    IC_CMD_SET_GROUP,
    IC_CMD_SET_CFG,
    IC_CMD_SET_MAPPING,
    IC_CMD_DEL_MAPPING,
    IC_CMD_PROVISION_START,
    IC_CMD_PROVISION_ABORT,
    IC_CMD_RESYNC,
    IC_CMD_SET_NETCFG,    /* network settings — reboot to apply */
    IC_CMD_SET_MQTTCFG,   /* MQTT broker settings — reboot to apply */
    IC_CMD_SET_AUTH,      /* web UI Basic-auth credentials */
    IC_CMD_PING_REPLY,    /* one network-ping echo outcome -> CLI (core 1 -> core 0) */
} ic_cmd_type_t;

typedef struct {
    ic_cmd_type_t type;
    union {
        struct { uint8_t ch; ic_tri_t on; } set_relay;
        struct { uint8_t slave, addr; uint16_t fields; ic_tri_t on;
                 uint8_t bri, min, max, power_on, sys_fail, fade_rate, fade_time; } set_dali;
        /* members is a replace-mask, applied when IC_GRP_F_MEMBERS is set. */
        struct { uint8_t slave, group; uint16_t fields; ic_tri_t on; uint8_t bri;
                 uint64_t members; } set_group;
        struct { uint16_t fields; uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
                 uint32_t baud; uint8_t de_mode; } set_cfg;
        struct { uint8_t fixture, button, count;
                 mapping_target_t targets[MAX_TARGETS_PER_BTN]; } set_mapping;
        struct { uint8_t fixture, button; } del_mapping;
        struct { uint8_t slave; ic_prov_mode_t mode; uint8_t target; } provision_start;
        struct { uint8_t slave; } provision_abort;
        /* Network + MQTT settings persisted by core 0 (reboot to apply, since
         * core 1 reads them from the launch snapshot). */
        struct { uint16_t fields; uint8_t dhcp;
                 uint8_t ip[4], mask[4], gw[4], dns[4]; } set_netcfg;
        struct { uint16_t fields; uint16_t port;
                 char host[64], user[48], pass[64], prefix[24]; } set_mqttcfg;
        struct { char user[32], pass[48]; } set_auth;
        /* One ICMP echo outcome. status is ic_ping_status_t; for the terminal
         * BUSY/ERROR cases only one is sent (seq unused). count echoes the
         * request total so core 0 knows when the run is done. */
        struct { uint8_t ip[4]; uint8_t seq, count, status; uint16_t rtt_ms; } ping_reply;
    };
} ic_cmd_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/* Init both queues. Call once on core 0 before launching core 1. */
void intercore_init(void);

/* core0 producer / core1 consumer (event queue). Non-blocking; push returns
 * false (and bumps the drop counter) if the queue is full. */
bool intercore_push_event(const ic_event_t *e);
bool intercore_pop_event(ic_event_t *e);

/* core1 producer / core0 consumer (command queue). Non-blocking. */
bool intercore_push_cmd(const ic_cmd_t *c);
bool intercore_pop_cmd(ic_cmd_t *c);

/* Total events dropped on a full event queue (diagnostics). */
uint32_t intercore_event_drops(void);

/* ── Config snapshot (core 0 -> core 1) ──────────────────────────────────────
 * The slave lists core 1 needs to build HA discovery. Filled by core 0 before
 * core 1 launch (and is effectively read-only thereafter; a runtime slave-list
 * change needs a reboot to re-discover, like a baud change). */
typedef struct {
    uint8_t  relay_slaves[MAX_RELAY_SLAVES];
    uint8_t  relay_count;
    uint8_t  dali_slaves[MAX_DALI_SLAVES];
    uint8_t  dali_count;

    /* MQTT broker (host already resolved to an IP by core 0). */
    uint8_t  mqtt_ip[4];
    uint16_t mqtt_port;
    char     mqtt_user[48];
    char     mqtt_pass[64];
    char     mqtt_prefix[24];
    bool     mqtt_have_creds;

    /* Network. */
    bool     net_dhcp;
    uint8_t  net_ip[4], net_mask[4], net_gw[4], net_dns[4];

    /* Config web UI Basic-auth credentials (core 1 checks these). */
    char     web_user[32];
    char     web_pass[48];
} ic_snapshot_t;

void intercore_set_snapshot(const ic_snapshot_t *s);
const ic_snapshot_t *intercore_snapshot(void);
