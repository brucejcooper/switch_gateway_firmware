#include "lighting_actions.h"
#include "dali_provision.h"
#include "modbus.h"
#include "state_cache.h"
#include "config.h"
#include "serial_protocol.h"
#include <stdio.h>

/* TinyModbus FC06 holding register addresses */
#define DALI_REG_GOTO_LAST  0x1003u
#define DALI_REG_OFF        0x1004u
#define DALI_REG_UP         0x1005u
#define DALI_REG_DOWN       0x1006u
#define DALI_REG_DAPC       0x1002u

/* Short address, command form (A0=1) */
static inline uint16_t dali_cmd_addr(uint8_t addr) {
    return ((uint16_t)(addr & 0x3F) << 1) | 0x01u;
}
/* Short address, DAPC form (A0=0) */
static inline uint16_t dali_dapc_addr(uint8_t addr) {
    return (uint16_t)(addr & 0x3F) << 1;
}

/* ── State event queue — written here, drained by serial_poll() ──────────── */
/* We use a simple ring buffer rather than the pico queue_t here because all
 * accesses are on core 0 (callbacks from modbus_poll(), which runs on core 0). */
#define EVT_QUEUE_SIZE 64

typedef enum {
    LEVT_RELAY, LEVT_DALI, LEVT_PRESENCE, LEVT_FAULT, LEVT_SCAN_COMPLETE
} levt_type_t;

typedef struct {
    levt_type_t type;
    union {
        struct { uint8_t ch; bool on; }                       relay;
        struct { uint8_t slave_id; uint8_t addr;
                 uint8_t arc; uint8_t status; }               dali;
        struct { uint8_t slave_id;
                 uint64_t appeared; uint64_t disappeared; }   presence;
        struct { uint8_t slave_id; uint8_t addr;
                 bool lamp_fail; bool power_cycle; }          fault;
        struct { uint8_t slave_id; uint64_t present_mask; }  scan_complete;
    };
} levt_t;

static levt_t  s_evts[EVT_QUEUE_SIZE];
static uint8_t s_evt_head = 0;
static uint8_t s_evt_tail = 0;

static void push_evt(const levt_t *e) {
    uint8_t next = (s_evt_head + 1) % EVT_QUEUE_SIZE;
    if (next == s_evt_tail) {
        printf("lighting: event queue full — dropped\n");
        return;
    }
    s_evts[s_evt_head] = *e;
    s_evt_head = next;
}

/* Called by serial_poll() to drain events */
bool lighting_evt_try_pop(levt_t *out);

bool lighting_evt_try_pop(levt_t *out) {
    if (s_evt_head == s_evt_tail) return false;
    *out = s_evts[s_evt_tail];
    s_evt_tail = (s_evt_tail + 1) % EVT_QUEUE_SIZE;
    return true;
}

/* ── Relay ───────────────────────────────────────────────────────────────── */
typedef struct { uint8_t channel; bool on; } relay_ctx_t;
static relay_ctx_t s_relay_ctx[RELAY_COUNT]; /* one per possible in-flight write */
static uint8_t     s_relay_ctx_idx = 0;

static void relay_write_cb(int res, uint8_t *buf, void *ctx) {
    relay_ctx_t *c = (relay_ctx_t *)ctx;
    if (res > 0) {
        state_cache_set_relay(c->channel, c->on);
        levt_t e = { .type = LEVT_RELAY, .relay = { c->channel, c->on } };
        push_evt(&e);
    } else {
        printf("relay write ch%u failed (%d)\n", c->channel, res);
    }
}

void lighting_set_relay(uint8_t channel, bool on) {
    relay_ctx_t *c = &s_relay_ctx[s_relay_ctx_idx++ % RELAY_COUNT];
    c->channel = channel;
    c->on      = on;
    modbus_set_coil(g_config.relay_slave_id, channel, on, relay_write_cb, c);
}

/* ── DALI callbacks — no state cache update on write; dirty_poll does that ── */
static void dali_write_cb(int res, uint8_t *buf, void *ctx) {
    /* Nothing needed here — TinyModbus will set the dirty bit, and
     * dirty_poll_tick() will read the actual resulting arc level and emit
     * the state event.  If there's an error we just log it. */
    if (res <= 0) {
        printf("DALI write failed (%d)\n", res);
    }
}

void lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on) {
    if (dali_provision_busy_slave(slave_id)) return;
    uint16_t reg = on ? DALI_REG_GOTO_LAST : DALI_REG_OFF;
    modbus_set_holding_register(slave_id, reg,
                                dali_cmd_addr(dali_addr),
                                dali_write_cb, NULL);
}

void lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up) {
    if (dali_provision_busy_slave(slave_id)) return;
    uint16_t reg = up ? DALI_REG_UP : DALI_REG_DOWN;
    modbus_set_holding_register(slave_id, reg,
                                dali_cmd_addr(dali_addr),
                                dali_write_cb, NULL);
}

void lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc) {
    if (dali_provision_busy_slave(slave_id)) return;
    uint16_t value = (dali_dapc_addr(dali_addr) << 8) | arc;
    modbus_set_holding_register(slave_id, DALI_REG_DAPC,
                                value, dali_write_cb, NULL);
}

/* ── Emit helpers (called by dirty_poll) ─────────────────────────────────── */
void lighting_emit_relay_state(uint8_t channel, bool on) {
    state_cache_set_relay(channel, on);
    levt_t e = { .type = LEVT_RELAY, .relay = { channel, on } };
    push_evt(&e);
}

void lighting_emit_dali_state(uint8_t slave_id, uint8_t addr,
                               uint8_t arc, uint8_t status) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx >= 0) state_cache_set_dali_arc((uint8_t)idx, addr, arc, status);
    levt_t e = {
        .type = LEVT_DALI,
        .dali = { slave_id, addr, arc, status }
    };
    push_evt(&e);
}

void lighting_emit_presence(uint8_t slave_id,
                             uint64_t appeared, uint64_t disappeared) {
    levt_t e = {
        .type     = LEVT_PRESENCE,
        .presence = { slave_id, appeared, disappeared }
    };
    push_evt(&e);
}

void lighting_emit_fault(uint8_t slave_id, uint8_t addr,
                          bool lamp_fail, bool power_cycle) {
    levt_t e = {
        .type  = LEVT_FAULT,
        .fault = { slave_id, addr, lamp_fail, power_cycle }
    };
    push_evt(&e);
}

/* fix #6: scan_complete — bridge uses present_mask to set per-ballast availability */
void lighting_emit_scan_complete(uint8_t slave_id, uint64_t present_mask) {
    levt_t e = {
        .type          = LEVT_SCAN_COMPLETE,
        .scan_complete = { slave_id, present_mask }
    };
    push_evt(&e);
}
