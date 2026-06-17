#include "lighting_actions.h"
#include "dali_provision.h"
#include "modbus.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "config.h"
#include "log.h"

/* TinyModbus holding register addresses (post-refactor):
 *   0x0000 + addr  per-ballast brightness bank (FC06 low byte = DAPC level,
 *                  0 = off, 1–254 proportional, 255 = no change)
 *   0x1000         RAW_CMD — single DALI send, value = (addr_byte<<8) | opcode
 *
 * The action registers 0x1002–0x100A no longer exist. GOTO_LAST_ACTIVE / UP /
 * DOWN are issued via 0x1000 with a short-address command-form address byte
 * (A0=1). DAPC and OFF go through the per-ballast brightness bank, which arms
 * the same async work selector as raw sends. */

/* ── State event queue — written here, drained by serial_poll() ──────────── */
/* We use a simple ring buffer rather than the pico queue_t here because all
 * accesses are on core 0 (callbacks from modbus_poll(), which runs on core 0). */
/* Sized to hold a full resync burst without overflow. Worst case for
 * MAX_DALI_SLAVES=2: 32 relays + 2*64 ballasts + 2*16 groups + 2 scan_completes
 * = 194 events. Round up to 256. The events drain on every serial_poll()
 * iteration, so this only needs to absorb a single synchronous burst. */
#define EVT_QUEUE_SIZE 256

typedef enum {
    LEVT_RELAY, LEVT_DALI, LEVT_PRESENCE, LEVT_FAULT,
    LEVT_SCAN_COMPLETE, LEVT_GROUP, LEVT_GATEWAY, LEVT_GW_DIAG
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
        struct { uint8_t slave_id; uint8_t group;
                 uint8_t bri; bool on; }                      group;
        struct { uint8_t slave_id; bool online; }            gateway;
        struct { uint8_t slave_id; uint32_t uptime_s;
                 uint16_t boot_count; uint8_t reset_reason; } gw_diag;
    };
} levt_t;

/* Per-slave/group debounce cache — last emitted (on, bri) for each group.
 * Zero-init: all groups start as { on=false, bri=0 }. */
static struct { bool on; uint8_t bri; } s_group_prev[MAX_DALI_SLAVES][16];

static levt_t  s_evts[EVT_QUEUE_SIZE];
static uint8_t s_evt_head = 0;
static uint8_t s_evt_tail = 0;

static void push_evt(const levt_t *e) {
    uint8_t next = (s_evt_head + 1) % EVT_QUEUE_SIZE;
    if (next == s_evt_tail) {
        LOG_ERR("light", "event queue full — dropped");
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
        LOG_ERR("light", "relay write failed ch=%u (%d)", c->channel, res);
    }
}

bool lighting_set_relay(uint8_t channel, bool on) {
    if (g_config.relay_slave_count == 0) {
        LOG_WARN("light", "relay write dropped — no relay slave configured");
        return false;
    }
    relay_ctx_t *c = &s_relay_ctx[s_relay_ctx_idx++ % RELAY_COUNT];
    c->channel = channel;
    c->on      = on;
    if (!modbus_set_coil(g_config.relay_slave_ids[0], channel, on, relay_write_cb, c)) {
        LOG_WARN("light", "relay ch=%u %s dropped — queue full", channel, on ? "on" : "off");
        return false;
    }
    LOG_INFO("light", "relay ch=%u %s", channel, on ? "on" : "off");
    return true;
}

bool lighting_relay_coil_on(const uint8_t *coil_bytes, uint8_t byte_count, uint8_t ch) {
    uint8_t bi = ch / 8u;
    if (bi >= byte_count) return false;
    /* Waveshare 32CH returns the coil-data bytes in reversed order; mirror the
     * byte index so channel `ch` reads from the byte the spec puts it in. Bit
     * position within the byte is unchanged on this board. */
    if (g_config.relay_byteswap) bi = (uint8_t)(byte_count - 1u - bi);
    return (coil_bytes[bi] >> (ch % 8u)) & 1u;
}

/* ── DALI callbacks — no state cache update on write; dirty_poll does that ── */
static void dali_write_cb(int res, uint8_t *buf, void *ctx) {
    /* Nothing needed here — TinyModbus will set the dirty bit, and
     * dirty_poll_tick() will read the actual resulting arc level and emit
     * the state event.  If there's an error we just log it. */
    if (res <= 0) {
        LOG_ERR("light", "DALI write failed (%d)", res);
    }
}

bool dali_raw_arm(uint8_t slave_id, uint16_t value, bool twice) {
    return modbus_set_holding_register(slave_id,
                                       twice ? 0x1001u : 0x1000u,
                                       value, dali_write_cb, NULL);
}

/* Synchronously record the on/off intent so a follow-up toggle (e.g. a rapid
 * button press) reads the commanded state immediately, before — and regardless
 * of — the async dirty-poll refresh.  The dirty-poll re-syncs the intent from
 * the next *stable* read, so mid-fade reads can't clobber an in-flight toggle. */
static void cache_predict_dali(uint8_t slave_id, uint8_t dali_addr, bool on) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx >= 0) state_cache_set_dali_commanded_on((uint8_t)idx, dali_addr, on);
}

static void cache_predict_group(uint8_t slave_id, uint8_t group, bool on) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    uint64_t mask = state_cache_get_dali_group((uint8_t)idx, group);
    while (mask) {
        uint8_t a = (uint8_t)__builtin_ctzll(mask);
        mask &= mask - 1;
        state_cache_set_dali_commanded_on((uint8_t)idx, a, on);
    }
}

bool lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u %s skipped — slave provisioning",
                slave_id, dali_addr, on ? "on" : "off");
        return false;
    }
    bool ok;
    if (on) {
        /* GOTO_LAST_ACTIVE (0x0A) — no per-ballast bank shortcut, use RAW_CMD.
         * Address byte: short address, command form (A0=1). */
        uint8_t  addr_byte = (uint8_t)((dali_addr & 0x3F) << 1) | 0x01u;
        uint16_t value     = ((uint16_t)addr_byte << 8) | 0x0Au;
        ok = dali_raw_arm(slave_id, value, false);
    } else {
        /* OFF — write 0 to the per-ballast brightness register. High byte is
         * ignored by the gateway on write. */
        uint16_t reg = 0x0000u + (uint16_t)dali_addr;
        ok = modbus_set_holding_register(slave_id, reg,
                                         0x0000u, dali_write_cb, NULL);
    }
    if (!ok) {
        LOG_WARN("light", "dali %u.%u %s dropped — queue full",
                 slave_id, dali_addr, on ? "on" : "off");
        return false;
    }
    LOG_INFO("light", "dali %u.%u %s",
             slave_id, dali_addr, on ? "on" : "off");
    /* Reflect the change in the cache immediately so a rapid follow-up toggle
     * reads the new state, then mark the slave active so dirty-poll confirms the
     * exact level.  Gated on a successful enqueue so a dropped write can't leave
     * the cache claiming a state the hardware never received. */
    cache_predict_dali(slave_id, dali_addr, on);
    dirty_poll_mark_active(slave_id);
    return true;
}

bool lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u step skipped — slave provisioning",
                slave_id, dali_addr);
        return false;
    }
    /* UP (0x01) / DOWN (0x02) via RAW_CMD with short-address command-form. */
    uint8_t  addr_byte = (uint8_t)((dali_addr & 0x3F) << 1) | 0x01u;
    uint16_t value     = ((uint16_t)addr_byte << 8) | (up ? 0x01u : 0x02u);
    if (!dali_raw_arm(slave_id, value, false)) {
        LOG_WARN("light", "dali %u.%u step %s dropped — queue full",
                 slave_id, dali_addr, up ? "up" : "down");
        return false;
    }
    LOG_INFO("light", "dali %u.%u step %s",
             slave_id, dali_addr, up ? "up" : "down");
    dirty_poll_mark_active(slave_id);
    return true;
}

bool lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u bri skipped — slave provisioning",
                slave_id, dali_addr);
        return false;
    }
    /* DAPC via per-ballast brightness bank. Low byte = arc level; high byte
     * ignored on write. */
    uint16_t reg = 0x0000u + (uint16_t)dali_addr;
    if (!modbus_set_holding_register(slave_id, reg,
                                     (uint16_t)arc, dali_write_cb, NULL)) {
        LOG_WARN("light", "dali %u.%u bri=%u dropped — queue full",
                 slave_id, dali_addr, arc);
        return false;
    }
    LOG_INFO("light", "dali %u.%u bri=%u", slave_id, dali_addr, arc);
    cache_predict_dali(slave_id, dali_addr, arc > 0);
    dirty_poll_mark_active(slave_id);
    return true;
}

/* ── DALI group helpers ──────────────────────────────────────────────────── */
/* Group address byte: bit 7 = 1 (group), bits 5..1 = group# (0-15) << 1,
 * bit 0 = 0 → DAPC form, bit 0 = 1 → command form. All sends use RAW_CMD
 * (FC06 0x1000); per-ballast brightness bank shortcut isn't applicable. */

bool lighting_group_set_level(uint8_t slave_id, uint8_t group, uint8_t arc) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "group %u.G%u bri skipped — slave provisioning", slave_id, group);
        return false;
    }
    uint8_t  addr_byte = (uint8_t)(0x80u | ((group & 0x0Fu) << 1));  /* DAPC */
    uint16_t value     = ((uint16_t)addr_byte << 8) | arc;
    if (!dali_raw_arm(slave_id, value, false)) {
        LOG_WARN("light", "group %u.G%u bri=%u dropped — queue full", slave_id, group, arc);
        return false;
    }
    LOG_INFO("light", "group %u.G%u bri=%u", slave_id, group, arc);
    cache_predict_group(slave_id, group, arc > 0);
    dirty_poll_mark_active(slave_id);
    return true;
}

bool lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "group %u.G%u %s skipped — slave provisioning",
                slave_id, group, on ? "on" : "off");
        return false;
    }
    uint8_t  addr_byte = (uint8_t)(0x81u | ((group & 0x0Fu) << 1));  /* command */
    uint8_t  opcode    = on ? 0x0Au /* GOTO_LAST_ACTIVE */ : 0x00u /* OFF */;
    uint16_t value     = ((uint16_t)addr_byte << 8) | opcode;
    if (!dali_raw_arm(slave_id, value, false)) {
        LOG_WARN("light", "group %u.G%u %s dropped — queue full",
                 slave_id, group, on ? "on" : "off");
        return false;
    }
    LOG_INFO("light", "group %u.G%u %s",
             slave_id, group, on ? "on" : "off");
    cache_predict_group(slave_id, group, on);
    dirty_poll_mark_active(slave_id);
    return true;
}

bool lighting_group_step(uint8_t slave_id, uint8_t group, bool up) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "group %u.G%u step skipped — slave provisioning", slave_id, group);
        return false;
    }
    uint8_t  addr_byte = (uint8_t)(0x81u | ((group & 0x0Fu) << 1));  /* command */
    uint16_t value     = ((uint16_t)addr_byte << 8) | (up ? 0x01u : 0x02u);
    if (!dali_raw_arm(slave_id, value, false)) {
        LOG_WARN("light", "group %u.G%u step %s dropped — queue full",
                 slave_id, group, up ? "up" : "down");
        return false;
    }
    LOG_INFO("light", "group %u.G%u step %s",
             slave_id, group, up ? "up" : "down");
    dirty_poll_mark_active(slave_id);
    return true;
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
    LOG_DBG("light", "emit dali %u.%u arc=%u status=0x%02x",
            slave_id, addr, arc, status);

    /* Walk every group containing this ballast and let the group emit helper
     * decide whether to publish a derived state_group event. */
    if (idx >= 0 && addr < DALI_ADDR_COUNT) {
        uint64_t bit = ((uint64_t)1) << addr;
        for (uint8_t g = 0; g < 16; g++) {
            if (state_cache_get_dali_group((uint8_t)idx, g) & bit) {
                lighting_emit_group_state_if_changed(slave_id, g);
            }
        }
    }
}

void lighting_emit_group_state_if_changed(uint8_t slave_id, uint8_t group) {
    int slave_idx = state_cache_slave_idx(slave_id);
    if (slave_idx < 0) return;
    if (group >= 16) return;

    bool    on_now  = state_cache_dali_group_any_on((uint8_t)slave_idx, group);
    uint8_t bri_now = state_cache_dali_group_max_bri((uint8_t)slave_idx, group);

    if (s_group_prev[slave_idx][group].on  == on_now &&
        s_group_prev[slave_idx][group].bri == bri_now) {
        return;
    }
    s_group_prev[slave_idx][group].on  = on_now;
    s_group_prev[slave_idx][group].bri = bri_now;

    levt_t e = {
        .type  = LEVT_GROUP,
        .group = { slave_id, group, bri_now, on_now }
    };
    push_evt(&e);
    LOG_INFO("light", "group %u.G%u state on=%s bri=%u",
             slave_id, group, on_now ? "true" : "false", bri_now);
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

void lighting_emit_gateway_state(uint8_t slave_id, bool online) {
    levt_t e = {
        .type    = LEVT_GATEWAY,
        .gateway = { slave_id, online }
    };
    push_evt(&e);
}

void lighting_emit_gw_diag(uint8_t slave_id, uint32_t uptime_s,
                           uint16_t boot_count, uint8_t reset_reason) {
    levt_t e = {
        .type    = LEVT_GW_DIAG,
        .gw_diag = { slave_id, uptime_s, boot_count, reset_reason }
    };
    push_evt(&e);
}

/* ── Resync ──────────────────────────────────────────────────────────────────
 * Push the full cached snapshot through the same event ring buffer that the
 * dirty-poll machinery uses, so a freshly-connected (or restarted) bridge can
 * recover the entire device state without waiting for the next change.
 *
 * No Modbus traffic, no DALI bus activity — every value is pulled from the
 * in-memory cache. The caller is responsible for serial_tx_hello() and
 * serial_tx_config_full() (mapping serialisation goes straight to printf
 * rather than through the event ring, so it lives in the JSON/text handler).
 */
void lighting_resync_emit(void) {
    LOG_INFO("light", "resync emit");

    /* Re-assert each gateway's current reachability. Core 1 re-seeds every
     * gateway-availability topic "online" during its post-connect discovery;
     * reachability is a one-shot transition in dirty_poll, so without replaying
     * it here an already-offline relay board / DALI gateway would show online
     * forever (relays have no per-channel presence to fall back on). */
    dirty_poll_emit_gateway_states();

    /* Per-relay state from cache */
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        levt_t e = { .type = LEVT_RELAY,
                     .relay = { .ch = ch,
                                .on = state_cache_get_relay_on(ch) } };
        push_evt(&e);
    }

    /* Per-DALI slave: per-ballast state, force-fire group state, scan_complete */
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        uint8_t sid = g_config.dali_slave_ids[i];
        uint64_t present_mask = 0;

        for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
            dali_ballast_state_t *b =
                state_cache_get_dali((uint8_t)i, a);
            if (!b || !b->present) continue;
            present_mask |= (uint64_t)1 << a;
            levt_t e = { .type = LEVT_DALI,
                         .dali = { sid, a,
                                   b->arc_level,
                                   b->status_byte } };
            push_evt(&e);
        }

        /* Force-fire group state by inverting the cached prev for this slave,
         * then walking groups. The next call to
         * lighting_emit_group_state_if_changed sees prev != current and
         * pushes a LEVT_GROUP. Empty groups are skipped. */
        for (uint8_t g = 0; g < 16; g++) {
            uint64_t members = state_cache_get_dali_group((uint8_t)i, g);
            if (!members) continue;
            /* Invert the cached prev so the comparison guarantees a push. */
            s_group_prev[i][g].on  = !state_cache_dali_group_any_on((uint8_t)i, g);
            s_group_prev[i][g].bri = state_cache_dali_group_max_bri((uint8_t)i, g) ^ 0xFF;
            lighting_emit_group_state_if_changed(sid, g);
        }

        lighting_emit_scan_complete(sid, present_mask);
    }
}
