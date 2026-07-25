#include "lighting_actions.h"
#include "dali_provision.h"
#include "modbus.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "config.h"
#include "mapping.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

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

/* ── RAW_CMD send queue ──────────────────────────────────────────────────────
 * The gateway has a single raw-command slot: a RAW_CMD write while the
 * previous raw command is still executing on the DALI bus (~25 ms) is rejected
 * with DEVICE_BUSY — and callers legitimately fire several raws back-to-back
 * (a multi-target toggle turning every light on, HA bursts). Without this
 * queue the 2nd..Nth write would land inside the 1st's execution window and be
 * silently dropped. So: FIFO the sends, keep exactly one in flight, pace
 * successive sends by about one DALI frame time, and retry busy/timeout with
 * back-off instead of dropping. */
#define RAWQ_DEPTH        16
#define RAWQ_GAP_MS       25   /* ≈ forward frame (16.7 ms) + bus settling */
#define RAWQ_RETRY_MS     15
#define RAWQ_RETRIES_MAX  8

typedef struct { uint8_t slave; uint16_t value; bool twice; } rawq_ent_t;

static struct {
    rawq_ent_t      q[RAWQ_DEPTH];
    uint8_t         head, count;
    uint8_t         retries;
    bool            in_flight;   /* head's Modbus write awaiting its callback */
    absolute_time_t next_at;     /* pacing gap / retry back-off */
} s_rawq;

static void rawq_pop(void) {
    s_rawq.head    = (uint8_t)((s_rawq.head + 1) % RAWQ_DEPTH);
    s_rawq.count--;
    s_rawq.retries = 0;
    s_rawq.next_at = make_timeout_time_ms(RAWQ_GAP_MS);
}

static void rawq_write_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s_rawq.in_flight = false;
    if (res > 0) {                        /* accepted — gateway is executing it */
        rawq_pop();
        return;
    }
    uint8_t excep = (res < 0 && buf) ? buf[2] : 0;
    bool retryable = (res == 0) || excep == 0x06;   /* timeout or DEVICE_BUSY */
    if (retryable && ++s_rawq.retries <= RAWQ_RETRIES_MAX) {
        s_rawq.next_at = make_timeout_time_ms(RAWQ_RETRY_MS);
    } else {
        const rawq_ent_t *e = &s_rawq.q[s_rawq.head];
        LOG_ERR("light", "raw cmd 0x%04X slave %u dropped (%s)",
                e->value, e->slave,
                res == 0 ? "timeout" : (excep == 0x06 ? "busy" : "error"));
        rawq_pop();
    }
}

/* True while the any-on probe has a query committed to the gateway and hasn't
 * read its terminal result yet — the raw queue must hold off in that window or
 * its command would overwrite the result register the probe is polling. */
static bool probe_query_in_flight(void);
/* Same, for the lowest-priority config-over-raw query SM (task 03): the FIFO
 * holds off while a config query's result is outstanding. */
static bool cfgq_query_in_flight(void);

void lighting_raw_tick(void) {
    if (s_rawq.in_flight || s_rawq.count == 0) return;
    if (probe_query_in_flight() || cfgq_query_in_flight()) return;
    if (!time_reached(s_rawq.next_at)) return;
    const rawq_ent_t *e = &s_rawq.q[s_rawq.head];
    if (modbus_set_holding_register(e->slave,
                                    e->twice ? 0x1001u : 0x1000u,
                                    e->value, rawq_write_cb, NULL))
        s_rawq.in_flight = true;
    else   /* modbus action queue full — try again shortly */
        s_rawq.next_at = make_timeout_time_ms(RAWQ_RETRY_MS);
}

bool dali_raw_arm(uint8_t slave_id, uint16_t value, bool twice) {
    if (s_rawq.count >= RAWQ_DEPTH) {
        LOG_WARN("light", "raw cmd 0x%04X slave %u dropped — raw queue full",
                 value, slave_id);
        return false;
    }
    uint8_t tail = (uint8_t)((s_rawq.head + s_rawq.count) % RAWQ_DEPTH);
    s_rawq.q[tail] = (rawq_ent_t){ .slave = slave_id, .value = value,
                                   .twice = twice };
    s_rawq.count++;
    return true;
}

/* ── Any-on probe ────────────────────────────────────────────────────────────
 * QUERY LAMP POWER ON (0x93): gear that is on answers 0xFF, gear that is off
 * stays silent. Sent to a group address every on-member answers the identical
 * frame, so a clean answer OR a collision-garbled one both mean "≥1 on" and a
 * NAK means "none on" — the ballasts themselves are the ground truth, immune
 * to the group-mask cache going stale across gateway reboots.
 *
 * Targets are queried sequentially through the modbus action queue,
 * short-circuiting on the first "on". Known benign race: another RAW_CMD user
 * (HA / CLI) firing between our write and our result read can garble the
 * result; bounded by the deadline, worst case the toggle assumes off, which
 * the next press self-corrects. */

#define PROBE_RESULT_REG        0x0108u
#define PROBE_QUERY_OPCODE      0x93u    /* QUERY LAMP POWER ON */
#define PROBE_POLL_MS           10       /* result-poll pacing (DALI cmd ~25 ms) */
#define PROBE_WRITE_RETRY_MS    15       /* RAW_CMD busy back-off */
#define PROBE_WRITE_RETRIES_MAX 5
#define PROBE_DEADLINE_MS       600      /* whole probe, all targets */

typedef struct { uint8_t slave; uint8_t addr_byte; } probe_tgt_t;

static struct {
    bool            active;      /* slot owned (running or holding a result) */
    bool            done;
    bool            valid;       /* done with a definitive answer */
    bool            any_on;
    uint8_t         fixture, button;
    probe_tgt_t     tgt[MAX_TARGETS_PER_BTN];
    uint8_t         tgt_count, tgt_ix;
    uint8_t         write_retries;
    enum { PST_SEND, PST_AWAIT_WRITE, PST_POLL_DELAY, PST_AWAIT_READ } st;
    absolute_time_t next_at;     /* pacing for send retries / result polls */
    absolute_time_t started_at;  /* probe start (button press) — for the log */
    absolute_time_t deadline;
    bool            cb_fired;
    int             cb_res;
    uint8_t         cb_excep;    /* modbus exception code when cb_res < 0 */
    uint16_t        cb_word;     /* raw_cmd_result word from the result read */
} s_probe;

static bool probe_query_in_flight(void) {
    return s_probe.active && !s_probe.done && s_probe.st != PST_SEND;
}

static void probe_write_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s_probe.cb_res   = res;
    s_probe.cb_excep = (res < 0 && buf) ? buf[2] : 0;
    s_probe.cb_fired = true;
}

static void probe_read_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s_probe.cb_res  = res;
    /* FC04 1-reg response: slave, fc, bytecount, hi, lo, crc… */
    if (res >= 5) s_probe.cb_word = (uint16_t)((buf[3] << 8) | buf[4]);
    s_probe.cb_fired = true;
}

/* Logged at INFO the moment the outcome is known (i.e. when the gateway's
 * answer arrives, usually mid-hold) — the toggle log at release then shows
 * where the decision came from. Elapsed is measured from the button press. */
static void probe_finish(bool valid, bool any_on, const char *why) {
    s_probe.done   = true;
    s_probe.valid  = valid;
    s_probe.any_on = any_on;
    LOG_INFO("light", "any-on probe F%u.B%u: %s (%s, %lu ms)",
             s_probe.fixture, s_probe.button,
             valid ? (any_on ? "on" : "off") : "unresolved", why,
             (unsigned long)(absolute_time_diff_us(s_probe.started_at,
                                                   get_absolute_time()) / 1000));
}

void lighting_any_on_probe_start(uint8_t fixture, uint8_t button) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    if (!rec) return;
    /* Single slot: keep a probe that's still running for a different button;
     * that button's decision matters more than this one's fallback. */
    if (s_probe.active && !s_probe.done &&
        !(s_probe.fixture == fixture && s_probe.button == button))
        return;

    memset(&s_probe, 0, sizeof s_probe);
    s_probe.fixture = fixture;
    s_probe.button  = button;
    for (int i = 0; i < rec->target_count; i++) {
        const mapping_target_t *t = &rec->targets[i];
        if (slave_kind(t->slave) != SLAVE_KIND_DALI) continue;
        if (dali_provision_busy_slave(t->slave))     continue;
        probe_tgt_t *p = &s_probe.tgt[s_probe.tgt_count++];
        p->slave     = t->slave;
        /* command-form address byte: group 100GGGG1, short address 0AAAAAA1 */
        p->addr_byte = TARGET_IS_GROUP(*t)
                           ? (uint8_t)(0x81u | (TARGET_GROUP_NUM(*t) << 1))
                           : (uint8_t)((TARGET_BALLAST_ADDR(*t) << 1) | 1u);
    }
    if (s_probe.tgt_count == 0) return;   /* nothing to ask (relay-only binding) */

    s_probe.active     = true;
    s_probe.st         = PST_SEND;
    s_probe.started_at = get_absolute_time();
    s_probe.next_at    = s_probe.started_at;
    s_probe.deadline   = make_timeout_time_ms(PROBE_DEADLINE_MS);
}

any_on_probe_res_t lighting_any_on_probe_get(uint8_t fixture, uint8_t button) {
    if (!s_probe.active ||
        s_probe.fixture != fixture || s_probe.button != button)
        return ANY_ON_PROBE_UNKNOWN;
    if (!s_probe.done)   return ANY_ON_PROBE_PENDING;
    if (!s_probe.valid)  return ANY_ON_PROBE_UNKNOWN;
    return s_probe.any_on ? ANY_ON_PROBE_ON : ANY_ON_PROBE_OFF;
}

void lighting_any_on_probe_tick(void) {
    if (!s_probe.active || s_probe.done) return;
    if (time_reached(s_probe.deadline)) {
        probe_finish(false, false, "deadline");
        return;
    }
    const probe_tgt_t *t = &s_probe.tgt[s_probe.tgt_ix];

    switch (s_probe.st) {
    case PST_SEND:
        if (!time_reached(s_probe.next_at)) return;
        /* Let queued raw commands drain first — the gateway's single raw slot
         * and result register can only serve one master at a time. If the
         * queue never drains, the probe deadline aborts us and the click
         * falls back to assume-off (warned). No deadlock: the raw queue only
         * pauses for a probe once the query is committed (not in PST_SEND). */
        if (s_rawq.count || s_rawq.in_flight) return;
        /* A config query already committed to the slot must finish its result
         * read first (it yields to us before committing, so this waits at most
         * one query ~30 ms — well inside the probe deadline). */
        if (cfgq_query_in_flight()) return;
        s_probe.cb_fired = false;
        if (modbus_set_holding_register(t->slave, 0x1000u,
                                        (uint16_t)((t->addr_byte << 8) | PROBE_QUERY_OPCODE),
                                        probe_write_cb, NULL))
            s_probe.st = PST_AWAIT_WRITE;
        else  /* action queue full — retry until the deadline */
            s_probe.next_at = make_timeout_time_ms(PROBE_WRITE_RETRY_MS);
        break;

    case PST_AWAIT_WRITE:
        if (!s_probe.cb_fired) return;
        if (s_probe.cb_res > 0) {
            s_probe.st      = PST_POLL_DELAY;
            s_probe.next_at = make_timeout_time_ms(PROBE_POLL_MS);
        } else if (s_probe.cb_res < 0 && s_probe.cb_excep == 0x06 &&
                   ++s_probe.write_retries <= PROBE_WRITE_RETRIES_MAX) {
            /* gateway busy with a previous raw command — back off and resend */
            s_probe.st      = PST_SEND;
            s_probe.next_at = make_timeout_time_ms(PROBE_WRITE_RETRY_MS);
        } else {
            probe_finish(false, false, "write failed");
        }
        break;

    case PST_POLL_DELAY:
        if (!time_reached(s_probe.next_at)) return;
        s_probe.cb_fired = false;
        if (modbus_get_input_registers(t->slave, PROBE_RESULT_REG, 1,
                                       probe_read_cb, NULL))
            s_probe.st = PST_AWAIT_READ;
        else
            s_probe.next_at = make_timeout_time_ms(PROBE_POLL_MS);
        break;

    case PST_AWAIT_READ:
        if (!s_probe.cb_fired) return;
        if (s_probe.cb_res < 5) {
            probe_finish(false, false, "result read failed");
            return;
        }
        switch (s_probe.cb_word >> 8) {
        case 0xFF:   /* still pending on the gateway */
            s_probe.st      = PST_POLL_DELAY;
            s_probe.next_at = make_timeout_time_ms(PROBE_POLL_MS);
            break;
        case 0x00:   /* clean answer — something is on */
        case 0x02:   /* garbled backward frame = collision of answers — ≥1 on */
            probe_finish(true, true, "answered");
            break;
        case 0x01:   /* NAK: nothing at this target is on — next target */
            if (++s_probe.tgt_ix >= s_probe.tgt_count) {
                probe_finish(true, false, "all nak");
            } else {
                s_probe.write_retries = 0;
                s_probe.st            = PST_SEND;
                s_probe.next_at       = get_absolute_time();
            }
            break;
        default:
            probe_finish(false, false, "unexpected result");
            break;
        }
        break;
    }
}

/* ── Config-over-raw query state machine (task 03, Phase A) ──────────────────
 * Fills the per-ballast config cache with raw DALI queries instead of reading
 * the gateway's BLOCK1-3 banks. Mirrors the any-on probe's write-then-poll
 * against the single raw slot (0x1000 write / 0x0108 result), but at strictly
 * lower priority: it only commits a query when the raw FIFO is drained and no
 * probe is pending, and both the FIFO and the probe wait on a query already in
 * flight (cfgq_query_in_flight). Ballasts to (re)read are held in a per-slave
 * pending bitmap; each ballast runs a 5-query sequence, then the accumulated
 * fields are written to the cache and re-emitted as state_dali.
 *
 * The FIFO (dali_raw_arm) can't carry these: it is fire-and-forget and never
 * reads a backward frame, whereas every config query needs the response byte
 * from 0x0108 — hence this separate SM sharing the same slot. */
#define CFGQ_RESULT_REG        0x0108u   /* raw-cmd result register (as the probe) */
#define CFGQ_POLL_MS           10        /* result-poll pacing (DALI query ~25 ms) */
#define CFGQ_WRITE_RETRY_MS    15        /* RAW_CMD busy back-off */
#define CFGQ_WRITE_RETRIES_MAX 5
#define CFGQ_FIELD_DEADLINE_MS 250       /* per-field guard (write + result polls) */

/* DALI-16 query opcodes (low byte of the forward frame). Verified against
 * IEC 62386-102 and the gateway's TinyModbus/src/dali.h: 0xA1 QUERY MAX LEVEL,
 * 0xA2 QUERY MIN LEVEL (these two sit the IEC way round after the 2026-06-06
 * gateway opcode fix — see controller/CLAUDE.md), 0xA3 QUERY POWER ON LEVEL,
 * 0xA4 QUERY SYSTEM FAILURE LEVEL, 0xA5 QUERY FADE TIME / FADE RATE (returns one
 * byte: high nibble = fade time, low nibble = fade rate). */
enum { CFGQ_MIN = 0, CFGQ_MAX, CFGQ_POWER_ON, CFGQ_SYS_FAIL, CFGQ_FADE, CFGQ_NFIELDS };
static const uint8_t CFGQ_OPCODE[CFGQ_NFIELDS] = {
    0xA2u,  /* MIN */
    0xA1u,  /* MAX */
    0xA3u,  /* POWER ON */
    0xA4u,  /* SYS FAIL */
    0xA5u,  /* FADE TIME / RATE */
};

static uint64_t s_cfgq_pending[MAX_DALI_SLAVES];  /* ballasts awaiting read, by slave index */

static struct {
    bool     active;                  /* a per-ballast sequence is running */
    uint8_t  slave_idx;               /* index into g_config.dali_slave_ids */
    uint8_t  slave_id;
    uint8_t  addr;
    uint8_t  field;                   /* CFGQ_MIN..CFGQ_NFIELDS-1 */
    uint8_t  vals[CFGQ_NFIELDS];      /* accumulated response bytes */
    bool     field_ok[CFGQ_NFIELDS];  /* which fields answered cleanly */
    uint8_t  write_retries;
    enum { CQ_SEND, CQ_AWAIT_WRITE, CQ_POLL_DELAY, CQ_AWAIT_READ } st;
    absolute_time_t next_at;
    absolute_time_t deadline;         /* per-field */
    bool     cb_fired;
    int      cb_res;
    uint8_t  cb_excep;
    uint16_t cb_word;
} s_cfgq;

/* True once a query has been committed to the gateway and its result not yet
 * read — the FIFO and the probe leave the shared result register alone in this
 * window (symmetric with probe_query_in_flight). */
static bool cfgq_query_in_flight(void) {
    return s_cfgq.active && s_cfgq.st != CQ_SEND;
}

void lighting_cfg_query_mark(uint8_t slave_id, uint8_t addr) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0 || addr >= DALI_ADDR_COUNT) return;
    s_cfgq_pending[idx] |= ((uint64_t)1 << addr);
}

void lighting_cfg_query_mark_mask(uint8_t slave_id, uint64_t mask) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    s_cfgq_pending[idx] |= mask;
}

bool lighting_cfg_query_busy(void) {
    if (s_cfgq.active) return true;
    for (int i = 0; i < MAX_DALI_SLAVES; i++)
        if (s_cfgq_pending[i]) return true;
    return false;
}

static void cfgq_write_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s_cfgq.cb_res   = res;
    s_cfgq.cb_excep = (res < 0 && buf) ? buf[2] : 0;
    s_cfgq.cb_fired = true;
}

static void cfgq_read_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s_cfgq.cb_res = res;
    /* FC04 1-reg response: slave, fc, bytecount, hi, lo, crc… */
    if (res >= 5) s_cfgq.cb_word = (uint16_t)((buf[3] << 8) | buf[4]);
    s_cfgq.cb_fired = true;
}

/* Write the accumulated fields into the cache and re-emit state_dali. A field
 * that didn't answer keeps its cached value (a partial read never zeroes good
 * config). Called once all CFGQ_NFIELDS have been attempted for one ballast. */
static void cfgq_commit_ballast(void) {
    uint8_t idx = s_cfgq.slave_idx;
    dali_ballast_state_t *b = state_cache_get_dali(idx, s_cfgq.addr);
    uint8_t mn = s_cfgq.field_ok[CFGQ_MIN]      ? s_cfgq.vals[CFGQ_MIN]      : (b ? b->min_level      : 0);
    uint8_t mx = s_cfgq.field_ok[CFGQ_MAX]      ? s_cfgq.vals[CFGQ_MAX]      : (b ? b->max_level      : 0);
    uint8_t po = s_cfgq.field_ok[CFGQ_POWER_ON] ? s_cfgq.vals[CFGQ_POWER_ON] : (b ? b->power_on_level : 0);
    uint8_t sf = s_cfgq.field_ok[CFGQ_SYS_FAIL] ? s_cfgq.vals[CFGQ_SYS_FAIL] : (b ? b->sys_fail_level : 0);
    uint8_t fade = s_cfgq.vals[CFGQ_FADE];
    uint8_t fr = s_cfgq.field_ok[CFGQ_FADE] ? (uint8_t)(fade & 0x0Fu)        : (b ? b->fade_rate : 0);
    uint8_t ft = s_cfgq.field_ok[CFGQ_FADE] ? (uint8_t)((fade >> 4) & 0x0Fu) : (b ? b->fade_time : 0);

    state_cache_set_dali_config(idx, s_cfgq.addr, mx, mn, po, sf, fr, ft);

    /* Re-emit so HA sees the refreshed config (levt→ic pulls config from the
     * cache). This path only touched config — carry the cached arc/status. */
    b = state_cache_get_dali(idx, s_cfgq.addr);
    lighting_emit_dali_state(s_cfgq.slave_id, s_cfgq.addr,
                             b ? b->arc_level : 0, b ? b->status_byte : 0);
    LOG_DBG("light", "cfg-query %u.%u min=%u max=%u pon=%u sf=%u fr=%u ft=%u",
            s_cfgq.slave_id, s_cfgq.addr, mn, mx, po, sf, fr, ft);
    s_cfgq.active = false;
}

/* Pick the next pending ballast (lowest slave index, lowest address) and start
 * its sequence. Returns false if nothing is pending. */
static bool cfgq_start_next(void) {
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        if (!s_cfgq_pending[i]) continue;
        uint8_t addr = (uint8_t)__builtin_ctzll(s_cfgq_pending[i]);
        s_cfgq_pending[i] &= ~((uint64_t)1 << addr);
        memset(&s_cfgq, 0, sizeof s_cfgq);
        s_cfgq.active    = true;
        s_cfgq.slave_idx = (uint8_t)i;
        s_cfgq.slave_id  = g_config.dali_slave_ids[i];
        s_cfgq.addr      = addr;
        s_cfgq.st        = CQ_SEND;
        s_cfgq.next_at   = get_absolute_time();
        return true;
    }
    return false;
}

/* Record the current field's outcome and advance; commit the ballast when the
 * last field is done. */
static void cfgq_field_done(bool ok, uint8_t value) {
    s_cfgq.field_ok[s_cfgq.field] = ok;
    if (ok) s_cfgq.vals[s_cfgq.field] = value;
    if (++s_cfgq.field >= CFGQ_NFIELDS) {
        cfgq_commit_ballast();
    } else {
        s_cfgq.write_retries = 0;
        s_cfgq.st            = CQ_SEND;
        s_cfgq.next_at       = get_absolute_time();
    }
}

void lighting_cfg_query_tick(void) {
    if (!g_config.dali_raw_cfg) return;

    if (!s_cfgq.active && !cfgq_start_next()) return;

    /* A slave mid-provisioning owns its bus — defer this ballast and retry. */
    if (dali_provision_busy_slave(s_cfgq.slave_id)) {
        lighting_cfg_query_mark(s_cfgq.slave_id, s_cfgq.addr);
        s_cfgq.active = false;
        return;
    }

    switch (s_cfgq.st) {
    case CQ_SEND:
        /* Lowest priority: yield the slot to the raw FIFO and to a pending
         * probe. cfgq_query_in_flight() is false here, so nothing waits on us. */
        if (s_rawq.count || s_rawq.in_flight)  return;
        if (s_probe.active && !s_probe.done)   return;
        if (!time_reached(s_cfgq.next_at))     return;
        {
            uint8_t  addr_byte = (uint8_t)((s_cfgq.addr & 0x3Fu) << 1) | 0x01u;
            uint16_t value     = ((uint16_t)addr_byte << 8) | CFGQ_OPCODE[s_cfgq.field];
            s_cfgq.cb_fired = false;
            if (modbus_set_holding_register(s_cfgq.slave_id, 0x1000u, value,
                                            cfgq_write_cb, NULL)) {
                s_cfgq.st       = CQ_AWAIT_WRITE;
                s_cfgq.deadline = make_timeout_time_ms(CFGQ_FIELD_DEADLINE_MS);
            } else {  /* action queue full — retry shortly */
                s_cfgq.next_at = make_timeout_time_ms(CFGQ_WRITE_RETRY_MS);
            }
        }
        break;

    case CQ_AWAIT_WRITE:
        if (!s_cfgq.cb_fired) {
            if (time_reached(s_cfgq.deadline)) cfgq_field_done(false, 0);
            return;
        }
        if (s_cfgq.cb_res > 0) {
            s_cfgq.st      = CQ_POLL_DELAY;
            s_cfgq.next_at = make_timeout_time_ms(CFGQ_POLL_MS);
        } else if (s_cfgq.cb_res < 0 && s_cfgq.cb_excep == 0x06 &&
                   ++s_cfgq.write_retries <= CFGQ_WRITE_RETRIES_MAX) {
            s_cfgq.st      = CQ_SEND;   /* gateway busy — back off and resend */
            s_cfgq.next_at = make_timeout_time_ms(CFGQ_WRITE_RETRY_MS);
        } else {
            cfgq_field_done(false, 0);  /* write failed — skip this field */
        }
        break;

    case CQ_POLL_DELAY:
        if (time_reached(s_cfgq.deadline)) { cfgq_field_done(false, 0); return; }
        if (!time_reached(s_cfgq.next_at)) return;
        s_cfgq.cb_fired = false;
        if (modbus_get_input_registers(s_cfgq.slave_id, CFGQ_RESULT_REG, 1,
                                       cfgq_read_cb, NULL))
            s_cfgq.st = CQ_AWAIT_READ;
        else
            s_cfgq.next_at = make_timeout_time_ms(CFGQ_POLL_MS);
        break;

    case CQ_AWAIT_READ:
        if (!s_cfgq.cb_fired) {
            if (time_reached(s_cfgq.deadline)) cfgq_field_done(false, 0);
            return;
        }
        if (s_cfgq.cb_res < 5) { cfgq_field_done(false, 0); return; }
        switch (s_cfgq.cb_word >> 8) {
        case 0xFF:   /* still pending on the gateway — keep polling */
            s_cfgq.st      = CQ_POLL_DELAY;
            s_cfgq.next_at = make_timeout_time_ms(CFGQ_POLL_MS);
            break;
        case 0x00:   /* answered — low byte is the queried value */
            cfgq_field_done(true, (uint8_t)(s_cfgq.cb_word & 0xFFu));
            break;
        default:     /* 0x01 NAK / 0x02 error — no value for this field */
            cfgq_field_done(false, 0);
            break;
        }
        break;
    }
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
