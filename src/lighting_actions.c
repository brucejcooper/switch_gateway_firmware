#include "lighting_actions.h"
#include "dali_provision.h"
#include "dali_queue.h"
#include "modbus.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "dali_state.h"
#include "config.h"
#include "mapping.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

/* TinyModbus register addresses:
 *   0x0000 + addr  per-ballast brightness bank (FC06 low byte = DAPC level,
 *                  0 = off, 1–254 proportional, 255 = no change)
 *   0x1000-0x1007  command-queue submit window (FC10/FC23; see dali_queue.c)
 *
 * DAPC and OFF go through the per-ballast brightness bank. Everything else
 * (GOTO_LAST_ACTIVE / UP / DOWN, group commands, config DTR0+SET, raw queries)
 * is submitted through the id-tagged command queue (dali_queue.c). The legacy
 * queue carries every raw command. */

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

/* Per-slave/group debounce cache — last emitted (on, bri, members) per group.
 * Zero-init: all groups start as { on=false, bri=0, members=0 }.
 *
 * `members` is in the comparison because it is part of the published payload.
 * Without it a membership change that happened not to alter on/bri never
 * re-published, so the only thing that ever put a new member list on the broker
 * was an immediate optimistic emit from the setter — publishing the mask we had
 * ASKED for. With members compared here, the QUERY GROUPS read-back is enough. */
static struct { bool on; uint8_t bri; uint64_t members; } s_group_prev[MAX_DALI_SLAVES][16];

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

/* ── Raw DALI commands via the gateway command queue ──────────────────────────
 * Broadcast/group/specialty DALI sends (GOTO_LAST_ACTIVE, UP/DOWN, group
 * on/off/level/step, config DTR0+SET) go through the id-tagged gateway command
 * queue (dali_queue.c) instead of the old single-slot 0x1000/0x0108 FIFO. Each
 * command carries an idempotency flag: idempotent commands (GOTO_LAST_ACTIVE,
 * group on/off/level, config re-store, all queries) may be resubmitted after a
 * transport fault or gateway reboot; UP/DOWN step frames are NOT (replaying
 * would double-step), so the driver drops them on a fault rather than resubmit.
 *
 * Per-ballast OFF and DAPC level do NOT come here — they use the per-ballast
 * brightness bank (0x0000+addr) directly. Raw sends are fire-and-forget (no
 * completion callback: a NAK is the normal outcome of a no-answer send); the
 * driver logs backlog-full drops. Self-describing slots remove the shared-result-register
 * interlocks and settle-gate the single-slot path needed. */
static bool raw_submit(uint8_t slave, dq_prio_t prio, uint8_t addr, uint8_t data,
                       uint8_t flags, uint8_t dtr0, bool idempotent) {
    return dq_submit(slave, prio, addr, data, flags, dtr0, idempotent, NULL, NULL);
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

#define PROBE_QUERY_OPCODE      0x93u    /* QUERY LAMP POWER ON */
#define PROBE_DEADLINE_MS       600      /* whole probe, all targets */

typedef struct { uint8_t slave; uint8_t addr_byte; } probe_tgt_t;

/* Monotonic token stamped on every probe/cfgq queue submission and echoed back
 * as the completion ctx. A callback whose token no longer matches the current
 * expected one is stale (its query was superseded or abandoned) and is ignored,
 * so a late answer can never contaminate a newer probe/field. */
static uint16_t s_query_gen;
static uint16_t next_query_gen(void) {   /* never returns 0 (0 = "no live query") */
    if (++s_query_gen == 0) s_query_gen = 1;
    return s_query_gen;
}

static struct {
    bool            active;      /* slot owned (running or holding a result) */
    bool            done;
    bool            valid;       /* done with a definitive answer */
    bool            any_on;
    uint8_t         fixture, button;
    probe_tgt_t     tgt[MAX_TARGETS_PER_BTN];
    uint8_t         tgt_count, tgt_ix;
    bool            in_flight;   /* a query for tgt_ix is submitted, awaiting result */
    bool            have;        /* a result landed via the callback */
    dq_result_t     res;
    uint8_t         answer;
    uint16_t        exp_gen;     /* token of the in-flight query */
    absolute_time_t started_at;  /* probe start (button press) — for the log */
    absolute_time_t deadline;
} s_probe;

/* Completion callback for a queued QUERY LAMP POWER ON. Runs from dq_tick
 * (main-loop) context; stash for the probe tick, guarded by the query token. */
static void probe_result_cb(uint8_t slave, dq_result_t res, uint8_t answer, void *ctx) {
    (void)slave;
    if (!s_probe.active || s_probe.done) return;
    if ((uint16_t)(uintptr_t)ctx != s_probe.exp_gen) return;   /* stale */
    s_probe.res    = res;
    s_probe.answer = answer;
    s_probe.have   = true;
}

/* Logged at INFO the moment the outcome is known (i.e. when the gateway's
 * answer arrives, usually mid-hold) — the toggle log at release then shows
 * where the decision came from. Elapsed is measured from the button press. */
static void probe_finish(bool valid, bool any_on, const char *why) {
    s_probe.done      = true;
    s_probe.valid     = valid;
    s_probe.any_on    = any_on;
    s_probe.in_flight = false;
    LOG_INFO("light", "any-on probe F%u.B%u: %s (%s, %lu ms)",
             s_probe.fixture, s_probe.button,
             valid ? (any_on ? "on" : "off") : "unresolved", why,
             (unsigned long)(absolute_time_diff_us(s_probe.started_at,
                                                   get_absolute_time()) / 1000));
}

/* ── Asynchronous toggle (intercore path) ────────────────────────────────────
 * The CLI can block on a probe; the intercore path cannot — it runs in the main
 * loop, where spinning the bus would stall Modbus, MQTT and the watchdog feed.
 * So a toggle here becomes: submit the same QUERY LAMP POWER ON through the
 * normal queue, and apply the inverse when the answer arrives.
 *
 * WHO ACTUALLY REACHES THIS: the HTTP API / web UI (/api/dali, /api/group with
 * on:"toggle"), and any client that pushes IC_TRI_TOGGLE. NOT Home Assistant.
 * We publish "schema":"json" light entities, whose command payloads are strictly
 * {"state":"ON"} / {"state":"OFF"}; HA resolves light.toggle ITSELF from the
 * state we publish, and ha_bridge's inbound parser only recognises "ON" anyway.
 * So HA toggle correctness rests on the accuracy of our state topic — i.e. on
 * the verification reads — not on this code.
 *
 * Safe to submit the follow-up command from inside the completion callback:
 * dq_tick() drives mbq_client_tick() (which fires callbacks) BEFORE its ship
 * loop reads the class rings, and dq_mbq_cb releases the trampoline before
 * invoking the user callback — so the new command lands in a ring the loop has
 * not visited yet, using a trampoline that is already free.
 *
 * A slot is held only for the flight of one query. An unresolved probe does
 * NOTHING rather than guessing, and says so — the same rule as the CLI. */
#define TOGGLE_SLOTS 4

static struct {
    bool     busy;
    uint8_t  slave;
    uint8_t  target;     /* short address, or group number */
    bool     is_group;
    uint16_t gen;        /* rejects a late answer from a superseded probe */
} s_tog[TOGGLE_SLOTS];

static void toggle_cb(uint8_t slave, dq_result_t res, uint8_t answer, void *ctx) {
    (void)slave; (void)answer;
    uint32_t tok = (uint32_t)(uintptr_t)ctx;
    uint8_t  ix  = (uint8_t)(tok >> 16);
    if (ix >= TOGGLE_SLOTS) return;
    if (!s_tog[ix].busy || s_tog[ix].gen != (uint16_t)tok) return;   /* stale */

    uint8_t slave_id = s_tog[ix].slave, target = s_tog[ix].target;
    bool    is_group = s_tog[ix].is_group;
    s_tog[ix].busy = false;

    bool any_on;
    switch (dq_yes_no(res, answer)) {
    case DQ_YN_YES: any_on = true;  break;
    case DQ_YN_NO:  any_on = false; break;
    default:
        LOG_WARN("light", "toggle %u.%s%u unresolved — not acting",
                 slave_id, is_group ? "G" : "", target);
        return;
    }
    LOG_INFO("light", "toggle %u.%s%u any_on=%d (probe) -> %s",
             slave_id, is_group ? "G" : "", target, any_on, any_on ? "off" : "on");
    if (is_group) lighting_group_set_on_off(slave_id, target, !any_on);
    else          lighting_set_on_off(slave_id, target, !any_on);
}

bool lighting_toggle_async(uint8_t slave_id, uint8_t target, bool is_group) {
    uint8_t ix = 0xFFu;
    for (uint8_t i = 0; i < TOGGLE_SLOTS; i++) if (!s_tog[i].busy) { ix = i; break; }
    if (ix == 0xFFu) {
        LOG_WARN("light", "toggle %u.%s%u dropped — no probe slot",
                 slave_id, is_group ? "G" : "", target);
        return false;
    }
    uint8_t addr_byte = is_group ? (uint8_t)(0x81u | ((target & 0x0Fu) << 1))
                                 : (uint8_t)(((target & 0x3Fu) << 1) | 0x01u);
    uint16_t g = next_query_gen();
    s_tog[ix].busy     = true;
    s_tog[ix].slave    = slave_id;
    s_tog[ix].target   = target;
    s_tog[ix].is_group = is_group;
    s_tog[ix].gen      = g;
    void *ctx = (void *)(uintptr_t)(((uint32_t)ix << 16) | g);
    if (!dq_submit(slave_id, DQ_PRIO_USER, addr_byte, PROBE_QUERY_OPCODE,
                   DQ_ANSWER, 0, true, toggle_cb, ctx)) {
        s_tog[ix].busy = false;
        LOG_WARN("light", "toggle %u.%s%u dropped — queue full",
                 slave_id, is_group ? "G" : "", target);
        return false;
    }
    return true;
}

/* Synchronous single-address any-on probe, for the TEXT CLI only.
 *
 * Same question the button's press-time probe asks, same opcode, same reading of
 * the answer — kept here rather than reimplemented in serial_protocol.c so the
 * two paths cannot drift on what "on" means. The button's version is async and
 * walks a binding's targets; this one blocks on a single address, which is what
 * a human-typed command wants.
 *
 * BLOCKS (dq_submit_sync spins the bus). Callable from the CLI, never from the
 * main-loop or intercore command paths — see the warning on dq_submit_sync.
 *
 * Returns false if the probe did not resolve (backlog full, deadline, dropped);
 * the caller must then refuse rather than guess, because guessing is exactly
 * what the cached-state toggle did. */
bool lighting_any_on_probe_sync(uint8_t slave_id, uint8_t addr_byte, bool *any_on) {
    dq_result_t res;
    uint8_t     ans;
    if (!dq_submit_sync(slave_id, addr_byte, PROBE_QUERY_OPCODE, DQ_ANSWER, 0,
                        true, PROBE_DEADLINE_MS, &res, &ans))
        return false;
    switch (dq_yes_no(res, ans)) {
    case DQ_YN_YES: *any_on = true;  return true;
    case DQ_YN_NO:  *any_on = false; return true;
    default:                     /* bus fault / dropped — no definitive answer */
        return false;
    }
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
    s_probe.started_at = get_absolute_time();
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

/* Query each target through the gateway command queue, sequentially, short-
 * circuiting on the first "on". Each query is an ordinary expects-answer command
 * with its own id, so there is no contention with the raw path or the config
 * query SM. A clean
 * answer (OK) or a collision-garbled one (ERR — several on-members answering the
 * same group frame at once) both mean "≥1 on"; a NAK means none on this target. */
void lighting_any_on_probe_tick(void) {
    if (!s_probe.active || s_probe.done) return;
    if (time_reached(s_probe.deadline)) {
        probe_finish(false, false, "deadline");
        return;
    }

    /* A query is in flight: act only when its result has landed. */
    if (s_probe.in_flight) {
        if (!s_probe.have) return;
        s_probe.have      = false;
        s_probe.in_flight = false;
        switch (dq_yes_no(s_probe.res, s_probe.answer)) {
        case DQ_YN_YES:
            probe_finish(true, true, "answered");
            return;
        case DQ_YN_NO:                        /* none on this target — try the next */
            if (++s_probe.tgt_ix >= s_probe.tgt_count)
                probe_finish(true, false, "all nak");
            return;                           /* next target submitted below next tick */
        default:                              /* bus fault / dropped — unresolved */
            probe_finish(false, false, "dropped");
            return;
        }
    }

    /* Submit the query for the current target. dq handles pacing/recovery; a
     * backlog-full false just retries next tick until the deadline. */
    const probe_tgt_t *t = &s_probe.tgt[s_probe.tgt_ix];
    uint16_t g = next_query_gen();
    if (dq_submit(t->slave, DQ_PRIO_USER, t->addr_byte, PROBE_QUERY_OPCODE,
                  DQ_ANSWER, 0, true, probe_result_cb, (void *)(uintptr_t)g)) {
        s_probe.exp_gen   = g;
        s_probe.have      = false;
        s_probe.in_flight = true;
    }
}

/* ── Config-over-raw query state machine ──────────────────────────────────────
 * Fills the per-ballast config cache (min/max/power-on/sys-fail/fade) with raw
 * DALI queries — the only config path; the gateway serves no config banks.
 * Ballasts to (re)read are held in a per-slave pending bitmap; each pending
 * ballast runs a 5-query sequence through the gateway command queue, then the
 * accumulated fields are written to the cache and re-emitted as state_dali.
 *
 * Each query is an ordinary expects-answer queue command with its own id, so the
 * queue driver delivers its answer directly (via cfgq_result_cb). There is no
 * shared-result-register contention any more, so the single-slot path's yield
 * interlocks, post-drain settle-gate and first-field discard are all gone. A
 * per-field deadline bounds a query that never resolves (wedged gateway); the
 * query token (s_query_gen) makes a late callback from an abandoned field
 * harmless. */
#define CFGQ_FIELD_DEADLINE_MS 300  /* bound one field through the queue (round-trips + recovery) */

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
    bool     in_flight;               /* the current field's query is submitted */
    bool     have;                    /* its result landed via the callback */
    dq_result_t res;
    uint8_t  answer;
    uint16_t exp_gen;                 /* token of the in-flight field query */
    absolute_time_t deadline;         /* per-field */
} s_cfgq;

/* Completion callback for a queued config query. Runs from dq_tick (main-loop)
 * context; stash for the cfgq tick, guarded by the query token. */
static void cfgq_result_cb(uint8_t slave, dq_result_t res, uint8_t answer, void *ctx) {
    (void)slave;
    if (!s_cfgq.active) return;
    if ((uint16_t)(uintptr_t)ctx != s_cfgq.exp_gen) return;   /* stale */
    s_cfgq.res = res; s_cfgq.answer = answer; s_cfgq.have = true;
}

void lighting_cfg_query_mark(uint8_t slave_id, uint8_t addr) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0 || addr >= DALI_ADDR_COUNT) return;
    s_cfgq_pending[idx] |= ((uint64_t)1 << addr);
}

/* Mark a whole set of ballasts for a raw config read at once — the BOOT SWEEP.
 *
 * The rotation in dirty_poll.c only marks one ballast per DALI_CFG_ROTATE_MS
 * (~14 s), so a cold start would leave HA's config numbers at zero for ~15
 * minutes on a full bus, one ballast filling in at a time. This arms every
 * present ballast the moment the boot scan knows who is there, so the numbers
 * come from the GEAR within seconds.
 *
 * This function existed and was never called; the gap it leaves was instead
 * papered over by adopting our own retained MQTT config back into the cache at
 * boot (IC_CMD_SEED_DALI_CFG, now deleted) — the controller treating itself as a
 * source of truth about the bus. It is the source, not a sink: state comes from
 * reading the devices. */
void lighting_cfg_query_mark_mask(uint8_t slave_id, uint64_t mask) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    s_cfgq_pending[idx] |= mask;
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
        return true;
    }
    return false;
}

/* Record the current field's outcome and advance; commit the ballast when the
 * last field is done. */
static void cfgq_field_done(bool ok, uint8_t value) {
    s_cfgq.field_ok[s_cfgq.field] = ok;
    if (ok) s_cfgq.vals[s_cfgq.field] = value;
    /* Advance; the tick submits the next field's query (or commits the ballast
     * once all CFGQ_NFIELDS have been attempted). */
    if (++s_cfgq.field >= CFGQ_NFIELDS)
        cfgq_commit_ballast();
}

void lighting_cfg_query_tick(void) {

    if (!s_cfgq.active && !cfgq_start_next()) return;

    /* A slave mid-provisioning owns its bus — defer this ballast and retry. */
    if (dali_provision_busy_slave(s_cfgq.slave_id)) {
        lighting_cfg_query_mark(s_cfgq.slave_id, s_cfgq.addr);
        s_cfgq.active = false;
        return;
    }

    /* A field query is in flight: act on its result, or abandon it on the field
     * deadline (bumping the token so the eventual late callback is ignored). */
    if (s_cfgq.in_flight) {
        if (s_cfgq.have) {
            s_cfgq.have = false; s_cfgq.in_flight = false;
            cfgq_field_done(s_cfgq.res == DQ_RES_OK, s_cfgq.answer);
        } else if (time_reached(s_cfgq.deadline)) {
            s_cfgq.in_flight = false;
            s_cfgq.exp_gen   = 0;             /* invalidate a late callback */
            cfgq_field_done(false, 0);        /* keep the cached value for this field */
        }
        return;
    }

    /* Submit the current field's query. A NAK/ERR (or a backlog-full retry that
     * eventually times out) leaves the field un-answered, and cfgq_field_done
     * keeps its cached value. */
    uint8_t  addr_byte = (uint8_t)((s_cfgq.addr & 0x3Fu) << 1) | 0x01u;
    uint16_t g         = next_query_gen();
    if (dq_submit(s_cfgq.slave_id, DQ_PRIO_BACKGROUND, addr_byte, CFGQ_OPCODE[s_cfgq.field],
                  DQ_ANSWER, 0, true, cfgq_result_cb, (void *)(uintptr_t)g)) {
        s_cfgq.exp_gen   = g;
        s_cfgq.have      = false;
        s_cfgq.in_flight = true;
        s_cfgq.deadline  = make_timeout_time_ms(CFGQ_FIELD_DEADLINE_MS);
    }   /* else backlog full — retry next tick */
}

/* ── Config write via a DTR0-load + SET queue entry ──────────────────────────
 * Stores `value` as that field: preload DTR0 with `value` (special command 0xA3)
 * then the field's SET command (short-address command form, sent twice) which
 * stores whatever DTR0 holds. The gateway command queue carries this as ONE
 * indivisible entry — the DQ_DTR0 flag makes it emit 0xA3<value> back-to-back
 * before the SET, so no other DTR-touching frame can interleave (the atomicity
 * one slot carries the pair). SET opcodes
 * verified against IEC 62386-102 / TinyModbus dali.h: MIN 0x2B, MAX 0x2A,
 * POWER ON 0x2D, SYS FAIL 0x2C, FADE RATE 0x2F, FADE TIME 0x2E. */
bool lighting_dali_cfg_write(uint8_t slave_id, uint8_t addr,
                             dali_cfg_field_t field, uint8_t value) {
    static const uint8_t SET_OPCODE[DALI_CFG_FIELD_COUNT] = {
        [DALI_CFG_MIN]       = 0x2Bu,
        [DALI_CFG_MAX]       = 0x2Au,
        [DALI_CFG_POWER_ON]  = 0x2Du,
        [DALI_CFG_SYS_FAIL]  = 0x2Cu,
        [DALI_CFG_FADE_RATE] = 0x2Fu,
        [DALI_CFG_FADE_TIME] = 0x2Eu,
    };
    if (field >= DALI_CFG_FIELD_COUNT) return false;
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u cfg write skipped — slave provisioning",
                slave_id, addr);
        return false;
    }
    uint8_t addr_byte = (uint8_t)((addr & 0x3Fu) << 1) | 0x01u;
    /* One queue entry: DTR0=value preloaded, then SET (twice). Idempotent
     * (re-storing the same value is safe to resubmit). */
    if (!raw_submit(slave_id, DQ_PRIO_CONFIG_WRITE, addr_byte, SET_OPCODE[field],
                    DQ_DTR0 | DQ_TWICE, value, true)) {
        LOG_WARN("light", "dali %u.%u cfg write dropped — queue full",
                 slave_id, addr);
        return false;
    }
    /* Verify + write-through: schedule a raw query-back of this ballast. The
     * config-query SM reads the actual stored value, updates state_cache and
     * re-emits state_dali. */
    lighting_cfg_query_mark(slave_id, addr);
    LOG_INFO("light", "dali %u.%u cfg field=%d set=%u (DTR0+SET queue entry, verify queued)",
             slave_id, addr, (int)field, value);
    return true;
}

/* NOTHING PREDICTS STATE ANY MORE.
 *
 * All three toggle paths ask the gear directly (QUERY LAMP POWER ON) rather
 * than consulting the cache, and the cache is only ever written from a measured
 * read — nothing records intent. A group's on/level are likewise derived from
 * its members' measured levels: on if any member's arc level is non-zero, level
 * the maximum of them. */

bool lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u %s skipped — slave provisioning",
                slave_id, dali_addr, on ? "on" : "off");
        return false;
    }
    bool ok;
    if (on) {
        /* GOTO_LAST_ACTIVE (0x0A) — no per-ballast bank shortcut, use the queue.
         * Address byte: short address, command form (A0=1). Idempotent. */
        uint8_t addr_byte = (uint8_t)((dali_addr & 0x3F) << 1) | 0x01u;
        ok = raw_submit(slave_id, DQ_PRIO_USER, addr_byte, 0x0Au, 0, 0, true);
    } else {
        /* OFF — DAPC level 0 as a queued frame, rather than writing 0 to the
         * gateway's per-ballast brightness bank (FC06 0x0000+addr) and let the
         * gateway and having it synthesise the DAPC: the
         * controller emits the frame itself. Address byte is DAPC form (bit 0
         * clear); idempotent — re-sending level 0 has no cumulative effect. */
        uint8_t dapc_addr = (uint8_t)((dali_addr & 0x3Fu) << 1);
        ok = raw_submit(slave_id, DQ_PRIO_USER, dapc_addr, 0x00u, 0, 0, true);
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
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty(slave_id, dali_addr);   /* read back what actually happened */
    return true;
}

bool lighting_dali_step(uint8_t slave_id, uint8_t dali_addr, bool up) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u step skipped — slave provisioning",
                slave_id, dali_addr);
        return false;
    }
    /* A dim step is an EVENT, not a state, so it cannot be regenerated from the
     * cache — but it also must not be queued as a command: a held button emits
     * repeats faster than DALI can issue them, and the queued tail keeps dimming
     * after release. Instead it accumulates into a per-ballast signed step
     * counter that the scheduler drains one frame at a time (dali_queue.c). Real
     * UP/DOWN frames still go on the wire, so the ballast's own fade rate and
     * min/max clamping govern the result — deliberately NOT a level computed
     * from cache. Steps beyond the clamp are dropped, which is invisible in a
     * ramp. */
    if (!dq_add_delta(slave_id, false, dali_addr, up ? 1 : -1)) {
        LOG_WARN("light", "dali %u.%u step %s dropped — unknown slave",
                 slave_id, dali_addr, up ? "up" : "down");
        return false;
    }
    LOG_INFO("light", "dali %u.%u step %s",
             slave_id, dali_addr, up ? "up" : "down");
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty(slave_id, dali_addr);
    return true;
}

bool lighting_dali_set_level(uint8_t slave_id, uint8_t dali_addr, uint8_t arc) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "dali %u.%u bri skipped — slave provisioning",
                slave_id, dali_addr);
        return false;
    }
    /* DAPC as a queued frame (address byte in DAPC form — bit 0 clear — and the
     * arc level as the data byte). Replaces the gateway's per-ballast brightness
     * bank; the ballast clamps to its own min/max,
     * so the read-back remains the authority on what actually happened.
     * Idempotent: re-sending the same level has no cumulative effect. */
    uint8_t dapc_addr = (uint8_t)((dali_addr & 0x3Fu) << 1);
    if (!raw_submit(slave_id, DQ_PRIO_USER, dapc_addr, arc, 0, 0, true)) {
        LOG_WARN("light", "dali %u.%u bri=%u dropped — queue full",
                 slave_id, dali_addr, arc);
        return false;
    }
    LOG_INFO("light", "dali %u.%u bri=%u", slave_id, dali_addr, arc);
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty(slave_id, dali_addr);
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
    uint8_t addr_byte = (uint8_t)(0x80u | ((group & 0x0Fu) << 1));  /* DAPC, idempotent */
    if (!raw_submit(slave_id, DQ_PRIO_USER, addr_byte, arc, 0, 0, true)) {
        LOG_WARN("light", "group %u.G%u bri=%u dropped — queue full", slave_id, group, arc);
        return false;
    }
    LOG_INFO("light", "group %u.G%u bri=%u", slave_id, group, arc);
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty_group(slave_id, group);
    return true;
}

bool lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "group %u.G%u %s skipped — slave provisioning",
                slave_id, group, on ? "on" : "off");
        return false;
    }
    uint8_t addr_byte = (uint8_t)(0x81u | ((group & 0x0Fu) << 1));  /* command, idempotent */
    uint8_t opcode    = on ? 0x0Au /* GOTO_LAST_ACTIVE */ : 0x00u /* OFF */;
    if (!raw_submit(slave_id, DQ_PRIO_USER, addr_byte, opcode, 0, 0, true)) {
        LOG_WARN("light", "group %u.G%u %s dropped — queue full",
                 slave_id, group, on ? "on" : "off");
        return false;
    }
    LOG_INFO("light", "group %u.G%u %s",
             slave_id, group, on ? "on" : "off");
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty_group(slave_id, group);
    return true;
}

bool lighting_group_step(uint8_t slave_id, uint8_t group, bool up) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "group %u.G%u step skipped — slave provisioning", slave_id, group);
        return false;
    }
    /* Same as the per-ballast step: accumulate into the group's step counter and
     * let the scheduler drain it one frame at a time. */
    if (!dq_add_delta(slave_id, true, group, up ? 1 : -1)) {
        LOG_WARN("light", "group %u.G%u step %s dropped — unknown slave",
                 slave_id, group, up ? "up" : "down");
        return false;
    }
    LOG_INFO("light", "group %u.G%u step %s",
             slave_id, group, up ? "up" : "down");
    dirty_poll_mark_active(slave_id);
    dali_state_mark_dirty_group(slave_id, group);
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

    /* Offer EVERY group to the emit helper, not just the ones currently
     * containing this ballast.
     *
     * Membership-scoping this walk had a hole: when a ballast is REMOVED from a
     * group, the read-back that discovers the removal has already cleared the
     * bit, so the group it just left was skipped and never re-published — a group
     * emptied of its last member kept its stale member list and derived on/bri in
     * HA.
     *
     * Unconditional is also simply correct: this ballast's level can change a
     * group's derived max_bri whether or not it is still a member, and
     * lighting_emit_group_state_if_changed compares (on, bri, members) so
     * nothing is published unless something actually moved. 16 cheap
     * comparisons against one DALI transaction is not a cost worth optimising. */
    if (idx >= 0 && addr < DALI_ADDR_COUNT) {
        for (uint8_t g = 0; g < 16; g++)
            lighting_emit_group_state_if_changed(slave_id, g);
    }
}

void lighting_emit_group_state_if_changed(uint8_t slave_id, uint8_t group) {
    int slave_idx = state_cache_slave_idx(slave_id);
    if (slave_idx < 0) return;
    if (group >= 16) return;

    bool    on_now  = state_cache_dali_group_any_on((uint8_t)slave_idx, group);
    uint8_t bri_now = state_cache_dali_group_max_bri((uint8_t)slave_idx, group);

    uint64_t mem_now = state_cache_get_dali_group((uint8_t)slave_idx, group);
    if (s_group_prev[slave_idx][group].on      == on_now &&
        s_group_prev[slave_idx][group].bri     == bri_now &&
        s_group_prev[slave_idx][group].members == mem_now) {
        return;
    }
    s_group_prev[slave_idx][group].on      = on_now;
    s_group_prev[slave_idx][group].bri     = bri_now;
    s_group_prev[slave_idx][group].members = mem_now;

    levt_t e = {
        .type  = LEVT_GROUP,
        .group = { slave_id, group, bri_now, on_now }
    };
    push_evt(&e);
    LOG_INFO("light", "group %u.G%u state on=%s bri=%u",
             slave_id, group, on_now ? "true" : "false", bri_now);
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

    /* Per-relay state, but ONLY for channels whose value we have actually read.
     * The boot coil sync can fail (it logs "relay sync: read failed" and leaves
     * the cache at its default false); republishing that here would put 32
     * confident "OFF"s on the broker for a board we never heard from. Unknown
     * channels stay silent until a coil read or a write ACK fills them in. */
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        if (!state_cache_relay_known(ch)) continue;
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
