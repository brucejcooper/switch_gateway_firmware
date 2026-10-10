#include "lighting_actions.h"
#include "dali_provision.h"
#include "dali_queue.h"
#include "mbq_client.h"
#include "modbus.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "dali_state.h"
#include "config.h"
#include "mapping.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

/* EVERY DALI frame this file emits leaves by the same door: dq_submit() into
 * the per-slave priority ladder (dali_queue.c), which hands it to the queue-v4
 * client and out as FC 0x41 (QUEUE-PROTOCOL.md §1). DAPC levels, OFF,
 * GOTO_LAST_ACTIVE, UP/DOWN steps, group commands, config DTR0+SET pairs and
 * raw queries are all the same kind of thing here — a (addr, data, flags,
 * dtr0) tuple with a priority and an idempotency flag.
 *
 * There is no register-window path left. Earlier revisions wrote DAPC to a
 * per-ballast brightness bank at FC06 0x0000+addr and pushed everything else
 * through a submit window at 0x1000-0x1007; both are gone, along with the
 * FC10/FC23 frames that carried them. Comments below that mention them are
 * describing what was replaced. */

/* ── State event queue — written here, drained by ic_seam_pump() ──────────── */
/* We use a simple ring buffer rather than the pico queue_t here because all
 * accesses are on core 0 (callbacks from modbus_poll(), which runs on core 0). */
/* Sized to hold a full resync burst without overflow. Worst case for
 * MAX_DALI_SLAVES=2: 32 relays + 2*64 ballasts + 2*16 groups + 2 scan_completes
 * = 194 events. Round up to 256. The events drain on every ic_seam_pump()
 * iteration, so this only needs to absorb a single synchronous burst. */
#define EVT_QUEUE_SIZE 256

/* levt_type_t / levt_t now live in lighting_actions.h — ic_seam.c (formerly
 * serial_protocol.c) needs the real type too (levt_to_ic()), not a hand-copy. */

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

/* Called by ic_seam_pump() to drain events — prototype in lighting_actions.h */
bool lighting_evt_try_pop(levt_t *out) {
    if (s_evt_head == s_evt_tail) return false;
    *out = s_evts[s_evt_tail];
    s_evt_tail = (s_evt_tail + 1) % EVT_QUEUE_SIZE;
    return true;
}

/* ── Relay ───────────────────────────────────────────────────────────────────
 *
 * Same latest-wins rule as DALI (task 14). A relay command from HA/web/CLI sets
 * the channel's desired state and a dirty bit; lighting_relay_tick() drains them
 * with ONE write in flight at a time, so a burst can never overflow the Modbus
 * action queue (measured: 7 of 32 relay writes lost to it) and whatever arrives
 * while a write is out simply folds into the next one.
 *
 * Each write is FC15 over ONE byte-aligned bank of 8 coils — a single data byte.
 * The Waveshare 32CH returns FC01 reads byte-swapped (relay_byteswap), and
 * whether its FC15 data is swapped the same way is undocumented; with one data
 * byte there is no byte order to get wrong, and bit order within a byte is the
 * standard one on this board. A 32-relay burst costs 4 transactions, not 32.
 * Channels in the bank that were not asked for are written with their last
 * commanded value (or the measured one if never commanded); if any is unknown
 * the bank falls back to one FC05 per dirty channel.
 *
 * The cache and HA are still updated only from the write ACK — never from the
 * request. Wall buttons bypass the table (lighting_button_relay). */
#define RELAY_BANKS (RELAY_COUNT / 8u)

static uint32_t s_relay_dirty;          /* channels with a pending desired state */
static uint32_t s_relay_want;           /* their desired state (bit = on)        */
static uint32_t s_relay_cmd;            /* last value written or being written   */
static uint32_t s_relay_cmd_known;      /* ...for these channels                 */
static bool     s_relay_busy;           /* a table-driven write is in flight     */
static uint32_t s_relay_c_written, s_relay_c_overwritten, s_relay_c_writes;

typedef struct { uint32_t mask; uint32_t val; uint32_t asked; } relay_wr_t;
static relay_wr_t s_relay_wr;           /* the in-flight table write             */

typedef struct { uint8_t channel; bool on; } relay_ctx_t;
static relay_ctx_t s_relay_ctx[RELAY_COUNT]; /* one per possible in-flight FC05 */
static uint8_t     s_relay_ctx_idx = 0;

static void relay_ack(uint8_t ch, bool on) {
    state_cache_set_relay(ch, on);
    levt_t e = { .type = LEVT_RELAY, .relay = { ch, on } };
    push_evt(&e);
}

static void relay_write_cb(int res, uint8_t *buf, void *ctx) {
    (void)buf;
    relay_ctx_t *c = (relay_ctx_t *)ctx;
    if (res > 0) relay_ack(c->channel, c->on);
    else LOG_ERR("light", "relay write failed ch=%u (%d)", c->channel, res);
}

/* One coil, FC05 — the button path and the unknown-neighbour fallback. */
static bool relay_fc05(uint8_t channel, bool on, modbus_action_cb_t cb) {
    relay_ctx_t *c = &s_relay_ctx[s_relay_ctx_idx++ % RELAY_COUNT];
    c->channel = channel;
    c->on      = on;
    if (!modbus_set_coil(g_config.relay_slave_ids[0], channel, on, cb, c)) return false;
    s_relay_cmd       = on ? (s_relay_cmd | (1u << channel)) : (s_relay_cmd & ~(1u << channel));
    s_relay_cmd_known |= 1u << channel;
    return true;
}

static void relay_table_fc05_cb(int res, uint8_t *buf, void *ctx) {
    s_relay_busy = false;
    relay_write_cb(res, buf, ctx);
}

static void relay_bank_cb(int res, uint8_t *buf, void *ctx) {
    (void)buf; (void)ctx;
    s_relay_busy = false;
    if (res <= 0) {
        LOG_ERR("light", "relay bank write failed mask=%08lx (%d)",
                (unsigned long)s_relay_wr.mask, res);
        return;
    }
    for (uint8_t ch = 0; ch < RELAY_COUNT; ch++) {
        uint32_t bit = 1u << ch;
        if (!(s_relay_wr.mask & bit)) continue;
        bool on = (s_relay_wr.val & bit) != 0;
        /* Emit the channels that were asked for (HA expects a reply even when
         * nothing changed), and any neighbour whose measured state this write
         * changed; a neighbour re-written with the value it had is not news. */
        if ((s_relay_wr.asked & bit) || !state_cache_relay_known(ch) ||
            state_cache_get_relay_on(ch) != on)
            relay_ack(ch, on);
    }
}

bool lighting_set_relay(uint8_t channel, bool on) {
    if (g_config.relay_slave_count == 0) {
        LOG_WARN("light", "relay write dropped — no relay slave configured");
        return false;
    }
    if (channel >= RELAY_COUNT) return false;
    uint32_t bit = 1u << channel;
    if (s_relay_dirty & bit) s_relay_c_overwritten++;
    s_relay_dirty |= bit;
    s_relay_want = on ? (s_relay_want | bit) : (s_relay_want & ~bit);
    s_relay_c_written++;
    modbus_backoff_kick(g_config.relay_slave_ids[0]);   /* a person is waiting */
    return true;
}

bool lighting_button_relay(uint8_t channel, bool on) {
    if (g_config.relay_slave_count == 0) {
        LOG_WARN("light", "relay write dropped — no relay slave configured");
        return false;
    }
    if (channel >= RELAY_COUNT) return false;
    modbus_backoff_kick(g_config.relay_slave_ids[0]);   /* a person is waiting */
    /* Buttons win: anything pending for this channel is older. A table write
     * already in flight was enqueued first, so this FC05 executes after it. */
    s_relay_dirty &= ~(1u << channel);
    if (!relay_fc05(channel, on, relay_write_cb)) {
        LOG_WARN("light", "button relay ch=%u: action queue full — queued as desired state",
                 channel);
        return lighting_set_relay(channel, on);
    }
    LOG_INFO("light", "button relay ch=%u %s", channel, on ? "on" : "off");
    return true;
}

void lighting_relay_tick(void) {
    if (s_relay_busy || !s_relay_dirty || g_config.relay_slave_count == 0) return;
    uint8_t slave = g_config.relay_slave_ids[0];

    uint8_t bank = 0;
    while (bank < RELAY_BANKS && !((s_relay_dirty >> (bank * 8u)) & 0xFFu)) bank++;
    uint32_t bank_mask = 0xFFu << (bank * 8u);
    uint32_t dirty     = s_relay_dirty & bank_mask;

    /* Fill the channels nobody asked for: last commanded, else measured. */
    uint32_t val = s_relay_want & dirty;
    bool complete = true;
    for (uint8_t ch = (uint8_t)(bank * 8u); ch < bank * 8u + 8u; ch++) {
        uint32_t bit = 1u << ch;
        if (dirty & bit) continue;
        if (s_relay_cmd_known & bit)        { if (s_relay_cmd & bit) val |= bit; }
        else if (state_cache_relay_known(ch)) { if (state_cache_get_relay_on(ch)) val |= bit; }
        else { complete = false; break; }
    }

    if (!complete) {
        /* A neighbour's state is unknown — never guess a coil. One FC05. */
        uint8_t ch = (uint8_t)(bank * 8u);
        while (!(dirty & (1u << ch))) ch++;
        bool on = (s_relay_want >> ch) & 1u;
        if (!relay_fc05(ch, on, relay_table_fc05_cb)) return;   /* action queue full: next tick */
        s_relay_dirty &= ~(1u << ch);
        s_relay_busy = true;
        s_relay_c_writes++;
        return;
    }

    uint8_t byte = (uint8_t)(val >> (bank * 8u));
    if (!modbus_set_coils(slave, (uint16_t)(bank * 8u), 8u, &byte, 1u, relay_bank_cb, NULL))
        return;                                       /* action queue full: next tick */
    s_relay_wr.mask = bank_mask;
    s_relay_wr.val  = val;
    s_relay_wr.asked = dirty;
    s_relay_cmd       = (s_relay_cmd & ~bank_mask) | val;
    s_relay_cmd_known |= bank_mask;
    s_relay_dirty    &= ~dirty;
    s_relay_busy = true;
    s_relay_c_writes++;
}

uint32_t lighting_relay_pending(void) { return s_relay_dirty; }

void lighting_relay_counters(uint32_t *written, uint32_t *overwritten, uint32_t *writes) {
    *written = s_relay_c_written; *overwritten = s_relay_c_overwritten; *writes = s_relay_c_writes;
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
 * The one send primitive in this file. DAPC levels and OFF come through here
 * too — everything is a queued frame now.
 *
 * Each command carries an idempotency flag, which is what the client needs to
 * recover a gateway reboot or a never-landed frame (QUEUE-PROTOCOL.md §4):
 * idempotent commands (DAPC, OFF, GOTO_LAST_ACTIVE, group on/off/level, config
 * re-store, all queries) are resubmitted, while UP/DOWN step frames are NOT
 * (replaying would double-step) and are reported DQ_RES_DROPPED instead.
 *
 * Raw sends are fire-and-forget: no completion callback, because a NAK is the
 * normal outcome of a no-answer send, and dirty_poll/dali_state read back what
 * the ballast actually did. A refusal here means the priority class is full,
 * and the caller logs the drop. */
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
 * Targets are queried sequentially through dq_submit, short-circuiting on the
 * first "on". There is no interleaving hazard from another producer any more:
 * the answer byte is delivered to THIS command's callback, bound to the
 * gateway-assigned seq, so a concurrent HA or CLI send cannot be mistaken for
 * our result. The deadline still bounds the probe; worst case the toggle
 * assumes off, which the next press self-corrects. */

#define PROBE_QUERY_OPCODE      0x93u    /* QUERY LAMP POWER ON */

/* How every any-on probe reads its result — the button's press-time probe, the
 * HA toggle and the CLI all go through here so they cannot drift.
 *
 * QUERY LAMP POWER ON sent to a GROUP is answered by every lit member at once.
 * Their backward frames overlap — each may start anywhere in the 5.5-10.5 ms
 * response window — and the gateway reports the overlap as a collision OR as a
 * bus fault depending on which pulse it tripped on first. 2026-09-30, house
 * group 3.6 with four members on: every press came back BUSFAULT, dq_yes_no()
 * said UNRESOLVED, the button assumed "off" and dimmed UP instead of switching
 * off. For THIS question any disturbance in the response window means someone
 * answered, so DQ_RES_ERR of either kind reads YES. The worst case — a
 * genuinely faulted bus read as "on" — makes a press send OFF, which is
 * harmless. dq_yes_no() itself stays strict (BUSFAULT = unresolved); callers
 * that know a garbled answer is still an answer say so explicitly — this probe
 * and commissioning's COMPARE (dali_commission.c), whose own VERIFY step is the
 * safety net there. */
static dq_yes_no_t probe_reading(dq_result_t res, uint8_t answer) {
    if (res == DQ_RES_ERR) return DQ_YN_YES;
    return dq_yes_no(res, answer);
}
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
 * Safe to submit the follow-up command from inside the completion callback, on
 * EITHER of the two contexts a callback can fire from (mbq_cb_t, in
 * mbq_client.h). Normal path: dq_tick() calls mbq_client_collect() (which
 * fires callbacks) BEFORE its ship loop reads the class rings, and dq_mbq_cb
 * releases the trampoline before invoking the user callback — so the new
 * command lands in a ring the loop has not visited yet, using a trampoline
 * that is already free. Rarer path: a stalled §4 recovery poll giving up
 * inside drive() (outside dq_tick's collect/ship ordering) — still safe
 * because mbq_client_submit() is pure bookkeeping (it stages a command, it
 * does not transmit), so it cannot interleave with the reply that triggered
 * it.
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
    switch (probe_reading(res, answer)) {
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
 * the answer — kept here rather than reimplemented in cli.c so the
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
    switch (probe_reading(res, ans)) {
    case DQ_YN_YES: *any_on = true;  return true;
    case DQ_YN_NO:  *any_on = false; return true;
    default:                     /* dropped / timed out — no definitive answer */
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
        switch (probe_reading(s_probe.res, s_probe.answer)) {
        case DQ_YN_YES:
            probe_finish(true, true, "answered");
            return;
        case DQ_YN_NO:                        /* none on this target — try the next */
            if (++s_probe.tgt_ix >= s_probe.tgt_count)
                probe_finish(true, false, "all nak");
            return;                           /* next target submitted below next tick */
        default:                              /* dropped / lost — unresolved */
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
 * harmless.
 *
 * The five fields are INDEPENDENT — no field's opcode depends on another's
 * answer — so they run as a ROLLING WINDOW of two rather than strictly
 * serially: as one field's result lands the next is submitted, and the collect
 * of the first rides the enqueue of the second instead of costing the transport
 * a bare poll (task 10). Two, not five: the backlog belongs to the priority
 * ladder in dali_queue, and a third field staged here would only sit in front
 * of a button press. Each field carries its own token and its own deadline, and
 * a field that fails or times out is still just one field — field_ok[] already
 * says which answered, and cfgq_commit_ballast keeps the cached value for the
 * rest. Fields complete out of order without consequence: each result is stored
 * under its own index and the ballast is committed when the last one is in. */
#define CFGQ_FIELD_DEADLINE_MS 300  /* bound one field through the queue (round-trips + recovery) */
#define CFGQ_WINDOW            2u   /* fields in flight at once */

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

/* Backing off an unavailable gateway. While a gateway's link is declared down
 * (LINK_DOWN_FAILS consecutive no-replies) its queries cannot drain, so its
 * backlog class fills and every submit is refused. The reader used to retry
 * those refusals every tick — a main-loop spin, ~4000 refusals/s, whose rate-
 * limited WARN still wiped the whole /api/log ring during a 1-minute outage
 * (house, 2026-09-30). Same rule as dali_state's LINK_RETRY_MS: leave a down
 * gateway alone and look again every CFGQ_LINK_RETRY_MS; its pending ballasts
 * stay pending and are read when it answers. A refusal on a HEALTHY link
 * (ordinary backpressure) waits CFGQ_REFUSED_WAIT_MS instead of one tick. */
#define CFGQ_LINK_RETRY_MS    5000u
#define CFGQ_REFUSED_WAIT_MS    20u
static absolute_time_t s_cfgq_hold[MAX_DALI_SLAVES];   /* don't start before this */
static absolute_time_t s_cfgq_refill_at;               /* next submit attempt */

static bool cfgq_link_down(uint8_t slave_id) {
    return mbq_client_consec_fail(slave_id) >= LINK_DOWN_FAILS;
}

/* One field's query while it is outstanding. Per-field rather than one shared
 * slot: two are live at once, and either may be answered or abandoned alone. */
typedef struct {
    uint16_t gen;                     /* token of this field's query; 0 = not live */
    bool     have;                    /* a result landed via the callback */
    dq_result_t res;
    uint8_t  answer;
    absolute_time_t deadline;
} cfgq_field_t;

static struct {
    bool     active;                  /* a per-ballast sequence is running */
    uint8_t  slave_idx;               /* index into g_config.dali_slave_ids */
    uint8_t  slave_id;
    uint8_t  addr;
    uint8_t  next;                    /* next field to submit (CFGQ_NFIELDS = all out) */
    uint8_t  settled;                 /* fields whose outcome is recorded */
    uint8_t  live;                    /* fields submitted and not yet settled */
    uint8_t  vals[CFGQ_NFIELDS];      /* accumulated response bytes */
    bool     field_ok[CFGQ_NFIELDS];  /* which fields answered cleanly */
    cfgq_field_t q[CFGQ_NFIELDS];
} s_cfgq;

/* Completion callback for a queued config query. Runs from dq_tick (main-loop)
 * context; stash for the cfgq tick, guarded by the query token — which names
 * the FIELD as well as the query, so whichever field still expects this token
 * owns the answer. A token no field holds belongs to an abandoned or superseded
 * query and is dropped. */
static void cfgq_result_cb(uint8_t slave, dq_result_t res, uint8_t answer, void *ctx) {
    (void)slave;
    uint16_t g = (uint16_t)(uintptr_t)ctx;
    if (!s_cfgq.active || g == 0) return;
    for (uint8_t f = 0; f < CFGQ_NFIELDS; f++) {
        if (s_cfgq.q[f].gen != g) continue;
        s_cfgq.q[f].res = res; s_cfgq.q[f].answer = answer; s_cfgq.q[f].have = true;
        return;
    }
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
        if (!time_reached(s_cfgq_hold[i])) continue;          /* gateway backed off */
        if (cfgq_link_down(g_config.dali_slave_ids[i])) {
            s_cfgq_hold[i] = make_timeout_time_ms(CFGQ_LINK_RETRY_MS);
            continue;
        }
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

/* Record one field's outcome and retire its query. */
static void cfgq_settle(uint8_t f, bool ok, uint8_t value) {
    s_cfgq.field_ok[f] = ok;
    if (ok) s_cfgq.vals[f] = value;
    s_cfgq.q[f].gen = 0;   /* answered or abandoned: the token is no longer live */
    s_cfgq.live--;
    s_cfgq.settled++;
}

void lighting_cfg_query_tick(void) {

    if (!s_cfgq.active && !cfgq_start_next()) return;

    /* A slave mid-provisioning owns its bus — defer this ballast and retry.
     * Queries still out are abandoned: the callback ignores everything while
     * the sequence is inactive, and cfgq_start_next clears every token before
     * the next ballast begins, so no late answer can reach a live field. */
    if (dali_provision_busy_slave(s_cfgq.slave_id)) {
        lighting_cfg_query_mark(s_cfgq.slave_id, s_cfgq.addr);
        s_cfgq.active = false;
        return;
    }

    /* The gateway stopped answering mid-ballast: put the ballast back and back
     * off the whole gateway (see CFGQ_LINK_RETRY_MS). Outstanding queries are
     * abandoned the same way as above. */
    if (cfgq_link_down(s_cfgq.slave_id)) {
        lighting_cfg_query_mark(s_cfgq.slave_id, s_cfgq.addr);
        s_cfgq_hold[s_cfgq.slave_idx] = make_timeout_time_ms(CFGQ_LINK_RETRY_MS);
        LOG_DBG("light", "cfg-query: slave %u unavailable — backing off %u ms",
                s_cfgq.slave_id, CFGQ_LINK_RETRY_MS);
        s_cfgq.active = false;
        return;
    }

    /* Collect whatever has landed, and abandon anything past its own deadline
     * (keeping that field's cached value). */
    for (uint8_t f = 0; f < CFGQ_NFIELDS; f++) {
        if (s_cfgq.q[f].gen == 0) continue;
        if (s_cfgq.q[f].have)
            cfgq_settle(f, s_cfgq.q[f].res == DQ_RES_OK, s_cfgq.q[f].answer);
        else if (time_reached(s_cfgq.q[f].deadline))
            cfgq_settle(f, false, 0);
    }

    if (s_cfgq.settled >= CFGQ_NFIELDS) { cfgq_commit_ballast(); return; }

    /* Refill the window. A NAK/ERR (or a backlog-full retry that eventually
     * times out) leaves that field un-answered and cfgq_commit_ballast keeps
     * its cached value. */
    if (!time_reached(s_cfgq_refill_at)) return;
    uint8_t addr_byte = (uint8_t)((s_cfgq.addr & 0x3Fu) << 1) | 0x01u;
    while (s_cfgq.live < CFGQ_WINDOW && s_cfgq.next < CFGQ_NFIELDS) {
        uint8_t  f = s_cfgq.next;
        uint16_t g = next_query_gen();
        if (!dq_submit(s_cfgq.slave_id, DQ_PRIO_BACKGROUND, addr_byte, CFGQ_OPCODE[f],
                       DQ_ANSWER, 0, true, cfgq_result_cb, (void *)(uintptr_t)g)) {
            /* backlog full — back off briefly, not every tick */
            s_cfgq_refill_at = make_timeout_time_ms(CFGQ_REFUSED_WAIT_MS);
            break;
        }
        s_cfgq.q[f].gen      = g;
        s_cfgq.q[f].have     = false;
        s_cfgq.q[f].deadline = make_timeout_time_ms(CFGQ_FIELD_DEADLINE_MS);
        s_cfgq.live++;
        s_cfgq.next++;
    }
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

/* ── Absolute commands: latest-wins desired state (task 14) ───────────────────
 *
 * HA, the web UI, the HTTP API, the CLI and the async toggle do not queue frames
 * any more. They overwrite the target's slot in the gateway's pending table and
 * the scheduler drains it as capacity frees up (dali_pending.h, dq_set_desired).
 * A burst used to be dispatched straight into the 8-deep user class and the rest
 * dropped — measured 2026-10-10 on the house, a 107-command no-op burst lost 57
 * of 75 DALI commands and 7 of 32 relays, and nothing told HA. The table cannot
 * overflow: the newest command per target is all that is kept.
 *
 * Nothing here writes the cache. The verification read is marked when each
 * frame actually ships (pending_shipped below), and the measured result is what
 * HA sees.
 *
 * Wall buttons do NOT come through here — see lighting_button_* below. */
static bool desired(uint8_t slave_id, bool is_group, uint8_t target,
                    dp_op_t op, uint8_t arc, const char *what) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "%s %u.%s%u skipped — slave provisioning",
                what, slave_id, is_group ? "G" : "", target);
        return false;
    }
    if (!dq_set_desired(slave_id, is_group, target, op, arc)) return false;
    dirty_poll_mark_active(slave_id);
    return true;
}

/* Ship hook: a pending slot's frame just left for the gateway — read back what
 * it actually did. */
static void pending_shipped(uint8_t slave_id, bool is_group, uint8_t target) {
    if (is_group) dali_state_mark_dirty_group(slave_id, target);
    else          dali_state_mark_dirty(slave_id, target);
}

void lighting_init(void) {
    dq_set_ship_hook(pending_shipped);
}

bool lighting_set_on_off(uint8_t slave_id, uint8_t dali_addr, bool on) {
    return desired(slave_id, false, dali_addr, on ? DP_ON : DP_OFF, 0, on ? "on" : "off");
}

/* ── Wall buttons: straight into the user class — buttons always win ─────────
 *
 * A press ships ahead of every pending desired state (the user class outranks
 * the pending rung, and the pending rung never takes the staging slot), then
 * dq_button_settle() makes sure nothing older still in the table can undo it:
 * the target's own pending slot is dropped, or — if anything else is pending on
 * that gateway — the press is re-asserted as the newest slot. If the user class
 * itself refuses the frame, the press goes into the table instead of being lost.
 */
static bool button_dali(uint8_t slave_id, bool is_group, uint8_t target,
                        dp_op_t op, uint8_t arc) {
    if (dali_provision_busy_slave(slave_id)) {
        LOG_DBG("light", "button %u.%s%u skipped — slave provisioning",
                slave_id, is_group ? "G" : "", target);
        return false;
    }
    dp_slot_t sl = { .op = (uint8_t)op, .arc = arc };
    uint8_t addr, data;
    dp_frame(dp_index(is_group, target), &sl, &addr, &data);
    if (raw_submit(slave_id, DQ_PRIO_USER, addr, data, 0, 0, true)) {
        dq_button_settle(slave_id, is_group, target, op, arc);
        dirty_poll_mark_active(slave_id);
        if (is_group) dali_state_mark_dirty_group(slave_id, target);
        else          dali_state_mark_dirty(slave_id, target);
    } else {
        LOG_WARN("light", "button %u.%s%u: user class full — queued as desired state",
                 slave_id, is_group ? "G" : "", target);
        if (!dq_set_desired(slave_id, is_group, target, op, arc)) return false;
        dirty_poll_mark_active(slave_id);
    }
    LOG_INFO("light", "button %u.%s%u %s", slave_id, is_group ? "G" : "", target,
             op == DP_ON ? "on" : op == DP_OFF ? "off" : "level");
    return true;
}

bool lighting_button_on_off(uint8_t slave_id, uint8_t dali_addr, bool on) {
    return button_dali(slave_id, false, dali_addr, on ? DP_ON : DP_OFF, 0);
}

bool lighting_button_group_on_off(uint8_t slave_id, uint8_t group, bool on) {
    return button_dali(slave_id, true, group, on ? DP_ON : DP_OFF, 0);
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
    /* DAPC. The ballast clamps to its own min/max, so the read-back remains the
     * authority on what actually happened. */
    return desired(slave_id, false, dali_addr, DP_LEVEL, arc, "bri");
}

/* ── DALI group helpers ──────────────────────────────────────────────────── */
/* Same desired-state path; the group's slot is ordered against its members' by
 * write stamp (dali_pending.h), so whichever was asked for last wins. */

bool lighting_group_set_level(uint8_t slave_id, uint8_t group, uint8_t arc) {
    return desired(slave_id, true, group, DP_LEVEL, arc, "group bri");
}

bool lighting_group_set_on_off(uint8_t slave_id, uint8_t group, bool on) {
    return desired(slave_id, true, group, on ? DP_ON : DP_OFF, 0, on ? "group on" : "group off");
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
