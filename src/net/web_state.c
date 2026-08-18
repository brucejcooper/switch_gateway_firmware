/*
 * web_state.c — core-1 read model for the config web UI. See web_state.h.
 */

#include "web_state.h"
#include "intercore.h"
#include "dali_provision.h"   /* prov_state_name */
#include "netif.h"         /* netif_get_ip — live (DHCP-leased) address */
#include "state_cache.h"   /* RELAY_COUNT, DALI_ADDR_COUNT */
#include "mapping.h"       /* MAX_FIXTURES, MAX_BUTTONS, MAX_TARGETS_PER_BTN, TARGET_* */
#include "config.h"        /* MAX_DALI_SLAVES */
#include "version.h"       /* FW_VERSION */
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "pico/time.h"   /* to_ms_since_boot — button-log ages */

#define WEB_GROUPS 16u     /* DALI groups 0..15 */

/* DALI status byte bits (mirrors state_cache usage). */
#define DALI_ST_LAMP_FAIL 0x02u
#define DALI_ST_FADING    0x10u

typedef struct {
    uint8_t arc, status, min, max, power_on, sys_fail, fade_rate, fade_time;
    bool    present, fading, lamp_fail, seen;
} ws_ballast_t;

typedef struct { bool on, seen; uint8_t bri; uint64_t members; } ws_group_t;

/* Recent button activity, so the Bindings tab can show which index a physical
 * button actually is. A press is instantaneous and the UI polls every ~1.5 s, so
 * showing "current" button state would miss almost every press — we keep a short
 * history instead and let the UI render it. IC_EVT_BUTTON bumps S.seq like any
 * other event, so a press always breaks the poll's 304 and arrives promptly. */
#define WEB_BTN_LOG 12u
typedef struct { uint8_t fixture, button, action; uint32_t t_ms; } ws_btn_t;

static struct {
    uint32_t relay_known;                 /* bit per channel that has reported */
    bool     relay_on[RELAY_COUNT];
    bool     relay_gw_online[MAX_RELAY_SLAVES];  /* relay board reachability */
    bool     relay_gw_seen[MAX_RELAY_SLAVES];

    ws_ballast_t ball[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
    bool         gw_online[MAX_DALI_SLAVES];
    bool         gw_seen[MAX_DALI_SLAVES];
    ws_group_t   grp[MAX_DALI_SLAVES][WEB_GROUPS];

    ws_btn_t     btn[WEB_BTN_LOG];        /* newest at btn_head-1, wrapping */
    uint8_t      btn_head;
    uint8_t      btn_used;

    struct { uint8_t count; mapping_target_t t[MAX_TARGETS_PER_BTN]; }
        map[MAX_FIXTURES][MAX_BUTTONS];

    struct { uint8_t state, count; bool seen; char msg[IC_PROV_MSG_MAX]; }
        prov[MAX_DALI_SLAVES];

    uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
    uint32_t gw_diag_interval_ms;
    uint32_t dali_sweep_interval_ms;
    uint32_t baud;
    uint8_t  de_mode;
    uint8_t  relay_byteswap;
    bool     cfg_seen;

    uint32_t seq;
} S;

void web_state_init(void) {
    memset(&S, 0, sizeof S);
}

uint32_t web_state_seq(void) { return S.seq; }

/* Map a real Modbus slave id to its 0-based DALI index via the launch snapshot. */
static int dali_idx(uint8_t slave_id) {
    const ic_snapshot_t *sn = intercore_snapshot();
    if (!sn) return -1;
    for (int i = 0; i < sn->dali_count && i < MAX_DALI_SLAVES; i++)
        if (sn->dali_slaves[i] == slave_id) return i;
    return -1;
}
static int relay_idx(uint8_t slave_id) {
    const ic_snapshot_t *sn = intercore_snapshot();
    if (!sn) return -1;
    for (int i = 0; i < sn->relay_count && i < MAX_RELAY_SLAVES; i++)
        if (sn->relay_slaves[i] == slave_id) return i;
    return -1;
}

void web_state_on_event(const ic_event_t *e) {
    switch (e->type) {
    case IC_EVT_RELAY:
        if (e->relay.ch < RELAY_COUNT) {
            S.relay_known |= (1u << e->relay.ch);
            S.relay_on[e->relay.ch] = e->relay.on;
        }
        break;
    case IC_EVT_DALI: {
        int s = dali_idx(e->dali.slave);
        if (s < 0 || e->dali.addr >= DALI_ADDR_COUNT) break;
        ws_ballast_t *b = &S.ball[s][e->dali.addr];
        b->seen      = true;
        b->arc       = e->dali.arc;
        b->status    = e->dali.status;
        b->fading    = (e->dali.status & DALI_ST_FADING) != 0;
        b->lamp_fail = (e->dali.status & DALI_ST_LAMP_FAIL) != 0;
        b->min = e->dali.min; b->max = e->dali.max;
        b->power_on = e->dali.power_on; b->sys_fail = e->dali.sys_fail;
        b->fade_rate = e->dali.fade_rate; b->fade_time = e->dali.fade_time;
        break;
    }
    case IC_EVT_GROUP: {
        int s = dali_idx(e->group.slave);
        if (s < 0 || e->group.group >= WEB_GROUPS) break;
        ws_group_t *g = &S.grp[s][e->group.group];
        g->seen = true; g->on = e->group.on; g->bri = e->group.bri;
        g->members = e->group.members;
        break;
    }
    case IC_EVT_GATEWAY: {
        /* Gateway availability fires for both DALI gateways and relay boards. */
        int s = dali_idx(e->gateway.slave);
        if (s >= 0) { S.gw_seen[s] = true; S.gw_online[s] = e->gateway.online; break; }
        int r = relay_idx(e->gateway.slave);
        if (r >= 0) { S.relay_gw_seen[r] = true; S.relay_gw_online[r] = e->gateway.online; }
        break;
    }
    case IC_EVT_SCAN_COMPLETE: {
        int s = dali_idx(e->scan_complete.slave);
        if (s < 0) break;
        for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
            bool p = (e->scan_complete.present_mask & (1ull << a)) != 0;
            S.ball[s][a].present = p;
            if (p) S.ball[s][a].seen = true;
        }
        break;
    }
    case IC_EVT_FAULT: {
        int s = dali_idx(e->fault.slave);
        if (s < 0 || e->fault.addr >= DALI_ADDR_COUNT) break;
        S.ball[s][e->fault.addr].lamp_fail = e->fault.lamp_fail;
        break;
    }
    case IC_EVT_MAPPING:
        if (e->mapping.fixture < MAX_FIXTURES && e->mapping.button < MAX_BUTTONS) {
            uint8_t c = e->mapping.count;
            if (c > MAX_TARGETS_PER_BTN) c = MAX_TARGETS_PER_BTN;
            S.map[e->mapping.fixture][e->mapping.button].count = c;
            memcpy(S.map[e->mapping.fixture][e->mapping.button].t,
                   e->mapping.targets, (size_t)c * sizeof(mapping_target_t));
        }
        break;
    case IC_EVT_PROVISION: {
        int s = dali_idx(e->provision.slave);
        if (s < 0) break;
        S.prov[s].seen = true;
        S.prov[s].state = e->provision.state;
        S.prov[s].count = e->provision.count;
        memcpy(S.prov[s].msg, e->provision.msg, IC_PROV_MSG_MAX);
        S.prov[s].msg[IC_PROV_MSG_MAX - 1] = '\0';
        break;
    }
    case IC_EVT_CFG:
        S.cfg_seen = true;
        S.long_press_ms      = e->cfg.long_press_ms;
        S.dim_repeat_ms      = e->cfg.dim_repeat_ms;
        S.reversal_window_ms = e->cfg.reversal_window_ms;
        S.gw_diag_interval_ms = e->cfg.gw_diag_interval_ms;
        S.dali_sweep_interval_ms = e->cfg.dali_sweep_interval_ms;
        S.baud               = e->cfg.baud;
        S.de_mode            = e->cfg.de_mode;
        S.relay_byteswap     = e->cfg.relay_byteswap;
        break;
    case IC_EVT_BUTTON:
        /* Not state — a log. Overwrites oldest; the UI only ever wants "what did
         * I just press", so a short ring is enough and cannot grow unbounded. */
        S.btn[S.btn_head] = (ws_btn_t){
            .fixture = e->button.fixture,
            .button  = e->button.button,
            .action  = (uint8_t)e->button.action,
            .t_ms    = to_ms_since_boot(get_absolute_time()),
        };
        S.btn_head = (uint8_t)((S.btn_head + 1u) % WEB_BTN_LOG);
        if (S.btn_used < WEB_BTN_LOG) S.btn_used++;
        break;
    case IC_EVT_PING_REQ:
        /* Routed to ping_submit in the drain; never reaches the read model. */
        break;
    case IC_EVT_GW_DIAG:
        /* Gateway sysinfo is published to HA only; the web UI doesn't surface it. */
        break;
    case IC_EVT_OTA_ACK:
    case IC_EVT_OTA_STATUS:
        /* Routed to ota_on_event in the drain; never reaches the read model. */
        break;
    case IC_EVT_TOMBSTONE:
        /* Retires a removed device's MQTT topics only; the web read model keeps
         * no per-device retained state to clear. */
        break;
    }
    S.seq++;
}

/* ── JSON renderer ──────────────────────────────────────────────────────────
 * Sections alternate between fixed literals (which also close the previous
 * array) and item streams. unit_at() produces the next single unit and the
 * cursor that follows it, WITHOUT mutating the live cursor, so the caller can
 * decline a unit that won't fit and resume it next chunk. */

enum {
    SEC_PRE = 0,      /* {"relays":[ */
    SEC_RELAY,        /* relay booleans, index = channel */
    SEC_RB_OPEN,      /* ],"relay_boards":[ */
    SEC_RB,           /* {slave,online} per relay board */
    SEC_DALI_OPEN,    /* ],"dali":{ */
    SEC_DALI,         /* per-slave: "<id>":{online,ballasts[],groups[]} (uses si/sub/i) */
    SEC_MAP_OPEN,     /* },"mappings":[ */
    SEC_MAP,          /* mapping objects */
    SEC_PROV_OPEN,    /* ],"provision":[ */
    SEC_PROV,         /* provision objects */
    SEC_BTN_OPEN,     /* ],"buttons":[ */
    SEC_BTN,          /* recent button events, newest first */
    SEC_TAIL,         /* ],"cfg":{...},"seq":N} */
    SEC_DONE,
};

/* DALI sub-phases within one slave object (SEC_DALI). */
enum { DSUB_OPEN = 0, DSUB_BALLASTS, DSUB_GROUPS_OPEN, DSUB_GROUPS, DSUB_CLOSE };

void web_render_begin(web_render_cursor_t *cur) {
    cur->sec = SEC_PRE; cur->si = 0; cur->sub = 0;
    cur->i = 0; cur->started = false; cur->seq = S.seq;
}

/* Map a DALI slave INDEX (position in the configured list) to its slave id. */
static int slave_id_for(int idx) {
    const ic_snapshot_t *sn = intercore_snapshot();
    return (sn && idx < sn->dali_count) ? sn->dali_slaves[idx] : 0;
}

/* Clamped accumulating snprintf. Returns the new offset, never past cap, so a
 * truncation can't make (cap - n) underflow to a huge size_t on the next call
 * and run snprintf off the end of out. Behaves identically to a bare
 * `n += snprintf(out + n, cap - n, ...)` whenever the write fits (the only case
 * that occurs today); the clamp is a latent-overflow guard for larger payloads. */
static int jcat(char *out, size_t cap, int n, const char *fmt, ...) {
    if (n < 0 || (size_t)n >= cap) return (int)cap;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(out + n, cap - n, fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    n += w;
    return (size_t)n > cap ? (int)cap : n;
}

static int render_members(uint64_t mask, char *out, size_t cap) {
    int n = 0; bool first = true;
    n = jcat(out, cap, n, "[");
    for (uint8_t a = 0; a < DALI_ADDR_COUNT && (size_t)n < cap; a++) {
        if (mask & (1ull << a)) {
            n = jcat(out, cap, n, "%s%u", first ? "" : ",", a);
            first = false;
        }
    }
    n = jcat(out, cap, n, "]");
    return n;
}

/* Build the cfg/net/mqtt tail (no secrets: mqtt_pass and web_pass_hash/web_salt are omitted). */
static int render_tail(char *out, size_t cap) {
    const ic_snapshot_t *sn = intercore_snapshot();
    int n = 0;
    n = jcat(out, cap, n,
        "],\"cfg\":{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,"
        "\"reversal_window_ms\":%u,\"gw_diag_interval_ms\":%lu,"
        "\"dali_sweep_interval_ms\":%lu,"
        "\"baud\":%lu,\"de_mode\":%u,\"relay_byteswap\":%u,"
        "\"fw_version\":\"" FW_VERSION "\","
        "\"max_fixtures\":%u,\"max_buttons\":%u,\"max_targets\":%u",
        S.long_press_ms, S.dim_repeat_ms, S.reversal_window_ms,
        (unsigned long)S.gw_diag_interval_ms,
        (unsigned long)S.dali_sweep_interval_ms,
        (unsigned long)S.baud, S.de_mode, S.relay_byteswap,
        MAX_FIXTURES, MAX_BUTTONS, MAX_TARGETS_PER_BTN);
    if (sn) {
        uint8_t cur[4]; netif_get_ip(cur);   /* live address (DHCP-leased or static) */
        n = jcat(out, cap, n,
            ",\"net\":{\"dhcp\":%s,\"current_ip\":\"%u.%u.%u.%u\","
            "\"ip\":\"%u.%u.%u.%u\",\"mask\":\"%u.%u.%u.%u\","
            "\"gw\":\"%u.%u.%u.%u\",\"dns\":\"%u.%u.%u.%u\","
            "\"hostname\":\"%.31s\"}",
            sn->net_dhcp ? "true" : "false",
            cur[0], cur[1], cur[2], cur[3],
            sn->net_ip[0], sn->net_ip[1], sn->net_ip[2], sn->net_ip[3],
            sn->net_mask[0], sn->net_mask[1], sn->net_mask[2], sn->net_mask[3],
            sn->net_gw[0], sn->net_gw[1], sn->net_gw[2], sn->net_gw[3],
            sn->net_dns[0], sn->net_dns[1], sn->net_dns[2], sn->net_dns[3],
            sn->net_hostname);
        /* The broker host is stored as a dotted-quad (config supports IP only for
         * now), resolved into mqtt_ip on the snapshot. The password is never
         * disclosed — only whether one is set. */
        n = jcat(out, cap, n,
            ",\"mqtt\":{\"host\":\"%u.%u.%u.%u\",\"port\":%u,\"user\":\"%.47s\","
            "\"prefix\":\"%.23s\",\"have_pass\":%s}",
            sn->mqtt_ip[0], sn->mqtt_ip[1], sn->mqtt_ip[2], sn->mqtt_ip[3],
            sn->mqtt_port, sn->mqtt_user, sn->mqtt_prefix,
            sn->mqtt_pass[0] ? "true" : "false");
        n = jcat(out, cap, n,
            ",\"web_user\":\"%.31s\"", sn->web_user);
    }
    /* Close the cfg object, then the top-level seq, then the root object. */
    n = jcat(out, cap, n, "},\"seq\":%lu}", (unsigned long)S.seq);
    return n;
}

/* Emit the unit at *cur into out; write the following cursor to *next.
 * Returns the unit length (0 for an empty item-section step that just advances). */
static int unit_at(const web_render_cursor_t *cur, char *out, size_t cap,
                   web_render_cursor_t *next) {
    const ic_snapshot_t *sn = intercore_snapshot();
    int dali_count = sn ? sn->dali_count : 0;
    *next = *cur;
    switch (cur->sec) {
    case SEC_PRE:
        next->sec = SEC_RELAY; next->i = 0; next->started = false;
        return snprintf(out, cap, "{\"relays\":[");
    case SEC_RB_OPEN:
        next->sec = SEC_RB; next->i = 0; next->started = false;
        return snprintf(out, cap, "],\"relay_boards\":[");
    case SEC_RB: {
        int rc = sn ? sn->relay_count : 0;
        if ((int)cur->i < rc) {
            int i = cur->i;
            next->i = i + 1; next->started = true;
            return snprintf(out, cap, "%s{\"slave\":%u,\"online\":%s}",
                cur->started ? "," : "", sn->relay_slaves[i],
                S.relay_gw_online[i] ? "true" : "false");
        }
        next->sec = SEC_DALI_OPEN; return 0;
    }
    case SEC_DALI_OPEN:
        next->sec = SEC_DALI; next->si = 0; next->sub = DSUB_OPEN;
        next->i = 0; next->started = false;
        return snprintf(out, cap, "],\"dali\":{");
    case SEC_MAP_OPEN:
        next->sec = SEC_MAP; next->i = 0; next->started = false;
        return snprintf(out, cap, "},\"mappings\":[");   /* } closes the dali object */
    case SEC_PROV_OPEN:
        next->sec = SEC_PROV; next->i = 0; next->started = false;
        return snprintf(out, cap, "],\"provision\":[");
    case SEC_TAIL:
        next->sec = SEC_DONE;
        return render_tail(out, cap);

    /* relays: index = channel, ALWAYS RELAY_COUNT entries — the board's channel
     * count is fixed and known, so the array's length should not depend on what
     * we happen to have heard from.
     *
     * null = never read. A boolean cannot distinguish that from "off", and the
     * cached default must never be presented as a measurement — unread is not
     * off, the same rule the CLI and MQTT paths follow. */
    case SEC_RELAY: {
        if (cur->i < RELAY_COUNT) {
            uint16_t ch = cur->i;
            next->i = ch + 1; next->started = true;
            const char *v = !(S.relay_known & (1u << ch)) ? "null"
                          : S.relay_on[ch] ? "true" : "false";
            return snprintf(out, cap, "%s%s", cur->started ? "," : "", v);
        }
        next->sec = SEC_RB_OPEN; return 0;
    }

    /* dali: { "<slave>": { online, ballasts[], groups[] }, ... }
     * ballasts indexed by short address, groups by group number; an unavailable
     * slot is null so positions stay stable. */
    case SEC_DALI: {
        if (cur->si >= dali_count) { next->sec = SEC_MAP_OPEN; return 0; }
        int si = cur->si;
        switch (cur->sub) {
        case DSUB_OPEN:
            next->sub = DSUB_BALLASTS; next->i = 0; next->started = false;
            return snprintf(out, cap, "%s\"%d\":{\"online\":%s,\"ballasts\":[",
                cur->si ? "," : "", slave_id_for(si),
                S.gw_online[si] ? "true" : "false");
        case DSUB_BALLASTS: {
            /* ALL 64 short addresses, not up to the highest one that answered.
             * A DALI bus has addresses 0-63 whether or not gear sits at them,
             * and which of those to SHOW is a presentation choice — the UI has
             * a toggle for it. The wire format reports the address space as it
             * is; unseen addresses serialise as null below.
             *
             * Same truncation that hid groups 10-15 and the unread relay
             * channels: an array cut at its high-water mark cannot express
             * "nothing here", only "nothing here yet". */
            if (cur->i < DALI_ADDR_COUNT) {
                int a = cur->i;
                ws_ballast_t *b = &S.ball[si][a];
                next->i = a + 1; next->started = true;
                const char *comma = cur->started ? "," : "";
                if (!b->present) return snprintf(out, cap, "%snull", comma);
                return snprintf(out, cap,
                    "%s{\"on\":%s,\"bri\":%u,\"min\":%u,\"max\":%u,\"power_on\":%u,"
                    "\"sys_fail\":%u,\"fade_rate\":%u,\"fade_time\":%u,"
                    "\"fading\":%s,\"lamp_fail\":%s}",
                    comma, b->arc > 0 ? "true" : "false", b->arc, b->min, b->max,
                    b->power_on, b->sys_fail, b->fade_rate, b->fade_time,
                    b->fading ? "true" : "false", b->lamp_fail ? "true" : "false");
            }
            next->sub = DSUB_GROUPS_OPEN; return 0;
        }
        case DSUB_GROUPS_OPEN:
            next->sub = DSUB_GROUPS; next->i = 0; next->started = false;
            return snprintf(out, cap, "],\"groups\":[");
        case DSUB_GROUPS: {
            /* ALWAYS all WEB_GROUPS slots, never truncated at the highest one
             * seen. A DALI bus has 16 groups, always — there is no create step,
             * you make a group by adding members — so a UI that lists only the
             * populated ones offers no way to build a new one. Emitting up to
             * group_max() meant a bus using G0/G1/G7/G9 served exactly 10
             * entries, and groups 10-15 could never be reached.
             *
             * Unseen slots serialise as null (below), which the UI decodes to an
             * empty group rather than dropping — same rule as the relay array. */
            if (cur->i < WEB_GROUPS) {
                int g = cur->i;
                ws_group_t *gp = &S.grp[si][g];
                next->i = g + 1; next->started = true;
                const char *comma = cur->started ? "," : "";
                if (!gp->seen) return snprintf(out, cap, "%snull", comma);
                char mem[8 + DALI_ADDR_COUNT * 4];
                render_members(gp->members, mem, sizeof mem);
                return snprintf(out, cap, "%s{\"on\":%s,\"bri\":%u,\"members\":%s}",
                    comma, gp->on ? "true" : "false", gp->bri, mem);
            }
            next->sub = DSUB_CLOSE; return 0;
        }
        case DSUB_CLOSE:
            next->si = cur->si + 1; next->sub = DSUB_OPEN; next->started = false;
            return snprintf(out, cap, "]}");   /* close groups[] + slave object */
        }
        next->sec = SEC_MAP_OPEN; return 0;
    }

    case SEC_MAP:
        for (uint16_t k = cur->i; k < MAX_FIXTURES * MAX_BUTTONS; k++) {
            int f = k / MAX_BUTTONS, btn = k % MAX_BUTTONS;
            uint8_t cnt = S.map[f][btn].count;
            if (cnt == 0) continue;
            int n = snprintf(out, cap, "%s{\"fixture\":%d,\"button\":%d,\"targets\":[",
                             cur->started ? "," : "", f, btn);
            for (uint8_t ti = 0; ti < cnt; ti++) {
                mapping_target_t *t = &S.map[f][btn].t[ti];
                if (TARGET_IS_GROUP(*t))
                    n = jcat(out, cap, n, "%s{\"slave\":%u,\"group\":%u}",
                             ti ? "," : "", t->slave, TARGET_GROUP_NUM(*t));
                else
                    n = jcat(out, cap, n, "%s{\"slave\":%u,\"addr\":%u}",
                             ti ? "," : "", t->slave, TARGET_BALLAST_ADDR(*t));
            }
            n = jcat(out, cap, n, "]}");
            next->i = k + 1; next->started = true;
            return n;
        }
        next->sec = SEC_PROV_OPEN; return 0;

    case SEC_PROV:
        for (uint16_t s = cur->i; s < MAX_DALI_SLAVES; s++) {
            if (!S.prov[s].seen) continue;
            next->i = s + 1; next->started = true;
            /* state kept numeric for compatibility, plus the label and the
             * human-readable message. The message was already stored here and
             * simply never serialised — it is the part that says WHY a run
             * failed ("Error: no ballasts found"), which a bare 11 does not. */
            /* Escape for JSON: the message is device-authored but does carry
             * quotes (e.g. addresses in prose), and an unescaped one would make
             * the whole /api/state document unparseable. */
            char em[IC_PROV_MSG_MAX * 2];
            {
                const char *src = S.prov[s].msg;
                size_t o = 0;
                for (; *src && o + 2 < sizeof em; src++) {
                    if (*src == '"' || *src == '\\') em[o++] = '\\';
                    else if ((unsigned char)*src < 0x20) continue;   /* drop controls */
                    em[o++] = *src;
                }
                em[o] = '\0';
            }
            return snprintf(out, cap,
                "%s{\"slave\":%d,\"state\":%u,\"state_text\":\"%s\","
                "\"count\":%u,\"msg\":\"%s\"}",
                cur->started ? "," : "", slave_id_for(s),
                S.prov[s].state, prov_state_name(S.prov[s].state),
                S.prov[s].count, em);
        }
        next->sec = SEC_BTN_OPEN; return 0;

    case SEC_BTN_OPEN:
        next->sec = SEC_BTN; next->i = 0; next->started = false;
        return snprintf(out, cap, "],\"buttons\":[");

    /* Newest first, so the UI can highlight [0] without scanning. `age_ms` is
     * resolved at render time rather than shipping a device timestamp the client
     * would have to reconcile against its own clock; the client adds whatever has
     * elapsed since the fetch. */
    case SEC_BTN: {
        static const char *const act[] = { "press", "release", "click", "long_press",
                                           "long_press_repeat", "stuck", "unstuck" };
        if (cur->i < S.btn_used) {
            uint8_t k = (uint8_t)((S.btn_head + WEB_BTN_LOG - 1u - cur->i) % WEB_BTN_LOG);
            const ws_btn_t *b = &S.btn[k];
            next->i = cur->i + 1; next->started = true;
            return snprintf(out, cap,
                "%s{\"fixture\":%u,\"button\":%u,\"action\":\"%s\",\"age_ms\":%lu}",
                cur->started ? "," : "", b->fixture, b->button,
                act[b->action <= 6 ? b->action : 0],
                (unsigned long)(to_ms_since_boot(get_absolute_time()) - b->t_ms));
        }
        next->sec = SEC_TAIL; return 0;
    }
    }
    next->sec = SEC_DONE;
    return 0;
}

size_t web_render_next(web_render_cursor_t *cur, char *buf, size_t cap, bool *done) {
    size_t n = 0;
    *done = false;
    if (cap < WEB_STATE_MIN_CHUNK) return 0;
    for (;;) {
        if (cur->sec >= SEC_DONE) { *done = true; break; }
        char unit[768];
        web_render_cursor_t nx;
        int len = unit_at(cur, unit, sizeof unit, &nx);
        if (len < 0) len = 0;
        if ((size_t)len >= sizeof unit) len = (int)sizeof unit - 1;  /* snprintf clamp */
        if ((size_t)len > cap - n) break;   /* won't fit; resume next chunk */
        if (len) { memcpy(buf + n, unit, (size_t)len); n += (size_t)len; }
        *cur = nx;
    }
    return n;
}
