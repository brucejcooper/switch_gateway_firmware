/*
 * web_state.c — core-1 read model for the config web UI. See web_state.h.
 */

#include "web_state.h"
#include "intercore.h"
#include "netif.h"         /* netif_get_ip — live (DHCP-leased) address */
#include "state_cache.h"   /* RELAY_COUNT, DALI_ADDR_COUNT */
#include "mapping.h"       /* MAX_FIXTURES, MAX_BUTTONS, MAX_TARGETS_PER_BTN, TARGET_* */
#include "config.h"        /* MAX_DALI_SLAVES */
#include "version.h"       /* FW_VERSION */
#include <string.h>
#include <stdio.h>

#define WEB_GROUPS 16u     /* DALI groups 0..15 */

/* DALI status byte bits (mirrors state_cache usage). */
#define DALI_ST_LAMP_FAIL 0x02u
#define DALI_ST_FADING    0x10u

typedef struct {
    uint8_t arc, status, min, max, power_on, sys_fail, fade_rate, fade_time;
    bool    present, fading, lamp_fail, seen;
} ws_ballast_t;

typedef struct { bool on, seen; uint8_t bri; uint64_t members; } ws_group_t;

static struct {
    uint32_t relay_known;                 /* bit per channel that has reported */
    bool     relay_on[RELAY_COUNT];
    bool     relay_gw_online[MAX_RELAY_SLAVES];  /* relay board reachability */
    bool     relay_gw_seen[MAX_RELAY_SLAVES];

    ws_ballast_t ball[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
    bool         gw_online[MAX_DALI_SLAVES];
    bool         gw_seen[MAX_DALI_SLAVES];
    ws_group_t   grp[MAX_DALI_SLAVES][WEB_GROUPS];

    struct { uint8_t count; mapping_target_t t[MAX_TARGETS_PER_BTN]; }
        map[MAX_FIXTURES][MAX_BUTTONS];

    struct { uint8_t state, count; bool seen; char msg[IC_PROV_MSG_MAX]; }
        prov[MAX_DALI_SLAVES];

    uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
    uint32_t baud;
    uint8_t  de_mode;
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
    case IC_EVT_PRESENCE: {
        int s = dali_idx(e->presence.slave);
        if (s < 0) break;
        for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
            if (e->presence.appeared & (1ull << a)) {
                S.ball[s][a].seen = true; S.ball[s][a].present = true;
            }
            if (e->presence.disappeared & (1ull << a)) S.ball[s][a].present = false;
        }
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
        S.baud               = e->cfg.baud;
        S.de_mode            = e->cfg.de_mode;
        break;
    case IC_EVT_BUTTON:
        /* Transient UI event; nothing to cache. */
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
    SEC_TAIL,         /* ],"cfg":{...},"seq":N} */
    SEC_DONE,
};

/* DALI sub-phases within one slave object (SEC_DALI). */
enum { DSUB_OPEN = 0, DSUB_BALLASTS, DSUB_GROUPS_OPEN, DSUB_GROUPS, DSUB_CLOSE };

void web_render_begin(web_render_cursor_t *cur) {
    cur->sec = SEC_PRE; cur->si = 0; cur->sub = 0;
    cur->i = 0; cur->started = false; cur->seq = S.seq;
}

/* Highest seen DALI short address / group for a slave index, or -1 if none.
 * Defines the indexed-array length so positions stay stable as devices drop. */
static int ballast_max(int si) {
    int m = -1;
    for (int a = 0; a < DALI_ADDR_COUNT; a++) if (S.ball[si][a].seen) m = a;
    return m;
}
static int group_max(int si) {
    int m = -1;
    for (int g = 0; g < (int)WEB_GROUPS; g++) if (S.grp[si][g].seen) m = g;
    return m;
}

static int slave_id_for(int idx) {
    const ic_snapshot_t *sn = intercore_snapshot();
    return (sn && idx < sn->dali_count) ? sn->dali_slaves[idx] : 0;
}

static int render_members(uint64_t mask, char *out, size_t cap) {
    int n = 0; bool first = true;
    n += snprintf(out + n, cap - n, "[");
    for (uint8_t a = 0; a < DALI_ADDR_COUNT && (size_t)n < cap; a++) {
        if (mask & (1ull << a)) {
            n += snprintf(out + n, cap - n, "%s%u", first ? "" : ",", a);
            first = false;
        }
    }
    n += snprintf(out + n, cap - n, "]");
    return n;
}

/* Build the cfg/net/mqtt tail (no secrets: mqtt_pass and web_pass are omitted). */
static int render_tail(char *out, size_t cap) {
    const ic_snapshot_t *sn = intercore_snapshot();
    int n = 0;
    n += snprintf(out + n, cap - n,
        "],\"cfg\":{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,"
        "\"reversal_window_ms\":%u,\"baud\":%lu,\"de_mode\":%u,"
        "\"fw_version\":\"" FW_VERSION "\","
        "\"max_fixtures\":%u,\"max_buttons\":%u,\"max_targets\":%u",
        S.long_press_ms, S.dim_repeat_ms, S.reversal_window_ms,
        (unsigned long)S.baud, S.de_mode,
        MAX_FIXTURES, MAX_BUTTONS, MAX_TARGETS_PER_BTN);
    if (sn) {
        uint8_t cur[4]; netif_get_ip(cur);   /* live address (DHCP-leased or static) */
        n += snprintf(out + n, cap - n,
            ",\"net\":{\"dhcp\":%s,\"current_ip\":\"%u.%u.%u.%u\","
            "\"ip\":\"%u.%u.%u.%u\",\"mask\":\"%u.%u.%u.%u\","
            "\"gw\":\"%u.%u.%u.%u\",\"dns\":\"%u.%u.%u.%u\"}",
            sn->net_dhcp ? "true" : "false",
            cur[0], cur[1], cur[2], cur[3],
            sn->net_ip[0], sn->net_ip[1], sn->net_ip[2], sn->net_ip[3],
            sn->net_mask[0], sn->net_mask[1], sn->net_mask[2], sn->net_mask[3],
            sn->net_gw[0], sn->net_gw[1], sn->net_gw[2], sn->net_gw[3],
            sn->net_dns[0], sn->net_dns[1], sn->net_dns[2], sn->net_dns[3]);
        /* The broker host is stored as a dotted-quad (config supports IP only for
         * now), resolved into mqtt_ip on the snapshot. The password is never
         * disclosed — only whether one is set. */
        n += snprintf(out + n, cap - n,
            ",\"mqtt\":{\"host\":\"%u.%u.%u.%u\",\"port\":%u,\"user\":\"%.47s\","
            "\"prefix\":\"%.23s\",\"have_pass\":%s}",
            sn->mqtt_ip[0], sn->mqtt_ip[1], sn->mqtt_ip[2], sn->mqtt_ip[3],
            sn->mqtt_port, sn->mqtt_user, sn->mqtt_prefix,
            sn->mqtt_pass[0] ? "true" : "false");
        n += snprintf(out + n, cap - n,
            ",\"web_user\":\"%.31s\"", sn->web_user);
    }
    /* Close the cfg object, then the top-level seq, then the root object. */
    n += snprintf(out + n, cap - n, "},\"seq\":%lu}", (unsigned long)S.seq);
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

    /* relays: booleans, index = channel (up to the highest channel ever seen). */
    case SEC_RELAY: {
        int rmax = -1;
        for (int ch = RELAY_COUNT - 1; ch >= 0; ch--)
            if (S.relay_known & (1u << ch)) { rmax = ch; break; }
        if ((int)cur->i <= rmax) {
            uint16_t ch = cur->i;
            next->i = ch + 1; next->started = true;
            return snprintf(out, cap, "%s%s", cur->started ? "," : "",
                            S.relay_on[ch] ? "true" : "false");
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
            int bmax = ballast_max(si);
            if ((int)cur->i <= bmax) {
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
            int gmax = group_max(si);
            if ((int)cur->i <= gmax) {
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
                    n += snprintf(out + n, cap - n, "%s{\"slave\":%u,\"group\":%u}",
                                  ti ? "," : "", t->slave, TARGET_GROUP_NUM(*t));
                else
                    n += snprintf(out + n, cap - n, "%s{\"slave\":%u,\"addr\":%u}",
                                  ti ? "," : "", t->slave, TARGET_BALLAST_ADDR(*t));
            }
            n += snprintf(out + n, cap - n, "]}");
            next->i = k + 1; next->started = true;
            return n;
        }
        next->sec = SEC_PROV_OPEN; return 0;

    case SEC_PROV:
        for (uint16_t s = cur->i; s < MAX_DALI_SLAVES; s++) {
            if (!S.prov[s].seen) continue;
            next->i = s + 1; next->started = true;
            return snprintf(out, cap, "%s{\"slave\":%d,\"state\":%u,\"count\":%u}",
                cur->started ? "," : "", slave_id_for(s),
                S.prov[s].state, S.prov[s].count);
        }
        next->sec = SEC_TAIL; return 0;
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
