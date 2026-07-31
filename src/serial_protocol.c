/*
 * serial_protocol.c — text CLI over USB CDC (core 0).
 *
 * Human-facing, line-based command interface (`help` lists the verbs). JSON
 * lives only on core 1's MQTT edge — this seam pushes typed events to core 1
 * and dispatches typed commands back, never JSON. Uses
 * stdio_getchar_timeout_us(0) to non-blockingly read USB CDC input; outbound
 * messages use printf(). Runs on core 0; no multicore sync needed here.
 */

#include "serial_protocol.h"
#include "dali_provision.h"
#include "lighting_actions.h"
#include "dali_queue.h"
#include "dq_soak.h"
#include "dali_state.h"
#include "mapping.h"
#include "button_fsm.h"
#include "config.h"
#include "state_cache.h"
#include "button_actions.h"
#include "net/ha_bridge.h"
#include "dirty_poll.h"
#include "bus_discovery.h"
#include "modbus.h"
#include "intercore.h"
#include "core1.h"
#include "ota_stage.h"
#include "log.h"
#include "cdc_out.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <hardware/watchdog.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <sys/stat.h>
#include <dirent.h>

#include "version.h"   /* FW_VERSION — generated from git at build time */

/* Consume levt_t events from lighting_actions */
extern bool lighting_evt_try_pop(void *out);  /* defined in lighting_actions.c */

/* When true, serial_poll() echoes received characters back to the terminal
 * and honours backspace for line editing.  On by default: the CLI is now a
 * human-only interface (machine clients went away when the on-board core-1
 * bridge replaced the Python host), so local echo is what you almost always
 * want.  Toggle with "echo off" if driving it from something that echoes
 * locally or wants a clean byte stream. */
static bool echo_on = true;

/* ── Minimal JSON key extraction (identical to pico_lighting design) ─────── */
static bool json_str(const char *json, const char *key, char *out, int out_len) {
    char needle[64];
    snprintf(needle, sizeof(needle), "\"%s\"", key);
    const char *kp = strstr(json, needle);
    if (!kp) return false;
    kp += strlen(needle);
    while (*kp == ' ' || *kp == ':') kp++;
    if (*kp == '"') {
        kp++;
        int i = 0;
        while (*kp && *kp != '"' && i < out_len - 1) out[i++] = *kp++;
        out[i] = '\0';
        return true;
    }
    int i = 0;
    while (*kp && *kp != ',' && *kp != '}' && *kp != ']' && i < out_len - 1)
        out[i++] = *kp++;
    out[i] = '\0';
    return i > 0;
}
/* Forward decls — defined alongside the other DALI helpers further down. */
static int  dali_cfg_set(uint8_t slave, uint8_t addr, const char *field, uint8_t value);
static int  dali_apply_kv(uint8_t slave, uint8_t addr, const char *key, const char *val);
static void emit_state_dali(uint8_t slave, uint8_t addr);
static void emit_state_relay(uint8_t ch);
static void emit_state_group(uint8_t slave, uint8_t group);
static void emit_state_cfg(void);
static void dali_group_set_members(uint8_t slave, uint8_t group, uint64_t new_mask);

static bool json_int(const char *j, const char *k, int *out) {
    char buf[16];
    if (!json_str(j, k, buf, sizeof(buf))) return false;
    *out = atoi(buf);
    return true;
}
static bool json_bool(const char *j, const char *k, bool *out) {
    char buf[8];
    if (!json_str(j, k, buf, sizeof(buf))) return false;
    *out = strcmp(buf, "true") == 0 || strcmp(buf, "1") == 0;
    return true;
}

/* Parse a JSON int array `"key":[a,b,c]` into out[]. Returns the count, or -1
 * if the key is absent (so an omitted array is left unchanged by the caller). */
static int json_uint8_array(const char *line, const char *key, uint8_t *out, int max) {
    char needle[32];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const char *p = strstr(line, needle);
    if (!p) return -1;
    p += strlen(needle);
    while (*p == ' ' || *p == ':') p++;
    if (*p != '[') return -1;
    p++;
    int n = 0;
    while (*p && *p != ']') {
        while (*p == ' ' || *p == ',') p++;
        if (*p < '0' || *p > '9') break;
        if (n < max) out[n++] = (uint8_t)atoi(p);
        while (*p && *p != ',' && *p != ']') p++;
    }
    return n;
}

/* Parse a CLI comma list "1,2,3" into out[]. Returns the count. */
static int parse_csv_u8(const char *s, uint8_t *out, int max) {
    int n = 0;
    while (*s && n < max) {
        while (*s == ',') s++;
        if (*s < '0' || *s > '9') break;
        out[n++] = (uint8_t)atoi(s);
        while (*s && *s != ',') s++;
    }
    return n;
}

/* Parse `"targets":[{...},{...}]` out of `line`, populating `out` with up
 * to `max` valid entries. Invalid target objects are skipped (matching the
 * legacy set_mapping behaviour). Returns the count of accepted targets. */
static uint8_t parse_targets(const char *line, mapping_target_t *out, uint8_t max) {
    uint8_t count = 0;
    const char *tp = strstr(line, "\"targets\"");
    if (!tp) return 0;
    const char *p = tp;
    while (count < max && (p = strchr(p, '{')) != NULL) {
        const char *end = strchr(p, '}');
        if (!end) break;
        char obj[64];
        size_t len = (size_t)(end - p + 1);
        if (len < sizeof(obj)) {
            memcpy(obj, p, len);
            obj[len] = '\0';
            int slave = 0, addr = 0, group = 0;
            bool has_slave = json_int(obj, "slave", &slave);
            bool has_addr  = json_int(obj, "addr",  &addr);
            bool has_group = json_int(obj, "group", &group);
            if (has_slave && slave > 0 && slave <= 247 &&
                (has_addr || has_group)) {
                out[count].slave = (uint8_t)slave;
                bool is_relay = slave_kind((uint8_t)slave) == SLAVE_KIND_RELAY;
                if (has_group && !is_relay) {
                    if (group >= 0 && group <= 15) {
                        out[count].addr = TARGET_MAKE_GROUP((uint8_t)group);
                        count++;
                    }
                } else if (has_addr) {
                    out[count].addr = (uint8_t)addr;
                    count++;
                }
            }
        }
        p = end + 1;
    }
    return count;
}

/* Emit a config_mapping line for one (fixture, button). Empty `targets`
 * (count == 0) clears the bridge's retained MQTT entry for that button. */
static void emit_config_mapping(uint8_t fixture, uint8_t button,
                                const mapping_target_t *targets, uint8_t count) {
    /* Forward the binding to core 1 (the MQTT bridge) as a typed struct. */
    ic_event_t ev = { .type = IC_EVT_MAPPING };
    ev.mapping.fixture = fixture;
    ev.mapping.button  = button;
    ev.mapping.count   = count;
    for (int i = 0; i < count && i < MAX_TARGETS_PER_BTN; i++)
        ev.mapping.targets[i] = targets[i];
    intercore_push_event(&ev);
}

/* ──────────────────────────────────────────────────────────────────────────
 *  Text command interface — human-friendly CLI over the same USB CDC line.
 *  Lines that don't start with '{' land here. Output is plain `key=value`
 *  text, never starting with '{', so the bridge ignores it harmlessly.
 * ────────────────────────────────────────────────────────────────────────── */

/* Forward declaration so dispatcher can reach it */
static void handle_text_line(const char *original_line);

/* Bridge dispatcher (the new handle_line) */
static void handle_line(const char *line) {
    /* skip leading whitespace */
    while (*line == ' ' || *line == '\t') line++;
    if (*line == '\0') return;

    /* USB CDC is a human text CLI only — JSON is gone (core 1 owns the MQTT
     * wire). Ignore any stray '{' line rather than parse it. */
    if (*line == '{') return;
    handle_text_line(line);
}

/* ── helpers ────────────────────────────────────────────────────────────── */

/* Print an err: <reason> line and return. Reason is a short token. */
static void txt_err(const char *reason) {
    printf("err: %s\n", reason);
}

static void txt_ok(void) {
    printf("ok\n");
}

/* Parse a boolean-ish on/off value — case-insensitive. Returns 1=on, 0=off,
 * 2=toggle, -1 on parse error.
 *
 * Accepts every spelling the CLI and the JSON API actually use: the documented
 * grammar is `on=true` (README, `relay 0 on=true`, `dali 2 1 on=toggle`) while
 * `echo on|off` uses the bare words, and HA sends JSON true/false. Accepting
 * only on/off/toggle here silently broke `dali <s> g<n> on=true` when this
 * helper was first reused for groups. */
static int parse_on_off_toggle(const char *s) {
    if (!s) return -1;
    if (strcasecmp(s, "on")   == 0 || strcasecmp(s, "true")  == 0 || strcmp(s, "1") == 0) return 1;
    if (strcasecmp(s, "off")  == 0 || strcasecmp(s, "false") == 0 || strcmp(s, "0") == 0) return 0;
    if (strcasecmp(s, "toggle") == 0) return 2;
    return -1;
}

/* Parse a decimal int in [lo, hi]. Returns true on success. */
static bool parse_int_range(const char *s, int lo, int hi, int *out) {
    if (!s || !*s) return false;
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0') return false;
    if (v < lo || v > hi) return false;
    *out = (int)v;
    return true;
}

/* Parse int in [lo, hi], accepting decimal or 0x-prefixed hex (base 0). */
static bool parse_int_range_auto(const char *s, long lo, long hi, long *out) {
    if (!s || !*s) return false;
    char *end = NULL;
    long v = strtol(s, &end, 0);
    if (end == s || *end != '\0') return false;
    if (v < lo || v > hi) return false;
    *out = v;
    return true;
}

/* Print one ballast in long form */
static void print_dali_long(uint8_t slave_id, uint8_t addr) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) { txt_err("invalid_slave"); return; }
    dali_ballast_state_t *b = state_cache_get_dali((uint8_t)idx, addr);
    if (!b) { txt_err("out_of_range"); return; }
    printf("dali %u.%u: present=%s on=%s bri=%u status=0x%02X "
           "fading=%s min=%u max=%u poweron=%u sysfail=%u "
           "fade_rate=%u fade_time=%u\n",
           slave_id, addr,
           b->present ? "true" : "false",
           b->arc_level > 0 ? "true" : "false",
           b->arc_level, b->status_byte,
           b->fading ? "true" : "false",
           b->min_level, b->max_level,
           b->power_on_level, b->sys_fail_level,
           b->fade_rate, b->fade_time);
}

/* Print one ballast in brief form (used in slave-wide listing) */
static void print_dali_brief(uint8_t slave_id, uint8_t addr) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    dali_ballast_state_t *b = state_cache_get_dali((uint8_t)idx, addr);
    if (!b) return;
    printf("dali %u.%u: on=%s bri=%u fading=%s\n",
           slave_id, addr,
           b->arc_level > 0 ? "true" : "false",
           b->arc_level,
           b->fading ? "true" : "false");
}

/* Print one group line. mask is the membership bits. */
static void print_group(uint8_t slave_id, uint8_t group, uint64_t members) {
    int idx = state_cache_slave_idx(slave_id);
    if (idx < 0) return;
    printf("group %u.G%u: members=[", slave_id, group);
    bool first = true;
    for (int a = 0; a < 64; a++) {
        if (members & ((uint64_t)1 << a)) {
            printf("%s%d", first ? "" : ",", a);
            first = false;
        }
    }
    bool any_on  = state_cache_dali_group_any_on((uint8_t)idx, group);
    uint8_t bri  = state_cache_dali_group_max_bri((uint8_t)idx, group);
    printf("] on=%s bri=%u\n", any_on ? "true" : "false", bri);
}

/* Print one mapping record */
static void print_mapping(const mapping_record_t *r) {
    printf("map F%u.B%u -> ", r->fixture, r->button);
    for (int i = 0; i < r->target_count; i++) {
        if (i > 0) printf(",");
        if (TARGET_IS_GROUP(r->targets[i])) {
            printf("%u:G%u", r->targets[i].slave,
                   TARGET_GROUP_NUM(r->targets[i]));
        } else {
            printf("%u:%u", r->targets[i].slave, r->targets[i].addr);
        }
    }
    printf("\n");
}

/* prov_state_name() is shared, in dali_provision.h beside the enum — there were
 * three separate copies of this table (here, ha_bridge.c, and none at all for
 * the web UI, which is why it showed a bare number). */

/* Parse a single "<slave>:<addr>" pair into a mapping_target_t. addr may be
 * a decimal short-address (0-63) or "G<group>" (group 0-15). Returns true
 * on success. */
static bool parse_target_pair(const char *s, mapping_target_t *out) {
    const char *colon = strchr(s, ':');
    if (!colon || colon == s || colon[1] == '\0') return false;
    char slave_buf[8];
    size_t slen = (size_t)(colon - s);
    if (slen >= sizeof(slave_buf)) return false;
    memcpy(slave_buf, s, slen);
    slave_buf[slen] = '\0';
    int slave = 0;
    if (!parse_int_range(slave_buf, 1, 247, &slave)) return false;
    out->slave = (uint8_t)slave;

    const char *addr_s = colon + 1;
    if (addr_s[0] == 'G' || addr_s[0] == 'g') {
        int group = 0;
        if (!parse_int_range(addr_s + 1, 0, 15, &group)) return false;
        if (slave_kind((uint8_t)slave) == SLAVE_KIND_RELAY) return false;
        out->addr = TARGET_MAKE_GROUP((uint8_t)group);
    } else {
        int addr = 0;
        if (!parse_int_range(addr_s, 0, 63, &addr)) return false;
        out->addr = (uint8_t)addr;
    }
    return true;
}

/* ── Raw Modbus CLI helpers ─────────────────────────────────────────────── */

typedef struct {
    volatile bool done;
    int           res;          /* raw result: >0 ok, 0 timeout, <0 exception */
    uint8_t       frame[256];
    int           frame_len;
} cli_mb_ctx_t;

static void cli_mb_cb(int res, uint8_t *buf, void *ctx) {
    cli_mb_ctx_t *c = (cli_mb_ctx_t *)ctx;
    c->res = res;
    int n = res > 0 ? res : (res < 0 ? -res : 0);
    if (n > (int)sizeof(c->frame)) n = (int)sizeof(c->frame);
    if (n > 0) memcpy(c->frame, buf, n);
    c->frame_len = n;
    c->done = true;
}

static void cli_mb_wait(cli_mb_ctx_t *c) {
    absolute_time_t deadline = make_timeout_time_ms(1500);
    while (!c->done && absolute_time_diff_us(get_absolute_time(), deadline) > 0) {
        modbus_poll();
        /* This blocks the core-0 main loop, which is the only watchdog feeder.
         * A single transaction can stall 1.5 s, and dali_group_set_members chains
         * up to four of these — past the 5 s watchdog against a dead gateway.
         * Feed here (core-1-gated, so a real core-1 wedge still reboots). */
        wd_feed();
    }
}

/* Report a transport-level failure; returns true if reported (caller should
 * return without printing further). */
static bool cli_mb_report_fail(const cli_mb_ctx_t *c) {
    if (!c->done) { txt_err("internal"); return true; }
    if (c->res == 0) { txt_err("timeout"); return true; }
    if (c->res < 0) {
        uint8_t code = c->frame_len >= 3 ? c->frame[2] : 0xFFu;
        printf("err: exception 0x%02X (%s)\n", code, modbus_exception_str(code));
        return true;
    }
    return false;
}

/* ── Raw DALI command + result helpers (via the gateway command queue) ──────── */

/* Decode a queue result word into a short label. Callers build the word as
 * 0x00<answer> (ok), 0x0100 (nak), or 0x02<subtype> (err) — the same shape the
 * old FC04 0x0108 register used, so downstream code (read_mem, identify) is
 * unchanged. */
static const char *dali_result_str(uint16_t r) {
    switch (r >> 8) {
        case 0x00: return "ok";
        case 0x01: return "nak";
        case 0x02:
            switch (r & 0xFFu) {
                case 1:  return "malformed";    /* garbled backward frame = collision */
                case 2:  return "pulse_short";
                case 3:  return "pulse_mid";
                case 4:  return "pulse_long";
                default: return "err";
            }
        default: return "err";
    }
}

/* Map a dq_result_t + answer byte to the legacy 0x00xx/0x0100/0x02xx word. */
static uint16_t dq_result_word(dq_result_t res, uint8_t answer) {
    if (res == DQ_RES_OK)  return answer;               /* 0x00 | answer */
    if (res == DQ_RES_NAK) return 0x0100u;
    return (uint16_t)(0x0200u | answer);                /* err subtype (or DROPPED->0x0203) */
}

/* Send a raw DALI frame through the gateway command queue and wait (blocking,
 * CLI only) for its result.  A single send opens the backward-frame window
 * (DQ_ANSWER) so a query's response byte is returned; a twice-send does not
 * (no answer, reports nak).  `idempotent` is passed to the queue: pass false for
 * the arbitrary-frame `dali_raw` command (a human may send an UP/DOWN step, which
 * must be DROPPED not resubmitted if a gateway reboot interrupts it), true for
 * known-idempotent internal uses (SET DTR, DAPC).  Sets *result to the legacy
 * result word and returns true; on backlog-full/timeout prints an error and
 * returns false. */
static uint32_t s_dali_raw_us;   /* elapsed time of the last dali_raw_result */

static bool dali_raw_result(uint8_t slave, uint16_t frame, bool twice,
                            bool idempotent, uint16_t *result) {
    dq_result_t res;
    uint8_t     ans   = 0;
    uint8_t     flags = twice ? DQ_TWICE : DQ_ANSWER;
    absolute_time_t t0 = get_absolute_time();
    bool got = dq_submit_sync(slave, (uint8_t)(frame >> 8), (uint8_t)frame,
                              flags, 0, idempotent, 300, &res, &ans);
    s_dali_raw_us = (uint32_t)absolute_time_diff_us(t0, get_absolute_time());
    if (!got) {
        txt_err(res == DQ_RES_BACKLOG ? "backlog full"
              : res == DQ_RES_TIMEOUT ? "timeout (no result in 300ms — bus busy)"
              : "timeout");
        return false;
    }
    if (res == DQ_RES_DROPPED) { txt_err("lost (gateway reboot)"); return false; }
    *result = dq_result_word(res, ans);
    return true;
}

/* As dali_raw_result, but preloads DTR0 = `dtr0` back-to-back before the frame
 * as ONE atomic queue entry (immune to a DTR0-clobbering command interleaving) —
 * used by read_mem's per-byte SET-DTR0 + READ-MEMORY pair.  READ MEMORY is a
 * query, so this is idempotent. */
static bool dali_raw_dtr0_result(uint8_t slave, uint16_t frame, uint8_t dtr0,
                                 uint16_t *result) {
    dq_result_t res;
    uint8_t     ans = 0;
    if (!dq_submit_sync(slave, (uint8_t)(frame >> 8), (uint8_t)frame,
                        DQ_ANSWER | DQ_DTR0, dtr0, true, 300, &res, &ans)) {
        txt_err("timeout"); return false;
    }
    *result = dq_result_word(res, ans);
    return true;
}

/* A garbled DALI backward frame can mean two gear answering at once (a real
 * collision, which garbles on every read) OR a one-off line glitch (which reads
 * clean on retry). Re-read a garbled byte up to this many times and only call it
 * a collision if it never comes back clean — kills transient false collisions. */
#define DALI_READ_RETRIES 3

/* Read `count` bytes from DALI memory `bank` starting at `offset`, from the
 * gear at short address `addr`.  Sequence: SET DTR1=bank, SET DTR0=offset, then
 * `count` READ MEMORY LOCATION (opcode 0xC5) — the gear auto-increments DTR0.
 * Sets *collision if a read garbles on every retry (two+ gear answering
 * differently) and *absent if a read NAKs (no gear there).  Returns false on
 * transport failure. */
static bool dali_read_mem(uint8_t slave, uint8_t addr, uint8_t bank,
                          uint8_t offset, uint8_t count, uint8_t *out,
                          bool *collision, bool *absent) {
    uint16_t r;
    *collision = false;
    *absent    = false;
    if (!dali_raw_result(slave, (uint16_t)(0xC300u | bank), false, true, &r)) return false; /* SET DTR1 (idempotent) */
    uint8_t addr_byte = (uint8_t)((addr << 1) | 0x01u);   /* short-address command form */
    for (uint8_t i = 0; i < count; i++) {
        bool got = false, nak = false, any_garble = false;
        for (uint8_t attempt = 0; attempt < DALI_READ_RETRIES; attempt++) {
            /* Re-set DTR0 before every read (not all gear auto-increment; some
             * have unreadable holes, e.g. bank-0 offset 0x01; a retry must
             * re-point at the same offset) — as ONE atomic queue entry so no
             * other DTR-touching command can slip between the DTR0 load and the
             * READ.  READ MEMORY LOCATION opcode 0xC5, backward frame expected. */
            uint16_t frame = (uint16_t)(((uint16_t)addr_byte << 8) | 0xC5u);
            if (!dali_raw_dtr0_result(slave, frame, (uint8_t)(offset + i), &r)) return false;
            if      ((r >> 8) == 0x00u) { got = true; break; }       /* clean byte — done, not a collision */
            else if (r == 0x0100u)      { nak = true; break; }       /* NAK: no gear / hole — definitive */
            else if ((r >> 8) == 0x02u) any_garble = true;           /* garbled — retry to rule out a glitch */
            /* any other unexpected result: fall through and retry too */
        }
        if      (got)        out[i] = (uint8_t)(r & 0xFFu);
        else if (any_garble) { *collision = true; out[i] = 0xFFu; }  /* garbled every attempt = 2+ gear */
        else if (nak)        { if (i == 0) *absent = true; out[i] = 0xFFu; }
        else                 out[i] = 0xFFu;
    }
    return true;
}

static void print_reg_table(const cli_mb_ctx_t *c, uint16_t base, uint16_t count) {
    int expected = 5 + count * 2;
    if (c->res < expected) { txt_err("short_response"); return; }
    if (c->frame[2] != count * 2) { txt_err("bad_bytecount"); return; }
    printf("  reg    dec       hex\n");
    for (uint16_t i = 0; i < count; i++) {
        uint16_t v = ((uint16_t)c->frame[3 + 2*i] << 8) | c->frame[3 + 2*i + 1];
        printf("%5u  %5u    0x%04X\n", (unsigned)(base + i), v, v);
    }
}

static void print_coil_table(const cli_mb_ctx_t *c, uint16_t base, uint16_t count) {
    int byte_count = (count + 7) / 8;
    int expected   = 5 + byte_count;
    if (c->res < expected) { txt_err("short_response"); return; }
    if (c->frame[2] != byte_count) { txt_err("bad_bytecount"); return; }
    printf(" coil  state\n");
    for (uint16_t i = 0; i < count; i++) {
        bool on = (c->frame[3 + i/8] >> (i % 8)) & 1u;
        printf("%5u  %s\n", (unsigned)(base + i), on ? "on" : "off");
    }
}

/* Common parser for the read-style commands: <slave> <reg> [count]. */
static bool parse_read_args(char **saveptr, long lo_count, long hi_count,
                            long *slave, long *reg, long *count) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *rs = strtok_r(NULL, " \t", saveptr);
    char *cs = strtok_r(NULL, " \t", saveptr);
    if (!parse_int_range_auto(as, 0, 247, slave)) { txt_err("invalid_slave"); return false; }
    if (!parse_int_range_auto(rs, 0, 65535, reg)) { txt_err("bad_arg"); return false; }
    *count = 1;
    if (cs && !parse_int_range_auto(cs, lo_count, hi_count, count)) {
        txt_err("bad_arg"); return false;
    }
    return true;
}

/* ── get_input <slave> <reg> [count] ────────────────────────────────────── */
static void cmd_get_input(char **saveptr) {
    long slave, reg, count;
    if (!parse_read_args(saveptr, 1, 64, &slave, &reg, &count)) return;
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "get_input slave=%ld reg=%ld count=%ld", slave, reg, count);
    modbus_get_input_registers((uint8_t)slave, (uint16_t)reg, (uint16_t)count,
                               cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    print_reg_table(&ctx, (uint16_t)reg, (uint16_t)count);
}

/* ── get_holding <slave> <reg> [count] ──────────────────────────────────── */
static void cmd_get_holding(char **saveptr) {
    long slave, reg, count;
    if (!parse_read_args(saveptr, 1, 64, &slave, &reg, &count)) return;
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "get_holding slave=%ld reg=%ld count=%ld", slave, reg, count);
    modbus_get_holding_registers((uint8_t)slave, (uint16_t)reg, (uint16_t)count,
                                  cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    print_reg_table(&ctx, (uint16_t)reg, (uint16_t)count);
}

/* ── set_holding <slave> <reg> <value> ──────────────────────────────────── */
static void cmd_set_holding(char **saveptr) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *rs = strtok_r(NULL, " \t", saveptr);
    char *vs = strtok_r(NULL, " \t", saveptr);
    long slave, reg, val;
    if (!parse_int_range_auto(as, 0, 247, &slave))   { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(rs, 0, 65535, &reg))   { txt_err("bad_arg"); return; }
    if (!parse_int_range_auto(vs, 0, 65535, &val))   { txt_err("bad_arg"); return; }
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "set_holding slave=%ld reg=%ld val=%ld", slave, reg, val);
    modbus_set_holding_register((uint8_t)slave, (uint16_t)reg, (uint16_t)val,
                                 cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    txt_ok();
}

/* ── get_coils <slave> <reg> [count] ────────────────────────────────────── */
static void cmd_get_coils(char **saveptr) {
    long slave, reg, count;
    if (!parse_read_args(saveptr, 1, 128, &slave, &reg, &count)) return;
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "get_coils slave=%ld reg=%ld count=%ld", slave, reg, count);
    modbus_get_coils((uint8_t)slave, (uint16_t)reg, (uint16_t)count,
                     cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    print_coil_table(&ctx, (uint16_t)reg, (uint16_t)count);
}

/* ── set_coil <slave> <reg> {true|false|toggle} ─────────────────────────── */
static void cmd_set_coil(char **saveptr) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *rs = strtok_r(NULL, " \t", saveptr);
    char *vs = strtok_r(NULL, " \t", saveptr);
    long slave, reg;
    if (!parse_int_range_auto(as, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(rs, 0, 65535, &reg)) { txt_err("bad_arg"); return; }
    if (!vs) { txt_err("bad_arg"); return; }
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "set_coil slave=%ld reg=%ld v=%s", slave, reg, vs);
    if (strcasecmp(vs, "true") == 0) {
        modbus_set_coil((uint8_t)slave, (uint16_t)reg, true, cli_mb_cb, &ctx);
    } else if (strcasecmp(vs, "false") == 0) {
        modbus_set_coil((uint8_t)slave, (uint16_t)reg, false, cli_mb_cb, &ctx);
    } else if (strcasecmp(vs, "toggle") == 0) {
        modbus_toggle((uint8_t)slave, (uint16_t)reg, cli_mb_cb, &ctx);
    } else {
        txt_err("bad_arg");
        return;
    }
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    txt_ok();
}

/* ── help ───────────────────────────────────────────────────────────────── */
static void cmd_help(void) {
    printf("help                                       list commands\n");
    printf("show config                                global tunables\n");
    printf("show mappings                              list all mappings\n");
    printf("show mapping <fix> <btn>                   one mapping\n");
    printf("show relay [<ch>]                          relay state (one or all)\n");
    printf("show dali [<slave> [<addr>]]               DALI state\n");
    printf("show group <slave> [<group>]               DALI group state\n");
    printf("show present <slave>                       present ballast addresses\n");
    printf("show provision                             provisioning status\n");
    printf("relay <ch> [on=true|false|toggle]          bare=get, else set\n");
    printf("dali <slave> <addr|gN> [key=val ...]       bare=get; keys: on bri min max\n");
    printf("                                           power_on sys_fail fade_rate fade_time\n");
    printf("                                           group (gN) also: members=0,1  add=2  remove=1\n");
    printf("cfg [key=val ...]                          bare=get; long_press_ms dim_repeat_ms fw_url\n");
    printf("                                           reversal_window_ms baud relay_slaves=1,2 dali_slaves=2,3\n");
    printf("tombstone dali <s> <a> | group <s> <g> | gw <s>   retire a removed device's retained MQTT topics\n");
    printf("btn <fixture> <button> [tap|press|release]  synthesize a button event (test)\n");
    printf("dali_raw <slave> <cmd16> [twice]           send raw 16-bit DALI frame, report result (dropped-not-resubmitted on gw reboot)\n");
    printf("read_mem <slave> <addr> <bank> <off> <n>   read n bytes from a ballast memory bank (hex)\n");
    printf("identify <slave> <addr>                    read a ballast's GTIN + serial (bank 0)\n");
    printf("blink <slave> <addr>                       flash one ballast to locate it physically\n");
    printf("gw_reboot <slave>                          reboot a TinyModbus gateway (recover stuck bus)\n");
    printf("dq [slave]                                 dump command-queue client state + counters\n");
    printf("soak [<slave> <addr> [n=] [depth=] [path=direct|bg|user]] | stop   queue soak\n");
    printf("busload [reset]                            RS-485 bus occupancy, split per slave\n");
    printf("dstate [slave]                             DALI world model: presence, sweep, verify marks\n");
    printf("dim <slave> <addr> {up|down}               step one ballast\n");
    printf("dim <slave> g<group> {up|down}             step one group\n");
    printf("provision <slave> {all|new|<addr>}         commission: all / incremental / re-provision collided <addr>\n");
    printf("provision abort                            stop commissioning\n");
    printf("scan <slave>                               blocking full scan\n");
    printf("discover                                   re-probe all configured slaves (blocking)\n");
    printf("ping <slave> [relay|dali]                  one ping at current master baud (kind inferred if omitted)\n");
    printf("set_slave_baud <slave> {9600|19200|38400} [relay|dali]  tell slave to switch its UART baud\n");
    printf("baud [<rate> [save]]                       show / set UART baud (live; 'save' persists)\n");
    printf("rs485 [{tx|always}]                         show / set RS485 DE mode (saved; tx=DE while transmitting)\n");
    printf("mqtt [show | set host= port= user= pass= prefix=]  broker config (reboot to apply)\n");
    printf("net  [show | set dhcp= ip= mask= gw= dns=]   network config (reboot to apply)\n");
    printf("net  ping <ip> [count]                      ICMP echo from the device (default 4)\n");
    printf("ls [<path>]                                list files in path (default /)\n");
    printf("cat <path>                                 dump file contents to console\n");
    printf("map <fix> <btn> [targets=<s>:<a>,...]     bare=get; targets= deletes (a=N or gN)\n");
    printf("log <level>                                set verbosity (off|err|warn|info|dbg)\n");
    printf("show log                                   current verbosity\n");
    printf("get_input <slave> <reg> [count]            raw FC04 read input registers\n");
    printf("get_holding <slave> <reg> [count]          raw FC03 read holding registers\n");
    printf("set_holding <slave> <reg> <value>          raw FC06 write holding register\n");
    printf("get_coils <slave> <reg> [count]            raw FC01 read coils\n");
    printf("set_coil <slave> <reg> {true|false|toggle} raw FC05 write coil\n");
    printf("  (reg/value accept decimal or 0x-prefixed hex)\n");
    printf("echo {on|off}                              echo input + backspace editing (or show state)\n");
    printf("reboot                                     watchdog reset\n");
    printf("resync                                     re-emit hello + all state + mappings\n");
}

/* ── show subcommands ──────────────────────────────────────────────────── */
static void cmd_show(char **saveptr) {
    char *what = strtok_r(NULL, " \t", saveptr);
    if (!what) { txt_err("bad_arg"); return; }

    if (strcasecmp(what, "config") == 0) {
        printf("long_press_ms=%u dim_repeat_ms=%u reversal_window_ms=%u "
               "gw_diag_interval_ms=%lu dali_sweep_interval_ms=%lu "
               "baud=%lu rs485=%s relay_byteswap=%u fw_url=%s relay_slaves=",
               g_config.long_press_ms, g_config.dim_repeat_ms,
               g_config.reversal_window_ms,
               (unsigned long)g_config.gw_diag_interval_ms,
               (unsigned long)g_config.dali_sweep_interval_ms,
               (unsigned long)modbus_get_baud(),
               modbus_de_mode_to_str(modbus_get_de_mode()),
               g_config.relay_byteswap ? 1u : 0u,
               g_config.fw_url[0] ? g_config.fw_url : "(default)");
        for (int i = 0; i < g_config.relay_slave_count; i++) {
            printf("%s%u", i > 0 ? "," : "", g_config.relay_slave_ids[i]);
        }
        printf(" dali_slaves=");
        for (int i = 0; i < g_config.dali_slave_count; i++) {
            printf("%s%u", i > 0 ? "," : "", g_config.dali_slave_ids[i]);
        }
        printf("\n");
        return;
    }

    if (strcasecmp(what, "mappings") == 0) {
        bool any = false;
        for (int f = 0; f < MAX_FIXTURES; f++) {
            for (int b = 0; b < MAX_BUTTONS; b++) {
                const mapping_record_t *r = mapping_lookup((uint8_t)f, (uint8_t)b);
                if (r) { print_mapping(r); any = true; }
            }
        }
        if (!any) printf("(no mappings)\n");
        return;
    }

    if (strcasecmp(what, "mapping") == 0) {
        char *fs = strtok_r(NULL, " \t", saveptr);
        char *bs = strtok_r(NULL, " \t", saveptr);
        int fix = 0, btn = 0;
        if (!parse_int_range(fs, 0, MAX_FIXTURES - 1, &fix) ||
            !parse_int_range(bs, 0, MAX_BUTTONS - 1, &btn)) {
            txt_err("bad_arg"); return;
        }
        const mapping_record_t *r = mapping_lookup((uint8_t)fix, (uint8_t)btn);
        if (!r) { printf("(none)\n"); return; }
        print_mapping(r);
        return;
    }

    if (strcasecmp(what, "relay") == 0) {
        char *cs = strtok_r(NULL, " \t", saveptr);
        if (!cs) {
            for (int ch = 0; ch < RELAY_COUNT; ch++) {
                /* "unread" is not "off". The cached false is a default until a
                 * coil read or a write ACK fills it in. */
                printf("relay %d: %s\n", ch,
                       !state_cache_relay_known((uint8_t)ch) ? "unread"
                       : state_cache_get_relay_on((uint8_t)ch) ? "on" : "off");
            }
            return;
        }
        int ch = 0;
        if (!parse_int_range(cs, 0, RELAY_COUNT - 1, &ch)) {
            txt_err("out_of_range"); return;
        }
        printf("relay %d: %s\n", ch,
               !state_cache_relay_known((uint8_t)ch) ? "unread"
               : state_cache_get_relay_on((uint8_t)ch) ? "on" : "off");
        return;
    }

    if (strcasecmp(what, "dali") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        if (!ss) {
            /* All DALI gateways */
            for (int i = 0; i < g_config.dali_slave_count; i++) {
                uint8_t sid = g_config.dali_slave_ids[i];
                for (int a = 0; a < DALI_ADDR_COUNT; a++) {
                    dali_ballast_state_t *b =
                        state_cache_get_dali((uint8_t)i, (uint8_t)a);
                    if (b && b->present) print_dali_brief(sid, (uint8_t)a);
                }
            }
            return;
        }
        int slave = 0;
        if (!parse_int_range(ss, 0, 247, &slave)) {
            txt_err("invalid_slave"); return;
        }
        int idx = state_cache_slave_idx((uint8_t)slave);
        if (idx < 0) { txt_err("invalid_slave"); return; }

        char *as = strtok_r(NULL, " \t", saveptr);
        if (!as) {
            for (int a = 0; a < DALI_ADDR_COUNT; a++) {
                dali_ballast_state_t *b =
                    state_cache_get_dali((uint8_t)idx, (uint8_t)a);
                if (b && b->present) print_dali_brief((uint8_t)slave, (uint8_t)a);
            }
            return;
        }
        int addr = 0;
        if (!parse_int_range(as, 0, 63, &addr)) {
            txt_err("out_of_range"); return;
        }
        print_dali_long((uint8_t)slave, (uint8_t)addr);
        return;
    }

    if (strcasecmp(what, "group") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        int slave = 0;
        if (!parse_int_range(ss, 0, 247, &slave)) {
            txt_err("invalid_slave"); return;
        }
        int idx = state_cache_slave_idx((uint8_t)slave);
        if (idx < 0) { txt_err("invalid_slave"); return; }

        char *gs = strtok_r(NULL, " \t", saveptr);
        if (!gs) {
            bool any = false;
            for (int g = 0; g < 16; g++) {
                uint64_t mask =
                    state_cache_get_dali_group((uint8_t)idx, (uint8_t)g);
                if (mask) {
                    print_group((uint8_t)slave, (uint8_t)g, mask);
                    any = true;
                }
            }
            if (!any) printf("(no groups)\n");
            return;
        }
        int group = 0;
        if (!parse_int_range(gs, 0, 15, &group)) {
            txt_err("out_of_range"); return;
        }
        uint64_t mask =
            state_cache_get_dali_group((uint8_t)idx, (uint8_t)group);
        print_group((uint8_t)slave, (uint8_t)group, mask);
        return;
    }

    if (strcasecmp(what, "present") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        int slave = 0;
        if (!parse_int_range(ss, 0, 247, &slave)) {
            txt_err("invalid_slave"); return;
        }
        int idx = state_cache_slave_idx((uint8_t)slave);
        if (idx < 0) { txt_err("invalid_slave"); return; }
        printf("present %d:", slave);
        for (int a = 0; a < DALI_ADDR_COUNT; a++) {
            dali_ballast_state_t *b =
                state_cache_get_dali((uint8_t)idx, (uint8_t)a);
            if (b && b->present) printf(" %d", a);
        }
        printf("\n");
        return;
    }

    if (strcasecmp(what, "provision") == 0) {
        printf("provision: state=%s slave=%u devices_found=%u msg=\"%s\"\n",
               prov_state_name(g_prov_status.run_state),
               g_prov_status.slave_id,
               g_prov_status.devices_found,
               g_prov_status.msg);
        return;
    }

    if (strcasecmp(what, "log") == 0) {
        printf("log: %s\n", log_level_name(g_log_level));
        return;
    }

    txt_err("bad_arg");
}

/* ── log <level> ─────────────────────────────────────────────────────────── */
static void cmd_log(char **saveptr) {
    char *ls = strtok_r(NULL, " \t", saveptr);
    log_level_t lvl;
    if (!ls || !log_level_parse(ls, &lvl)) {
        txt_err("bad_arg");
        return;
    }
    g_log_level = lvl;
    printf("log: %s\n", log_level_name(g_log_level));
}

/* ── relay <ch> {on|off|toggle} ────────────────────────────────────────── */
/* relay <ch> [on=true|false|toggle] — bare = get (emit state_relay). */
static void cmd_relay(char **saveptr) {
    char *cs = strtok_r(NULL, " \t", saveptr);
    int ch = 0;
    if (!parse_int_range(cs, 0, RELAY_COUNT - 1, &ch)) { txt_err("out_of_range"); return; }
    char *kv = strtok_r(NULL, " \t", saveptr);
    if (!kv) {   /* get */
        /* Never read means never published — the cached false is a default, not
         * a measurement. Same rule lighting_resync_emit follows. */
        if (!state_cache_relay_known((uint8_t)ch)) { txt_err("not_read_yet"); return; }
        emit_state_relay((uint8_t)ch);
        txt_ok();
        return;
    }
    do {
        char *eq = strchr(kv, '=');
        if (!eq) { txt_err("bad_arg expected key=value"); return; }
        *eq = '\0';
        if (strcasecmp(kv, "on") != 0) { txt_err("bad_field"); return; }
        const char *val = eq + 1;
        /* A toggle needs a known starting point. Inverting the default false on a
         * channel we have never read is a guess, and a 50/50 one. */
        if (strcasecmp(val, "toggle") == 0 && !state_cache_relay_known((uint8_t)ch)) {
            txt_err("not_read_yet");
            return;
        }
        bool on = (strcasecmp(val, "toggle") == 0) ? !state_cache_get_relay_on((uint8_t)ch)
                : (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 || strcasecmp(val, "on") == 0);
        /* No predictive write and no emit here. relay_write_cb caches and emits
         * when the Modbus write is ACKed — that ACK is the confirmation. The
         * predictive write was worse than redundant: on a FAILED write nothing
         * corrected it, so the cache (and HA) kept the requested state until the
         * next coil poll happened to disagree, up to RELAY_POLL_INTERVAL_MS later. */
        lighting_set_relay((uint8_t)ch, on);
    } while ((kv = strtok_r(NULL, " \t", saveptr)) != NULL);
    txt_ok();
}

/* dali <slave> <addr|gN> [key=value ...] — bare = get; key=value = set.
 * keys: on(true|false|toggle) bri min max power_on sys_fail fade_rate fade_time
 * (groups take on/bri only). */
static void cmd_dali(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    char *as = strtok_r(NULL, " \t", saveptr);
    int slave = 0;
    if (!parse_int_range(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (state_cache_slave_idx((uint8_t)slave) < 0) { txt_err("invalid_slave"); return; }
    if (!as) { txt_err("bad_arg"); return; }

    bool is_group = (as[0] == 'g' || as[0] == 'G');
    int group = 0, addr = 0;
    if (is_group) { if (!parse_int_range(as + 1, 0, 15, &group)) { txt_err("out_of_range"); return; } }
    else          { if (!parse_int_range(as, 0, 63, &addr))      { txt_err("out_of_range"); return; } }

    char *kv = strtok_r(NULL, " \t", saveptr);
    if (!kv) {   /* get */
        if (is_group) emit_state_group((uint8_t)slave, (uint8_t)group);
        else          emit_state_dali((uint8_t)slave, (uint8_t)addr);
        txt_ok();
        return;
    }
    /* Emit only when a key produced a value we actually KNOW. A brightness set
     * does not: the ballast clamps and fades, so its real level arrives later via
     * the verification read, which publishes then. This mirrors the emit_now
     * discipline the JSON path already used — the CLI used to emit
     * unconditionally, republishing a stale or predicted level. */
    bool emit_now = false;
    do {
        char *eq = strchr(kv, '=');
        if (!eq) { txt_err("bad_arg expected key=value"); return; }
        *eq = '\0';
        const char *key = kv, *val = eq + 1;
        if (is_group) {
            if (strcasecmp(key, "bri") == 0)
                /* No emit: the group's reported brightness is derived from its
                 * members' cached levels, and those are still the pre-command
                 * ones until each member's verification read settles. Publishing
                 * now would show the old value, then jump. */
                lighting_group_set_level((uint8_t)slave, (uint8_t)group, (uint8_t)atoi(val));
            else if (strcasecmp(key, "on") == 0) {
                /* "toggle" used to match none of the true-ish spellings and so
                 * evaluated to FALSE — a group toggle silently meant OFF, with
                 * no error. The per-ballast path handled toggle properly; only
                 * groups were wrong.
                 *
                 * A toggle now READS THEN WRITES, exactly as a button press
                 * does: QUERY LAMP POWER ON straight to the group selector, and
                 * invert whatever the gear reports. It used to invert the cached
                 * membership state, which is the staleness the button's probe
                 * was introduced to defeat — a gateway reboot loses the masks
                 * and the cache then answers confidently and wrongly. */
                int t = parse_on_off_toggle(val);
                if (t < 0) { txt_err("bad_arg"); return; }
                bool on;
                if (t == 2) {
                    bool any;
                    uint8_t gb = (uint8_t)(0x81u | (((uint8_t)group & 0x0Fu) << 1));
                    if (!lighting_any_on_probe_sync((uint8_t)slave, gb, &any)) {
                        /* Refuse rather than guess — guessing is what the cache
                         * did, and a wrong guess drives the lights the wrong way. */
                        txt_err("probe_unresolved");
                        return;
                    }
                    printf("any_on=%s (probe)\n", any ? "true" : "false");
                    on = !any;
                } else {
                    on = (t == 1);
                }
                /* No emit: a group's on/level are DERIVED from its members'
                 * measured levels, and none of them has been re-read yet.
                 * Publishing here would push a value computed from pre-command
                 * levels. lighting_group_set_on_off marks every member for
                 * verification; those reads publish the member states and each
                 * one recomputes and republishes the group. */
                lighting_group_set_on_off((uint8_t)slave, (uint8_t)group, on);
            }
            else if (strcasecmp(key, "members") == 0 || strcasecmp(key, "add") == 0 ||
                     strcasecmp(key, "remove") == 0) {
                uint8_t arr[DALI_ADDR_COUNT];
                int gn = parse_csv_u8(val, arr, DALI_ADDR_COUNT);
                int gidx = state_cache_slave_idx((uint8_t)slave);
                uint64_t mask = gidx >= 0 ? state_cache_get_dali_group((uint8_t)gidx, (uint8_t)group) : 0;
                uint64_t bits = 0;
                for (int i = 0; i < gn; i++) if (arr[i] < 64) bits |= (uint64_t)1 << arr[i];
                if      (strcasecmp(key, "members") == 0) mask = bits;
                else if (strcasecmp(key, "add") == 0)     mask |= bits;
                else                                      mask &= ~bits;
                /* No emit: membership is only known once QUERY GROUPS confirms
                 * it, which dali_group_set_members arms. */
                dali_group_set_members((uint8_t)slave, (uint8_t)group, mask);
            }
            else { txt_err("bad_field"); return; }
        } else {
            /* Resolve a toggle HERE, by asking the ballast, before delegating.
             * dali_apply_kv is shared with the JSON/intercore path, which runs
             * in the main loop and must not block — so the blocking probe cannot
             * live inside it. The CLI resolves to a concrete true/false and the
             * shared code just applies it. */
            const char *v = val;
            char        resolved[6];
            if (strcasecmp(key, "on") == 0 && strcasecmp(val, "toggle") == 0) {
                bool any;
                uint8_t ab = (uint8_t)((((uint8_t)addr & 0x3Fu) << 1) | 0x01u);
                if (!lighting_any_on_probe_sync((uint8_t)slave, ab, &any)) {
                    txt_err("probe_unresolved");
                    return;
                }
                printf("any_on=%s (probe)\n", any ? "true" : "false");
                snprintf(resolved, sizeof resolved, "%s", any ? "false" : "true");
                v = resolved;
            }
            int r = dali_apply_kv((uint8_t)slave, (uint8_t)addr, key, v);
            if (r == 0)  { txt_err("bad_field");   return; }
            if (r == -1) { txt_err("not_present"); return; }
            if (r == -2) return;   /* transport error already printed */
            if (r == 1)  emit_now = true;   /* a value worth reporting now */
        }
    } while ((kv = strtok_r(NULL, " \t", saveptr)) != NULL);
    dirty_poll_mark_active((uint8_t)slave);
    if (emit_now) {
        if (is_group) emit_state_group((uint8_t)slave, (uint8_t)group);
        else          emit_state_dali((uint8_t)slave, (uint8_t)addr);
    }
    txt_ok();
}

/* tombstone dali <slave> <addr> | group <slave> <grp> | gw <slave>
 * Retire a removed device: core 1 publishes empty retained payloads to its
 * discovery + data topics so HA drops the entity and the broker holds no stale
 * retained state (task 03, Phase C). Operator-driven — removal is a deliberate
 * decommission, not a transient DALI dropout (which is availability). */
static void cmd_tombstone(char **saveptr) {
    char *kind = strtok_r(NULL, " \t", saveptr);
    char *ss   = strtok_r(NULL, " \t", saveptr);
    if (!kind || !ss) { txt_err("bad_arg"); return; }
    int slave;
    if (!parse_int_range(ss, 1, 247, &slave)) { txt_err("invalid_slave"); return; }
    ic_event_t ev = { .type = IC_EVT_TOMBSTONE };
    ev.tombstone.slave = (uint8_t)slave;
    if (strcasecmp(kind, "dali") == 0 || strcasecmp(kind, "group") == 0) {
        bool grp = strcasecmp(kind, "group") == 0;
        char *as = strtok_r(NULL, " \t", saveptr);
        int a;
        if (!parse_int_range(as, 0, grp ? 15 : 63, &a)) { txt_err("out_of_range"); return; }
        ev.tombstone.addr = (uint8_t)a;
        ev.tombstone.kind = grp ? IC_TOMB_GROUP : IC_TOMB_BALLAST;
    } else if (strcasecmp(kind, "gw") == 0) {
        ev.tombstone.kind = IC_TOMB_GATEWAY;
    } else { txt_err("bad_arg expected dali|group|gw"); return; }
    intercore_push_event(&ev);
    LOG_INFO("cli", "tombstone %s slave %d queued", kind, slave);
    txt_ok();
}

/* cfg [key=value ...] — controller device config; bare = get (emit state_cfg).
 * keys: long_press_ms dim_repeat_ms reversal_window_ms baud (reboot to apply)
 *       relay_slaves=1,2  dali_slaves=2,3  (comma lists). fw_version is read-only. */
static void cmd_cfg(char **saveptr) {
    char *kv = strtok_r(NULL, " \t", saveptr);
    if (!kv) { emit_state_cfg(); txt_ok(); return; }
    bool any = false;
    do {
        char *eq = strchr(kv, '=');
        if (!eq) { txt_err("bad_arg expected key=value"); return; }
        *eq = '\0';
        const char *key = kv, *val = eq + 1;
        if      (strcasecmp(key, "long_press_ms") == 0)      g_config.long_press_ms = (uint16_t)atoi(val);
        else if (strcasecmp(key, "dim_repeat_ms") == 0)      g_config.dim_repeat_ms = (uint16_t)atoi(val);
        else if (strcasecmp(key, "reversal_window_ms") == 0) g_config.reversal_window_ms = (uint16_t)atoi(val);
        else if (strcasecmp(key, "gw_diag_interval_ms") == 0) g_config.gw_diag_interval_ms = (uint32_t)strtoul(val, NULL, 10);
        else if (strcasecmp(key, "dali_sweep_interval_ms") == 0) g_config.dali_sweep_interval_ms = (uint32_t)strtoul(val, NULL, 10);
        else if (strcasecmp(key, "baud") == 0)               modbus_set_baud((uint32_t)atoi(val));
        else if (strcasecmp(key, "relay_byteswap") == 0)
            g_config.relay_byteswap = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 || strcasecmp(val, "on") == 0);
        else if (strcasecmp(key, "relay_slaves") == 0)
            g_config.relay_slave_count = (uint8_t)parse_csv_u8(val, g_config.relay_slave_ids, MAX_RELAY_SLAVES);
        else if (strcasecmp(key, "dali_slaves") == 0)
            g_config.dali_slave_count  = (uint8_t)parse_csv_u8(val, g_config.dali_slave_ids, MAX_DALI_SLAVES);
        else if (strcasecmp(key, "fw_url") == 0) {
            /* empty value clears the override (compiled-in default applies) */
            strncpy(g_config.fw_url, val, sizeof g_config.fw_url - 1);
            g_config.fw_url[sizeof g_config.fw_url - 1] = '\0';
        }
        else { txt_err("bad_field"); return; }
        any = true;
    } while ((kv = strtok_r(NULL, " \t", saveptr)) != NULL);
    if (any && !config_save()) { txt_err("save_failed"); return; }
    emit_state_cfg();
    txt_ok();
}

/* ── btn <fixture> <button> [tap|press|release] ────────────────────────────
 * Synthesize a button event (test hook).  Default `tap` = press+release, which
 * runs the same FSM path as a real short press. */
static void cmd_btn(char **saveptr) {
    char *fs = strtok_r(NULL, " \t", saveptr);
    char *bs = strtok_r(NULL, " \t", saveptr);
    char *as = strtok_r(NULL, " \t", saveptr);
    int fix, btn;
    if (!parse_int_range(fs, 0, MAX_FIXTURES - 1, &fix)) { txt_err("bad_fixture"); return; }
    if (!parse_int_range(bs, 0, MAX_BUTTONS - 1, &btn))  { txt_err("bad_button"); return; }
    const char *act = as ? as : "tap";
    if (strcasecmp(act, "press") == 0) {
        button_fsm_inject((uint8_t)fix, (uint8_t)btn, BTN_RAW_PRESS);
    } else if (strcasecmp(act, "release") == 0) {
        button_fsm_inject((uint8_t)fix, (uint8_t)btn, BTN_RAW_RELEASE);
    } else if (strcasecmp(act, "tap") == 0) {
        button_fsm_inject((uint8_t)fix, (uint8_t)btn, BTN_RAW_PRESS);
        button_fsm_inject((uint8_t)fix, (uint8_t)btn, BTN_RAW_RELEASE);
    } else { txt_err("bad_arg"); return; }
    LOG_INFO("cli", "btn F%d.B%d %s (synthesized)", fix, btn, act);
    txt_ok();
}

/* ── DALI config get/set ───────────────────────────────────────────────────
 * dali_cfg_field maps a field name to an INTERNAL (base,high) encoding — the
 * `base` values 0x0040/0x0080/0x00C0 are just a compact way to identify the
 * field pair (min|max, power_on|sys_fail, fade_rate|fade_time); they are NOT
 * gateway register addresses (the gateway config banks were retired in task 04).
 * dali_cfg_set translates (base,high) into a DALI field enum and writes it via a
 * raw DTR0+SET queue entry (lighting_dali_cfg_write). The ballast may clamp a
 * value (e.g. min to its physical minimum); the query-back rescan then reflects
 * the actual stored value. */
static bool dali_cfg_field(const char *name, uint16_t *base, bool *high) {
    if (strcasecmp(name, "min")       == 0) { *base = 0x0040u; *high = true;  return true; }
    if (strcasecmp(name, "max")       == 0) { *base = 0x0040u; *high = false; return true; }
    if (strcasecmp(name, "power_on")  == 0) { *base = 0x0080u; *high = true;  return true; }
    if (strcasecmp(name, "sys_fail")  == 0) { *base = 0x0080u; *high = false; return true; }
    if (strcasecmp(name, "fade_rate") == 0) { *base = 0x00C0u; *high = true;  return true; }
    if (strcasecmp(name, "fade_time") == 0) { *base = 0x00C0u; *high = false; return true; }
    return false;
}

/* Returns 1 ok, 0 bad field, -1 not present, -2 transport error (printed). */
static int dali_cfg_set(uint8_t slave, uint8_t addr, const char *field, uint8_t value) {
    uint16_t base; bool high;
    if (!dali_cfg_field(field, &base, &high)) return 0;
    int idx = state_cache_slave_idx(slave);
    dali_ballast_state_t *b = idx >= 0 ? state_cache_get_dali((uint8_t)idx, addr) : NULL;
    if (!b) return -1;

    /* Config writes are always raw DTR0-load + SET pairs, verified by a
     * query-back that write-throughs the cache. No predictive cache update here
     * — the retained value only changes on verify. dali_cfg_field's (base,high)
     * maps 1:1 onto the field enum. */
    dali_cfg_field_t f = base == 0x0040u ? (high ? DALI_CFG_MIN       : DALI_CFG_MAX)
                       : base == 0x0080u ? (high ? DALI_CFG_POWER_ON  : DALI_CFG_SYS_FAIL)
                       :                    (high ? DALI_CFG_FADE_RATE : DALI_CFG_FADE_TIME);
    /* 2 = applied async; the query-back write-throughs the cache and emits
     * the verified (possibly clamped) value — no premature pre-verify emit. */
    return lighting_dali_cfg_write(slave, addr, f, value) ? 2 : -2;
}

/* ── Unified {t,d,v} state reports — used by get_* and as set_* confirmation ─
 * Same shape the client sends to set: {"t":"state_X","d":{addr},"v":{values}}. */
static unsigned long now_ms_u(void) {
    return (unsigned long)to_ms_since_boot(get_absolute_time());
}

/* Push the cached state of one entity to core 1 (the network bridge). These are
 * the get/predictive paths; the unsolicited dirty-poll path goes through the
 * lighting event ring + levt_to_ic in serial_poll(). USB CDC carries no JSON —
 * it is a human text CLI only (see `show`). */
/* NOTE for all emit_state_* below: these publish to CORE 1 (MQTT/HA) via the
 * intercore queue. They print NOTHING to the console.
 *
 * That is a hangover from when the Python host bridge consumed this CLI's JSON
 * over the serial wire — emitting the report WAS the reply. Since the on-board
 * bridge moved to core 1 (2026-06-09) the report goes to a channel the human at
 * the terminal cannot see, so every CLI verb using these looked like it did
 * nothing: `dali 3 0 on=true` switched the light and printed not a byte. Text
 * command handlers must therefore finish with txt_ok() or txt_err(); the JSON /
 * intercore path in ic_dispatch_cmd must NOT, since nothing is listening there. */
static void emit_state_dali(uint8_t slave, uint8_t addr) {
    int idx = state_cache_slave_idx(slave);
    dali_ballast_state_t *b = idx >= 0 ? state_cache_get_dali((uint8_t)idx, addr) : NULL;
    ic_event_t ev = { .type = IC_EVT_DALI };
    ev.dali.slave     = slave;
    ev.dali.addr      = addr;
    ev.dali.arc       = b ? b->arc_level   : 0;
    ev.dali.status    = b ? b->status_byte : 0;
    ev.dali.min       = b ? b->min_level       : 0;
    ev.dali.max       = b ? b->max_level       : 0;
    ev.dali.power_on  = b ? b->power_on_level   : 0;
    ev.dali.sys_fail  = b ? b->sys_fail_level   : 0;
    ev.dali.fade_rate = b ? b->fade_rate        : 0;
    ev.dali.fade_time = b ? b->fade_time        : 0;
    intercore_push_event(&ev);
}

static void emit_state_relay(uint8_t ch) {
    ic_event_t ev = { .type = IC_EVT_RELAY };
    ev.relay.ch = ch;
    ev.relay.on = state_cache_get_relay_on(ch);
    intercore_push_event(&ev);
}

static void emit_state_group(uint8_t slave, uint8_t group) {
    int idx = state_cache_slave_idx(slave);
    ic_event_t ev = { .type = IC_EVT_GROUP };
    ev.group.slave   = slave;
    ev.group.group   = group;
    ev.group.on      = idx >= 0 && state_cache_dali_group_any_on((uint8_t)idx, group);
    ev.group.bri     = idx >= 0 ? state_cache_dali_group_max_bri((uint8_t)idx, group) : 0;
    ev.group.members = idx >= 0 ? state_cache_get_dali_group((uint8_t)idx, group) : 0;
    intercore_push_event(&ev);
}

/* Set a DALI group's membership.
 *
 * Task 05: this used to write the gateway's group-mask registers (0x0110+) and
 * let the GATEWAY diff old-vs-new and emit the ADD/REMOVE FROM GROUP frames.
 * That cache is being removed, and it was also the root of the group-toggle bug
 * — a rebooted gateway lost the masks and the diff had nothing to work from.
 * The controller now diffs against its own cache and emits the frames itself.
 *
 * ADD TO GROUP is 0x60|g and REMOVE FROM GROUP is 0x70|g, both SET-class so
 * they go send-twice, addressed to each ballast whose membership changed. They
 * are idempotent (re-adding a member is a no-op), so a retry is safe.
 *
 * The membership cache is NOT taken on trust afterwards: each touched ballast is
 * marked for a full re-interrogation, so QUERY GROUPS confirms what actually
 * stuck. Predicting it locally is what made the old bug invisible. */
static void dali_group_set_members(uint8_t slave, uint8_t group, uint64_t new_mask) {
    int idx = state_cache_slave_idx(slave);
    if (idx < 0) return;
    uint64_t old_mask = state_cache_get_dali_group((uint8_t)idx, group);
    uint64_t diff = old_mask ^ new_mask;
    if (!diff) return;

    for (uint8_t a = 0; a < DALI_ADDR_COUNT; a++) {
        uint64_t bit = (uint64_t)1 << a;
        if (!(diff & bit)) continue;
        bool add = (new_mask & bit) != 0;
        uint8_t addr_byte = (uint8_t)(((a & 0x3Fu) << 1) | 0x01u);
        uint8_t opcode    = (uint8_t)((add ? 0x60u : 0x70u) | (group & 0x0Fu));

        /* The user class is bounded, so spin the bus until this one is accepted
         * rather than dropping members of a membership change on the floor. */
        absolute_time_t dl = make_timeout_time_ms(2000);
        while (!dq_submit(slave, DQ_PRIO_GROUP_WRITE, addr_byte, opcode,
                          DQ_TWICE, 0, true, NULL, NULL)) {
            if (absolute_time_diff_us(get_absolute_time(), dl) <= 0) {
                LOG_WARN("cli", "group %u.G%u %s addr %u dropped — queue busy",
                         slave, group, add ? "add" : "remove", a);
                break;
            }
            modbus_poll();
            dq_tick();
            wd_feed();
        }
        dali_state_mark_scan(slave, a);   /* confirm by reading it back */
    }
    /* NO optimistic cache write. dali_state_mark_scan above re-reads QUERY GROUPS
     * for every touched ballast, and commit_ballast writes the measured mask —
     * that is the only writer. Recording new_mask here would publish membership
     * we had merely requested, and a write that silently failed would sit in the
     * cache (and in HA) until something else happened to re-read it.
     * lighting_emit_group_state_if_changed now compares members, so the read-back
     * publishes the new list on its own. */
    (void)idx;
}

static void emit_state_cfg(void) {
    /* fw_version / slave lists / grid dimensions are derived on core 1 from its
     * config snapshot + version.h. */
    ic_event_t ev = { .type = IC_EVT_CFG };
    ev.cfg.long_press_ms      = g_config.long_press_ms;
    ev.cfg.dim_repeat_ms      = g_config.dim_repeat_ms;
    ev.cfg.reversal_window_ms = g_config.reversal_window_ms;
    ev.cfg.baud               = modbus_get_baud();
    ev.cfg.de_mode            = (uint8_t)modbus_get_de_mode();
    ev.cfg.relay_byteswap     = g_config.relay_byteswap ? 1u : 0u;
    ev.cfg.gw_diag_interval_ms = g_config.gw_diag_interval_ms;
    ev.cfg.dali_sweep_interval_ms = g_config.dali_sweep_interval_ms;
    intercore_push_event(&ev);
}

/* Apply one DALI key=value to a ballast (shared by set_dali JSON + `dali` CLI).
 * NOTHING HERE PREDICTS. State keys (on/bri) command the ballast and return 2 —
 * "applied, nothing measured yet" — so no value is cached or published until a
 * verification read reports what the gear actually did. Config keys go through
 * dali_cfg_set, which has always worked this way.  Returns 1 ok, 0 unknown key, -1 not present, -2 transport err. */
static int dali_apply_kv(uint8_t slave, uint8_t addr, const char *key, const char *val) {
    /* No cached-state reads needed: nothing here predicts a level any more, so
     * there is no old value to carry forward. */
    if (strcasecmp(key, "bri") == 0) {
        uint8_t lvl = (uint8_t)atoi(val);
        lighting_dali_set_level(slave, addr, lvl);
        /* The requested level is NOT written to the cache and NOT published.
         * A ballast clamps to its own min/max (and its physical minimum, which
         * can be far above the request) and may fade there over seconds, so the
         * number asked for is frequently not the number the light ends up at.
         * Publishing it made HA show the request, then jump when the real level
         * arrived. Only the verification read publishes, once the fade settles —
         * see commit_ballast()/fade_watch in dali_state.c.
         * Returns 2 = applied, nothing known to report yet. */
        return 2;
    }
    if (strcasecmp(key, "on") == 0) {
        /* "toggle" reaches here only from the JSON/intercore path — the CLI
         * resolves it with a blocking probe before calling in (see cmd_dali).
         * This path runs in the main loop and must not block, so it hands off to
         * the ASYNC probe: query the gear, apply the inverse when the answer
         * lands. It used to invert cached commanded_on, which is the staleness
         * the button's probe exists to defeat. */
        if (strcasecmp(val, "toggle") == 0) {
            if (!lighting_toggle_async(slave, addr, false)) return -2;
            return 2;   /* applied later — nothing to report yet */
        }
        bool on = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 ||
                   strcasecmp(val, "on") == 0);
        lighting_set_on_off(slave, addr, on);
        /* Not even OFF is written ahead of the read. The endpoint is certainly
         * level 0, but with a fade time the light is NOT there yet, and a cache
         * that says 0 while the lamp is still ramping down is an inferred value
         * on a topic that must only carry measured ones. The verification read
         * marked by lighting_set_on_off reports the real level, including each
         * step of the ramp. */
        return 2;   /* applied — nothing measured yet to report */
        /* On: GOTO_LAST_ACTIVE restores a level the controller does NOT know.
         * Don't assume it (the old code guessed max_level/254) — commanded_on is
         * already set by lighting_set_on_off, and the dirty-poll reports the real
         * brightness shortly.  Return 2 = "applied, no level to report yet". */
        return 2;
    }
    return dali_cfg_set(slave, addr, key, (uint8_t)atoi(val));
}

/* ── dali_raw <slave> <cmd16> [twice] ──────────────────────────────────────
 * Send an arbitrary 16-bit DALI forward frame to a gateway. cmd16 is the full
 * frame — address/selector byte in the high 8 bits, opcode/data in the low 8
 * bits — and accepts decimal or 0x-hex. It is submitted through the gateway
 * command queue (dali_queue.c) and blocks for the DALI result. Append "twice"
 * (or "2x") for config commands that DALI requires be repeated within 100 ms
 * (e.g. STORE_DTR_AS_*, SET_SCENE); a single send opens the backward-frame
 * window and reports the DALI response (ok/nak/err). */
static void cmd_dali_raw(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    char *cs = strtok_r(NULL, " \t", saveptr);
    char *ts = strtok_r(NULL, " \t", saveptr);
    long slave, cmd16;
    if (!parse_int_range_auto(ss, 0, 247, &slave))   { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(cs, 0, 65535, &cmd16)) { txt_err("bad_arg"); return; }
    bool twice = false;
    if (ts) {
        if (strcasecmp(ts, "twice") == 0 || strcasecmp(ts, "2x") == 0) {
            twice = true;
        } else {
            txt_err("bad_arg"); return;
        }
    }

    uint16_t value = (uint16_t)cmd16;
    uint16_t result;
    LOG_INFO("cli", "dali_raw slave=%ld frame=0x%04x%s",
             slave, value, twice ? " twice" : "");
    /* Arbitrary human-supplied frame (could be a non-idempotent UP/DOWN step):
     * pass idempotent=false so a gateway reboot mid-command DROPS it (reports the
     * error) rather than resubmitting and risking a double-apply. (A plain
     * transport timeout is already safe — the driver only resends a command that
     * did not land.) */
    if (!dali_raw_result((uint8_t)slave, value, twice, false, &result)) return;
    /* Let dirty-poll pick up any resulting ballast state change. */
    dirty_poll_mark_active((uint8_t)slave);
    /* Elapsed time is the point of this command on the bench: it is a USER-class
     * submit, so it measures exactly the latency a button press would see under
     * whatever load the bus is currently carrying. */
    if ((result >> 8) == 0x00u)
        printf("ok: result 0x%04X (response byte 0x%02X) in %lu.%02lu ms\n",
               result, result & 0xFFu, (unsigned long)(s_dali_raw_us / 1000u),
               (unsigned long)((s_dali_raw_us % 1000u) / 10u));
    else
        printf("ok: result 0x%04X (%s) in %lu.%02lu ms\n", result,
               dali_result_str(result), (unsigned long)(s_dali_raw_us / 1000u),
               (unsigned long)((s_dali_raw_us % 1000u) / 10u));
}

/* ── read_mem <slave> <addr> <bank> <offset> <count> ───────────────────────
 * Read raw bytes from a DALI ballast's memory bank via raw commands.  Prints
 * the bytes as a hex string, or "collision" if the address has 2+ gear. */
static void cmd_read_mem(char **saveptr) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *ds = strtok_r(NULL, " \t", saveptr);
    char *bs = strtok_r(NULL, " \t", saveptr);
    char *os = strtok_r(NULL, " \t", saveptr);
    char *cs = strtok_r(NULL, " \t", saveptr);
    long slave, addr, bank, offset, count;
    if (!parse_int_range_auto(as, 0, 247, &slave))  { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(ds, 0, 63,  &addr))   { txt_err("bad_arg"); return; }
    if (!parse_int_range_auto(bs, 0, 255, &bank))   { txt_err("bad_arg"); return; }
    if (!parse_int_range_auto(os, 0, 255, &offset)) { txt_err("bad_arg"); return; }
    if (!parse_int_range_auto(cs, 1, 64,  &count))  { txt_err("bad_arg"); return; }
    static uint8_t buf[64];
    bool collision, absent;
    LOG_INFO("cli", "read_mem slave=%ld addr=%ld bank=%ld off=%ld count=%ld",
             slave, addr, bank, offset, count);
    if (!dali_read_mem((uint8_t)slave, (uint8_t)addr, (uint8_t)bank,
                       (uint8_t)offset, (uint8_t)count, buf, &collision, &absent))
        return;
    if (collision) { printf("collision: address %ld has multiple devices\n", addr); return; }
    if (absent)    { txt_err("no_response"); return; }
    printf("mem %ld.%ld bank %ld off %ld: ", slave, addr, bank, offset);
    for (long i = 0; i < count; i++) printf("%02X", buf[i]);
    printf("\n");
}

/* ── identify <slave> <addr> ───────────────────────────────────────────────
 * Fetch a ballast's GTIN (memory bank 0, offset 0x03, 6 bytes) and serial /
 * identification number (offset 0x0B, 8 bytes) and print them as hex.  A
 * garbled read = a collision (two+ gear sharing the address). */
static void cmd_identify(char **saveptr) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *ds = strtok_r(NULL, " \t", saveptr);
    long slave, addr;
    if (!parse_int_range_auto(as, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(ds, 0, 63,  &addr))  { txt_err("bad_arg"); return; }
    uint8_t gtin[6], serial[8];
    bool col, absent;
    LOG_INFO("cli", "identify slave=%ld addr=%ld", slave, addr);
    if (!dali_read_mem((uint8_t)slave, (uint8_t)addr, 0, 0x03u, 6, gtin, &col, &absent)) return;
    if (col)    { printf("collision: address %ld has multiple devices\n", addr); return; }
    if (absent) { txt_err("no_response"); return; }
    if (!dali_read_mem((uint8_t)slave, (uint8_t)addr, 0, 0x0Bu, 8, serial, &col, &absent)) return;
    if (col)    { printf("collision: address %ld has multiple devices\n", addr); return; }
    if (absent) { txt_err("no_response"); return; }
    printf("identify %ld.%ld: gtin=", slave, addr);
    for (int i = 0; i < 6; i++) printf("%02X", gtin[i]);
    printf(" serial=");
    for (int i = 0; i < 8; i++) printf("%02X", serial[i]);
    printf("\n");
}

/* ── blink <slave> <addr> ──────────────────────────────────────────────────
 * Flash one ballast (DAPC full/off, ~8 toggles) so it can be located
 * physically — DALI control gear has no IDENTIFY command, so we just pulse its
 * level.  Leaves the light on at full. */
static void cmd_blink(char **saveptr) {
    char *as = strtok_r(NULL, " \t", saveptr);
    char *ds = strtok_r(NULL, " \t", saveptr);
    long slave, addr;
    if (!parse_int_range_auto(as, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(ds, 0, 63,  &addr))  { txt_err("bad_arg"); return; }
    uint16_t r;
    // DAPC (direct arc power): address byte = (addr<<1) with bit0=0, data = level.
    uint16_t dapc = (uint16_t)((uint16_t)((addr << 1) & 0x7Eu) << 8);
    LOG_INFO("cli", "blink slave=%ld addr=%ld", slave, addr);
    for (int i = 0; i < 8; i++) {
        uint8_t level = (i & 1) ? 0x00u : 0xFEu;
        /* DAPC is idempotent (re-sending sets the same level). */
        if (!dali_raw_result((uint8_t)slave, (uint16_t)(dapc | level), false, true, &r)) return;
        sleep_ms(300);
    }
    dali_raw_result((uint8_t)slave, (uint16_t)(dapc | 0xFEu), false, true, &r);  // leave on (DAPC, idempotent)
    dirty_poll_mark_active((uint8_t)slave);
    txt_ok();
}

/* ── gw_reboot <slave> ─────────────────────────────────────────────────────
 * Force a TinyModbus gateway to perform a software reset by writing the magic
 * value 0xA5A5 to its SYSTEM_RESET register (0x1F05). Recovery hatch for a
 * gateway wedged in a stuck-busy DALI state (raw channel returning exception
 * 0x06 for every command). The gateway ACKs, then reboots and is unresponsive
 * for ~1 s before it re-runs its own bus scan. */
static void cmd_gw_reboot(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    long slave;
    if (!parse_int_range_auto(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    static cli_mb_ctx_t ctx;
    memset(&ctx, 0, sizeof(ctx));
    LOG_INFO("cli", "gw_reboot slave=%ld", slave);
    /* TinyModbus MBREG_SYSTEM_RESET / MBREG_SYSTEM_RESET_MAGIC (registers.h). */
    modbus_set_holding_register((uint8_t)slave, 0x1F05u, 0xA5A5u, cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
    txt_ok();
}

/* `dq [slave]` — dump the command-queue client's per-slave state + counters
 * (dev-grade; for the bench fault drills). Bare = all DALI slaves. */
static void cmd_dq(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    long slave = -1;
    if (ss && !parse_int_range_auto(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    dq_debug_print((int)slave);
}

/* `busload [reset]` — who is using the RS-485 bus. Prints the share of wall time
 * the bus was occupied since the last reset, split per slave. The point of the
 * split is that a slave being retried into the ground (a dead board, three
 * attempts a poll) costs the same bus time as real work, and that cost is
 * invisible in every other view. */
static void cmd_busload(char **saveptr) {
    char *a1 = strtok_r(NULL, " \t", saveptr);
    if (a1 && strcasecmp(a1, "reset") == 0) { modbus_busload_reset(); txt_ok(); return; }

    uint64_t win = 0, busy = 0;
    uint32_t txns = 0;
    modbus_busload_slave_t per[MODBUS_BUSLOAD_SLAVES];
    modbus_busload_get(&win, &busy, &txns, per);
    if (win == 0) win = 1;

    uint32_t rt = 0, rb = 0;
    modbus_busload_retries(&rt, &rb);

    /* Queue-overflow drops. Both counters are incremented on hot paths that must
     * not log (the button PIO/IRQ path and the MQTT process pump), so this is the
     * only place they become visible — until now they were counted and never
     * read by anything, which made a real loss of presses or HA commands
     * completely silent. Non-zero here is always worth investigating. */
    cdc_out_printf("busload: window %lu.%02lu s, bus busy %lu%% (%lu txns, %lu.%02lu ms avg)\n",
           (unsigned long)(win / 1000000u), (unsigned long)((win % 1000000u) / 10000u),
           (unsigned long)(busy * 100u / win), (unsigned long)txns,
           (unsigned long)(txns ? (busy / txns) / 1000u : 0u),
           (unsigned long)(txns ? ((busy / txns) % 1000u) / 10u : 0u));
    uint32_t ta = 0, tm = 0, tn = 0;
    modbus_busload_turnaround(&ta, &tm, &tn);
    uint32_t skipped = 0; uint8_t nbo = 0;
    modbus_busload_backoff(&skipped, &nbo);
    cdc_out_printf("  retries: %lu timeout, %lu device-busy\n",
           (unsigned long)rt, (unsigned long)rb);
    cdc_out_printf("  backoff: %u slave(s) backed off, %lu txns skipped (bus untouched)\n",
           nbo, (unsigned long)skipped);
    cdc_out_printf("  turnaround: avg %lu.%02lu ms, max %lu.%02lu ms over %lu replies "
           "(timeout is %u ms)\n",
           (unsigned long)(ta / 1000u), (unsigned long)((ta % 1000u) / 10u),
           (unsigned long)(tm / 1000u), (unsigned long)((tm % 1000u) / 10u),
           (unsigned long)tn, 50u);
    for (uint8_t i = 0; i < MODBUS_BUSLOAD_SLAVES; i++) {
        if (!per[i].txns) continue;
        cdc_out_printf("  slave %-3u %5lu txns  %3lu%% of bus  %lu.%02lu ms avg\n",
               per[i].slave, (unsigned long)per[i].txns,
               (unsigned long)(per[i].us * 100u / win),
               (unsigned long)((per[i].us / per[i].txns) / 1000u),
               (unsigned long)(((per[i].us / per[i].txns) % 1000u) / 10u));
    }
    /* Printed unconditionally, including when zero: a line that only appears on
     * failure cannot be told apart from a counter nobody wired up, which is what
     * these two were until now. Seeing 0 is the point. Both are incremented on
     * hot paths that must not log (the button PIO/IRQ path, the MQTT pump), so
     * this is the only place they surface. */
    cdc_out_printf("  queue drops: %lu button event(s), %lu HA command(s)\n",
                   (unsigned long)button_raw_drop_count(),
                   (unsigned long)ha_bridge_cmd_drop_count());
}

/* `soak [<slave> <addr> [n=<pairs>] [depth=<1-8>]] | stop` — bench queue soak.
 * Bare = report the running/last run. Hammers the gateway command queue with
 * alternating SET DTR0 / QUERY CONTENT DTR0 pairs and verifies every answer
 * against the value its paired write put there (see dq_soak.h). */
static void cmd_soak(char **saveptr) {
    char *a1 = strtok_r(NULL, " \t", saveptr);
    if (!a1) { dq_soak_report(); return; }
    if (strcasecmp(a1, "stop") == 0) {
        if (!dq_soak_active()) { txt_err("not_running"); return; }
        dq_soak_stop();
        txt_ok();
        return;
    }
    long slave, addr;
    if (!parse_int_range_auto(a1, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (state_cache_slave_idx((uint8_t)slave) < 0) { txt_err("invalid_slave"); return; }
    char *a2 = strtok_r(NULL, " \t", saveptr);
    if (!parse_int_range_auto(a2, 0, 63, &addr)) { txt_err("invalid_addr"); return; }

    long pairs = 0, depth = SOAK_DEPTH_DEFAULT;
    soak_path_t path = SOAK_PATH_DIRECT;
    for (char *t = strtok_r(NULL, " \t", saveptr); t; t = strtok_r(NULL, " \t", saveptr)) {
        char *eq = strchr(t, '=');
        if (!eq) { txt_err("bad_arg"); return; }
        *eq = '\0';
        if      (strcasecmp(t, "n") == 0)     { if (!parse_int_range_auto(eq + 1, 0, 10000000, &pairs)) { txt_err("bad_n"); return; } }
        else if (strcasecmp(t, "depth") == 0) { if (!parse_int_range_auto(eq + 1, 1, 8, &depth)) { txt_err("bad_depth"); return; } }
        else if (strcasecmp(t, "path") == 0) {
            if      (strcasecmp(eq + 1, "direct") == 0) path = SOAK_PATH_DIRECT;
            else if (strcasecmp(eq + 1, "bg") == 0)     path = SOAK_PATH_BG;
            else if (strcasecmp(eq + 1, "user") == 0)   path = SOAK_PATH_USER;
            else { txt_err("bad_path"); return; }
        }
        else { txt_err("bad_arg"); return; }
    }
    if (!dq_soak_start((uint8_t)slave, (uint8_t)addr, (uint8_t)depth, (uint32_t)pairs, path)) {
        txt_err("already_running");
        return;
    }
    txt_ok();
}

/* ── dim <slave> <addr|gN> {up|down} ───────────────────────────────────── */
static void cmd_dim(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    char *as = strtok_r(NULL, " \t", saveptr);
    char *vs = strtok_r(NULL, " \t", saveptr);
    int slave = 0;
    if (!parse_int_range(ss, 0, 247, &slave)) {
        txt_err("invalid_slave"); return;
    }
    if (state_cache_slave_idx((uint8_t)slave) < 0) {
        txt_err("invalid_slave"); return;
    }
    if (!as || !vs) { txt_err("bad_arg"); return; }
    bool up;
    if (strcasecmp(vs, "up") == 0)        up = true;
    else if (strcasecmp(vs, "down") == 0) up = false;
    else { txt_err("bad_arg"); return; }

    bool is_group = (as[0] == 'g' || as[0] == 'G');
    if (is_group) {
        int group = 0;
        if (!parse_int_range(as + 1, 0, 15, &group)) {
            txt_err("out_of_range"); return;
        }
        LOG_INFO("cli", "dim g%d on slave %d %s", group, slave, up ? "up" : "down");
        lighting_group_step((uint8_t)slave, (uint8_t)group, up);
    } else {
        int addr = 0;
        if (!parse_int_range(as, 0, 63, &addr)) {
            txt_err("out_of_range"); return;
        }
        LOG_INFO("cli", "dim %d.%d %s", slave, addr, up ? "up" : "down");
        lighting_dali_step((uint8_t)slave, (uint8_t)addr, up);
    }
    dirty_poll_mark_active((uint8_t)slave);
    txt_ok();
}

/* ── provision <slave> {all|new|<addr>} | provision abort ─────────────── */
static void cmd_provision(char **saveptr) {
    char *first = strtok_r(NULL, " \t", saveptr);
    if (!first) { txt_err("bad_arg"); return; }

    if (strcasecmp(first, "abort") == 0) {
        LOG_INFO("cli", "provision abort");
        dali_provision_abort();
        txt_ok();
        return;
    }

    int slave = 0;
    if (!parse_int_range(first, 0, 247, &slave)) {
        txt_err("invalid_slave"); return;
    }
    if (state_cache_slave_idx((uint8_t)slave) < 0) {
        txt_err("invalid_slave"); return;
    }
    char *modeS = strtok_r(NULL, " \t", saveptr);
    if (!modeS) { txt_err("bad_arg"); return; }

    prov_mode_t mode;
    uint8_t target = 0;
    if (strcasecmp(modeS, "all") == 0) {
        mode = PROV_ALL;
    } else if (strcasecmp(modeS, "new") == 0) {
        mode = PROV_UNADDRESSED;
    } else {
        int addr = 0;
        if (!parse_int_range(modeS, 0, 63, &addr)) {
            txt_err("bad_arg"); return;
        }
        mode = PROV_ADDRESS;
        target = (uint8_t)addr;
    }

    if (dali_provision_busy()) {
        txt_err("provision_busy"); return;
    }
    LOG_INFO("cli", "provision slave=%d mode=%s target=%u", slave, modeS, target);
    bool started = dali_provision_start((uint8_t)slave, mode, target);
    if (!started) { txt_err("provision_busy"); return; }
    txt_ok();
}

/* ── scan <slave> ──────────────────────────────────────────────────────── */
static void cmd_scan(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    int slave = 0;
    if (!parse_int_range(ss, 0, 247, &slave)) {
        txt_err("invalid_slave"); return;
    }
    int idx = state_cache_slave_idx((uint8_t)slave);
    if (idx < 0) { txt_err("invalid_slave"); return; }

    /* Asynchronous. This used to block until the interrogation finished, which
     * on a populated bus is many seconds — and unlike boot, `scan` gets typed on
     * a LIVE system where someone may be at a wall switch. Blocking here stalls
     * the main loop, so buttons stop responding for the duration. Arm it and let
     * the main loop drive it; `dstate` shows progress and a scan-complete event
     * goes to HA when it finishes. */
    LOG_INFO("cli", "scan slave %d (async)", slave);
    dirty_poll_arm_scan((uint8_t)slave);
    printf("scanning slave %d — async; watch `dstate %d`\n", slave, slave);
}

/* ── discover ──────────────────────────────────────────────────────────── */
/* Re-runs bus_discover_slave() across every configured slave. Blocking —
 * worst case ~4 s per slave (three 1 s pings + a set-baud + settle). We
 * pet the watchdog between slaves so each gets a fresh 5 s window. */
static void cmd_discover(char **saveptr) {
    (void)saveptr;
    int ok = 0, fail = 0;
    LOG_INFO("cli", "discover: relay_count=%u dali_count=%u",
             g_config.relay_slave_count, g_config.dali_slave_count);
    printf("discovering %u relay + %u dali slaves...\n",
           g_config.relay_slave_count, g_config.dali_slave_count);

    for (int i = 0; i < g_config.relay_slave_count; i++) {
        watchdog_update();
        uint8_t id = g_config.relay_slave_ids[i];
        bool r = bus_discover_slave(id, SLAVE_KIND_RELAY);
        printf("  slave %u relay: %s\n", id, r ? "ok" : "fail");
        if (r) ok++; else fail++;
    }
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        watchdog_update();
        uint8_t id = g_config.dali_slave_ids[i];
        bool r = bus_discover_slave(id, SLAVE_KIND_DALI);
        printf("  slave %u dali: %s\n", id, r ? "ok" : "fail");
        if (r) ok++; else fail++;
    }
    watchdog_update();
    printf("discover complete: %d ok, %d fail\n", ok, fail);
}

/* ── ping / set_slave_baud (bus diagnostics) ───────────────────────────── */
/* Resolve an optional "relay"|"dali" token (NULL → infer from the configured
 * slave lists). On failure prints the err: line and returns false. */
static bool resolve_kind(const char *tok, uint8_t slave, slave_kind_t *out) {
    if (tok) {
        if      (strcasecmp(tok, "relay") == 0) *out = SLAVE_KIND_RELAY;
        else if (strcasecmp(tok, "dali")  == 0) *out = SLAVE_KIND_DALI;
        else { txt_err("bad_arg"); return false; }
        return true;
    }
    *out = slave_kind(slave);
    if (*out == SLAVE_KIND_UNKNOWN) { txt_err("unknown_kind"); return false; }
    return true;
}

/* ping <slave> [relay|dali] — single ping at the current master baud. */
static void cmd_ping(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    int slave = 0;
    if (!parse_int_range(ss, 1, 247, &slave)) { txt_err("invalid_slave"); return; }

    slave_kind_t kind;
    if (!resolve_kind(strtok_r(NULL, " \t", saveptr), (uint8_t)slave, &kind)) return;

    const char *ks = (kind == SLAVE_KIND_RELAY) ? "relay" : "dali";
    unsigned long baud = (unsigned long)modbus_get_baud();
    LOG_INFO("cli", "ping slave %d (%s) at %lu", slave, ks, baud);
    bool ok = bus_ping_slave((uint8_t)slave, kind);
    printf("ping slave %d (%s) at %lu: %s\n", slave, ks, baud,
           ok ? "ok" : "no_response");
}

/* set_slave_baud <slave> {9600|19200|38400} [relay|dali] — tell a slave to switch
 * its own UART baud. The master UART is left as-is; use 'baud' to match afterwards. */
static void cmd_set_slave_baud(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    char *bs = strtok_r(NULL, " \t", saveptr);
    int  slave = 0;
    long baud  = 0;
    if (!parse_int_range(ss, 1, 247, &slave)) { txt_err("invalid_slave"); return; }
    if (!parse_int_range_auto(bs, 300, 921600, &baud)) { txt_err("invalid_baud"); return; }
    if (baud != 9600 && baud != 19200 && baud != 38400) { txt_err("unsupported_baud"); return; }

    slave_kind_t kind;
    if (!resolve_kind(strtok_r(NULL, " \t", saveptr), (uint8_t)slave, &kind)) return;

    const char *ks = (kind == SLAVE_KIND_RELAY) ? "relay" : "dali";
    LOG_INFO("cli", "set_slave_baud slave %d (%s) -> %ld", slave, ks, baud);
    bool ok = bus_set_slave_baud((uint8_t)slave, kind, (uint32_t)baud);
    printf("set_slave_baud slave %d (%s) -> %ld: %s\n", slave, ks, baud,
           ok ? "ok" : "fail");
    if (ok) {
        printf("  slave now at %ld; master still at %lu — run 'baud %ld' to match\n",
               baud, (unsigned long)modbus_get_baud(), baud);
    }
}

/* ── baud ──────────────────────────────────────────────────────────────── */
static void cmd_baud(char **saveptr) {
    char *bs = strtok_r(NULL, " \t", saveptr);
    if (!bs) {
        printf("baud: %lu\n", (unsigned long)modbus_get_baud());
        return;
    }
    long baud = 0;
    if (!parse_int_range_auto(bs, 300, 921600, &baud)) {
        txt_err("invalid_baud");
        return;
    }
    char *sv = strtok_r(NULL, " \t", saveptr);
    bool persist = sv && strcasecmp(sv, "save") == 0;

    LOG_INFO("cli", "baud %ld%s", baud, persist ? " save" : "");
    modbus_uart_set_baud_runtime((uint32_t)baud);
    if (persist) modbus_set_baud((uint32_t)baud);
    printf("baud: %lu%s\n", (unsigned long)modbus_get_baud(),
           persist ? " (saved)" : " (live only)");
}

/* ── rs485 transmit (DE) mode ──────────────────────────────────────────── */
static void cmd_rs485(char **saveptr) {
    char *arg = strtok_r(NULL, " \t", saveptr);
    if (!arg) {
        printf("rs485: %s\n", modbus_de_mode_to_str(modbus_get_de_mode()));
        return;
    }
    for (modbus_de_mode_t m = 0; m < MODBUS_DE_MAX; m++) {
        if (strcasecmp(arg, modbus_de_mode_to_str(m)) == 0) {
            LOG_INFO("cli", "rs485 %s", modbus_de_mode_to_str(m));
            set_modbus_de_mode(m);  /* applies live + persists */
            printf("rs485: %s (saved)\n", modbus_de_mode_to_str(m));
            return;
        }
    }
    txt_err("bad_arg");
}

/* ── ls / cat (pico-vfs littlefs) ──────────────────────────────────────── */
static void cmd_ls(char **saveptr) {
    const char *path = strtok_r(NULL, " \t", saveptr);
    if (!path) path = "/";

    DIR *d = opendir(path);
    if (!d) { txt_err("opendir_failed"); return; }

    int count = 0;
    struct dirent *e;
    while ((e = readdir(d)) != NULL) {
        if (e->d_name[0] == '\0') break;
        if (strcmp(e->d_name, ".") == 0 || strcmp(e->d_name, "..") == 0) continue;

        if (e->d_type == DT_DIR) {
            printf("  %s/\n", e->d_name);
        } else {
            /* d_type doesn't carry size — stat the full path */
            char full[320];
            int plen = strlen(path);
            bool need_slash = plen > 0 && path[plen - 1] != '/';
            snprintf(full, sizeof(full), "%s%s%s",
                     path, need_slash ? "/" : "", e->d_name);
            struct stat st;
            if (stat(full, &st) == 0) {
                printf("  %-32s %lu\n", e->d_name, (unsigned long)st.st_size);
            } else {
                printf("  %-32s ?\n", e->d_name);
            }
        }
        count++;
    }
    closedir(d);
    if (count == 0) printf("(empty)\n");
}

static void cmd_cat(char **saveptr) {
    const char *path = strtok_r(NULL, " \t", saveptr);
    if (!path) { txt_err("bad_arg"); return; }

    FILE *fp = fopen(path, "rb");
    if (!fp) { txt_err("fopen_failed"); return; }

    char buf[128];
    size_t total = 0;
    char last = 0;
    size_t n;
    while ((n = fread(buf, 1, sizeof(buf), fp)) > 0) {
        fwrite(buf, 1, n, stdout);
        last = buf[n - 1];
        total += n;
        watchdog_update();
    }
    fclose(fp);
    if (total == 0 || last != '\n') putchar('\n');
}

/* ── map set / map del ─────────────────────────────────────────────────── */
/* map <fixture> <button> [targets=<s>:<a>[,<s>:<a>...]] — bare = get (emit
 * config_mapping); `targets=` (empty) deletes. Address a may be N or gN. */
static void cmd_map(char **saveptr) {
    char *fs = strtok_r(NULL, " \t", saveptr);
    char *bs = strtok_r(NULL, " \t", saveptr);
    int fix = 0, btn = 0;
    if (!parse_int_range(fs, 0, MAX_FIXTURES - 1, &fix) ||
        !parse_int_range(bs, 0, MAX_BUTTONS - 1, &btn)) { txt_err("bad_arg"); return; }

    char *kv = strtok_r(NULL, " \t", saveptr);
    if (!kv) {   /* get */
        const mapping_record_t *rec = mapping_lookup((uint8_t)fix, (uint8_t)btn);
        emit_config_mapping((uint8_t)fix, (uint8_t)btn,
                            rec ? rec->targets : NULL, rec ? rec->target_count : 0);
        return;
    }
    char *eq = strchr(kv, '=');
    if (!eq) { txt_err("bad_arg expected key=value"); return; }
    *eq = '\0';
    if (strcasecmp(kv, "targets") != 0) { txt_err("bad_field"); return; }
    char *list = eq + 1;
    if (*list == '\0') {   /* targets= (empty) → delete */
        if (!mapping_delete((uint8_t)fix, (uint8_t)btn)) { txt_err("save_failed"); return; }
        emit_config_mapping((uint8_t)fix, (uint8_t)btn, NULL, 0);
        return;
    }
    mapping_target_t targets[MAX_TARGETS_PER_BTN];
    uint8_t count = 0;
    char *pair_save = NULL;
    for (char *p = strtok_r(list, ",", &pair_save);
         p && count < MAX_TARGETS_PER_BTN;
         p = strtok_r(NULL, ",", &pair_save)) {
        if (!parse_target_pair(p, &targets[count])) { txt_err("bad_arg"); return; }
        count++;
    }
    if (!mapping_set((uint8_t)fix, (uint8_t)btn, targets, count)) { txt_err("save_failed"); return; }
    emit_config_mapping((uint8_t)fix, (uint8_t)btn, targets, count);
}

/* ── reboot ────────────────────────────────────────────────────────────── */
/* Shared reboot path: the local `reboot` CLI command and a remote IC_CMD_REBOOT
 * (MQTT/web) both land here. `src` names the trigger for the log. The short
 * sleep lets the USB CDC line drain and gives core 1 time to flush its network
 * ack (HTTP 200 / MQTT) before the watchdog resets the whole chip. Never returns.
 * This is the out-of-band recovery for a wedged USB stack: it does not touch USB,
 * so it works even when the CLI/picotool reset are dead. */
static void controller_reboot(const char *src) {
    LOG_INFO("cli", "reboot (%s)", src);
    printf("rebooting...\n");
    sleep_ms(150);
    watchdog_enable(200, 0);
    while (1) { ; }
}

static void cmd_reboot(void) {
    controller_reboot("cli");
}

/* ── echo ──────────────────────────────────────────────────────────────── */
static void cmd_echo(char **saveptr) {
    char *arg = strtok_r(NULL, " \t", saveptr);
    if (!arg) {
        printf("echo %s\n", echo_on ? "on" : "off");
        return;
    }
    int v = parse_on_off_toggle(arg);
    if (v < 0 || v == 2) { txt_err("bad_arg"); return; }
    echo_on = (v == 1);
    LOG_INFO("cli", "echo %s", echo_on ? "on" : "off");
    txt_ok();
}

/* ── mqtt / net config CLI ─────────────────────────────────────────────── */
static bool parse_ip4(const char *s, uint8_t ip[4]) {
    unsigned a, b, c, d;
    if (sscanf(s, "%u.%u.%u.%u", &a, &b, &c, &d) != 4) return false;
    if (a > 255 || b > 255 || c > 255 || d > 255) return false;
    ip[0] = (uint8_t)a; ip[1] = (uint8_t)b; ip[2] = (uint8_t)c; ip[3] = (uint8_t)d;
    return true;
}

/* ── network ping (core 1 runs the ICMP echo; replies arrive asynchronously as
 *    IC_CMD_PING_REPLY and print here). One run in flight at a time. ───────── */
static struct {
    bool     active;
    uint8_t  ip[4];
    uint8_t  count;        /* echoes requested */
    uint8_t  seen;         /* outcomes printed (reply + timeout) */
    uint8_t  recv;         /* successful replies */
    uint32_t deadline_ms;  /* give up waiting on core 1 past this */
} s_cli_ping;

static void cli_ping_summary(void) {
    int lost = (int)s_cli_ping.count - (int)s_cli_ping.recv;
    printf("%u sent, %u received, %d lost\n",
           s_cli_ping.count, s_cli_ping.recv, lost < 0 ? 0 : lost);
    s_cli_ping.active = false;
}

static void cli_ping_start(const uint8_t ip[4], int count) {
    if (s_cli_ping.active) { txt_err("ping_busy"); return; }
    ic_event_t ev; memset(&ev, 0, sizeof ev);
    ev.type = IC_EVT_PING_REQ;
    memcpy(ev.ping_req.ip, ip, 4);
    ev.ping_req.count = (uint8_t)count;
    if (!intercore_push_event(&ev)) { txt_err("queue_full"); return; }
    s_cli_ping.active = true;
    memcpy(s_cli_ping.ip, ip, 4);
    s_cli_ping.count = (uint8_t)count;
    s_cli_ping.seen  = 0;
    s_cli_ping.recv  = 0;
    /* each echo waits up to ~1 s on core 1, plus slack for queue/round-trip. */
    s_cli_ping.deadline_ms =
        to_ms_since_boot(get_absolute_time()) + (uint32_t)count * 1200u + 2000u;
    printf("PING %u.%u.%u.%u:\n", ip[0], ip[1], ip[2], ip[3]);
}

static void cli_ping_on_reply(const ic_cmd_t *c) {
    if (!s_cli_ping.active) return;   /* late/duplicate after a summary */
    switch ((ic_ping_status_t)c->ping_reply.status) {
    case IC_PING_BUSY:
        printf("  busy (network core couldn't borrow a socket — retry)\n");
        s_cli_ping.active = false;
        return;
    case IC_PING_ERROR:
        printf("  error (socket/send failed)\n");
        s_cli_ping.active = false;
        return;
    case IC_PING_REPLY:
        printf("  seq=%u  reply  rtt=%ums\n", c->ping_reply.seq, c->ping_reply.rtt_ms);
        s_cli_ping.recv++;
        s_cli_ping.seen++;
        break;
    case IC_PING_TIMEOUT:
        printf("  seq=%u  timeout\n", c->ping_reply.seq);
        s_cli_ping.seen++;
        break;
    }
    if (s_cli_ping.seen >= s_cli_ping.count) cli_ping_summary();
}

/* Called from serial_poll: bail out of a stuck run if core 1 never replies. */
static void cli_ping_poll(void) {
    if (!s_cli_ping.active) return;
    if ((int32_t)(to_ms_since_boot(get_absolute_time()) - s_cli_ping.deadline_ms) >= 0) {
        printf("  (no response from network core)\n");
        cli_ping_summary();
    }
}

static void cmd_mqtt(char **sp) {
    char *sub = strtok_r(NULL, " \t", sp);
    if (!sub || strcasecmp(sub, "show") == 0) {
        printf("mqtt host=%s port=%u user=%s prefix=%s creds=%s\n",
               g_config.mqtt_host, g_config.mqtt_port, g_config.mqtt_user,
               g_config.mqtt_prefix, g_config.mqtt_user[0] ? "yes" : "no");
        return;
    }
    if (strcasecmp(sub, "set") != 0) { txt_err("bad_arg"); return; }
    char *tok; bool any = false;
    while ((tok = strtok_r(NULL, " \t", sp))) {
        char *eq = strchr(tok, '='); if (!eq) continue;
        *eq = '\0'; const char *k = tok, *v = eq + 1;
        if      (!strcasecmp(k, "host"))   { strncpy(g_config.mqtt_host,   v, sizeof g_config.mqtt_host - 1);   any = true; }
        else if (!strcasecmp(k, "port"))   { g_config.mqtt_port = (uint16_t)atoi(v);                            any = true; }
        else if (!strcasecmp(k, "user"))   { strncpy(g_config.mqtt_user,   v, sizeof g_config.mqtt_user - 1);   any = true; }
        else if (!strcasecmp(k, "pass"))   { strncpy(g_config.mqtt_pass,   v, sizeof g_config.mqtt_pass - 1);   any = true; }
        else if (!strcasecmp(k, "prefix")) { strncpy(g_config.mqtt_prefix, v, sizeof g_config.mqtt_prefix - 1); any = true; }
        else { txt_err("bad_key"); return; }
    }
    if (!any) { txt_err("no_fields"); return; }
    if (!config_save()) { txt_err("save_failed"); return; }
    printf("ok (reboot to apply)\n");
}

/* A DHCP/DNS hostname is one label: 1..31 chars, letters/digits/hyphen, and not
 * leading/trailing a hyphen (RFC 1123). Reject anything else so we never put an
 * illegal label on the wire (which some DHCP servers drop silently). */
static bool hostname_ok(const char *s) {
    size_t n = strlen(s);
    if (n < 1 || n > 31) return false;
    if (s[0] == '-' || s[n - 1] == '-') return false;
    for (size_t i = 0; i < n; i++) {
        char ch = s[i];
        bool good = (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z') ||
                    (ch >= '0' && ch <= '9') || ch == '-';
        if (!good) return false;
    }
    return true;
}

static void cmd_net(char **sp) {
    char *sub = strtok_r(NULL, " \t", sp);
    if (!sub || strcasecmp(sub, "show") == 0) {
        /* The LIVE address first — under DHCP the configured static fields are
         * all zeros, so printing only those reported 0.0.0.0 for a device with a
         * perfectly good lease. */
        uint8_t lip[4];
        core1_live_ip(lip);
        printf("net addr=%u.%u.%u.%u%s\n", lip[0], lip[1], lip[2], lip[3],
               (lip[0] | lip[1] | lip[2] | lip[3]) ? "" : " (no lease yet)");
        printf("net dhcp=%s ip=%u.%u.%u.%u mask=%u.%u.%u.%u gw=%u.%u.%u.%u dns=%u.%u.%u.%u hostname=%s\n",
               g_config.net_dhcp ? "on" : "off",
               g_config.net_ip[0], g_config.net_ip[1], g_config.net_ip[2], g_config.net_ip[3],
               g_config.net_mask[0], g_config.net_mask[1], g_config.net_mask[2], g_config.net_mask[3],
               g_config.net_gw[0], g_config.net_gw[1], g_config.net_gw[2], g_config.net_gw[3],
               g_config.net_dns[0], g_config.net_dns[1], g_config.net_dns[2], g_config.net_dns[3],
               g_config.net_hostname);
        return;
    }
    if (strcasecmp(sub, "ping") == 0) {
        uint8_t ip[4];
        char *ips = strtok_r(NULL, " \t", sp);
        if (!ips || !parse_ip4(ips, ip)) { txt_err("bad_ip"); return; }
        int count = 4;
        char *cs = strtok_r(NULL, " \t", sp);
        if (cs && !parse_int_range(cs, 1, 16, &count)) { txt_err("bad_count"); return; }
        cli_ping_start(ip, count);
        return;
    }
    if (strcasecmp(sub, "set") != 0) { txt_err("bad_arg"); return; }
    char *tok; bool any = false;
    while ((tok = strtok_r(NULL, " \t", sp))) {
        char *eq = strchr(tok, '='); if (!eq) continue;
        *eq = '\0'; const char *k = tok, *v = eq + 1;
        if (!strcasecmp(k, "dhcp")) {
            int o = parse_on_off_toggle(v);
            if (o != 0 && o != 1) { txt_err("bad_arg"); return; }
            g_config.net_dhcp = (uint8_t)o; any = true;
        }
        else if (!strcasecmp(k, "ip"))   { if (!parse_ip4(v, g_config.net_ip))   { txt_err("bad_ip"); return; } any = true; }
        else if (!strcasecmp(k, "mask")) { if (!parse_ip4(v, g_config.net_mask)) { txt_err("bad_ip"); return; } any = true; }
        else if (!strcasecmp(k, "gw"))   { if (!parse_ip4(v, g_config.net_gw))   { txt_err("bad_ip"); return; } any = true; }
        else if (!strcasecmp(k, "dns"))  { if (!parse_ip4(v, g_config.net_dns))  { txt_err("bad_ip"); return; } any = true; }
        else if (!strcasecmp(k, "hostname")) {
            if (!hostname_ok(v)) { txt_err("bad_hostname"); return; }
            strncpy(g_config.net_hostname, v, sizeof g_config.net_hostname - 1);
            g_config.net_hostname[sizeof g_config.net_hostname - 1] = '\0';
            any = true;
        }
        else { txt_err("bad_key"); return; }
    }
    if (!any) { txt_err("no_fields"); return; }
    if (!config_save()) { txt_err("save_failed"); return; }
    printf("ok (reboot to apply)\n");
}

/* ── Top-level text dispatcher ─────────────────────────────────────────── */
static void handle_text_line(const char *original_line) {
    LOG_DBG("cli", "rx: %s", original_line);   /* normal traffic, not an event → DEBUG */

    /* strtok_r mutates the buffer, so copy. SERIAL_LINE_MAX is the upper
     * bound on the line length set by the input buffer. */
    char buf[SERIAL_LINE_MAX];
    strncpy(buf, original_line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *saveptr = NULL;
    char *cmd = strtok_r(buf, " \t", &saveptr);
    if (!cmd) return;

    if (strcasecmp(cmd, "help") == 0) {
        cmd_help();
        return;
    }
    if (strcasecmp(cmd, "show") == 0) {
        cmd_show(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "relay") == 0) {
        cmd_relay(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "btn") == 0) {
        cmd_btn(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "dali") == 0) {
        cmd_dali(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "cfg") == 0) {
        cmd_cfg(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "tombstone") == 0) {
        cmd_tombstone(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "mqtt") == 0) {
        cmd_mqtt(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "net") == 0) {
        cmd_net(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "dali_raw") == 0) {
        cmd_dali_raw(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "read_mem") == 0) {
        cmd_read_mem(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "identify") == 0) {
        cmd_identify(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "blink") == 0) {
        cmd_blink(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "gw_reboot") == 0) {
        cmd_gw_reboot(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "dq") == 0) {
        cmd_dq(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "soak") == 0) {
        cmd_soak(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "dstate") == 0) {
        char *ss2 = strtok_r(NULL, " \t", &saveptr);
        long sl = -1;
        if (ss2 && !parse_int_range_auto(ss2, 0, 247, &sl)) { txt_err("invalid_slave"); return; }
        dali_state_debug_print((int)sl);
        return;
    }
    if (strcasecmp(cmd, "busload") == 0) {
        cmd_busload(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "dim") == 0) {
        cmd_dim(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "provision") == 0) {
        cmd_provision(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "scan") == 0) {
        cmd_scan(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "discover") == 0) {
        cmd_discover(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "ping") == 0) {
        cmd_ping(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "set_slave_baud") == 0) {
        cmd_set_slave_baud(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "baud") == 0) {
        cmd_baud(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "rs485") == 0) {
        cmd_rs485(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "ls") == 0) {
        cmd_ls(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "cat") == 0) {
        cmd_cat(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "map") == 0) {
        cmd_map(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "log") == 0) {
        cmd_log(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "get_input") == 0) {
        cmd_get_input(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "get_holding") == 0) {
        cmd_get_holding(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "set_holding") == 0) {
        cmd_set_holding(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "get_coils") == 0) {
        cmd_get_coils(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "set_coil") == 0) {
        cmd_set_coil(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "echo") == 0) {
        cmd_echo(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "reboot") == 0) {
        cmd_reboot();
        return;
    }
    if (strcasecmp(cmd, "resync") == 0) {
        LOG_INFO("cli", "resync");
        serial_tx_hello();
        lighting_resync_emit();
        serial_tx_config_full();
        txt_ok();
        return;
    }

    LOG_WARN("cli", "parse err: unknown_command for input %s", original_line);
    txt_err("unknown_command");
}

/* ── State event serialisation ───────────────────────────────────────────── */
/* lighting_actions.c defines levt_t internally; we replicate the minimal
 * struct here for serialisation.  In a real project these would share a header. */
typedef enum { LEVT_RELAY, LEVT_DALI, LEVT_PRESENCE, LEVT_FAULT,
               LEVT_SCAN_COMPLETE, LEVT_GROUP, LEVT_GATEWAY, LEVT_GW_DIAG } levt_type_t;
typedef struct {
    levt_type_t type;
    union {
        struct { uint8_t ch; bool on; } relay;
        struct { uint8_t slave_id; uint8_t addr; uint8_t arc; uint8_t status; } dali;
        struct { uint8_t slave_id; uint64_t appeared; uint64_t disappeared; } presence;
        struct { uint8_t slave_id; uint8_t addr; bool lamp_fail; bool power_cycle; } fault;
        struct { uint8_t slave_id; uint64_t present_mask; } scan_complete;
        struct { uint8_t slave_id; uint8_t group; uint8_t bri; bool on; } group;
        struct { uint8_t slave_id; bool online; } gateway;
        struct { uint8_t slave_id; uint32_t uptime_s;
                 uint16_t boot_count; uint8_t reset_reason; } gw_diag;
    };
} levt_t;

/* ── Inter-core seam (core0 -> core1 events, core1 -> core0 commands) ─────────
 * Convert a drained levt_t (the core-0 staging-ring event) to the typed
 * ic_event_t forwarded to core 1. Runs on core 0 in serial_poll(), so it can
 * read state_cache for the bundled DALI config and group membership. Returns
 * false for kinds with no core-1 representation (none currently). */
static bool levt_to_ic(const levt_t *e, ic_event_t *ic) {
    switch (e->type) {
    case LEVT_RELAY:
        ic->type = IC_EVT_RELAY;
        ic->relay.ch = e->relay.ch;
        ic->relay.on = e->relay.on;
        return true;
    case LEVT_DALI: {
        ic->type = IC_EVT_DALI;
        ic->dali.slave  = e->dali.slave_id;
        ic->dali.addr   = e->dali.addr;
        ic->dali.arc    = e->dali.arc;
        ic->dali.status = e->dali.status;
        int didx = state_cache_slave_idx(e->dali.slave_id);
        dali_ballast_state_t *b =
            didx >= 0 ? state_cache_get_dali((uint8_t)didx, e->dali.addr) : NULL;
        ic->dali.min       = b ? b->min_level       : 0;
        ic->dali.max       = b ? b->max_level       : 0;
        ic->dali.power_on  = b ? b->power_on_level   : 0;
        ic->dali.sys_fail  = b ? b->sys_fail_level   : 0;
        ic->dali.fade_rate = b ? b->fade_rate        : 0;
        ic->dali.fade_time = b ? b->fade_time        : 0;
        /* Those six are placeholders until a raw config read has landed. Tell
         * core 1 so it can stay silent rather than publishing zeros as fact. */
        ic->dali.cfg_known = b ? b->cfg_known : false;
        return true;
    }
    case LEVT_PRESENCE:
        ic->type = IC_EVT_PRESENCE;
        ic->presence.slave       = e->presence.slave_id;
        ic->presence.appeared    = e->presence.appeared;
        ic->presence.disappeared = e->presence.disappeared;
        return true;
    case LEVT_FAULT:
        ic->type = IC_EVT_FAULT;
        ic->fault.slave       = e->fault.slave_id;
        ic->fault.addr        = e->fault.addr;
        ic->fault.lamp_fail   = e->fault.lamp_fail;
        ic->fault.power_cycle = e->fault.power_cycle;
        return true;
    case LEVT_SCAN_COMPLETE:
        ic->type = IC_EVT_SCAN_COMPLETE;
        ic->scan_complete.slave        = e->scan_complete.slave_id;
        ic->scan_complete.present_mask = e->scan_complete.present_mask;
        return true;
    case LEVT_GROUP: {
        ic->type = IC_EVT_GROUP;
        ic->group.slave = e->group.slave_id;
        ic->group.group = e->group.group;
        ic->group.bri   = e->group.bri;
        ic->group.on    = e->group.on;
        int sidx = state_cache_slave_idx(e->group.slave_id);
        ic->group.members = sidx >= 0
            ? state_cache_get_dali_group((uint8_t)sidx, e->group.group) : 0;
        return true;
    }
    case LEVT_GATEWAY:
        ic->type = IC_EVT_GATEWAY;
        ic->gateway.slave  = e->gateway.slave_id;
        ic->gateway.online = e->gateway.online;
        return true;
    case LEVT_GW_DIAG:
        ic->type = IC_EVT_GW_DIAG;
        ic->gw_diag.slave        = e->gw_diag.slave_id;
        ic->gw_diag.uptime_s     = e->gw_diag.uptime_s;
        ic->gw_diag.boot_count   = e->gw_diag.boot_count;
        ic->gw_diag.reset_reason = e->gw_diag.reset_reason;
        return true;
    }
    return false;
}

/* Dispatch a command from core 1 through the same domain entry points the JSON
 * and text CLIs use. The tiny value-string reconstruction for set_dali keeps all
 * clamping/handling inside dali_apply_kv() (one place); the cost is negligible
 * and only on infrequent commands. */
static void ic_dispatch_cmd(const ic_cmd_t *c) {
    switch (c->type) {
    case IC_CMD_SCAN: {
        uint8_t slave = c->scan.slave;
        if (state_cache_slave_idx(slave) < 0) {
            LOG_WARN("cli", "ic scan reject: slave=%u not configured", slave); break; }
        /* Same call the `scan` CLI verb makes, and asynchronous for the same
         * reason: on a populated bus an interrogation is many seconds, and this
         * arrives on a LIVE system where someone may be at a wall switch.
         * Blocking here would stall the main loop and freeze the buttons. */
        LOG_INFO("cli", "ic scan slave %u (async)", slave);
        dirty_poll_arm_scan(slave);
        break;
    }
    case IC_CMD_SET_RELAY: {
        uint8_t ch = c->set_relay.ch;
        if (ch >= RELAY_COUNT) { LOG_WARN("cli", "ic set_relay reject: ch=%u", ch); break; }
        if (c->set_relay.on == IC_TRI_TOGGLE && !state_cache_relay_known(ch)) {
            LOG_WARN("cli", "relay %u toggle ignored — state never read", ch);
            break;
        }
        bool on = (c->set_relay.on == IC_TRI_TOGGLE) ? !state_cache_get_relay_on(ch)
                                                     : (c->set_relay.on == IC_TRI_ON);
        lighting_set_relay(ch, on);
        /* No predictive write and no emit: relay_write_cb caches and emits on the
         * write ACK, and a failed write must not leave HA showing the requested
         * state. Emitting here would publish the PRE-command cached value anyway. */
        break;
    }
    case IC_CMD_SET_DALI: {
        uint8_t slave = c->set_dali.slave, addr = c->set_dali.addr;
        if (state_cache_slave_idx(slave) < 0 || addr > 63) {
            LOG_WARN("cli", "ic set_dali reject: slave=%u addr=%u", slave, addr); break; }
        char vb[12];
        bool any = false, emit_now = false;
        if (c->set_dali.fields & IC_DALI_F_ON) {
            const char *v = c->set_dali.on == IC_TRI_TOGGLE ? "toggle"
                          : c->set_dali.on == IC_TRI_ON     ? "true" : "false";
            if (dali_apply_kv(slave, addr, "on", v) == 1) emit_now = true;
            any = true;
        }
        #define IC_DALI_APPLY(flag, key, field)                                  \
            if (c->set_dali.fields & (flag)) {                                   \
                snprintf(vb, sizeof vb, "%u", c->set_dali.field);                \
                if (dali_apply_kv(slave, addr, (key), vb) == 1) emit_now = true; \
                any = true;                                                      \
            }
        IC_DALI_APPLY(IC_DALI_F_BRI,       "bri",       bri);
        IC_DALI_APPLY(IC_DALI_F_MIN,       "min",       min);
        IC_DALI_APPLY(IC_DALI_F_MAX,       "max",       max);
        IC_DALI_APPLY(IC_DALI_F_POWER_ON,  "power_on",  power_on);
        IC_DALI_APPLY(IC_DALI_F_SYS_FAIL,  "sys_fail",  sys_fail);
        IC_DALI_APPLY(IC_DALI_F_FADE_RATE, "fade_rate", fade_rate);
        IC_DALI_APPLY(IC_DALI_F_FADE_TIME, "fade_time", fade_time);
        #undef IC_DALI_APPLY
        if (any) {
            dirty_poll_mark_active(slave);
            if (emit_now) emit_state_dali(slave, addr);
        }
        break;
    }
    case IC_CMD_SET_GROUP: {
        uint8_t slave = c->set_group.slave, group = c->set_group.group;
        if (state_cache_slave_idx(slave) < 0 || group > 15) {
            LOG_WARN("cli", "ic set_group reject: slave=%u group=%u", slave, group); break; }
        bool any = false;
        if (c->set_group.fields & IC_GRP_F_MEMBERS) {
            dali_group_set_members(slave, group, c->set_group.members);
            any = true;
        }
        if (c->set_group.fields & IC_GRP_F_BRI) {
            lighting_group_set_level(slave, group, c->set_group.bri);
            any = true;
        } else if (c->set_group.fields & IC_GRP_F_ON) {
            /* IC_TRI_TOGGLE used to compare unequal to IC_TRI_ON and so silently
             * meant OFF — the same bug the CLI group path carried until it was
             * fixed, never fixed here. A toggle now probes the group and applies
             * the inverse asynchronously.
             *
             * Reached from the HTTP API / web UI. HA cannot send a toggle: its
             * JSON light schema carries only {"state":"ON"|"OFF"} and
             * ha_bridge maps anything that is not "ON" to OFF. */
            if (c->set_group.on == IC_TRI_TOGGLE) {
                lighting_toggle_async(slave, group, true);
            } else {
                lighting_group_set_on_off(slave, group, c->set_group.on == IC_TRI_ON);
            }
            any = true;
        }
        if (any) {
            dirty_poll_mark_active(slave);
            /* Only a MEMBERSHIP change is publishable straight away — that is
             * controller-owned state. on/bri are derived from member levels that
             * have not been re-read yet, so they are left to the member
             * verification reads, which recompute the group as each lands. This
             * unconditional emit was the first of the two publishes seen per
             * group command, and it carried pre-command values. */
            if (c->set_group.fields & IC_GRP_F_MEMBERS) emit_state_group(slave, group);
        }
        break;
    }
    case IC_CMD_SET_CFG: {
        bool any = false;
        if (c->set_cfg.fields & IC_CFG_F_LONG_PRESS) { g_config.long_press_ms      = c->set_cfg.long_press_ms;      any = true; }
        if (c->set_cfg.fields & IC_CFG_F_DIM_REPEAT) { g_config.dim_repeat_ms      = c->set_cfg.dim_repeat_ms;      any = true; }
        if (c->set_cfg.fields & IC_CFG_F_REVERSAL)   { g_config.reversal_window_ms = c->set_cfg.reversal_window_ms; any = true; }
        if ((c->set_cfg.fields & IC_CFG_F_BAUD) && c->set_cfg.baud > 0) { modbus_set_baud(c->set_cfg.baud); any = true; }
        if (c->set_cfg.fields & IC_CFG_F_RELAY_BYTESWAP) { g_config.relay_byteswap = c->set_cfg.relay_byteswap ? true : false; any = true; }
        if (c->set_cfg.fields & IC_CFG_F_GW_DIAG)    { g_config.gw_diag_interval_ms = c->set_cfg.gw_diag_interval_ms; any = true; }
        if (c->set_cfg.fields & IC_CFG_F_DALI_SWEEP) { g_config.dali_sweep_interval_ms = c->set_cfg.dali_sweep_interval_ms; any = true; }
        if (any && !config_save()) LOG_ERR("cli", "config_save failed — set_cfg not persisted");
        /* de_mode applies live and self-persists; keep it out of the `any` save. */
        if ((c->set_cfg.fields & IC_CFG_F_DE_MODE) && c->set_cfg.de_mode < MODBUS_DE_MAX)
            set_modbus_de_mode((modbus_de_mode_t)c->set_cfg.de_mode);
        emit_state_cfg();
        break;
    }
    case IC_CMD_SET_NETCFG: {
        uint16_t f = c->set_netcfg.fields;
        if (f & IC_NET_F_DHCP) g_config.net_dhcp = c->set_netcfg.dhcp ? 1 : 0;
        if (f & IC_NET_F_IP)   memcpy(g_config.net_ip,   c->set_netcfg.ip,   4);
        if (f & IC_NET_F_MASK) memcpy(g_config.net_mask, c->set_netcfg.mask, 4);
        if (f & IC_NET_F_GW)   memcpy(g_config.net_gw,   c->set_netcfg.gw,   4);
        if (f & IC_NET_F_DNS)  memcpy(g_config.net_dns,  c->set_netcfg.dns,  4);
        if ((f & IC_NET_F_HOSTNAME) && hostname_ok(c->set_netcfg.hostname)) {
            strncpy(g_config.net_hostname, c->set_netcfg.hostname, sizeof g_config.net_hostname - 1);
            g_config.net_hostname[sizeof g_config.net_hostname - 1] = '\0';
        }
        if (f && !config_save()) LOG_ERR("cli", "config_save failed — netcfg not persisted");
        break;
    }
    case IC_CMD_SET_MQTTCFG: {
        uint16_t f = c->set_mqttcfg.fields;
        if (f & IC_MQTT_F_HOST)   { strncpy(g_config.mqtt_host,   c->set_mqttcfg.host,   sizeof g_config.mqtt_host   - 1); g_config.mqtt_host[sizeof g_config.mqtt_host - 1] = '\0'; }
        if (f & IC_MQTT_F_PORT)   g_config.mqtt_port = c->set_mqttcfg.port;
        if (f & IC_MQTT_F_USER)   { strncpy(g_config.mqtt_user,   c->set_mqttcfg.user,   sizeof g_config.mqtt_user   - 1); g_config.mqtt_user[sizeof g_config.mqtt_user - 1] = '\0'; }
        if (f & IC_MQTT_F_PASS)   { strncpy(g_config.mqtt_pass,   c->set_mqttcfg.pass,   sizeof g_config.mqtt_pass   - 1); g_config.mqtt_pass[sizeof g_config.mqtt_pass - 1] = '\0'; }
        if (f & IC_MQTT_F_PREFIX) { strncpy(g_config.mqtt_prefix, c->set_mqttcfg.prefix, sizeof g_config.mqtt_prefix - 1); g_config.mqtt_prefix[sizeof g_config.mqtt_prefix - 1] = '\0'; }
        if (f && !config_save()) LOG_ERR("cli", "config_save failed — mqttcfg not persisted");
        break;
    }
    case IC_CMD_SET_AUTH:
        strncpy(g_config.web_user, c->set_auth.user, sizeof g_config.web_user - 1);
        g_config.web_user[sizeof g_config.web_user - 1] = '\0';
        strncpy(g_config.web_pass, c->set_auth.pass, sizeof g_config.web_pass - 1);
        g_config.web_pass[sizeof g_config.web_pass - 1] = '\0';
        if (!config_save()) LOG_ERR("cli", "config_save failed — web auth not persisted");
        break;
    case IC_CMD_SET_MAPPING: {
        uint8_t count = c->set_mapping.count;
        if (count > MAX_TARGETS_PER_BTN) count = MAX_TARGETS_PER_BTN;
        if (mapping_set(c->set_mapping.fixture, c->set_mapping.button,
                        c->set_mapping.targets, count))
            emit_config_mapping(c->set_mapping.fixture, c->set_mapping.button,
                                c->set_mapping.targets, count);
        else
            LOG_ERR("cli", "mapping_set failed F%u.B%u — binding not saved",
                    c->set_mapping.fixture, c->set_mapping.button);
        break;
    }
    case IC_CMD_DEL_MAPPING:
        if (mapping_delete(c->del_mapping.fixture, c->del_mapping.button))
            emit_config_mapping(c->del_mapping.fixture, c->del_mapping.button, NULL, 0);
        else
            LOG_ERR("cli", "mapping_delete failed F%u.B%u",
                    c->del_mapping.fixture, c->del_mapping.button);
        break;
    case IC_CMD_PROVISION_START: {
        bool valid = false;
        for (int i = 0; i < g_config.dali_slave_count; i++)
            if (g_config.dali_slave_ids[i] == c->provision_start.slave) valid = true;
        if (!valid || dali_provision_busy()) {
            LOG_WARN("cli", "ic provision_start reject: slave=%u valid=%d busy=%d",
                     c->provision_start.slave, (int)valid, (int)dali_provision_busy());
            break;
        }
        prov_mode_t mode = c->provision_start.mode == IC_PROV_UNADDRESSED ? PROV_UNADDRESSED
                         : c->provision_start.mode == IC_PROV_ADDRESS     ? PROV_ADDRESS
                                                                          : PROV_ALL;
        if (mode == PROV_ADDRESS && c->provision_start.target > 63) {
            LOG_WARN("cli", "ic provision_start reject: target=%u", c->provision_start.target);
            break;
        }
        dali_provision_start(c->provision_start.slave, mode, c->provision_start.target);
        break;
    }
    case IC_CMD_PROVISION_ABORT:
        dali_provision_abort();
        break;
    case IC_CMD_RESYNC:
        serial_tx_hello();
        emit_state_cfg();          /* core 1 builds the controller config device */
        lighting_resync_emit();
        serial_tx_config_full();
        break;
    case IC_CMD_PING_REPLY:
        cli_ping_on_reply(c);      /* print one 'net ping' echo outcome */
        break;
    case IC_CMD_REBOOT:
        controller_reboot("remote");   /* never returns */
        break;
    case IC_CMD_OTA_BEGIN:
        ota_stage_begin();
        break;
    case IC_CMD_OTA_DATA: {
        ic_event_t evt = { 0 };
        /* Clamp against g_ota_buf's size before the flash write: len is a
         * uint16_t (<=65535) but the shared buffer is only OTA_BUF_SZ; a corrupt
         * or future-buggy core-1 len must never make ota_stage_write read past
         * the buffer into adjacent memory. Over-length is treated as a fault. */
        if (c->ota_data.len <= OTA_BUF_SZ && ota_stage_write(g_ota_buf, c->ota_data.len)) {
            evt.type = IC_EVT_OTA_ACK;
        } else {
            evt.type = IC_EVT_OTA_STATUS;
            evt.ota_status.code = OTA_FLASH_FAIL;
            LOG_ERR("ota", "ota_stage_write failed (len=%u)", c->ota_data.len);
        }
        intercore_push_event(&evt);
        break;
    }
    case IC_CMD_OTA_END: {
        bool ok = ota_stage_finish(c->ota_end.total_len);
        ic_event_t evt = { .type = IC_EVT_OTA_STATUS };
        evt.ota_status.code = ok ? OTA_OK : OTA_VERIFY_FAIL;
        if (!ok) LOG_ERR("ota", "ota_stage_finish failed (total_len=%lu)",
                          (unsigned long)c->ota_end.total_len);
        intercore_push_event(&evt);
        break;
    }
    case IC_CMD_OTA_ABORT:
        ota_stage_abort();
        break;
    case IC_CMD_OTA_CONFIRM:
        ota_stage_confirm();
        break;
    }
}

/* ── Public interface ────────────────────────────────────────────────────── */
void serial_protocol_init(void) {
    /* stdio_usb is enabled by pico_enable_stdio_usb in CMakeLists; the inter-core
     * queues are initialised in main() before core 1 launch. Nothing to do. */
}

/* Boot/ready announce — a human text log (no JSON on USB). core 1 derives the
 * slave lists / fw_version from its config snapshot, not from a hello event. */
void serial_tx_hello(void) {
    LOG_INFO("serial", "hello fw=" FW_VERSION " baud=%lu relays=%u dali=%u",
             (unsigned long)modbus_get_baud(),
             g_config.relay_slave_count, g_config.dali_slave_count);
}

void serial_tx_config_full(void) {
    /* Emit every populated binding as its own config_mapping line, incrementally
     * — NOT serialised into one fixed buffer. A single 4 KB serialise buffer
     * (the old mapping_serialise_all, since removed) overflowed once the table
     * grew past ~45 bindings and printed nothing, so get_mappings answered empty
     * and HA showed no bindings after every reconnect, even though the
     * controller's table was intact. */
    for (uint8_t f = 0; f < MAX_FIXTURES; f++)
        for (uint8_t b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *rec = mapping_lookup(f, b);
            if (rec) emit_config_mapping(f, b, rec->targets, rec->target_count);
        }
}

void serial_poll(void) {
    static char line_buf[SERIAL_LINE_MAX];
    static int  line_pos = 0;

    /* Drain inbound characters (non-blocking, as in original lua_poll) */
    int ch;
    while ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (echo_on) { putchar('\r'); putchar('\n'); }
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                handle_line(line_buf);
                line_pos = 0;
            }
        } else if (echo_on && (ch == 0x08 || ch == 0x7f)) {
            /* backspace / delete: erase the last buffered character and rub it
             * out on the terminal (cursor back, overwrite with space, back
             * again).  Only active while echo is on. */
            if (line_pos > 0) {
                line_pos--;
                fputs("\b \b", stdout);
            }
        } else if (line_pos < SERIAL_LINE_MAX - 1) {
            line_buf[line_pos++] = (char)ch;
            if (echo_on) putchar((char)ch);
        } else {
            line_pos = 0; /* overflow — discard */
        }
        /* Push echoed characters (incl. the "\b \b" erase) out immediately so
         * the terminal updates on each keypress rather than at line end. */
        if (echo_on) fflush(stdout);
    }

    /* Drain inbound commands from core 1 (the network bridge) and dispatch them
     * through the same domain entry points the CLI/JSON use. Empty until core 1
     * is launched. */
    ic_cmd_t cmd;
    while (intercore_pop_cmd(&cmd)) {
        ic_dispatch_cmd(&cmd);
    }

    /* Drain outbound state events and forward each to core 1 (the network
     * bridge) as a typed struct. The event queue is sized for a full resync
     * burst, so this does not drop. USB CDC carries no JSON. */
    levt_t evt;
    while (lighting_evt_try_pop(&evt)) {
        ic_event_t ic;
        if (levt_to_ic(&evt, &ic)) intercore_push_event(&ic);
    }

    /* Push any pending provision status update to core 1. */
    serial_provision_poll();

    /* Time out a network-ping run if core 1 stops replying. */
    cli_ping_poll();

    /* Push queued log output to the USB-CDC, non-blocking (writes only what the
     * TX FIFO can take this instant). Producers on either core enqueue lines into
     * the ring without ever blocking; this is the sole drain. */
    cdc_out_drain();
}
