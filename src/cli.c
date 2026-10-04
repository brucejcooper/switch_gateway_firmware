/*
 * cli.c — human USB-CDC text CLI (core 0).
 *
 * Human-facing, line-based command interface (`help` lists the verbs). JSON
 * lives only on core 1's MQTT edge — this file never builds or parses it.
 * printf()/cdc_out_printf() write text for a person at a terminal; a `dali`
 * or `cfg` bare-get instead asks ic_seam.c to publish a {t,d,v} report to
 * core 1 (see emit_state_dali() & co. there) — the CLI just triggers it, it
 * does not carry it.
 *
 * Split out of serial_protocol.c (2026-08) along with ic_seam.c — see
 * ic_seam.c's file-top comment for the full rationale. This half kept: the
 * ~39 verb handlers, the cli_commands[] dispatch table, help text, line
 * editing/echo, and txt_ok()/txt_err(). A few pieces are shared with the
 * seam (dali_apply_kv, the emit_state_ family, emit_config_mapping —
 * primarily owned by ic_seam.c since the JSON/intercore path is their main
 * consumer; and controller_reboot, cli_ping_on_reply, cli_ping_poll —
 * primarily owned here since they're CLI features the seam merely reaches
 * into — see cli.h) — each is declared in whichever header names its
 * primary owner.
 *
 * Uses stdio_getchar_timeout_us(0) to non-blockingly read USB CDC input;
 * outbound text uses printf()/cdc_out_printf(). Runs on core 0; no multicore
 * sync needed here.
 */

#include "cli.h"
#include "ic_seam.h"
#include "cli_provision.h"
#include "dali_provision.h"
#include "lighting_actions.h"
#include "dali_queue.h"
#include "dq_soak.h"
#include "mbq_client.h"
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

/* When true, cli_poll() echoes received characters back to the terminal
 * and honours backspace for line editing.  On by default: the CLI is now a
 * human-only interface (machine clients went away when the on-board core-1
 * bridge replaced the Python host), so local echo is what you almost always
 * want.  Toggle with "echo off" if driving it from something that echoes
 * locally or wants a clean byte stream. */
static bool echo_on = true;

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

/* "this field was too long" — names the field and its real maximum, so the
 * caller learns where the boundary is instead of just that they crossed it.
 * Keeps the err: <token> shape (err: <field>_too_long) the CLI's other errors
 * use, with the limit as trailing detail. */
static void txt_err_len(const char *field, unsigned max_len) {
    printf("err: %s_too_long (max %u chars)\n", field, max_len);
}

/* Route a string field through config_apply_str() and turn each failure into
 * the right error line: too long names the real limit, and the charset
 * refusal names the banned bytes ('"', '\\', control) — before this mapper
 * every non-OK printed as "too_long", so a quote in a password read as a
 * length problem. `key` is the schema key; `shown` is the CLI-facing field
 * name (they differ for the mqtt verb: key "mqtt_pass", shown "pass"). */
static bool cli_apply_str(const char *key, const char *shown, const char *v) {
    switch (config_apply_str(key, v)) {
    case CFG_APPLY_OK:       return true;
    case CFG_APPLY_TOO_LONG: txt_err_len(shown, config_field_maxlen(key)); return false;
    default:                 printf("err: %s_invalid_chars (no quotes, backslash or control chars)\n", shown);
                             return false;
    }
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
/* cmd_help is defined below the dispatch table (~handle_text_line) — its body
 * is just "print every entry's help text", so it needs the table to exist
 * first. Forward-declared here since earlier code doesn't call it directly. */
static void cmd_help(void);

/* ── show subcommands ──────────────────────────────────────────────────── */
static void cmd_show(char **saveptr) {
    char *what = strtok_r(NULL, " \t", saveptr);
    if (!what) { txt_err("bad_arg"); return; }

    if (strcasecmp(what, "config") == 0) {
        printf("long_press_ms=%u dim_repeat_ms=%u reversal_window_ms=%u "
               "gw_diag_interval_ms=%lu dali_sweep_interval_ms=%lu "
               "baud=%lu relay_byteswap=%u fw_url=%s relay_slaves=",
               g_config.long_press_ms, g_config.dim_repeat_ms,
               g_config.reversal_window_ms,
               (unsigned long)g_config.gw_diag_interval_ms,
               (unsigned long)g_config.dali_sweep_interval_ms,
               (unsigned long)modbus_get_baud(),
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
static void emit_state_relay(uint8_t ch) {
    ic_event_t ev = { .type = IC_EVT_RELAY };
    ev.relay.ch = ch;
    ev.relay.on = state_cache_get_relay_on(ch);
    intercore_push_event(&ev);
}

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
     * discipline the JSON path uses. */
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
                /*
                 *
                 * A toggle now READS THEN WRITES, exactly as a button press
                 * does: QUERY LAMP POWER ON straight to the group selector, and
                 * invert whatever the gear reports, rather than a cached
                 * membership state a gateway reboot could have lost. */
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
             * dali_apply_kv is shared with the JSON/intercore path (ic_seam.c),
             * which runs in the main loop and must not block — so the blocking
             * probe cannot live inside it. The CLI resolves to a concrete
             * true/false and the shared code just applies it. */
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
 * retained state. Operator-driven — removal is a deliberate
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
 * keys: long_press_ms dim_repeat_ms reversal_window_ms baud fw_url (reboot to apply)
 *       relay_slaves=1,2  dali_slaves=2,3  (comma lists, reboot to apply — see
 *       config.h's pending_relay/dali_slave_* fields). fw_version is read-only. */
/* Parse `val` as decimal and route it through config_apply_u32(key, ...),
 * printing the CLI's usual err: line (naming the real range, not just that
 * the value was rejected) and returning false on failure. `key` must be one
 * of CFG_SCHEMA's numeric/bool keys — every call site below only reaches this
 * after its own strcasecmp has matched one, so UNKNOWN_KEY/INVALID here would
 * mean this file and config.c's schema have drifted apart; fail closed rather
 * than silently accept if that ever happens. Shared by cmd_cfg (long_press_ms
 * etc.) and cmd_mqtt (port) so the error text is written once. */
static bool cfg_apply_numeric_cli(const char *key, const char *val) {
    uint32_t u = (uint32_t)strtoul(val, NULL, 10);
    cfg_apply_result_t r = config_apply_u32(key, u);
    if (r == CFG_APPLY_OK) return true;
    if (r == CFG_APPLY_OUT_OF_RANGE) {
        uint32_t mn = 0, mx = 0;
        config_field_range(key, &mn, &mx);
        char msg[64];
        snprintf(msg, sizeof msg, "%s_out_of_range (%lu..%lu)", key, (unsigned long)mn, (unsigned long)mx);
        txt_err(msg);
    } else {
        txt_err("bad_field");
    }
    return false;
}

static void cmd_cfg(char **saveptr) {
    char *kv = strtok_r(NULL, " \t", saveptr);
    if (!kv) { emit_state_cfg(); txt_ok(); return; }
    bool any = false, deferred = false;
    do {
        char *eq = strchr(kv, '=');
        if (!eq) { txt_err("bad_arg expected key=value"); return; }
        *eq = '\0';
        const char *key = kv, *val = eq + 1;
        /* Schema-covered numerics: config_apply_u32() enforces CFG_SCHEMA's
         * range (config.c) — previously atoi() with no bound at all, so e.g.
         * long_press_ms=70000 silently wrapped to 4464 downstream. */
        if      (strcasecmp(key, "long_press_ms") == 0)          { if (!cfg_apply_numeric_cli("long_press_ms", val)) return; }
        else if (strcasecmp(key, "dim_repeat_ms") == 0)          { if (!cfg_apply_numeric_cli("dim_repeat_ms", val)) return; }
        else if (strcasecmp(key, "reversal_window_ms") == 0)     { if (!cfg_apply_numeric_cli("reversal_window_ms", val)) return; }
        else if (strcasecmp(key, "gw_diag_interval_ms") == 0)    { if (!cfg_apply_numeric_cli("gw_diag_interval_ms", val)) return; }
        else if (strcasecmp(key, "dali_sweep_interval_ms") == 0) { if (!cfg_apply_numeric_cli("dali_sweep_interval_ms", val)) return; }
        else if (strcasecmp(key, "baud") == 0)               modbus_set_baud((uint32_t)atoi(val));
        else if (strcasecmp(key, "relay_byteswap") == 0) {
            uint32_t u = (strcmp(val, "true") == 0 || strcmp(val, "1") == 0 || strcasecmp(val, "on") == 0) ? 1u : 0u;
            config_apply_u32("relay_byteswap", u);   /* CFG_BOOL: always CFG_APPLY_OK */
        }
        else if (strcasecmp(key, "relay_slaves") == 0) {
            /* Persist-only, reboot-to-apply — see the field comment in config.h.
             * Consumers bind slave_id -> array index at boot, so rewriting the
             * live list here would desync them from state_cache_slave_idx()'s
             * live lookup mid-session and misattribute cached state. */
            g_config.pending_relay_slave_count = (uint8_t)parse_csv_u8(val, g_config.pending_relay_slave_ids, MAX_RELAY_SLAVES);
            g_config.relay_slaves_pending = true;
            deferred = true;
        }
        else if (strcasecmp(key, "dali_slaves") == 0) {
            g_config.pending_dali_slave_count = (uint8_t)parse_csv_u8(val, g_config.pending_dali_slave_ids, MAX_DALI_SLAVES);
            g_config.dali_slaves_pending = true;
            deferred = true;
        }
        else if (strcasecmp(key, "fw_url") == 0) {
            /* empty value clears the override (compiled-in default applies) —
             * config_apply_str()'s CFG_STR case allows an empty string, same
             * as the str_set() call this replaces. */
            if (!cli_apply_str("fw_url", "fw_url", val)) return;
            /* fwupdate.c reads this from the intercore snapshot, which is built
             * once at core1_launch — so the new URL is not live until a reboot.
             * Without this note the next `check` silently polls the OLD url and
             * reports no update, which reads as a broken manifest. */
            deferred = true;
        }
        else { txt_err("bad_field"); return; }
        any = true;
    } while ((kv = strtok_r(NULL, " \t", saveptr)) != NULL);
    if (any && !config_save()) { txt_err("save_failed"); return; }
    emit_state_cfg();
    if (deferred) printf("ok (reboot to apply)\n");
    else          txt_ok();
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

/* `mbq [<slave>]` — the queue transport's own view: FSM state, per-command
 * binding, and the protocol counters of queue-protocol §6. Reboots are split by
 * the rule that noticed them, because the split says HOW they are being seen —
 * a gateway whose reboots only ever surface as R4 deadline expiries is one
 * whose structural rules (R2/R3) are not firing, which is itself the finding.
 *
 * `mbq drop-reply <slave> <n>` — discard the next n replies from that slave
 * inside the client, after transport accounting and before resolution. This is
 * the loss drill: dropping the reply to an enqueue+ack exercises R1/R1', and
 * dropping the reply to an ack-less first enqueue exercises the deadline and
 * boot_count corner. It forces the REAL recovery paths on real hardware — it
 * does not simulate them. */
static void cmd_mbq(char **saveptr) {
    char *a1 = strtok_r(NULL, " \t", saveptr);
    if (a1 && strcasecmp(a1, "drop-reply") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        char *ns = strtok_r(NULL, " \t", saveptr);
        long slave, n;
        if (!parse_int_range_auto(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
        if (!parse_int_range_auto(ns, 0, 1000, &n))    { txt_err("bad_n"); return; }
        if (!mbq_client_drop_reply((uint8_t)slave, (uint16_t)n)) {
            txt_err("invalid_slave");
            return;
        }
        txt_ok();
        return;
    }
    if (a1 && strcasecmp(a1, "no-ack") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        char *ns = strtok_r(NULL, " \t", saveptr);
        long slave, n;
        if (!parse_int_range_auto(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
        if (!parse_int_range_auto(ns, 0, 1000, &n))    { txt_err("bad_n"); return; }
        if (!mbq_client_no_ack((uint8_t)slave, (uint16_t)n)) { txt_err("invalid_slave"); return; }
        txt_ok();
        return;
    }
    if (a1 && strcasecmp(a1, "gw") == 0) {
        char *ss = strtok_r(NULL, " \t", saveptr);
        long slave;
        if (!parse_int_range_auto(ss, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
        static cli_mb_ctx_t ctx;
        memset(&ctx, 0, sizeof ctx);
        modbus_get_input_registers((uint8_t)slave, MBQ_GW_DIAG_ADDR,
                                   MBQ_GW_DIAG_REGS, cli_mb_cb, &ctx);
        cli_mb_wait(&ctx);
        if (cli_mb_report_fail(&ctx)) return;
        if (ctx.frame_len < 3 + (int)(MBQ_GW_DIAG_REGS * 2u)) { txt_err("short_read"); return; }
        const uint8_t *p = ctx.frame + 3;
        static const char *nm[MBQ_GW_DIAG_REGS] = {
            "accepts", "a_full", "executes", "results",
            "acks", "stalls", "q_discard", "spare",
        };
        cdc_out_printf("mbq gateway counters, slave %ld (FC04 0x%04X):\n",
                       slave, MBQ_GW_DIAG_ADDR);
        for (unsigned i = 0; i < MBQ_GW_DIAG_REGS; i++)
            cdc_out_printf("  %-10s %u\n", nm[i],
                           (unsigned)(((uint16_t)p[i * 2u] << 8) | p[i * 2u + 1u]));
        /* What each one means when it is NOT zero: a_full says the master's
         * in-flight accounting is wrong (the cap should make it unreachable),
         * stalls is just the backpressure rule doing its job, q_discard means
         * malformed frames are reaching the gateway. */
        return;
    }
    long slave = -1;
    if (a1 && !parse_int_range_auto(a1, 0, 247, &slave)) { txt_err("invalid_slave"); return; }
    mbq_client_debug_print((int)slave);
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
    cdc_out_printf("action queue: peak %lu of 16, dropped %lu\n",
                   (unsigned long)modbus_action_peak(),
                   (unsigned long)modbus_drop_count());
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
    /* Printed even at zero: a corrupt reply is now discarded rather than acted
     * on, so this is the only place it becomes visible, and "0" is the point. */
    cdc_out_printf("  bad CRC: %lu reply frame(s) discarded since boot\n",
           (unsigned long)modbus_crc_error_count());
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

    /* Asynchronous: an interrogation on a populated bus is many seconds, and
     * `scan` gets typed on a LIVE system where someone may be at a wall switch.
     * Blocking would stall the main loop and freeze the buttons. Arm it and let
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
    /* modbus_set_baud() now applies the UART divisor itself before persisting,
     * so the persist path no longer needs a separate runtime call first. */
    if (persist) modbus_set_baud((uint32_t)baud);
    else         modbus_uart_set_baud_runtime((uint32_t)baud);
    printf("baud: %lu%s\n", (unsigned long)modbus_get_baud(),
           persist ? " (saved)" : " (live only)");
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
 * (MQTT/web, dispatched by ic_seam.c) both land here — see cli.h. `src` names
 * the trigger for the log. The short sleep lets the USB CDC line drain and
 * gives core 1 time to flush its network ack (HTTP 200 / MQTT) before the
 * watchdog resets the whole chip. Never returns. This is the out-of-band
 * recovery for a wedged USB stack: it does not touch USB, so it works even
 * when the CLI/picotool reset are dead. */
void controller_reboot(const char *src) {
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
 *    IC_CMD_PING_REPLY, dispatched by ic_seam.c into cli_ping_on_reply() below,
 *    and print here). One run in flight at a time. ───────────────────────── */
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

void cli_ping_on_reply(const ic_cmd_t *c) {
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

/* Called from ic_seam_pump(): bail out of a stuck run if core 1 never replies. */
void cli_ping_poll(void) {
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
        /* config_apply_str() refuses rather than truncates (strfit.h's rule),
         * and always terminates — these four used strncpy(dst, v, sizeof dst
         * - 1) with no explicit NUL, safe only because the field's last byte
         * happened never to be written non-zero. The error names the real
         * limit so "too long" does not leave the user guessing where the
         * boundary is. `port` now goes through config_apply_u32() the same
         * way — previously a bare atoi() accepted anything, including 0 or a
         * value that silently wrapped past 65535 through the uint16_t cast. */
        if      (!strcasecmp(k, "host"))   { if (!cli_apply_str("mqtt_host",   "host",   v)) return; any = true; }
        else if (!strcasecmp(k, "port"))   { if (!cfg_apply_numeric_cli("mqtt_port", v)) return; any = true; }
        else if (!strcasecmp(k, "user"))   { if (!cli_apply_str("mqtt_user",   "user",   v)) return; any = true; }
        else if (!strcasecmp(k, "pass"))   { if (!cli_apply_str("mqtt_pass",   "pass",   v)) return; any = true; }
        else if (!strcasecmp(k, "prefix")) { if (!cli_apply_str("mqtt_prefix", "prefix", v)) return; any = true; }
        else { txt_err("bad_key"); return; }
    }
    if (!any) { txt_err("no_fields"); return; }
    if (!config_save()) { txt_err("save_failed"); return; }
    printf("ok (reboot to apply)\n");
}

/* hostname_ok()/hostname_set() moved to config.{c,h} — config_load() and the
 * core-1 web/API handler need the same rule this file used to keep private. */

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
            /* config_apply_str()'s CFG_HOSTNAME row calls config_hostname_set()
             * itself — same charset/lowercasing rule, same error either way. */
            if (config_apply_str("net_hostname", v) != CFG_APPLY_OK) {
                txt_err("bad_hostname"); return;
            }
            any = true;
        }
        else { txt_err("bad_key"); return; }
    }
    if (!any) { txt_err("no_fields"); return; }
    if (!config_save()) { txt_err("save_failed"); return; }
    printf("ok (reboot to apply)\n");
}

/* ── CLI dispatch table ────────────────────────────────────────────────────
 * Single source of truth for both dispatch (handle_text_line) and `help`
 * output. Replaces a hand-written strcasecmp chain that had drifted from
 * cmd_help's separately hand-written verb list — cmd_cfg grew
 * gw_diag_interval_ms/dali_sweep_interval_ms/relay_byteswap keys that help
 * never mentioned (fixed below, in the "cfg" entry's help text).
 *
 * Matching is strcasecmp EXACT-match (never prefix), so table order cannot
 * introduce ambiguity — no verb here is a prefix of another in a way that
 * would matter. Still, first match wins, top to bottom, same as the old
 * chain. */
typedef void (*cli_handler_t)(char **saveptr);

/* Thin wrappers: cmd_help/cmd_reboot take no CLI args, so their signature is
 * (void), not (char **) like the rest — wrap rather than change them. */
static void cmd_help_v(char **saveptr)   { (void)saveptr; cmd_help(); }
static void cmd_reboot_v(char **saveptr) { (void)saveptr; cmd_reboot(); }

/* dstate/resync were inlined in the old dispatch chain with no named
 * handler; given one each here so every table entry is a plain function. */
static void cmd_dstate(char **saveptr) {
    char *ss2 = strtok_r(NULL, " \t", saveptr);
    long sl = -1;
    if (ss2 && !parse_int_range_auto(ss2, 0, 247, &sl)) { txt_err("invalid_slave"); return; }
    dali_state_debug_print((int)sl);
}
static void cmd_resync(char **saveptr) {
    (void)saveptr;
    LOG_INFO("cli", "resync");
    serial_tx_hello();
    lighting_resync_emit();
    serial_tx_config_full();
    txt_ok();
}

typedef struct {
    const char *verb;
    cli_handler_t fn;
    const char *help;   /* one or more lines, printed verbatim by `help` */
} cli_cmd_t;

static const cli_cmd_t cli_commands[] = {
    { "help", cmd_help_v,
      "help                                       list commands\n" },
    { "show", cmd_show,
      "show config                                global tunables\n"
      "show mappings                              list all mappings\n"
      "show mapping <fix> <btn>                   one mapping\n"
      "show relay [<ch>]                          relay state (one or all)\n"
      "show dali [<slave> [<addr>]]               DALI state\n"
      "show group <slave> [<group>]               DALI group state\n"
      "show present <slave>                       present ballast addresses\n"
      "show provision                             provisioning status\n"
      "show log                                   current verbosity\n" },
    { "relay", cmd_relay,
      "relay <ch> [on=true|false|toggle]          bare=get, else set\n" },
    { "btn", cmd_btn,
      "btn <fixture> <button> [tap|press|release]  synthesize a button event (test)\n" },
    { "dali", cmd_dali,
      "dali <slave> <addr|gN> [key=val ...]       bare=get; keys: on bri min max\n"
      "                                           power_on sys_fail fade_rate fade_time\n"
      "                                           group (gN) also: members=0,1  add=2  remove=1\n" },
    { "cfg", cmd_cfg,
      "cfg [key=val ...]                          bare=get; long_press_ms dim_repeat_ms\n"
      "                                           reversal_window_ms gw_diag_interval_ms\n"
      "                                           dali_sweep_interval_ms relay_byteswap\n"
      "                                           fw_url (reboot to apply)\n"
      "                                           baud relay_slaves=1,2 dali_slaves=2,3 (reboot to apply)\n" },
    { "tombstone", cmd_tombstone,
      "tombstone dali <s> <a> | group <s> <g> | gw <s>   retire a removed device's retained MQTT topics\n" },
    { "mqtt", cmd_mqtt,
      "mqtt [show | set host= port= user= pass= prefix=]  broker config (reboot to apply)\n" },
    { "net", cmd_net,
      "net  [show | set dhcp= ip= mask= gw= dns=]   network config (reboot to apply)\n"
      "net  ping <ip> [count]                      ICMP echo from the device (default 4)\n" },
    { "dali_raw", cmd_dali_raw,
      "dali_raw <slave> <cmd16> [twice]           send raw 16-bit DALI frame, report result (dropped-not-resubmitted on gw reboot)\n" },
    { "read_mem", cmd_read_mem,
      "read_mem <slave> <addr> <bank> <off> <n>   read n bytes from a ballast memory bank (hex)\n" },
    { "identify", cmd_identify,
      "identify <slave> <addr>                    read a ballast's GTIN + serial (bank 0)\n" },
    { "blink", cmd_blink,
      "blink <slave> <addr>                       flash one ballast to locate it physically\n" },
    { "gw_reboot", cmd_gw_reboot,
      "gw_reboot <slave>                          reboot a TinyModbus gateway (recover stuck bus)\n" },
    { "dq", cmd_dq,
      "dq [slave]                                 dump command-queue client state + counters\n" },
    { "mbq", cmd_mbq,
      "mbq [slave] | gw <slave>                  queue transport state / gateway counters\n"
      "mbq drop-reply|no-ack <slave> <n>          §4 loss drill / backpressure drill\n" },
    { "soak", cmd_soak,
      "soak [<slave> <addr> [n=] [depth=] [path=direct|bg|user]] | stop   queue soak\n" },
    { "dstate", cmd_dstate,
      "dstate [slave]                             DALI world model: presence, sweep, verify marks\n" },
    { "busload", cmd_busload,
      "busload [reset]                            RS-485 bus occupancy, split per slave\n" },
    { "dim", cmd_dim,
      "dim <slave> <addr> {up|down}               step one ballast\n"
      "dim <slave> g<group> {up|down}             step one group\n" },
    { "provision", cmd_provision,
      "provision <slave> {all|new|<addr>}         commission: all / incremental / re-provision collided <addr>\n"
      "provision abort                            stop commissioning\n" },
    { "scan", cmd_scan,
      "scan <slave>                               blocking full scan\n" },
    { "discover", cmd_discover,
      "discover                                   re-probe all configured slaves (blocking)\n" },
    { "ping", cmd_ping,
      "ping <slave> [relay|dali]                  one ping at current master baud (kind inferred if omitted)\n" },
    { "set_slave_baud", cmd_set_slave_baud,
      "set_slave_baud <slave> {9600|19200|38400} [relay|dali]  tell slave to switch its UART baud\n" },
    { "baud", cmd_baud,
      "baud [<rate> [save]]                       show / set UART baud (live; 'save' persists)\n" },
    { "ls", cmd_ls,
      "ls [<path>]                                list files in path (default /)\n" },
    { "cat", cmd_cat,
      "cat <path>                                 dump file contents to console\n" },
    { "map", cmd_map,
      "map <fix> <btn> [targets=<s>:<a>,...]     bare=get; targets= deletes (a=N or gN)\n" },
    { "log", cmd_log,
      "log <level>                                set verbosity (off|err|warn|info|dbg)\n" },
    { "get_input", cmd_get_input,
      "get_input <slave> <reg> [count]            raw FC04 read input registers\n" },
    { "get_holding", cmd_get_holding,
      "get_holding <slave> <reg> [count]          raw FC03 read holding registers\n" },
    { "set_holding", cmd_set_holding,
      "set_holding <slave> <reg> <value>          raw FC06 write holding register\n" },
    { "get_coils", cmd_get_coils,
      "get_coils <slave> <reg> [count]            raw FC01 read coils\n" },
    { "set_coil", cmd_set_coil,
      "set_coil <slave> <reg> {true|false|toggle} raw FC05 write coil\n"
      "  (reg/value accept decimal or 0x-prefixed hex)\n" },
    { "echo", cmd_echo,
      "echo {on|off}                              echo input + backspace editing (or show state)\n" },
    { "reboot", cmd_reboot_v,
      "reboot                                     watchdog reset\n" },
    { "resync", cmd_resync,
      "resync                                     re-emit hello + all state + mappings\n" },
};
#define CLI_COMMANDS_COUNT (sizeof(cli_commands) / sizeof(cli_commands[0]))

static void cmd_help(void) {
    for (size_t i = 0; i < CLI_COMMANDS_COUNT; i++)
        fputs(cli_commands[i].help, stdout);
}

/* ── Top-level text dispatcher ─────────────────────────────────────────── */
static void handle_text_line(const char *original_line) {
    LOG_DBG("cli", "rx: %s", original_line);   /* normal traffic, not an event → DEBUG */

    /* strtok_r mutates the buffer, so copy. CLI_LINE_MAX is the upper
     * bound on the line length set by the input buffer. */
    char buf[CLI_LINE_MAX];
    strncpy(buf, original_line, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    char *saveptr = NULL;
    char *cmd = strtok_r(buf, " \t", &saveptr);
    if (!cmd) return;

    for (size_t i = 0; i < CLI_COMMANDS_COUNT; i++) {
        if (strcasecmp(cmd, cli_commands[i].verb) == 0) {
            cli_commands[i].fn(&saveptr);
            return;
        }
    }

    LOG_WARN("cli", "parse err: unknown_command for input %s", original_line);
    txt_err("unknown_command");
}

/* ── Public interface ────────────────────────────────────────────────────── */
void cli_init(void) {
    /* stdio_usb is enabled by pico_enable_stdio_usb in CMakeLists; the inter-core
     * queues are initialised in main() before core 1 launch. Nothing to do. */
}

void cli_poll(void) {
    static char line_buf[CLI_LINE_MAX];
    static int  line_pos = 0;
    /* Set the instant a line overflows the buffer; while poisoned, every byte
     * up to the next newline is swallowed unbuffered instead of refilling
     * line_buf from offset 0. Without this, the discard on overflow reset only
     * line_pos, so bytes kept landing at line_buf[0] — the tail of an
     * over-length paste (everything after the point it overflowed) filled the
     * buffer again and got dispatched as its own command at the next newline —
     * an attacker- or paste-controlled command executing from the middle of a
     * line nothing meant to run standalone. */
    static bool poisoned = false;

    /* Drain inbound characters (non-blocking, as in original lua_poll) */
    int ch;
    while ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        if (ch == '\n' || ch == '\r') {
            if (echo_on) { putchar('\r'); putchar('\n'); }
            if (poisoned) {
                poisoned = false;
                line_pos = 0;
                txt_err("line_too_long");
            } else if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                handle_line(line_buf);
                line_pos = 0;
            }
        } else if (poisoned) {
            /* mid-overflow: discard silently, wait for the line to end */
        } else if (echo_on && (ch == 0x08 || ch == 0x7f)) {
            /* backspace / delete: erase the last buffered character and rub it
             * out on the terminal (cursor back, overwrite with space, back
             * again).  Only active while echo is on. */
            if (line_pos > 0) {
                line_pos--;
                fputs("\b \b", stdout);
            }
        } else if (line_pos < CLI_LINE_MAX - 1) {
            line_buf[line_pos++] = (char)ch;
            if (echo_on) putchar((char)ch);
        } else {
            line_pos  = 0;
            poisoned  = true;   /* overflow — poison until the newline */
        }
        /* Push echoed characters (incl. the "\b \b" erase) out immediately so
         * the terminal updates on each keypress rather than at line end. */
        if (echo_on) fflush(stdout);
    }
}
