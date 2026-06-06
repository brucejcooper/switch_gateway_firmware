/*
 * serial_protocol.c — JSON line protocol over USB CDC.
 *
 * Uses stdio_getchar_timeout_us(0) to non-blockingly read USB CDC input;
 * outbound messages use printf(). Single-core; no multicore sync needed.
 */

#include "serial_protocol.h"
#include "dali_provision.h"
#include "lighting_actions.h"
#include "mapping.h"
#include "button_fsm.h"
#include "config.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "bus_discovery.h"
#include "modbus.h"
#include "log.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <hardware/watchdog.h>
#include <pico/stdio.h>
#include <pico/time.h>
#include <sys/stat.h>
#include <dirent.h>

/* Consume levt_t events from lighting_actions */
extern bool lighting_evt_try_pop(void *out);  /* defined in lighting_actions.c */

/* When true, serial_poll() echoes received characters back to the terminal
 * and honours backspace for line editing.  Off by default; toggled with the
 * "echo on" / "echo off" command.  Useful when driving the CLI from a dumb
 * terminal (e.g. miniterm) that does not echo locally. */
static bool echo_on = false;

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
    char mbuf[256];
    int n = snprintf(mbuf, sizeof(mbuf),
                     "{\"t\":\"config_mapping\","
                     "\"d\":{\"fixture\":%u,\"button\":%u},"
                     "\"v\":{\"targets\":[",
                     fixture, button);
    for (int i = 0; i < count; i++) {
        if (TARGET_IS_GROUP(targets[i])) {
            n += snprintf(mbuf + n, sizeof(mbuf) - n,
                          "%s{\"slave\":%u,\"group\":%u}",
                          i > 0 ? "," : "",
                          targets[i].slave, TARGET_GROUP_NUM(targets[i]));
        } else {
            n += snprintf(mbuf + n, sizeof(mbuf) - n,
                          "%s{\"slave\":%u,\"addr\":%u}",
                          i > 0 ? "," : "",
                          targets[i].slave, targets[i].addr);
        }
    }
    n += snprintf(mbuf + n, sizeof(mbuf) - n, "]}}\n");
    printf("%s", mbuf);
}

static void diff_emit_cb(uint8_t fixture, uint8_t button,
                         const mapping_target_t *new_targets,
                         uint8_t new_count, void *user) {
    (void)user;
    emit_config_mapping(fixture, button, new_targets, new_count);
}

/* ── Process one inbound JSON line ──────────────────────────────────────── */
static void handle_json_line(const char *line) {
    char t[32];
    if (!json_str(line, "t", t, sizeof(t))) return;

    int id = 0;
    json_int(line, "id", &id);
    unsigned cmd_id = (unsigned)id;

    /* ── cmd_relay ─────────────────────────────────────────────────────── */
    if (strcmp(t, "cmd_relay") == 0) {
        int ch = 0; bool on = false;
        json_int(line, "ch", &ch);
        json_bool(line, "on", &on);
        lighting_set_relay((uint8_t)ch, on);
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── cmd_dali ──────────────────────────────────────────────────────── */
    if (strcmp(t, "cmd_dali") == 0) {
        int slave = 0, addr = 0, bri = 0;
        bool on = false;
        json_int(line, "slave", &slave);
        json_int(line, "addr",  &addr);
        bool has_bri = json_int(line, "bri", &bri);
        json_bool(line, "on", &on);
        if (state_cache_slave_idx((uint8_t)slave) < 0) {
            serial_tx_ack(cmd_id, false, "invalid_slave");
            return;
        }
        if (addr < 0 || addr > 63) {
            serial_tx_ack(cmd_id, false, "out_of_range");
            return;
        }
        if (has_bri) {
            lighting_dali_set_level((uint8_t)slave, (uint8_t)addr, (uint8_t)bri);
        } else {
            lighting_set_on_off((uint8_t)slave, (uint8_t)addr, on);
        }
        dirty_poll_mark_active((uint8_t)slave);
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── cmd_dali_group ────────────────────────────────────────────────── */
    if (strcmp(t, "cmd_dali_group") == 0) {
        int slave = 0, group = 0, bri = 0;
        bool on = false;
        json_int(line, "slave", &slave);
        json_int(line, "group", &group);
        bool has_bri = json_int(line, "bri", &bri);
        json_bool(line, "on", &on);
        if (state_cache_slave_idx((uint8_t)slave) < 0) {
            serial_tx_ack(cmd_id, false, "invalid_slave");
            return;
        }
        if (group < 0 || group > 15) {
            serial_tx_ack(cmd_id, false, "invalid_group");
            return;
        }
        if (has_bri) {
            lighting_group_set_level((uint8_t)slave, (uint8_t)group, (uint8_t)bri);
        } else {
            lighting_group_set_on_off((uint8_t)slave, (uint8_t)group, on);
        }
        dirty_poll_mark_active((uint8_t)slave);
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── cmd_dali_broadcast ────────────────────────────────────────────── */
    if (strcmp(t, "cmd_dali_broadcast") == 0) {
        int slave = 0, addr_byte = 0, bri = 0;
        bool on = false;
        json_int(line, "slave",     &slave);
        json_int(line, "addr_byte", &addr_byte);
        bool has_bri = json_int(line, "bri", &bri);
        json_bool(line, "on", &on);
        /* All forms now go through RAW_CMD (0x1000). The per-ballast bank
         * shortcut isn't usable here because addr_byte may be a group or
         * broadcast, not a short address. */
        uint16_t val;
        if (has_bri) {
            /* DAPC: clear bit 0 of the address byte to force DAPC form. */
            val = ((uint16_t)((uint8_t)addr_byte & 0xFEu) << 8) | (uint8_t)bri;
        } else if (on) {
            /* GOTO_LAST_ACTIVE (0x0A): set bit 0 to force command form. */
            val = ((uint16_t)((uint8_t)addr_byte | 0x01u) << 8) | 0x0Au;
        } else {
            /* OFF (0x00): set bit 0 to force command form. */
            val = ((uint16_t)((uint8_t)addr_byte | 0x01u) << 8) | 0x00u;
        }
        dali_raw_arm((uint8_t)slave, val, false);
        dirty_poll_mark_active((uint8_t)slave);
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── get_config ────────────────────────────────────────────────────── */
    if (strcmp(t, "get_config") == 0) {
        serial_tx_config_full();
        return;
    }

    /* ── resync ────────────────────────────────────────────────────────── */
    if (strcmp(t, "resync") == 0) {
        serial_tx_hello();
        lighting_resync_emit();
        serial_tx_config_full();
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── set_mapping ───────────────────────────────────────────────────── */
    if (strcmp(t, "set_mapping") == 0) {
        int fixture = 0, button = 0;
        json_int(line, "fixture", &fixture);
        json_int(line, "button",  &button);
        mapping_target_t targets[MAX_TARGETS_PER_BTN];
        uint8_t count = parse_targets(line, targets, MAX_TARGETS_PER_BTN);
        bool ok = mapping_set((uint8_t)fixture, (uint8_t)button, targets, count);
        if (ok) {
            emit_config_mapping((uint8_t)fixture, (uint8_t)button, targets, count);
        }
        serial_tx_ack(cmd_id, ok, ok ? NULL : "save_failed");
        return;
    }

    /* ── del_mapping ───────────────────────────────────────────────────── */
    if (strcmp(t, "del_mapping") == 0) {
        int fixture = 0, button = 0;
        json_int(line, "fixture", &fixture);
        json_int(line, "button",  &button);
        bool ok = mapping_delete((uint8_t)fixture, (uint8_t)button);
        if (ok) {
            emit_config_mapping((uint8_t)fixture, (uint8_t)button, NULL, 0);
        }
        serial_tx_ack(cmd_id, ok, ok ? NULL : "save_failed");
        return;
    }

    /* ── replace_mappings_begin ─────────────────────────────────────────
     * Wipe-and-replace transaction. Sequence: begin → add* → commit (or
     * abort). The 256-byte serial line cap means we can't send all 24×8
     * possible mappings in one message, so the txn buffers everything in
     * RAM and only swaps live + saves to flash on commit. */
    if (strcmp(t, "replace_mappings_begin") == 0) {
        mapping_txn_begin();
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── replace_mapping_add ─────────────────────────────────────────────
     * Same payload as set_mapping, but stages into the txn scratch buffer.
     * Bad ranges latch a sticky error so the eventual commit refuses. */
    if (strcmp(t, "replace_mapping_add") == 0) {
        if (!mapping_txn_active()) {
            serial_tx_ack(cmd_id, false, "no_txn");
            return;
        }
        int fixture = 0, button = 0;
        json_int(line, "fixture", &fixture);
        json_int(line, "button",  &button);
        mapping_target_t targets[MAX_TARGETS_PER_BTN];
        uint8_t count = parse_targets(line, targets, MAX_TARGETS_PER_BTN);
        bool ok = mapping_txn_add((uint8_t)fixture, (uint8_t)button,
                                  targets, count);
        serial_tx_ack(cmd_id, ok, ok ? NULL : "out_of_range");
        return;
    }

    /* ── replace_mappings_commit ─────────────────────────────────────────
     * Atomically swap the scratch in, save to flash, and emit
     * config_mapping for every (fixture, button) whose targets changed. */
    if (strcmp(t, "replace_mappings_commit") == 0) {
        if (!mapping_txn_active()) {
            serial_tx_ack(cmd_id, false, "no_txn");
            return;
        }
        if (mapping_txn_errored()) {
            mapping_txn_abort();
            serial_tx_ack(cmd_id, false, "txn_invalid");
            return;
        }
        /* Walk diffs BEFORE the swap so we can compare scratch vs live. */
        mapping_txn_for_each_change(diff_emit_cb, NULL);
        bool ok = mapping_txn_commit();
        serial_tx_ack(cmd_id, ok, ok ? NULL : "save_failed");
        return;
    }

    /* ── replace_mappings_abort ─────────────────────────────────────────── */
    if (strcmp(t, "replace_mappings_abort") == 0) {
        mapping_txn_abort();
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── set_config ────────────────────────────────────────────────────── */
    if (strcmp(t, "set_config") == 0) {
        int v = 0;
        if (json_int(line, "long_press_ms", &v))
            g_config.long_press_ms = (uint16_t)v;
        if (json_int(line, "dim_repeat_ms", &v))
            g_config.dim_repeat_ms = (uint16_t)v;
        if (json_int(line, "reversal_window_ms", &v))
            g_config.reversal_window_ms = (uint16_t)v;
        config_save();
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── set_baud ──────────────────────────────────────────────────────── */
    if (strcmp(t, "set_baud") == 0) {
        int baud = 0;
        if (json_int(line, "baud", &baud) && baud > 0) {
            modbus_set_baud((uint32_t)baud);
            printf("{\"t\":\"ack\",\"id\":%u,\"d\":{\"ok\":true,"
                   "\"note\":\"reboot required for baud change\"}}\n", cmd_id);
        } else {
            serial_tx_ack(cmd_id, false, "invalid_baud");
        }
        return;
    }

    /* ── provision_start ──────────────────────────────────────────────── */
    if (strcmp(t, "provision_start") == 0) {
        serial_handle_provision_start(line, cmd_id);
        return;
    }

    /* ── provision_abort ──────────────────────────────────────────────── */
    if (strcmp(t, "provision_abort") == 0) {
        serial_handle_provision_abort(line, cmd_id);
        return;
    }
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

    if (*line == '{') {
        handle_json_line(line);
    } else {
        handle_text_line(line);
    }
}

/* ── helpers ────────────────────────────────────────────────────────────── */

/* Print an err: <reason> line and return. Reason is a short token. */
static void txt_err(const char *reason) {
    printf("err: %s\n", reason);
}

static void txt_ok(void) {
    printf("ok\n");
}

/* Parse "on" / "off" / "toggle" — case-insensitive. Returns 1=on, 0=off,
 * 2=toggle, -1 on parse error. */
static int parse_on_off_toggle(const char *s) {
    if (!s) return -1;
    if (strcasecmp(s, "on") == 0)     return 1;
    if (strcasecmp(s, "off") == 0)    return 0;
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

/* Provisioning state name, matching serial_protocol_provision.c */
static const char *prov_state_name(prov_run_state_t s) {
    switch (s) {
    case PROV_STATE_IDLE:          return "idle";
    case PROV_STATE_CLEARING:      return "clearing";
    case PROV_STATE_SCANNING_USED: return "scanning";
    case PROV_STATE_TERMINATING:   return "terminating";
    case PROV_STATE_INITIALISING:  return "initialising";
    case PROV_STATE_RANDOMISING:   return "randomising";
    case PROV_STATE_SEARCHING:     return "searching";
    case PROV_STATE_ASSIGNING:     return "assigning";
    case PROV_STATE_VERIFYING:     return "verifying";
    case PROV_STATE_WITHDRAWING:   return "withdrawing";
    case PROV_STATE_COMPLETE:      return "complete";
    case PROV_STATE_ERROR:         return "error";
    default:                       return "unknown";
    }
}

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

/* ── Raw DALI command + async result helpers (FC06 0x1000/0x1001 -> FC04 0x0108) ── */

/* Decode a raw_cmd_result word (FC04 0x0108) into a short label. */
static const char *dali_result_str(uint16_t r) {
    switch (r >> 8) {
        case 0x00: return "ok";
        case 0x01: return "nak";
        case 0xFF: return "pending";
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

/* Send a raw DALI frame (single, or twice when `twice`) and poll FC04 0x0108
 * for the asynchronous outcome.  Retries the write on DEVICE_BUSY (0x06).  On
 * success sets *result to the raw_cmd_result word and returns true; on a
 * Modbus transport failure prints the error and returns false. */
static bool dali_raw_result(uint8_t slave, uint16_t frame, bool twice,
                            uint16_t *result) {
    static cli_mb_ctx_t ctx;
    uint16_t reg = twice ? 0x1001u : 0x1000u;

    bool acked = false;
    for (int attempt = 0; attempt < 6 && !acked; attempt++) {
        memset(&ctx, 0, sizeof(ctx));
        modbus_set_holding_register(slave, reg, frame, cli_mb_cb, &ctx);
        cli_mb_wait(&ctx);
        if (!ctx.done) { txt_err("internal"); return false; }
        if (ctx.res > 0) { acked = true; break; }
        if (ctx.res < 0) {
            uint8_t code = ctx.frame_len >= 3 ? ctx.frame[2] : 0xFFu;
            if (code == 0x06) { sleep_ms(15); continue; }   /* busy: retry */
            printf("err: exception 0x%02X (%s)\n", code, modbus_exception_str(code));
            return false;
        }
        txt_err("timeout"); return false;
    }
    if (!acked) { txt_err("device_busy"); return false; }

    /* A single DALI command completes in ~25 ms; poll until it leaves PENDING. */
    absolute_time_t deadline = make_timeout_time_ms(300);
    uint16_t r = 0xFF00u;
    do {
        memset(&ctx, 0, sizeof(ctx));
        modbus_get_input_registers(slave, 0x0108u, 1, cli_mb_cb, &ctx);
        cli_mb_wait(&ctx);
        if (cli_mb_report_fail(&ctx)) return false;
        if (ctx.res >= 5) r = ((uint16_t)ctx.frame[3] << 8) | ctx.frame[4];
    } while ((r >> 8) == 0xFFu &&
             absolute_time_diff_us(get_absolute_time(), deadline) > 0);
    *result = r;
    return true;
}

/* Read `count` bytes from DALI memory `bank` starting at `offset`, from the
 * gear at short address `addr`.  Sequence: SET DTR1=bank, SET DTR0=offset, then
 * `count` READ MEMORY LOCATION (opcode 0xC5) — the gear auto-increments DTR0.
 * Sets *collision if any read garbles (two+ gear answering differently) and
 * *absent if a read NAKs (no gear there).  Returns false on transport failure. */
static bool dali_read_mem(uint8_t slave, uint8_t addr, uint8_t bank,
                          uint8_t offset, uint8_t count, uint8_t *out,
                          bool *collision, bool *absent) {
    uint16_t r;
    *collision = false;
    *absent    = false;
    if (!dali_raw_result(slave, (uint16_t)(0xC300u | bank), false, &r)) return false; /* SET DTR1 */
    uint8_t addr_byte = (uint8_t)((addr << 1) | 0x01u);   /* short-address command form */
    for (uint8_t i = 0; i < count; i++) {
        /* Re-set DTR0 before every read: not all gear auto-increment, and some
         * have unreadable holes (e.g. bank-0 offset 0x01) that desync a run. */
        if (!dali_raw_result(slave, (uint16_t)(0xA300u | (uint8_t)(offset + i)), false, &r))
            return false;
        uint16_t frame = (uint16_t)(((uint16_t)addr_byte << 8) | 0xC5u); /* READ MEMORY LOCATION */
        if (!dali_raw_result(slave, frame, false, &r)) return false;
        if      ((r >> 8) == 0x00u) out[i] = (uint8_t)(r & 0xFFu);   /* byte read */
        else if ((r >> 8) == 0x02u) { *collision = true; out[i] = 0xFFu; }  /* garbled = 2+ gear */
        else if (r == 0x0100u)      { if (i == 0) *absent = true; out[i] = 0xFFu; } /* NAK: no gear / hole */
        else                        out[i] = 0xFFu;
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
    printf("relay <ch> {on|off|toggle}                 set one relay\n");
    printf("dali <slave> <addr> {on|off|toggle|<bri>}  set one ballast\n");
    printf("btn <fixture> <button> [tap|press|release]  synthesize a button event (test)\n");
    printf("dali <slave> g<group> {on|off|<bri>}       set one group (no toggle)\n");
    printf("dali_raw <slave> <cmd16> [twice]           send raw 16-bit DALI frame, report result\n");
    printf("read_mem <slave> <addr> <bank> <off> <n>   read n bytes from a ballast memory bank (hex)\n");
    printf("identify <slave> <addr>                    read a ballast's GTIN + serial (bank 0)\n");
    printf("blink <slave> <addr>                       flash one ballast to locate it physically\n");
    printf("gw_reboot <slave>                          reboot a TinyModbus gateway (recover stuck bus)\n");
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
    printf("ls [<path>]                                list files in path (default /)\n");
    printf("cat <path>                                 dump file contents to console\n");
    printf("map set <fix> <btn> <s>:<a>[,<s>:<a>...]   set mapping (a may be N or GN)\n");
    printf("map del <fix> <btn>                        delete mapping\n");
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
               "baud=%lu rs485=%s relay_slaves=",
               g_config.long_press_ms, g_config.dim_repeat_ms,
               g_config.reversal_window_ms,
               (unsigned long)modbus_get_baud(),
               modbus_de_mode_to_str(modbus_get_de_mode()));
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
                printf("relay %d: %s\n", ch,
                       state_cache_get_relay_on((uint8_t)ch) ? "on" : "off");
            }
            return;
        }
        int ch = 0;
        if (!parse_int_range(cs, 0, RELAY_COUNT - 1, &ch)) {
            txt_err("out_of_range"); return;
        }
        printf("relay %d: %s\n", ch,
               state_cache_get_relay_on((uint8_t)ch) ? "on" : "off");
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
static void cmd_relay(char **saveptr) {
    char *cs = strtok_r(NULL, " \t", saveptr);
    char *vs = strtok_r(NULL, " \t", saveptr);
    int ch = 0;
    if (!parse_int_range(cs, 0, RELAY_COUNT - 1, &ch)) {
        txt_err("out_of_range"); return;
    }
    int v = parse_on_off_toggle(vs);
    if (v < 0) { txt_err("bad_arg"); return; }
    bool target;
    if (v == 2) {
        target = !state_cache_get_relay_on((uint8_t)ch);
    } else {
        target = v == 1;
    }
    LOG_INFO("cli", "relay %d %s", ch, target ? "on" : "off");
    lighting_set_relay((uint8_t)ch, target);
    txt_ok();
}

/* ── dali <slave> <addr|gN> {on|off|toggle|<bri>} ──────────────────────── */
static void cmd_dali(char **saveptr) {
    char *ss = strtok_r(NULL, " \t", saveptr);
    char *as = strtok_r(NULL, " \t", saveptr);
    char *vs = strtok_r(NULL, " \t", saveptr);
    int slave = 0;
    if (!parse_int_range(ss, 0, 247, &slave)) {
        txt_err("invalid_slave"); return;
    }
    int idx = state_cache_slave_idx((uint8_t)slave);
    if (idx < 0) { txt_err("invalid_slave"); return; }
    if (!as || !vs) { txt_err("bad_arg"); return; }

    bool is_group = (as[0] == 'g' || as[0] == 'G');
    if (is_group) {
        int group = 0;
        if (!parse_int_range(as + 1, 0, 15, &group)) {
            txt_err("out_of_range"); return;
        }
        int v = parse_on_off_toggle(vs);
        if (v == 2) {
            txt_err("bad_arg group_toggle_unsupported");
            return;
        }
        if (v >= 0) {
            LOG_INFO("cli", "dali g%d on slave %d -> %s", group, slave, v ? "on" : "off");
            lighting_group_set_on_off((uint8_t)slave, (uint8_t)group, v == 1);
        } else {
            int bri = 0;
            if (!parse_int_range(vs, 0, 254, &bri)) {
                txt_err("bad_arg"); return;
            }
            LOG_INFO("cli", "dali g%d on slave %d -> bri=%d", group, slave, bri);
            lighting_group_set_level((uint8_t)slave, (uint8_t)group, (uint8_t)bri);
        }
        dirty_poll_mark_active((uint8_t)slave);
        txt_ok();
        return;
    }

    int addr = 0;
    if (!parse_int_range(as, 0, 63, &addr)) {
        txt_err("out_of_range"); return;
    }
    int v = parse_on_off_toggle(vs);
    if (v >= 0) {
        bool on;
        if (v == 2) {
            dali_ballast_state_t *b =
                state_cache_get_dali((uint8_t)idx, (uint8_t)addr);
            on = (b && b->arc_level > 0) ? false : true;
        } else {
            on = v == 1;
        }
        LOG_INFO("cli", "dali %d.%d -> %s", slave, addr, on ? "on" : "off");
        lighting_set_on_off((uint8_t)slave, (uint8_t)addr, on);
    } else {
        int bri = 0;
        if (!parse_int_range(vs, 0, 254, &bri)) {
            txt_err("bad_arg"); return;
        }
        LOG_INFO("cli", "dali %d.%d -> bri=%d", slave, addr, bri);
        lighting_dali_set_level((uint8_t)slave, (uint8_t)addr, (uint8_t)bri);
    }
    dirty_poll_mark_active((uint8_t)slave);
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

/* ── dali_raw <slave> <cmd16> [twice] ──────────────────────────────────────
 * Send an arbitrary 16-bit DALI forward frame to a gateway. cmd16 is the full
 * frame — address/selector byte in the high 8 bits, opcode/data in the low 8
 * bits — and accepts decimal or 0x-hex. It is written to the gateway RAW_CMD
 * holding register (0x1000 single send, 0x1001 send-twice). Append "twice" (or
 * "2x") for config commands that DALI requires be repeated within 100 ms (e.g.
 * STORE_DTR_AS_*, SET_SCENE). Reports the Modbus-level ack/exception, not the
 * DALI backward frame. */
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
    if (!dali_raw_result((uint8_t)slave, value, twice, &result)) return;
    /* Let dirty-poll pick up any resulting ballast state change. */
    dirty_poll_mark_active((uint8_t)slave);
    if ((result >> 8) == 0x00u)
        printf("ok: result 0x%04X (response byte 0x%02X)\n", result, result & 0xFFu);
    else
        printf("ok: result 0x%04X (%s)\n", result, dali_result_str(result));
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
        if (!dali_raw_result((uint8_t)slave, (uint16_t)(dapc | level), false, &r)) return;
        sleep_ms(300);
    }
    dali_raw_result((uint8_t)slave, (uint16_t)(dapc | 0xFEu), false, &r);  // leave on
    dirty_poll_mark_active((uint8_t)slave);
    txt_ok();
}

/* ── gw_reboot <slave> ─────────────────────────────────────────────────────
 * Force a TinyModbus gateway to perform a software reset by writing the magic
 * value 0xA5A5 to its SYSTEM_RESET register (0x1015). Recovery hatch for a
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
    modbus_set_holding_register((uint8_t)slave, 0x1015u, 0xA5A5u, cli_mb_cb, &ctx);
    cli_mb_wait(&ctx);
    if (cli_mb_report_fail(&ctx)) return;
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

    LOG_INFO("cli", "scan slave %d (blocking)", slave);
    printf("scanning slave %d...\n", slave);
    dirty_poll_full_scan((uint8_t)slave);
    int present = 0;
    for (int a = 0; a < DALI_ADDR_COUNT; a++) {
        dali_ballast_state_t *b =
            state_cache_get_dali((uint8_t)idx, (uint8_t)a);
        if (b && b->present) present++;
    }
    printf("scan complete: %d ballasts present\n", present);
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
static void cmd_map(char **saveptr) {
    char *sub = strtok_r(NULL, " \t", saveptr);
    if (!sub) { txt_err("bad_arg"); return; }

    if (strcasecmp(sub, "set") == 0) {
        char *fs = strtok_r(NULL, " \t", saveptr);
        char *bs = strtok_r(NULL, " \t", saveptr);
        char *ts = strtok_r(NULL, " \t", saveptr);
        int fix = 0, btn = 0;
        if (!parse_int_range(fs, 0, MAX_FIXTURES - 1, &fix) ||
            !parse_int_range(bs, 0, MAX_BUTTONS - 1, &btn) ||
            !ts) {
            txt_err("bad_arg"); return;
        }
        mapping_target_t targets[MAX_TARGETS_PER_BTN];
        uint8_t count = 0;
        char *pair_save = NULL;
        for (char *p = strtok_r(ts, ",", &pair_save);
             p && count < MAX_TARGETS_PER_BTN;
             p = strtok_r(NULL, ",", &pair_save)) {
            if (!parse_target_pair(p, &targets[count])) {
                txt_err("bad_arg"); return;
            }
            count++;
        }
        if (count == 0) { txt_err("bad_arg"); return; }
        LOG_INFO("cli", "map set F%d B%d count=%u", fix, btn, count);
        if (!mapping_set((uint8_t)fix, (uint8_t)btn, targets, count)) {
            txt_err("save_failed"); return;
        }
        txt_ok();
        return;
    }

    if (strcasecmp(sub, "del") == 0) {
        char *fs = strtok_r(NULL, " \t", saveptr);
        char *bs = strtok_r(NULL, " \t", saveptr);
        int fix = 0, btn = 0;
        if (!parse_int_range(fs, 0, MAX_FIXTURES - 1, &fix) ||
            !parse_int_range(bs, 0, MAX_BUTTONS - 1, &btn)) {
            txt_err("bad_arg"); return;
        }
        LOG_INFO("cli", "map del F%d B%d", fix, btn);
        if (!mapping_delete((uint8_t)fix, (uint8_t)btn)) {
            txt_err("save_failed"); return;
        }
        txt_ok();
        return;
    }

    txt_err("bad_arg");
}

/* ── reboot ────────────────────────────────────────────────────────────── */
static void cmd_reboot(void) {
    LOG_INFO("cli", "reboot");
    printf("rebooting...\n");
    /* Give the line a moment to drain the USB CDC buffer */
    sleep_ms(50);
    watchdog_enable(200, 0);
    while (1) { ; }
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

/* ── Top-level text dispatcher ─────────────────────────────────────────── */
static void handle_text_line(const char *original_line) {
    LOG_INFO("cli", "rx: %s", original_line);

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
               LEVT_SCAN_COMPLETE, LEVT_GROUP } levt_type_t;
typedef struct {
    levt_type_t type;
    union {
        struct { uint8_t ch; bool on; } relay;
        struct { uint8_t slave_id; uint8_t addr; uint8_t arc; uint8_t status; } dali;
        struct { uint8_t slave_id; uint64_t appeared; uint64_t disappeared; } presence;
        struct { uint8_t slave_id; uint8_t addr; bool lamp_fail; bool power_cycle; } fault;
        struct { uint8_t slave_id; uint64_t present_mask; } scan_complete;
        struct { uint8_t slave_id; uint8_t group; uint8_t bri; bool on; } group;
    };
} levt_t;

static void emit_event(const levt_t *e) {
    uint32_t ts = to_ms_since_boot(get_absolute_time());

    switch (e->type) {
    case LEVT_RELAY: {
        uint8_t rsid = g_config.relay_slave_count > 0
                       ? g_config.relay_slave_ids[0] : 0;
        printf("{\"t\":\"state_relay\",\"ts\":%lu,"
               "\"d\":{\"ch\":%u},\"v\":{\"on\":%s}}\n",
               (unsigned long)ts, e->relay.ch,
               e->relay.on ? "true" : "false");
        LOG_DBG("serial", "evt state_relay slave=%u addr=%u",
                rsid, e->relay.ch);
        break;
    }

    case LEVT_DALI: {
        bool fading    = (e->dali.status & 0x10) != 0;
        bool lamp_fail = (e->dali.status & 0x02) != 0;
        printf("{\"t\":\"state_dali\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u,\"addr\":%u},"
               "\"v\":{\"on\":%s,\"bri\":%u,\"fading\":%s,"
               "\"lamp_fail\":%s}}\n",
               (unsigned long)ts,
               e->dali.slave_id, e->dali.addr,
               e->dali.arc > 0 ? "true" : "false",
               e->dali.arc,
               fading    ? "true" : "false",
               lamp_fail ? "true" : "false");
        LOG_DBG("serial", "evt state_dali slave=%u addr=%u",
                e->dali.slave_id, e->dali.addr);
        break;
    }

    case LEVT_PRESENCE: {
        char appeared[192]    = "[";
        char disappeared[192] = "[";
        int  ap = 1, dp = 1;
        bool fa = true, fd = true;
        for (int a = 0; a < 64; a++) {
            if (e->presence.appeared    & ((uint64_t)1 << a))
                ap += snprintf(appeared + ap, sizeof(appeared) - ap,
                               "%s%d", fa ? (fa=false,"") : ",", a);
            if (e->presence.disappeared & ((uint64_t)1 << a))
                dp += snprintf(disappeared + dp, sizeof(disappeared) - dp,
                               "%s%d", fd ? (fd=false,"") : ",", a);
        }
        strncat(appeared,    "]", sizeof(appeared)    - strlen(appeared)    - 1);
        strncat(disappeared, "]", sizeof(disappeared) - strlen(disappeared) - 1);
        printf("{\"t\":\"presence\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u},"
               "\"v\":{\"appeared\":%s,\"disappeared\":%s}}\n",
               (unsigned long)ts, e->presence.slave_id, appeared, disappeared);
        LOG_DBG("serial", "presence slave=%u +0x%016llx -0x%016llx",
                e->presence.slave_id,
                (unsigned long long)e->presence.appeared,
                (unsigned long long)e->presence.disappeared);
        break;
    }

    case LEVT_FAULT:
        printf("{\"t\":\"fault\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u,\"addr\":%u},"
               "\"v\":{\"lamp_fail\":%s,\"power_cycle\":%s}}\n",
               (unsigned long)ts,
               e->fault.slave_id, e->fault.addr,
               e->fault.lamp_fail   ? "true" : "false",
               e->fault.power_cycle ? "true" : "false");
        LOG_INFO("serial", "fault slave=%u.%u lamp=%s pwr=%s",
                 e->fault.slave_id, e->fault.addr,
                 e->fault.lamp_fail   ? "true" : "false",
                 e->fault.power_cycle ? "true" : "false");
        break;

    case LEVT_SCAN_COMPLETE: {
        /* Emit present bitmask as a JSON array of responding addresses.
         * fix #6: bridge uses this to set per-ballast MQTT availability.
         * Also emit per-group membership read live from state_cache —
         * see Layer 2 design notes (Option A). */
        char present_arr[256] = "[";
        int  pp = 1;
        bool first = true;
        for (int a = 0; a < 64; a++) {
            if (e->scan_complete.present_mask & ((uint64_t)1 << a)) {
                pp += snprintf(present_arr + pp, sizeof(present_arr) - pp,
                               "%s%d", first ? "" : ",", a);
                first = false;
            }
        }
        strncat(present_arr, "]",
                sizeof(present_arr) - strlen(present_arr) - 1);

        /* Build the groups object. Empty groups are omitted. Member arrays
         * are intersected with the present mask. */
        /* Worst case: 16 groups × 64 members × 3 chars per "NN," ~ 3200,
         * plus per-group "G":[]" framing. Round up generously. */
        char groups_obj[4096] = "{";
        int  gp = 1;
        bool first_group = true;
        int  slave_idx = state_cache_slave_idx(e->scan_complete.slave_id);
        if (slave_idx >= 0) {
            for (uint8_t g = 0; g < 16; g++) {
                uint64_t members =
                    state_cache_get_dali_group((uint8_t)slave_idx, g)
                    & e->scan_complete.present_mask;
                if (!members) continue;
                gp += snprintf(groups_obj + gp, sizeof(groups_obj) - gp,
                               "%s\"%u\":[", first_group ? "" : ",", g);
                first_group = false;
                bool first_m = true;
                for (int a = 0; a < 64; a++) {
                    if (members & ((uint64_t)1 << a)) {
                        gp += snprintf(groups_obj + gp,
                                       sizeof(groups_obj) - gp,
                                       "%s%d", first_m ? "" : ",", a);
                        first_m = false;
                    }
                }
                gp += snprintf(groups_obj + gp, sizeof(groups_obj) - gp, "]");
            }
        }
        strncat(groups_obj, "}",
                sizeof(groups_obj) - strlen(groups_obj) - 1);

        printf("{\"t\":\"scan_complete\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u},"
               "\"v\":{\"present\":%s,\"groups\":%s}}\n",
               (unsigned long)ts,
               e->scan_complete.slave_id,
               present_arr,
               groups_obj);
        {
            int present_count = 0;
            int group_count   = 0;
            for (int a = 0; a < 64; a++) {
                if (e->scan_complete.present_mask & ((uint64_t)1 << a))
                    present_count++;
            }
            int sidx = state_cache_slave_idx(e->scan_complete.slave_id);
            if (sidx >= 0) {
                for (uint8_t g = 0; g < 16; g++) {
                    uint64_t members =
                        state_cache_get_dali_group((uint8_t)sidx, g)
                        & e->scan_complete.present_mask;
                    if (members) group_count++;
                }
            }
            LOG_INFO("serial",
                     "scan_complete slave=%u present_count=%d group_count=%d",
                     e->scan_complete.slave_id, present_count, group_count);
        }
        break;
    }

    case LEVT_GROUP:
        printf("{\"t\":\"state_group\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u,\"group\":%u},"
               "\"v\":{\"on\":%s,\"bri\":%u}}\n",
               (unsigned long)ts,
               e->group.slave_id, e->group.group,
               e->group.on ? "true" : "false",
               e->group.bri);
        LOG_DBG("serial", "evt state_group slave=%u addr=%u",
                e->group.slave_id, e->group.group);
        break;
}
}

/* ── Public interface ────────────────────────────────────────────────────── */
void serial_protocol_init(void) {
    /* Nothing — stdio_usb is enabled by pico_enable_stdio_usb in CMakeLists */
}

void serial_tx_hello(void) {
    printf("{\"t\":\"hello\",\"id\":0,\"ts\":0,"
           "\"d\":{\"fw_version\":\"2.0.0\","
           "\"relay_slaves\":[");
    for (int i = 0; i < g_config.relay_slave_count; i++) {
        printf("%s%u", i > 0 ? "," : "", g_config.relay_slave_ids[i]);
    }
    printf("],\"dali_slaves\":[");
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        printf("%s%u", i > 0 ? "," : "", g_config.dali_slave_ids[i]);
    }
    printf("],\"long_press_ms\":%u,\"dim_repeat_ms\":%u,"
           "\"reversal_window_ms\":%u,\"baud\":%lu}}\n",
           g_config.long_press_ms, g_config.dim_repeat_ms,
           g_config.reversal_window_ms,
           (unsigned long)modbus_get_baud());
    LOG_INFO("serial", "hello");
}

void serial_tx_ack(unsigned cmd_id, bool ok, const char *err) {
    if (ok) {
        printf("{\"t\":\"ack\",\"id\":%u,\"d\":{\"ok\":true}}\n", cmd_id);
    } else {
        printf("{\"t\":\"ack\",\"id\":%u,\"d\":{\"ok\":false,\"err\":\"%s\"}}\n",
               cmd_id, err ? err : "unknown");
    }
    LOG_INFO("serial", "ack id=%u ok=%s", cmd_id, ok ? "true" : "false");
}

void serial_tx_config_full(void) {
    static char buf[4096];
    int n = mapping_serialise_all(buf, sizeof(buf));
    if (n > 0) printf("%.*s", n, buf);
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

    /* Drain outbound state events */
    levt_t evt;
    while (lighting_evt_try_pop(&evt)) {
        emit_event(&evt);
    }

    /* Emit any pending provision status update */
    serial_provision_poll();
}
