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
#include "config.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "modbus.h"
#include "log.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <hardware/watchdog.h>
#include <pico/stdio.h>
#include <pico/time.h>

/* Consume levt_t events from lighting_actions */
extern bool lighting_evt_try_pop(void *out);  /* defined in lighting_actions.c */

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
            if (has_slave && (has_addr || has_group)) {
                out[count].slave = (uint8_t)slave;
                if (has_group && (uint8_t)slave != SLAVE_RELAY) {
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
    if (!parse_int_range(slave_buf, 0, 247, &slave)) return false;
    out->slave = (uint8_t)slave;

    const char *addr_s = colon + 1;
    if (addr_s[0] == 'G' || addr_s[0] == 'g') {
        int group = 0;
        if (!parse_int_range(addr_s + 1, 0, 15, &group)) return false;
        if ((uint8_t)slave == SLAVE_RELAY) return false;
        out->addr = TARGET_MAKE_GROUP((uint8_t)group);
    } else {
        int addr = 0;
        if (!parse_int_range(addr_s, 0, 63, &addr)) return false;
        out->addr = (uint8_t)addr;
    }
    return true;
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
    printf("dali <slave> g<group> {on|off|<bri>}       set one group (no toggle)\n");
    printf("dim <slave> <addr> {up|down}               step one ballast\n");
    printf("dim <slave> g<group> {up|down}             step one group\n");
    printf("provision <slave> {all|new|<addr>}         start commissioning\n");
    printf("provision abort                            stop commissioning\n");
    printf("scan <slave>                               blocking full scan\n");
    printf("map set <fix> <btn> <s>:<a>[,<s>:<a>...]   set mapping (a may be N or GN)\n");
    printf("map del <fix> <btn>                        delete mapping\n");
    printf("log <level>                                set verbosity (off|err|warn|info|dbg)\n");
    printf("show log                                   current verbosity\n");
    printf("reboot                                     watchdog reset\n");
    printf("resync                                     re-emit hello + all state + mappings\n");
}

/* ── show subcommands ──────────────────────────────────────────────────── */
static void cmd_show(char **saveptr) {
    char *what = strtok_r(NULL, " \t", saveptr);
    if (!what) { txt_err("bad_arg"); return; }

    if (strcasecmp(what, "config") == 0) {
        printf("long_press_ms=%u dim_repeat_ms=%u reversal_window_ms=%u "
               "baud=%lu relay_slave=%u dali_slaves=",
               g_config.long_press_ms, g_config.dim_repeat_ms,
               g_config.reversal_window_ms,
               (unsigned long)modbus_get_baud(),
               g_config.relay_slave_id);
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
    if (strcasecmp(cmd, "dali") == 0) {
        cmd_dali(&saveptr);
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
    if (strcasecmp(cmd, "map") == 0) {
        cmd_map(&saveptr);
        return;
    }
    if (strcasecmp(cmd, "log") == 0) {
        cmd_log(&saveptr);
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
    case LEVT_RELAY:
        printf("{\"t\":\"state_relay\",\"ts\":%lu,"
               "\"d\":{\"ch\":%u},\"v\":{\"on\":%s}}\n",
               (unsigned long)ts, e->relay.ch,
               e->relay.on ? "true" : "false");
        LOG_DBG("serial", "evt state_relay slave=%u addr=%u",
                g_config.relay_slave_id, e->relay.ch);
        break;

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
           "\"relay_slave\":%u,\"dali_slaves\":[",
           g_config.relay_slave_id);
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
            if (line_pos > 0) {
                line_buf[line_pos] = '\0';
                handle_line(line_buf);
                line_pos = 0;
            }
        } else if (line_pos < SERIAL_LINE_MAX - 1) {
            line_buf[line_pos++] = (char)ch;
        } else {
            line_pos = 0; /* overflow — discard */
        }
    }

    /* Drain outbound state events */
    levt_t evt;
    while (lighting_evt_try_pop(&evt)) {
        emit_event(&evt);
    }

    /* Emit any pending provision status update */
    serial_provision_poll();
}
