/*
 * serial_protocol.c — JSON line protocol over USB CDC.
 *
 * Replaces lualoop.c.  Uses stdio_getchar_timeout_us(0) to non-blockingly
 * read USB CDC input — exactly as lua_poll() did in the original firmware.
 * Outbound messages use printf() — same as the original.
 *
 * All parsing is single-core; no multicore synchronisation needed.
 */

#include "serial_protocol.h"
#include "dali_provision.h"
#include "lighting_actions.h"
#include "mapping.h"
#include "config.h"
#include "state_cache.h"
#include "dirty_poll.h"
#include "modbus.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
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

/* ── Process one inbound JSON line ──────────────────────────────────────── */
static void handle_line(const char *line) {
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
        if (has_bri) {
            lighting_dali_set_level((uint8_t)slave, (uint8_t)addr, (uint8_t)bri);
        } else {
            lighting_set_on_off((uint8_t)slave, (uint8_t)addr, on);
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
        /* Pass raw addr_byte directly to FC06 value */
        uint16_t reg = on ? 0x1003u : 0x1004u;
        if (has_bri) {
            uint16_t val = ((uint16_t)(addr_byte & 0xFE) << 8) | (uint8_t)bri;
            modbus_set_holding_register((uint8_t)slave, 0x1002u, val, NULL, NULL);
        } else {
            modbus_set_holding_register((uint8_t)slave, reg,
                                        (uint16_t)addr_byte, NULL, NULL);
        }
        dirty_poll_mark_active((uint8_t)slave);
        serial_tx_ack(cmd_id, true, NULL);
        return;
    }

    /* ── get_config ────────────────────────────────────────────────────── */
    if (strcmp(t, "get_config") == 0) {
        serial_tx_config_full();
        return;
    }

    /* ── set_mapping ───────────────────────────────────────────────────── */
    if (strcmp(t, "set_mapping") == 0) {
        int fixture = 0, button = 0;
        json_int(line, "fixture", &fixture);
        json_int(line, "button",  &button);

        mapping_target_t targets[MAX_TARGETS_PER_BTN];
        uint8_t count = 0;
        const char *tp = strstr(line, "\"targets\"");
        if (tp) {
            const char *p = tp;
            while (count < MAX_TARGETS_PER_BTN && (p = strchr(p, '{')) != NULL) {
                const char *end = strchr(p, '}');
                if (!end) break;
                char obj[64];
                size_t len = (size_t)(end - p + 1);
                if (len < sizeof(obj)) {
                    memcpy(obj, p, len);
                    obj[len] = '\0';
                    int slave = 0, addr = 0;
                    if (json_int(obj, "slave", &slave) &&
                        json_int(obj, "addr",  &addr)) {
                        targets[count].slave = (uint8_t)slave;
                        targets[count].addr  = (uint8_t)addr;
                        count++;
                    }
                }
                p = end + 1;
            }
        }
        bool ok = mapping_set((uint8_t)fixture, (uint8_t)button, targets, count);
        /* Echo mapping update back so bridge can refresh retained MQTT topic */
        if (ok) {
            char mbuf[256];
            int n = snprintf(mbuf, sizeof(mbuf),
                             "{\"t\":\"config_mapping\","
                             "\"d\":{\"fixture\":%d,\"button\":%d},"
                             "\"v\":{\"targets\":[",
                             fixture, button);
            for (int i = 0; i < count; i++) {
                n += snprintf(mbuf + n, sizeof(mbuf) - n,
                              "%s{\"slave\":%u,\"addr\":%u}",
                              i > 0 ? "," : "",
                              targets[i].slave, targets[i].addr);
            }
            n += snprintf(mbuf + n, sizeof(mbuf) - n, "]}}\n");
            printf("%s", mbuf);
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
            printf("{\"t\":\"config_mapping\","
                   "\"d\":{\"fixture\":%d,\"button\":%d},"
                   "\"v\":{\"targets\":[]}}\n", fixture, button);
        }
        serial_tx_ack(cmd_id, ok, ok ? NULL : "save_failed");
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

/* ── State event serialisation ───────────────────────────────────────────── */
/* lighting_actions.c defines levt_t internally; we replicate the minimal
 * struct here for serialisation.  In a real project these would share a header. */
typedef enum { LEVT_RELAY, LEVT_DALI, LEVT_PRESENCE, LEVT_FAULT,
               LEVT_SCAN_COMPLETE } levt_type_t;
typedef struct {
    levt_type_t type;
    union {
        struct { uint8_t ch; bool on; } relay;
        struct { uint8_t slave_id; uint8_t addr; uint8_t arc; uint8_t status; } dali;
        struct { uint8_t slave_id; uint64_t appeared; uint64_t disappeared; } presence;
        struct { uint8_t slave_id; uint8_t addr; bool lamp_fail; bool power_cycle; } fault;
        struct { uint8_t slave_id; uint64_t present_mask; } scan_complete;
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
        break;

    case LEVT_SCAN_COMPLETE: {
        /* Emit present bitmask as a JSON array of responding addresses.
         * fix #6: bridge uses this to set per-ballast MQTT availability. */
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
        printf("{\"t\":\"scan_complete\",\"ts\":%lu,"
               "\"d\":{\"slave\":%u},"
               "\"v\":{\"present\":%s}}\n",
               (unsigned long)ts,
               e->scan_complete.slave_id,
               present_arr);
        break;
    }
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
}

void serial_tx_ack(unsigned cmd_id, bool ok, const char *err) {
    if (ok) {
        printf("{\"t\":\"ack\",\"id\":%u,\"d\":{\"ok\":true}}\n", cmd_id);
    } else {
        printf("{\"t\":\"ack\",\"id\":%u,\"d\":{\"ok\":false,\"err\":\"%s\"}}\n",
               cmd_id, err ? err : "unknown");
    }
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
