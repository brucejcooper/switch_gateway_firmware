/*
 * ha_bridge.c — Home Assistant discovery + availability + translation (core 1).
 *
 * Reproduces bridge/mqtt_bridge.py: MQTT auto-discovery, the availability model,
 * event->state-topic translation, and inbound command translation (jsmn).
 *
 * Everything that can burst is published INCREMENTALLY, gated on free socket-TX
 * space, a bounded amount per tick:
 *   - the ~900-entity discovery set (one entity per disc_step),
 *   - the per-slave 64-address availability sweep from scan_complete.
 * Blasting these synchronously would overrun the socket TX buffer and freeze the
 * core-1 heartbeat. Payloads are built one at a time into a single reused buffer.
 */

#include "ha_bridge.h"
#include "mqtt_app.h"
#include "intercore.h"
#include "web_state.h"   /* fan-out: feed the web read model from the event drain */
#include "ping.h"        /* IC_EVT_PING_REQ is routed here, not published */
#include "ota.h"         /* IC_EVT_OTA_ACK/OTA_STATUS are routed here, not published */
#include "fwupdate.h"    /* HA 'update' entity: manifest check + install trigger */
#include "log.h"
#include "version.h"     /* FW_VERSION */
#include "jsmn.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <stdarg.h>

#define MFR_MECH   "Mechination"
#define MFR_WAVE   "Waveshare"
#define CTRL_MODEL "WIZnet W5100S-EVB-Pico Controller"

/* HA number-entity regex patterns. The bytes on the wire must contain JSON-
 * escaped backslashes (\\), so each regex backslash is four backslashes in C. */
#define BINDING_PATTERN \
  "^\\\\s*(\\\\d{1,3}\\\\s*:\\\\s*[gG]?\\\\d{1,3}(\\\\s*,\\\\s*\\\\d{1,3}\\\\s*:\\\\s*[gG]?\\\\d{1,3})*)?\\\\s*$"
#define MEMBERS_PATTERN \
  "^\\\\s*(\\\\d{1,2}(\\\\s*,\\\\s*\\\\d{1,2})*)?\\\\s*$"

#define DALI_ADDRS    64
#define DALI_GROUPS   16
#define RELAY_CHANS   32
#define MAX_FIX       24
#define MAX_BTN        7
#define PL_CAP      1536
#define DISC_BATCH     6
#define TX_GATE     1100        /* min free TX before a discovery/sweep step */
#define PUMP_GATE    900        /* min free TX before draining a state event */
#define SWEEP_BATCH    8        /* availability topics per sweep tick */

static char s_pl[PL_CAP];
static char s_top[112];

/* Clamped accumulating snprintf into the shared s_pl payload buffer. Guards the
 * `s_pl + n, PL_CAP - n` idiom: once a write would overflow, n saturates at
 * PL_CAP so the next call's (PL_CAP - n) can't underflow to a huge size_t and
 * run snprintf off the end of s_pl. Identical to a bare `n += snprintf(...)`
 * whenever the payload fits — the only case that occurs today (~600 B vs 1536). */
static int pcat(int n, const char *fmt, ...) {
    if (n < 0 || n >= PL_CAP) return PL_CAP;
    va_list ap;
    va_start(ap, fmt);
    int w = vsnprintf(s_pl + n, (size_t)(PL_CAP - n), fmt, ap);
    va_end(ap);
    if (w < 0) return n;
    n += w;
    return n > PL_CAP ? PL_CAP : n;
}

/* ── publish helpers ─────────────────────────────────────────────────────── */
static void pub_cfg(const char *component, const char *objid) {
    snprintf(s_top, sizeof s_top, "homeassistant/%s/%s/config", component, objid);
    mqtt_app_publish(s_top, s_pl, strlen(s_pl), true);
}
static void pub_str(const char *topic, const char *payload, bool retain) {
    mqtt_app_publish(topic, payload, strlen(payload), retain);
}
static int dev(int n, const char *ids, const char *name,
               const char *mfr, const char *model, const char *via) {
    n = pcat(n, ",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
                "\"manufacturer\":\"%s\",\"model\":\"%s\"", ids, name, mfr, model);
    if (via) n = pcat(n, ",\"via_device\":\"%s\"", via);
    n = pcat(n, "}}");
    return n;
}

static int slave_idx(uint8_t id) {
    const ic_snapshot_t *sn = intercore_snapshot();
    for (int i = 0; i < sn->dali_count; i++) if (sn->dali_slaves[i] == id) return i;
    return -1;
}

/* ── discovery builders ──────────────────────────────────────────────────── */
/* One size for every discovery identifier / display-name / topic-fragment
 * buffer in this file. These were individually sized 24/28/32/40/48, and the
 * one that was too small (controller config object_ids, 32 vs the 35 chars
 * "lighting_cfg_dali_sweep_interval_ms" needs) truncated silently — snprintf
 * does not complain and the discovery topic is the only place it shows. A
 * single generous constant removes the whole class: adding a longer entity key
 * cannot quietly cut a topic short. Cost is stack in core-1 discovery frames,
 * which have 8 KB to play with. */
#define DISC_BUF 64

static void disc_plate_event(uint8_t fix, uint8_t btn) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], objid[DISC_BUF];
    /* No value_template: HA's MQTT *event* platform parses the raw payload as
     * JSON and takes event_type from it (and any extra keys as attributes). A
     * value_template would be applied first and its *result* must itself be a
     * JSON object — "{{ value_json.event_type }}" renders the bare string
     * "press", which HA rejects ("No valid JSON event payload"). The publish
     * already sends {"event_type":"press"} (see ha_bridge_publish_event), which
     * is exactly what the platform wants. */
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_switch_%02u_btn_%u\",\"name\":\"Button %u\","
        "\"state_topic\":\"%s/button/%u/%u\","
        "\"event_types\":[\"press\",\"release\",\"click\",\"long_press\",\"long_press_repeat\",\"stuck\",\"unstuck\"],"
        "\"availability_topic\":\"%s/availability\",\"icon\":\"mdi:gesture-tap-button\"",
        fix, btn, btn + 1, P, fix, btn, P);
    snprintf(ids, sizeof ids, "lighting_switch_plate_%02u", fix);
    snprintf(name, sizeof name, "Fixture %02u", fix + 1);
    dev(n, ids, name, MFR_MECH, "Switch Gateway Button Matrix", "lighting_controller");
    snprintf(objid, sizeof objid, "lighting_switch_%02u_btn_%u", fix, btn);
    pub_cfg("event", objid);
}

static void disc_plate_binding(uint8_t fix, uint8_t btn) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_binding_%02u_btn_%u\",\"name\":\"Button %u Binding\","
        "\"command_topic\":\"%s/binding/%u/%u/set\",\"state_topic\":\"%s/binding/%u/%u/state\","
        "\"pattern\":\"%s\",\"availability_topic\":\"%s/availability\","
        "\"entity_category\":\"config\",\"icon\":\"mdi:link-variant\"",
        fix, btn, btn + 1, P, fix, btn, P, fix, btn, BINDING_PATTERN, P);
    snprintf(ids, sizeof ids, "lighting_switch_plate_%02u", fix);
    snprintf(name, sizeof name, "Fixture %02u", fix + 1);
    dev(n, ids, name, MFR_MECH, "Switch Gateway Button Matrix", "lighting_controller");
    snprintf(objid, sizeof objid, "lighting_binding_%02u_btn_%u", fix, btn);
    pub_cfg("text", objid);
    snprintf(s_top, sizeof s_top, "%s/binding/%u/%u/state", P, fix, btn);
    pub_str(s_top, "", true);
}

/* Per-fixture "Button Stuck" diagnostic: ON while any button on the plate has
 * been held past the FSM's stuck threshold (commands suppressed until release).
 * State topic is retained and re-seeded from the live mask at discovery so a
 * reconnect doesn't hide an ongoing stuck condition. */
static uint8_t s_stuck_mask[MAX_FIX];

static void disc_plate_stuck(uint8_t fix) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_switch_%02u_stuck\",\"name\":\"Button Stuck\","
        "\"state_topic\":\"%s/plate/%u/stuck\","
        "\"device_class\":\"problem\",\"entity_category\":\"diagnostic\","
        "\"availability_topic\":\"%s/availability\",\"icon\":\"mdi:gesture-tap-hold\"",
        fix, P, fix, P);
    snprintf(ids, sizeof ids, "lighting_switch_plate_%02u", fix);
    snprintf(name, sizeof name, "Fixture %02u", fix + 1);
    dev(n, ids, name, MFR_MECH, "Switch Gateway Button Matrix", "lighting_controller");
    snprintf(objid, sizeof objid, "lighting_switch_%02u_stuck", fix);
    pub_cfg("binary_sensor", objid);
    snprintf(s_top, sizeof s_top, "%s/plate/%u/stuck", P, fix);
    pub_str(s_top, s_stuck_mask[fix] ? "ON" : "OFF", true);
}

static void disc_relay(uint8_t slave, uint8_t ch) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_relay_%u\",\"name\":null,"
        "\"state_topic\":\"%s/relay/%u/state\",\"command_topic\":\"%s/relay/%u/set\","
        "\"state_value_template\":\"{{ value_json.state }}\","
        "\"payload_on\":\"{\\\"state\\\":\\\"ON\\\"}\","
        "\"payload_off\":\"{\\\"state\\\":\\\"OFF\\\"}\","
        "\"availability\":[{\"topic\":\"%s/availability\"},"
        "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\"",
        ch, P, ch, P, ch, P, P, slave);
    snprintf(ids, sizeof ids, "lighting_relay_%u", ch);
    snprintf(name, sizeof name, "Relay Channel %u", ch);
    dev(n, ids, name, MFR_WAVE, "Relay Channel", "lighting_relay_board");
    snprintf(objid, sizeof objid, "lighting_relay_%u", ch);
    pub_cfg("switch", objid);
}

static void disc_ballast(uint8_t slave, uint8_t addr) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], gw[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_dali_%u_%u\",\"name\":null,\"schema\":\"json\","
        "\"state_topic\":\"%s/dali/%u/%u/state\",\"command_topic\":\"%s/dali/%u/%u/set\","
        "\"supported_color_modes\":[\"brightness\"],\"brightness_scale\":254,"
        "\"availability\":[{\"topic\":\"%s/availability\"},"
        "{\"topic\":\"%s/gateway/%u/availability\"},"
        "{\"topic\":\"%s/dali/%u/%u/availability\"}],\"availability_mode\":\"all\"",
        slave, addr, P, slave, addr, P, slave, addr, P, P, slave, P, slave, addr);
    snprintf(ids, sizeof ids, "lighting_dali_%u_%u", slave, addr);
    snprintf(name, sizeof name, "DALI %u.%02u", slave, addr);
    snprintf(gw, sizeof gw, "lighting_dali_gw_%u", slave);
    dev(n, ids, name, MFR_MECH, "DALI Ballast", gw);
    snprintf(objid, sizeof objid, "lighting_dali_%u_%u", slave, addr);
    pub_cfg("light", objid);
    snprintf(s_top, sizeof s_top, "%s/dali/%u/%u/availability", P, slave, addr);
    pub_str(s_top, "offline", true);
}

/* one of the 6 per-ballast config number entities */
static void disc_ballast_cfg(uint8_t slave, uint8_t addr, uint8_t k) {
    static const struct { const char *key, *name; int lo, hi; } F[] = {
        { "min", "Min Level", 0, 254 }, { "max", "Max Level", 0, 254 },
        { "power_on", "Power-On Level", 0, 254 }, { "sys_fail", "System-Failure Level", 0, 254 },
        { "fade_rate", "Fade Rate", 0, 15 }, { "fade_time", "Fade Time", 0, 15 },
    };
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], gw[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_dali_%u_%u_%s\",\"name\":\"%s\","
        "\"command_topic\":\"%s/dali/%u/%u/config/%s/set\","
        "\"state_topic\":\"%s/dali/%u/%u/config\",\"value_template\":\"{{ value_json.%s }}\","
        "\"min\":%d,\"max\":%d,\"step\":1,\"mode\":\"box\","
        "\"availability\":[{\"topic\":\"%s/availability\"},"
        "{\"topic\":\"%s/gateway/%u/availability\"},"
        "{\"topic\":\"%s/dali/%u/%u/availability\"}],\"availability_mode\":\"all\","
        "\"entity_category\":\"config\"",
        slave, addr, F[k].key, F[k].name, P, slave, addr, F[k].key,
        P, slave, addr, F[k].key, F[k].lo, F[k].hi, P, P, slave, P, slave, addr);
    snprintf(ids, sizeof ids, "lighting_dali_%u_%u", slave, addr);
    snprintf(name, sizeof name, "DALI %u.%02u", slave, addr);
    snprintf(gw, sizeof gw, "lighting_dali_gw_%u", slave);
    dev(n, ids, name, MFR_MECH, "DALI Ballast", gw);
    snprintf(objid, sizeof objid, "lighting_dali_%u_%u_%s", slave, addr, F[k].key);
    pub_cfg("number", objid);
}

static void disc_group_light(uint8_t slave, uint8_t grp) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], gw[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_group_%u_%u\",\"name\":null,\"schema\":\"json\","
        "\"state_topic\":\"%s/group/%u/%u/state\",\"command_topic\":\"%s/group/%u/%u/set\","
        "\"supported_color_modes\":[\"brightness\"],\"brightness_scale\":254,"
        "\"availability\":[{\"topic\":\"%s/availability\"},"
        "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\"",
        slave, grp, P, slave, grp, P, slave, grp, P, P, slave);
    snprintf(ids, sizeof ids, "lighting_group_%u_%u", slave, grp);
    snprintf(name, sizeof name, "DALI Group %u.%u", slave, grp);
    snprintf(gw, sizeof gw, "lighting_dali_gw_%u", slave);
    dev(n, ids, name, MFR_MECH, "DALI Group", gw);
    snprintf(objid, sizeof objid, "lighting_group_%u_%u", slave, grp);
    pub_cfg("light", objid);
    snprintf(s_top, sizeof s_top, "%s/group/%u/%u/state", P, slave, grp);
    pub_str(s_top, "{\"state\":\"OFF\"}", true);
}

static void disc_group_members(uint8_t slave, uint8_t grp) {
    const char *P = mqtt_app_prefix();
    char ids[DISC_BUF], name[DISC_BUF], gw[DISC_BUF], objid[DISC_BUF];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_group_%u_%u_members\",\"name\":\"Members\","
        "\"command_topic\":\"%s/group/%u/%u/members/set\","
        "\"state_topic\":\"%s/group/%u/%u/members/state\","
        "\"pattern\":\"%s\",\"availability\":[{\"topic\":\"%s/availability\"},"
        "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\","
        "\"entity_category\":\"config\",\"icon\":\"mdi:format-list-bulleted\"",
        slave, grp, P, slave, grp, P, slave, grp, MEMBERS_PATTERN, P, P, slave);
    snprintf(ids, sizeof ids, "lighting_group_%u_%u", slave, grp);
    snprintf(name, sizeof name, "DALI Group %u.%u", slave, grp);
    snprintf(gw, sizeof gw, "lighting_dali_gw_%u", slave);
    dev(n, ids, name, MFR_MECH, "DALI Group", gw);
    snprintf(objid, sizeof objid, "lighting_group_%u_%u_members", slave, grp);
    pub_cfg("text", objid);
    snprintf(s_top, sizeof s_top, "%s/group/%u/%u/members/state", P, slave, grp);
    pub_str(s_top, "", true);
}

static void disc_prov(uint8_t slave, uint8_t k) {
    const char *P = mqtt_app_prefix();
    char gwids[DISC_BUF], gwname[DISC_BUF], objid[DISC_BUF], status[DISC_BUF];
    snprintf(gwids, sizeof gwids, "lighting_dali_gw_%u", slave);
    snprintf(gwname, sizeof gwname, "DALI Gateway %u", slave);
    snprintf(status, sizeof status, "%s/provision/%u/status", P, slave);

    #define PROV_TAIL(cat) do { \
        int _n = pcat((int)strlen(s_pl), \
            ",\"availability\":[{\"topic\":\"%s/availability\"}," \
            "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\"," \
            "\"entity_category\":\"" cat "\"", P, P, slave); \
        dev(_n, gwids, gwname, MFR_MECH, "TinyModbus ATtiny1604 DALI Gateway", NULL); \
    } while (0)

    switch (k) {
    case 0:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_state\","
            "\"name\":\"Commission State\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.state }}\",\"icon\":\"mdi:lightning-bolt-circle\"",
            slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_state", slave); pub_cfg("sensor", objid);
        break;
    case 1:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_msg\","
            "\"name\":\"Commission Status\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.msg }}\",\"icon\":\"mdi:text-box-outline\"",
            slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_msg", slave); pub_cfg("sensor", objid);
        break;
    case 2:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_count\","
            "\"name\":\"Devices Found\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.devices_found }}\",\"icon\":\"mdi:counter\"",
            slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_count", slave); pub_cfg("sensor", objid);
        break;
    case 3: case 4: case 5: case 6: {
        static const char *suf[]  = { "all", "unaddressed", "address", "abort" };
        static const char *nm[]   = { "Commission All", "Commission New Devices",
                                      "Commission Address", "Abort Commission" };
        static const char *icon[] = { "mdi:lightbulb-group", "mdi:lightbulb-plus",
                                      "mdi:lightbulb-on", "mdi:stop-circle" };
        static const char *cmd[]  = { "start/all", "start/unaddressed", "start/address", "abort" };
        int b = k - 3;
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_%s\",\"name\":\"%s\","
            "\"command_topic\":\"%s/provision/%u/%s\",\"payload_press\":\"PRESS\",\"icon\":\"%s\"",
            slave, suf[b], nm[b], P, slave, cmd[b], icon[b]);
        PROV_TAIL("config");
        snprintf(objid, sizeof objid, "lighting_prov_%u_%s", slave, suf[b]); pub_cfg("button", objid);
        break;
    }
    case 7:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_target\","
            "\"name\":\"Target Address\",\"command_topic\":\"%s/provision/%u/start/target\","
            "\"state_topic\":\"%s/provision/%u/target_state\","
            "\"min\":0,\"max\":63,\"step\":1,\"mode\":\"box\",\"icon\":\"mdi:numeric\"",
            slave, P, slave, P, slave);
        PROV_TAIL("config");
        snprintf(objid, sizeof objid, "lighting_prov_%u_target", slave); pub_cfg("number", objid);
        break;
    case 8:
        snprintf(s_top, sizeof s_top, "%s/provision/%u/target_state", P, slave);
        pub_str(s_top, "0", true);
        break;
    case 9:
        snprintf(s_top, sizeof s_top, "%s/provision/%u/status", P, slave);
        pub_str(s_top, "{\"state\":\"idle\",\"msg\":\"Idle\",\"devices_found\":0}", true);
        break;
    }
    #undef PROV_TAIL
}

/* Per-gateway diagnostic sensors (uptime / reboots / reset reason), grouped
 * under the EXISTING gateway device (lighting_dali_gw_<slave>, the same device
 * disc_prov creates) — they're attributes of that gateway, not a new device.
 * k 0..2 = the three sensors, k 3 = seed an initial retained state. */
static void disc_gw_diag(uint8_t slave, uint8_t k) {
    const char *P = mqtt_app_prefix();
    char gwids[DISC_BUF], gwname[DISC_BUF], objid[DISC_BUF], diag[DISC_BUF];
    snprintf(gwids, sizeof gwids, "lighting_dali_gw_%u", slave);
    snprintf(gwname, sizeof gwname, "DALI Gateway %u", slave);
    snprintf(diag, sizeof diag, "%s/gateway/%u/diag", P, slave);

    #define GWD_TAIL() do { \
        int _n = pcat((int)strlen(s_pl), \
            ",\"availability\":[{\"topic\":\"%s/availability\"}," \
            "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\"," \
            "\"entity_category\":\"diagnostic\"", P, P, slave); \
        dev(_n, gwids, gwname, MFR_MECH, "TinyModbus ATtiny1604 DALI Gateway", NULL); \
    } while (0)

    switch (k) {
    case 0:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_gwdiag_%u_uptime\","
            "\"name\":\"Uptime\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.uptime }}\","
            "\"unit_of_measurement\":\"s\",\"device_class\":\"duration\","
            "\"icon\":\"mdi:timer-outline\"", slave, diag);
        GWD_TAIL();
        snprintf(objid, sizeof objid, "lighting_gwdiag_%u_uptime", slave); pub_cfg("sensor", objid);
        break;
    case 1:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_gwdiag_%u_reboots\","
            "\"name\":\"Reboots\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.reboots }}\","
            "\"icon\":\"mdi:restart\"", slave, diag);
        GWD_TAIL();
        snprintf(objid, sizeof objid, "lighting_gwdiag_%u_reboots", slave); pub_cfg("sensor", objid);
        break;
    case 2:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_gwdiag_%u_reset\","
            "\"name\":\"Reset Reason\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.reset_reason }}\","
            "\"icon\":\"mdi:information-outline\"", slave, diag);
        GWD_TAIL();
        snprintf(objid, sizeof objid, "lighting_gwdiag_%u_reset", slave); pub_cfg("sensor", objid);
        break;
    case 3:
        /* Seed a retained state so the entities populate before the first
         * gw_diag poll (≤ gw_diag_interval_ms) lands. */
        snprintf(s_top, sizeof s_top, "%s/gateway/%u/diag", P, slave);
        pub_str(s_top, "{\"uptime\":0,\"reboots\":0,\"reset_reason\":\"unknown\"}", true);
        break;
    }
    #undef GWD_TAIL
}

/* Controller-config entities. The tables live at file scope and the step bounds
 * are DERIVED from their sizes: they used to be hardcoded (k < 4, k - 4, k == 6,
 * total 7), so adding a fifth number silently published nothing — the new entity
 * sat in the table but past the guard. */
static const struct { const char *key, *name; int lo, hi, step; } CFG_NUM[] = {
    { "long_press_ms",          "Long-Press (ms)",            100,   5000,   50 },
    { "dim_repeat_ms",          "Dim Repeat (ms)",             50,   2000,   10 },
    { "reversal_window_ms",     "Reversal Window (ms)",        50,   2000,   10 },
    { "gw_diag_interval_ms",    "Gateway Diag Poll (ms)",       0, 600000, 1000 },
    { "dali_sweep_interval_ms", "DALI Background Sweep (ms)",   0, 600000, 1000 },
};
static const struct { const char *key, *name, *icon; } CFG_SENS[] = {
    { "fw_version", "Firmware",    "mdi:chip" },
    { "baud",       "Modbus Baud", "mdi:speedometer" },
};
#define CFG_NUM_N   (sizeof CFG_NUM  / sizeof CFG_NUM[0])
#define CFG_SENS_N  (sizeof CFG_SENS / sizeof CFG_SENS[0])
#define CFG_STEPS   (CFG_NUM_N + CFG_SENS_N + 1u)   /* + the relay_byteswap switch */

static void disc_controller(uint8_t k) {
    const char *P = mqtt_app_prefix();
    char objid[DISC_BUF];
    if (k >= CFG_NUM_N + CFG_SENS_N) {
        /* relay_byteswap switch — reverses Waveshare 32CH coil read-back order. */
        int n = snprintf(s_pl, PL_CAP,
            "{\"unique_id\":\"lighting_cfg_relay_byteswap\",\"name\":\"Relay Byte-Swap\","
            "\"command_topic\":\"%s/config/relay_byteswap/set\",\"state_topic\":\"%s/config/state\","
            "\"value_template\":\"{{ value_json.relay_byteswap }}\","
            "\"payload_on\":\"1\",\"payload_off\":\"0\",\"state_on\":\"1\",\"state_off\":\"0\","
            "\"availability_topic\":\"%s/availability\",\"entity_category\":\"config\","
            "\"icon\":\"mdi:swap-horizontal\"",
            P, P, P);
        pcat(n,
            ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
            "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
            "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
        pub_cfg("switch", "lighting_cfg_relay_byteswap");
        return;
    }
    if (k < CFG_NUM_N) {
        int n = snprintf(s_pl, PL_CAP,
            "{\"unique_id\":\"lighting_cfg_%s\",\"name\":\"%s\","
            "\"command_topic\":\"%s/config/%s/set\",\"state_topic\":\"%s/config/state\","
            "\"value_template\":\"{{ value_json.%s }}\",\"min\":%d,\"max\":%d,\"step\":%d,"
            "\"mode\":\"box\",\"availability_topic\":\"%s/availability\",\"entity_category\":\"config\"",
            CFG_NUM[k].key, CFG_NUM[k].name, P, CFG_NUM[k].key, P, CFG_NUM[k].key,
            CFG_NUM[k].lo, CFG_NUM[k].hi, CFG_NUM[k].step, P);
        pcat(n,
            ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
            "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
            "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
        snprintf(objid, sizeof objid, "lighting_cfg_%s", CFG_NUM[k].key); pub_cfg("number", objid);
    } else {
        int b = k - (int)CFG_NUM_N;
        int n = snprintf(s_pl, PL_CAP,
            "{\"unique_id\":\"lighting_cfg_%s\",\"name\":\"%s\",\"state_topic\":\"%s/config/state\","
            "\"value_template\":\"{{ value_json.%s }}\",\"availability_topic\":\"%s/availability\","
            "\"entity_category\":\"diagnostic\",\"icon\":\"%s\"",
            CFG_SENS[b].key, CFG_SENS[b].name, P, CFG_SENS[b].key, P, CFG_SENS[b].icon);
        pcat(n,
            ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
            "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
            "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
        snprintf(objid, sizeof objid, "lighting_cfg_%s", CFG_SENS[b].key); pub_cfg("sensor", objid);
    }
}

static void seed_gateway_avail(uint8_t slave) {
    snprintf(s_top, sizeof s_top, "%s/gateway/%u/availability", mqtt_app_prefix(), slave);
    pub_str(s_top, "online", true);
}

/* ── incremental discovery cursor ────────────────────────────────────────── */
typedef enum {
    PH_GW_AVAIL, PH_CONTROLLER, PH_PLATE_EVT, PH_PLATE_BIND, PH_PLATE_STUCK, PH_RELAY,
    PH_BALLAST, PH_BALLAST_CFG, PH_GROUP_L, PH_GROUP_M, PH_PROV, PH_GW_DIAG, PH_DONE
} disc_phase_t;

static bool         s_disc_active;
static disc_phase_t s_ph;
static uint16_t     s_i, s_j, s_k;

typedef enum { DISC_MORE, DISC_STALL, DISC_DONE } disc_res_t;

static disc_res_t disc_step(void) {
    const ic_snapshot_t *sn = intercore_snapshot();
    if (mqtt_app_tx_free() < TX_GATE) return DISC_STALL;

    switch (s_ph) {
    case PH_GW_AVAIL:
        if (s_i < sn->dali_count) { seed_gateway_avail(sn->dali_slaves[s_i]); s_i++; }
        else if (s_i < sn->dali_count + sn->relay_count) {
            seed_gateway_avail(sn->relay_slaves[s_i - sn->dali_count]); s_i++;
        }
        if (s_i >= sn->dali_count + sn->relay_count) { s_ph = PH_CONTROLLER; s_i = 0; }
        return DISC_MORE;
    case PH_CONTROLLER:
        disc_controller((uint8_t)s_i);
        if (++s_i >= CFG_STEPS) { s_ph = PH_PLATE_EVT; s_i = s_j = 0; }
        return DISC_MORE;
    case PH_PLATE_EVT:
        disc_plate_event((uint8_t)s_i, (uint8_t)s_j);
        if (++s_j >= MAX_BTN) { s_j = 0; if (++s_i >= MAX_FIX) { s_ph = PH_PLATE_BIND; s_i = 0; } }
        return DISC_MORE;
    case PH_PLATE_BIND:
        disc_plate_binding((uint8_t)s_i, (uint8_t)s_j);
        if (++s_j >= MAX_BTN) { s_j = 0; if (++s_i >= MAX_FIX) { s_ph = PH_PLATE_STUCK; s_i = 0; } }
        return DISC_MORE;
    case PH_PLATE_STUCK:
        disc_plate_stuck((uint8_t)s_i);
        if (++s_i >= MAX_FIX) { s_ph = PH_RELAY; s_i = 0; }
        return DISC_MORE;
    case PH_RELAY:
        if (sn->relay_count > 0) disc_relay(sn->relay_slaves[0], (uint8_t)s_i);
        if (++s_i >= RELAY_CHANS || sn->relay_count == 0) { s_ph = PH_BALLAST; s_i = s_j = 0; }
        return DISC_MORE;
    case PH_BALLAST:
        if (s_i >= sn->dali_count) { s_ph = PH_BALLAST_CFG; s_i = s_j = s_k = 0; return DISC_MORE; }
        disc_ballast(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= DALI_ADDRS) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_BALLAST_CFG:
        if (s_i >= sn->dali_count) { s_ph = PH_GROUP_L; s_i = s_j = 0; return DISC_MORE; }
        disc_ballast_cfg(sn->dali_slaves[s_i], (uint8_t)s_j, (uint8_t)s_k);
        if (++s_k >= 6) { s_k = 0; if (++s_j >= DALI_ADDRS) { s_j = 0; s_i++; } }
        return DISC_MORE;
    case PH_GROUP_L:
        if (s_i >= sn->dali_count) { s_ph = PH_GROUP_M; s_i = s_j = 0; return DISC_MORE; }
        disc_group_light(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= DALI_GROUPS) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_GROUP_M:
        if (s_i >= sn->dali_count) { s_ph = PH_PROV; s_i = s_j = 0; return DISC_MORE; }
        disc_group_members(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= DALI_GROUPS) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_PROV:
        if (s_i >= sn->dali_count) { s_ph = PH_GW_DIAG; s_i = s_j = 0; return DISC_MORE; }
        disc_prov(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= 10) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_GW_DIAG:
        if (s_i >= sn->dali_count) { s_ph = PH_DONE; return DISC_MORE; }
        disc_gw_diag(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= 4) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_DONE:
    default:
        return DISC_DONE;
    }
}

/* Inbound MQTT commands are pushed to core 0 from ha_on_message, which runs in
 * the MQTT process pump (hot path) — so a full-queue drop must be COUNTED, never
 * logged (USB-CDC back-pressure rule). A climbing count means HA commands are
 * being dropped. The HTTP write path already returns 503 on the same condition. */
static uint32_t s_cmd_drops = 0;
uint32_t ha_bridge_cmd_drop_count(void) { return s_cmd_drops; }
static inline bool ic_push_cmd(const ic_cmd_t *c) {
    bool ok = intercore_push_cmd(c);
    if (!ok) s_cmd_drops++;
    return ok;
}

/* Reboot button on the controller device. Published once at discovery-complete
 * (re-run on every reconnect, like subscribe_all) rather than through the phase
 * machine — it's a single static entity with no per-slave iteration. */
static void disc_system_reboot(void) {
    const char *P = mqtt_app_prefix();
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_reboot\",\"name\":\"Reboot\","
        "\"command_topic\":\"%s/system/reboot\",\"payload_press\":\"PRESS\","
        "\"availability_topic\":\"%s/availability\",\"entity_category\":\"config\","
        "\"device_class\":\"restart\",\"icon\":\"mdi:restart\"",
        P, P);
    pcat(n,
        ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
        "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
        "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
    pub_cfg("button", "lighting_reboot");
}

/* Firmware 'update' entity on the controller device — mirrors the brewery
 * fleet's HA update entity, backed by fwupdate.c's manifest checker. Single
 * static entity, published once at discovery-complete (like disc_system_
 * reboot) rather than through the phase machine. HA's mqtt update platform
 * parses a JSON object on state_topic directly (installed_version/
 * latest_version/release_url/release_summary — the documented "JSON state
 * message" form); value_template here is a pass-through, present so the
 * field names are explicit rather than implicit. */
static void disc_fw_update(void) {
    const char *P = mqtt_app_prefix();
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_fw_update\",\"name\":\"Firmware\","
        "\"state_topic\":\"%s/update/state\",\"command_topic\":\"%s/update/install\","
        "\"payload_install\":\"INSTALL\",\"device_class\":\"firmware\","
        "\"entity_category\":\"config\",\"value_template\":\"{{ value_json | tojson }}\"",
        P, P);
    pcat(n,
        ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
        "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
        "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
    pub_cfg("update", "lighting_fw_update");
}

/* "Check for Updates" button on the controller device — forces an immediate
 * manifest re-check (fwupdate_announce) rather than waiting out the periodic
 * FWCHECK_INTERVAL_S. Mirrors disc_system_reboot; published at discovery-complete. */
static void disc_fw_check(void) {
    const char *P = mqtt_app_prefix();
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_fw_check\",\"name\":\"Check for Updates\","
        "\"command_topic\":\"%s/update/check\",\"payload_press\":\"CHECK\","
        "\"availability_topic\":\"%s/availability\",\"entity_category\":\"config\","
        "\"icon\":\"mdi:cloud-refresh\"",
        P, P);
    pcat(n,
        ",\"device\":{\"identifiers\":[\"lighting_controller\"],"
        "\"name\":\"Switch Gateway Controller\",\"manufacturer\":\"" MFR_MECH "\","
        "\"model\":\"" CTRL_MODEL "\",\"sw_version\":\"" FW_VERSION "\"}}");
    pub_cfg("button", "lighting_fw_check");
}

static void subscribe_all(void) {
    const char *P = mqtt_app_prefix();
    static const char *const filt[] = {
        /* COMMANDS ONLY. Every filter here is something asking us to DO
         * something; none is a topic we ourselves publish. The controller is the
         * source of bus state, not a sink for it — it learns levels and config by
         * reading the gear, never by reading its own retained messages back.
         *
         * "dali/+/+/config" used to be here, so the broker replayed our retained
         * per-ballast config at boot and core 0 adopted it (IC_CMD_SEED_DALI_CFG).
         * That made stale, possibly-wrong values — a swapped or externally
         * reconfigured ballast — reappear as fact. The boot config sweep armed at
         * scan-complete reads the real values from the gear instead. */
        "relay/+/set", "dali/+/+/config/+/set", "dali/+/+/set",
        "group/+/+/members/set", "group/+/+/set",
        "provision/+/start/all", "provision/+/start/unaddressed",
        "provision/+/start/address", "provision/+/start/target", "provision/+/abort",
        "mapping/+/+/set", "mapping/+/+/delete", "binding/+/+/set",
        "config/set", "config/+/set",
        "system/reboot", "update/install", "update/check",
    };
    char f[64];
    for (unsigned i = 0; i < sizeof filt / sizeof filt[0]; i++) {
        snprintf(f, sizeof f, "%s/%s", P, filt[i]);
        /* Runs once at discovery-complete (not the per-tick pump), so logging is
         * allowed. A dropped SUBSCRIBE silently kills a whole command class. */
        if (!mqtt_app_subscribe(f))
            LOG_WARN("mqtt", "subscribe failed: %s", f);
    }
    /* HA birth/will (not under our prefix): lets us re-assert on an HA restart. */
    if (!mqtt_app_subscribe("homeassistant/status"))
        LOG_WARN("mqtt", "subscribe failed: homeassistant/status");
}

/* ── availability sweep (scan_complete -> 64 per-ballast topics) ──────────── */
static bool     s_sweep_pending;
static uint8_t  s_sweep_slave;
static uint64_t s_sweep_mask;
static uint8_t  s_sweep_addr;

static void sweep_step(void) {
    const char *P = mqtt_app_prefix();
    for (int n = 0; n < SWEEP_BATCH && s_sweep_addr < DALI_ADDRS; n++) {
        if (mqtt_app_tx_free() < TX_GATE) return;
        bool present = s_sweep_mask & ((uint64_t)1 << s_sweep_addr);
        snprintf(s_top, sizeof s_top, "%s/dali/%u/%u/availability", P, s_sweep_slave, s_sweep_addr);
        pub_str(s_top, present ? "online" : "offline", true);
        s_sweep_addr++;
    }
    if (s_sweep_addr >= DALI_ADDRS) s_sweep_pending = false;
}

/* ── event -> MQTT state translation ─────────────────────────────────────── */
static const char *const PROV_STATE_STR[] = {
    "idle", "clearing", "scanning", "terminating", "initialising", "randomising",
    "searching", "assigning", "verifying", "withdrawing", "complete", "error"
};
static const char *prov_state_str(uint8_t s) {
    return s < sizeof PROV_STATE_STR / sizeof PROV_STATE_STR[0] ? PROV_STATE_STR[s] : "unknown";
}

/* Decode the gateway's RSTCTRL.RSTFR byte (FC04 0x0310 reg3) to a short string.
 * A cold boot sets several flags at once, so report the most diagnostic one. */
static const char *reset_reason_str(uint8_t rr) {
    if (rr & (1u << 3)) return "watchdog";   /* WDRF — flags a wedge/crash */
    if (rr & (1u << 4)) return "software";    /* SWRF — gw_reboot / MBREG_SYSTEM_RESET */
    if (rr & (1u << 2)) return "external";    /* EXTRF — reset pin */
    /* PORF before BORF: a real power-up sets both (BOD trips as the rail rises),
     * so PORF means a full power cycle; BORF *without* PORF is a true brownout dip. */
    if (rr & (1u << 0)) return "power-on";    /* PORF */
    if (rr & (1u << 1)) return "brownout";    /* BORF (dip, no full power loss) */
    if (rr & (1u << 5)) return "updi";        /* UPDIRF — programmer */
    return "unknown";
}

static void json_escape(const char *in, char *out, size_t cap) {
    size_t o = 0;
    for (; *in && o + 2 < cap; in++) {
        unsigned char c = (unsigned char)*in;
        if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
        else if (c >= 0x20) out[o++] = c;
        else out[o++] = ' ';
    }
    out[o] = '\0';
}

/* Last-published snapshot, so ha_bridge_tick only republishes update/state
 * when fwupdate's view actually changed (a fresh manifest check landed) —
 * keeps this off the byte-path without needing a queue. Seeded false so the
 * very first fw_state_tick() (right after disc_fw_update at discovery-
 * complete) always publishes once, even before any manifest check lands. */
static bool s_fw_pub_valid;
static char s_fw_pub_latest[48];
static char s_fw_pub_release[128];
static char s_fw_pub_notes[256];

static void pub_fw_state(void) {
    const char *P = mqtt_app_prefix();
    char rel[256], notes[512];
    json_escape(fwupdate_release_url(), rel, sizeof rel);
    json_escape(fwupdate_notes(), notes, sizeof notes);
    snprintf(s_pl, PL_CAP,
        "{\"installed_version\":\"%s\",\"latest_version\":\"%s\","
        "\"release_url\":\"%s\",\"release_summary\":\"%s\"}",
        fwupdate_installed(), fwupdate_latest(), rel, notes);
    snprintf(s_top, sizeof s_top, "%s/update/state", P);
    pub_str(s_top, s_pl, true);

    strncpy(s_fw_pub_latest, fwupdate_latest(), sizeof s_fw_pub_latest - 1);
    s_fw_pub_latest[sizeof s_fw_pub_latest - 1] = '\0';
    strncpy(s_fw_pub_release, fwupdate_release_url(), sizeof s_fw_pub_release - 1);
    s_fw_pub_release[sizeof s_fw_pub_release - 1] = '\0';
    strncpy(s_fw_pub_notes, fwupdate_notes(), sizeof s_fw_pub_notes - 1);
    s_fw_pub_notes[sizeof s_fw_pub_notes - 1] = '\0';
    s_fw_pub_valid = true;
}

/* Called every connected ha_bridge_tick (outside discovery/sweep): cheap
 * string compares against the last-published snapshot, so a normal tick with
 * no fresh manifest data costs three strcmp()s and no publish. */
static void fw_state_tick(void) {
    if (mqtt_app_tx_free() < PUMP_GATE) return;
    if (s_fw_pub_valid &&
        !strcmp(s_fw_pub_latest, fwupdate_latest()) &&
        !strcmp(s_fw_pub_release, fwupdate_release_url()) &&
        !strcmp(s_fw_pub_notes, fwupdate_notes()))
        return;
    pub_fw_state();
}

static void members_csv(uint64_t mask, char *out, size_t cap) {
    int n = 0; bool first = true;
    for (int a = 0; a < 64 && n < (int)cap - 4; a++)
        if (mask & ((uint64_t)1 << a)) { n += snprintf(out + n, cap - n, "%s%d", first ? "" : ",", a); first = false; }
    out[n] = '\0';
}

/* ── Tombstones (task 03, Phase C) ────────────────────────────────────────────
 * Retire a removed device by publishing empty retained payloads to its discovery
 * and data topics: HA drops the entity and the broker keeps no stale retained
 * state. Operator-triggered only (CLI `tombstone …` -> IC_EVT_TOMBSTONE) — never
 * automatic, since a transient DALI dropout is availability (offline), not
 * removal. Each device's topic set is small and bounded, so one publish burst
 * stays within the socket TX budget. A whole-gateway retirement clears only the
 * gateway-scoped topics here; its ballasts/groups are retired individually
 * (`tombstone dali <slave> <addr>` / `tombstone group <slave> <g>`). */
static void tomb(const char *topic) { mqtt_app_publish(topic, "", 0, true); }

static void tombstone_ballast(uint8_t slave, uint8_t addr) {
    const char *P = mqtt_app_prefix();
    static const char *const F[] = { "min","max","power_on","sys_fail","fade_rate","fade_time" };
    char tp[96];
    snprintf(tp, sizeof tp, "homeassistant/light/lighting_dali_%u_%u/config", slave, addr); tomb(tp);
    for (unsigned i = 0; i < 6; i++) {
        snprintf(tp, sizeof tp, "homeassistant/number/lighting_dali_%u_%u_%s/config", slave, addr, F[i]);
        tomb(tp);
    }
    snprintf(tp, sizeof tp, "%s/dali/%u/%u/state", P, slave, addr);        tomb(tp);
    snprintf(tp, sizeof tp, "%s/dali/%u/%u/config", P, slave, addr);       tomb(tp);
    snprintf(tp, sizeof tp, "%s/dali/%u/%u/availability", P, slave, addr); tomb(tp);
    LOG_INFO("mqtt", "tombstoned ballast %u.%u", slave, addr);
}

static void tombstone_group(uint8_t slave, uint8_t grp) {
    const char *P = mqtt_app_prefix();
    char tp[96];
    snprintf(tp, sizeof tp, "homeassistant/light/lighting_group_%u_%u/config", slave, grp);        tomb(tp);
    snprintf(tp, sizeof tp, "homeassistant/text/lighting_group_%u_%u_members/config", slave, grp); tomb(tp);
    snprintf(tp, sizeof tp, "%s/group/%u/%u/state", P, slave, grp);         tomb(tp);
    snprintf(tp, sizeof tp, "%s/group/%u/%u/members/state", P, slave, grp); tomb(tp);
    LOG_INFO("mqtt", "tombstoned group %u.%u", slave, grp);
}

/* Clears the whole gateway HA device `lighting_dali_gw_<slave>` — its diagnostic
 * + provisioning discovery configs (disc_gw_diag / disc_prov) and the gateway-
 * scoped data topics — so no ghost device is left after retirement. Its ballasts
 * and groups are separate HA devices, retired individually. ~15 empty retained
 * publishes; best-effort QoS0 into the ~2 KB socket (drained during a manual
 * decommission) and idempotent, so a re-run completes it if a publish was ever
 * dropped under concurrent TX. Discovery configs go first (they remove the HA
 * device) so a dropped tail leaves only a harmless stale data topic. */
static void tombstone_gateway(uint8_t slave) {
    const char *P = mqtt_app_prefix();
    char tp[96];
    /* Diagnostic sensors (disc_gw_diag) — device lighting_dali_gw_<slave>. */
    static const char *const gwdiag[] = { "uptime", "reboots", "reset" };
    for (unsigned i = 0; i < 3; i++) {
        snprintf(tp, sizeof tp, "homeassistant/sensor/lighting_gwdiag_%u_%s/config", slave, gwdiag[i]);
        tomb(tp);
    }
    /* Provisioning entities (disc_prov) — same device. */
    static const char *const prov_sensor[] = { "state", "msg", "count" };
    for (unsigned i = 0; i < 3; i++) {
        snprintf(tp, sizeof tp, "homeassistant/sensor/lighting_prov_%u_%s/config", slave, prov_sensor[i]);
        tomb(tp);
    }
    static const char *const prov_btn[] = { "all", "unaddressed", "address", "abort" };
    for (unsigned i = 0; i < 4; i++) {
        snprintf(tp, sizeof tp, "homeassistant/button/lighting_prov_%u_%s/config", slave, prov_btn[i]);
        tomb(tp);
    }
    snprintf(tp, sizeof tp, "homeassistant/number/lighting_prov_%u_target/config", slave); tomb(tp);
    /* Gateway-scoped retained data topics. */
    snprintf(tp, sizeof tp, "%s/gateway/%u/availability", P, slave);   tomb(tp);
    snprintf(tp, sizeof tp, "%s/gateway/%u/diag", P, slave);           tomb(tp);
    snprintf(tp, sizeof tp, "%s/provision/%u/status", P, slave);       tomb(tp);
    snprintf(tp, sizeof tp, "%s/provision/%u/target_state", P, slave); tomb(tp);
    LOG_INFO("mqtt", "tombstoned gateway %u device (diag+prov entities; ballasts/groups retire individually)", slave);
}

void ha_bridge_publish_event(const ic_event_t *e) {
    const char *P = mqtt_app_prefix();
    char top[80], pl[320];
    switch (e->type) {
    case IC_EVT_RELAY:
        snprintf(top, sizeof top, "%s/relay/%u/state", P, e->relay.ch);
        snprintf(pl, sizeof pl, "{\"state\":\"%s\"}", e->relay.on ? "ON" : "OFF");
        mqtt_app_publish(top, pl, strlen(pl), true);
        break;
    case IC_EVT_DALI:
        snprintf(top, sizeof top, "%s/dali/%u/%u/state", P, e->dali.slave, e->dali.addr);
        if (e->dali.arc == 0) snprintf(pl, sizeof pl, "{\"state\":\"OFF\"}");
        else snprintf(pl, sizeof pl,
            "{\"state\":\"ON\",\"color_mode\":\"brightness\",\"brightness\":%u}", e->dali.arc);
        mqtt_app_publish(top, pl, strlen(pl), true);
        snprintf(top, sizeof top, "%s/dali/%u/%u/config", P, e->dali.slave, e->dali.addr);
        snprintf(pl, sizeof pl,
            "{\"min\":%u,\"max\":%u,\"power_on\":%u,\"sys_fail\":%u,\"fade_rate\":%u,\"fade_time\":%u}",
            e->dali.min, e->dali.max, e->dali.power_on, e->dali.sys_fail, e->dali.fade_rate, e->dali.fade_time);
        mqtt_app_publish(top, pl, strlen(pl), true);
        break;
    case IC_EVT_GROUP: {
        snprintf(top, sizeof top, "%s/group/%u/%u/state", P, e->group.slave, e->group.group);
        if (!e->group.on) snprintf(pl, sizeof pl, "{\"state\":\"OFF\"}");
        else snprintf(pl, sizeof pl,
            "{\"state\":\"ON\",\"color_mode\":\"brightness\",\"brightness\":%u}", e->group.bri);
        mqtt_app_publish(top, pl, strlen(pl), true);
        char csv[208];
        members_csv(e->group.members, csv, sizeof csv);
        snprintf(top, sizeof top, "%s/group/%u/%u/members/state", P, e->group.slave, e->group.group);
        mqtt_app_publish(top, csv, strlen(csv), true);
        break;
    }
    case IC_EVT_GATEWAY:
        snprintf(top, sizeof top, "%s/gateway/%u/availability", P, e->gateway.slave);
        pub_str(top, e->gateway.online ? "online" : "offline", true);
        break;
    case IC_EVT_PRESENCE:
        for (int a = 0; a < 64; a++) {
            if (e->presence.appeared & ((uint64_t)1 << a)) {
                snprintf(top, sizeof top, "%s/dali/%u/%u/availability", P, e->presence.slave, a);
                pub_str(top, "online", true);
            }
            if (e->presence.disappeared & ((uint64_t)1 << a)) {
                snprintf(top, sizeof top, "%s/dali/%u/%u/availability", P, e->presence.slave, a);
                pub_str(top, "offline", true);
            }
        }
        break;
    case IC_EVT_BUTTON: {
        static const char *const act[] = { "press", "release", "click", "long_press",
                                           "long_press_repeat", "stuck", "unstuck" };
        snprintf(top, sizeof top, "%s/button/%u/%u", P, e->button.fixture, e->button.button);
        snprintf(pl, sizeof pl, "{\"event_type\":\"%s\"}",
                 act[e->button.action <= 6 ? e->button.action : 0]);
        mqtt_app_publish(top, pl, strlen(pl), false);   /* momentary, not retained */
        /* stuck/unstuck also drive the fixture's retained Button Stuck
         * diagnostic (ON while any button on the plate is stuck) */
        if (e->button.action == IC_BTN_STUCK || e->button.action == IC_BTN_UNSTUCK) {
            uint8_t f = e->button.fixture;
            if (f < MAX_FIX && e->button.button < 8) {
                if (e->button.action == IC_BTN_STUCK)
                    s_stuck_mask[f] |=  (uint8_t)(1u << e->button.button);
                else
                    s_stuck_mask[f] &= (uint8_t)~(1u << e->button.button);
                snprintf(top, sizeof top, "%s/plate/%u/stuck", P, f);
                pub_str(top, s_stuck_mask[f] ? "ON" : "OFF", true);
            }
        }
        break;
    }
    case IC_EVT_MAPPING: {
        char arr[256]; int n = snprintf(arr, sizeof arr, "[");
        for (int i = 0; i < e->mapping.count; i++) {
            const mapping_target_t *t = &e->mapping.targets[i];
            if (TARGET_IS_GROUP(*t))
                n += snprintf(arr + n, sizeof arr - n, "%s{\"slave\":%u,\"group\":%u}", i ? "," : "", t->slave, TARGET_GROUP_NUM(*t));
            else
                n += snprintf(arr + n, sizeof arr - n, "%s{\"slave\":%u,\"addr\":%u}", i ? "," : "", t->slave, t->addr);
        }
        snprintf(arr + n, sizeof arr - n, "]");
        snprintf(top, sizeof top, "%s/mapping/%u/%u", P, e->mapping.fixture, e->mapping.button);
        snprintf(pl, sizeof pl, "{\"fixture\":%u,\"button\":%u,\"targets\":%s}", e->mapping.fixture, e->mapping.button, arr);
        mqtt_app_publish(top, pl, strlen(pl), true);
        char bs[160]; int m = 0; bs[0] = '\0';
        for (int i = 0; i < e->mapping.count; i++) {
            const mapping_target_t *t = &e->mapping.targets[i];
            if (TARGET_IS_GROUP(*t)) m += snprintf(bs + m, sizeof bs - m, "%s%u:g%u", i ? "," : "", t->slave, TARGET_GROUP_NUM(*t));
            else m += snprintf(bs + m, sizeof bs - m, "%s%u:%u", i ? "," : "", t->slave, t->addr);
        }
        snprintf(top, sizeof top, "%s/binding/%u/%u/state", P, e->mapping.fixture, e->mapping.button);
        mqtt_app_publish(top, bs, strlen(bs), true);
        break;
    }
    case IC_EVT_PROVISION: {
        char msg[2 * 80];
        json_escape(e->provision.msg, msg, sizeof msg);
        snprintf(top, sizeof top, "%s/provision/%u/status", P, e->provision.slave);
        snprintf(pl, sizeof pl, "{\"state\":\"%s\",\"msg\":\"%s\",\"devices_found\":%u}",
                 prov_state_str(e->provision.state), msg, e->provision.count);
        mqtt_app_publish(top, pl, strlen(pl), true);
        break;
    }
    case IC_EVT_CFG:
        snprintf(top, sizeof top, "%s/config/state", P);
        snprintf(pl, sizeof pl,
            "{\"long_press_ms\":%u,\"dim_repeat_ms\":%u,\"reversal_window_ms\":%u,\"baud\":%lu,"
            "\"relay_byteswap\":%u,\"gw_diag_interval_ms\":%lu,"
            "\"dali_sweep_interval_ms\":%lu,"
            "\"fw_version\":\"" FW_VERSION "\",\"max_fixtures\":%u,\"max_buttons\":%u,\"max_targets\":%u}",
            e->cfg.long_press_ms, e->cfg.dim_repeat_ms, e->cfg.reversal_window_ms,
            (unsigned long)e->cfg.baud, e->cfg.relay_byteswap ? 1u : 0u,
            (unsigned long)e->cfg.gw_diag_interval_ms,
            (unsigned long)e->cfg.dali_sweep_interval_ms,
            MAX_FIX, MAX_BTN, MAX_TARGETS_PER_BTN);
        mqtt_app_publish(top, pl, strlen(pl), true);
        break;
    case IC_EVT_GW_DIAG:
        snprintf(top, sizeof top, "%s/gateway/%u/diag", P, e->gw_diag.slave);
        snprintf(pl, sizeof pl,
            "{\"uptime\":%lu,\"reboots\":%u,\"reset_reason\":\"%s\"}",
            (unsigned long)e->gw_diag.uptime_s, e->gw_diag.boot_count,
            reset_reason_str(e->gw_diag.reset_reason));
        mqtt_app_publish(top, pl, strlen(pl), true);
        break;
    case IC_EVT_SCAN_COMPLETE:   /* handled by the sweep cursor, not here */
        break;
    case IC_EVT_FAULT:           /* mqtt_bridge.py logs faults only — no topic */
        break;
    case IC_EVT_PING_REQ:        /* routed to ping_submit in the drain, never published */
        break;
    case IC_EVT_OTA_ACK:         /* routed to ota_on_event in the drain, never published */
    case IC_EVT_OTA_STATUS:      /* routed to ota_on_event in the drain, never published */
        break;
    case IC_EVT_TOMBSTONE:
        switch (e->tombstone.kind) {
        case IC_TOMB_BALLAST: tombstone_ballast(e->tombstone.slave, e->tombstone.addr); break;
        case IC_TOMB_GROUP:   tombstone_group(e->tombstone.slave, e->tombstone.addr);   break;
        case IC_TOMB_GATEWAY: tombstone_gateway(e->tombstone.slave);                    break;
        default: break;
        }
        break;
    }
}

/* ── inbound MQTT -> ic_cmd_t ────────────────────────────────────────────── */
static uint8_t s_prov_target[MAX_DALI_SLAVES];

static int split(char *s, char d, char **out, int max) {
    int n = 0;
    out[n++] = s;
    for (; *s && n < max; s++) if (*s == d) { *s = '\0'; out[n++] = s + 1; }
    return n;
}
static bool jtok_eq(const char *js, const jsmntok_t *t, const char *s) {
    int len = t->end - t->start;
    return t->type == JSMN_STRING && (int)strlen(s) == len && strncmp(js + t->start, s, len) == 0;
}
static int jfind(const char *js, const jsmntok_t *t, int n, const char *key) {
    for (int i = 1; i < n; i++) if (jtok_eq(js, &t[i], key)) return i + 1;
    return -1;
}
static long jint(const char *js, const jsmntok_t *t) {
    char b[16]; int len = t->end - t->start; if (len >= (int)sizeof b) len = (int)sizeof b - 1;
    memcpy(b, js + t->start, len); b[len] = '\0'; return strtol(b, NULL, 10);
}

/* HA JSON light cmd -> set_dali-style fields (shared by dali + group). */
static void parse_light(const char *pay, uint16_t *fields, ic_tri_t *on, uint8_t *bri) {
    jsmn_parser jp; jsmntok_t tk[24]; jsmn_init(&jp);
    int nt = jsmn_parse(&jp, pay, strlen(pay), tk, 24);
    if (nt < 1) return;
    int bi = jfind(pay, tk, nt, "brightness");
    if (bi >= 0) { *bri = (uint8_t)jint(pay, &tk[bi]); *fields |= IC_DALI_F_BRI; return; }
    int si = jfind(pay, tk, nt, "state");
    if (si >= 0) { *on = jtok_eq(pay, &tk[si], "ON") ? IC_TRI_ON : IC_TRI_OFF; *fields |= IC_DALI_F_ON; }
}

static uint8_t parse_binding(const char *str, mapping_target_t *out, uint8_t max) {
    const char *s = str; uint8_t c = 0;
    while (*s && c < max) {
        while (*s == ' ' || *s == ',') s++;
        if (!*s) break;
        char *e;
        long sl = strtol(s, &e, 10);
        if (e == s) break;
        s = e;
        if (*s != ':') break;
        s++;
        bool grp = (*s == 'g' || *s == 'G');
        if (grp) s++;
        long v = strtol(s, &e, 10);
        if (e == s) break;
        s = e;
        out[c].slave = (uint8_t)sl;
        out[c].addr = grp ? TARGET_MAKE_GROUP(v) : (uint8_t)(v & 0x3F);
        c++;
    }
    return c;
}

static uint8_t parse_targets_json(const char *pay, mapping_target_t *out, uint8_t max) {
    jsmn_parser jp; jsmntok_t tk[64]; jsmn_init(&jp);
    int nt = jsmn_parse(&jp, pay, strlen(pay), tk, 64);
    if (nt < 1 || tk[0].type != JSMN_ARRAY) return 0;
    uint8_t c = 0; int i = 1;
    while (i < nt && c < max) {
        if (tk[i].type != JSMN_OBJECT) break;
        int sz = tk[i].size; i++;
        int sl = 0, addr = 0, grp = -1;
        for (int f = 0; f < sz && i + 1 < nt; f++) {
            if (jtok_eq(pay, &tk[i], "slave"))      sl = (int)jint(pay, &tk[i + 1]);
            else if (jtok_eq(pay, &tk[i], "addr"))  addr = (int)jint(pay, &tk[i + 1]);
            else if (jtok_eq(pay, &tk[i], "group")) grp = (int)jint(pay, &tk[i + 1]);
            i += 2;
        }
        out[c].slave = (uint8_t)sl;
        out[c].addr = grp >= 0 ? TARGET_MAKE_GROUP(grp) : (uint8_t)(addr & 0x3F);
        c++;
    }
    return c;
}

static void ha_on_message(const char *topic, uint16_t tlen,
                          const uint8_t *payload, uint16_t plen) {
    char t[160]; uint16_t tl = tlen < sizeof t - 1 ? tlen : sizeof t - 1;
    memcpy(t, topic, tl); t[tl] = '\0';
    char pay[512]; uint16_t pn = plen < sizeof pay - 1 ? plen : sizeof pay - 1;
    memcpy(pay, payload, pn); pay[pn] = '\0';

    /* Home Assistant birth message: HA publishes "online" to homeassistant/status
     * when it (re)starts. Re-run discovery + a full state resync so HA is brought
     * fully up to date immediately, instead of relying solely on the broker's
     * retained copies. (Discovery stays retained, as the fallback.) Same kick as
     * ha_on_connected; ha_bridge_tick runs the burst. */
    if (!strcmp(t, "homeassistant/status")) {
        if (!strcmp(pay, "online")) {
            LOG_INFO("mqtt", "HA online — re-publishing discovery + resync");
            s_disc_active   = true;
            s_ph            = PH_GW_AVAIL;
            s_i = s_j = s_k = 0;
            s_sweep_pending = false;
        }
        return;
    }

    const char *P = mqtt_app_prefix(); size_t pl0 = strlen(P);
    if (strncmp(t, P, pl0) != 0 || t[pl0] != '/') return;
    char *seg[8];
    int ns = split(t + pl0 + 1, '/', seg, 8);
    if (ns < 1) return;

    ic_cmd_t c; memset(&c, 0, sizeof c);

    /* Out-of-band reboot. Payload-gated ("PRESS"/"REBOOT") so a stray retained or
     * empty message can't reset the controller. This is the remote recovery for a
     * wedged USB stack — core 0 reboots via the watchdog, no USB involved. */
    if (!strcmp(seg[0], "system") && ns >= 2 && !strcmp(seg[1], "reboot")) {
        if (!strcmp(pay, "PRESS") || !strcmp(pay, "REBOOT")) {
            /* Clean shutdown: publish a retained offline now so HA greys us out
             * immediately, rather than waiting for the broker to notice the LWT
             * after the reboot drops the TCP link (task 03, Phase C). */
            char av[48];
            snprintf(av, sizeof av, "%s/availability", mqtt_app_prefix());
            pub_str(av, "offline", true);
            c.type = IC_CMD_REBOOT;
            ic_push_cmd(&c);
        }
        return;
    }

    /* HA update entity's Install button. Payload-gated ("INSTALL") like the
     * reboot command above. fwupdate_install() is a no-op until a manifest
     * has been fetched successfully (fwupdate_latest_url() empty) and
     * ota_trigger() itself no-ops while an OTA is already RUNNING, so a
     * stray/duplicate press here is harmless. This never touches intercore —
     * fwupdate/ota run entirely on core 1. */
    if (!strcmp(seg[0], "update") && ns >= 2 && !strcmp(seg[1], "install")) {
        if (!strcmp(pay, "INSTALL")) fwupdate_install();
        return;
    }
    /* "Check for Updates" button — force an immediate manifest re-check.
     * Payload-gated ("CHECK") like the others; fwupdate_announce() just sets a
     * flag consumed by the next fwupdate_poll(), so a stray press is harmless. */
    if (!strcmp(seg[0], "update") && ns >= 2 && !strcmp(seg[1], "check")) {
        if (!strcmp(pay, "CHECK")) fwupdate_announce();
        return;
    }
    if (!strcmp(seg[0], "relay") && ns >= 3 && !strcmp(seg[2], "set")) {
        c.type = IC_CMD_SET_RELAY;
        c.set_relay.ch = (uint8_t)atoi(seg[1]);
        c.set_relay.on = strstr(pay, "OFF") ? IC_TRI_OFF : IC_TRI_ON;
        ic_push_cmd(&c);
        return;
    }
    if (!strcmp(seg[0], "dali") && ns >= 4) {
        uint8_t s = (uint8_t)atoi(seg[1]), a = (uint8_t)atoi(seg[2]);
        if (!strcmp(seg[3], "set")) {
            c.type = IC_CMD_SET_DALI; c.set_dali.slave = s; c.set_dali.addr = a;
            parse_light(pay, &c.set_dali.fields, &c.set_dali.on, &c.set_dali.bri);
            if (c.set_dali.fields) ic_push_cmd(&c);
        } else if (ns >= 6 && !strcmp(seg[3], "config") && !strcmp(seg[5], "set")) {
            c.type = IC_CMD_SET_DALI; c.set_dali.slave = s; c.set_dali.addr = a;
            uint8_t v = (uint8_t)atoi(pay); const char *f = seg[4];
            if      (!strcmp(f, "min"))       { c.set_dali.fields = IC_DALI_F_MIN;       c.set_dali.min = v; }
            else if (!strcmp(f, "max"))       { c.set_dali.fields = IC_DALI_F_MAX;       c.set_dali.max = v; }
            else if (!strcmp(f, "power_on"))  { c.set_dali.fields = IC_DALI_F_POWER_ON;  c.set_dali.power_on = v; }
            else if (!strcmp(f, "sys_fail"))  { c.set_dali.fields = IC_DALI_F_SYS_FAIL;  c.set_dali.sys_fail = v; }
            else if (!strcmp(f, "fade_rate")) { c.set_dali.fields = IC_DALI_F_FADE_RATE; c.set_dali.fade_rate = v; }
            else if (!strcmp(f, "fade_time")) { c.set_dali.fields = IC_DALI_F_FADE_TIME; c.set_dali.fade_time = v; }
            if (c.set_dali.fields) ic_push_cmd(&c);
        }
        return;
    }
    if (!strcmp(seg[0], "group") && ns >= 4) {
        uint8_t s = (uint8_t)atoi(seg[1]), g = (uint8_t)atoi(seg[2]);
        if (!strcmp(seg[3], "set")) {
            c.type = IC_CMD_SET_GROUP; c.set_group.slave = s; c.set_group.group = g;
            uint16_t f = 0; ic_tri_t on = IC_TRI_OFF; uint8_t bri = 0;
            parse_light(pay, &f, &on, &bri);
            if (f & IC_DALI_F_BRI) { c.set_group.fields |= IC_GRP_F_BRI; c.set_group.bri = bri; }
            if (f & IC_DALI_F_ON)  { c.set_group.fields |= IC_GRP_F_ON;  c.set_group.on = on; }
            if (c.set_group.fields) ic_push_cmd(&c);
        } else if (ns >= 5 && !strcmp(seg[3], "members") && !strcmp(seg[4], "set")) {
            c.type = IC_CMD_SET_GROUP; c.set_group.slave = s; c.set_group.group = g;
            c.set_group.fields = IC_GRP_F_MEMBERS;
            uint64_t m = 0; const char *q = pay;
            while (*q) {
                while (*q == ' ' || *q == ',') q++;
                if (!*q) break;
                char *e; long v = strtol(q, &e, 10); if (e == q) break; q = e;
                if (v >= 0 && v < 64) m |= (uint64_t)1 << v;
            }
            c.set_group.members = m;
            ic_push_cmd(&c);
        }
        return;
    }
    if (!strcmp(seg[0], "provision") && ns >= 3) {
        uint8_t s = (uint8_t)atoi(seg[1]); int idx = slave_idx(s);
        if (!strcmp(seg[2], "abort")) {
            c.type = IC_CMD_PROVISION_ABORT; c.provision_abort.slave = s; ic_push_cmd(&c);
        } else if (!strcmp(seg[2], "start") && ns >= 4) {
            const char *m = seg[3];
            if (!strcmp(m, "target")) {
                if (idx >= 0) {
                    s_prov_target[idx] = (uint8_t)atoi(pay);
                    char ts[8]; snprintf(ts, sizeof ts, "%u", s_prov_target[idx]);
                    char top[64]; snprintf(top, sizeof top, "%s/provision/%u/target_state", P, s);
                    mqtt_app_publish(top, ts, strlen(ts), true);
                }
                return;
            }
            c.type = IC_CMD_PROVISION_START; c.provision_start.slave = s;
            if      (!strcmp(m, "all"))         c.provision_start.mode = IC_PROV_ALL;
            else if (!strcmp(m, "unaddressed")) c.provision_start.mode = IC_PROV_UNADDRESSED;
            else if (!strcmp(m, "address")) {
                c.provision_start.mode = IC_PROV_ADDRESS;
                c.provision_start.target = idx >= 0 ? s_prov_target[idx] : 0;
            } else return;
            ic_push_cmd(&c);
        }
        return;
    }
    if (!strcmp(seg[0], "mapping") && ns >= 4) {
        uint8_t f = (uint8_t)atoi(seg[1]), b = (uint8_t)atoi(seg[2]);
        if (!strcmp(seg[3], "set")) {
            c.type = IC_CMD_SET_MAPPING; c.set_mapping.fixture = f; c.set_mapping.button = b;
            c.set_mapping.count = parse_targets_json(pay, c.set_mapping.targets, MAX_TARGETS_PER_BTN);
            ic_push_cmd(&c);
        } else if (!strcmp(seg[3], "delete")) {
            c.type = IC_CMD_DEL_MAPPING; c.del_mapping.fixture = f; c.del_mapping.button = b;
            ic_push_cmd(&c);
        }
        return;
    }
    if (!strcmp(seg[0], "binding") && ns >= 4 && !strcmp(seg[3], "set")) {
        c.type = IC_CMD_SET_MAPPING;
        c.set_mapping.fixture = (uint8_t)atoi(seg[1]);
        c.set_mapping.button = (uint8_t)atoi(seg[2]);
        c.set_mapping.count = parse_binding(pay, c.set_mapping.targets, MAX_TARGETS_PER_BTN);
        ic_push_cmd(&c);
        return;
    }
    if (!strcmp(seg[0], "config")) {
        c.type = IC_CMD_SET_CFG;
        if (ns >= 2 && !strcmp(seg[1], "set")) {
            jsmn_parser jp; jsmntok_t tk[24]; jsmn_init(&jp);
            int nt = jsmn_parse(&jp, pay, strlen(pay), tk, 24);
            int i;
            if ((i = jfind(pay, tk, nt, "long_press_ms")) >= 0)      { c.set_cfg.fields |= IC_CFG_F_LONG_PRESS; c.set_cfg.long_press_ms = (uint16_t)jint(pay, &tk[i]); }
            if ((i = jfind(pay, tk, nt, "dim_repeat_ms")) >= 0)      { c.set_cfg.fields |= IC_CFG_F_DIM_REPEAT; c.set_cfg.dim_repeat_ms = (uint16_t)jint(pay, &tk[i]); }
            if ((i = jfind(pay, tk, nt, "reversal_window_ms")) >= 0) { c.set_cfg.fields |= IC_CFG_F_REVERSAL;   c.set_cfg.reversal_window_ms = (uint16_t)jint(pay, &tk[i]); }
            if ((i = jfind(pay, tk, nt, "relay_byteswap")) >= 0)     { c.set_cfg.fields |= IC_CFG_F_RELAY_BYTESWAP; c.set_cfg.relay_byteswap = jint(pay, &tk[i]) ? 1u : 0u; }
            if ((i = jfind(pay, tk, nt, "gw_diag_interval_ms")) >= 0) { c.set_cfg.fields |= IC_CFG_F_GW_DIAG; c.set_cfg.gw_diag_interval_ms = (uint32_t)jint(pay, &tk[i]); }
            if ((i = jfind(pay, tk, nt, "dali_sweep_interval_ms")) >= 0) { c.set_cfg.fields |= IC_CFG_F_DALI_SWEEP; c.set_cfg.dali_sweep_interval_ms = (uint32_t)jint(pay, &tk[i]); }
        } else if (ns >= 3 && !strcmp(seg[2], "set")) {
            uint16_t v = (uint16_t)atoi(pay); const char *f = seg[1];
            if      (!strcmp(f, "long_press_ms"))      { c.set_cfg.fields = IC_CFG_F_LONG_PRESS; c.set_cfg.long_press_ms = v; }
            else if (!strcmp(f, "dim_repeat_ms"))      { c.set_cfg.fields = IC_CFG_F_DIM_REPEAT; c.set_cfg.dim_repeat_ms = v; }
            else if (!strcmp(f, "reversal_window_ms")) { c.set_cfg.fields = IC_CFG_F_REVERSAL;   c.set_cfg.reversal_window_ms = v; }
            else if (!strcmp(f, "relay_byteswap"))     { c.set_cfg.fields = IC_CFG_F_RELAY_BYTESWAP; c.set_cfg.relay_byteswap = v ? 1u : 0u; }
            else if (!strcmp(f, "gw_diag_interval_ms")) { c.set_cfg.fields = IC_CFG_F_GW_DIAG; c.set_cfg.gw_diag_interval_ms = (uint32_t)strtoul(pay, NULL, 10); }
            else if (!strcmp(f, "dali_sweep_interval_ms")) { c.set_cfg.fields = IC_CFG_F_DALI_SWEEP; c.set_cfg.dali_sweep_interval_ms = (uint32_t)strtoul(pay, NULL, 10); }
        }
        if (c.set_cfg.fields) ic_push_cmd(&c);
        return;
    }
}

/* ── hooks + tick ────────────────────────────────────────────────────────── */
static void ha_on_connected(void) {
    LOG_INFO("mqtt", "session up — (re)publishing discovery");
    s_disc_active = true;
    s_ph = PH_GW_AVAIL;
    s_i = s_j = s_k = 0;
    s_sweep_pending = false;
    fwupdate_announce();   /* force a manifest check + state publish this connect */
}

void ha_bridge_init(void) {
    mqtt_app_set_connected_cb(ha_on_connected);
    mqtt_app_set_message_cb(ha_on_message);
}

void ha_bridge_tick(void) {
    /* Disconnected: MQTT gets nothing, but still fold every event into the web
     * read model — the config UI must stay live exactly when the broker is down
     * (its whole point is commissioning before Home Assistant exists). The queue
     * is drained fully each tick; web_state_on_event is cheap (no I/O). */
    if (!mqtt_app_connected()) {
        s_disc_active = false;
        s_sweep_pending = false;
        ic_event_t ev;
        while (intercore_pop_event(&ev)) {
            if (ev.type == IC_EVT_PING_REQ) { ping_submit(ev.ping_req.ip, ev.ping_req.count); continue; }
            if (ev.type == IC_EVT_OTA_ACK || ev.type == IC_EVT_OTA_STATUS) { ota_on_event(&ev); continue; }
            web_state_on_event(&ev);
        }
        return;
    }

    /* Finish the discovery burst before publishing any state. */
    if (s_disc_active) {
        for (int n = 0; n < DISC_BATCH; n++) {
            disc_res_t r = disc_step();
            if (r == DISC_STALL) break;
            if (r == DISC_DONE) {
                s_disc_active = false;
                LOG_INFO("mqtt", "discovery complete — subscribe + resync");
                subscribe_all();
                disc_system_reboot();
                disc_fw_update();
                disc_fw_check();
                pub_fw_state();
                ic_cmd_t rc; memset(&rc, 0, sizeof rc); rc.type = IC_CMD_RESYNC;
                ic_push_cmd(&rc);
                break;
            }
        }
        return;
    }

    /* Finish an availability sweep before draining more events. */
    if (s_sweep_pending) { sweep_step(); return; }

    /* Cheap dirty-check against fwupdate's last-known manifest data; only
     * actually publishes when something changed (or on the very first call,
     * as a fallback if discovery-complete's seed publish above was skipped
     * by a stall). Off the hot byte-path — normal MQTT publish cadence. */
    fw_state_tick();

    /* Pump state events to MQTT, TX-flow-controlled and bounded per tick. */
    ic_event_t ev;
    for (int budget = 24; budget > 0; budget--) {
        if (mqtt_app_tx_free() < PUMP_GATE) break;
        if (!intercore_pop_event(&ev)) break;
        if (ev.type == IC_EVT_PING_REQ) { ping_submit(ev.ping_req.ip, ev.ping_req.count); continue; }
        if (ev.type == IC_EVT_OTA_ACK || ev.type == IC_EVT_OTA_STATUS) { ota_on_event(&ev); continue; }
        web_state_on_event(&ev);   /* keep the web read model in lock-step */
        if (ev.type == IC_EVT_SCAN_COMPLETE) {
            s_sweep_pending = true;
            s_sweep_slave   = ev.scan_complete.slave;
            s_sweep_mask    = ev.scan_complete.present_mask;
            s_sweep_addr    = 0;
            break;   /* run the sweep on following ticks before more events */
        }
        ha_bridge_publish_event(&ev);
    }
}
