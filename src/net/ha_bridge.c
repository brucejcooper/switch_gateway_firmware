/*
 * ha_bridge.c — Home Assistant discovery + availability + translation (core 1).
 *
 * Discovery is published INCREMENTALLY: one entity per disc_step(), a bounded
 * batch per tick, gated on free socket-TX space. Blasting ~600 retained configs
 * synchronously would overrun the 2 KB socket TX buffer and freeze the core-1
 * heartbeat. Payloads are built one at a time into a single reused buffer.
 *
 * Topic/payload shapes mirror bridge/mqtt_bridge.py (the behavioural spec).
 */

#include "ha_bridge.h"
#include "mqtt_app.h"
#include "intercore.h"
#include "log.h"

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#define MFR_MECH   "Mechination"
#define MFR_WAVE   "Waveshare"

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
#define DISC_BATCH     6        /* entities published per tick */
#define TX_HEADROOM 1100        /* require this much free TX before a step */

static char s_pl[PL_CAP];       /* reused payload buffer */
static char s_top[112];         /* reused topic buffer */

/* ── publish helpers ─────────────────────────────────────────────────────── */
static void pub_cfg(const char *component, const char *objid) {
    snprintf(s_top, sizeof s_top, "homeassistant/%s/%s/config", component, objid);
    mqtt_app_publish(s_top, s_pl, strlen(s_pl), true);
}
static void pub_str(const char *topic, const char *payload, bool retain) {
    mqtt_app_publish(topic, payload, strlen(payload), retain);
}

/* Append a HA device block. via NULL => omit via_device. */
static int dev(int n, const char *ids, const char *name,
               const char *mfr, const char *model, const char *via) {
    n += snprintf(s_pl + n, PL_CAP - n,
                  ",\"device\":{\"identifiers\":[\"%s\"],\"name\":\"%s\","
                  "\"manufacturer\":\"%s\",\"model\":\"%s\"", ids, name, mfr, model);
    if (via) n += snprintf(s_pl + n, PL_CAP - n, ",\"via_device\":\"%s\"", via);
    n += snprintf(s_pl + n, PL_CAP - n, "}}");
    return n;
}

/* ── per-entity discovery builders ───────────────────────────────────────── */
static void disc_plate_event(uint8_t fix, uint8_t btn) {
    const char *P = mqtt_app_prefix();
    char ids[40], name[24];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_switch_%02u_btn_%u\",\"name\":\"Button %u\","
        "\"state_topic\":\"%s/button/%u/%u\","
        "\"event_types\":[\"press\",\"release\",\"long_press_start\",\"long_press_repeat\"],"
        "\"value_template\":\"{{ value_json.event_type }}\","
        "\"availability_topic\":\"%s/availability\","
        "\"icon\":\"mdi:gesture-tap-button\"",
        fix, btn, btn + 1, P, fix, btn, P);
    snprintf(ids, sizeof ids, "lighting_switch_plate_%02u", fix);
    snprintf(name, sizeof name, "Fixture %02u", fix + 1);
    dev(n, ids, name, MFR_MECH, "Switch Gateway Button Matrix", "lighting_controller");
    char objid[40];
    snprintf(objid, sizeof objid, "lighting_switch_%02u_btn_%u", fix, btn);
    pub_cfg("event", objid);
}

static void disc_plate_binding(uint8_t fix, uint8_t btn) {
    const char *P = mqtt_app_prefix();
    char ids[40], name[24];
    int n = snprintf(s_pl, PL_CAP,
        "{\"unique_id\":\"lighting_binding_%02u_btn_%u\",\"name\":\"Button %u Binding\","
        "\"command_topic\":\"%s/binding/%u/%u/set\","
        "\"state_topic\":\"%s/binding/%u/%u/state\","
        "\"pattern\":\"%s\",\"availability_topic\":\"%s/availability\","
        "\"entity_category\":\"config\",\"icon\":\"mdi:link-variant\"",
        fix, btn, btn + 1, P, fix, btn, P, fix, btn, BINDING_PATTERN, P);
    snprintf(ids, sizeof ids, "lighting_switch_plate_%02u", fix);
    snprintf(name, sizeof name, "Fixture %02u", fix + 1);
    dev(n, ids, name, MFR_MECH, "Switch Gateway Button Matrix", "lighting_controller");
    char objid[40];
    snprintf(objid, sizeof objid, "lighting_binding_%02u_btn_%u", fix, btn);
    pub_cfg("text", objid);
    snprintf(s_top, sizeof s_top, "%s/binding/%u/%u/state", P, fix, btn);
    pub_str(s_top, "", true);
}

static void disc_relay(uint8_t slave, uint8_t ch) {
    const char *P = mqtt_app_prefix();
    char ids[32], name[24];
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
    char objid[32];
    snprintf(objid, sizeof objid, "lighting_relay_%u", ch);
    pub_cfg("switch", objid);
}

static void disc_ballast(uint8_t slave, uint8_t addr) {
    const char *P = mqtt_app_prefix();
    char ids[40], name[24], gw[32];
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
    char objid[40];
    snprintf(objid, sizeof objid, "lighting_dali_%u_%u", slave, addr);
    pub_cfg("light", objid);
    snprintf(s_top, sizeof s_top, "%s/dali/%u/%u/availability", P, slave, addr);
    pub_str(s_top, "offline", true);     /* scan_complete brings present ones online */
}

static void disc_group_light(uint8_t slave, uint8_t grp) {
    const char *P = mqtt_app_prefix();
    char ids[40], name[28], gw[32];
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
    char objid[40];
    snprintf(objid, sizeof objid, "lighting_group_%u_%u", slave, grp);
    pub_cfg("light", objid);
    snprintf(s_top, sizeof s_top, "%s/group/%u/%u/state", P, slave, grp);
    pub_str(s_top, "{\"state\":\"OFF\"}", true);
}

static void disc_group_members(uint8_t slave, uint8_t grp) {
    const char *P = mqtt_app_prefix();
    char ids[40], name[28], gw[32];
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
    char objid[48];
    snprintf(objid, sizeof objid, "lighting_group_%u_%u_members", slave, grp);
    pub_cfg("text", objid);
    snprintf(s_top, sizeof s_top, "%s/group/%u/%u/members/state", P, slave, grp);
    pub_str(s_top, "", true);
}

/* One provisioning entity (k = 0..9) for a gateway slave. */
static void disc_prov(uint8_t slave, uint8_t k) {
    const char *P = mqtt_app_prefix();
    char gwids[32], gwname[24], objid[48], status[64];
    snprintf(gwids, sizeof gwids, "lighting_dali_gw_%u", slave);
    snprintf(gwname, sizeof gwname, "DALI Gateway %u", slave);
    snprintf(status, sizeof status, "%s/provision/%u/status", P, slave);

    /* common availability tail + device, appended after the per-entity head. */
    #define PROV_TAIL(cat) do { \
        int _n = (int)strlen(s_pl); \
        _n += snprintf(s_pl+_n, PL_CAP-_n, \
            ",\"availability\":[{\"topic\":\"%s/availability\"}," \
            "{\"topic\":\"%s/gateway/%u/availability\"}],\"availability_mode\":\"all\"," \
            "\"entity_category\":\"" cat "\"", P, P, slave); \
        dev(_n, gwids, gwname, MFR_MECH, "TinyModbus ATtiny1604 DALI Gateway", NULL); \
    } while (0)

    switch (k) {
    case 0:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_state\","
            "\"name\":\"Commission State\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.state }}\","
            "\"icon\":\"mdi:lightning-bolt-circle\"", slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_state", slave);
        pub_cfg("sensor", objid);
        break;
    case 1:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_msg\","
            "\"name\":\"Commission Status\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.msg }}\","
            "\"icon\":\"mdi:text-box-outline\"", slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_msg", slave);
        pub_cfg("sensor", objid);
        break;
    case 2:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_count\","
            "\"name\":\"Devices Found\",\"state_topic\":\"%s\","
            "\"value_template\":\"{{ value_json.devices_found }}\","
            "\"icon\":\"mdi:counter\"", slave, status);
        PROV_TAIL("diagnostic");
        snprintf(objid, sizeof objid, "lighting_prov_%u_count", slave);
        pub_cfg("sensor", objid);
        break;
    case 3: case 4: case 5: case 6: {
        static const char *suf[]  = { "all", "unaddressed", "address", "abort" };
        static const char *nm[]   = { "Commission All", "Commission New Devices",
                                      "Commission Address", "Abort Commission" };
        static const char *icon[] = { "mdi:lightbulb-group", "mdi:lightbulb-plus",
                                      "mdi:lightbulb-on", "mdi:stop-circle" };
        static const char *cmd[]  = { "start/all", "start/unaddressed",
                                      "start/address", "abort" };
        int b = k - 3;
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_%s\",\"name\":\"%s\","
            "\"command_topic\":\"%s/provision/%u/%s\",\"payload_press\":\"PRESS\","
            "\"icon\":\"%s\"", slave, suf[b], nm[b], P, slave, cmd[b], icon[b]);
        PROV_TAIL("config");
        snprintf(objid, sizeof objid, "lighting_prov_%u_%s", slave, suf[b]);
        pub_cfg("button", objid);
        break;
    }
    case 7:
        snprintf(s_pl, PL_CAP, "{\"unique_id\":\"lighting_prov_%u_target\","
            "\"name\":\"Target Address\",\"command_topic\":\"%s/provision/%u/start/target\","
            "\"state_topic\":\"%s/provision/%u/target_state\","
            "\"min\":0,\"max\":63,\"step\":1,\"mode\":\"box\",\"icon\":\"mdi:numeric\"",
            slave, P, slave, P, slave);
        PROV_TAIL("config");
        snprintf(objid, sizeof objid, "lighting_prov_%u_target", slave);
        pub_cfg("number", objid);
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

/* ── incremental discovery cursor ────────────────────────────────────────── */
typedef enum {
    PH_GW_AVAIL, PH_PLATE_EVT, PH_PLATE_BIND, PH_RELAY,
    PH_BALLAST, PH_GROUP_L, PH_GROUP_M, PH_PROV, PH_DONE
} disc_phase_t;

static bool         s_disc_active = false;
static disc_phase_t s_ph;
static uint16_t     s_i, s_j;

typedef enum { DISC_MORE, DISC_STALL, DISC_DONE } disc_res_t;

static void seed_gateway_avail(uint8_t slave) {
    snprintf(s_top, sizeof s_top, "%s/gateway/%u/availability", mqtt_app_prefix(), slave);
    pub_str(s_top, "online", true);
}

/* Publish one unit and advance. Returns DISC_STALL (retry) if TX is full. */
static disc_res_t disc_step(void) {
    const ic_snapshot_t *sn = intercore_snapshot();
    if (mqtt_app_tx_free() < TX_HEADROOM) return DISC_STALL;

    switch (s_ph) {
    case PH_GW_AVAIL: {
        /* i over dali slaves then relay slaves. */
        if (s_i < sn->dali_count) { seed_gateway_avail(sn->dali_slaves[s_i]); s_i++; }
        else if (s_i < sn->dali_count + sn->relay_count) {
            seed_gateway_avail(sn->relay_slaves[s_i - sn->dali_count]); s_i++;
        }
        if (s_i >= sn->dali_count + sn->relay_count) { s_ph = PH_PLATE_EVT; s_i = s_j = 0; }
        return DISC_MORE;
    }
    case PH_PLATE_EVT:
        disc_plate_event((uint8_t)s_i, (uint8_t)s_j);
        if (++s_j >= MAX_BTN) { s_j = 0; if (++s_i >= MAX_FIX) { s_ph = PH_PLATE_BIND; s_i = 0; } }
        return DISC_MORE;
    case PH_PLATE_BIND:
        disc_plate_binding((uint8_t)s_i, (uint8_t)s_j);
        if (++s_j >= MAX_BTN) { s_j = 0; if (++s_i >= MAX_FIX) { s_ph = PH_RELAY; s_i = 0; } }
        return DISC_MORE;
    case PH_RELAY:
        if (sn->relay_count > 0) disc_relay(sn->relay_slaves[0], (uint8_t)s_i);
        if (++s_i >= RELAY_CHANS || sn->relay_count == 0) { s_ph = PH_BALLAST; s_i = s_j = 0; }
        return DISC_MORE;
    case PH_BALLAST:
        if (s_i >= sn->dali_count) { s_ph = PH_GROUP_L; s_i = s_j = 0; return DISC_MORE; }
        disc_ballast(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= DALI_ADDRS) { s_j = 0; s_i++; }
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
        if (s_i >= sn->dali_count) { s_ph = PH_DONE; return DISC_MORE; }
        disc_prov(sn->dali_slaves[s_i], (uint8_t)s_j);
        if (++s_j >= 10) { s_j = 0; s_i++; }
        return DISC_MORE;
    case PH_DONE:
    default:
        return DISC_DONE;
    }
}

/* ── hooks ───────────────────────────────────────────────────────────────── */
static void ha_on_connected(void) {
    LOG_INFO("mqtt", "session up — starting discovery");
    s_disc_active = true;
    s_ph = PH_GW_AVAIL;
    s_i = s_j = 0;
}

static void ha_on_message(const char *topic, uint16_t tlen,
                          const uint8_t *payload, uint16_t plen) {
    /* Command translation is added in the next commit. */
    (void)topic; (void)tlen; (void)payload; (void)plen;
}

void ha_bridge_init(void) {
    mqtt_app_set_connected_cb(ha_on_connected);
    mqtt_app_set_message_cb(ha_on_message);
}

void ha_bridge_tick(void) {
    if (!s_disc_active) return;
    for (int n = 0; n < DISC_BATCH; n++) {
        disc_res_t r = disc_step();
        if (r == DISC_STALL) break;
        if (r == DISC_DONE) {
            s_disc_active = false;
            LOG_INFO("mqtt", "discovery complete");
            break;
        }
    }
}

/* Filled in by the translation commit. */
void ha_bridge_publish_event(const ic_event_t *e) { (void)e; }
