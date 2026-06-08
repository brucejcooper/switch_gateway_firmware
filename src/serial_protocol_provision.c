/*
 * serial_protocol_provision.c — provisioning message handling.
 *
 * Adds to serial_protocol:
 *   Inbound:  "provision_start" → dali_provision_start()
 *             "provision_abort" → dali_provision_abort()
 *   Outbound: "provision_status" emitted whenever g_prov_status changes
 *
 * serial_provision_status_changed() is called from dali_provision.c
 * (via set_status()) whenever the run state or message changes.
 * The actual emit happens on the next serial_poll() tick so it is
 * always on the main loop, never from a Modbus callback.
 */

#include "serial_protocol.h"
#include "dali_provision.h"
#include "config.h"
#include "intercore.h"
#include "log.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pico/time.h>

/* ── Minimal JSON helpers (self-contained, no dependency on serial_protocol.c) */
static bool prov_json_str(const char *json, const char *key,
                           char *out, int out_len) {
    char needle[32];
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
static bool prov_json_int(const char *j, const char *k, int *out) {
    char buf[16];
    if (!prov_json_str(j, k, buf, sizeof(buf))) return false;
    *out = atoi(buf);
    return true;
}


/* Flag set by dali_provision.c, cleared after emit */
static volatile bool s_status_dirty = false;

void serial_provision_status_changed(void) {
    s_status_dirty = true;
}

/* Called from serial_poll() each iteration */
void serial_provision_poll(void) {
    if (!s_status_dirty) return;
    s_status_dirty = false;

    const prov_status_t *ps = &g_prov_status;
    uint32_t ts = to_ms_since_boot(get_absolute_time());

    const char *state_str;
    switch (ps->run_state) {
    case PROV_STATE_IDLE:            state_str = "idle";            break;
    case PROV_STATE_CLEARING:        state_str = "clearing";        break;
    case PROV_STATE_SCANNING_USED:   state_str = "scanning";        break;
    case PROV_STATE_TERMINATING:     state_str = "terminating";     break;
    case PROV_STATE_INITIALISING:    state_str = "initialising";    break;
    case PROV_STATE_RANDOMISING:     state_str = "randomising";     break;
    case PROV_STATE_SEARCHING:       state_str = "searching";       break;
    case PROV_STATE_ASSIGNING:       state_str = "assigning";       break;
    case PROV_STATE_VERIFYING:       state_str = "verifying";       break;
    case PROV_STATE_WITHDRAWING:     state_str = "withdrawing";     break;
    case PROV_STATE_COMPLETE:        state_str = "complete";        break;
    case PROV_STATE_ERROR:           state_str = "error";           break;
    default:                         state_str = "unknown";         break;
    }

    printf("{\"t\":\"provision_status\",\"ts\":%lu,"
           "\"d\":{\"slave\":%u},"
           "\"v\":{\"state\":\"%s\",\"msg\":\"%s\","
           "\"devices_found\":%u}}\n",
           (unsigned long)ts,
           ps->slave_id,
           state_str,
           ps->msg,
           ps->devices_found);

    /* Forward to core 1. The raw run_state enum is carried; core 1 maps it to
     * the state + message strings (the free-form msg is not carried). */
    ic_event_t ev = { .type = IC_EVT_PROVISION };
    ev.provision.slave = ps->slave_id;
    ev.provision.state = (uint8_t)ps->run_state;
    ev.provision.count = ps->devices_found;
    intercore_push_event(&ev);
}

/* Called from serial_protocol handle_line() for provision_start messages */
void serial_handle_provision_start(const char *line, unsigned cmd_id) {
    /* Parse: {"t":"provision_start","id":N,"d":{"slave":2,"mode":"all"}}      */
    /* mode values: "all", "unaddressed", "address"                             */
    /* "address" mode also requires "target": N                                 */

    char mode_str[20] = {0};
    int  slave = 0, target = 0;

    /* P1: json helpers — self-contained copy, avoids extern-of-static UB */
    prov_json_int(line, "slave",  &slave);
    prov_json_str(line, "mode",   mode_str, sizeof(mode_str));
    prov_json_int(line, "target", &target);

    /* Validate slave ID */
    bool valid_slave = false;
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        if (g_config.dali_slave_ids[i] == (uint8_t)slave) {
            valid_slave = true;
            break;
        }
    }
    if (!valid_slave) {
        serial_tx_ack(cmd_id, false, "invalid_slave");
        return;
    }

    if (dali_provision_busy()) {
        serial_tx_ack(cmd_id, false, "busy");
        return;
    }

    prov_mode_t mode;
    if (strcmp(mode_str, "all") == 0) {
        mode = PROV_ALL;
    } else if (strcmp(mode_str, "unaddressed") == 0) {
        mode = PROV_UNADDRESSED;
    } else if (strcmp(mode_str, "address") == 0) {
        if (target < 0 || target > 63) {
            serial_tx_ack(cmd_id, false, "invalid_target");
            return;
        }
        mode = PROV_ADDRESS;
    } else {
        serial_tx_ack(cmd_id, false, "invalid_mode");
        return;
    }

    LOG_INFO("prov", "dispatch start slave=%u mode=%s target=%u",
             (unsigned)slave, mode_str, (unsigned)target);
    bool started = dali_provision_start((uint8_t)slave, mode, (uint8_t)target);
    serial_tx_ack(cmd_id, started, started ? NULL : "start_failed");
}

void serial_handle_provision_abort(const char *line, unsigned cmd_id) {
    (void)line;
    dali_provision_abort();
    serial_tx_ack(cmd_id, true, NULL);
}
