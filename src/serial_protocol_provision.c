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



/* Flag set by dali_provision.c, cleared after emit */
static volatile bool s_status_dirty = false;

void serial_provision_status_changed(void) {
    s_status_dirty = true;
}

/* Called from serial_poll() each iteration. Forwards a changed provisioning
 * status to core 1 (the network bridge). The raw run_state enum is carried —
 * core 1 maps it to the short state string; the free-form msg goes verbatim. */
void serial_provision_poll(void) {
    if (!s_status_dirty) return;
    s_status_dirty = false;

    const prov_status_t *ps = &g_prov_status;
    ic_event_t ev = { .type = IC_EVT_PROVISION };
    ev.provision.slave = ps->slave_id;
    ev.provision.state = (uint8_t)ps->run_state;
    ev.provision.count = ps->devices_found;
    snprintf(ev.provision.msg, sizeof ev.provision.msg, "%s", ps->msg);
    intercore_push_event(&ev);
}

