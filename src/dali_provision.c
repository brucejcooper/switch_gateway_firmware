/*
 * dali_provision.c — operator-facing driver for a DALI commissioning run.
 *
 * The commissioning SEQUENCE runs on the controller
 * (dali_commission.c) as ordinary queued DALI frames, not in the
 * gateway behind MBREG_COMMISSION — write a mode word, poll a status word until
 * terminal — and that register goes away with the gateway strip.
 *
 * What is left in this file is the operator surface, unchanged: the run-state
 * enum, the human-readable message, and g_prov_status, which the CLI
 * (`provision`), the MQTT provision/<slave>/status topics and the HA
 * buttons/sensors all consume. This now mirrors the local state machine's
 * progress instead of a remote status word.
 *
 * All three modes are supported, including PROV_ADDRESS (re-provision the gear
 * at one collided short address), which maps to DCOM_MODE_ONE.
 */

#include "dali_provision.h"
#include "dali_commission.h"
#include "dirty_poll.h"
#include "serial_protocol.h"
#include "log.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>



/* ── Internal state ──────────────────────────────────────────────────────
 * Deliberately thin: the run itself lives in dali_commission.c. This only
 * tracks that a run is ours and whether we have published progress yet. */
prov_status_t g_prov_status = { .run_state = PROV_STATE_IDLE };

static struct {
    bool        active;
    uint8_t     slave_id;
    bool        reported;   /* progress published at least once this run */
    prov_mode_t mode;       /* what was asked for — decides whether "found none"
                             * is a failure or simply nothing to do */
} s;

/* ── Helpers ────────────────────────────────────────────────────────────── */

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static void set_status(prov_run_state_t state, const char *msg) {
    g_prov_status.run_state = state;
    strncpy(g_prov_status.msg, msg, PROV_STATUS_MSG_MAX - 1);
    g_prov_status.msg[PROV_STATUS_MSG_MAX - 1] = '\0';
    LOG_INFO("prov", "state=%d msg=\"%s\"", (int)state, msg);
    serial_provision_status_changed();
}


/* ── Modbus callbacks ──────────────────────────────────────────────────── */





/* ── Public API ────────────────────────────────────────────────────────── */

void dali_provision_init(void) {
    memset(&s, 0, sizeof(s));
}

bool dali_provision_busy(void) { return s.active; }

bool dali_provision_busy_slave(uint8_t slave_id) {
    return s.active && s.slave_id == slave_id;
}

bool dali_provision_start(uint8_t slave_id, prov_mode_t mode,
                           uint8_t target_addr) {
    if (s.active || dcom_busy()) return false;
    if (mode == PROV_ADDRESS && target_addr >= 64u) return false;

    dcom_mode_t dm = (mode == PROV_ADDRESS)      ? DCOM_MODE_ONE
                   : (mode == PROV_UNADDRESSED)  ? DCOM_MODE_NEW
                                                 : DCOM_MODE_ALL;

    memset(&s, 0, sizeof(s));
    s.active   = true;
    s.slave_id = slave_id;
    s.mode     = mode;

    g_prov_status.slave_id      = slave_id;
    g_prov_status.devices_found = 0;

    if (!dcom_start(slave_id, dm, target_addr)) {
        s.active = false;
        return false;
    }
    if (mode == PROV_ADDRESS) {
        char msg[PROV_STATUS_MSG_MAX];
        snprintf(msg, sizeof msg, "Re-provisioning address %u...", target_addr);
        set_status(PROV_STATE_INITIALISING, msg);
    } else {
        set_status(PROV_STATE_INITIALISING,
                   mode == PROV_UNADDRESSED ?
                   "Starting incremental commissioning..." :
                   "Starting full commissioning...");
    }
    return true;
}

void dali_provision_abort(void) {
    if (!s.active) return;
    LOG_INFO("prov", "abort requested");
    set_status(PROV_STATE_TERMINATING, "Aborting commissioning...");
    dcom_abort();
}

/* ── Tick ──────────────────────────────────────────────────────────────────
 * The commissioning sequence itself runs on the controller
 * (dali_commission.c) as queued DALI frames. Rather than "write a mode to
 * MBREG_COMMISSION, then poll its status word every 500 ms" is now just
 * mirroring the local state machine's progress into the operator-facing status
 * that the CLI, MQTT topics and HA entities already consume — that surface is
 * unchanged. */
void dali_provision_tick(void) {
    if (!s.active) return;

    dcom_tick();

    char msg[PROV_STATUS_MSG_MAX];
    uint8_t n = dcom_count();

    if (dcom_busy()) {
        if (!s.reported || n != g_prov_status.devices_found) {
            s.reported = true;
            g_prov_status.devices_found = n;
            snprintf(msg, sizeof msg, "Commissioning (%s)... %u device%s addressed",
                     dcom_phase_name(), n, n == 1 ? "" : "s");
            set_status(PROV_STATE_SEARCHING, msg);
        }
        return;
    }

    /* Terminal. Release the busy gate BEFORE the blocking re-scan so anything
     * asking "are you provisioning?" gets the right answer. */
    g_prov_status.devices_found = n;
    s.active = false;

    switch (dcom_state()) {
    case DCOM_COMPLETE:
        /* Re-interrogate, but do NOT block on it. dcom already armed a full scan
         * when the run ended; the main loop drives it and state converges over
         * the next few seconds. Blocking here would stall buttons and MQTT for
         * the whole scan — seconds on a real bus — for no benefit, since nothing
         * in this function needs the result. */
        snprintf(msg, sizeof msg, "Complete: %u device%s commissioned — re-syncing",
                 n, n == 1 ? "" : "s");
        set_status(PROV_STATE_COMPLETE, msg);
        break;
    case DCOM_ERR_ABORTED:
        set_status(PROV_STATE_ERROR, "Aborted");
        break;
    case DCOM_ERR_NO_BALLASTS:
        /* Finding nothing means different things per mode.
         *
         * PROV_UNADDRESSED issues INITIALISE with INIT_DATA_NO_SHORT, which
         * includes ONLY gear that has no short address. On a fully-commissioned
         * bus that correctly matches nothing, so zero found is the expected
         * result — "there is nothing new to add", not a failure. Reporting it as
         * an error meant incremental provisioning ALWAYS looked broken on a
         * healthy bus, which is exactly what it looked like from the UI.
         *
         * PROV_ALL should have matched every ballast, so zero is a real fault
         * (bad wiring, no bus power, gear not responding to the search).
         * PROV_ADDRESS targeting an address with nothing at it is also worth
         * flagging — that is the check that proves a re-provision cannot touch
         * gear it was not aimed at. */
        if (s.mode == PROV_UNADDRESSED) {
            set_status(PROV_STATE_COMPLETE,
                       "No unaddressed ballasts — nothing to commission");
        } else {
            set_status(PROV_STATE_ERROR, "Error: no ballasts found");
        }
        break;
    case DCOM_ERR_VERIFY:
        set_status(PROV_STATE_ERROR, "Error: repeated verify failures");
        break;
    case DCOM_ERR_ADDR_EXHAUST:
        set_status(PROV_STATE_ERROR, "Error: short-address pool exhausted");
        break;
    case DCOM_ERR_TIMEOUT:
        set_status(PROV_STATE_ERROR, "Error: a commissioning step got no result");
        break;
    default:
        set_status(PROV_STATE_ERROR, "Error: commissioning ended unexpectedly");
        break;
    }
}
