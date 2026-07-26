/*
 * dali_provision.c — driver for TinyModbus's native commissioning channel.
 *
 * TinyModbus implements IEC 62386-102 control-gear commissioning natively
 * via MBREG_COMMISSION (FC03/FC06 0x1014):
 *
 *   FC06 write 0x0001        start full commissioning (wipe + reassign 0..N-1)
 *   FC06 write 0x0002        start incremental commissioning (only fresh gear)
 *   FC06 write 0x0000        abort (latches; firmware terminates at next boundary)
 *
 *   FC03 read returns the live status word with read-clears on terminal:
 *     0x0000      idle
 *     0x01nn      running, nn = ballasts addressed so far this run
 *     0x02nn      complete, nn = total programmed (read-clears)
 *     0x03nn      error / aborted (read-clears). Subtype in low byte:
 *                 0x01 aborted by master, 0x02 no ballasts found,
 *                 0x03 >3 verify failures, 0x04 short-address pool exhausted,
 *                 0x05 bus shorted >200 TE.
 *
 * This file is a thin polling state machine: arm the channel, then poll
 * status every ~500 ms until terminal, then trigger a local
 * dirty_poll_full_scan to repopulate the controller's state cache from the
 * freshly-addressed bus.
 *
 * PROV_ADDRESS mode (re-init one specific short address) has no equivalent
 * in the TinyModbus commissioning channel and is rejected.
 */

#include "dali_provision.h"
#include "modbus.h"
#include "dirty_poll.h"
#include "serial_protocol.h"
#include "log.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

#define MBREG_COMMISSION  0x1F04u   /* relocated in task 04 (was 0x1014) */

#define POLL_INTERVAL_MS  500u
#define POST_SCAN_WAIT_MS 200u

/* ── Internal sub-state ─────────────────────────────────────────────────── */
typedef enum {
    SS_IDLE,
    SS_ARMING,         /* FC06 start write enqueued; awaiting ACK         */
    SS_POLLING,        /* periodic FC03 read of 0x1014 until terminal      */
    SS_ABORTING,       /* FC06 abort write enqueued / polling for terminal */
    SS_POST_SCAN_WAIT, /* settle delay before dirty_poll_full_scan         */
    SS_DONE,
} sub_state_t;

prov_status_t g_prov_status = { .run_state = PROV_STATE_IDLE };

static struct {
    bool        active;
    uint8_t     slave_id;
    sub_state_t sub;
    uint32_t    next_poll_ms;
    uint32_t    wait_until_ms;
    bool        poll_pending;      /* a FC03 read is in flight              */
    volatile bool     status_ready;
    volatile uint16_t status_val;
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

static const char *err_subtype_msg(uint8_t sub) {
    switch (sub) {
    case 0x01: return "aborted by master";
    case 0x02: return "no ballasts found";
    case 0x03: return "consecutive verify failures";
    case 0x04: return "short-address pool exhausted";
    case 0x05: return "bus stuck";
    default:   return "unknown error";
    }
}

/* ── Modbus callbacks ──────────────────────────────────────────────────── */

static void arm_cb(int res, uint8_t *buf, void *ctx) {
    (void)buf; (void)ctx;
    if (res <= 0) {
        /* DEVICE_BUSY (commissioning already running) or other failure.
         * Either way we are not driving a run — bail. */
        set_status(PROV_STATE_ERROR, "Failed to arm commissioning");
        s.active = false;
        s.sub    = SS_DONE;
        return;
    }
    s.next_poll_ms = now_ms() + POLL_INTERVAL_MS;
    s.sub          = SS_POLLING;
}

static void abort_cb(int res, uint8_t *buf, void *ctx) {
    (void)buf; (void)ctx;
    if (res <= 0) {
        LOG_WARN("prov", "abort write failed (%d) — still polling for terminal", res);
    }
    s.next_poll_ms = now_ms() + POLL_INTERVAL_MS;
    s.sub          = SS_ABORTING;
}

static void poll_cb(int res, uint8_t *buf, void *ctx) {
    (void)ctx;
    s.poll_pending = false;
    /* FC03 1-register frame: addr+FC+bytecount+2data+CRC = 7 bytes */
    if (res < 7) return;
    s.status_val   = ((uint16_t)buf[3] << 8) | buf[4];
    s.status_ready = true;
}

static void start_poll(void) {
    if (s.poll_pending) return;
    s.poll_pending = true;
    s.status_ready = false;
    modbus_get_holding_registers(s.slave_id, MBREG_COMMISSION, 1, poll_cb, NULL);
}

/* ── Public API ────────────────────────────────────────────────────────── */

void dali_provision_init(void) {
    memset(&s, 0, sizeof(s));
    s.sub = SS_IDLE;
}

bool dali_provision_busy(void) { return s.active; }

bool dali_provision_busy_slave(uint8_t slave_id) {
    return s.active && s.slave_id == slave_id;
}

bool dali_provision_start(uint8_t slave_id, prov_mode_t mode,
                           uint8_t target_addr) {
    if (s.active) return false;
    if (mode == PROV_ADDRESS && target_addr >= 64u) return false;

    // MBREG_COMMISSION command word: 0x0001 full, 0x0002 incremental,
    // 0x03YY re-provision the gear at short address YY (fixes a collision).
    uint16_t cmd;
    const char *what;
    if (mode == PROV_ADDRESS)        { cmd = 0x0300u | target_addr; what = "re-provision addr"; }
    else if (mode == PROV_UNADDRESSED) { cmd = 0x0002u;             what = "incremental"; }
    else                             { cmd = 0x0001u;               what = "full"; }

    memset(&s, 0, sizeof(s));
    s.active   = true;
    s.slave_id = slave_id;
    s.sub      = SS_ARMING;

    g_prov_status.slave_id      = slave_id;
    g_prov_status.devices_found = 0;

    LOG_INFO("prov", "start slave=%u mode=%s%s", slave_id, what,
             mode == PROV_ADDRESS ? "" : "");
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

    modbus_set_holding_register(slave_id, MBREG_COMMISSION, cmd, arm_cb, NULL);
    return true;
}

void dali_provision_abort(void) {
    if (!s.active) return;
    if (s.sub == SS_DONE || s.sub == SS_POST_SCAN_WAIT) return;
    LOG_INFO("prov", "abort requested");
    set_status(PROV_STATE_TERMINATING, "Aborting commissioning...");
    modbus_set_holding_register(s.slave_id, MBREG_COMMISSION,
                                0x0000u, abort_cb, NULL);
}

/* ── Tick ──────────────────────────────────────────────────────────────── */

void dali_provision_tick(void) {
    if (!s.active) return;

    char msg[PROV_STATUS_MSG_MAX];

    switch (s.sub) {

    case SS_IDLE:
    case SS_ARMING:
    case SS_DONE:
        /* Waiting for callback or nothing to do */
        break;

    case SS_POLLING:
    case SS_ABORTING:
        if (s.status_ready) {
            uint16_t val = s.status_val;
            uint8_t  hi  = (uint8_t)(val >> 8);
            uint8_t  lo  = (uint8_t)val;
            s.status_ready = false;

            switch (hi) {
            case 0x00:
                /* Idle — saw a terminal-clear during this poll, or the
                 * abort completed. Either way, the run is done. */
                if (s.sub == SS_ABORTING) {
                    set_status(PROV_STATE_ERROR, "Aborted by master");
                } else {
                    set_status(PROV_STATE_ERROR,
                               "Status returned to idle unexpectedly");
                }
                s.active = false;
                s.sub    = SS_DONE;
                break;

            case 0x01:
                g_prov_status.devices_found = lo;
                snprintf(msg, sizeof(msg),
                         s.sub == SS_ABORTING ?
                         "Aborting... %u device%s addressed so far" :
                         "Commissioning... %u device%s addressed",
                         lo, lo == 1 ? "" : "s");
                set_status(PROV_STATE_SEARCHING, msg);
                s.next_poll_ms = now_ms() + POLL_INTERVAL_MS;
                break;

            case 0x02:
                g_prov_status.devices_found = lo;
                snprintf(msg, sizeof(msg),
                         "Commissioning finished: %u device%s — re-syncing...",
                         lo, lo == 1 ? "" : "s");
                set_status(PROV_STATE_SEARCHING, msg);
                s.wait_until_ms = now_ms() + POST_SCAN_WAIT_MS;
                s.sub = SS_POST_SCAN_WAIT;
                break;

            case 0x03:
                snprintf(msg, sizeof(msg), "Error: %s", err_subtype_msg(lo));
                set_status(PROV_STATE_ERROR, msg);
                s.active = false;
                s.sub    = SS_DONE;
                break;
            }
        } else if ((int32_t)(now_ms() - s.next_poll_ms) >= 0) {
            start_poll();
        }
        break;

    case SS_POST_SCAN_WAIT:
        if ((int32_t)(now_ms() - s.wait_until_ms) < 0) break;
        /* Release the lock before the blocking scan so any other
         * subsystem that asks "are you busy?" gets the right answer. */
        s.active = false;
        dirty_poll_full_scan(s.slave_id);
        snprintf(msg, sizeof(msg),
                 "Complete: %u device%s commissioned",
                 g_prov_status.devices_found,
                 g_prov_status.devices_found == 1 ? "" : "s");
        set_status(PROV_STATE_COMPLETE, msg);
        s.sub = SS_DONE;
        break;
    }
}
