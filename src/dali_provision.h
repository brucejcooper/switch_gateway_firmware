#pragma once
#include <stdint.h>
#include <stdbool.h>

/*
 * dali_provision.h — DALI commissioning state machine.
 *
 * Orchestrates the DALI binary-search address assignment protocol via the
 * gateway's commission register (MBREG_COMMISSION 0x1F04): the controller writes
 * a start/abort command and polls it until terminal.
 * Runs cooperatively with the main loop — never blocks.
 *
 * Three modes matching the original Lua implementation:
 *   PROV_ALL         — clear all addresses, reassign from 0
 *   PROV_UNADDRESSED — address only devices with no short address
 *   PROV_ADDRESS     — put one specific short address back into init mode
 *
 * One provisioning run can be active at a time, on one slave bus.
 * While running, the dirty_poll subsystem is suspended for that slave.
 */

/* ── Provision mode ──────────────────────────────────────────────────────── */
typedef enum {
    PROV_ALL,           /* clear + full reprovision               */
    PROV_UNADDRESSED,   /* address only new/unaddressed devices    */
    PROV_ADDRESS,       /* re-initialise one specific short addr   */
} prov_mode_t;

/* ── Run state (for status reporting) ───────────────────────────────────── */
typedef enum {
    PROV_STATE_IDLE,
    PROV_STATE_CLEARING,
    PROV_STATE_SCANNING_USED,
    PROV_STATE_TERMINATING,
    PROV_STATE_INITIALISING,
    PROV_STATE_RANDOMISING,
    PROV_STATE_SEARCHING,
    PROV_STATE_ASSIGNING,
    PROV_STATE_VERIFYING,
    PROV_STATE_WITHDRAWING,
    PROV_STATE_COMPLETE,
    PROV_STATE_ERROR,
} prov_run_state_t;

/* Maximum length of the human-readable status message */
#define PROV_STATUS_MSG_MAX 80

typedef struct {
    prov_run_state_t run_state;
    uint8_t          slave_id;
    uint8_t          devices_found;
    char             msg[PROV_STATUS_MSG_MAX];
} prov_status_t;

/* Global status — readable at any time by serial_protocol */
extern prov_status_t g_prov_status;

/* ── Lifecycle ───────────────────────────────────────────────────────────── */

void dali_provision_init(void);

/*
 * Start a provisioning run.
 * Returns false if a run is already in progress.
 * slave_id: the TinyModbus slave to provision (2 or 3)
 * mode:     which devices to target
 * target_addr: only used for PROV_ADDRESS (0–63)
 */
bool dali_provision_start(uint8_t slave_id, prov_mode_t mode,
                           uint8_t target_addr);

/*
 * Call from main loop every iteration.
 * Advances the state machine by one step if a Modbus response is ready
 * or a timer has elapsed.  Non-blocking.
 */
void dali_provision_tick(void);

/* True while a provisioning run is active */
bool dali_provision_busy(void);

/* True if the slave_id is currently being provisioned */
bool dali_provision_busy_slave(uint8_t slave_id);

/* Abort a running provision (e.g. on serial disconnect) */
void dali_provision_abort(void);
