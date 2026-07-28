#pragma once
/*
 * dali_commission.h — controller-side DALI commissioning (task 05).
 *
 * The IEC 62386-102 binary-search address assignment, driven from the
 * controller as ordinary queued DALI frames. It used to live in the gateway
 * behind MBREG_COMMISSION (write a mode, poll a status word); the gateway strip
 * removes that register and its state machine, so the sequence moves here.
 *
 * Ported from the gateway's proven implementation (TinyModbus dali_sync.c
 * OP_COMMISSION) rather than rewritten — including the two non-obvious
 * behaviours that were found on hardware there:
 *
 *  - COMPARE is anyone-pulls-low, so ANY decoded response means "match".
 *    Only a clean no-answer means "none". A garbled/collided backward frame is
 *    a match, not an error.
 *  - After a bit-0 COMPARE says "no", the gear's SEARCHADDR registers hold the
 *    bit-0-cleared value while our local copy has bit 0 restored. The search
 *    address must be RE-SENT before PROGRAM, or PROGRAM silently addresses
 *    nothing and VERIFY fails.
 *
 * No tight inter-step timing is required — every step is one queued frame, and
 * the gear's INITIALISE window is 15 minutes against a run of tens of seconds.
 */

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    DCOM_MODE_ALL = 1,      /* wipe and renumber from 0 */
    DCOM_MODE_NEW = 2,      /* only gear with no short address */
    DCOM_MODE_ONE = 3,      /* only the gear at one collided short address */
} dcom_mode_t;

typedef enum {
    DCOM_IDLE = 0,
    DCOM_RUNNING,
    DCOM_COMPLETE,
    DCOM_ERR_ABORTED,
    DCOM_ERR_NO_BALLASTS,
    DCOM_ERR_VERIFY,
    DCOM_ERR_ADDR_EXHAUST,
} dcom_state_t;

/* Start a run on `slave_id`. `target` is the short address to re-provision,
 * used only by DCOM_MODE_ONE. Returns false if a run is already active. */
bool dcom_start(uint8_t slave_id, dcom_mode_t mode, uint8_t target);

/* Request an abort. Takes effect at the next step boundary, and always runs
 * TERMINATE so the bus is not left in commissioning mode. */
void dcom_abort(void);

/* Per-loop driver: advances one step per completed frame. Non-blocking. */
void dcom_tick(void);

dcom_state_t dcom_state(void);
uint8_t      dcom_count(void);        /* ballasts addressed so far this run */
uint8_t      dcom_slave(void);
bool         dcom_busy(void);
const char  *dcom_phase_name(void);   /* current phase, for status reporting */
