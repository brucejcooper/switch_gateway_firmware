/*
 * dali_provision.c — DALI commissioning state machine.
 *
 * All DALI system commands are issued via TinyModbus holding registers:
 *   0x1000 RAW_CMD       — single send, expects backward frame (for queries)
 *   0x1001 RAW_CMD_TWICE — sends twice, no backward frame (for set commands)
 *   0x100B SET_DTR0      — loads DTR0 before a set command
 *
 * Query results are polled from FC04 input register 0x015D:
 *   0xFF00 — pending
 *   0x0100 — NAK (no response)
 *   0x02xx — OK, response byte = xx
 *   0x03xx — bus error
 *
 * The state machine is driven by dali_provision_tick() called from main loop.
 * Each tick either:
 *   a) Issues the next Modbus write (non-blocking, enqueued via modbus.c), or
 *   b) Polls 0x015D for a query result and advances state on receipt, or
 *   c) Waits for a minimum inter-command gap (enforced by timestamp).
 *
 * DALI system command encoding:
 *   RAW_CMD / RAW_CMD_TWICE value = (addr_byte << 8) | cmd_byte
 *   For system commands (no short address), addr_byte is the command's
 *   own high byte (e.g. 0xA1 for Terminate, 0xA9 for Compare).
 *
 * Binary search:
 *   Search range [low, high] (24-bit). We maintain low and high as the
 *   unexamined range. Each iteration:
 *     1. Set Search Address = midpoint
 *     2. Compare — if YES: high = midpoint (something at or below mid)
 *                  if NO:  low  = midpoint + 1 (nothing at or below mid)
 *     3. When low == high: that IS the device's random address.
 */

#include "dali_provision.h"
#include "modbus.h"
#include "config.h"
#include "dirty_poll.h"
#include "serial_protocol.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>

/* ── TinyModbus register addresses ───────────────────────────────────────── */
#define REG_RAW_CMD        0x1000u
#define REG_RAW_CMD_TWICE  0x1001u
#define REG_SET_DTR0       0x100Bu
#define REG_RAW_RESULT     0x015Du   /* FC04 input register */

/* RAW_RESULT values */
#define RAW_RESULT_IDLE    0x0000u
#define RAW_RESULT_PENDING 0xFF00u
#define RAW_RESULT_NAK     0x0100u
#define RAW_RESULT_OK      0x0200u   /* low byte = response */
#define RAW_RESULT_ERROR   0x0300u

/* ── DALI system command bytes ────────────────────────────────────────────── */
/* Each encoded as (high_byte << 8) | low_byte for RAW_CMD / RAW_CMD_TWICE   */
#define DALI_TERMINATE          0xA100u
#define DALI_RANDOMISE          0xA700u
#define DALI_COMPARE            0xA900u
#define DALI_WITHDRAW           0xAB00u
#define DALI_INITIALISE_ALL     0xA500u   /* put ALL devices into init mode */
#define DALI_INITIALISE_NEW     0xA5FFu   /* put UNADDRESSED devices only   */
#define DALI_CLEAR_SHORT_ADDR   0xFF80u   /* broadcast: delete short address */
/* SET_SEARCH_ADDR_{H,M,L}: high byte is cmd, low byte is address fragment  */
#define DALI_SET_SEARCH_H(b)    (0xB100u | ((b) & 0xFF))
#define DALI_SET_SEARCH_M(b)    (0xB300u | ((b) & 0xFF))
#define DALI_SET_SEARCH_L(b)    (0xB500u | ((b) & 0xFF))
/* PROGRAM_SHORT_ADDRESS: sends short_addr in form (addr<<1)|1              */
#define DALI_PROG_SHORT(a)      (0xB700u | (((a) & 0x3F) << 1) | 0x01u)
/* VERIFY_SHORT_ADDRESS: query — expects YES response                        */
#define DALI_VERIFY_SHORT(a)    (0xB900u | (((a) & 0x3F) << 1) | 0x01u)
/* INITIALISE for a specific short address already present                   */
#define DALI_INITIALISE_ADDR(a) (0xA501u | (((a) & 0x3F) << 1))
/* QUERY_CONTROL_GEAR_PRESENT for addr: check if a short addr is occupied   */
#define DALI_QUERY_PRESENT(a)   ((((a) & 0x3F) << 1 | 0x01u) << 8 | 0x91u)

/* Minimum gap between DALI system commands (ms) — 22 Te ≈ 9.2 ms, use 12  */
#define DALI_CMD_GAP_MS  12u
/* Extra delay after RANDOMISE — devices need time to generate random addr   */
#define DALI_RANDOMISE_DELAY_MS 120u

/* ── Internal sub-states ─────────────────────────────────────────────────── */
typedef enum {
    /* Idle */
    SS_IDLE,

    /* Clearing existing addresses */
    SS_CLEAR_DTR0,          /* write SET_DTR0 = 0xFF */
    SS_CLEAR_BROADCAST,     /* write RAW_CMD_TWICE = 0xFF80 */
    SS_CLEAR_DONE,

    /* Scanning which addresses are already used */
    SS_SCAN_QUERY,          /* query CONTROL_GEAR_PRESENT for current addr */
    SS_SCAN_WAIT,           /* wait for 0x015D result */
    SS_SCAN_DONE,

    /* Terminate → Initialise → Randomise */
    SS_TERMINATE,
    SS_INITIALISE,
    SS_RANDOMISE,
    SS_RANDOMISE_WAIT,      /* extra delay for RNG to settle */

    /* Binary search loop */
    SS_SEARCH_SET_H,        /* write Search Address H */
    SS_SEARCH_SET_M,        /* write Search Address M */
    SS_SEARCH_SET_L,        /* write Search Address L */
    SS_SEARCH_COMPARE,      /* write RAW_CMD = Compare */
    SS_SEARCH_COMPARE_WAIT, /* poll 0x015D */

    /* Assign address to found device */
    SS_ASSIGN_PROG,         /* write RAW_CMD_TWICE = PROGRAM_SHORT_ADDRESS */
    SS_ASSIGN_VERIFY,       /* write RAW_CMD = VERIFY_SHORT_ADDRESS */
    SS_ASSIGN_VERIFY_WAIT,  /* poll 0x015D */
    SS_ASSIGN_WITHDRAW,     /* write RAW_CMD_TWICE = WITHDRAW */

    /* Final terminate + scan trigger + wait + Pico rescan */
    SS_FINAL_TERMINATE,
    SS_FINAL_SCAN,
    SS_FINAL_WAIT,       /* wait for TinyModbus scan to complete (~200 ms) */
    SS_DONE,
    SS_ABORTED,
} sub_state_t;

/* ── Module state ─────────────────────────────────────────────────────────── */
prov_status_t g_prov_status = { .run_state = PROV_STATE_IDLE };

static struct {
    bool        active;
    uint8_t     slave_id;
    prov_mode_t mode;
    uint8_t     target_addr;   /* PROV_ADDRESS only */

    sub_state_t sub;

    /* Available address pool: which short addresses are free to assign */
    bool        used[64];      /* true = already occupied */
    uint8_t     avail[64];     /* ordered list of free addresses */
    uint8_t     avail_count;
    uint8_t     avail_idx;     /* next index into avail[] to assign */

    /* Scan state */
    uint8_t     scan_addr;     /* current address being scanned (0–63) */

    /* Binary search state */
    uint32_t    search_low;
    uint32_t    search_high;
    uint32_t    search_mid;

    /* Timing */
    uint32_t    last_cmd_ms;   /* time of last Modbus write */
    uint32_t    wait_until_ms; /* for delay states */

    /* Query result polling */
    volatile bool     result_ready;
    volatile uint16_t result_val;
    bool              poll_pending;  /* P4: true while poll_result() in flight */
} s;

/* ── Helpers ──────────────────────────────────────────────────────────────── */

static void set_status(prov_run_state_t run_state, const char *msg) {
    g_prov_status.run_state = run_state;
    strncpy(g_prov_status.msg, msg, PROV_STATUS_MSG_MAX - 1);
    g_prov_status.msg[PROV_STATUS_MSG_MAX - 1] = '\0';
    /* serial_protocol will see this on next serial_poll() tick */
    serial_provision_status_changed();
}

static uint32_t now_ms(void) {
    return to_ms_since_boot(get_absolute_time());
}

static bool gap_elapsed(void) {
    return (now_ms() - s.last_cmd_ms) >= DALI_CMD_GAP_MS;
}

/* Non-query command callback — just records the time */
static void cmd_cb(int res, uint8_t *buf, void *ctx) {
    s.last_cmd_ms = now_ms();
    (void)res; (void)buf; (void)ctx;
}

/* Query result callback — RAW_CMD (0x1000) */
static void query_result_cb(int res, uint8_t *buf, void *ctx) {
    s.last_cmd_ms  = now_ms();
    s.poll_pending = false;   /* P4: clear pending flag */
    /* P2: FC04 1-register frame = slave+FC+bytecount+2data+CRC = 7 bytes */
    if (res >= 7) {
        s.result_val = ((uint16_t)buf[3] << 8) | buf[4];
    } else {
        s.result_val = RAW_RESULT_NAK;
    }
    s.result_ready = true;
    (void)ctx;
}

static void write_raw(uint16_t cmd) {
    modbus_set_holding_register(s.slave_id, REG_RAW_CMD_TWICE,
                                cmd, cmd_cb, NULL);
}

/* Callback used solely to consume the stale RAW_RESULT register read */
static void discard_cb(int res, uint8_t *buf, void *ctx) {
    (void)res; (void)buf; (void)ctx;
}

static void write_raw_query(uint16_t cmd) {
    s.result_ready = false;
    s.result_val   = RAW_RESULT_PENDING;
    /* Read and discard the stale result register before issuing the query.
     * This ensures query_result_cb sees a fresh response, not a leftover. */
    modbus_get_input_registers(s.slave_id, REG_RAW_RESULT, 1,
                               discard_cb, NULL);
    modbus_set_holding_register(s.slave_id, REG_RAW_CMD,
                                cmd, cmd_cb, NULL);
    /* Result will arrive asynchronously; poll_result() reads it */
}

static void poll_result(void) {
    if (s.poll_pending) return;   /* P4: only one read in flight at a time */
    s.poll_pending = true;
    modbus_get_input_registers(s.slave_id, REG_RAW_RESULT, 1,
                               query_result_cb, NULL);
}

static void build_avail_list(void) {
    s.avail_count = 0;
    for (int a = 0; a < 64; a++) {
        if (!s.used[a]) {
            s.avail[s.avail_count++] = (uint8_t)a;
        }
    }
}

static void advance(sub_state_t next) {
    s.sub = next;
}

static void fail(const char *reason) {
    char msg[PROV_STATUS_MSG_MAX];
    snprintf(msg, sizeof(msg), "Error: %s", reason);
    set_status(PROV_STATE_ERROR, msg);
    /* Send terminate to clean up the DALI bus */
    write_raw(DALI_TERMINATE);
    s.active = false;
    s.sub    = SS_DONE;
}

/* ── Public API ───────────────────────────────────────────────────────────── */

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

    memset(&s, 0, sizeof(s));
    s.active      = true;
    s.slave_id    = slave_id;
    s.mode        = mode;
    s.target_addr = target_addr;
    s.search_low  = 0;
    s.search_high = 0xFFFFFF;
    s.last_cmd_ms = now_ms();

    g_prov_status.slave_id      = slave_id;
    g_prov_status.devices_found = 0;

    /* Choose entry sub-state based on mode */
    switch (mode) {
    case PROV_ALL:
        set_status(PROV_STATE_CLEARING, "Clearing existing addresses...");
        s.sub = SS_CLEAR_DTR0;
        break;
    case PROV_UNADDRESSED:
        set_status(PROV_STATE_SCANNING_USED,
                   "Scanning for used addresses... (0/64)");
        s.scan_addr = 0;
        s.sub = SS_SCAN_QUERY;
        break;
    case PROV_ADDRESS:
        set_status(PROV_STATE_TERMINATING, "Terminating previous session...");
        /* Mark everything except target as "used" so only target addr is free */
        for (int a = 0; a < 64; a++) s.used[a] = (a != target_addr);
        build_avail_list();
        s.sub = SS_TERMINATE;
        break;
    }
    return true;
}

void dali_provision_abort(void) {
    if (!s.active) return;
    write_raw(DALI_TERMINATE);
    s.active = false;
    s.sub    = SS_ABORTED;
    set_status(PROV_STATE_IDLE, "Aborted");
}

/* ── Main tick ────────────────────────────────────────────────────────────── */

void dali_provision_tick(void) {
    if (!s.active) return;

    char msg[PROV_STATUS_MSG_MAX];

    switch (s.sub) {

    /* ── Clearing ────────────────────────────────────────────────────────── */

    case SS_CLEAR_DTR0:
        if (!gap_elapsed()) break;
        modbus_set_holding_register(s.slave_id, REG_SET_DTR0,
                                    0x00FF, cmd_cb, NULL);
        advance(SS_CLEAR_BROADCAST);
        break;

    case SS_CLEAR_BROADCAST:
        if (!gap_elapsed()) break;
        write_raw(DALI_CLEAR_SHORT_ADDR);
        advance(SS_CLEAR_DONE);
        break;

    case SS_CLEAR_DONE:
        if (!gap_elapsed()) break;
        /* All addresses cleared — nothing is "used" */
        memset(s.used, 0, sizeof(s.used));
        build_avail_list();
        set_status(PROV_STATE_TERMINATING, "Terminating previous session...");
        s.sub = SS_TERMINATE;
        break;

    /* ── Scanning used addresses ─────────────────────────────────────────── */

    case SS_SCAN_QUERY:
        if (!gap_elapsed()) break;
        /* Query CONTROL_GEAR_PRESENT for s.scan_addr */
        write_raw_query(DALI_QUERY_PRESENT(s.scan_addr));
        advance(SS_SCAN_WAIT);
        break;

    case SS_SCAN_WAIT:
        if (!gap_elapsed()) break;
        poll_result();
        if (!s.result_ready) break;

        /* 0x02xx = response received = address is occupied */
        if ((s.result_val & 0xFF00) == RAW_RESULT_OK) {
            s.used[s.scan_addr] = true;
        }
        s.scan_addr++;
        if (s.scan_addr < 64) {
            snprintf(msg, sizeof(msg),
                     "Scanning for used addresses... (%u/64)", s.scan_addr);
            set_status(PROV_STATE_SCANNING_USED, msg);
            s.sub = SS_SCAN_QUERY;
        } else {
            build_avail_list();
            snprintf(msg, sizeof(msg),
                     "%u addresses available for assignment", s.avail_count);
            set_status(PROV_STATE_TERMINATING, msg);
            s.sub = SS_TERMINATE;
        }
        break;

    case SS_SCAN_DONE:
        /* Unused — fall through handled in SS_SCAN_WAIT */
        break;

    /* ── Terminate → Initialise → Randomise ─────────────────────────────── */

    case SS_TERMINATE:
        if (!gap_elapsed()) break;
        write_raw(DALI_TERMINATE);
        advance(SS_INITIALISE);
        break;

    case SS_INITIALISE:
        if (!gap_elapsed()) break;
        {
            uint16_t init_cmd;
            switch (s.mode) {
            case PROV_ALL:
                set_status(PROV_STATE_INITIALISING, "Initialising all devices...");
                init_cmd = DALI_INITIALISE_ALL;
                break;
            case PROV_UNADDRESSED:
                set_status(PROV_STATE_INITIALISING, "Initialising unaddressed devices...");
                init_cmd = DALI_INITIALISE_NEW;
                break;
            case PROV_ADDRESS:
                snprintf(msg, sizeof(msg),
                         "Initialising address %u...", s.target_addr);
                set_status(PROV_STATE_INITIALISING, msg);
                init_cmd = DALI_INITIALISE_ADDR(s.target_addr);
                break;
            default:
                fail("invalid mode");
                return;
            }
            write_raw(init_cmd);
            advance(SS_RANDOMISE);
        }
        break;

    case SS_RANDOMISE:
        if (!gap_elapsed()) break;
        set_status(PROV_STATE_RANDOMISING, "Randomising device addresses...");
        write_raw(DALI_RANDOMISE);
        s.wait_until_ms = now_ms() + DALI_RANDOMISE_DELAY_MS;
        advance(SS_RANDOMISE_WAIT);
        break;

    case SS_RANDOMISE_WAIT:
        /* Give devices time to generate their random addresses */
        if (now_ms() < s.wait_until_ms) break;
        /* Start binary search from scratch */
        s.search_low  = 0;
        s.search_high = 0xFFFFFF;
        set_status(PROV_STATE_SEARCHING,
                   "Searching for devices...");
        s.sub = SS_SEARCH_SET_H;
        break;

    /* ── Binary search ───────────────────────────────────────────────────── */

    case SS_SEARCH_SET_H:
        if (!gap_elapsed()) break;
        if (s.search_low > 0xFFFFFF) {
            /* No more devices */
            s.sub = SS_FINAL_TERMINATE;
            break;
        }
        s.search_mid = (s.search_low + s.search_high) / 2;
        /* Set Search Address H, M, L — three writes, all non-query */
        write_raw(DALI_SET_SEARCH_H((s.search_mid >> 16) & 0xFF));
        advance(SS_SEARCH_SET_M);
        break;

    case SS_SEARCH_SET_M:
        if (!gap_elapsed()) break;
        write_raw(DALI_SET_SEARCH_M((s.search_mid >> 8) & 0xFF));
        advance(SS_SEARCH_SET_L);
        break;

    case SS_SEARCH_SET_L:
        if (!gap_elapsed()) break;
        write_raw(DALI_SET_SEARCH_L(s.search_mid & 0xFF));
        advance(SS_SEARCH_COMPARE);
        break;

    case SS_SEARCH_COMPARE:
        if (!gap_elapsed()) break;
        write_raw_query(DALI_COMPARE);
        advance(SS_SEARCH_COMPARE_WAIT);
        break;

    case SS_SEARCH_COMPARE_WAIT:
        if (!gap_elapsed()) break;
        poll_result();
        if (!s.result_ready) break;

        {
            bool yes = (s.result_val & 0xFF00) == RAW_RESULT_OK
                       || (s.result_val & 0xFF00) == RAW_RESULT_ERROR;
            /* Note: bus collision (0x03xx) counts as YES per DALI spec and
             * original Lua: any response at all = at least one device present */

            if (yes) {
                if (s.search_low == s.search_mid) {
                    /* Found it — search_mid IS the random address */
                    snprintf(msg, sizeof(msg),
                             "Found device at 0x%06lX",
                             (unsigned long)s.search_mid);
                    set_status(PROV_STATE_SEARCHING, msg);

                    if (s.avail_count == 0 || s.avail_idx >= s.avail_count) {
                        fail("no free short addresses remaining");
                        return;
                    }
                    s.sub = SS_ASSIGN_PROG;
                } else {
                    s.search_high = s.search_mid;
                    s.sub = SS_SEARCH_SET_H;
                }
            } else {
                /* Nothing at or below mid — search above it */
                if (s.search_mid == 0xFFFFFF) {
                    /* Nothing found in entire remaining range */
                    s.sub = SS_FINAL_TERMINATE;
                } else {
                    s.search_low = s.search_mid + 1;
                    s.sub = SS_SEARCH_SET_H;
                }
            }
        }
        break;

    /* ── Assign short address to found device ────────────────────────────── */

    case SS_ASSIGN_PROG:
        if (!gap_elapsed()) break;
        {
            uint8_t short_addr = s.avail[s.avail_idx];
            snprintf(msg, sizeof(msg),
                     "Assigning address %u to device at 0x%06lX...",
                     short_addr, (unsigned long)s.search_mid);
            set_status(PROV_STATE_ASSIGNING, msg);
            write_raw(DALI_PROG_SHORT(short_addr));
            advance(SS_ASSIGN_VERIFY);
        }
        break;

    case SS_ASSIGN_VERIFY:
        if (!gap_elapsed()) break;
        write_raw_query(DALI_VERIFY_SHORT(s.avail[s.avail_idx]));
        advance(SS_ASSIGN_VERIFY_WAIT);
        break;

    case SS_ASSIGN_VERIFY_WAIT:
        if (!gap_elapsed()) break;
        poll_result();
        if (!s.result_ready) break;

        {
            bool ok = (s.result_val & 0xFF00) == RAW_RESULT_OK;
            uint8_t short_addr = s.avail[s.avail_idx];
            if (!ok) {
                snprintf(msg, sizeof(msg),
                         "Verify failed for address %u at device 0x%06lX",
                         short_addr, (unsigned long)s.search_mid);
                fail(msg);
                return;
            }
            snprintf(msg, sizeof(msg),
                     "Address %u assigned — %u device%s found so far",
                     short_addr,
                     g_prov_status.devices_found + 1,
                     g_prov_status.devices_found + 1 == 1 ? "" : "s");
            set_status(PROV_STATE_ASSIGNING, msg);
            g_prov_status.devices_found++;
            s.avail_idx++;
            s.sub = SS_ASSIGN_WITHDRAW;
        }
        break;

    case SS_ASSIGN_WITHDRAW:
        if (!gap_elapsed()) break;
        write_raw(DALI_WITHDRAW);
        /* Next search starts from one above the address we just found */
        s.search_low  = s.search_mid + 1;
        s.search_high = 0xFFFFFF;
        if (s.search_low > 0xFFFFFF) {
            s.sub = SS_FINAL_TERMINATE;
        } else {
            s.sub = SS_SEARCH_SET_H;
        }
        break;

    /* ── Wrap up ─────────────────────────────────────────────────────────── */

    case SS_FINAL_TERMINATE:
        if (!gap_elapsed()) break;
        write_raw(DALI_TERMINATE);
        advance(SS_FINAL_SCAN);
        break;

    case SS_FINAL_SCAN:
        if (!gap_elapsed()) break;
        /* Trigger TinyModbus firmware bus scan */
        modbus_set_holding_register(s.slave_id, 0x1010u, 0x0001u,
                                    cmd_cb, NULL);
        set_status(PROV_STATE_SEARCHING, "Waiting for bus rescan...");
        /* P3: wait ~300 ms for firmware scan to populate registers */
        s.wait_until_ms = now_ms() + 300u;
        advance(SS_FINAL_WAIT);
        break;

    case SS_FINAL_WAIT:
        if (now_ms() < s.wait_until_ms) break;
        /* P3: full blocking scan updates state_cache and emits scan_complete,
         * which the bridge uses to update per-ballast MQTT availability.
         * This is the same scan done at boot — after provisioning the Pico
         * must re-discover which addresses are now populated. */
        s.active = false;   /* release lock before blocking scan */
        dirty_poll_full_scan(s.slave_id);
        snprintf(msg, sizeof(msg),
                 "Complete: %u device%s commissioned",
                 g_prov_status.devices_found,
                 g_prov_status.devices_found == 1 ? "" : "s");
        set_status(PROV_STATE_COMPLETE, msg);
        s.sub = SS_DONE;
        break;

    case SS_DONE:
    case SS_ABORTED:
    case SS_IDLE:
        break;
    }
}
