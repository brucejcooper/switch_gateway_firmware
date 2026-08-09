#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* ── DE mode (persisted to /modbus_config.txt, as in original) ───────────── */
typedef enum {
    MODBUS_DE_WHENTX,
    MODBUS_DE_ALWAYS,
    MODBUS_DE_MAX,
} modbus_de_mode_t;

void set_modbus_de_mode(modbus_de_mode_t mode);

/* Current RS485 DE (transmit) mode, and its short string name
 * ("tx" / "always"). to_str returns "?" for an out-of-range value. */
modbus_de_mode_t modbus_get_de_mode(void);
const char *modbus_de_mode_to_str(modbus_de_mode_t mode);

/* Decode a Modbus exception code (the byte after the OR'd-0x80 function code
 * in an error response) into a short human-readable string. Returns "unknown"
 * for codes not in the standard table. */
const char *modbus_exception_str(uint8_t code);

/* ── Callback type ───────────────────────────────────────────────────────── */
/* res > 0: bytes received; res == 0: timeout; res < 0: Modbus exception.
 * buf contains the raw response frame (addr, fc, payload, crc).
 * This is identical to the original callback contract. */
typedef void (*modbus_action_cb_t)(int res, uint8_t *buf, void *ctx);

/* ── Lifecycle ───────────────────────────────────────────────────────────── */
void modbus_init(void);

/* Call from main loop — advances bus state machine. Non-blocking. */
void modbus_poll(void);

/* Total actions dropped because the action queue was full (diagnostics). */
uint32_t modbus_drop_count(void);

/* ── Bus-occupancy accounting (bench instrumentation) ─────────────────────────
 * Answers "who is using the RS-485 bus, and how much of it is left?". Each
 * transaction is charged from leaving the action queue until the SM returns to
 * IDLE — including its retries and the post-frame dead time, i.e. what it really
 * costs every other producer. A dead slave being retried is therefore visible as
 * the bus hog it is. Call modbus_busload_reset() to start a window, then
 * modbus_busload_get() to read it. */
#define MODBUS_BUSLOAD_SLAVES 6u

typedef struct {
    uint8_t  slave;
    uint32_t txns;
    uint64_t us;
} modbus_busload_slave_t;

void modbus_busload_reset(void);

/* window_us = wall time since the reset; busy_us = bus-occupied time within it;
 * txns = transactions completed; `per` (if non-NULL) receives
 * MODBUS_BUSLOAD_SLAVES per-slave buckets (txns == 0 marks an unused bucket).
 * Any pointer may be NULL. */
void modbus_busload_get(uint64_t *window_us, uint64_t *busy_us, uint32_t *txns,
                        modbus_busload_slave_t *per);

/* Replies discarded because the frame's CRC-16 did not check out, since boot.
 * A bad-CRC frame is treated exactly like no reply at all (retried if the
 * action allows it), so this counter is the only place a corrupted-but-present
 * reply becomes visible. Should be 0 on a healthy bus — compare against the
 * gateway's own `crc_error` (FC04 0x0300 reg2) to place the fault on the
 * controller's or the gateway's receive side. */
uint32_t modbus_crc_error_count(void);

/* Re-sends within the busload window: `timeouts` = the reply was missing or
 * invalid, `device_busy` = the slave answered exception 0x06. These distinguish
 * "the slave is slow to answer" from "the reply is being lost" — a transaction
 * that retries and then succeeds is invisible in every per-transaction success
 * count, but costs a full response timeout of bus time. Either may be NULL. */
void modbus_busload_retries(uint32_t *timeouts, uint32_t *device_busy);

/* Absent-slave backoff: transactions skipped without touching the bus, and how
 * many slaves are currently backed off. A slave that is not there otherwise
 * costs a full response timeout per attempt, which is the single largest
 * contributor to worst-case command latency. Either pointer may be NULL. */
void modbus_busload_backoff(uint32_t *skipped, uint8_t *slaves_backed_off);

/* Slave turnaround within the busload window: time from our request clearing the
 * line to the first valid reply byte pattern completing, over all slaves that
 * answered. Compare against MODBUS_RESPONSE_TIMEOUT_MS — a slave whose worst
 * turnaround approaches it is producing "lost" replies that are really late ones.
 * Any pointer may be NULL. */
void modbus_busload_turnaround(uint32_t *avg_us, uint32_t *max_us, uint32_t *n);

/* ── Enqueue API ─────────────────────────────────────────────────────────────
 * Each returns true if the action was queued, false if the action queue was full
 * and it was dropped (in which case the callback will NEVER fire). Stateful
 * producers (e.g. dirty_poll) MUST check the result: a producer that advances an
 * internal "awaiting" state before enqueuing will wedge permanently if it ignores
 * a drop, because the completion callback that would clear the state never runs.
 * Fire-and-forget callers (commands) may ignore it — the drop is logged. */
bool modbus_get_coils(uint8_t device_id, uint16_t addr, uint16_t count,
                      modbus_action_cb_t cb, void *ctx);

bool modbus_set_coil(uint8_t device_id, uint16_t addr, bool is_on,
                     modbus_action_cb_t cb, void *ctx);

/* Waveshare toggle coil (0x5500) */
bool modbus_toggle(uint8_t device_id, uint16_t addr,
                   modbus_action_cb_t cb, void *ctx);

bool modbus_get_holding_registers(uint8_t device_id, uint16_t addr,
                                   uint16_t count,
                                   modbus_action_cb_t cb, void *ctx);

bool modbus_set_holding_register(uint8_t device_id, uint16_t addr,
                                  uint16_t value,
                                  modbus_action_cb_t cb, void *ctx);

/* ── New: FC04 Read Input Registers (for TinyModbus) ────────────────────── */
bool modbus_get_input_registers(uint8_t device_id, uint16_t addr,
                                 uint16_t count,
                                 modbus_action_cb_t cb, void *ctx);

/* FC 0x41 — the DALI command queue's single function code (queue v4; see
 * TinyModbus/QUEUE-PROTOCOL.md §1). One request optionally enqueues a command
 * and optionally acks the head of the gateway's response queue: `ctl` is the
 * packed [flags:3|ack:5] byte and `payload` is the enqueue payload — 0 bytes
 * (pure ack/poll, 5 B on the wire), 2 bytes `addr,data` (7 B), or 3 bytes
 * `addr,data,dtr0` (8 B, QF_DTR0 set). The reply is correspondingly 6 or 7
 * bytes; the caller knows which because it chose the request shape.
 *
 * A request that carries an enqueue payload is issued with retries DISABLED
 * (spec §5: "no transport auto-retry of enqueue frames"). A blind re-send of a
 * frame whose reply was merely lost enqueues the command a SECOND time — a
 * silent double execution, which for a non-idempotent UP/DOWN step is a real
 * defect. Resolving a lost enqueue reply is mbq_client's §4 job, and it needs
 * to see the timeout rather than have the transport paper over it. Pure
 * ack/poll frames keep the retries: re-acking a popped seq is a no-op. */
bool modbus_mbq_v4(uint8_t dev, uint8_t ctl, const uint8_t *payload,
                   uint8_t payload_len, modbus_action_cb_t cb, void *ctx);


/* ── Baud rate (new — replaces hardcoded 9600) ───────────────────────────── */
/* Call before modbus_init() if a non-default baud rate is desired.
 * Persisted to /modbus_baud.txt. Default 19200 — the reliable bus speed for the
 * TinyModbus slaves (38400 is marginal against the internal-RC oscillator). */
void modbus_set_baud(uint32_t baud);
uint32_t modbus_get_baud(void);

/* Switch the master UART baud rate at runtime without touching the
 * persisted config file. Only safe when no Modbus action is in flight —
 * intended for boot-time discovery. */
void modbus_uart_set_baud_runtime(uint32_t baud);
