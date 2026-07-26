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

/* ── New: FC16 / FC23 for the gateway command queue (task 04) ───────────────
 * FC16 writes `count` registers (big-endian bytes in `data`, count*2 of them)
 * at `addr`.  FC23 writes wr_count registers at wr_addr THEN reads rd_count at
 * rd_addr in one transaction — the response carries the post-write read block
 * (the queue's submit-N / collect-N-1 pipeline).  Both cap the write side at
 * MODBUS_WR_PAYLOAD_MAX bytes (8 registers) and return false (dropped, cb never
 * fires) if that is exceeded or the register counts are zero. */
bool modbus_write_multiple_registers(uint8_t device_id, uint16_t addr,
                                     uint16_t count, const uint8_t *data,
                                     modbus_action_cb_t cb, void *ctx);

bool modbus_read_write_multiple_registers(uint8_t device_id,
                                          uint16_t rd_addr, uint16_t rd_count,
                                          uint16_t wr_addr, uint16_t wr_count,
                                          const uint8_t *wr_data,
                                          modbus_action_cb_t cb, void *ctx);

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
