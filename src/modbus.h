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

/* ── Original API (unchanged) ────────────────────────────────────────────── */
void modbus_get_coils(uint8_t device_id, uint16_t addr, uint16_t count,
                      modbus_action_cb_t cb, void *ctx);

void modbus_set_coil(uint8_t device_id, uint16_t addr, bool is_on,
                     modbus_action_cb_t cb, void *ctx);

/* Waveshare toggle coil (0x5500) */
void modbus_toggle(uint8_t device_id, uint16_t addr,
                   modbus_action_cb_t cb, void *ctx);

void modbus_get_holding_registers(uint8_t device_id, uint16_t addr,
                                   uint16_t count,
                                   modbus_action_cb_t cb, void *ctx);

void modbus_set_holding_register(uint8_t device_id, uint16_t addr,
                                  uint16_t value,
                                  modbus_action_cb_t cb, void *ctx);

/* ── New: FC04 Read Input Registers (for TinyModbus) ────────────────────── */
void modbus_get_input_registers(uint8_t device_id, uint16_t addr,
                                 uint16_t count,
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
