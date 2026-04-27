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
 * Persisted to /modbus_baud.txt. Default 38400 to match TinyModbus config. */
void modbus_set_baud(uint32_t baud);
uint32_t modbus_get_baud(void);
