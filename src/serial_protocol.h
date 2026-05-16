#pragma once

#include <stdbool.h>

/* Initialise serial protocol (nothing to do — stdio_usb handles CDC init) */
void serial_protocol_init(void);

/* Call from main loop — reads USB CDC input, emits pending state events. */
void serial_poll(void);

/* Emit hello message after boot */
void serial_tx_hello(void);

/* Emit ack for an HA command */
void serial_tx_ack(unsigned cmd_id, bool ok, const char *err);

/* Emit full mapping config dump */
void serial_tx_config_full(void);

#define SERIAL_LINE_MAX 256

/* ── Provisioning support (serial_protocol_provision.c) ─────────────────── */
/* Called from dali_provision.c when status changes — schedules an emit */
void serial_provision_status_changed(void);
/* Call from serial_poll() each iteration */
void serial_provision_poll(void);
/* Dispatch provision_start / provision_abort inbound messages */
void serial_handle_provision_start(const char *line, unsigned cmd_id);
void serial_handle_provision_abort(const char *line, unsigned cmd_id);
