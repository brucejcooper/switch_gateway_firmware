#pragma once

#include <stdbool.h>

/* Initialise serial protocol (nothing to do — stdio_usb handles CDC init) */
void serial_protocol_init(void);

/* Call from main loop — reads USB CDC input, emits pending state events. */
void serial_poll(void);

/* Emit hello message after boot */
void serial_tx_hello(void);

/* Push every binding to core 1 as config_mapping events (used by resync). */
void serial_tx_config_full(void);

#define SERIAL_LINE_MAX 256

/* ── Provisioning support (serial_protocol_provision.c) ─────────────────── */
/* Called from dali_provision.c when status changes — schedules an emit */
void serial_provision_status_changed(void);
/* Call from serial_poll() each iteration */
void serial_provision_poll(void);
