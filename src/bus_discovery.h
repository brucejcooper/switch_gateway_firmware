#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Try to reach `slave_id` at 19200 baud. If it doesn't respond, drop the
 * master UART to 9600, ping again, and on success send the per-device
 * "set baud to 19200" FC06 write, then return the master UART to 19200
 * and re-verify. Returns true if the slave is reachable at 19200 by the
 * time the function returns. Logs progress at INFO level via the log
 * facility. Blocking; expected to be called only during boot before the
 * main loop starts. `kind` must be SLAVE_KIND_RELAY or SLAVE_KIND_DALI. */
bool bus_discover_slave(uint8_t slave_id, slave_kind_t kind);

/* Single ping at the CURRENT master UART baud. Returns true if the slave
 * answered. Relay slaves are pinged with FC01 (read 1 coil), DALI slaves with
 * FC03 (read the slave_addr holding register). Blocking (up to ~1 s).
 * Diagnostic helper exposed for the CLI. */
bool bus_ping_slave(uint8_t slave_id, slave_kind_t kind);

/* Tell a slave to switch its own UART baud via an FC06 write to its baud
 * register (relay 0x2000, DALI 0x1013). Only 9600, 19200 and 38400 are
 * supported; any other rate returns false without sending anything. The master UART is
 * left unchanged — the caller must match baud afterwards. Returns true on the
 * slave's ACK. Blocking (up to ~1 s). */
bool bus_set_slave_baud(uint8_t slave_id, slave_kind_t kind, uint32_t baud);
