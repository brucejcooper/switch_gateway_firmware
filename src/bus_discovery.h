#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/* Try to reach `slave_id` at 38400 baud. If it doesn't respond, drop the
 * master UART to 9600, ping again, and on success send the per-device
 * "set baud to 38400" FC06 write, then return the master UART to 38400
 * and re-verify. Returns true if the slave is reachable at 38400 by the
 * time the function returns. Logs progress at INFO level via the log
 * facility. Blocking; expected to be called only during boot before the
 * main loop starts. `kind` must be SLAVE_KIND_RELAY or SLAVE_KIND_DALI. */
bool bus_discover_slave(uint8_t slave_id, slave_kind_t kind);
