#pragma once

/*
 * cli_provision.h — provisioning status -> core-1 event forwarding
 * (cli_provision.c, renamed from serial_protocol_provision.c when
 * serial_protocol.c split into cli.c + ic_seam.c).
 *
 * Stays its own small file, as it always has: it's a CLI-adjacent status
 * notifier (armed by dali_provision.c and the `provision` verb) whose actual
 * work is a plain IC_EVT_PROVISION push, polled from ic_seam_pump() alongside
 * the rest of the domain-state-to-core-1 traffic.
 */

#include <stdbool.h>

/* Called from dali_provision.c when status changes — schedules an emit */
void serial_provision_status_changed(void);
/* Call from ic_seam_pump() each iteration */
void serial_provision_poll(void);
