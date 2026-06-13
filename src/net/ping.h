#pragma once
#include <stdint.h>

/*
 * ping.h — CLI network ping (ICMP echo) on core 1.
 *
 * Driven by a 'net ping <ip> [count]' CLI command on core 0, which sends an
 * IC_EVT_PING_REQ down the event queue. Core 1 runs the ICMP exchange and
 * pushes one IC_CMD_PING_REPLY per echo back to core 0 for printing.
 */

/* Submit a ping job: send `count` ICMP echoes to ip[4]. One job at a time — a
 * submit while another is active replies BUSY for the new target. */
void ping_submit(const uint8_t ip[4], uint8_t count);

/* Advance the ping state machine. Call once per core-1 loop iteration. */
void ping_tick(void);
