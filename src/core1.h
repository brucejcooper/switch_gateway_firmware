#pragma once
/*
 * core1.h — launch + liveness for the core-1 network bridge.
 *
 * Core 1 runs the W5100S + MQTT bridge (built out over the following commits).
 * Core 0 owns the hardware watchdog and feeds it only while core 1's heartbeat
 * is fresh, so a genuine core-1 wedge reboots the chip — but a network outage
 * (core 1 still looping through its reconnect state machine) never does, keeping
 * the lights working with Home Assistant gone.
 */

#include <stdint.h>

/* Core 0 feeds the watchdog unconditionally for this long after core 1 launch
 * (W5100S init / DHCP can take seconds before the first useful beat). */
#define CORE1_GRACE_MS 15000u

/* Core-1 heartbeat is considered stale (core 1 wedged) past this many ms. Must
 * comfortably exceed the worst-case flash write — during a flash_safe_execute()
 * BOTH cores park, so core 1's beat pauses for the whole write. A 4 KB sector is
 * tens of ms; littlefs GC can be longer, but well under 2 s. */
#define CORE1_STALE_MS 2000u

/* Launch core 1. Call once on core 0 AFTER intercore_init() and BEFORE the first
 * core-0 flash write (e.g. fs_init's first-boot format). Blocks (bounded) until
 * core 1 has registered as the flash lockout victim. Core 1 then parks until
 * core1_config_ready(). */
void core1_launch(void);

/* Release core 1 to bring up the network. Call on core 0 after the config
 * snapshot (intercore_set_snapshot) is populated. */
void core1_config_ready(void);

/* Last core-1 loop timestamp (ms since boot). Updated every core-1 iteration,
 * including mid-reconnect — means "core 1's loop is alive", not "connected".
 * A single aligned 32-bit read is atomic on Cortex-M0+. */
uint32_t core1_heartbeat_ms(void);
