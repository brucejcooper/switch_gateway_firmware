#pragma once
/*
 * ha_bridge.h — Home Assistant translation layer on core 1.
 *
 * Reproduces mqtt_bridge.py's duties in C: MQTT auto-discovery, the availability
 * model, event->state-topic translation, and inbound command translation. Driven
 * by the mqtt_app connected/message hooks plus a tick for the incremental
 * discovery publish.
 */

#include "intercore.h"

/* Register the mqtt_app connected/message hooks. Call once on core 1. */
void ha_bridge_init(void);

/* Pump incremental discovery (and, later, queued state publishes). Call every
 * core-1 loop iteration while connected. */
void ha_bridge_tick(void);

/* Translate one inter-core event to its retained MQTT state topic(s).
 * Called as the event queue is drained while connected. */
void ha_bridge_publish_event(const ic_event_t *e);
