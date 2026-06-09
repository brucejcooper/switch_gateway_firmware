#pragma once
/*
 * core_mqtt_config.h — compile-time configuration for coreMQTT (found on the
 * include path; overrides the library defaults).
 */

/* Outstanding QoS1/2 publishes tracked. We publish QoS0 (retained discovery +
 * state), so this can be small. */
#define MQTT_STATE_ARRAY_MAX_COUNT      10U

/* Bounds on how long a single coreMQTT recv/send waits for the rest of a packet
 * over our non-blocking transport (it spins on getTime). Kept well under the
 * core-1 heartbeat-staleness threshold. */
#define MQTT_RECV_POLLING_TIMEOUT_MS    50U
#define MQTT_SEND_TIMEOUT_MS            500U

/* Route coreMQTT logging to our '#'-prefixed CDC log. Errors/warnings only;
 * info/debug silenced to keep the stream readable. coreMQTT calls these with
 * doubled parens, e.g. LogError(( "x=%d", x )). */
#include <stdio.h>
#define LogError( message )  do { printf("# E mqtt: "); printf message; printf("\n"); } while (0)
#define LogWarn( message )   do { printf("# W mqtt: "); printf message; printf("\n"); } while (0)
#define LogInfo( message )
#define LogDebug( message )
