#pragma once
/*
 * mqtt_app.h — MQTT connection manager for core 1.
 *
 * Drives a non-blocking TCP socket on the W5100S (reliability: getSn_SR state
 * machine, explicit SOCK_CLOSE_WAIT recycle, Sn_IR act+clear, per-op wall-clock
 * timeout, Sn_KPALVTR hardware keepalive) and coreMQTT on top of it (rigorous
 * PINGRESP keepalive, Last-Will = <prefix>/availability "offline", exponential
 * backoff + jitter). The connection is treated as disposable: any fault tears
 * down to a known state and rebuilds.
 *
 * The broker is addressed by IP (the recommended fixed-LAN setup). Hostname /
 * DNS resolution is a noted follow-up (ioLibrary DNS_run blocks, unusable in the
 * cooperative core-1 loop without a cooperative resolver).
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

typedef struct {
    uint8_t  broker_ip[4];
    uint16_t broker_port;     /* default 1883 */
    char     prefix[24];      /* topic prefix, e.g. "lighting" */
    char     user[48];
    char     pass[64];
    bool     have_creds;
} mqtt_config_t;

/* cfg NULL => compile-time placeholder defaults (broker 0.0.0.0:1883, prefix
 * "lighting", no creds) — the real /config.json values are wired in later. */
void mqtt_app_init(const mqtt_config_t *cfg);

/* Pump the connection manager. Call every core-1 loop iteration. */
void mqtt_app_tick(void);

/* True while an MQTT session to the broker is established. */
bool mqtt_app_connected(void);

/* Publish (QoS0). Safe to call only while connected; returns false otherwise or
 * on transport error. */
bool mqtt_app_publish(const char *topic, const void *payload, size_t len, bool retain);

/* Subscribe (QoS0) to a topic filter. */
bool mqtt_app_subscribe(const char *filter);

/* Free space in the socket TX buffer (bytes). Used to flow-control the bulk
 * discovery publish so it never blocks on a full buffer. 0 if not connected. */
uint16_t mqtt_app_tx_free(void);

/* The configured topic prefix (e.g. "lighting"). */
const char *mqtt_app_prefix(void);

/* Hook invoked once each time an MQTT session becomes established (after the
 * availability=online publish): used to (re)publish discovery, subscribe, and
 * request a resync. Set by the translation layer. */
void mqtt_app_set_connected_cb(void (*cb)(void));

/* Hook invoked for each inbound PUBLISH (topic/payload), set by the translation
 * layer. Lengths are not NUL-terminated. */
void mqtt_app_set_message_cb(void (*cb)(const char *topic, uint16_t topic_len,
                                        const uint8_t *payload, uint16_t payload_len));
