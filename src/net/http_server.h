#pragma once
#include <stdint.h>
/*
 * http_server.h — embedded config web server on core 1.
 *
 * A tiny non-blocking HTTP/1.1 server that coexists with the MQTT client on the
 * W5100S. It serves the compiled-in single-page UI (web_assets) and a small REST
 * API backed by the core-1 read model (web_state) for reads and the inter-core
 * command queue for writes. Basic-auth gated (credentials from the config
 * snapshot). Plain HTTP — no TLS on the W5100S — so this is a LAN-only fallback
 * for commissioning a device before it reaches Home Assistant.
 *
 * Socket budget: the W5100S has 4 hardware sockets; 0=DHCP, 1=MQTT. This server
 * uses sockets 2-3 (and, opportunistically, socket 0 while DHCP is idle — see
 * the netif arbiter). Each connection serves one request then closes (no
 * keep-alive) so the scarce sockets are reclaimed promptly. All I/O is
 * non-blocking and bounded per tick so MQTT and the watchdog stay healthy.
 */

void http_server_init(void);
void http_server_tick(void);

/* Count of socket()/listen() (re)arm failures — a climbing value means a web
 * listener is dead. Counted (not logged) because arm_listen runs in the hot
 * pump where logging would stall core 1 (USB-CDC back-pressure). */
uint32_t http_server_arm_fail_count(void);
