#pragma once
/*
 * netif.h — W5100S bring-up, addressing (DHCP/static), PHY link and recovery.
 *
 * A non-blocking state machine pumped from the core-1 loop. It gets the board
 * onto the network and keeps it there: version sanity, RTR/RCR tuning, DHCP
 * lease management, PHY link parking, and a full chip reset + re-init on
 * repeated failure (SPI desync leaves the chip in a state socket-level recovery
 * can't clear). The TCP socket + MQTT layers sit on top of this.
 */

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    bool    dhcp;                       /* true: DHCP (default); false: static */
    uint8_t ip[4], sn[4], gw[4], dns[4];/* used only when dhcp == false */
} netif_config_t;

/* Configure SPI glue + addressing mode. cfg NULL => DHCP. Does not touch the
 * chip yet (that happens in the first netif_tick). */
void netif_init(const netif_config_t *cfg);

/* Pump the state machine. Call every core-1 loop iteration (non-blocking). */
void netif_tick(void);

/* True once link is up and an IP is assigned (DHCP leased or static applied). */
bool netif_ready(void);

/* Current PHY link state. */
bool netif_link_up(void);

/* Copy the current IP (zeros if not yet assigned). */
void netif_get_ip(uint8_t ip[4]);

/* DNS server address (from DHCP or static config); zeros if unknown. */
void netif_get_dns(uint8_t dns[4]);
