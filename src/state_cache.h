#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "config.h"

#define RELAY_COUNT     32
#define DALI_ADDR_COUNT 64

typedef struct {
    uint8_t arc_level;
    uint8_t status_byte;
    uint8_t max_level;
    uint8_t min_level;
    uint8_t power_on_level;
    uint8_t sys_fail_level;
    uint8_t fade_rate;
    uint8_t fade_time;
    bool    present;
    /* True once a raw config read has actually landed for this ballast. Until
     * then its six config fields are ZERO because nobody has asked the gear yet
     * — not because the ballast reports zeros — so they must not be published.
     * Publishing them would assert min=0/max=0 as fact for a device we have not
     * finished interrogating. */
    bool    cfg_known;
    bool    fading;
} dali_ballast_state_t;

void state_cache_init(void);

/* Relay */
void state_cache_set_relay(uint8_t ch, bool on);

/* True once a coil READ has landed for this channel. Until then the cached
 * false is a default, not a measurement, so nothing about the channel may be
 * published — a failed boot coil sync must not put 32 confident "OFF"s on the
 * broker. Set by state_cache_set_relay (every writer is a read or a write-ACK). */
bool state_cache_relay_known(uint8_t ch);
bool state_cache_get_relay_on(uint8_t ch);

/* DALI — slave_idx is 0-based index into g_config.dali_slave_ids */
dali_ballast_state_t *state_cache_get_dali(uint8_t slave_idx, uint8_t addr);
void state_cache_set_dali_arc(uint8_t slave_idx, uint8_t addr,
                               uint8_t arc, uint8_t status);
void state_cache_set_dali_config(uint8_t slave_idx, uint8_t addr,
                                  uint8_t max_l, uint8_t min_l,
                                  uint8_t power_on, uint8_t sys_fail,
                                  uint8_t fade_rate, uint8_t fade_time);
void state_cache_set_dali_present(uint8_t slave_idx, uint8_t addr, bool present);

/* Returns slave index for a given slave_id, or -1 if unknown */
int  state_cache_slave_idx(uint8_t slave_id);

/* Per-slave DALI group membership cache, populated by the DALI interrogation
 * (dali_state.c, QUERY GROUPS 0-7 / 8-15).
 * group_members[g] is a 64-bit mask of ballasts in group g. */
void     state_cache_set_dali_group(uint8_t slave_idx, uint8_t group, uint64_t mask);
uint64_t state_cache_get_dali_group(uint8_t slave_idx, uint8_t group);

/* True if any present ballast in this group has arc_level > 0. */
bool     state_cache_dali_group_any_on(uint8_t slave_idx, uint8_t group);

/* Max arc level across present members; 0 if no member is on. */
uint8_t  state_cache_dali_group_max_bri(uint8_t slave_idx, uint8_t group);

