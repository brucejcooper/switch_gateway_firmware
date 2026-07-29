#include "state_cache.h"
#include "config.h"            /* fix #3: g_config */
#include "mapping.h"
#include <string.h>

static bool                  s_relay[RELAY_COUNT];
static dali_ballast_state_t  s_dali[MAX_DALI_SLAVES][DALI_ADDR_COUNT];
static uint64_t              s_dali_groups[MAX_DALI_SLAVES][16];

void state_cache_init(void) {
    memset(s_relay,        0, sizeof(s_relay));
    memset(s_dali,         0, sizeof(s_dali));
    memset(s_dali_groups,  0, sizeof(s_dali_groups));
}

void state_cache_set_relay(uint8_t ch, bool on) {
    if (ch < RELAY_COUNT) s_relay[ch] = on;
}

bool state_cache_get_relay_on(uint8_t ch) {
    return ch < RELAY_COUNT ? s_relay[ch] : false;
}

int state_cache_slave_idx(uint8_t slave_id) {
    for (int i = 0; i < g_config.dali_slave_count; i++) {
        if (g_config.dali_slave_ids[i] == slave_id) return i;
    }
    return -1;
}

dali_ballast_state_t *state_cache_get_dali(uint8_t slave_idx, uint8_t addr) {
    if (slave_idx >= MAX_DALI_SLAVES || addr >= DALI_ADDR_COUNT) return NULL;
    return &s_dali[slave_idx][addr];
}

void state_cache_set_dali_arc(uint8_t slave_idx, uint8_t addr,
                               uint8_t arc, uint8_t status) {
    if (slave_idx >= MAX_DALI_SLAVES || addr >= DALI_ADDR_COUNT) return;
    bool fading = (status & 0x10) != 0;
    s_dali[slave_idx][addr].arc_level   = arc;
    s_dali[slave_idx][addr].status_byte = status;
    s_dali[slave_idx][addr].fading      = fading;
    /* Re-sync the on/off intent from a stable read (catches external changes) —
     * but only once the command settle window has elapsed.  Within the window
     * the just-commanded intent is authoritative, so a stale read (taken between
     * command-send and fade-start) or a mid-fade read can't clobber a rapid
     * in-flight toggle.  Rapid tapping keeps extending cmd_time, so the intent
     * stays command-driven until the user stops. */
    #define DALI_CMD_SETTLE_US 700000  /* 0.7 s */
    if (!fading &&
        absolute_time_diff_us(s_dali[slave_idx][addr].cmd_time, get_absolute_time())
            > DALI_CMD_SETTLE_US) {
        s_dali[slave_idx][addr].commanded_on = (arc > 0);
    }
}

void state_cache_set_dali_commanded_on(uint8_t slave_idx, uint8_t addr, bool on) {
    if (slave_idx >= MAX_DALI_SLAVES || addr >= DALI_ADDR_COUNT) return;
    s_dali[slave_idx][addr].commanded_on = on;
    s_dali[slave_idx][addr].cmd_time     = get_absolute_time();
}

void state_cache_set_dali_config(uint8_t slave_idx, uint8_t addr,
                                  uint8_t max_l, uint8_t min_l,
                                  uint8_t power_on, uint8_t sys_fail,
                                  uint8_t fade_rate, uint8_t fade_time) {
    if (slave_idx >= MAX_DALI_SLAVES || addr >= DALI_ADDR_COUNT) return;
    dali_ballast_state_t *b = &s_dali[slave_idx][addr];
    b->max_level      = max_l;
    b->min_level      = min_l;
    b->power_on_level = power_on;
    b->sys_fail_level = sys_fail;
    b->fade_rate      = fade_rate;
    b->fade_time      = fade_time;
}

void state_cache_set_dali_present(uint8_t slave_idx, uint8_t addr, bool present) {
    if (slave_idx >= MAX_DALI_SLAVES || addr >= DALI_ADDR_COUNT) return;
    s_dali[slave_idx][addr].present = present;
}

void state_cache_set_dali_group(uint8_t slave_idx, uint8_t group, uint64_t mask) {
    if (slave_idx >= MAX_DALI_SLAVES || group >= 16) return;
    s_dali_groups[slave_idx][group] = mask;
}

uint64_t state_cache_get_dali_group(uint8_t slave_idx, uint8_t group) {
    if (slave_idx >= MAX_DALI_SLAVES || group >= 16) return 0;
    return s_dali_groups[slave_idx][group];
}

bool state_cache_dali_group_any_on(uint8_t slave_idx, uint8_t group) {
    if (slave_idx >= MAX_DALI_SLAVES || group >= 16) return false;
    uint64_t mask = s_dali_groups[slave_idx][group];
    while (mask) {
        uint8_t a = (uint8_t)__builtin_ctzll(mask);
        mask &= mask - 1;
        if (a >= DALI_ADDR_COUNT) continue;
        dali_ballast_state_t *b = &s_dali[slave_idx][a];
        if (b->present && b->commanded_on) return true;
    }
    return false;
}

uint8_t state_cache_dali_group_max_bri(uint8_t slave_idx, uint8_t group) {
    if (slave_idx >= MAX_DALI_SLAVES || group >= 16) return 0;
    uint64_t mask = s_dali_groups[slave_idx][group];
    uint8_t  max  = 0;
    while (mask) {
        uint8_t a = (uint8_t)__builtin_ctzll(mask);
        mask &= mask - 1;
        if (a >= DALI_ADDR_COUNT) continue;
        dali_ballast_state_t *b = &s_dali[slave_idx][a];
        if (b->present && b->arc_level > max) max = b->arc_level;
    }
    return max;
}

