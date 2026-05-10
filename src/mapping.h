#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "config.h"

#define MAX_FIXTURES        24
#define MAX_BUTTONS          8    /* 7 physical + 1 spare, keep byte-aligned */
#define MAX_TARGETS_PER_BTN  8

#define SLAVE_RELAY 0u

typedef struct {
    uint8_t slave;   /* SLAVE_RELAY or DALI slave ID; 0xFF = empty */
    uint8_t addr;
} mapping_target_t;

typedef struct {
    uint8_t           fixture;
    uint8_t           button;
    uint8_t           target_count;
    uint8_t           _pad;
    mapping_target_t  targets[MAX_TARGETS_PER_BTN];
} mapping_record_t;

extern mapping_record_t g_mappings[MAX_FIXTURES][MAX_BUTTONS];

bool mapping_load(void);
bool mapping_save(void);

const mapping_record_t *mapping_lookup(uint8_t fixture, uint8_t button);

bool mapping_set(uint8_t fixture, uint8_t button,
                 const mapping_target_t *targets, uint8_t count);
bool mapping_delete(uint8_t fixture, uint8_t button);

/* Serialise all non-empty mappings as JSON lines into buf.
 * Returns bytes written, or -1 if buffer too small. */
int mapping_serialise_all(char *buf, int buf_len);
