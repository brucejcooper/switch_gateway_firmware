#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "pico/time.h"
#include "pico/util/queue.h"
#include "mapping.h"

typedef enum { BTN_RAW_PRESS, BTN_RAW_RELEASE } btn_raw_type_t;

typedef struct {
    btn_raw_type_t  type;
    uint8_t         fixture;
    uint8_t         button;
    absolute_time_t time;
} btn_raw_event_t;

/* Written by PIO ISR in button_actions.c, read by button_fsm_tick() */
extern queue_t g_btn_raw_queue;
#define BTN_RAW_QUEUE_DEPTH 64

void button_fsm_init(void);

/* Call from main loop — drains raw queue, advances all FSMs,
 * pushes lighting commands into the modbus action queue indirectly
 * by calling the lighting command functions directly. */
void button_fsm_tick(void);
