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

/* Synthesize a raw button event (for testing without physical hardware):
 * queues it just like the PIO ISR would.  A tap = PRESS then RELEASE. */
void button_fsm_inject(uint8_t fixture, uint8_t button, btn_raw_type_t type);

/* Call from main loop — drains raw queue, advances all FSMs,
 * pushes lighting commands into the modbus action queue indirectly
 * by calling the lighting command functions directly. */
void button_fsm_tick(void);

/* Defined in button_fsm_emit.c, called from button_fsm.c. Declared here rather
 * than as a bare extern inside button_fsm.c so the two sides share one
 * prototype, so a signature change on either side is caught by the
 * compiler. `action` is a literal: press/release/click/long_press/unstuck. */
void emit_button_event(uint8_t fixture, uint8_t button, const char *action);
