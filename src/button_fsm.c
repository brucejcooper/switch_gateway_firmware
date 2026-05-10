/*
 * button_fsm.c — per-button state machine.
 *
 * Single-core version: instead of pushing to a lighting_cmd_queue consumed
 * by a separate executor, we call the lighting action functions directly.
 * This is safe because everything runs on core 0 and modbus_poll() is
 * non-blocking (ISR-driven).
 */

#include "button_fsm.h"
#include "lighting_actions.h"   /* direct lighting command functions */
#include "mapping.h"            /* fix #2: SLAVE_RELAY, MAX_FIXTURES, MAX_BUTTONS */
#include "state_cache.h"
#include "config.h"
#include "pico/time.h"
#include <string.h>

/* forward declaration — implemented in button_fsm_emit.c */
void emit_button_event(uint8_t fixture, uint8_t button, const char *action);

queue_t g_btn_raw_queue;

typedef enum { DIM_UP, DIM_DOWN } dim_dir_t;

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_HOLDING,
    BTN_STATE_RELEASED_AFTER_HOLD,
} btn_state_t;

typedef struct {
    btn_state_t state;
    uint32_t    press_time_ms;
    uint32_t    release_time_ms;
    uint32_t    last_repeat_ms;
    dim_dir_t   last_dir;
    bool        dir_locked;
} btn_fsm_state_t;

static btn_fsm_state_t s_fsm[MAX_FIXTURES][MAX_BUTTONS];

void button_fsm_init(void) {
    queue_init(&g_btn_raw_queue, sizeof(btn_raw_event_t), BTN_RAW_QUEUE_DEPTH);
    memset(s_fsm, 0, sizeof(s_fsm));
}

static dim_dir_t initial_dir(uint8_t fixture, uint8_t button) {
    return state_cache_any_on(fixture, button) ? DIM_DOWN : DIM_UP;
}

static void do_toggle(uint8_t fixture, uint8_t button) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    if (!rec) return;
    bool any_on = state_cache_any_on(fixture, button);
    for (int i = 0; i < rec->target_count; i++) {
        lighting_set_on_off(rec->targets[i].slave, rec->targets[i].addr, !any_on);
    }
}

static void do_dim_step(uint8_t fixture, uint8_t button, dim_dir_t dir) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    if (!rec) return;
    for (int i = 0; i < rec->target_count; i++) {
        if (rec->targets[i].slave == SLAVE_RELAY) continue; /* relays don't dim */
        lighting_dali_step(rec->targets[i].slave, rec->targets[i].addr,
                           dir == DIM_UP);
    }
}

static void process_raw(uint8_t fixture, uint8_t button,
                         btn_raw_type_t raw, uint32_t now_ms) {
    btn_fsm_state_t *fsm = &s_fsm[fixture][button];

    switch (fsm->state) {
    case BTN_STATE_IDLE:
        if (raw == BTN_RAW_PRESS) {
            fsm->press_time_ms = now_ms;
            fsm->dir_locked    = false;
            fsm->state         = BTN_STATE_PRESSED;
        }
        break;

    case BTN_STATE_PRESSED:
        if (raw == BTN_RAW_RELEASE) {
            emit_button_event(fixture, button, "press");
            do_toggle(fixture, button);
            fsm->state = BTN_STATE_IDLE;
        }
        break;

    case BTN_STATE_HOLDING:
        if (raw == BTN_RAW_RELEASE) {
            emit_button_event(fixture, button, "release");
            fsm->release_time_ms = now_ms;
            fsm->state           = BTN_STATE_RELEASED_AFTER_HOLD;
        }
        break;

    case BTN_STATE_RELEASED_AFTER_HOLD:
        if (raw == BTN_RAW_PRESS) {
            uint32_t gap = now_ms - fsm->release_time_ms;
            if (gap <= g_config.reversal_window_ms) {
                /* Direction reversal — skip re-applying long_press threshold */
                fsm->last_dir      = (fsm->last_dir == DIM_DOWN) ? DIM_UP : DIM_DOWN;
                fsm->dir_locked    = true;
                fsm->press_time_ms = now_ms;
                emit_button_event(fixture, button, "long_press_start");
                do_dim_step(fixture, button, fsm->last_dir);
                fsm->last_repeat_ms = now_ms;
                fsm->state          = BTN_STATE_HOLDING;
            } else {
                fsm->press_time_ms = now_ms;
                fsm->dir_locked    = false;
                fsm->state         = BTN_STATE_PRESSED;
            }
        }
        break;
    }
}

void button_fsm_tick(void) {
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());

    /* Drain raw event queue */
    btn_raw_event_t raw;
    while (queue_try_remove(&g_btn_raw_queue, &raw)) {
        if (raw.fixture < MAX_FIXTURES && raw.button < MAX_BUTTONS) {
            process_raw(raw.fixture, raw.button, raw.type, raw.time_ms);
        }
    }

    /* Timer-driven transitions */
    for (int f = 0; f < MAX_FIXTURES; f++) {
        for (int b = 0; b < MAX_BUTTONS; b++) {
            btn_fsm_state_t *fsm = &s_fsm[f][b];

            switch (fsm->state) {
            case BTN_STATE_PRESSED: {
                uint32_t held = now_ms - fsm->press_time_ms;
                if (held >= g_config.long_press_ms) {
                    if (!fsm->dir_locked) {
                        fsm->last_dir  = initial_dir(f, b);
                        fsm->dir_locked = true;
                    }
                    emit_button_event(f, b, "long_press_start");
                    do_dim_step(f, b, fsm->last_dir);
                    fsm->last_repeat_ms = now_ms;
                    fsm->state          = BTN_STATE_HOLDING;
                }
                break;
            }
            case BTN_STATE_HOLDING: {
                if (now_ms - fsm->last_repeat_ms >= g_config.dim_repeat_ms) {
                    emit_button_event(f, b, "long_press_repeat");
                    do_dim_step(f, b, fsm->last_dir);
                    fsm->last_repeat_ms = now_ms;
                }
                break;
            }
            case BTN_STATE_RELEASED_AFTER_HOLD: {
                if (now_ms - fsm->release_time_ms > g_config.reversal_window_ms) {
                    fsm->state = BTN_STATE_IDLE;
                }
                break;
            }
            default:
                break;
            }
        }
    }
}
