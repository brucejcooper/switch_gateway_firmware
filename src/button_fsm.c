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
#include "mapping.h"
#include "state_cache.h"
#include "config.h"
#include "log.h"
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
    btn_state_t     state;
    absolute_time_t press_time;            /* for "held=X ms" log */
    absolute_time_t long_press_deadline;   /* PRESSED -> HOLDING fires here */
    absolute_time_t next_repeat_at;        /* next long_press_repeat */
    absolute_time_t reversal_window_end;   /* RELEASED_AFTER_HOLD timeout */
    dim_dir_t       last_dir;
    bool            dir_locked;
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
        if (TARGET_IS_GROUP(rec->targets[i])) {
            lighting_group_set_on_off(rec->targets[i].slave,
                                       TARGET_GROUP_NUM(rec->targets[i]),
                                       !any_on);
        } else {
            lighting_set_on_off(rec->targets[i].slave,
                                 TARGET_BALLAST_ADDR(rec->targets[i]),
                                 !any_on);
        }
    }
}

static void do_dim_step(uint8_t fixture, uint8_t button, dim_dir_t dir) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    if (!rec) return;
    for (int i = 0; i < rec->target_count; i++) {
        if (slave_kind(rec->targets[i].slave) == SLAVE_KIND_RELAY) continue; /* relays don't dim */
        if (TARGET_IS_GROUP(rec->targets[i])) {
            lighting_group_step(rec->targets[i].slave,
                                 TARGET_GROUP_NUM(rec->targets[i]),
                                 dir == DIM_UP);
        } else {
            lighting_dali_step(rec->targets[i].slave,
                                TARGET_BALLAST_ADDR(rec->targets[i]),
                                dir == DIM_UP);
        }
    }
}

static void process_raw(uint8_t fixture, uint8_t button,
                         btn_raw_type_t raw, absolute_time_t event_time) {
    btn_fsm_state_t *fsm = &s_fsm[fixture][button];

    switch (fsm->state) {
    case BTN_STATE_IDLE:
        if (raw == BTN_RAW_PRESS) {
            LOG_INFO("btn", "F%u.B%u press", fixture, button);
            fsm->press_time          = event_time;
            fsm->long_press_deadline = delayed_by_ms(event_time,
                                                     g_config.long_press_ms);
            fsm->dir_locked          = false;
            fsm->state               = BTN_STATE_PRESSED;
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
            int64_t held_us = absolute_time_diff_us(fsm->press_time, event_time);
            LOG_INFO("btn", "F%u.B%u release held=%lu ms",
                     fixture, button, (unsigned long)(held_us / 1000));
            emit_button_event(fixture, button, "release");
            fsm->reversal_window_end = delayed_by_ms(event_time,
                                                     g_config.reversal_window_ms);
            fsm->state               = BTN_STATE_RELEASED_AFTER_HOLD;
        }
        break;

    case BTN_STATE_RELEASED_AFTER_HOLD:
        if (raw == BTN_RAW_PRESS) {
            /* still in reversal window iff event_time hasn't reached the end */
            bool in_window = absolute_time_diff_us(event_time,
                                                   fsm->reversal_window_end) > 0;
            if (in_window) {
                /* Direction reversal — skip re-applying long_press threshold */
                fsm->last_dir   = (fsm->last_dir == DIM_DOWN) ? DIM_UP : DIM_DOWN;
                fsm->dir_locked = true;
                fsm->press_time = event_time;
                LOG_INFO("btn", "F%u.B%u long_press_start dir=%s",
                         fixture, button,
                         fsm->last_dir == DIM_UP ? "up" : "down");
                emit_button_event(fixture, button, "long_press_start");
                do_dim_step(fixture, button, fsm->last_dir);
                fsm->next_repeat_at = delayed_by_ms(event_time,
                                                    g_config.dim_repeat_ms);
                fsm->state          = BTN_STATE_HOLDING;
            } else {
                LOG_INFO("btn", "F%u.B%u press", fixture, button);
                fsm->press_time          = event_time;
                fsm->long_press_deadline = delayed_by_ms(event_time,
                                                         g_config.long_press_ms);
                fsm->dir_locked          = false;
                fsm->state               = BTN_STATE_PRESSED;
            }
        }
        break;
    }
}

void button_fsm_tick(void) {
    /* Drain raw event queue */
    btn_raw_event_t raw;
    while (queue_try_remove(&g_btn_raw_queue, &raw)) {
        if (raw.fixture < MAX_FIXTURES && raw.button < MAX_BUTTONS) {
            process_raw(raw.fixture, raw.button, raw.type, raw.time);
        }
    }

    /* Timer-driven transitions */
    for (int f = 0; f < MAX_FIXTURES; f++) {
        for (int b = 0; b < MAX_BUTTONS; b++) {
            btn_fsm_state_t *fsm = &s_fsm[f][b];

            switch (fsm->state) {
            case BTN_STATE_PRESSED:
                if (time_reached(fsm->long_press_deadline)) {
                    if (!fsm->dir_locked) {
                        fsm->last_dir   = initial_dir(f, b);
                        fsm->dir_locked = true;
                    }
                    LOG_INFO("btn", "F%u.B%u long_press_start dir=%s",
                             (unsigned)f, (unsigned)b,
                             fsm->last_dir == DIM_UP ? "up" : "down");
                    emit_button_event(f, b, "long_press_start");
                    do_dim_step(f, b, fsm->last_dir);
                    fsm->next_repeat_at = make_timeout_time_ms(g_config.dim_repeat_ms);
                    fsm->state          = BTN_STATE_HOLDING;
                }
                break;

            case BTN_STATE_HOLDING:
                if (time_reached(fsm->next_repeat_at)) {
                    LOG_DBG("btn", "F%u.B%u repeat", (unsigned)f, (unsigned)b);
                    emit_button_event(f, b, "long_press_repeat");
                    do_dim_step(f, b, fsm->last_dir);
                    fsm->next_repeat_at = make_timeout_time_ms(g_config.dim_repeat_ms);
                }
                break;

            case BTN_STATE_RELEASED_AFTER_HOLD:
                if (time_reached(fsm->reversal_window_end)) {
                    fsm->state = BTN_STATE_IDLE;
                }
                break;

            default:
                break;
            }
        }
    }
}
