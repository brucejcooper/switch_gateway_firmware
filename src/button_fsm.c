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

queue_t g_btn_raw_queue;

typedef enum { DIM_UP, DIM_DOWN } dim_dir_t;

/* How long a click will wait for the press-time any-on probe before giving up
 * and assuming the DALI targets are off. The probe starts at press, so by
 * release it has usually already resolved; this only pads slow/queued cases. */
#define CLICK_PROBE_GRACE_MS 150

/* A hold longer than this is a physically stuck button, not a person dimming
 * (a full min→max dim sweep takes a few seconds). The FSM stops issuing dim
 * commands, warns, and reports the fixture stuck to HA until release. */
#define BTN_STUCK_MS 30000

typedef enum {
    BTN_STATE_IDLE,
    BTN_STATE_PRESSED,
    BTN_STATE_HOLDING,
    BTN_STATE_RELEASED_AFTER_HOLD,
    BTN_STATE_STUCK,           /* held past BTN_STUCK_MS — waiting for release */
} btn_state_t;

typedef struct {
    btn_state_t     state;
    absolute_time_t press_time;            /* for "held=X ms" log */
    absolute_time_t long_press_deadline;   /* PRESSED -> HOLDING fires here */
    absolute_time_t next_repeat_at;        /* next long_press_repeat */
    absolute_time_t reversal_window_end;   /* RELEASED_AFTER_HOLD timeout */
    dim_dir_t       last_dir;
    bool            dir_locked;
    bool            click_pending;         /* click released, waiting on probe */
    absolute_time_t click_deadline;        /* stop waiting and assume off here */
} btn_fsm_state_t;

static btn_fsm_state_t s_fsm[MAX_FIXTURES][MAX_BUTTONS];

void button_fsm_init(void) {
    queue_init(&g_btn_raw_queue, sizeof(btn_raw_event_t), BTN_RAW_QUEUE_DEPTH);
    memset(s_fsm, 0, sizeof(s_fsm));
}

/* The cached "any on?" for a binding's DALI targets — the fallback when the
 * press-time probe does not resolve. Cached arc levels are what the controller
 * last measured (verification reads, the background sweep), so this is right
 * far more often than a blind guess; it is only wrong if something changed a
 * ballast behind the controller's back since the last read. */
static bool cached_dali_any_on(const mapping_record_t *rec) {
    for (int i = 0; i < rec->target_count; i++) {
        const mapping_target_t *t = &rec->targets[i];
        if (slave_kind(t->slave) != SLAVE_KIND_DALI) continue;
        int idx = state_cache_slave_idx(t->slave);
        if (idx < 0) continue;
        if (TARGET_IS_GROUP(*t)) {
            if (state_cache_dali_group_any_on((uint8_t)idx, TARGET_GROUP_NUM(*t)))
                return true;
        } else {
            const dali_ballast_state_t *b =
                state_cache_get_dali((uint8_t)idx, TARGET_BALLAST_ADDR(*t));
            if (b && b->present && b->arc_level > 0) return true;
        }
    }
    return false;
}

/* Decide the binding's current "any on?" for a toggle / dim-direction call.
 * DALI targets use the press-time any-on probe (ground truth straight from the
 * ballasts, immune to the group-mask cache going stale across gateway
 * reboots). If the probe never resolved — gateway slow, queue congestion,
 * probe slot taken — fall back to the cached levels and WARN so the fallback
 * is visible. (This used to assume "off", so an unresolved probe on a lit
 * group turned every press into "on"/dim-up and the lights could not be
 * switched off from the wall — house, 2026-09-30.) Relay targets always use
 * the cache: the controller is the relays' only writer. */
static bool resolve_any_on(uint8_t fixture, uint8_t button, const char **src) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    bool relay_on = false, has_dali = false;
    if (rec) {
        for (int i = 0; i < rec->target_count; i++) {
            const mapping_target_t *t = &rec->targets[i];
            if (slave_kind(t->slave) == SLAVE_KIND_RELAY)
                relay_on = relay_on ||
                           state_cache_get_relay_on(TARGET_BALLAST_ADDR(*t));
            else if (slave_kind(t->slave) == SLAVE_KIND_DALI)
                has_dali = true;
        }
    }
    if (!has_dali) { *src = "cache"; return relay_on; }
    switch (lighting_any_on_probe_get(fixture, button)) {
    case ANY_ON_PROBE_ON:  *src = "probe"; return true;
    case ANY_ON_PROBE_OFF: *src = "probe"; return relay_on;
    default: {
        bool cached = cached_dali_any_on(rec);
        LOG_WARN("btn", "F%u.B%u any-on probe unresolved — using cache (%s)",
                 fixture, button, cached ? "on" : "off");
        *src = "cache-fallback";
        return relay_on || cached;
    }
    }
}

static dim_dir_t initial_dir(uint8_t fixture, uint8_t button) {
    const char *src;
    return resolve_any_on(fixture, button, &src) ? DIM_DOWN : DIM_UP;
}

void button_fsm_inject(uint8_t fixture, uint8_t button, btn_raw_type_t type) {
    btn_raw_event_t e = { .type = type, .fixture = fixture, .button = button,
                          .time = get_absolute_time() };
    queue_try_add(&g_btn_raw_queue, &e);
}

static void do_toggle(uint8_t fixture, uint8_t button) {
    const mapping_record_t *rec = mapping_lookup(fixture, button);
    if (!rec) return;
    const char *src;
    bool any_on = resolve_any_on(fixture, button, &src);
    LOG_INFO("btn", "toggle F%u.B%u any_on=%d (%s) -> %s",
             fixture, button, any_on, src, any_on ? "off" : "on");
    for (int i = 0; i < rec->target_count; i++) {
        const mapping_target_t *t = &rec->targets[i];
        if (TARGET_IS_GROUP(*t)) {
            lighting_button_group_on_off(t->slave, TARGET_GROUP_NUM(*t), !any_on);
        } else if (slave_kind(t->slave) == SLAVE_KIND_RELAY) {
            /* a relay target is a coil, not a DALI ballast — addr is the channel */
            lighting_button_relay(TARGET_BALLAST_ADDR(*t), !any_on);
        } else {
            lighting_button_on_off(t->slave, TARGET_BALLAST_ADDR(*t), !any_on);
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
            /* a previous click still waiting on its probe? dispatch it before
             * this press restarts the probe slot underneath it */
            if (fsm->click_pending) {
                fsm->click_pending = false;
                do_toggle(fixture, button);
            }
            emit_button_event(fixture, button, "press");   /* physical down (logged in emit) */
            fsm->press_time          = event_time;
            fsm->long_press_deadline = delayed_by_ms(event_time,
                                                     g_config.long_press_ms);
            fsm->dir_locked          = false;
            lighting_any_on_probe_start(fixture, button);
            fsm->state               = BTN_STATE_PRESSED;
        }
        break;

    case BTN_STATE_PRESSED:
        if (raw == BTN_RAW_RELEASE) {
            /* short tap: down->up before the long-press threshold = a click */
            emit_button_event(fixture, button, "click");
            emit_button_event(fixture, button, "release");
            if (lighting_any_on_probe_get(fixture, button) == ANY_ON_PROBE_PENDING) {
                /* answer still in flight — give it a short grace before acting */
                fsm->click_pending  = true;
                fsm->click_deadline = delayed_by_ms(event_time,
                                                    CLICK_PROBE_GRACE_MS);
            } else {
                do_toggle(fixture, button);
            }
            fsm->state = BTN_STATE_IDLE;
        }
        break;

    case BTN_STATE_HOLDING:
        if (raw == BTN_RAW_RELEASE) {
            int64_t held_us = absolute_time_diff_us(fsm->press_time, event_time);
            LOG_DBG("btn", "F%u.B%u held=%lu ms",
                    fixture, button, (unsigned long)(held_us / 1000));
            emit_button_event(fixture, button, "release");
            fsm->reversal_window_end = delayed_by_ms(event_time,
                                                     g_config.reversal_window_ms);
            fsm->state               = BTN_STATE_RELEASED_AFTER_HOLD;
        }
        break;

    case BTN_STATE_STUCK:
        if (raw == BTN_RAW_RELEASE) {
            int64_t held_us = absolute_time_diff_us(fsm->press_time, event_time);
            LOG_INFO("btn", "F%u.B%u stuck button released after %lu ms",
                     fixture, button, (unsigned long)(held_us / 1000));
            emit_button_event(fixture, button, "unstuck");
            emit_button_event(fixture, button, "release");
            /* no click/toggle — the hold was never a user intent */
            fsm->state = BTN_STATE_IDLE;
        }
        break;

    case BTN_STATE_RELEASED_AFTER_HOLD:
        if (raw == BTN_RAW_PRESS) {
            emit_button_event(fixture, button, "press");   /* physical down */
            /* still in reversal window iff event_time hasn't reached the end */
            bool in_window = absolute_time_diff_us(event_time,
                                                   fsm->reversal_window_end) > 0;
            if (in_window) {
                /* Direction reversal — skip re-applying long_press threshold */
                fsm->last_dir   = (fsm->last_dir == DIM_DOWN) ? DIM_UP : DIM_DOWN;
                fsm->dir_locked = true;
                fsm->press_time = event_time;
                LOG_DBG("btn", "F%u.B%u dir=%s", fixture, button,
                        fsm->last_dir == DIM_UP ? "up" : "down");
                emit_button_event(fixture, button, "long_press");
                do_dim_step(fixture, button, fsm->last_dir);
                fsm->next_repeat_at = delayed_by_ms(event_time,
                                                    g_config.dim_repeat_ms);
                fsm->state          = BTN_STATE_HOLDING;
            } else {
                fsm->press_time          = event_time;
                fsm->long_press_deadline = delayed_by_ms(event_time,
                                                         g_config.long_press_ms);
                fsm->dir_locked          = false;
                lighting_any_on_probe_start(fixture, button);
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

            /* deferred click: act as soon as the probe resolves, or when the
             * grace window closes (resolve_any_on then assumes off + WARNs) */
            if (fsm->click_pending) {
                if (lighting_any_on_probe_get((uint8_t)f, (uint8_t)b)
                        != ANY_ON_PROBE_PENDING ||
                    time_reached(fsm->click_deadline)) {
                    fsm->click_pending = false;
                    do_toggle((uint8_t)f, (uint8_t)b);
                }
            }

            switch (fsm->state) {
            case BTN_STATE_PRESSED:
                if (time_reached(fsm->long_press_deadline)) {
                    if (!fsm->dir_locked) {
                        fsm->last_dir   = initial_dir(f, b);
                        fsm->dir_locked = true;
                    }
                    LOG_DBG("btn", "F%u.B%u dir=%s", (unsigned)f, (unsigned)b,
                            fsm->last_dir == DIM_UP ? "up" : "down");
                    emit_button_event(f, b, "long_press");
                    do_dim_step(f, b, fsm->last_dir);
                    fsm->next_repeat_at = make_timeout_time_ms(g_config.dim_repeat_ms);
                    fsm->state          = BTN_STATE_HOLDING;
                }
                break;

            case BTN_STATE_HOLDING:
                if (absolute_time_diff_us(fsm->press_time, get_absolute_time())
                        >= (int64_t)BTN_STUCK_MS * 1000) {
                    LOG_WARN("btn",
                             "F%u.B%u held >%u ms — button stuck, suppressing "
                             "commands until release", (unsigned)f, (unsigned)b,
                             (unsigned)BTN_STUCK_MS);
                    emit_button_event((uint8_t)f, (uint8_t)b, "stuck");
                    fsm->state = BTN_STATE_STUCK;
                    break;
                }
                if (time_reached(fsm->next_repeat_at)) {
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
