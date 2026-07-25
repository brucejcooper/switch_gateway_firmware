/*
 * button_fsm_emit.c — Button event emission to core 1.
 *
 * Implements emit_button_event() which is called from button_fsm.c at
 * each classified button event.  Kept in a separate file to avoid
 * a circular dependency (button_fsm.c -> serial, serial -> button_fsm.h).
 */

#include <string.h>
#include "intercore.h"
#include "log.h"

/*
 * Push a classified button event to core 1 (the MQTT bridge) as a typed
 * inter-core event; core 1 publishes it to Home Assistant.  Called from
 * button_fsm.c at each classified event:
 *   press             — physical button down (every interaction)
 *   release           — physical button up (every interaction)
 *   click             — completed short tap (down->up before long-press)
 *   long_press        — hold threshold crossed (or direction reversed)
 *   long_press_repeat — repeat tick while holding
 *
 * No JSON/USB output here — USB CDC is a human text CLI only; the button
 * event reaches HA via core 1, not the serial port.
 */
void emit_button_event(uint8_t fixture, uint8_t button, const char *action) {
    ic_event_t ev = { .type = IC_EVT_BUTTON };
    ev.button.fixture = fixture;
    ev.button.button  = button;
    ev.button.action  = strcmp(action, "release") == 0           ? IC_BTN_RELEASE
                      : strcmp(action, "click") == 0              ? IC_BTN_CLICK
                      : strcmp(action, "long_press") == 0         ? IC_BTN_LONG_PRESS
                      : strcmp(action, "long_press_repeat") == 0  ? IC_BTN_LONG_PRESS_REPEAT
                      : strcmp(action, "stuck") == 0              ? IC_BTN_STUCK
                      : strcmp(action, "unstuck") == 0            ? IC_BTN_UNSTUCK
                                                                  : IC_BTN_PRESS;
    /* Log every emitted event so the USB CLI mirrors exactly what reaches HA
     * (the repeat tick is high-frequency, so keep it at debug level). */
    if (ev.button.action == IC_BTN_LONG_PRESS_REPEAT)
        LOG_DBG("btn", "F%u.B%u %s", fixture, button, action);
    else
        LOG_INFO("btn", "F%u.B%u %s", fixture, button, action);
    intercore_push_event(&ev);
}
