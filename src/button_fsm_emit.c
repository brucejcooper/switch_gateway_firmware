/*
 * button_fsm_emit.c — Button event emission to serial protocol.
 *
 * Implements emit_button_event() which is called from button_fsm.c at
 * each classified button event.  Kept in a separate file to avoid
 * a circular dependency (button_fsm.c -> serial, serial -> button_fsm.h).
 *
 * Integration: add to CMakeLists.txt add_executable() block.
 * In button_fsm.c, declare: static void emit_button_event(uint8_t, uint8_t, const char *);
 * Or forward-declare in button_fsm.h and include this file's header.
 */

#include <stdio.h>
#include <string.h>
#include <pico/time.h>
#include "intercore.h"

/*
 * Emit a button event as a JSON line to USB CDC stdout.
 * Called from button_fsm.c at each classified event:
 *   press             — short press released
 *   release           — released after a hold
 *   long_press_start  — hold threshold crossed (or direction reversed)
 *   long_press_repeat — repeat tick while holding
 *
 * Uses printf() directly — safe because everything runs on core 0
 * and all serial output uses printf/stdio_usb.
 */
void emit_button_event(uint8_t fixture, uint8_t button, const char *action) {
    printf("{\"t\":\"button\",\"ts\":%lu,"
           "\"d\":{\"fixture\":%u,\"button\":%u},"
           "\"v\":{\"action\":\"%s\"}}\n",
           (unsigned long)to_ms_since_boot(get_absolute_time()),
           fixture, button, action);

    /* Forward to core 1 (the MQTT bridge) as a typed event. */
    ic_event_t ev = { .type = IC_EVT_BUTTON };
    ev.button.fixture = fixture;
    ev.button.button  = button;
    ev.button.action  = strcmp(action, "release") == 0           ? IC_BTN_RELEASE
                      : strcmp(action, "long_press_start") == 0   ? IC_BTN_LONG_PRESS_START
                      : strcmp(action, "long_press_repeat") == 0  ? IC_BTN_LONG_PRESS_REPEAT
                                                                  : IC_BTN_PRESS;
    intercore_push_event(&ev);
}
