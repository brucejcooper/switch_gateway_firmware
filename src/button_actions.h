#pragma once
#include <stdint.h>

/* Initialise the PIO button grid scanner.
 * On press/release events, pushes btn_raw_event_t into g_btn_raw_queue
 * (defined in button_fsm.h) instead of calling lua_schedule_btn_change(). */
void buttongrid_init(void);

/* Count of raw button events dropped because the FSM queue was full — a lost
 * press/release. Counted (not logged) because this is the PIO scan path.
 * Should stay 0; a non-zero value means the FSM drain is being starved. */
uint32_t button_raw_drop_count(void);
