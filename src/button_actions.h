#pragma once
#include <stdint.h>

/* Initialise the PIO button grid scanner.
 * On press/release events, pushes btn_raw_event_t into g_btn_raw_queue
 * (defined in button_fsm.h) instead of calling lua_schedule_btn_change(). */
void buttongrid_init(void);

/* Raw button events dropped on a full core-0 queue. A climbing count means
 * presses are being lost; shown by `busload`. */
uint32_t button_raw_drop_count(void);
