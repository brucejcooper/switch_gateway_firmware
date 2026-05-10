#pragma once

/* Initialise the PIO button grid scanner.
 * On press/release events, pushes btn_raw_event_t into g_btn_raw_queue
 * (defined in button_fsm.h) instead of calling lua_schedule_btn_change(). */
void buttongrid_init(void);
