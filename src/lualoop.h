#ifndef __lualoop_h__
#define __lualoop_h__

// #include "lua.h"
#include <stdbool.h>

void lua_init();
void lua_poll();
// void scheduleCoro(lua_State *thread, int nargs);
void lua_schedule_btn_change(int btnidx, bool isPressed);

#endif