#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <hardware/timer.h>
#include "modbus.h"
// #include "entity_status.h"
// #include "dali_provision.h"
#include <hardware/flash.h>
#include <lua.h>
#include <lauxlib.h>
#include <lualib.h>
#include <sys/types.h>
#include <dirent.h>
#include <hardware/watchdog.h>
#include "lualoop.h"
#include "filesystem/vfs.h"
#include "button_actions.h"
#include <pico/time.h>


extern void buttongrid_task(void *data);


// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}


#define MAX_LINE_LEN 256

int main() {
    stdio_init_all();
    fs_init();
    alarm_pool_init_default();
    int rc = pico_led_init();
    hard_assert(rc == PICO_OK);




    busy_wait_ms(2500);

    modbus_init();      // Init script might call modbus, so it gets initialised first
    lua_init();         // Then lua.  Anything that modbus depends on will be initialised before the init script gets called
    buttongrid_init();  // Button grid goes last, as it injects commands into the lua state. 

    while (true) {
        modbus_poll();
        lua_poll();
    }    
    panic_unsupported();
}