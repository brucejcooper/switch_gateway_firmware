#include "buttongrid.pio.h"
#include "hardware/pio.h"
#include <stdbool.h>
#include <stdint.h>
#include <pico/time.h>
#include "button_actions.h"
#include "lualoop.h"


#define SER_PIN 6
#define CLK_PIN 7
#define COL_PIN_BASE 8

#define INITIAL_LONGPRESS_DELAY 800
#define LONGPRESS_REPEAT_PERIOD 333
#define REPRESS_TIMEOUT 250


#define NUM_DATA_VALUES (24/4)

static PIO pio;
static uint sm;
static uint32_t values[NUM_DATA_VALUES] = {0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F};


// ISR - reads it and determines if an action (press or release) has occurred.
static void sample_available() {
    // Goes up each time we read a sample, so that we can tell which index we are working with. 
    static unsigned index = 0;
    static bool led_on;
    uint32_t tmp;


    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    led_on = !led_on;

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t reading = pio_sm_get(pio, sm);
        uint32_t changed = reading ^ values[index];
        // printf(",%d = 0x%08x", index, reading);
        // if (index == 5) {
        //     printf("\n");
        // }

        if (changed) {
            values[index] = reading;
            for (unsigned bit = 0; bit < 32; bit++) {
                if (changed & 0x01) {
                    lua_schedule_btn_change(index * 32 + bit, (reading & 0x01) == 0);
                }
                changed >>= 1;
                reading >>= 1;
            }
        }
        index = (index + 1) % NUM_DATA_VALUES;
    }
}


void buttongrid_init() {
    unsigned offset;

    bool success = pio_claim_free_sm_and_add_program(&buttongrid_program, &pio, &sm, &offset);
    hard_assert(success);
    unsigned pioIrq = pio == pio0 ? PIO0_IRQ_0 : pio == pio1 ? PIO1_IRQ_0 : PIO2_IRQ_0;
    irq_set_exclusive_handler(pioIrq, sample_available);
    irq_set_enabled(pioIrq, true);

    buttongrid_initpio(pio, sm, offset, SER_PIN, CLK_PIN, COL_PIN_BASE);
    // Discard one set of readings, as on power on they may be invalid.  This should take approximately 6*5ms = 30ms.
    for (int i = 0; i < NUM_DATA_VALUES; i++) {
        pio_sm_get_blocking(pio, sm);
    }
    // Clear any existing IRQ flags, then enable us for future data
    pio_interrupt_clear(pio, sm);
    pio_set_irq0_source_enabled(pio, pis_sm0_rx_fifo_not_empty, true);
}
