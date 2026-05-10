/*
 * button_actions.c — PIO button grid scanner.
 *
 * The PIO program and ISR are identical to the original.  The only change:
 * lua_schedule_btn_change() is replaced with a direct push to g_btn_raw_queue,
 * which button_fsm_tick() drains on each main loop iteration.
 */

#include "buttongrid.pio.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "button_actions.h"
#include "button_fsm.h"       /* for g_btn_raw_queue and btn_raw_event_t */
#include "mapping.h"          /* fix #2: MAX_FIXTURES, MAX_BUTTONS */
#include <stdbool.h>
#include <stdint.h>

#define SER_PIN       6
#define CLK_PIN       7
#define COL_PIN_BASE  8

/* 24 fixtures, each with up to 7 buttons.  PIO delivers 24 32-bit words
 * (one per column group of 8 rows), 6 words × 32 bits = 192 button slots.
 * Only the lower 7 bits of each byte are wired to buttons. */
#define NUM_DATA_VALUES (24 / 4)

static PIO      pio;
static uint     sm;

/* Last known state — all bits start at 1 (released, pulled high) */
static uint32_t values[NUM_DATA_VALUES] = {
    0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F,
    0x7F7F7F7F, 0x7F7F7F7F, 0x7F7F7F7F,
};

/* PIO ISR — identical to original except for the event sink */
static void sample_available(void) {
    static unsigned index   = 0;
    static bool     led_on  = false;

    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    led_on = !led_on;

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t reading = pio_sm_get(pio, sm);
        uint32_t changed = reading ^ values[index];

        if (changed) {
            values[index] = reading;
            for (unsigned bit = 0; bit < 32; bit++) {
                if (changed & 0x01) {
                    /* Convert flat bit index to (fixture, button) */
                    unsigned flat    = index * 32 + bit;
                    uint8_t  fixture = flat / 8;   /* 8 column inputs per fixture */
                    uint8_t  button  = flat % 8;
                    bool     pressed = (reading & 0x01) == 0; /* active low */

                    if (fixture < MAX_FIXTURES && button < MAX_BUTTONS) {
                        btn_raw_event_t evt = {
                            .type    = pressed ? BTN_RAW_PRESS : BTN_RAW_RELEASE,
                            .fixture = fixture,
                            .button  = button,
                            .time_ms = to_ms_since_boot(get_absolute_time()),
                        };
                        /* Non-blocking push — drop if queue full.
                         * The FSM queue is deep enough (64 slots) that this
                         * should never happen at 200 scans/sec. */
                        queue_try_add(&g_btn_raw_queue, &evt);
                    }
                }
                changed >>= 1;
                reading >>= 1;
            }
        }
        index = (index + 1) % NUM_DATA_VALUES;
    }
}

void buttongrid_init(void) {
    unsigned offset;
    bool success = pio_claim_free_sm_and_add_program(
        &buttongrid_program, &pio, &sm, &offset);
    hard_assert(success);

    unsigned pioIrq = (pio == pio0) ? PIO0_IRQ_0
                    : (pio == pio1) ? PIO1_IRQ_0
                                    : PIO2_IRQ_0;
    irq_set_exclusive_handler(pioIrq, sample_available);
    irq_set_enabled(pioIrq, true);

    buttongrid_initpio(pio, sm, offset, SER_PIN, CLK_PIN, COL_PIN_BASE);

    /* Discard first scan cycle — readings may be invalid on power-on */
    for (int i = 0; i < NUM_DATA_VALUES; i++) {
        pio_sm_get_blocking(pio, sm);
    }

    pio_interrupt_clear(pio, sm);
    pio_set_irq0_source_enabled(pio, pis_sm0_rx_fifo_not_empty, true);
}
