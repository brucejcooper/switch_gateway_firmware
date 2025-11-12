#include "buttongrid.pio.h"
#include "hardware/pio.h"
#include "pico-sdk/src/extra_doxygen/intctrl.h"
#include "projdefs.h"
#include <stdbool.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <stdio.h>
#include <queue.h>
#include "modbus.h"
#include <timers.h>
#include <pico/time.h>
#include "entity_status.h"



typedef struct {
    char name[8];
    bool is_pressed;
    TimerHandle_t timer;
    int velocity;
    action_ctx_t binding;
} button_state_t;



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
static QueueHandle_t action_queue;


#define NUM_BUTTONS (24*8)

static button_state_t button_state[NUM_BUTTONS];


bool button_is_pressed(unsigned index) {
    if (index >= NUM_BUTTONS) {
        return false;
    }
    return button_state[index].is_pressed;
}


// ISR - reads it and determines if an action (press or release) has occurred.
static void sample_available() {
    // Goes up each time we read a sample, so that we can tell which index we are working with. 
    static unsigned index = 0;
    static bool led_on;
    BaseType_t higherPriorityWoken = pdFALSE;
    uint32_t tmp;

    gpio_put(PICO_DEFAULT_LED_PIN, led_on);
    led_on = !led_on;

    while (!pio_sm_is_rx_fifo_empty(pio, sm)) {
        uint32_t reading = pio_sm_get(pio, sm);
        uint32_t changed = reading ^ values[index];
        if (changed) {
            values[index] = reading;
            for (unsigned bit = 0; bit < 32; bit++) {
                if (changed & 0x01) {
                    // Low bit indicates whether it is a press or a release.
                    tmp = index << 5 | bit << 1 | (reading & 0x01);
                    xQueueSendFromISR(action_queue, &tmp, &higherPriorityWoken);
                    if (higherPriorityWoken) {
                        portYIELD_FROM_ISR(higherPriorityWoken);
                    }
                }
                changed >>= 1;
                reading >>= 1;
            }
        }
        index = (index + 1) % NUM_DATA_VALUES;
    }
}



static void post_action_cb(int status, uint8_t *buf, action_ctx_t binding) {
    printf("Post action, binding is %d %d\n", binding.device, binding.address);
    // After any button change action we need to update the brightness/coil status again, if there is a binding (which there should be because we did something...)
    entity_status_update(binding.device, binding.address, false);
}


void timer_cb(TimerHandle_t timer) {
    button_state_t *btn = pvTimerGetTimerID(timer);
    int idx = (int) (btn-button_state);
    uint32_t tmp;
    // printf("BTN %d\n", idx);

    if (btn->is_pressed) {
        if (btn->velocity == 0) {
            // Its the first long press... We default to dimming
            btn->velocity = -1;
        }
        if (btn->binding.device > 0 && entity_status_is_device_dimmable(btn->binding.device)) {
            modbus_fade(3, 0, btn->velocity, (modbus_action_cb_t) post_action_cb, btn->binding);
        }
        xTimerChangePeriod(timer, LONGPRESS_REPEAT_PERIOD, 0);
    } else {
        // Button has remained released for long enough for a press to no longer count as a re-press
        // Reset our button to normal state.
        xTimerStop(timer, 0);
        btn->velocity = 0;
    }
 }

void buttongrid_task(void *data) {
    unsigned offset;

    button_state_t *btn = button_state;
    for (unsigned i = 0; i< NUM_BUTTONS; i++, btn++) {
        sprintf(btn->name, "btn%03d", i);
        btn->binding.device = 3;
        btn->binding.address = 0;
        btn->velocity = 0;
        btn->is_pressed = false;
        btn->timer = xTimerCreate(btn->name, INITIAL_LONGPRESS_DELAY, true, btn, timer_cb);
    }

    action_queue = xQueueCreate(20, sizeof(uint32_t));

    bool success = pio_claim_free_sm_and_add_program(&buttongrid_program, &pio, &sm, &offset);
    hard_assert(success);

    unsigned pioIrq = pio == pio0 ? PIO0_IRQ_0 : pio == pio1 ? PIO1_IRQ_0 : PIO2_IRQ_0;

    irq_set_exclusive_handler(pioIrq, sample_available);
    irq_set_enabled(pioIrq, true);

    buttongrid_init(pio, sm, offset, SER_PIN, CLK_PIN, COL_PIN_BASE);

    // Discard one set of readings, as on power on they may be invalid.  This should take approximately 6*5ms = 30ms.
    for (int i = 0; i < NUM_DATA_VALUES; i++) {
        pio_sm_get_blocking(pio, sm);
    }
    // Clear any existing IRQ flags, then enable us for future data
    pio_interrupt_clear(pio, sm);
    pio_set_irq0_source_enabled(pio, pis_sm0_rx_fifo_not_empty, true);

    uint32_t cmd;
    while (true) {
        if( xQueueReceive( action_queue, &cmd,  portMAX_DELAY) == pdPASS ) {
            int btnIdx = cmd >> 1;
            button_state_t *btn = button_state + btnIdx;

            bool pressed = (cmd & 0x01) == 0;
            btn->is_pressed = pressed;
            entity_status_print(0, btnIdx);

            if (entity_status_is_device_dimmable(btn->binding.device)) {
                if (pressed) {
                    // printf("Button %d pressed\n", btnIdx);

                    if (btn->velocity) {
                        // Its a re-press - switch velocity
                        btn->velocity *= -1;
                    } else {
                        // normal press.
                        btn->velocity = 0;
                        // Initial timeout is a bit longer than repeat
                        xTimerChangePeriod(btn->timer, INITIAL_LONGPRESS_DELAY, portMAX_DELAY);
                    }
                } else {
                    printf("Button %d released with binding %d-%d\n", btnIdx, btn->binding.device, btn->binding.address);
                    xTimerChangePeriod(btn->timer, REPRESS_TIMEOUT, 0);
                    if (btn->velocity == 0 && btn->binding.device) {
                        printf("Button %d clicked\n", btnIdx);
                        modbus_toggle(3, 0, (modbus_action_cb_t) post_action_cb, btn->binding);
                    }
                }
                xTimerReset(btn->timer, 0);
            } else {
                // Just a relay.  No need for any timers.  Toggle as soon as you get a press.
                if (pressed) {
                    modbus_toggle(3, 0, (modbus_action_cb_t) post_action_cb, btn->binding);
                }
            }
        }
    }
    vTaskDelete(NULL);
}