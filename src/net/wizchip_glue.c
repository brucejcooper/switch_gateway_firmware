/*
 * wizchip_glue.c — RP2040 SPI0 glue for the W5100S (ioLibrary_Driver).
 *
 * Pins (WIZnet W5100S-EVB-Pico, from the board header): SCK GP18, MOSI GP19,
 * MISO GP16, CSn GP17, RSTn GP20, INTn GP21. SPI0 is owned solely by core 1.
 */

#include "wizchip_glue.h"
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "wizchip_conf.h"

#define WIZ_SPI       spi0
#define WIZ_SPI_HZ    (10u * 1000u * 1000u)   /* conservative bring-up clock */

#ifndef PICO_DEFAULT_SPI_SCK_PIN
#define PICO_DEFAULT_SPI_SCK_PIN 18
#endif
#ifndef PICO_DEFAULT_SPI_TX_PIN
#define PICO_DEFAULT_SPI_TX_PIN 19
#endif
#ifndef PICO_DEFAULT_SPI_RX_PIN
#define PICO_DEFAULT_SPI_RX_PIN 16
#endif
#ifndef PICO_DEFAULT_SPI_CSN_PIN
#define PICO_DEFAULT_SPI_CSN_PIN 17
#endif
#ifndef W5100S_EVB_PICO_RSTN_PIN
#define W5100S_EVB_PICO_RSTN_PIN 20
#endif
#ifndef W5100S_EVB_PICO_INTN_PIN
#define W5100S_EVB_PICO_INTN_PIN 21
#endif

#define PIN_SCK   PICO_DEFAULT_SPI_SCK_PIN
#define PIN_MOSI  PICO_DEFAULT_SPI_TX_PIN
#define PIN_MISO  PICO_DEFAULT_SPI_RX_PIN
#define PIN_CS    PICO_DEFAULT_SPI_CSN_PIN
#define PIN_RST   W5100S_EVB_PICO_RSTN_PIN
#define PIN_INT   W5100S_EVB_PICO_INTN_PIN

/* Critical section: disable interrupts around each W5100S access so a register
 * or socket-buffer burst can't be interrupted mid-transaction by core 1's
 * flash-lockout (SIO) IRQ — which would park core 1 with CS asserted while
 * core 0 writes flash. Each access is microseconds (a full 2 KB burst ~0.5 ms),
 * so this only briefly defers a flash write, never blocks it. */
static uint32_t s_irq_save;
static void cris_enter(void) { s_irq_save = save_and_disable_interrupts(); }
static void cris_exit(void)  { restore_interrupts(s_irq_save); }

static void cs_select(void)   { gpio_put(PIN_CS, 0); }
static void cs_deselect(void) { gpio_put(PIN_CS, 1); }

static uint8_t spi_read_byte(void) {
    uint8_t b;
    spi_read_blocking(WIZ_SPI, 0xFF, &b, 1);
    return b;
}
static void spi_write_byte(uint8_t b) {
    spi_write_blocking(WIZ_SPI, &b, 1);
}
static void spi_read_burst(uint8_t *buf, uint16_t len) {
    spi_read_blocking(WIZ_SPI, 0xFF, buf, len);
}
static void spi_write_burst(uint8_t *buf, uint16_t len) {
    spi_write_blocking(WIZ_SPI, buf, len);
}

void wizchip_glue_init(void) {
    spi_init(WIZ_SPI, WIZ_SPI_HZ);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);  gpio_set_dir(PIN_CS,  GPIO_OUT); gpio_put(PIN_CS,  1);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT); gpio_put(PIN_RST, 1);
    gpio_init(PIN_INT); gpio_set_dir(PIN_INT, GPIO_IN);  gpio_pull_up(PIN_INT);

    reg_wizchip_cris_cbfunc(cris_enter, cris_exit);
    reg_wizchip_cs_cbfunc(cs_select, cs_deselect);
    reg_wizchip_spi_cbfunc(spi_read_byte, spi_write_byte);
    reg_wizchip_spiburst_cbfunc(spi_read_burst, spi_write_burst);
}

void wizchip_glue_reset(void) {
    gpio_put(PIN_RST, 0);
    sleep_ms(10);    /* RSTn low pulse (datasheet min ~500 us; be generous) */
    gpio_put(PIN_RST, 1);
    sleep_ms(150);   /* PLL lock / internal boot before first SPI access */
}
