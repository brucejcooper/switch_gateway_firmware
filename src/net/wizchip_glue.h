#pragma once
/*
 * wizchip_glue.h — RP2040 <-> W5100S SPI/CS/reset glue for ioLibrary_Driver.
 *
 * Owns SPI0 exclusively (core 1). Registers the cris / chip-select / SPI byte +
 * burst callbacks the ioLibrary calls, and exposes the hardware reset pulse.
 */

#include <stdint.h>

/* Configure SPI0 + CS/RST/INT GPIO and register the ioLibrary callbacks.
 * Call once on core 1 before wizchip_init(). */
void wizchip_glue_init(void);

/* Hardware-reset the W5100S via RSTn (blocking ~60 ms). Used at bring-up and on
 * recovery from an SPI-desynced / wedged chip. */
void wizchip_glue_reset(void);
