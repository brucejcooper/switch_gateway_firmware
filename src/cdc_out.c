#include "cdc_out.h"

#include "pico/critical_section.h"
#include "pico/stdio_usb.h"   /* extern stdio_driver_t stdio_usb */
#include "pico/stdio/driver.h" /* full stdio_driver_t definition (.out_chars etc.) */
#include "pico/time.h"
#include "tusb.h"             /* tud_cdc_write_available */

#include <stdio.h>

/* Ring size: comfortably absorbs a `log dbg` burst between drains while costing
 * little of the RP2040's 264 KB. Power of two so the index masks are cheap. */
#define CDC_RING_SZ   4096u
#define CDC_RING_MASK (CDC_RING_SZ - 1u)

static uint8_t  s_ring[CDC_RING_SZ];
/* Free-running counters (masked to index). head: written by producers only.
 * tail: written by the single consumer (drain) only. 32-bit aligned loads are
 * atomic on Cortex-M0+, so each side can read the other's counter unlocked. */
static volatile uint32_t s_head;
static volatile uint32_t s_tail;
static volatile uint32_t s_dropped;        /* whole messages dropped on a full ring */

/* Serialises producers against each other (core-0 main vs core-0 ISR vs core-1)
 * and the dropped-counter RMW. The consumer does NOT take it: it owns s_tail and
 * only reads s_head/s_dropped, so it can never be blocked by — nor block — a
 * producer. Held only for a per-line memcpy (sub-µs), so it can't stall a core. */
static critical_section_t s_cs;

void cdc_out_init(void) {
    critical_section_init(&s_cs);
}

void cdc_out_write(const char *buf, int len) {
    if (len <= 0) return;
    critical_section_enter_blocking(&s_cs);
    uint32_t used = s_head - s_tail;
    if ((uint32_t)len <= CDC_RING_SZ - used) {
        for (int i = 0; i < len; i++)
            s_ring[(s_head + (uint32_t)i) & CDC_RING_MASK] = (uint8_t)buf[i];
        s_head += (uint32_t)len;            /* publish only after the bytes are in */
    } else {
        s_dropped++;                        /* drop the whole line, never a fragment */
    }
    critical_section_exit(&s_cs);
}

void cdc_out_drain(void) {
    /* Nothing can be delivered while no host has the port open: stdio_usb.out_chars
     * silently writes 0 when disconnected, so draining now would advance the tail
     * over bytes that never went out. Leave them queued — the ring fills and
     * drop-counts, and on reconnect the drop notice + fresh lines appear. */
    if (!stdio_usb_connected()) return;

    /* 1. Report drops first, when the host has caught up enough to take the note.
     * Snapshot the count, and only subtract what we actually reported so any
     * drops racing in during this call survive to the next drain. */
    uint32_t dropped = s_dropped;
    if (dropped) {
        char note[56];
        int n = snprintf(note, sizeof note, "# %lu W cdc: %lu log msgs dropped\n",
                         (unsigned long)to_ms_since_boot(get_absolute_time()),
                         (unsigned long)dropped);
        if (n > 0 && (int)tud_cdc_write_available() >= n) {
            stdio_usb.out_chars(note, n);   /* fits => non-blocking even at any timeout */
            critical_section_enter_blocking(&s_cs);
            s_dropped -= dropped;
            critical_section_exit(&s_cs);
        }
    }

    /* 2. Drain the ring, but only ever hand stdio_usb.out_chars as many bytes as
     * the TX FIFO can take this instant. Because n <= avail, out_chars writes all
     * n and returns without entering its blocking timeout path. */
    for (;;) {
        int avail = (int)tud_cdc_write_available();
        if (avail <= 0) break;
        uint32_t used = s_head - s_tail;
        if (used == 0) break;
        uint32_t tail_idx = s_tail & CDC_RING_MASK;
        uint32_t contig   = CDC_RING_SZ - tail_idx;     /* bytes before wrap */
        uint32_t n = used < contig ? used : contig;
        if (n > (uint32_t)avail) n = (uint32_t)avail;
        stdio_usb.out_chars((const char *)&s_ring[tail_idx], (int)n);
        s_tail += n;                                    /* sole writer of s_tail */
    }
    stdio_usb.out_flush();
}
