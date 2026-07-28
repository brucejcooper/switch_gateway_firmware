#include "cdc_out.h"
#include <stdarg.h>
#include <stdio.h>
#include "partition_map.h"  /* compiled here purely so its _Static_assert()s run */

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

/* Retained log scrollback — a SECOND, independent ring that is never consumed:
 * new bytes overwrite the oldest. cdc_out_history() dumps it for GET /api/log so
 * recent logs are readable over the network even with no USB reader (or a wedged
 * USB stack). Written under s_cs alongside the delivery ring; 8 KB holds a useful
 * recent window without meaningfully denting the RP2040's 264 KB SRAM. */
#define HIST_SZ   8192u
#define HIST_MASK (HIST_SZ - 1u)
static char     s_hist[HIST_SZ];
static uint32_t s_hist_head;               /* total bytes ever written (masked to index) */

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
    /* Always retain in the scrollback ring, independent of delivery: it overwrites
     * the oldest bytes and is never dropped, so /api/log shows recent history even
     * when the delivery ring was full or no host was reading. */
    for (int i = 0; i < len; i++)
        s_hist[(s_hist_head + (uint32_t)i) & HIST_MASK] = buf[i];
    s_hist_head += (uint32_t)len;
    critical_section_exit(&s_cs);
}

int cdc_out_history(char *dst, int cap) {
    if (cap <= 0) return 0;
    critical_section_enter_blocking(&s_cs);
    uint32_t head  = s_hist_head;
    uint32_t avail = head < HIST_SZ ? head : HIST_SZ;
    if (avail > (uint32_t)cap) avail = (uint32_t)cap;   /* keep the newest cap bytes */
    uint32_t start = head - avail;
    for (uint32_t i = 0; i < avail; i++)
        dst[i] = s_hist[(start + i) & HIST_MASK];
    critical_section_exit(&s_cs);
    return (int)avail;
}

void cdc_out_printf(const char *fmt, ...) {
    char line[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    if (n < 0) return;
    if (n > (int)sizeof line - 1) n = (int)sizeof line - 1;
    cdc_out_write(line, n);
}

void cdc_out_drain(void) {
    /* Nothing can be delivered while no host has the port open: stdio_usb.out_chars
     * silently writes 0 when disconnected, so draining now would advance the tail
     * over bytes that never went out. Leave them queued — the ring fills and
     * drop-counts, and on reconnect the drop notice + fresh lines appear.
     *
     * BUT still pump the USB device task on the way out. Every other core-0 caller
     * of tud_task() (stdio_usb.out_chars, .in_chars) is gated behind
     * stdio_usb_connected(), so once DTR drops (host closes/stalls) the ONLY thing
     * left driving tud_task() is stdio_usb's background timer/IRQ. If that poke
     * ever stalls, USB freezes in both directions with no thread-context fallback —
     * the wedge that needed a physical reset. out_flush() runs tud_task() under
     * stdio_usb_mutex (safe against the background IRQ) and is cheap when idle, so
     * core 0's main loop keeps the stack serviced whether or not a host is open. */
    if (!stdio_usb_connected()) { stdio_usb.out_flush(); return; }

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
