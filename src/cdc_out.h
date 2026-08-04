#pragma once
#include <stdint.h>

/*
 * cdc_out — non-blocking, core-safe output ring for the USB-CDC console.
 *
 * Why this exists: printf() straight to
 * the USB-CDC stdout, which BLOCKS the calling core when the host isn't draining
 * the FIFO fast enough.  Under a `log dbg` firehose that stall tripped the 5 s
 * watchdog and rebooted the controller (see CLAUDE.md / the Jun-16 reboots).
 *
 * Model: producers (either core, ISR or main) format a whole line and hand it to
 * cdc_out_write(), which copies it into a ring under a brief critical section and
 * returns immediately — it NEVER blocks on the host.  If the line doesn't fit,
 * the whole line is dropped and a counter is bumped.  A single consumer
 * (cdc_out_drain(), called from core-0's serial_poll) pushes ring bytes to the
 * CDC only as far as tud_cdc_write_available() allows, so the drain never blocks
 * either, and emits a "<N> log msgs dropped" notice when space frees up.
 */

/* Initialise the ring's critical section. Call once at the very start of main(),
 * before the first LOG_* / cdc_out_write() and before core 1 launches. */
void cdc_out_init(void);

/* Enqueue a complete message (typically one formatted log line). Non-blocking and
 * safe from either core / ISR context. If the ring lacks room for the whole
 * buffer it is dropped wholesale and the drop counter is incremented (so lines
 * are never half-written). */
void cdc_out_write(const char *buf, int len);

/* Push as many queued bytes to the USB-CDC as the TX FIFO can accept right now,
 * then return — never blocks. Emits a dropped-count notice first when one is
 * pending and there is room. Call from core-0's main loop (serial_poll). */
void cdc_out_drain(void);

/* Format one line and enqueue it via cdc_out_write(). Use this instead of
 * printf() for any output longer than the USB-CDC FIFO: stdout is configured
 * non-blocking (PICO_STDIO_USB_STDOUT_TIMEOUT_US=0), so a printf burst that
 * outruns the host is silently TRUNCATED MID-LINE, which quietly corrupts long
 * diagnostic dumps. Going through the ring hands the bytes to the drain in
 * serial_poll instead, which paces them to what the FIFO can take. Lines longer
 * than 256 bytes are truncated (with the formatting intact). */
void cdc_out_printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Copy the most recent log scrollback (oldest→newest) into dst, up to cap bytes,
 * and return the byte count. This is a SEPARATE retained ring from the delivery
 * ring above: it is never consumed (new bytes overwrite the oldest), so recent
 * logs stay readable over the network (GET /api/log) even when USB has no reader
 * or is wedged. Safe to call from either core. */
int cdc_out_history(char *dst, int cap);
