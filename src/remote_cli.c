/* remote_cli.c — see remote_cli.h. */
#include "remote_cli.h"
#include "cli.h"
#include "log.h"
#include "pico/stdio.h"
#include "pico/stdio/driver.h"
#include "hardware/sync.h"
#include <string.h>

/* Slot lifecycle, every transition under s_lock:
 *   FREE --submit(core1)--> POSTED --service(core0)--> DONE --release(core1)--> FREE
 *   release while POSTED/RUNNING sets `orphan`; core 0 then frees on completion. */
typedef enum { S_FREE, S_POSTED, S_RUNNING, S_DONE } slot_state_t;

static spin_lock_t *s_lock;
static volatile slot_state_t s_state = S_FREE;
static volatile bool         s_orphan;
static volatile uint32_t     s_ticket;      /* ticket of the slot's request */
static uint32_t              s_next_ticket = 1;
static rcli_owner_t          s_owner;
static char                  s_line[CLI_LINE_MAX];
static char                  s_out[RCLI_OUT_MAX];
static volatile uint16_t     s_out_len;
static volatile bool         s_truncated;

void rcli_init(void) {
    s_lock = spin_lock_init((uint)spin_lock_claim_unused(true));
}

/* ── core 1 ── */

bool rcli_submit(rcli_owner_t owner, const char *line, uint32_t *ticket) {
    size_t n = line ? strlen(line) : 0;
    if (n == 0 || n >= sizeof s_line) return false;
    uint32_t irq = spin_lock_blocking(s_lock);
    if (s_state != S_FREE) { spin_unlock(s_lock, irq); return false; }
    memcpy(s_line, line, n + 1);
    s_owner   = owner;
    s_orphan  = false;
    s_ticket  = s_next_ticket++;
    if (s_next_ticket == 0) s_next_ticket = 1;   /* 0 means "none" */
    *ticket   = s_ticket;
    s_state   = S_POSTED;                       /* publish last */
    spin_unlock(s_lock, irq);
    return true;
}

bool rcli_result(uint32_t ticket, const char **out, uint16_t *len, bool *truncated) {
    uint32_t irq = spin_lock_blocking(s_lock);
    bool done = (s_ticket == ticket && s_state == S_DONE);
    if (done) { *out = s_out; *len = s_out_len; *truncated = s_truncated; }
    spin_unlock(s_lock, irq);
    return done;
}

void rcli_release(uint32_t ticket) {
    uint32_t irq = spin_lock_blocking(s_lock);
    if (s_ticket == ticket && s_state != S_FREE) {
        if (s_state == S_DONE) s_state  = S_FREE;
        else                   s_orphan = true;   /* core 0 frees it when finished */
    }
    spin_unlock(s_lock, irq);
}

uint32_t rcli_ticket_of(rcli_owner_t owner) {
    uint32_t irq = spin_lock_blocking(s_lock);
    uint32_t t = (s_state != S_FREE && !s_orphan && s_owner == owner) ? s_ticket : 0u;
    spin_unlock(s_lock, irq);
    return t;
}

/* ── core 0 ── */

/* A stdio driver that appends to s_out. Enabled only while a remote command
 * runs; USB stays enabled alongside it, so the console sees the output too. */
static void cap_out_chars(const char *buf, int len) {
    uint16_t room = (uint16_t)(sizeof s_out - s_out_len);
    if ((uint16_t)len > room) { s_truncated = true; len = room; }
    memcpy(s_out + s_out_len, buf, (size_t)len);
    s_out_len = (uint16_t)(s_out_len + len);
}
static stdio_driver_t s_cap_drv = { .out_chars = cap_out_chars };

void rcli_service(void) {
    uint32_t irq = spin_lock_blocking(s_lock);
    if (s_state != S_POSTED) { spin_unlock(s_lock, irq); return; }
    s_state = S_RUNNING;
    spin_unlock(s_lock, irq);

    s_out_len   = 0;
    s_truncated = false;
    /* Audit trail: every network-issued command lands in /api/log. */
    LOG_INFO("cli", "remote (%s): %s",
             s_owner == RCLI_OWNER_HTTP ? "http" : "mqtt", s_line);
    stdio_flush();
    stdio_set_driver_enabled(&s_cap_drv, true);
    cli_exec_line(s_line);
    stdio_flush();
    stdio_set_driver_enabled(&s_cap_drv, false);

    irq = spin_lock_blocking(s_lock);
    s_state = s_orphan ? S_FREE : S_DONE;
    spin_unlock(s_lock, irq);
}
