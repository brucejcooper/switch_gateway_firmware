/*
 * intercore.c — the two SPSC queues bridging core 0 (domain) and core 1 (net).
 *
 * pico_util queue_t is multicore-safe (spinlock-backed) and lives in SRAM, so it
 * leaves the SIO inter-core FIFO free for flash_safe_execute()'s lockout
 * handshake. See intercore.h for the contract.
 */

#include "intercore.h"
#include "pico/util/queue.h"
#include "hardware/sync.h"   /* __dmb() — snapshot publish barrier */

/* Event queue (core0 -> core1). Sized to hold a full synchronous resync burst
 * without dropping: worst case 32 relays + 2*64 ballasts + 2*16 groups +
 * 2 scan_completes + up to 24*8 mappings = ~386 events, all pushed in one
 * IC_CMD_RESYNC handler before core 1 next drains. 512 leaves margin
 * (~50 KB heap at this element size — plenty on the RP2040's 264 KB). */
#define IC_EVENT_QUEUE_DEPTH 512
/* Command queue (core1 -> core0). Not a scheduler FIFO — core 0 empties it into
 * the latest-wins desired-state tables every loop pass, so depth costs no
 * latency. A 107-command HA burst in 11 ms measured 0 drops at 16 (2026-10-10);
 * 64 is headroom for the times core 0 is briefly busy (a group-membership write
 * spins up to 2 s per member). */
#define IC_CMD_QUEUE_DEPTH   64

static queue_t   s_event_q;
static queue_t   s_cmd_q;
static volatile uint32_t s_event_drops = 0;

void intercore_init(void) {
    queue_init(&s_event_q, sizeof(ic_event_t), IC_EVENT_QUEUE_DEPTH);
    queue_init(&s_cmd_q,   sizeof(ic_cmd_t),   IC_CMD_QUEUE_DEPTH);
}

bool intercore_push_event(const ic_event_t *e) {
    if (queue_try_add(&s_event_q, e)) return true;
    s_event_drops++;
    return false;
}

bool intercore_pop_event(ic_event_t *e) {
    return queue_try_remove(&s_event_q, e);
}

bool intercore_push_cmd(const ic_cmd_t *c) {
    return queue_try_add(&s_cmd_q, c);
}

bool intercore_pop_cmd(ic_cmd_t *c) {
    return queue_try_remove(&s_cmd_q, c);
}

uint32_t intercore_event_drops(void) {
    return s_event_drops;
}

static ic_snapshot_t s_snapshot;
void intercore_set_snapshot(const ic_snapshot_t *s) {
    s_snapshot = *s;
    /* Release barrier: this copy MUST be globally visible before the caller
     * (main.c) sets the s_config_ready flag core 1 spins on. Today the
     * ordering holds only INCIDENTALLY — no LTO is configured, so GCC treats
     * the separate intercore_set_snapshot()/core1_config_ready() cross-TU
     * calls as memory clobbers it can't reorder around. Turn on LTO, or ever
     * inline either, and core 1 could observe config_ready==true against a
     * half-written snapshot and bring up the network on wrong MQTT/slave
     * config, silently. This makes the ordering enforced, not incidental;
     * core1_entry() has the paired acquire barrier. */
    __dmb();
}
const ic_snapshot_t *intercore_snapshot(void) { return &s_snapshot; }
