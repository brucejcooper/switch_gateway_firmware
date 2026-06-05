/*
 * bus_discovery.c — boot-time bus probe + auto baud recovery.
 *
 * Slaves shipped from the factory default to 9600 baud; the controller's
 * canonical bus speed is 38400.  (38400 was previously thought "marginal
 * against the internal-RC oscillator", but that was actually a TinyModbus RX
 * interrupt-starvation bug dropping bytes — now fixed — so 38400 runs clean.)
 * At boot, for each configured slave, try 38400 first; on timeout, drop to 9600
 * and re-ping; if it responds, send the per-device set-baud command, snap back
 * to 38400, and verify.
 *
 * Blocking by design — must be called from main() before the watchdog is
 * enabled and before the main poll loop starts. Worst case ~10 s per slave.
 */

#include "bus_discovery.h"
#include "modbus.h"
#include "log.h"
#include "pico/time.h"
#include <string.h>

#define DISC_TIMEOUT_MS 1000
#define BAUD_FAST  38400u
#define BAUD_FALLBACK 9600u
#define POST_RECONFIG_SETTLE_MS 100

typedef struct {
    volatile bool done;
    volatile bool ok;
    volatile int  res;
} disc_ctx_t;

static void disc_cb(int res, uint8_t *buf, void *ctx) {
    (void)buf;
    disc_ctx_t *c = (disc_ctx_t *)ctx;
    c->res = res;
    c->ok  = (res > 0);
    c->done = true;
}

static bool wait_done(disc_ctx_t *c) {
    absolute_time_t dl = make_timeout_time_ms(DISC_TIMEOUT_MS);
    while (!c->done && absolute_time_diff_us(get_absolute_time(), dl) > 0) {
        modbus_poll();
    }
    return c->done && c->ok;
}

/* FC01 read 1 coil — universal Waveshare ping */
static bool ping_relay(uint8_t slave_id) {
    disc_ctx_t ctx = { false, false, 0 };
    modbus_get_coils(slave_id, 0, 1, disc_cb, &ctx);
    return wait_done(&ctx);
}

/* FC03 read 1 reg — TinyModbus slave_addr */
static bool ping_dali(uint8_t slave_id) {
    disc_ctx_t ctx = { false, false, 0 };
    modbus_get_holding_registers(slave_id, 0x1012, 1, disc_cb, &ctx);
    return wait_done(&ctx);
}

static bool ping(uint8_t slave_id, slave_kind_t kind) {
    return (kind == SLAVE_KIND_RELAY)
            ? ping_relay(slave_id)
            : ping_dali(slave_id);
}

bool bus_ping_slave(uint8_t slave_id, slave_kind_t kind) {
    return ping(slave_id, kind);
}

/* Map a target baud to the slave's baud register + register value. The register
 * and the encoding differ by kind:
 *   relay (Waveshare 0x2000): 9600=0x0001, 19200=0x0002, 38400=0x0003
 *   dali  (TinyModbus BAUD_INDEX 0x1013): 9600=0x0000, 19200=0x0001, 38400=0x0002
 * Returns false for an unsupported baud. */
static bool baud_reg_value(slave_kind_t kind, uint32_t baud,
                           uint16_t *reg, uint16_t *val) {
    if (kind == SLAVE_KIND_RELAY) {
        *reg = 0x2000;
        switch (baud) {
            case 9600u:  *val = 0x0001; return true;
            case 19200u: *val = 0x0002; return true;
            case 38400u: *val = 0x0003; return true;
        }
    } else {
        *reg = 0x1013;
        switch (baud) {
            case 9600u:  *val = 0x0000; return true;
            case 19200u: *val = 0x0001; return true;
            case 38400u: *val = 0x0002; return true;
        }
    }
    return false;
}

/* FC06 write the slave's baud register to switch its UART to `baud`. */
bool bus_set_slave_baud(uint8_t slave_id, slave_kind_t kind, uint32_t baud) {
    uint16_t reg, val;
    if (!baud_reg_value(kind, baud, &reg, &val)) return false;
    disc_ctx_t ctx = { false, false, 0 };
    modbus_set_holding_register(slave_id, reg, val, disc_cb, &ctx);
    return wait_done(&ctx);
}

bool bus_discover_slave(uint8_t slave_id, slave_kind_t kind) {
    const char *kind_s = (kind == SLAVE_KIND_RELAY) ? "relay" : "dali";

    if (ping(slave_id, kind)) {
        LOG_INFO("disc", "slave %u (%s) ok at %u", slave_id, kind_s, BAUD_FAST);
        return true;
    }

    LOG_WARN("disc", "slave %u (%s) no response at %u, trying %u",
             slave_id, kind_s, BAUD_FAST, BAUD_FALLBACK);
    modbus_uart_set_baud_runtime(BAUD_FALLBACK);

    if (!ping(slave_id, kind)) {
        LOG_ERR("disc", "slave %u (%s) unreachable at both bauds",
                slave_id, kind_s);
        modbus_uart_set_baud_runtime(BAUD_FAST);
        return false;
    }

    LOG_INFO("disc", "slave %u (%s) found at %u, reconfiguring to %u",
             slave_id, kind_s, BAUD_FALLBACK, BAUD_FAST);
    if (!bus_set_slave_baud(slave_id, kind, BAUD_FAST)) {
        LOG_ERR("disc", "slave %u (%s) set-baud command failed",
                slave_id, kind_s);
        modbus_uart_set_baud_runtime(BAUD_FAST);
        return false;
    }

    /* Slave acks at 9600, then switches its own UART. Snap us back. */
    modbus_uart_set_baud_runtime(BAUD_FAST);
    sleep_ms(POST_RECONFIG_SETTLE_MS);

    if (!ping(slave_id, kind)) {
        LOG_ERR("disc", "slave %u (%s) didn't reappear at %u after reconfig",
                slave_id, kind_s, BAUD_FAST);
        return false;
    }

    LOG_INFO("disc", "slave %u (%s) now at %u", slave_id, kind_s, BAUD_FAST);
    return true;
}
