/*
 * modbus.c — Modbus RTU master for the switch gateway.
 *
 * Architecture is unchanged from the original: ISR-driven RX with an alarm
 * for DE release timing, non-blocking poll loop.  Changes from original:
 *   - ACTION_DALI (0x44) removed; TinyModbus uses standard FC04/FC06
 *   - FC04 Read Input Registers added (ACTION_GET_INPUT_REGISTERS)
 *   - Baud rate is configurable, defaulting to 19200 (38400 is marginal
 *     against the TinyModbus ATtiny's internal-RC-oscillator RX sampling)
 *   - Baud rate and DE mode are both persisted to flash filesystem
 */

#include "modbus.h"
#include "mbq_client.h"   /* MBQ_CMD_BYTES — FC65 payload size */
#include "config.h"
#include "log.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "pico/util/queue.h"
#include "stdio.h"
#include <ctype.h>
#include <pico/time.h>
#include <pico/types.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

/* A present slave starts replying within ~10 ms (even at 19200), so 50 ms is
 * ample to detect "no answer" without falsely firing on a slow reply.  A bare
 * timeout (no valid frame) is retried up to MODBUS_MAX_RETRIES times before the
 * failure is delivered to the caller, transparently recovering the occasional
 * dropped frame.  A CRC-valid reply — including a Modbus exception — is a real
 * answer and is never retried. */
#define MODBUS_RESPONSE_TIMEOUT_MS 50
#define MODBUS_MAX_RETRIES         2

/* A gateway mid-DALI-frame answers a new RAW_CMD with DEVICE_BUSY (0x06). Rather
 * than dropping the command, wait roughly one average DALI forward-frame time and
 * try again, polling up to DALI_BUSY_RETRIES times. This transparently serialises
 * rapid same-gateway commands (e.g. a button bound to several ballasts). */
#define DALI_BUSY_RETRY_US         10000u   /* ~avg DALI forward-frame execution */
#define DALI_BUSY_RETRIES          6u

/* Hard upper bound on a single in-flight transaction (all retries + deadtimes
 * included; legitimate worst case is ~250 ms). If the bus SM sits in any non-IDLE
 * state longer than this it is wedged — a missed DE-release alarm, an endless RX
 * dribble keeping the inter-byte timeout fresh, or a torn state transition. The
 * poll loop force-recovers to IDLE so commands resume instead of stopping
 * forever. */
#define MODBUS_OP_MAX_MS           400u

#define UART_ID      uart1
#define UART_TX_PIN  4
#define UART_RX_PIN  5
#define UART_DE_PIN  1

#define DEFAULT_BAUD_RATE 19200u

typedef enum {
    MODBUS_IDLE,
    MODBUS_TRANSMITTING,
    MODBUS_RECEIVING,
    MODBUS_RECEIVED_RESPONSE,
    MODBUS_DEADTIME,
} modbus_bus_state_t;

typedef enum {
    ACTION_GET_COILS             = 0x01,
    ACTION_GET_HOLDING_REGISTERS = 0x03,
    ACTION_GET_INPUT_REGISTERS   = 0x04,   /* new: FC04 for TinyModbus */
    ACTION_SET_COIL              = 0x05,
    ACTION_SET_HOLDING_REGISTER  = 0x06,
    ACTION_WRITE_MULTI_REG       = 0x10,   /* new: FC16 Write Multiple Registers */
    ACTION_READ_WRITE_MULTI_REG  = 0x17,   /* new: FC23 Read/Write Multiple Registers */
    /* User-defined codes for the DALI command queue. MODBUS Application Protocol
     * V1.1b reserves 65-72 and 100-110 for exactly this, so they are conformant
     * rather than an extension of the standard set — ordinary RTU framing, CRC,
     * t3.5 silence and exception replies; only the payload shape is ours.
     * See TinyModbus/src/modbus.c FC_MBQ_ENQUEUE for the rationale. */
    ACTION_MBQ_ENQUEUE           = 0x41,   /* FC65: enqueue 1 cmd anywhere + status */
    ACTION_MBQ_STATUS            = 0x42,   /* FC66: status only */
} modbus_action_type_t;

/* Largest register block any FC16/FC23 write carries here is the gateway's
 * 4-command submit window (0x1000-0x1007) = 8 registers = 16 bytes.  Sized to
 * that; a longer write is rejected at enqueue time. */
#define MODBUS_WR_PAYLOAD_MAX 16u

typedef struct {
    modbus_action_type_t action;
    uint8_t              device;
    uint16_t             address;   /* FC03/04/06: start addr.  FC23: READ start addr. */
    uint16_t             arg;       /* read count / single-write value.  FC23: READ reg count. */
    /* FC16/FC23 write side. */
    uint16_t             wr_address;              /* write start addr */
    uint16_t             wr_count;                /* write register count */
    uint8_t              wr_payload[MODBUS_WR_PAYLOAD_MAX]; /* big-endian register bytes */
    uint8_t              wr_len;                  /* bytes used in wr_payload */
    modbus_action_cb_t   callback;
    void                *ctx;
} modbus_action_t;

static modbus_de_mode_t      de_mode    = MODBUS_DE_WHENTX;
static uint32_t              s_baud     = DEFAULT_BAUD_RATE;

/* t3.5 — the inter-frame silence that DEFINES the end of an RTU frame.
 *
 * MODBUS over Serial Line V1.02 §2.5.1.1: a frame is over when the line has been
 * quiet for 3.5 character times, and only then is it validated. Above 19200 the
 * spec stops deriving the timer (servicing it per character gets expensive) and
 * prescribes a fixed 1.750 ms. The character is 11 bits in the spec's model
 * (start + 8 data + parity + stop, or 2 stop bits when parity is unused), so we
 * derive from 11 even though the UART runs 8N1 — deriving from 10 would give a
 * window 9% short of what a conformant peer may legitimately use. */
static uint32_t t35_us(uint32_t baud) {
    if (baud > 19200u) return 1750u;              /* spec's fixed value */
    return baud ? (uint32_t)(38500000u / baud) : 2005u;   /* 3.5 * 11 * 1e6 / baud */
}
static volatile absolute_time_t rx_silence_at;
static uint8_t              *readptr;
static size_t                readbuf_sz;
static size_t                readbuf_max;
static uint16_t              readbuf_crc;
static modbus_action_t       currentAction;
static absolute_time_t       readTimeout;
static uint8_t               buf[256];
static volatile modbus_bus_state_t bus_state = MODBUS_IDLE;
static queue_t               action_queue;
static queue_t               response_queue;

/* Retry bookkeeping for the in-flight action: attempts remaining and whether a
 * re-transmit is pending once the post-timeout gap elapses. */
static uint8_t               s_attempts_left;
static uint8_t               s_busy_left;     /* DEVICE_BUSY retries remaining */
static bool                  s_retry_pending;
static absolute_time_t       s_op_deadline;   /* hard deadline for the whole in-flight op */
static uint32_t              s_drop_count = 0;    /* actions dropped on a full queue */
static uint32_t              s_recover_count = 0; /* forced SM recoveries (wedge watchdog) */

/* ── Bus-occupancy accounting (bench instrumentation) ─────────────────────────
 * Who is actually using the RS-485 bus, and for how long. A transaction's
 * occupancy is measured from the moment it leaves the action queue until the SM
 * returns to IDLE — so it includes its retries and the post-frame dead time,
 * i.e. the real cost of that transaction to every other producer. Zero cost when
 * nobody reads it; reset + read via modbus_busload_*(). */
static absolute_time_t       s_bl_window;         /* start of the measurement window */
static absolute_time_t       s_bl_act_start;      /* current transaction's start */
static uint64_t              s_bl_busy_us;        /* total bus-occupied time in the window */
static uint32_t              s_bl_txns;
static uint32_t              s_bl_retries;        /* re-sends after no valid reply */
static absolute_time_t       s_bl_rx_start;       /* DE release -> waiting for the reply */
static uint64_t              s_bl_turn_sum;       /* slave turnaround: request out -> reply in */
static uint32_t              s_bl_turn_n, s_bl_turn_max;
static uint32_t              s_bl_busy_retries;   /* re-sends after a DEVICE_BUSY (0x06) */
static modbus_busload_slave_t s_bl[MODBUS_BUSLOAD_SLAVES];
static uint32_t              s_bl_skipped;        /* transactions skipped by backoff */

/* ── Absent-slave backoff ────────────────────────────────────────────────────
 * A slave that is not there costs MODBUS_MAX_RETRIES+1 attempts x the response
 * timeout — ~168 ms of bus per transaction, every time any producer polls it.
 * On the bench that single dead board held ~8% of the RS-485 bus and was the
 * LARGEST contributor to worst-case button latency (measured 137 ms of the
 * 236 ms idle worst case): a user command arriving behind one waits the whole
 * thing out, because a Modbus transaction cannot be preempted.
 *
 * So after MODBUS_BACKOFF_TRIP consecutive dead transactions, stop putting that
 * slave on the wire. Requests to it are completed IMMEDIATELY as a timeout
 * (res = 0) without transmitting — producers see exactly what they saw before,
 * just without the bus cost, so no caller needs to know this exists. The
 * backoff doubles from 1 s to a 60 s cap, and ONE probe is allowed through each
 * time it expires, so a slave that comes back is picked up within a minute. Any
 * valid reply — including a Modbus exception, which is the slave alive and
 * disagreeing — resets it to healthy at once. */
#define MODBUS_BACKOFF_SLAVES  8u
#define MODBUS_BACKOFF_TRIP    3u
#define MODBUS_BACKOFF_MIN_MS  1000u
#define MODBUS_BACKOFF_MAX_MS  60000u

typedef struct {
    bool            in_use;
    uint8_t         slave;
    uint8_t         fails;        /* consecutive dead transactions */
    uint32_t        backoff_ms;   /* 0 = healthy */
    absolute_time_t retry_at;
    uint32_t        skipped;      /* transactions spared the bus */
} mb_backoff_t;
static mb_backoff_t s_bo[MODBUS_BACKOFF_SLAVES];

static mb_backoff_t *backoff_for(uint8_t slave) {
    mb_backoff_t *free_slot = NULL;
    for (uint8_t i = 0; i < MODBUS_BACKOFF_SLAVES; i++) {
        if (s_bo[i].in_use && s_bo[i].slave == slave) return &s_bo[i];
        if (!s_bo[i].in_use && !free_slot) free_slot = &s_bo[i];
    }
    if (!free_slot) return NULL;   /* more slaves than slots: no backoff for it */
    free_slot->in_use = true;
    free_slot->slave  = slave;
    return free_slot;
}

/* A transaction to `slave` finished. `answered` = we got a valid frame back
 * (data or exception); false = no reply after every retry. */
static void backoff_note(uint8_t slave, bool answered) {
    mb_backoff_t *b = backoff_for(slave);
    if (!b) return;
    if (answered) {
        if (b->backoff_ms)
            LOG_INFO("modbus", "slave %u answering again — backoff cleared", slave);
        b->fails = 0;
        b->backoff_ms = 0;
        return;
    }
    if (b->fails < 0xFFu) b->fails++;
    if (b->fails < MODBUS_BACKOFF_TRIP) return;
    b->backoff_ms = b->backoff_ms ? (b->backoff_ms * 2u) : MODBUS_BACKOFF_MIN_MS;
    if (b->backoff_ms > MODBUS_BACKOFF_MAX_MS) b->backoff_ms = MODBUS_BACKOFF_MAX_MS;
    b->retry_at = make_timeout_time_ms(b->backoff_ms);
    LOG_WARN("modbus", "slave %u unresponsive — backing off %lu ms",
             slave, (unsigned long)b->backoff_ms);
}

/* True if this request should be failed without touching the bus. Lets exactly
 * one probe through each time the backoff expires. Sets *is_probe so the caller
 * can make that probe CHEAP. */
static bool backoff_should_skip(uint8_t slave, bool *is_probe) {
    *is_probe = false;
    mb_backoff_t *b = backoff_for(slave);
    if (!b || !b->backoff_ms) return false;
    if (time_reached(b->retry_at)) {
        /* Probe window: let one through, and re-arm immediately so a burst of
         * queued requests does not all become probes. */
        b->retry_at = make_timeout_time_ms(b->backoff_ms);
        *is_probe = true;
        return false;
    }
    b->skipped++;
    return true;
}

static bool modbus_rx_valid(void) { return true; }

static const char *modbus_de_mode_str[MODBUS_DE_MAX] = { "tx", "always" };

/* Standard Modbus exception codes (MODBUS Application Protocol v1.1b3 §7). */
const char *modbus_exception_str(uint8_t code) {
    switch (code) {
    case 0x01: return "illegal function";
    case 0x02: return "illegal data address";
    case 0x03: return "illegal data value";
    case 0x04: return "device failure";
    case 0x05: return "acknowledge";
    case 0x06: return "device busy";
    case 0x07: return "negative acknowledge";
    case 0x08: return "memory parity error";
    case 0x0A: return "gateway path unavailable";
    case 0x0B: return "gateway target no response";
    default:   return "unknown";
    }
}

/* Emit the *intent* of an action at DEBUG level (what we're asking the slave
 * to do), separate from the raw tx byte dump which also lives at DEBUG. The
 * higher-level *why* (which subsystem requested the read and for what reason)
 * is logged by the callers, e.g. dirty_poll.c. */
static void log_action_intent(const modbus_action_t *a) {
    switch (a->action) {
    case ACTION_GET_COILS:
        LOG_DBG("modbus", "read coils slave=%u addr=%u count=%u",
                a->device, a->address, a->arg);
        break;
    case ACTION_GET_HOLDING_REGISTERS:
        LOG_DBG("modbus", "read holding registers slave=%u addr=%u count=%u",
                a->device, a->address, a->arg);
        break;
    case ACTION_GET_INPUT_REGISTERS:
        LOG_DBG("modbus", "read input registers slave=%u addr=%u count=%u",
                a->device, a->address, a->arg);
        break;
    case ACTION_SET_COIL:
        LOG_DBG("modbus", "set coil slave=%u addr=%u -> %s",
                a->device, a->address,
                a->arg == 0x5500u ? "toggle" : (a->arg ? "on" : "off"));
        break;
    case ACTION_SET_HOLDING_REGISTER:
        LOG_DBG("modbus", "set holding register slave=%u addr=%u value=%u",
                a->device, a->address, a->arg);
        break;
    case ACTION_WRITE_MULTI_REG:
        LOG_DBG("modbus", "write multi reg slave=%u addr=%u count=%u",
                a->device, a->wr_address, a->wr_count);
        break;
    case ACTION_READ_WRITE_MULTI_REG:
        LOG_DBG("modbus", "read/write multi slave=%u rd=%u/%u wr=%u/%u",
                a->device, a->address, a->arg, a->wr_address, a->wr_count);
        break;
    default:
        LOG_DBG("modbus", "action 0x%02x slave=%u addr=%u arg=%u",
                (unsigned)a->action, a->device, a->address, a->arg);
        break;
    }
}

/* ── CRC-16 table (identical to original) ────────────────────────────────── */
static const uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
    0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
    0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
    0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
    0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
    0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
    0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
    0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
    0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
    0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
    0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
    0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
    0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
    0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
    0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
    0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
    0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
    0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
    0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
    0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
    0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
    0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
    0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
    0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
    0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
    0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
    0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
    0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
    0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040,
};

static void append_crc16(uint8_t *b, size_t sz) {
    uint8_t  tmp;
    uint16_t crcVal = 0xFFFF;
    while (sz--) {
        tmp     = *b++ ^ crcVal;
        crcVal >>= 8;
        crcVal ^= crcTable[tmp];
    }
    *b++ = crcVal;
    *b++ = crcVal >> 8;
}

/* ── UART ISR (identical to original) ───────────────────────────────────── */
static void uart_irq(void) {
    uint8_t tmp;
    int     nread = 0;

    while (uart_is_readable(UART_ID)) {
        readTimeout   = make_timeout_time_us(7500);
        rx_silence_at = make_timeout_time_us(t35_us(s_baud));
        uint8_t ch  = uart_getc(UART_ID);
        if (readbuf_sz < readbuf_max) {
            *readptr++ = ch;
            readbuf_sz++;
            tmp          = ch ^ readbuf_crc;
            readbuf_crc >>= 8;
            readbuf_crc ^= crcTable[tmp];
        }
        /* Deliberately NOT completing here on a zero running CRC. That finished a
         * frame the instant its bytes happened to validate, without ever observing
         * t3.5 — so the frame boundary was inferred from content rather than from
         * silence, which is what RTU actually defines it by. It also meant a
         * truncated reply whose prefix chanced to CRC-clean (1 in 65536 per byte
         * position) would be accepted as whole. The poll loop now closes the frame
         * on silence; see MODBUS_RECEIVING. */
        nread++;
    }
    /* Note: do NOT call LOG_* from ISR context.  If nread==0 the next
     * MODBUS_RECEIVING tick will time out and emit a warn from the poll
     * loop instead. */
    (void)nread;
}

/* ── DE release alarm (identical to original) ────────────────────────────── */
static int64_t release_de(alarm_id_t id, void *userdata) {
    while (uart_get_hw(UART_ID)->fr & UART_UARTFR_BUSY_BITS) {
        ;   /* wait for the final byte to clear the shift register */
    }
    if (de_mode == MODBUS_DE_WHENTX) {
        gpio_put(UART_DE_PIN, 0);   /* DE/RE low -> receiver enabled */
    }
    /* Transmit is complete and the line has settled. Discard anything the RX
     * FIFO captured during the transmit (the controller's own frame, latched
     * via the muted/floating receiver) and only NOW arm the receiver for the
     * slave's reply. This is the fix for the half-duplex "rx == tx" self-echo. */
    while (uart_is_readable(UART_ID)) uart_getc(UART_ID);
    readptr     = buf;
    readbuf_sz  = 0;
    readbuf_max = sizeof(buf);
    readbuf_crc = 0xFFFF;
    readTimeout = make_timeout_time_ms(MODBUS_RESPONSE_TIMEOUT_MS);
    s_bl_rx_start = get_absolute_time();
    uart_set_irqs_enabled(UART_ID, true, false);
    bus_state = MODBUS_RECEIVING;
    return 0;
}

/* ── Config helpers ──────────────────────────────────────────────────────── */
void set_modbus_de_mode(modbus_de_mode_t mode) {
    de_mode = mode;
    g_config.de_mode = (uint8_t)mode;   /* persisted via the unified config.json */
    if (!config_save())
        LOG_ERR("modbus", "config_save failed — de_mode not persisted");
}

modbus_de_mode_t modbus_get_de_mode(void) { return de_mode; }

const char *modbus_de_mode_to_str(modbus_de_mode_t mode) {
    if (mode >= MODBUS_DE_MAX) return "?";
    return modbus_de_mode_str[mode];
}

void modbus_set_baud(uint32_t baud) {
    s_baud = baud;
    g_config.baud = baud;               /* persisted via the unified config.json */
    if (!config_save())
        LOG_ERR("modbus", "config_save failed — baud not persisted");
}

uint32_t modbus_get_baud(void) { return s_baud; }

void modbus_uart_set_baud_runtime(uint32_t baud) {
    LOG_INFO("modbus", "uart baud -> %lu", (unsigned long)baud);
    s_baud = baud;
    uart_set_baudrate(UART_ID, baud);
}

/* ── Init ────────────────────────────────────────────────────────────────── */
void modbus_init(void) {
    queue_init(&action_queue,   sizeof(modbus_action_t), 256);
    queue_init(&response_queue, sizeof(modbus_action_t),  20);

    /* Baud + DE mode now come from the unified config (loaded by config_load()
     * before modbus_init(); migrated once from the legacy /modbus_*.txt files). */
    de_mode = (g_config.de_mode < MODBUS_DE_MAX) ? (modbus_de_mode_t)g_config.de_mode
                                                 : MODBUS_DE_WHENTX;
    s_baud  = g_config.baud ? g_config.baud : DEFAULT_BAUD_RATE;

    LOG_INFO("modbus", "init de_mode=%s baud=%lu",
             modbus_de_mode_str[de_mode], (unsigned long)s_baud);

    gpio_init(UART_DE_PIN);
    gpio_put(UART_DE_PIN, 0);
    gpio_set_dir(UART_DE_PIN, GPIO_OUT);

    uart_init(UART_ID, s_baud);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));
    /* Bias RX to the UART idle level (high). In half-duplex the transceiver
     * tri-states RxD while we transmit (RE tracks DE), leaving this pin
     * floating; without a pull it capacitively couples from the adjacent TX pin
     * and the UART latches our own outgoing frame as a phantom reply. The
     * pull-up is the electrical root-cause fix; release_de()'s deferred RX-arm
     * is the matching firmware discipline (don't listen while transmitting). */
    gpio_pull_up(UART_RX_PIN);

    /* RX FIFO OFF: one interrupt per byte.
     *
     * t3.5 frame closing measures silence as "time since a byte was drained",
     * which is only the same as "time since a byte arrived on the wire" if the
     * ISR runs per byte. With the FIFO on, the SDK arms RXIM at a 4-character
     * threshold, so during continuous reception the ISR fires every 4 bytes —
     * 2.08 ms at 19200, LONGER than t3.5 itself (2.005 ms). The main loop then
     * sees an apparent silence mid-frame and closes it early: measured as
     * `short_response` and ~50%% timeout-retries on long replies.
     *
     * Per-byte interrupts cost 1920/s at 19200, nothing on a 125 MHz core. The
     * trade is losing 32 bytes of hardware buffering against interrupt latency,
     * which at 16.7 ms of holdup was never much protection anyway. */
    uart_set_fifo_enabled(UART_ID, false);

    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, uart_irq);
    irq_set_enabled(UART_IRQ, true);
}

/* Hex-dump a frame at DEBUG level. Shared by the tx and rx paths so a request
 * and its response are logged in the same "[AA BB ..]" format. Bounded to
 * MODBUS_LOG_HEX_MAX bytes (well above any real frame); a longer frame is noted
 * with a "+N" trailer rather than silently truncated. */
#define MODBUS_LOG_HEX_MAX 64
static void log_frame_hex(const char *dir, const uint8_t *b, size_t n) {
    char hex[3 * MODBUS_LOG_HEX_MAX + 8];
    int hp = 0;
    size_t lim = n < MODBUS_LOG_HEX_MAX ? n : MODBUS_LOG_HEX_MAX;
    for (size_t i = 0; i < lim && hp + 4 < (int)sizeof(hex); i++)
        hp += snprintf(hex + hp, sizeof(hex) - hp, "%s%02X", i ? " " : "", b[i]);
    if (n > lim) hp += snprintf(hex + hp, sizeof(hex) - hp, " +%u", (unsigned)(n - lim));
    LOG_DBG("modbus", "%s [%s]", dir, hex);
}

/* Build the request frame for currentAction into buf and start transmitting it.
 * Called both for a fresh action (from MODBUS_IDLE) and for a retransmit (from
 * MODBUS_DEADTIME).  Leaves the bus in MODBUS_TRANSMITTING. */
static void start_action_tx(void) {
    size_t sz;
    if (currentAction.action == ACTION_WRITE_MULTI_REG) {
        /* FC16: dev, fc, start(2), count(2), bytecount, data[...] */
        buf[0] = currentAction.device;
        buf[1] = currentAction.action;
        buf[2] = currentAction.wr_address >> 8;
        buf[3] = currentAction.wr_address & 0xFF;
        buf[4] = currentAction.wr_count >> 8;
        buf[5] = currentAction.wr_count & 0xFF;
        buf[6] = currentAction.wr_len;
        for (uint8_t i = 0; i < currentAction.wr_len; i++)
            buf[7 + i] = currentAction.wr_payload[i];
        sz = 7u + currentAction.wr_len;
    } else if (currentAction.action == ACTION_READ_WRITE_MULTI_REG) {
        /* FC23: dev, fc, read-start(2), read-count(2), write-start(2),
         * write-count(2), write-bytecount, write-data[...].  The gateway
         * executes the write before the read, so the response returns the
         * post-write read block (submit-N / collect-N-1 pipeline). */
        buf[0]  = currentAction.device;
        buf[1]  = currentAction.action;
        buf[2]  = currentAction.address >> 8;      /* read start */
        buf[3]  = currentAction.address & 0xFF;
        buf[4]  = currentAction.arg >> 8;          /* read count */
        buf[5]  = currentAction.arg & 0xFF;
        buf[6]  = currentAction.wr_address >> 8;    /* write start */
        buf[7]  = currentAction.wr_address & 0xFF;
        buf[8]  = currentAction.wr_count >> 8;      /* write count */
        buf[9]  = currentAction.wr_count & 0xFF;
        buf[10] = currentAction.wr_len;
        for (uint8_t i = 0; i < currentAction.wr_len; i++)
            buf[11 + i] = currentAction.wr_payload[i];
        sz = 11u + currentAction.wr_len;
    } else if (currentAction.action == ACTION_MBQ_ENQUEUE) {
        /* FC65: dev, fc, cmd[4]. No addresses or counts — the gateway picks the
         * slot and the status block it returns is always the whole thing. */
        buf[0] = currentAction.device;
        buf[1] = currentAction.action;
        for (uint8_t i = 0; i < MBQ_CMD_BYTES; i++)
            buf[2 + i] = currentAction.wr_payload[i];
        sz = 2u + MBQ_CMD_BYTES;
    } else if (currentAction.action == ACTION_MBQ_STATUS) {
        buf[0] = currentAction.device;   /* FC66: dev, fc — nothing else to say */
        buf[1] = currentAction.action;
        sz = 2;
    } else {
        buf[0] = currentAction.device;
        buf[1] = currentAction.action;
        buf[2] = currentAction.address >> 8;
        buf[3] = currentAction.address & 0xFF;
        buf[4] = currentAction.arg >> 8;
        buf[5] = currentAction.arg & 0xFF;
        sz = 6;
    }

    readTimeout = make_timeout_time_ms(MODBUS_RESPONSE_TIMEOUT_MS);
    append_crc16(buf, sz);
    sz += 2;

    log_frame_hex("tx", buf, sz);

    while (uart_is_readable(UART_ID)) uart_getc(UART_ID);

    gpio_put(UART_DE_PIN, 1);

    /* Schedule DE release after all bytes transmitted. This alarm is the ONLY
     * exit from MODBUS_TRANSMITTING, so a scheduling failure (e.g. alarm pool
     * exhausted) must not be ignored — that would wedge the bus forever. If it
     * fails, fall back to releasing inline (it busy-waits for TX to drain). */
    bool de_scheduled =
        alarm_pool_add_alarm_in_us(alarm_pool_get_default(),
                                   (10ULL * 1000000ULL * sz / s_baud),
                                   release_de, NULL, true) > 0;
    for (size_t i = 0; i < sz; i++) {
        while (!uart_is_writable(UART_ID)) {}
        uart_get_hw(UART_ID)->dr = buf[i];
    }

    /* Do NOT enable RX here: while DE is asserted the receiver is muted (RE
     * tracks DE), and a half-duplex transceiver's RxD pin floats during the
     * transmit, so the controller can capacitively latch its OWN outgoing
     * frame.  release_de() turns RX on only after the line settles. */
    bus_state = MODBUS_TRANSMITTING;

    /* Fallback if the alarm couldn't be scheduled: release inline so we still
     * leave TRANSMITTING. Done after the bus_state write above so release_de's
     * transition to RECEIVING isn't clobbered. */
    if (!de_scheduled) {
        LOG_ERR("modbus", "DE-release alarm failed to schedule — releasing inline");
        release_de(0, NULL);   /* drains TX, drops DE, arms RX, -> RECEIVING */
    }
}

/* Charge the just-finished transaction's bus occupancy to its slave. Called on
 * every path that returns the SM to IDLE (normal completion and forced
 * recovery), so the accounting can't silently lose time. */
static void busload_account(void) {
    uint64_t us = (uint64_t)absolute_time_diff_us(s_bl_act_start, get_absolute_time());
    s_bl_busy_us += us;
    s_bl_txns++;
    uint8_t dev = currentAction.device;
    for (uint8_t i = 0; i < MODBUS_BUSLOAD_SLAVES; i++) {
        if (s_bl[i].txns && s_bl[i].slave != dev) continue;
        s_bl[i].slave = dev;
        s_bl[i].txns++;
        s_bl[i].us += us;
        return;
    }
    /* More distinct slaves than buckets: fold the overflow into the last one. */
    s_bl[MODBUS_BUSLOAD_SLAVES - 1].txns++;
    s_bl[MODBUS_BUSLOAD_SLAVES - 1].us += us;
}

void modbus_busload_reset(void) {
    memset(s_bl, 0, sizeof s_bl);
    s_bl_busy_us = 0;
    s_bl_txns    = 0;
    s_bl_retries = 0;
    s_bl_busy_retries = 0;
    s_bl_turn_sum = 0;
    s_bl_turn_n = 0;
    s_bl_turn_max = 0;
    s_bl_skipped = 0;
    s_bl_window  = get_absolute_time();
}

void modbus_busload_backoff(uint32_t *skipped, uint8_t *slaves_backed_off) {
    if (skipped) *skipped = s_bl_skipped;
    if (slaves_backed_off) {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MODBUS_BACKOFF_SLAVES; i++)
            if (s_bo[i].in_use && s_bo[i].backoff_ms) n++;
        *slaves_backed_off = n;
    }
}

void modbus_busload_retries(uint32_t *timeouts, uint32_t *device_busy) {
    if (timeouts)    *timeouts    = s_bl_retries;
    if (device_busy) *device_busy = s_bl_busy_retries;
}

void modbus_busload_turnaround(uint32_t *avg_us, uint32_t *max_us, uint32_t *n) {
    if (avg_us) *avg_us = s_bl_turn_n ? (uint32_t)(s_bl_turn_sum / s_bl_turn_n) : 0u;
    if (max_us) *max_us = s_bl_turn_max;
    if (n)      *n      = s_bl_turn_n;
}

void modbus_busload_get(uint64_t *window_us, uint64_t *busy_us, uint32_t *txns,
                        modbus_busload_slave_t *per) {
    if (window_us) *window_us = (uint64_t)absolute_time_diff_us(s_bl_window, get_absolute_time());
    if (busy_us)   *busy_us   = s_bl_busy_us;
    if (txns)      *txns      = s_bl_txns;
    if (per)       memcpy(per, s_bl, sizeof s_bl);
}

/* ── Poll loop ───────────────────────────────────────────────────────────── */
void modbus_poll(void) {
    /* Stuck-state watchdog: a transaction must complete within MODBUS_OP_MAX_MS.
     * If the SM has sat in any non-IDLE state past its deadline it is wedged;
     * force it back to IDLE so commands resume. Deliver a failure (res=0) to the
     * in-flight callback first so the producer's own state (e.g. dirty_poll's
     * AWAITING_*) is released too. This is the catch-all cure for the "stops
     * sending, needs a reboot" failure. */
    if (bus_state != MODBUS_IDLE && time_reached(s_op_deadline)) {
        s_recover_count++;
        LOG_ERR("modbus", "bus SM wedged in state %d >%ums — forcing recovery "
                "(slave=%u fc=0x%02x, total recoveries=%lu)",
                (int)bus_state, MODBUS_OP_MAX_MS,
                currentAction.device, (unsigned)currentAction.action,
                (unsigned long)s_recover_count);
        uart_set_irqs_enabled(UART_ID, false, false);
        gpio_put(UART_DE_PIN, 0);
        modbus_action_cb_t cb = currentAction.callback;
        currentAction.callback = NULL;          /* one-shot; don't double-fire */
        busload_account();
        bus_state = MODBUS_IDLE;
        if (cb) cb(0, buf, currentAction.ctx);  /* deliver as a timeout */
        return;
    }

    /* Periodic health summary: surface drops/recoveries even if each individual
     * event got rate-limited away, and report queue depth so a building backlog
     * is visible BEFORE it overflows. Only logs when something is non-zero. */
    {
        static uint32_t last_report_ms = 0;
        static uint32_t last_drops = 0, last_recov = 0;
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if (now - last_report_ms >= 10000u) {
            last_report_ms = now;
            uint32_t depth = (uint32_t)queue_get_level(&action_queue);
            if (s_drop_count != last_drops || s_recover_count != last_recov || depth > 32) {
                LOG_WARN("modbus", "health: qdepth=%lu drops=%lu recoveries=%lu",
                         (unsigned long)depth, (unsigned long)s_drop_count,
                         (unsigned long)s_recover_count);
                last_drops = s_drop_count;
                last_recov = s_recover_count;
            }
        }
    }

    switch (bus_state) {

    case MODBUS_IDLE:
        if (queue_try_remove(&action_queue, &currentAction)) {
            /* Slave is in backoff: complete as a timeout without transmitting.
             * Identical outcome for the caller, zero bus cost. */
            bool is_probe = false;
            if (backoff_should_skip(currentAction.device, &is_probe)) {
                s_bl_skipped++;
                if (currentAction.callback)
                    currentAction.callback(0, buf, currentAction.ctx);
                break;
            }
            s_bl_act_start  = get_absolute_time();
            /* A backoff PROBE gets ONE attempt, not the full retry budget. Its
             * only question is "are you back?", which a single try answers — and
             * the retries are what make a dead slave expensive: 3 attempts x the
             * 50 ms response timeout is ~186 ms of bus that any user command
             * arriving behind it must wait out. Measured: this alone was the
             * whole remaining latency tail (247 ms worst case) once the backoff
             * had removed the frequent hits. */
            s_attempts_left = is_probe ? 0u : MODBUS_MAX_RETRIES;
            s_busy_left     = DALI_BUSY_RETRIES;
            s_retry_pending = false;
            s_op_deadline   = make_timeout_time_ms(MODBUS_OP_MAX_MS);
            log_action_intent(&currentAction);
            start_action_tx();
        }
        break;

    case MODBUS_TRANSMITTING:
        break;

    case MODBUS_RECEIVING:
        /* Frame ends at t3.5 of silence, per §2.5.1.1 — not at the first byte
         * offset where the running CRC happens to reach zero. */
        if (readbuf_sz > 0 && time_reached(rx_silence_at)) {
            bus_state = MODBUS_RECEIVED_RESPONSE;   /* silence closed the frame */
        } else if (absolute_time_diff_us(readTimeout, get_absolute_time()) <= 0) {
            break;                                  /* still within the stall guard */
        } else {
            readbuf_sz = 0;                         /* stalled: no valid frame */
        }
        /* fall through */

    case MODBUS_RECEIVED_RESPONSE: {
        uart_set_irqs_enabled(UART_ID, false, false);

        /* Log the raw response bytes (mirrors the tx dump). Covers both normal
         * and exception replies; a timeout leaves readbuf_sz == 0 (nothing). */
        if (readbuf_sz > 0) {
            log_frame_hex("rx", buf, readbuf_sz);
            /* Slave turnaround: how long this slave took to answer, measured from
             * the moment our last byte cleared the line. Against a 50 ms response
             * timeout this is the margin — a slave creeping up on it produces
             * "lost" replies that are really late ones, and the retry then finds
             * its own earlier write already applied. */
            uint64_t t = (uint64_t)absolute_time_diff_us(s_bl_rx_start, get_absolute_time());
            uint32_t t32 = (t > 0xFFFFFFFFull) ? 0xFFFFFFFFu : (uint32_t)t;
            s_bl_turn_sum += t32;
            s_bl_turn_n++;
            if (t32 > s_bl_turn_max) s_bl_turn_max = t32;
        }

        bool retry = false;
        bool busy_retry = false;
        if (readbuf_sz == 0) {
            /* No valid reply.  Retry the same request if attempts remain,
             * otherwise deliver the failure (length 0) to the caller. */
            if (s_attempts_left > 0) {
                s_attempts_left--;
                retry = true;
                s_bl_retries++;
                LOG_WARN("modbus",
                         "rx timeout slave=%u fc=0x%02x — retrying (%u left)",
                         currentAction.device, (unsigned)currentAction.action,
                         (unsigned)s_attempts_left);
            } else {
                LOG_WARN("modbus", "rx timeout slave=%u fc=0x%02x (gave up)",
                         currentAction.device, (unsigned)currentAction.action);
                backoff_note(currentAction.device, false);
                if (currentAction.callback)
                    currentAction.callback(0, buf, currentAction.ctx);
            }
        } else if (readbuf_sz >= 2 && (buf[1] & 0x80)) {
            uint8_t exc = (readbuf_sz >= 3) ? buf[2] : 0xFFu;
            if (exc == 0x06u && s_busy_left > 0) {
                /* gateway mid-DALI-frame: wait ~a frame time and poll again */
                s_busy_left--;
                retry = true;
                busy_retry = true;
                s_bl_busy_retries++;
                LOG_DBG("modbus", "slave=%u fc=0x%02x busy — retrying (%u left)",
                        currentAction.device, (unsigned)currentAction.action,
                        (unsigned)s_busy_left);
            } else {
                backoff_note(currentAction.device, true);   /* alive, just disagreeing */
                LOG_WARN("modbus", "rx exception slave=%u fc=0x%02x code=0x%02x (%s)",
                         currentAction.device, (unsigned)currentAction.action,
                         (unsigned)exc, modbus_exception_str(exc));
                if (currentAction.callback)
                    currentAction.callback(-(int)readbuf_sz, buf, currentAction.ctx);
            }
        } else {
            LOG_DBG("modbus", "rx ok slave=%u fc=0x%02x bytes=%d",
                    currentAction.device, (unsigned)currentAction.action,
                    (int)readbuf_sz);
            backoff_note(currentAction.device, true);
            if (currentAction.callback)
                currentAction.callback((int)readbuf_sz, buf, currentAction.ctx);
        }

        s_retry_pending = retry;
        bus_state = MODBUS_DEADTIME;
        /* fix #8: normal deadtime = 1.5 char times (min 750 us).  Before a
         * retry, wait the full 3.5-char inter-frame silence (min 2 ms) so the
         * slave's receiver resets any partial frame from the failed attempt. */
        {
            uint32_t dt, floor;
            if (busy_retry)  { dt = DALI_BUSY_RETRY_US; floor = DALI_BUSY_RETRY_US; }
            else if (retry)  { dt = 35000000u / s_baud; floor = 2000u; }
            else             { dt = 15000000u / s_baud; floor = 750u; }
            if (dt < floor) dt = floor;
            readTimeout = make_timeout_time_us((uint64_t)dt);
        }
        break;
    }

    case MODBUS_DEADTIME:
        if (absolute_time_diff_us(readTimeout, get_absolute_time()) > 0) {
            if (s_retry_pending) {
                s_retry_pending = false;
                start_action_tx();   /* re-send the same request */
            } else {
                busload_account();
                bus_state = MODBUS_IDLE;
            }
        }
        break;
    }
}

/* ── Action enqueuers ────────────────────────────────────────────────────── */
static inline bool enqueue(modbus_action_t *act) {
    if (queue_try_add(&action_queue, act)) return true;
    s_drop_count++;
    /* Rate-limited so a sustained overflow can't itself flood the log / stall the
     * CDC. The count is the real signal; surfaced via modbus_drop_count(). */
    if ((s_drop_count & 0x3F) == 1)
        LOG_WARN("modbus", "action queue FULL — dropped fc=0x%02x slave=%u addr=%u "
                 "(total dropped=%lu)", (unsigned)act->action, act->device,
                 act->address, (unsigned long)s_drop_count);
    return false;
}

uint32_t modbus_drop_count(void) { return s_drop_count; }

bool modbus_get_coils(uint8_t dev, uint16_t addr, uint16_t count,
                      modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_GET_COILS, .device = dev,
                          .address = addr, .arg = count, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

bool modbus_toggle(uint8_t dev, uint16_t addr,
                   modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_SET_COIL, .device = dev,
                          .address = addr, .arg = 0x5500, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

bool modbus_set_coil(uint8_t dev, uint16_t addr, bool is_on,
                     modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_SET_COIL, .device = dev, .address = addr,
                          .arg = is_on ? 0xFF00u : 0x0000u, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

bool modbus_get_holding_registers(uint8_t dev, uint16_t addr, uint16_t count,
                                   modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_GET_HOLDING_REGISTERS, .device = dev,
                          .address = addr, .arg = count, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

bool modbus_set_holding_register(uint8_t dev, uint16_t addr, uint16_t value,
                                  modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_SET_HOLDING_REGISTER, .device = dev,
                          .address = addr, .arg = value, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

/* New: FC04 — same frame structure as FC03 */
bool modbus_get_input_registers(uint8_t dev, uint16_t addr, uint16_t count,
                                 modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_GET_INPUT_REGISTERS, .device = dev,
                          .address = addr, .arg = count, .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

/* New: FC23 Read/Write Multiple Registers.  The slave writes wr_count registers
 * at wr_addr (data = big-endian bytes), THEN reads rd_count registers at rd_addr;
 * the response carries the post-write read block.  wr_count may be 0 for a
 * read-only FC23 (some slaves reject that; the gateway accepts it). */
bool modbus_mbq_enqueue(uint8_t dev, const uint8_t cmd[MBQ_CMD_BYTES],
                        modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_MBQ_ENQUEUE, .device = dev,
                          .wr_len = MBQ_CMD_BYTES, .callback = cb, .ctx = ctx };
    memcpy(a.wr_payload, cmd, MBQ_CMD_BYTES);
    return enqueue(&a);
}

bool modbus_mbq_status(uint8_t dev, modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { .action = ACTION_MBQ_STATUS, .device = dev,
                          .callback = cb, .ctx = ctx };
    return enqueue(&a);
}

bool modbus_read_write_multiple_registers(uint8_t dev,
                                          uint16_t rd_addr, uint16_t rd_count,
                                          uint16_t wr_addr, uint16_t wr_count,
                                          const uint8_t *wr_data,
                                          modbus_action_cb_t cb, void *ctx) {
    if (rd_count == 0) return false;
    if ((size_t)wr_count * 2u > MODBUS_WR_PAYLOAD_MAX) return false;
    modbus_action_t a = {
        .action = ACTION_READ_WRITE_MULTI_REG, .device = dev,
        .address = rd_addr, .arg = rd_count,
        .wr_address = wr_addr, .wr_count = wr_count,
        .wr_len = (uint8_t)(wr_count * 2u),
        .callback = cb, .ctx = ctx,
    };
    if (wr_count) memcpy(a.wr_payload, wr_data, (size_t)wr_count * 2u);
    return enqueue(&a);
}
