/*
 * modbus.c — Modbus RTU master for the switch gateway.
 *
 * Architecture is unchanged from the original: ISR-driven RX with an alarm
 * for DE release timing, non-blocking poll loop.  Changes from original:
 *   - ACTION_DALI (0x44) removed; TinyModbus uses standard FC04/FC06
 *   - FC04 Read Input Registers added (ACTION_GET_INPUT_REGISTERS)
 *   - Baud rate is configurable, defaulting to 19200
 *   - Baud rate and DE mode are both persisted to flash filesystem
 */

#include "modbus.h"
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
} modbus_action_type_t;

typedef struct {
    modbus_action_type_t action;
    uint8_t              device;
    uint16_t             address;
    uint16_t             arg;
    modbus_action_cb_t   callback;
    void                *ctx;
} modbus_action_t;

static modbus_de_mode_t      de_mode    = MODBUS_DE_WHENTX;
static uint32_t              s_baud     = DEFAULT_BAUD_RATE;
static volatile bool         transmitting;
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
static bool                  s_retry_pending;

static bool modbus_rx_valid(void) { return true; }

static const char *modbus_de_mode_str[MODBUS_DE_MAX] = { "tx", "always" };

/* Emit the *intent* of an action at INFO level (what we're asking the slave
 * to do), separate from the raw tx byte dump which lives at DEBUG. */
static void log_action_intent(const modbus_action_t *a) {
    switch (a->action) {
    case ACTION_GET_COILS:
        LOG_INFO("modbus", "read coils slave=%u addr=%u count=%u",
                 a->device, a->address, a->arg);
        break;
    case ACTION_GET_HOLDING_REGISTERS:
        LOG_INFO("modbus", "read holding registers slave=%u addr=%u count=%u",
                 a->device, a->address, a->arg);
        break;
    case ACTION_GET_INPUT_REGISTERS:
        LOG_INFO("modbus", "read input registers slave=%u addr=%u count=%u",
                 a->device, a->address, a->arg);
        break;
    case ACTION_SET_COIL:
        LOG_INFO("modbus", "set coil slave=%u addr=%u -> %s",
                 a->device, a->address,
                 a->arg == 0x5500u ? "toggle" : (a->arg ? "on" : "off"));
        break;
    case ACTION_SET_HOLDING_REGISTER:
        LOG_INFO("modbus", "set holding register slave=%u addr=%u value=%u",
                 a->device, a->address, a->arg);
        break;
    default:
        LOG_INFO("modbus", "action 0x%02x slave=%u addr=%u arg=%u",
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
        readTimeout = make_timeout_time_us(7500);
        uint8_t ch  = uart_getc(UART_ID);
        if (readbuf_sz < readbuf_max) {
            *readptr++ = ch;
            readbuf_sz++;
            tmp          = ch ^ readbuf_crc;
            readbuf_crc >>= 8;
            readbuf_crc ^= crcTable[tmp];
        }
        if (readbuf_crc == 0 && modbus_rx_valid()) {
            bus_state = MODBUS_RECEIVED_RESPONSE;
        }
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
        ;
    }
    if (de_mode == MODBUS_DE_WHENTX) {
        gpio_put(UART_DE_PIN, 0);
    }
    bus_state = MODBUS_RECEIVING;
    return 0;
}

/* ── Config helpers ──────────────────────────────────────────────────────── */
static char *trimwhitespace(char *str) {
    char *end;
    while (isspace((unsigned char)*str)) str++;
    if (*str == 0) return str;
    end = str + strlen(str) - 1;
    while (end > str && isspace((unsigned char)*end)) end--;
    end[1] = '\0';
    return str;
}

static modbus_de_mode_t parseDeMode(const char *str) {
    for (modbus_de_mode_t mode = 0; mode < MODBUS_DE_MAX; mode++) {
        if (strcmp(str, modbus_de_mode_str[mode]) == 0) return mode;
    }
    LOG_WARN("modbus", "unknown DE mode '%s' — assuming tx", str);
    return MODBUS_DE_WHENTX;
}

void set_modbus_de_mode(modbus_de_mode_t mode) {
    de_mode    = mode;
    FILE *fp   = fopen("/modbus_config.txt", "w");
    if (fp) {
        fprintf(fp, "%s\n", modbus_de_mode_str[mode]);
        fclose(fp);
    }
}

modbus_de_mode_t modbus_get_de_mode(void) { return de_mode; }

const char *modbus_de_mode_to_str(modbus_de_mode_t mode) {
    if (mode >= MODBUS_DE_MAX) return "?";
    return modbus_de_mode_str[mode];
}

void modbus_set_baud(uint32_t baud) {
    s_baud   = baud;
    FILE *fp = fopen("/modbus_baud.txt", "w");
    if (fp) {
        fprintf(fp, "%lu\n", (unsigned long)baud);
        fclose(fp);
    }
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

    /* Load DE mode */
    char tmp_buf[32];
    FILE *fp = fopen("/modbus_config.txt", "r");
    if (fp) {
        size_t n = fread(tmp_buf, 1, sizeof(tmp_buf) - 1, fp);
        tmp_buf[n] = '\0';
        fclose(fp);
        de_mode = parseDeMode(trimwhitespace(tmp_buf));
    } else {
        de_mode = MODBUS_DE_WHENTX;
    }

    /* Load baud rate */
    fp = fopen("/modbus_baud.txt", "r");
    if (fp) {
        unsigned long b = DEFAULT_BAUD_RATE;
        fscanf(fp, "%lu", &b);
        fclose(fp);
        s_baud = (uint32_t)b;
    } else {
        s_baud = DEFAULT_BAUD_RATE;
    }

    LOG_INFO("modbus", "init de_mode=%s baud=%lu",
             modbus_de_mode_str[de_mode], (unsigned long)s_baud);

    gpio_init(UART_DE_PIN);
    gpio_put(UART_DE_PIN, 0);
    gpio_set_dir(UART_DE_PIN, GPIO_OUT);

    uart_init(UART_ID, s_baud);
    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;
    irq_set_exclusive_handler(UART_IRQ, uart_irq);
    irq_set_enabled(UART_IRQ, true);
}

/* Build the request frame for currentAction into buf and start transmitting it.
 * Called both for a fresh action (from MODBUS_IDLE) and for a retransmit (from
 * MODBUS_DEADTIME).  Leaves the bus in MODBUS_TRANSMITTING. */
static void start_action_tx(void) {
    buf[0] = currentAction.device;
    buf[1] = currentAction.action;
    buf[2] = currentAction.address >> 8;
    buf[3] = currentAction.address & 0xFF;
    buf[4] = currentAction.arg >> 8;
    buf[5] = currentAction.arg & 0xFF;
    size_t sz = 6;

    readTimeout = make_timeout_time_ms(MODBUS_RESPONSE_TIMEOUT_MS);
    append_crc16(buf, sz);
    sz += 2;

    char hex[3 * 16 + 1];
    int hp = 0;
    for (size_t i = 0; i < sz && hp + 3 < (int)sizeof(hex); i++) {
        hp += snprintf(hex + hp, sizeof(hex) - hp,
                       "%s%02X", i ? " " : "", buf[i]);
    }
    LOG_DBG("modbus", "tx [%s]", hex);

    while (uart_is_readable(UART_ID)) uart_getc(UART_ID);

    gpio_put(UART_DE_PIN, 1);
    transmitting = true;

    /* Schedule DE release after all bytes transmitted */
    alarm_pool_add_alarm_in_us(alarm_pool_get_default(),
                               (10ULL * 1000000ULL * sz / s_baud),
                               release_de, NULL, true);
    for (size_t i = 0; i < sz; i++) {
        while (!uart_is_writable(UART_ID)) {}
        uart_get_hw(UART_ID)->dr = buf[i];
    }

    readptr     = buf;
    readbuf_sz  = 0;
    readbuf_max = sizeof(buf);
    readbuf_crc = 0xFFFF;
    uart_set_irqs_enabled(UART_ID, true, false);
    bus_state = MODBUS_TRANSMITTING;
}

/* ── Poll loop ───────────────────────────────────────────────────────────── */
void modbus_poll(void) {
    switch (bus_state) {

    case MODBUS_IDLE:
        if (queue_try_remove(&action_queue, &currentAction)) {
            s_attempts_left = MODBUS_MAX_RETRIES;
            s_retry_pending = false;
            log_action_intent(&currentAction);
            start_action_tx();
        }
        break;

    case MODBUS_TRANSMITTING:
        break;

    case MODBUS_RECEIVING:
        if (absolute_time_diff_us(readTimeout, get_absolute_time()) <= 0) {
            break;
        }
        readbuf_sz = 0;   /* timed out with no valid frame */
        /* fall through */

    case MODBUS_RECEIVED_RESPONSE: {
        uart_set_irqs_enabled(UART_ID, false, false);

        bool retry = false;
        if (readbuf_sz == 0) {
            /* No valid reply.  Retry the same request if attempts remain,
             * otherwise deliver the failure (length 0) to the caller. */
            if (s_attempts_left > 0) {
                s_attempts_left--;
                retry = true;
                LOG_WARN("modbus",
                         "rx timeout slave=%u fc=0x%02x — retrying (%u left)",
                         currentAction.device, (unsigned)currentAction.action,
                         (unsigned)s_attempts_left);
            } else {
                LOG_WARN("modbus", "rx timeout slave=%u fc=0x%02x (gave up)",
                         currentAction.device, (unsigned)currentAction.action);
                if (currentAction.callback)
                    currentAction.callback(0, buf, currentAction.ctx);
            }
        } else if (readbuf_sz >= 2 && (buf[1] & 0x80)) {
            uint8_t exc = (readbuf_sz >= 3) ? buf[2] : 0xFFu;
            LOG_WARN("modbus", "rx exception slave=%u fc=0x%02x code=0x%02x",
                     currentAction.device, (unsigned)currentAction.action,
                     (unsigned)exc);
            if (currentAction.callback)
                currentAction.callback(-(int)readbuf_sz, buf, currentAction.ctx);
        } else {
            LOG_INFO("modbus", "rx ok slave=%u fc=0x%02x bytes=%d",
                     currentAction.device, (unsigned)currentAction.action,
                     (int)readbuf_sz);
            if (currentAction.callback)
                currentAction.callback((int)readbuf_sz, buf, currentAction.ctx);
        }

        s_retry_pending = retry;
        bus_state = MODBUS_DEADTIME;
        /* fix #8: normal deadtime = 1.5 char times (min 750 us).  Before a
         * retry, wait the full 3.5-char inter-frame silence (min 2 ms) so the
         * slave's receiver resets any partial frame from the failed attempt. */
        {
            uint32_t dt    = retry ? (35000000u / s_baud) : (15000000u / s_baud);
            uint32_t floor = retry ? 2000u : 750u;
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
                bus_state = MODBUS_IDLE;
            }
        }
        break;
    }
}

/* ── Action enqueuers ────────────────────────────────────────────────────── */
static inline void enqueue(modbus_action_t *act) {
    if (!queue_try_add(&action_queue, act)) {
        LOG_WARN("modbus", "queue full — dropped action");
    }
}

void modbus_get_coils(uint8_t dev, uint16_t addr, uint16_t count,
                      modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { ACTION_GET_COILS, dev, addr, count, cb, ctx };
    enqueue(&a);
}

void modbus_toggle(uint8_t dev, uint16_t addr,
                   modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { ACTION_SET_COIL, dev, addr, 0x5500, cb, ctx };
    enqueue(&a);
}

void modbus_set_coil(uint8_t dev, uint16_t addr, bool is_on,
                     modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = {
        ACTION_SET_COIL, dev, addr,
        is_on ? 0xFF00u : 0x0000u, cb, ctx
    };
    enqueue(&a);
}

void modbus_get_holding_registers(uint8_t dev, uint16_t addr, uint16_t count,
                                   modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { ACTION_GET_HOLDING_REGISTERS, dev, addr, count, cb, ctx };
    enqueue(&a);
}

void modbus_set_holding_register(uint8_t dev, uint16_t addr, uint16_t value,
                                  modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { ACTION_SET_HOLDING_REGISTER, dev, addr, value, cb, ctx };
    enqueue(&a);
}

/* New: FC04 — same frame structure as FC03 */
void modbus_get_input_registers(uint8_t dev, uint16_t addr, uint16_t count,
                                 modbus_action_cb_t cb, void *ctx) {
    modbus_action_t a = { ACTION_GET_INPUT_REGISTERS, dev, addr, count, cb, ctx };
    enqueue(&a);
}
