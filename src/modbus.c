#include "hardware/uart.h"
#include "hardware/gpio.h"
#include <FreeRTOS.h>
#include <pico/time.h>
#include "projdefs.h"
#include "stdio.h"
#include <semphr.h>
#include <sys/types.h>
#include <task.h>
#include <stdbool.h>
#include "modbus.h"
#include "entity_status.h"

static TaskHandle_t notificationTask;

#define MODBUS_RESPONSE_TIMEOUT_TICKS pdMS_TO_TICKS(500)

#define RX_TASK_INDEX 1
#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5
#define UART_DE_PIN 1
static volatile bool transmitting;
static uint8_t *readbuf;
static uint8_t *readptr;
static size_t readbuf_sz;
static size_t readbuf_max;
static uint16_t readbuf_crc;

static bool modbus_rx_valid() { return true; }

static const uint16_t crcTable[] = {
    0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241, 0XC601,
    0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440, 0XCC01, 0X0CC0,
    0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40, 0X0A00, 0XCAC1, 0XCB81,
    0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841, 0XD801, 0X18C0, 0X1980, 0XD941,
    0X1B00, 0XDBC1, 0XDA81, 0X1A40, 0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01,
    0X1DC0, 0X1C80, 0XDC41, 0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0,
    0X1680, 0XD641, 0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081,
    0X1040, 0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
    0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441, 0X3C00,
    0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41, 0XFA01, 0X3AC0,
    0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840, 0X2800, 0XE8C1, 0XE981,
    0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41, 0XEE01, 0X2EC0, 0X2F80, 0XEF41,
    0X2D00, 0XEDC1, 0XEC81, 0X2C40, 0XE401, 0X24C0, 0X2580, 0XE541, 0X2700,
    0XE7C1, 0XE681, 0X2640, 0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0,
    0X2080, 0XE041, 0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281,
    0X6240, 0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
    0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41, 0XAA01,
    0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840, 0X7800, 0XB8C1,
    0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41, 0XBE01, 0X7EC0, 0X7F80,
    0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40, 0XB401, 0X74C0, 0X7580, 0XB541,
    0X7700, 0XB7C1, 0XB681, 0X7640, 0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101,
    0X71C0, 0X7080, 0XB041, 0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0,
    0X5280, 0X9241, 0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481,
    0X5440, 0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
    0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841, 0X8801,
    0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40, 0X4E00, 0X8EC1,
    0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41, 0X4400, 0X84C1, 0X8581,
    0X4540, 0X8701, 0X47C0, 0X4680, 0X8641, 0X8201, 0X42C0, 0X4380, 0X8341,
    0X4100, 0X81C1, 0X8081, 0X4040};

void append_crc16(uint8_t *buf, size_t sz) {
  uint8_t tmp;
  uint16_t crcVal = 0xFFFF;

  while (sz--) {
    tmp = *buf++ ^ crcVal;
    crcVal >>= 8;
    crcVal ^= crcTable[tmp];
  }
  *buf++ = crcVal;
  *buf++ = crcVal >> 8;
}

void uart_irq() {
  uint8_t tmp;

  // This will go off every 4 characters, or when 32 bit periods of no data
  // have occurred which corresponds to 3.33 ms @9600 baud
  while (uart_is_readable(UART_ID)) {
    uint8_t ch = uart_getc(UART_ID);
    if (readbuf_sz < readbuf_max) {
      *readptr++ = ch;
      readbuf_sz++;

      // Update the CRC16 of our read.
      tmp = ch ^ readbuf_crc;
      readbuf_crc >>= 8;
      readbuf_crc ^= crcTable[tmp];
    }

    if (readbuf_crc == 0 && modbus_rx_valid()) {
      BaseType_t higherPriorityWoken = pdFALSE;
      // Notify the calling task that we a valid response
      vTaskNotifyGiveIndexedFromISR(notificationTask, RX_TASK_INDEX,
                                    &higherPriorityWoken);
      portYIELD_FROM_ISR(higherPriorityWoken);
    }
  }
}

static QueueHandle_t action_queue;

typedef enum {
  ACTION_GET_COILS = 0x01,
  ACTION_GET_HOLDING_REGISTERS = 0x03,
  ACTION_SET_COIL = 0x05,
  ACTION_SET_HOLDING_REGISTER = 0x06,
  ACTION_DALI = 0x44,
} modbus_action_type_t;


typedef struct {
  modbus_action_type_t action;
  uint8_t device;
  uint16_t address;
  uint16_t arg;
  modbus_action_cb_t callback;
  action_ctx_t callbackarg;
} modbus_action_t;




/**
 * This must run on the same priority task as the idle task, otherwise whenever
 * it yeilds it will simply come back here.
 */
int modbus_send(uint8_t *pkt, size_t max_sz, size_t sz, unsigned timeoutTicks) {
  uint16_t crc = 0xFFFF;

  append_crc16(pkt, sz);
  sz += 2;

  // Clear the RX buffer, in case some random stuff arrived
  while (uart_is_readable(UART_ID)) {
    uart_getc(UART_ID);
  }

  // Also clear any pending task notifications
  xTaskNotifyStateClearIndexed(NULL, RX_TASK_INDEX);
  notificationTask = xTaskGetCurrentTaskHandle();

  // Set DE
  gpio_put(UART_DE_PIN, 1);
  transmitting = true;

  uart_set_irqs_enabled(UART_ID, true, false);

  // Transmit the outbound command
  for (int i = 0; i < sz; i++) {
    while (!uart_is_writable(UART_ID)) {
      // In most cases the TX fifo will be able to completely contain our
      // command, but just in case, give somebody else a chance while we wait
      // for the queue to drain.
      vPortYield();
    }
    // Send a character
    uart_get_hw(UART_ID)->dr = pkt[i];
  }
  // There is no reliable way to get notified when the transmit buffer is
  // complete, other than to poll it. Instead, we we wait until its almost done,
  // then do some last minute polling.  This ain't pretty.
  vTaskDelay(pdMS_TO_TICKS(10000 * sz / BAUD_RATE));
  // Do a busy wait loop waiting for the transmit to complete.  Hopefully it will be less than 1 tick.
  while (uart_get_hw(UART_ID)->fr & UART_UARTFR_BUSY_BITS) {
    vPortYield();
  }
  // gpio_put(UART_DE_PIN, 0);

  // Get ready to read results
  readbuf = readptr = pkt;
  readbuf_sz = 0;
  readbuf_max = max_sz;
  readbuf_crc = 0xFFFF;


  // Wait for IRQ to tell us we have a valid packet, or we time out
  if (ulTaskNotifyTakeIndexed(RX_TASK_INDEX, pdTRUE, timeoutTicks) == 0) {
    // We timed out before we got a valid packet
    readbuf_sz = 0;
  }
  uart_set_irqs_enabled(UART_ID, false, false);
  if (readbuf[1] & 0x80) {
    // Response was an error - return a negative response.
    readbuf_sz = -readbuf_sz;
  }

  // Unlock
  return readbuf_sz;
}



void modbus_task(void *data) {
  uint8_t buf[256];
  size_t sz;
  size_t bufoffset;
  int ret;


  action_queue = xQueueCreate(256, sizeof(modbus_action_t));


  gpio_init(UART_DE_PIN);
  gpio_put(UART_DE_PIN, 0);
  gpio_set_dir(UART_DE_PIN, GPIO_OUT);

  uart_init(UART_ID, BAUD_RATE);

  // Set the TX and RX pins by using the function select on the GPIO
  // Set datasheet for more information on function select
  gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
  gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

  // We need to set up the handler first
  // Select correct interrupt for the UART we are using
  int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

  // And set up and enable the interrupt handlers
  irq_set_exclusive_handler(UART_IRQ, uart_irq);
  irq_set_enabled(UART_IRQ, true);

  while (true) {
    modbus_action_t action;

    if (xQueueReceive(action_queue, &action, portMAX_DELAY) == pdTRUE) {
      bufoffset = 0;
      buf[0] = action.device;
      buf[1] = action.action;
      buf[2] = action.address >> 8;
      buf[3] = action.address;
      if (action.action == ACTION_DALI) {
        buf[4] = action.arg;
        sz = 5;
      } else {
        buf[4] = action.arg >> 8;
        buf[5] = action.arg;
        sz = 6;
      }
      // printf("sending %02x:%02x:%02x%02x:%02x%02x (%d bytes)\n", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], sz);
      ret = modbus_send(buf, sizeof(buf), sz, pdMS_TO_TICKS(500));
      if (action.callback) {
        action.callback(ret, buf, action.callbackarg);
      }
    }
    // Minimum delay between receipt and transmission of new frame.
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  vTaskDelete(NULL);
}


void modbus_get_coils(uint8_t device_id, uint16_t addr, uint16_t count, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_GET_COILS,
    .address = addr,
    .arg = count,
    .callback = cb,
    .callbackarg = callbackarg
  };
  xQueueSend(action_queue, &act, portMAX_DELAY);
}



void modbus_toggle(uint8_t device_id, uint16_t addr, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_SET_COIL,
    .address = addr,
    .arg = 0x5500,
    .callback = cb,
    .callbackarg = callbackarg
  };
  xQueueSend(action_queue, &act, portMAX_DELAY);
}

void modbus_set_coil(uint8_t device_id, uint16_t addr, bool is_on, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_SET_COIL,
    .address = addr,
    .arg = is_on ? 0xFF00 : 0x0000,
    .callback = cb,
    .callbackarg = callbackarg
  };
  xQueueSend(action_queue, &act, portMAX_DELAY);
}


void modbus_get_holding_registers(uint8_t device_id, uint16_t addr, uint16_t count, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_GET_HOLDING_REGISTERS,
    .address = addr,
    .arg = count,
    .callback = cb,
    .callbackarg = callbackarg
  };
  // printf("Fetching holding registers %d, %d, %d\n", act.device, act.address, act.arg);
  xQueueSend(action_queue, &act, portMAX_DELAY);
}

void modbus_set_holding_register(uint8_t device_id, uint16_t addr, uint16_t value, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_SET_HOLDING_REGISTER,
    .address = addr,
    .arg = value,
    .callback = cb,
    .callbackarg = callbackarg
  };
  xQueueSend(action_queue, &act, portMAX_DELAY);
}




void modbus_dali(uint8_t device_id, uint16_t cmd, uint8_t repeat, modbus_action_cb_t cb, action_ctx_t callbackarg) {
  modbus_action_t act = {
    .device = device_id,
    .action = ACTION_DALI,
    .address = cmd,
    .arg = repeat,
    .callback = cb,
    .callbackarg = callbackarg
  };
  xQueueSend(action_queue, &act, portMAX_DELAY);
}



void modbus_fade(uint8_t device_id, uint16_t addr, int direction, modbus_action_cb_t cb, action_ctx_t callbackarg) {
    return modbus_dali(device_id, (direction >= 0 ? 0x108 : 0x107) | (addr << 9), 1, cb, callbackarg);
}
