#include "dali_provision.h"
#include "entity_status.h"
#include <FreeRTOS.h>
#include <stdlib.h>
#include <task.h>
#include <stdio.h>
#include <semphr.h>
#include <queue.h>
#include "modbus.h"
#include "portmacrocommon.h"
#include "projdefs.h"

#define NOT_FOUND 0x1000000
#define DALI_NAK -10
#define MODBUS_NAK -64
#define CORRUPT_RESULT -128

typedef struct {
    int device:8;
    int incremental:1;
} provision_cmd_args_t;

typedef struct {
    int modbus_device;
    bool incremental;
    int availableDeviceAddresses[64];
    int availableSz;
    SemaphoreHandle_t commandCompleteSem;
} provision_ctx_t;

static int result;

QueueHandle_t provisionQueue;

#define MIN_DALI_DEVICE 2
#define MAX_DALI_DEVICE 3

// One for device 2, one for device 3
provision_ctx_t currentTask = { .modbus_device = 0, .availableSz = 0 };


#define isDaliYes(x) ((x) == 0xFF)

void modbus_dali_callback(int status, uint8_t *data, action_ctx_t ctx) {
    switch (status) {
        case 5:
            // Normal response
            result = data[2];
            break;
        case -5:
            // Device returned a modbus error.
            result = -data[2];
            break;
        case 0:
            // No response at all - is the gateway even there?
            result = MODBUS_NAK;
            break;
        default:
            // Illegal response
            result = CORRUPT_RESULT;
            break;
    }
    xSemaphoreGive(currentTask.commandCompleteSem);
}

int awaitModbusResponse() {
    if (xSemaphoreTake(currentTask.commandCompleteSem, pdMS_TO_TICKS(1000)) == pdFALSE) {
        return -1;
    }
    return result;
}


static int syncDali(provision_ctx_t *ctx, uint16_t cmd, unsigned repeat) {
    action_ctx_t actx = {
        .device = ctx->modbus_device,
        .address = 0,
    };
    modbus_dali(ctx->modbus_device, cmd, repeat, modbus_dali_callback, actx);
    return awaitModbusResponse();
}



static bool anyDaliDevicesLessThanOrEqualTo(provision_ctx_t *ctx,uint32_t val) {
    // Set Search Address H
    syncDali(ctx, 0xb100 | ((val >> 16) & 0xff), 1);
    // Set Search Address M
    syncDali(ctx, 0xb300 | ((val >> 8) & 0xff), 1);
    // Set Search Address L
    syncDali(ctx, 0xb500 | (val & 0xff), 1);
    // Compare - Will return a value if true, or None/0xFF otherwise.
    return isDaliYes(syncDali(ctx, 0xa900, 1));
}    

static int findLowestRandomAddress(provision_ctx_t *ctx,  uint32_t low, uint32_t high) {
    // As a sanity check, make sure there's at least one device in initialisation mode, otherwise we'll be looking for ages.
    bool lessThanHigh = anyDaliDevicesLessThanOrEqualTo(ctx, high);
    if (!lessThanHigh) {
        return NOT_FOUND;
    }

    while (low <= high) {
        uint32_t testPoint = (low + high) / 2;
        bool lessThanMiddle = anyDaliDevicesLessThanOrEqualTo(ctx, testPoint);
        entity_status_print_provision_status(ctx->modbus_device, "Search %06x - %06x - %06x", low, testPoint, high);
        if (lessThanMiddle) {
            if (low == testPoint) {
                return low;
            }
            high = testPoint;
        } else { 
            // It must be above the middle value
            low = testPoint + 1;
        }
    }
    return NOT_FOUND;
}


static void clear_all_addresses(provision_ctx_t *ctx) {
    entity_status_print_provision_status(ctx->modbus_device, "Clearing all existing Short IDs");
    // Set DTR0 to 255
    syncDali(ctx, 0xa3ff, 1);
    // Broadcast clear Short address
    syncDali(ctx, 0xff80, 2);
}

static bool find_available_addresses(provision_ctx_t *ctx) {
    ctx->availableSz = 0;

    entity_status_print_provision_status(ctx->modbus_device, "checking for available device IDs");
    for (int addr = 0; addr < 64; addr++) {
        // Query Control Gear Present for each address.
        int present = syncDali(ctx, 0x0191 | (addr << 9), 1);
        printf("Present response for %d is %d\n", addr, present);
        if (present == MODBUS_NAK) {
            entity_status_print_provision_status(ctx->modbus_device, "DALI Gateway is not responding.  Assuming it is missing");
            return false;
        } else if (present == DALI_NAK) {
            ctx->availableDeviceAddresses[ctx->availableSz++] = addr;
            entity_status_print_provision_status(ctx->modbus_device, "%d addresses available for assignment", ctx->availableSz);
        } else {
            entity_status_print_provision_status(ctx->modbus_device, "Address %d is already used\n", addr);
        }
    }
    return true;
}


static void provision(provision_ctx_t *ctx) {
    entity_status_print_provision_status(ctx->modbus_device, "Starting %s provision", ctx->incremental ? "incremental" : "full");
    
    if (!ctx->incremental) {
        clear_all_addresses(ctx);
    }
    if (!find_available_addresses(ctx)) {
        goto cleanup;
    }
    
    // Terminate any existing provisioning processes.
    entity_status_print_provision_status(ctx->modbus_device, "Termninating previous provision, if any");
    syncDali(ctx, 0xa100, 1);

    // Initialise - with either None (0xFF) if incremental, or Broadcase if full
    entity_status_print_provision_status(ctx->modbus_device, "Initialising");
    syncDali(ctx, ctx->incremental ? 0xa5ff : 0xa500, 2);
    
    entity_status_print_provision_status(ctx->modbus_device, "Ranomising long addresses");
    syncDali(ctx, 0xa700, 2);

    
    // Give the devices a moment to randomise...
    vTaskDelay(pdMS_TO_TICKS(100));

    
    int32_t lowestRandomDeviceIdFound = -1;
    int addressIdx = 0;
    int devicesAdded = 0;
    while (lowestRandomDeviceIdFound < NOT_FOUND) {
        lowestRandomDeviceIdFound = findLowestRandomAddress(ctx, lowestRandomDeviceIdFound + 1, NOT_FOUND-1);

        if (lowestRandomDeviceIdFound < NOT_FOUND) {
            if (addressIdx >= ctx->availableSz) {
                printf("Ran out of available addresses after %d items\n", addressIdx);
                goto cleanup;
            }
            uint8_t deviceAddress = ctx->availableDeviceAddresses[addressIdx++];
            entity_status_print_provision_status(ctx->modbus_device, "Found device at random address %06x. Assigning address %d", lowestRandomDeviceIdFound, deviceAddress);

            // Program Short Address
            syncDali(ctx, 0xb701 | (deviceAddress << 1), 2);

            // Verify Short Address
            if (!isDaliYes(syncDali(ctx, 0xb901 | (deviceAddress << 1), 2))) {
                printf("Device %06x did not verify its address assignment of %d\n", lowestRandomDeviceIdFound, deviceAddress);
                goto cleanup;
            }
            // Withdraw the device we just assigned
            syncDali(ctx, 0xab00, 1);
            devicesAdded++;
        } else {
            entity_status_print_provision_status(ctx->modbus_device, "No more devices found");
        }
    }
    entity_status_print_provision_status(ctx->modbus_device, "Provisioning Complete - found %d devices", devicesAdded);
cleanup:
    vTaskDelay(pdMS_TO_TICKS(2000));
    entity_status_print_provision_status(ctx->modbus_device, "Idle");
}


void dali_provision_task(void *arg) {
    provision_ctx_t *ctx = (provision_ctx_t *) arg;
    // We've only got 2 DALI gateways, so the queue only needs to be long enough to support both of those, plus existing requests being cancelled.
    provisionQueue = xQueueCreate(4, sizeof(provision_cmd_args_t));
    provision_cmd_args_t args;

    currentTask.commandCompleteSem = xSemaphoreCreateBinary();
    
    while (true) {
        if (xQueueReceive(provisionQueue, &args, portMAX_DELAY) == pdTRUE) {
            currentTask.modbus_device = args.device;
            currentTask.incremental = args.incremental;
            provision(&currentTask);
        }
    }
    vTaskDelete(NULL);
}

void startProvisioning(int device, bool incremental) {
    provision_cmd_args_t arg = {
        .device = device,
        .incremental = incremental
    };
    // If we fill up the queue, this becomes a no-op... 
    if (xQueueSend(provisionQueue, &arg, 0) == pdFALSE) {
        printf("Command queue for DALI provisioning is full\n");
    }
}

