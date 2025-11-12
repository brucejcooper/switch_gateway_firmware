
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include "entity_status.h"
#include "portmacrocommon.h"
#include "projdefs.h"
#include <cJSON.h>
#include <stdio.h>


bool isConnected = false;
static TaskHandle_t connTrackTask;



void setConnected(bool connected) {
    isConnected = connected;
    if (connected) {
        // Wake up the status Dumper
        xTaskNotifyGiveIndexed(connTrackTask, 1);
    }
}

void status_dumper(void *data) {

    while (true) {
        // Wait for a notification (that we are)
        if (ulTaskNotifyTakeIndexed(1, pdTRUE, portMAX_DELAY) == pdTRUE) {
            if (isConnected) {
                entity_status_print_gateway_device();
            }

            for (int fixture = 0; isConnected && fixture < 24; fixture++) {
                entity_status_print_fxture_device(fixture);
            }

                // The downstream modbus gateways
            for (int devAddr = 1; isConnected && devAddr <= 3; devAddr++) {
                entity_status_print_device(devAddr);
            }

            if (isConnected) {
                entity_status_update_all();
            }
        }
    }
    vTaskDelete(NULL);
}


void connection_tracking_init() {
     xTaskCreate(status_dumper, "onconnect_status", 1024, NULL, 4, &connTrackTask);
}
