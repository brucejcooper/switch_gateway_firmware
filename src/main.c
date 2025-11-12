
#include <stdio.h>
#include <stdlib.h>
#include <pico/stdlib.h>
#include <pico/stdio.h>
#include <FreeRTOS.h>
#include <string.h>
#include <task.h>
#include "modbus.h"
#include "entity_status.h"
#include "projdefs.h"
#include "connection_tracking.h"
#include "dali_provision.h"
#include <cJSON.h>

#ifndef LED_DELAY_MS
#define LED_DELAY_MS 250
#endif


extern void buttongrid_task(void *data);



// Perform initialisation
int pico_led_init(void) {
    // A device like Pico that uses a GPIO for the LED will define PICO_DEFAULT_LED_PIN
    // so we can use normal GPIO functionality to turn the led on and off
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    return PICO_OK;
}


#define MAX_LINE_LEN 256


int splitString(char *src, char* parts[], size_t maxParts) {
    int count = 0;

    char *tok = strtok(src, "/");
    while (tok != NULL) {
        if (count >= maxParts) {
            printf("Read too many tokens\n");
            return count;
        }
        parts[count++] = tok;
        tok = strtok(NULL, "/");
    }
    return count;
}

void post_set_callback(int status, uint8_t *buf, action_ctx_t addr) {
    // we just did something to a light, so update it.
    entity_status_update(addr.device, addr.address, true);
}

//homeassistant/status:online
//homeassistant/status:offline

void processCommand(char *topicPath[], size_t numPath, const char *body) {
    if (numPath == 2 && strcmp(topicPath[0], "homeassistant") == 0 && strcmp(topicPath[1], "status") == 0) {
        bool isOnline = strcmp(body, "online") == 0;
        printf("HA is %s\n", isOnline ? "online": "offline");
        setConnected(isOnline);
        return;
    }

    if (numPath > 3 && (strcmp(topicPath[0], "mech") == 0) && (strcmp(topicPath[1], device_unique_id_str) == 0)) {
        // Its for us.
        if (strcmp(topicPath[numPath-1], "set") == 0) {
            if (strncmp(topicPath[2], "dali", 4) == 0) {
                action_ctx_t la;
                la.device = atoi(topicPath[2]+4);

                if (la.device > 0 && la.device <= 3 && numPath == 5 && (strcmp(topicPath[3], "prov") == 0 || strcmp(topicPath[3], "incprov") == 0)) {
                    startProvisioning(la.device, strcmp(topicPath[3], "incprov") == 0);
                    return;
                } 
                
                if (la.device > 0 && la.device <= 3 && numPath == 6 && strcmp(topicPath[4], "light") == 0) {
                    la.address = atoi(topicPath[3]);
                    if (la.address < 64) {
                        cJSON *json = cJSON_Parse(body);
                        if (json != NULL) {
                            cJSON *brightness = cJSON_GetObjectItemCaseSensitive(json, "brightness");
                            if (cJSON_IsNumber(brightness) && brightness->valueint >= 0 && brightness->valueint < 254) {
                                printf("Dali Device %d address %d set to %d\n",la.device, la.address, brightness->valueint);   
                                modbus_set_holding_register(la.device, la.address, brightness->valueint, post_set_callback, la);
                            }
                            cJSON_Delete(json);
                        } else {
                            printf("Error parsing JSON\n");
                        }
                        return;
                    }
                }
            }
            if (strncmp(topicPath[2], "modbus", 6) == 0) {
                int deviceId = atoi(topicPath[2]+6);
                if (numPath == 6  && strcmp(topicPath[4], "coil") == 0) {
                    int addressId = atoi(topicPath[3]);
                    printf("Modbus Device %d address %d set to %s\n",deviceId, addressId, body);   
                }
                return;
            }
        }
    }
    printf("unknown command\n");
}



void repl(void *data) {
    char buf[MAX_LINE_LEN];
    int ch;
    char *ptr = buf;

    vTaskDelay(pdMS_TO_TICKS(2000));

    entity_status_update_all();

    while (true) {
        if ((ch = stdio_getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
            if (ch == '\n') {
                *ptr = '\0';

                char *msgStart = strchr(buf, ':');
                if (msgStart) {
                    *msgStart++ = 0;
                    char *parts[10];
                    int numParts = splitString(buf, parts, 10);
                    if (numParts == 10) {
                        printf("Topic too long\n");
                    } else {
                        processCommand(parts, numParts, msgStart);
                    }
                } else {
                    printf("Ignoring %s\n", buf);
                }
                ptr = buf;
            } else if (ptr-buf < MAX_LINE_LEN && ch != '\r') {
                *ptr++ = ch;
            }
        }
        taskYIELD(); // this runs at the same priority as the idle task, so we need to give it a chance.
    }
    // Do not let a task procedure return
    vTaskDelete(NULL);
}




int main() {
    stdio_init_all();
    int rc = pico_led_init();
    // hard_assert(rc == PICO_OK);

    entity_status_init();
    connection_tracking_init();
    
    xTaskCreate(buttongrid_task, "buttongrid", 2048, NULL, 16, NULL);
    xTaskCreate(modbus_task, "modbus", 2048, NULL, 8, NULL);
    xTaskCreate(dali_provision_task, "dali provisioning", 2048, NULL, 4, NULL);
    xTaskCreate(repl, "repl", 2048, NULL, 0, NULL);
    vTaskStartScheduler();
    // we should never return from FreeRTOS
    panic_unsupported();
}