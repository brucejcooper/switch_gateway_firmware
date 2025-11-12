#ifndef __entstatus_h__
#define __entstatus_h__

#include <stdbool.h>
#include <stdint.h>
#include <pico/unique_id.h>

typedef struct {
    unsigned int device:8;
    unsigned int address:16;
    unsigned int forcePrint:1;
} action_ctx_t;

extern char device_unique_id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];



extern void entity_status_init();
extern void entity_status_relay_update(uint8_t device, uint16_t address);
extern void entity_status_dali_update(uint8_t device, uint16_t address);
extern void entity_status_update(unsigned device, unsigned address, bool forcePrint);
extern void entity_status_print(uint8_t device, uint16_t address);
extern void entity_status_print_provision_status(unsigned device, const char *format, ...);


extern bool entity_status_is_device_dimmable(unsigned device);


void entity_status_print_gateway_device();
void entity_status_print_device(unsigned device_id);
void entity_status_print_fxture_device(unsigned fixture);

void entity_status_update_all();

#endif