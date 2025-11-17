#ifndef __modbus_h__
#define __modbus_h__


#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>


typedef void (*modbus_action_cb_t)(int status, uint8_t buf[], void *ctx);


extern void modbus_init();
extern void modbus_poll();
extern void modbus_toggle(uint8_t device_id, uint16_t addr, modbus_action_cb_t cb, void *ctx);
extern void modbus_set_coil(uint8_t device_id, uint16_t addr, bool is_on, modbus_action_cb_t cb, void *ctx);
extern void modbus_get_coils(uint8_t device_id, uint16_t addr, uint16_t count, modbus_action_cb_t cb, void *ctx);
extern void modbus_get_holding_registers(uint8_t device_id, uint16_t addr, uint16_t count, modbus_action_cb_t cb, void *ctx);
extern void modbus_set_holding_register(uint8_t device_id, uint16_t addr, uint16_t value, modbus_action_cb_t cb, void *ctx);
extern void modbus_dali(uint8_t device_id, uint16_t cmd, uint8_t repeat, modbus_action_cb_t cb, void *ctx);
extern void modbus_fade(uint8_t device_id, uint16_t addr, int direction, modbus_action_cb_t cb, void *ctx);



#endif