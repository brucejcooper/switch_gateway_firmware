#pragma once
/*
 * intercore.h — the core0 <-> core1 contract.
 *
 * Core 0 runs all domain logic (Modbus master, button FSM, dirty-poll, state
 * cache, persistence) and the USB text CLI. Core 1 runs the network bridge
 * (W5100S + MQTT to Home Assistant). They exchange *typed C structs* over two
 * pico_util SPSC queues — NOT serialised JSON. JSON exists only at core 1's MQTT
 * edge. This avoids a format-on-core0 / re-parse-on-core1 round trip and shrinks
 * the largest message (scan_complete) from ~4.5 KB of JSON to 9 bytes.
 *
 * Both queues live in SRAM behind pico_util spinlocks, leaving the SIO inter-core
 * FIFO free for the flash_safe_execute() multicore-lockout handshake.
 *
 *   core0 -> core1 : ic_event_t  (domain state changes -> MQTT publishes)
 *   core1 -> core0 : ic_cmd_t    (MQTT commands -> domain actions)
 */

#include <stdint.h>
#include <stdbool.h>
#include "mapping.h"   /* mapping_target_t, MAX_TARGETS_PER_BTN */

/* Tri-state for on/off/toggle commands. */
typedef enum { IC_TRI_OFF = 0, IC_TRI_ON = 1, IC_TRI_TOGGLE = 2 } ic_tri_t;

/* Button action — mirrors the FSM's action strings. */
typedef enum {
    IC_BTN_PRESS = 0,
    IC_BTN_RELEASE,
    IC_BTN_LONG_PRESS_START,
    IC_BTN_LONG_PRESS_REPEAT,
} ic_btn_action_t;

/* ── core0 -> core1 : events ─────────────────────────────────────────────── */
typedef enum {
    IC_EVT_RELAY = 0,
    IC_EVT_DALI,
    IC_EVT_GROUP,
    IC_EVT_GATEWAY,
    IC_EVT_PRESENCE,
    IC_EVT_FAULT,
    IC_EVT_SCAN_COMPLETE,
    IC_EVT_BUTTON,
    IC_EVT_MAPPING,
    IC_EVT_PROVISION,
    IC_EVT_CFG,
} ic_evt_type_t;

typedef struct {
    ic_evt_type_t type;
    union {
        struct { uint8_t ch; bool on; } relay;
        /* DALI ballast state + bundled config (filled from state_cache at push
         * time so core 1 needs no cache of its own). status is the raw DALI
         * status byte (bit1 lamp_fail, bit4 fading); arc 0 = off. */
        struct { uint8_t slave, addr, arc, status,
                         min, max, power_on, sys_fail, fade_rate, fade_time; } dali;
        /* Group derived state + present-member mask (intersect with present on
         * the consumer if needed). */
        struct { uint8_t slave, group, bri; bool on; uint64_t members; } group;
        struct { uint8_t slave; bool online; } gateway;
        struct { uint8_t slave; uint64_t appeared, disappeared; } presence;
        struct { uint8_t slave, addr; bool lamp_fail, power_cycle; } fault;
        struct { uint8_t slave; uint64_t present_mask; } scan_complete;
        struct { uint8_t fixture, button; ic_btn_action_t action; } button;
        struct { uint8_t fixture, button, count;
                 mapping_target_t targets[MAX_TARGETS_PER_BTN]; } mapping;
        /* state = raw prov_run_state_t value; consumer maps it to state + message
         * strings (the gateway's free-form msg is not carried — keeps the event
         * element small so the queue holds a full resync burst). */
        struct { uint8_t slave, state, count; } provision;
        struct { uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
                 uint32_t baud; } cfg;
    };
} ic_event_t;

/* ── core1 -> core0 : commands ───────────────────────────────────────────── */

/* set_dali field-present bits */
#define IC_DALI_F_ON        (1u << 0)
#define IC_DALI_F_BRI       (1u << 1)
#define IC_DALI_F_MIN       (1u << 2)
#define IC_DALI_F_MAX       (1u << 3)
#define IC_DALI_F_POWER_ON  (1u << 4)
#define IC_DALI_F_SYS_FAIL  (1u << 5)
#define IC_DALI_F_FADE_RATE (1u << 6)
#define IC_DALI_F_FADE_TIME (1u << 7)

/* set_group / set_cfg field-present bits */
#define IC_GRP_F_ON         (1u << 0)
#define IC_GRP_F_BRI        (1u << 1)
#define IC_GRP_F_MEMBERS    (1u << 2)

#define IC_CFG_F_LONG_PRESS (1u << 0)
#define IC_CFG_F_DIM_REPEAT (1u << 1)
#define IC_CFG_F_REVERSAL   (1u << 2)
#define IC_CFG_F_BAUD       (1u << 3)

/* provision mode — mirrors prov_mode_t */
typedef enum { IC_PROV_ALL = 0, IC_PROV_UNADDRESSED = 1, IC_PROV_ADDRESS = 2 } ic_prov_mode_t;

typedef enum {
    IC_CMD_SET_RELAY = 0,
    IC_CMD_SET_DALI,
    IC_CMD_SET_GROUP,
    IC_CMD_SET_CFG,
    IC_CMD_SET_MAPPING,
    IC_CMD_DEL_MAPPING,
    IC_CMD_PROVISION_START,
    IC_CMD_PROVISION_ABORT,
    IC_CMD_RESYNC,
} ic_cmd_type_t;

typedef struct {
    ic_cmd_type_t type;
    union {
        struct { uint8_t ch; ic_tri_t on; } set_relay;
        struct { uint8_t slave, addr; uint16_t fields; ic_tri_t on;
                 uint8_t bri, min, max, power_on, sys_fail, fade_rate, fade_time; } set_dali;
        /* members is a replace-mask, applied when IC_GRP_F_MEMBERS is set. */
        struct { uint8_t slave, group; uint16_t fields; ic_tri_t on; uint8_t bri;
                 uint64_t members; } set_group;
        struct { uint16_t fields; uint16_t long_press_ms, dim_repeat_ms, reversal_window_ms;
                 uint32_t baud; } set_cfg;
        struct { uint8_t fixture, button, count;
                 mapping_target_t targets[MAX_TARGETS_PER_BTN]; } set_mapping;
        struct { uint8_t fixture, button; } del_mapping;
        struct { uint8_t slave; ic_prov_mode_t mode; uint8_t target; } provision_start;
        struct { uint8_t slave; } provision_abort;
    };
} ic_cmd_t;

/* ── API ─────────────────────────────────────────────────────────────────── */

/* Init both queues. Call once on core 0 before launching core 1. */
void intercore_init(void);

/* core0 producer / core1 consumer (event queue). Non-blocking; push returns
 * false (and bumps the drop counter) if the queue is full. */
bool intercore_push_event(const ic_event_t *e);
bool intercore_pop_event(ic_event_t *e);

/* core1 producer / core0 consumer (command queue). Non-blocking. */
bool intercore_push_cmd(const ic_cmd_t *c);
bool intercore_pop_cmd(ic_cmd_t *c);

/* Total events dropped on a full event queue (diagnostics). */
uint32_t intercore_event_drops(void);
