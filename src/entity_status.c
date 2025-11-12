#include <stdio.h>
#include "entity_status.h"
#include <pico/unique_id.h>
#include "modbus.h"
#include <FreeRTOS.h>
#include <semphr.h>
#include <stdarg.h>


#define NUM_DALI_DEVICES 64
#define NUM_RELAY_DEVICES 32

static SemaphoreHandle_t outputlock;

typedef enum {
    DT_NONE,
    DT_DALI,
    DT_RELAY,
} device_type_t;

typedef struct {
    bool available;
    int brightness;
    int status;
    int min;
    int max;
    int power_on;
    int power_failure;
    int fade_rate;
    int fade_time;
} dali_state_t;

typedef struct {
    bool is_on;
} relay_state_t;


typedef struct {
    device_type_t dt;
    uint8_t device_id;
    bool available; // This reflects if the modbus dali gateway device is available.  Each light also may be available or not.
    // -- end of shared union values
    dali_state_t lights[NUM_DALI_DEVICES];
} dali_device_t;

typedef struct {
    device_type_t dt;
    uint8_t device_id;
    bool available; // Whether the modbus relay gateway device is available. 
    // -- end of shared union values
    relay_state_t coils[NUM_RELAY_DEVICES];
} relay_device_t;


typedef union {
    struct {
        device_type_t dt;
        uint8_t device_id;
        bool available;
    };
    dali_device_t dali;
    relay_device_t relay;
} device_t;



#define MAX_DEVICES 3


static relay_device_t relay_device = {
    .dt = DT_RELAY,
    .device_id = 1,
    .available = false,
};


static dali_device_t dali2_device = {
    .dt = DT_DALI,
    .device_id = 2,
    .available = false,
};

static dali_device_t dali3_device = {
    .dt = DT_DALI,
    .device_id = 3,
    .available = false,
};


static device_t *devices[MAX_DEVICES+1] = {
    NULL, // No device at address 0
    (device_t *) &relay_device,
    (device_t *) &dali2_device,
    (device_t *) &dali3_device,
};
char device_unique_id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];

void entity_status_print_device_available(unsigned device_id);
void entity_status_print_dali_light_available(device_t *device, uint16_t light);
static void update_dali_light_status_cb(int retsz, uint8_t *buf, action_ctx_t light);
extern bool button_is_pressed(unsigned index);

static inline char *topicprefix(device_t *dev) {
    return dev->dt == DT_DALI ? "dali" : "modbus";
}


bool entity_status_is_device_dimmable(unsigned device) {
    device_t *dev = devices[device];
    return dev && dev->dt == DT_DALI;
}



void entity_status_init() {
    outputlock = xSemaphoreCreateMutex();
    pico_get_unique_board_id_string(device_unique_id_str, sizeof(device_unique_id_str));


    for (int addr = 0; addr < NUM_RELAY_DEVICES; addr++) {
        relay_device.coils[addr].is_on = false;
    }
    for (int addr = 0; addr < NUM_DALI_DEVICES; addr++) {
        dali2_device.lights[addr].available = false;
        dali2_device.lights[addr].brightness = -1;
        dali3_device.lights[addr].available = false;
        dali3_device.lights[addr].brightness = -1;
    }
}

static void device_set_available(uint8_t device, bool is_available) {
    device_t *dev = devices[device];
    if (dev) {
        if (dev->available != is_available) {
            dev->available = is_available;
            entity_status_print_device_available(device);
        }
    }
}

static void dali_light_set_available(uint8_t device, uint16_t address, bool is_available) {
    device_t *dev = devices[device];
    if (dev) {
        dali_state_t *light = &dev->dali.lights[address];

        if (light->available != is_available) {
            light->available = is_available;
            entity_status_print_dali_light_available(dev, address);
        }
    }
}


static bool is_valid_light_address(unsigned device, unsigned address) {
    device_t *dev = devices[device];
    if (!dev) {
        return false;
    }
    if (dev->dt == DT_DALI) {
        return address < NUM_DALI_DEVICES;
    } else if (dev->dt == DT_RELAY) {
        return address < NUM_RELAY_DEVICES;
    }
    return false;
}


static void update_dali_light_status(action_ctx_t light, int brightness, int status, bool forcePrint) {
    if (brightness == 0xFF)  {
        brightness = -1;
        dali_light_set_available(light.device, light.address, false);
    } else {
        dali_light_set_available(light.device, light.address, true);
    }
    device_t *dev = devices[light.device];
    if (dev) {
        dali_state_t *state = &dev->dali.lights[light.address];
        
        
        bool shouldPrint = forcePrint;
        if (state->brightness != brightness || state->status != status) {
            state->brightness = brightness;
            state->status = status;
            shouldPrint = true;
        }

        if (shouldPrint) {
            entity_status_print(light.device, light.address);
        }
        if (status & 0x10) {
            // Device is dimming.  Ask again. 
            modbus_get_holding_registers(light.device,light.address, 1, update_dali_light_status_cb, light);
        }
    }
}

static void update_dali_light_status_cb(int retsz, uint8_t *buf, action_ctx_t light) {
    if (is_valid_light_address(light.device, light.address)) {
        device_t *dev = devices[light.device];
        if (dev) {
            if (retsz > 0) {
                device_set_available(light.device, true);
                int status = buf[3];
                int brightness = buf[4];
                update_dali_light_status(light, brightness, status, light.forcePrint);
            } else {
                device_set_available(light.device, false);
            }
        }
    }
}

static void update_dali_all_light_status_cb(int retsz, uint8_t *buf, action_ctx_t address) {

    if (retsz > 0) {
        device_set_available(address.device, true);

        uint8_t *ptr = buf + 3;
        for (int addr = 0; addr < NUM_DALI_DEVICES; addr++) {
            int status = *ptr++;
            int brightness = *ptr++;
            address.address = addr;
            update_dali_light_status(address, brightness, status, true);
        }
    } else {
        device_set_available(address.device, false);

    }
}


static void update_relay_status(unsigned device_id, unsigned addr, bool is_on, bool forcePrint) {
    device_t *dev = devices[device_id];
    if (dev) {
        relay_state_t *state = &dev->relay.coils[addr];
        bool shouldPrint = forcePrint;
        if (state->is_on != is_on) {
            state->is_on = is_on;
            shouldPrint = true;
        }
        if (shouldPrint) {
            entity_status_print(device_id, addr);
        }
    }
}

static void update_dali_all_relays_status_cb(int retsz, uint8_t *buf, action_ctx_t address) {

    if (retsz > 0) {
        device_set_available(address.device, true);

        uint8_t *ptr = buf + 3;
        uint32_t allBits = buf[3] | (buf[4] << 8) | (buf[5] << 16) | (buf[6] << 24);
        for (int addr = 0; addr < 32; addr++) {
            update_relay_status(address.device, addr, allBits & 0x01, true);
            allBits >>= 1;
        }
    } else {
        device_set_available(address.device, false);
    }

}



static void update_relay_status_cb(int retsz, uint8_t *buf, action_ctx_t light) {
    if (is_valid_light_address(light.device, light.address)) {
        printf("Coil fetch result %d, %d\n", retsz, buf[3]);
        if (retsz > 0) {
            device_set_available(light.device, true);
            int numbytes = buf[2];
            // Bytes start at byte 3
            update_relay_status(light.device, light.address, buf[3] & 0x01, light.forcePrint);
        } else {
            device_set_available(light.device, false);
        }
    }
}

void entity_status_update(unsigned device, unsigned address, bool forcePrint) {
    action_ctx_t arg = {
        .device = device,
        .address = address,
        .forcePrint = forcePrint
    };
    if (is_valid_light_address(device, address)) {
        if (device <= MAX_DEVICES) {
            device_t *dev = devices[(device)];
            if (dev) {
                switch (dev->dt) {
                    case DT_DALI:
                        modbus_get_holding_registers(device,address, 1,update_dali_light_status_cb, arg);
                        break;
                    case DT_RELAY:
                        modbus_get_coils(device,address, 1, update_relay_status_cb, arg);
                        break;
                    case DT_NONE:
                        break;
                    default:
                        printf("Unknown device type %d", dev->dt);
                }
            }
        }
    }
}

void entity_status_update_all() {

    for (int i = 0; i < 24 * 8; i++) {
        entity_status_print(0,i);
    }

    for (unsigned device_id = 1; device_id <= MAX_DEVICES; device_id++) {
        device_t *dev = devices[device_id];

        if (dev) {
            action_ctx_t addr = {
                .device = device_id,
                .address = 0,
            };
            switch (dev->dt) {
                case DT_DALI:
                    modbus_get_holding_registers(device_id,0, 64,update_dali_all_light_status_cb, addr);
                    break;
                case DT_RELAY:
                    modbus_get_coils(device_id,0, 32,update_dali_all_relays_status_cb, addr);
                    break;
                default:
                    break;
            }
        }
    }
}


void entity_status_print_device_available(unsigned device_id) {
    device_t *device = devices[device_id];
    if (device) {
        xSemaphoreTake(outputlock, portMAX_DELAY);
        printf("mech/%s/%s%d/available:%s\n", device_unique_id_str, topicprefix(device), device->device_id, device->available ? "online": "offline");
        xSemaphoreGive( outputlock );
    }
}

void entity_status_print_dali_light_available(device_t *device, uint16_t light) {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    printf("mech/%s/dali%d/%d/available:%s\n", device_unique_id_str, device->device_id, light, device->dali.lights[light].available ? "online": "offline");
    xSemaphoreGive( outputlock );
}


#define printj_startobj(key) printf("\"%s\": {", key, val)
#define printj_addstr(key,val) printf("\"%s\": \"%s\"", key, val)
#define printj_addint(key,val) printf("\"%s\": %d", key, val)


void entity_status_print_relay_entity(device_t *device, unsigned device_id, unsigned addr) {
    putchar('{');
        printf("\"uniq_id\": \"mech%s_%s%d_%d_coil\"", device_unique_id_str, topicprefix(device), device_id, addr);
        putchar(',');
        printf("\"~\": \"mech/%s/%s%d/%d/coil\"", device_unique_id_str, topicprefix(device), device_id, addr);
        putchar(',');
        printf("\"avty\": [");
            printf( "\"mech/%s/available\"", device_unique_id_str);
            putchar(',');
            printf("\"mech/%s/%s%d/available\"", device_unique_id_str, topicprefix(device), device_id);
        putchar(']');
        putchar(',');
        printf("\"avty_mode\": \"all\"");
        putchar(',');
        printf("\"name\": \"Relay\"");
        putchar(',');
        printf("\"stat_t\": \"~/state\"");
        putchar(',');
        printf("\"cmd_t\": \"~/set\"");
        putchar(',');
        printf("\"p\": \"light\"");
    putchar('}');

}

void entity_status_print_relay_device(device_t *device, unsigned device_id, unsigned addr) {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    printf("homeassistant/device/mech%s_modbus%d_%d/config:", device_unique_id_str, device_id, addr);
    putchar('{');
        printf("\"dev\":{");
            printf("\"name\": \"Light %d@%d\"", addr, device_id);
            putchar(',');
            printf("\"ids\": [\"mech%s_modbus%d_%d\"]", device_unique_id_str, device_id, addr);
            putchar(',');
            printf("\"via_device\": \"mech%s_modbus%d\"", device_unique_id_str, device_id);
        printf("}, \"cmps\": [");
            entity_status_print_relay_entity(device, device_id, addr);
        putchar(']');
    putchar('}');
    putchar('\n');
    xSemaphoreGive( outputlock );
}


void entity_status_print_dali_entity(device_t *device, unsigned device_id, unsigned addr) {
    putchar('{');
        printf("\"uniq_id\": \"mech%s_dali%d_%d_light\"", device_unique_id_str,  device_id, addr);
        printf(",\"~\": \"mech/%s/dali%d/%d/light\"", device_unique_id_str, device_id, addr);
        printf(",\"avty\": [");
            printf( "\"mech/%s/available\"", device_unique_id_str);
            printf(",\"mech/%s/dali%d/available\"", device_unique_id_str, device_id);
            printf(",\"mech/%s/dali%d/%d/available\"", device_unique_id_str, device_id, addr);
        printf("],\"avty_mode\": \"all\"");
        printf(",\"name\": \"Brightness\"");
        printf(",\"stat_t\": \"~/state\"");
        printf(",\"cmd_t\": \"~/set\"");
        printf(",\"p\": \"light\"");
        printf(",\"brightness\": true");
    putchar('}');
}

void entity_status_print_button_entity(unsigned fixture, unsigned btn) {
    putchar('{');
        printf("\"uniq_id\": \"mech%s_fix%d_btn%d\"", device_unique_id_str,  fixture, btn);
        printf(",\"~\": \"mech/%s/fix%d/btn%d\"", device_unique_id_str, fixture, btn);
        printf(",\"avty\": [");
            printf( "\"mech/%s/available\"", device_unique_id_str);
        printf("],\"avty_mode\": \"all\"");
        printf(",\"name\": \"Button %d\"", btn+1);
        printf(",\"stat_t\": \"~/state\"");
        printf(",\"cmd_t\": \"~/set\"");
        printf(",\"p\": \"binary_sensor\"");
    putchar('}');
}


void entity_status_print_commandbutton_entity(unsigned device_id, bool incremental) {
    putchar('{');
        printf("\"uniq_id\": \"mech%s_dali%d_%s\"", device_unique_id_str,  device_id, incremental ? "incprov" : "prov");
        printf(",\"~\": \"mech/%s/dali%d/%s\"", device_unique_id_str, device_id, incremental ? "incprov" : "prov");
        printf(",\"avty\": [");
            printf( "\"mech/%s/available\"", device_unique_id_str);
            putchar(',');
            printf( "\"mech/%s/dali%d/available\"", device_unique_id_str, device_id);
        printf("],\"avty_mode\": \"all\"");
        printf(",\"name\": \"%s Provision\"", incremental ? "Incremental" : "Full");
        printf(",\"stat_t\": \"~/state\"");
        printf(",\"cmd_t\": \"~/set\"");
        printf(",\"p\": \"button\"");
    putchar('}');
}

void entity_status_print_commandstatus_entity(unsigned device_id) {
    putchar('{');
        printf("\"uniq_id\": \"mech%s_dali%d_provstatus\"", device_unique_id_str,  device_id);
        printf(",\"~\": \"mech/%s/dali%d/provstatus\"", device_unique_id_str, device_id);
        printf(",\"avty\": [");
            printf( "\"mech/%s/available\"", device_unique_id_str);
            putchar(',');
            printf( "\"mech/%s/dali%d/available\"", device_unique_id_str, device_id);
        printf("],\"avty_mode\": \"all\"");
        printf(",\"name\": \"Provision staus\"");
        printf(",\"stat_t\": \"~/state\"");
        printf(",\"cmd_t\": \"~/set\"");
        printf(",\"p\": \"sensor\"");
    putchar('}');
}



void entity_status_print_dali_device(device_t *device, unsigned device_id, unsigned addr) {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    printf("homeassistant/device/mech%s_dali%d_%d/config:{", device_unique_id_str, device_id, addr);
        printf("\"dev\": {");
            printf("\"name\": \"Light %d@%d\"", addr, device_id);
            printf(", \"ids\": [\"mech%s_dali%d_%d\"]", device_unique_id_str, device_id, addr);
            printf(", \"via_device\": \"mech%s_dali%d\"", device_unique_id_str, device_id);
        printf("}, \"cmps\": [");
            entity_status_print_dali_entity(device, device_id, addr);
        putchar(']');
    printf("}\n");
    xSemaphoreGive( outputlock );
    entity_status_print_dali_light_available(device, addr);

}

void entity_status_print_gateway_device() {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    printf("homeassistant/device/mech%s/config:{", device_unique_id_str);
        printf("\"dev\":{");
            printf("\"name\":\"Gateway\"");
            putchar(',');
            printf("\"ids\": [\"mech%s\"]", device_unique_id_str);
        putchar('}');
        putchar(',');
        printf("\"cmps\": [");
        // No components.
        putchar(']');
    printf("}\n");
    printf("mech/%s/available:online\n", device_unique_id_str);
    xSemaphoreGive( outputlock );
}

void entity_status_print_fxture_device(unsigned fixture) {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    printf("homeassistant/device/mech%s_fix%d/config:", device_unique_id_str, fixture);
    putchar('{');
        printf("\"dev\":{");
            printf("\"name\":\"Switchplate %d\"", fixture);
            putchar(',');
            printf("\"via_device\": \"mech%s\"", device_unique_id_str);
            putchar(',');
            printf("\"ids\": [\"mech%s_fix%d\"]",device_unique_id_str, fixture);
        printf("},\"cmps\":[");
            for (int btn = 0; btn < 8; btn++) {
                if (btn != 0) {
                    printf(",");
                }
                entity_status_print_button_entity(fixture, btn);
            }
        putchar(']');
    putchar('}');
    putchar('\n');
    xSemaphoreGive( outputlock );
}



void entity_status_print_device(unsigned device_id) {
    device_t *device = devices[device_id];
    if (device) {
        xSemaphoreTake(outputlock, portMAX_DELAY);
        // Print the discovery message for the Modbus/DALI Gateway device.
        printf("homeassistant/device/mech%s_%s%d/config:", device_unique_id_str, topicprefix(device), device_id);
        putchar('{');
            printf("\"dev\": {");
                printf("\"name\": \"%s %d\"", topicprefix(device), device_id);
                putchar(',');
                printf("\"ids\": [\"mech%s_%s%d\"]", device_unique_id_str, topicprefix(device), device_id);
                putchar(',');
                printf("\"via_device\": \"mech%s\"", device_unique_id_str);
            putchar('}');
            putchar(',');
            printf("\"cmps\":");
            putchar('[');
            // No Components.
            if (device->dt == DT_DALI) {
                entity_status_print_commandbutton_entity(device_id, false);
                putchar(',');
                entity_status_print_commandbutton_entity(device_id, true);
                putchar(',');
                entity_status_print_commandstatus_entity(device_id);
            }
            // TODO for dali devices, add the provisioner buttons and the provisioning status.
            putchar(']');
        putchar('}');
        putchar('\n');
        xSemaphoreGive( outputlock );

        switch (device->dt) {
            case DT_DALI:
                for (int addr = 0; addr < 64; addr++) {
                    entity_status_print_dali_device(device, device_id, addr);
                }
                break;
            case DT_RELAY:
                for (int addr = 0; addr < 32; addr++) {
                    entity_status_print_relay_device(device, device_id, addr);
                }
                break;
            default:
                break;
        }
        entity_status_print_device_available(device_id);
    }
}


void entity_status_print_provision_status(unsigned device, const char *format, ...) {
    xSemaphoreTake(outputlock, portMAX_DELAY);
    va_list args;
    va_start(args, format);
    printf("mech/%s/dali%d/provstatus/state:", device_unique_id_str, device);
    vprintf(format, args);
    va_end(args);
    putchar('\n');
    xSemaphoreGive( outputlock );
}



void entity_status_print(uint8_t device, uint16_t address) {
    int brightness;
    relay_state_t *relay;
    dali_state_t *dali;
    xSemaphoreTake(outputlock, portMAX_DELAY);
    if (device == 0) {
        printf("mech/%s/fix%d/btn%d/state:%s\n", device_unique_id_str, address/8, address%8, button_is_pressed(address) ? "ON":"OFF");
    } else {
        device_t *dev = devices[device];
        if (dev) {
            switch (dev->dt) {
                case DT_DALI:
                dali = &dev->dali.lights[address];
                brightness = dali->brightness;
                printf("mech/%s/dali%d/%d/light/state:{\"state\":\"%s\", \"brightness\":%d}\n", device_unique_id_str, device, address, brightness > 0 ? "ON" : "OFF", brightness);
                break;
                
                case DT_RELAY:
                relay = &dev->relay.coils[address];
                printf("mech/%s/modbus%d/%d/coil/state:%s\n", device_unique_id_str, device, address, dev->relay.coils[address].is_on ? "ON" : "OFF");
                break;

                case DT_NONE:
                break;

                default:
                    printf("Ingnoring request to update status on illegal device type");
                break;
            }
        }
    }
    xSemaphoreGive( outputlock );
}
