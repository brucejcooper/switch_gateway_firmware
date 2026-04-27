"""Constants for the Lighting Modbus integration."""

DOMAIN = "lighting_modbus"

# ── Config entry keys ─────────────────────────────────────────────────────────
CONF_SERIAL_PORT    = "serial_port"
CONF_BAUDRATE       = "baudrate"
CONF_RELAY_SLAVE    = "relay_slave"
CONF_DALI_SLAVES    = "dali_slaves"
CONF_RELAY_CHANNELS = "relay_channels"   # {channel_index: {name, type}}
CONF_DALI_BALLASTS  = "dali_ballasts"    # {slave_id: {ballast_index: {name}}}

# ── Device type tags for relay channels ───────────────────────────────────────
RELAY_TYPE_LIGHT  = "light"
RELAY_TYPE_FAN    = "fan"
RELAY_TYPE_SWITCH = "switch"
RELAY_TYPES = [RELAY_TYPE_LIGHT, RELAY_TYPE_FAN, RELAY_TYPE_SWITCH]

# ── Polling intervals ─────────────────────────────────────────────────────────
FAST_POLL_INTERVAL = 1
FULL_SCAN_INTERVAL = 60

# ── Waveshare 32CH Modbus map ─────────────────────────────────────────────────
RELAY_COIL_START = 0x0000
RELAY_COIL_COUNT = 32

# ── TinyModbus FC04 Input Register map ───────────────────────────────────────
DALI_BLOCK0_START          = 0x0000
DALI_BLOCK0_COUNT          = 64
DALI_BLOCK1_START          = 0x0040
DALI_BLOCK1_COUNT          = 64
DALI_BLOCK2_START          = 0x0080
DALI_BLOCK2_COUNT          = 64
DALI_BLOCK3_START          = 0x00C0
DALI_BLOCK3_COUNT          = 64
DALI_GROUP_STATUS_START    = 0x0100
DALI_GROUP_STATUS_COUNT    = 16
DALI_PRESENT_BITMASK_START = 0x0151
DALI_PRESENT_BITMASK_COUNT = 4
DALI_DIRTY_BITMASK_START   = 0x0159
DALI_DIRTY_BITMASK_COUNT   = 4
DALI_SCAN_STATUS_REG       = 0x0150

# ── TinyModbus FC06 Holding Register action map ───────────────────────────────
DALI_REG_DAPC             = 0x1002
DALI_REG_GOTO_LAST_ACTIVE = 0x1003
DALI_REG_OFF              = 0x1004
DALI_REG_SCAN_TRIGGER     = 0x1010
DALI_REG_RESCAN_INTERVAL  = 0x1011
DALI_REG_SLAVE_ADDR       = 0x1012
DALI_REG_BAUD_INDEX       = 0x1013

DALI_MAX_REGS_PER_READ = 16

# ── Switch plate / button constants ───────────────────────────────────────────
MAX_FIXTURES   = 24
MAX_BUTTONS    = 7

# Button event types — must match what the Pico firmware emits
BTN_EVENT_PRESS             = "press"
BTN_EVENT_RELEASE           = "release"
BTN_EVENT_LONG_PRESS_START  = "long_press_start"
BTN_EVENT_LONG_PRESS_REPEAT = "long_press_repeat"
BUTTON_EVENT_TYPES = [
    BTN_EVENT_PRESS,
    BTN_EVENT_RELEASE,
    BTN_EVENT_LONG_PRESS_START,
    BTN_EVENT_LONG_PRESS_REPEAT,
]

# ── Device model strings ──────────────────────────────────────────────────────
RELAY_BOARD_MANUFACTURER = "Waveshare"
RELAY_BOARD_MODEL        = "Modbus RTU Relay 32CH"
DALI_GW_MANUFACTURER     = "Mechination"
DALI_GW_MODEL            = "TinyModbus ATtiny1604 DALI Gateway"
SWITCH_PLATE_MANUFACTURER = "Mechination"
SWITCH_PLATE_MODEL        = "Switch Gateway Button Matrix"

# ── Config flow step IDs ──────────────────────────────────────────────────────
STEP_USER          = "user"
STEP_RELAY_CHANNELS = "relay_channels"
STEP_DALI_BALLASTS  = "dali_ballasts"
