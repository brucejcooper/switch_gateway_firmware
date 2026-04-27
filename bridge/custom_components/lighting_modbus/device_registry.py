"""
device_registry.py — DeviceInfo factories for every physical device.

All entity files import from here so device identity is defined once.

Device hierarchy:
  Integration entry
  ├── Relay Board                          (one, no via_device)
  │   └── Relay Light / Fan / Switch      via_device → Relay Board
  ├── DALI Gateway N                       (one per slave, no via_device)
  │   ├── [Provisioning entities]          on Gateway device itself
  │   └── DALI Light                       via_device → Gateway N
  └── Switch Plate N                       (one per fixture 0-23)
      └── Button event entities            on Switch Plate device itself
"""

from homeassistant.helpers.entity import DeviceInfo
from .const import (
    DOMAIN,
    RELAY_BOARD_MANUFACTURER,
    RELAY_BOARD_MODEL,
    DALI_GW_MANUFACTURER,
    DALI_GW_MODEL,
    SWITCH_PLATE_MANUFACTURER,
    SWITCH_PLATE_MODEL,
)


def relay_board_device(entry_id: str) -> DeviceInfo:
    """The Waveshare 32CH relay board — parent of all relay-backed entities."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"relay_board_{entry_id}")},
        name="Relay Board",
        manufacturer=RELAY_BOARD_MANUFACTURER,
        model=RELAY_BOARD_MODEL,
    )


def dali_gateway_device(entry_id: str, slave_id: int) -> DeviceInfo:
    """A TinyModbus DALI gateway — parent of provisioning entities,
    via_device for individual ballast light entities."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"dali_gw_{entry_id}_{slave_id}")},
        name=f"DALI Gateway {slave_id}",
        manufacturer=DALI_GW_MANUFACTURER,
        model=DALI_GW_MODEL,
    )


def dali_light_device(entry_id: str, slave_id: int, dali_addr: int,
                      name: str) -> DeviceInfo:
    """Individual DALI ballast — its own HA device, linked via_device
    to the gateway so the physical chain is visible in HA."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"dali_light_{entry_id}_{slave_id}_{dali_addr}")},
        name=name,
        manufacturer=DALI_GW_MANUFACTURER,
        model="DALI Ballast",
        via_device=(DOMAIN, f"dali_gw_{entry_id}_{slave_id}"),
    )


def relay_light_device(entry_id: str, channel: int, name: str) -> DeviceInfo:
    """Individual relay-backed light — its own HA device, linked via_device
    to the relay board."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"relay_light_{entry_id}_{channel}")},
        name=name,
        manufacturer=RELAY_BOARD_MANUFACTURER,
        model="Relay Channel",
        via_device=(DOMAIN, f"relay_board_{entry_id}"),
    )


def relay_fan_device(entry_id: str, channel: int, name: str) -> DeviceInfo:
    """Individual relay-backed fan — its own HA device, linked via_device
    to the relay board."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"relay_fan_{entry_id}_{channel}")},
        name=name,
        manufacturer=RELAY_BOARD_MANUFACTURER,
        model="Relay Channel",
        via_device=(DOMAIN, f"relay_board_{entry_id}"),
    )


def relay_switch_device(entry_id: str, channel: int, name: str) -> DeviceInfo:
    """Individual relay-backed switch — its own HA device."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"relay_switch_{entry_id}_{channel}")},
        name=name,
        manufacturer=RELAY_BOARD_MANUFACTURER,
        model="Relay Channel",
        via_device=(DOMAIN, f"relay_board_{entry_id}"),
    )


def switch_plate_device(entry_id: str, fixture: int) -> DeviceInfo:
    """A physical switch plate (fixture).  Named by number — user assigns
    a meaningful name and room in the HA device registry UI."""
    return DeviceInfo(
        identifiers={(DOMAIN, f"switch_plate_{entry_id}_{fixture}")},
        name=f"Switch Plate {fixture + 1:02d}",
        manufacturer=SWITCH_PLATE_MANUFACTURER,
        model=SWITCH_PLATE_MODEL,
    )
