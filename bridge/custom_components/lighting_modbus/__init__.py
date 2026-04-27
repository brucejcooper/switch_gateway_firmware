"""Lighting Modbus integration — setup and serial message routing."""

from __future__ import annotations

import asyncio
import json
import logging

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.exceptions import ConfigEntryNotReady

from .const import DOMAIN
from .coordinator import LightingModbusCoordinator

_LOGGER = logging.getLogger(__name__)

# All platforms in use — including the new event, sensor, button, number
PLATFORMS = ["light", "switch", "fan", "event", "sensor", "button", "number"]


async def async_setup_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Set up Lighting Modbus from a config entry."""
    coordinator = LightingModbusCoordinator(hass, entry)

    connected = await coordinator.async_connect()
    if not connected:
        raise ConfigEntryNotReady("Could not connect to Modbus serial bus")

    await coordinator.async_refresh()

    hass.data.setdefault(DOMAIN, {})[entry.entry_id] = coordinator

    await hass.config_entries.async_forward_entry_setups(entry, PLATFORMS)
    return True


async def async_unload_entry(hass: HomeAssistant, entry: ConfigEntry) -> bool:
    """Unload a config entry."""
    unload_ok = await hass.config_entries.async_unload_platforms(entry, PLATFORMS)
    if unload_ok:
        coordinator: LightingModbusCoordinator = hass.data[DOMAIN].pop(
            entry.entry_id
        )
        await coordinator.async_disconnect()
    return unload_ok
