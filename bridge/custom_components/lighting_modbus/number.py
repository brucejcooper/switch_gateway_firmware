"""
Number platform — target DALI address for the Commission Address button.

One number entity per DALI gateway, on the gateway device.
Range 0–63. User sets this before pressing the Commission Address button.
Value is stored in the coordinator so the button can read it.
"""

from __future__ import annotations

import logging

from homeassistant.components.number import NumberEntity, NumberMode
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import CONF_DALI_SLAVES, DOMAIN
from .coordinator import LightingModbusCoordinator
from .device_registry import dali_gateway_device

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator: LightingModbusCoordinator = hass.data[DOMAIN][entry.entry_id]
    entities: list[NumberEntity] = []

    for slave in entry.data[CONF_DALI_SLAVES]:
        entities.append(ProvisionTargetNumber(coordinator, entry, slave))

    async_add_entities(entities)


class ProvisionTargetNumber(NumberEntity):
    """Target DALI short address for the Commission Address button."""

    _attr_entity_category = "config"
    _attr_native_min_value = 0
    _attr_native_max_value = 63
    _attr_native_step      = 1
    _attr_mode             = NumberMode.BOX
    _attr_icon             = "mdi:numeric"

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        slave: int,
    ) -> None:
        self._coordinator   = coordinator
        self._slave         = slave
        self._attr_name     = f"DALI {slave} Target Address"
        self._attr_unique_id = f"{entry.entry_id}_dali_{slave}_prov_target"
        self._attr_device_info = dali_gateway_device(entry.entry_id, slave)

    @property
    def native_value(self) -> float:
        return float(self._coordinator.provision_target.get(self._slave, 0))

    async def async_set_native_value(self, value: float) -> None:
        self._coordinator.provision_target[self._slave] = int(value)
        self.async_write_ha_state()
