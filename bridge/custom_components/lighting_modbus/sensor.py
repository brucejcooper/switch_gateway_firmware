"""
Sensor platform — DALI gateway provisioning status sensors.

Three sensors per gateway:
  - Commission State   (machine state string)
  - Commission Status  (human-readable progress message)
  - Devices Found      (running count during a provisioning run)

All live on the DALI Gateway device (not on individual ballast devices).
Updated by the coordinator when a provision_status message arrives from the Pico.
"""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.sensor import SensorEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

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
    entities: list[SensorEntity] = []

    for slave in entry.data[CONF_DALI_SLAVES]:
        entities += [
            ProvisionStateSensor(coordinator, entry, slave),
            ProvisionMsgSensor(coordinator, entry, slave),
            ProvisionCountSensor(coordinator, entry, slave),
        ]

    async_add_entities(entities)


class _GatewaySensor(CoordinatorEntity[LightingModbusCoordinator], SensorEntity):
    """Base class for sensors on the DALI gateway device."""

    _attr_should_poll = False
    _attr_entity_category = "diagnostic"

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        slave: int,
        suffix: str,
        name: str,
        icon: str,
    ) -> None:
        super().__init__(coordinator)
        self._slave         = slave
        self._attr_name     = name
        self._attr_unique_id = f"{entry.entry_id}_dali_{slave}_{suffix}"
        self._attr_icon     = icon
        # Live on the gateway device — not on an individual ballast device
        self._attr_device_info = dali_gateway_device(entry.entry_id, slave)


class ProvisionStateSensor(_GatewaySensor):
    """Machine state of the current (or most recent) provisioning run."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_state",
            name=f"DALI {slave} Commission State",
            icon="mdi:lightning-bolt-circle",
        )

    @property
    def native_value(self) -> str:
        return self.coordinator.provision_state.get(self._slave, "idle")


class ProvisionMsgSensor(_GatewaySensor):
    """Human-readable status message, updated throughout a provisioning run."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_msg",
            name=f"DALI {slave} Commission Status",
            icon="mdi:text-box-outline",
        )

    @property
    def native_value(self) -> str:
        return self.coordinator.provision_msg.get(self._slave, "Idle")


class ProvisionCountSensor(_GatewaySensor):
    """Number of devices commissioned in the current or most recent run."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_count",
            name=f"DALI {slave} Devices Found",
            icon="mdi:counter",
        )

    @property
    def native_value(self) -> int:
        return self.coordinator.provision_count.get(self._slave, 0)
