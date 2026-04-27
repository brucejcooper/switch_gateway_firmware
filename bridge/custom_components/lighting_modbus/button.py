"""
Button platform — DALI provisioning buttons, on gateway devices.

Per DALI gateway:
  - Commission All          (full reprovision, clears all addresses)
  - Commission New Devices  (unaddressed devices only, safe on live system)
  - Commission Address       (re-initialise the address set in the number entity)
  - Abort Commission

The "target address" for Commission Address is a separate number entity
(number platform) but is registered here to keep provisioning entities together.
"""

from __future__ import annotations

import logging

from homeassistant.components.button import ButtonEntity
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
    entities: list[ButtonEntity] = []

    for slave in entry.data[CONF_DALI_SLAVES]:
        entities += [
            ProvisionAllButton(coordinator, entry, slave),
            ProvisionUnaddressedButton(coordinator, entry, slave),
            ProvisionAddressButton(coordinator, entry, slave),
            ProvisionAbortButton(coordinator, entry, slave),
        ]

    async_add_entities(entities)


class _GatewayButton(ButtonEntity):
    """Base class for buttons that live on the DALI gateway device."""

    _attr_entity_category = "config"

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        slave: int,
        suffix: str,
        name: str,
        icon: str,
    ) -> None:
        self._coordinator   = coordinator
        self._slave         = slave
        self._attr_name     = name
        self._attr_unique_id = f"{entry.entry_id}_dali_{slave}_{suffix}"
        self._attr_icon     = icon
        self._attr_device_info = dali_gateway_device(entry.entry_id, slave)


class ProvisionAllButton(_GatewayButton):
    """Clear all DALI addresses and recommission from scratch."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_all",
            name=f"DALI {slave} Commission All",
            icon="mdi:lightbulb-group",
        )

    async def async_press(self) -> None:
        await self._coordinator.async_provision_start(
            self._slave, mode="all"
        )


class ProvisionUnaddressedButton(_GatewayButton):
    """Address only devices with no short address — safe on a live system."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_unaddressed",
            name=f"DALI {slave} Commission New Devices",
            icon="mdi:lightbulb-plus",
        )

    async def async_press(self) -> None:
        await self._coordinator.async_provision_start(
            self._slave, mode="unaddressed"
        )


class ProvisionAddressButton(_GatewayButton):
    """Re-initialise one specific DALI short address.

    Uses the target address stored in the coordinator (set by the
    number entity in number.py).
    """

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_address",
            name=f"DALI {slave} Commission Address",
            icon="mdi:lightbulb-on",
        )

    async def async_press(self) -> None:
        target = self._coordinator.provision_target.get(self._slave, 0)
        await self._coordinator.async_provision_start(
            self._slave, mode="address", target=target
        )


class ProvisionAbortButton(_GatewayButton):
    """Abort a provisioning run in progress."""

    def __init__(self, coordinator, entry, slave):
        super().__init__(
            coordinator, entry, slave,
            suffix="prov_abort",
            name=f"DALI {slave} Abort Commission",
            icon="mdi:stop-circle",
        )

    async def async_press(self) -> None:
        await self._coordinator.async_provision_abort(self._slave)
