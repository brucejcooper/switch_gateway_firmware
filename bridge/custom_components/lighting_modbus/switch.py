"""Switch platform — relay channels configured as generic switches."""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.switch import SwitchEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import CONF_RELAY_CHANNELS, DOMAIN, RELAY_TYPE_SWITCH
from .coordinator import LightingModbusCoordinator
from .device_registry import relay_switch_device

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator: LightingModbusCoordinator = hass.data[DOMAIN][entry.entry_id]
    entities: list[SwitchEntity] = []

    relay_channels: dict[str, dict] = entry.data.get(CONF_RELAY_CHANNELS, {})
    for ch_str, ch_cfg in relay_channels.items():
        if ch_cfg.get("type") == RELAY_TYPE_SWITCH:
            ch = int(ch_str)
            name = ch_cfg["name"]
            entities.append(RelaySwitch(coordinator, entry, ch, name))

    async_add_entities(entities)


class RelaySwitch(CoordinatorEntity[LightingModbusCoordinator], SwitchEntity):
    """A relay channel as a generic switch — its own HA device."""

    _attr_should_poll = False

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        channel: int,
        name: str,
    ) -> None:
        super().__init__(coordinator)
        self._channel = channel
        self._attr_name        = name
        self._attr_unique_id   = f"{entry.entry_id}_relay_switch_{channel}"
        self._attr_device_info = relay_switch_device(
            entry.entry_id, channel, name
        )

    @property
    def is_on(self) -> bool:
        return self.coordinator.relay_coils[self._channel]

    async def async_turn_on(self, **kwargs: Any) -> None:
        await self.coordinator.async_set_relay(self._channel, True)
        self.async_write_ha_state()

    async def async_turn_off(self, **kwargs: Any) -> None:
        await self.coordinator.async_set_relay(self._channel, False)
        self.async_write_ha_state()
