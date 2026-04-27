"""Fan platform — relay-backed fans, each its own HA device."""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.fan import FanEntity, FanEntityFeature
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import CONF_RELAY_CHANNELS, DOMAIN, RELAY_TYPE_FAN
from .coordinator import LightingModbusCoordinator
from .device_registry import relay_fan_device

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator: LightingModbusCoordinator = hass.data[DOMAIN][entry.entry_id]
    entities: list[FanEntity] = []

    relay_channels: dict[str, dict] = entry.data.get(CONF_RELAY_CHANNELS, {})
    for ch_str, ch_cfg in relay_channels.items():
        if ch_cfg.get("type") == RELAY_TYPE_FAN:
            ch = int(ch_str)
            name = ch_cfg["name"]
            entities.append(RelayFan(coordinator, entry, ch, name))

    async_add_entities(entities)


class RelayFan(CoordinatorEntity[LightingModbusCoordinator], FanEntity):
    """A relay channel wired to a single-speed fan — its own HA device."""

    _attr_supported_features = FanEntityFeature(0)
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
        self._attr_unique_id   = f"{entry.entry_id}_relay_fan_{channel}"
        self._attr_device_info = relay_fan_device(
            entry.entry_id, channel, name
        )

    @property
    def is_on(self) -> bool:
        return self.coordinator.relay_coils[self._channel]

    @property
    def percentage(self) -> int | None:
        return 100 if self.is_on else 0

    async def async_turn_on(
        self,
        percentage: int | None = None,
        preset_mode: str | None = None,
        **kwargs: Any,
    ) -> None:
        await self.coordinator.async_set_relay(self._channel, True)
        self.async_write_ha_state()

    async def async_turn_off(self, **kwargs: Any) -> None:
        await self.coordinator.async_set_relay(self._channel, False)
        self.async_write_ha_state()

    async def async_set_percentage(self, percentage: int) -> None:
        await self.coordinator.async_set_relay(self._channel, percentage > 0)
        self.async_write_ha_state()
