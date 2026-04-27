"""Light platform — DALI dimmable lights and relay-backed lights."""

from __future__ import annotations

import logging
from typing import Any

from homeassistant.components.light import (
    ATTR_BRIGHTNESS,
    ColorMode,
    LightEntity,
)
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback
from homeassistant.helpers.update_coordinator import CoordinatorEntity

from .const import (
    CONF_DALI_BALLASTS,
    CONF_DALI_SLAVES,
    CONF_RELAY_CHANNELS,
    DOMAIN,
    RELAY_TYPE_LIGHT,
)
from .coordinator import LightingModbusCoordinator
from .device_registry import (
    dali_light_device,
    relay_light_device,
)

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator: LightingModbusCoordinator = hass.data[DOMAIN][entry.entry_id]
    entities: list[LightEntity] = []

    # ── DALI lights — each ballast is its own HA device ───────────────────
    dali_slaves: list[int] = entry.data[CONF_DALI_SLAVES]
    dali_ballasts: dict[str, dict] = entry.data.get(CONF_DALI_BALLASTS, {})

    for slave in dali_slaves:
        slave_cfg = dali_ballasts.get(str(slave), {})
        for addr_str, ballast_cfg in slave_cfg.items():
            addr = int(addr_str)
            name = ballast_cfg.get("name", f"DALI {slave}.{addr:02d}")
            entities.append(
                DaliLight(coordinator, entry, slave, addr, name)
            )

    # ── Relay-backed lights — each channel is its own HA device ──────────
    relay_channels: dict[str, dict] = entry.data.get(CONF_RELAY_CHANNELS, {})
    for ch_str, ch_cfg in relay_channels.items():
        if ch_cfg.get("type") == RELAY_TYPE_LIGHT:
            ch = int(ch_str)
            name = ch_cfg["name"]
            entities.append(RelayLight(coordinator, entry, ch, name))

    async_add_entities(entities)


# ── DALI light entity ─────────────────────────────────────────────────────────

class DaliLight(CoordinatorEntity[LightingModbusCoordinator], LightEntity):
    """A single DALI ballast — its own HA device, via_device to gateway."""

    _attr_color_mode = ColorMode.BRIGHTNESS
    _attr_supported_color_modes = {ColorMode.BRIGHTNESS}
    _attr_should_poll = False

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        slave: int,
        dali_addr: int,
        name: str,
    ) -> None:
        super().__init__(coordinator)
        self._slave   = slave
        self._addr    = dali_addr
        self._attr_name        = name
        self._attr_unique_id   = f"{entry.entry_id}_dali_{slave}_{dali_addr}"
        # Each ballast is its own device, linked via_device to the gateway
        self._attr_device_info = dali_light_device(
            entry.entry_id, slave, dali_addr, name
        )

    @property
    def available(self) -> bool:
        return (
            super().available
            and self.coordinator.is_ballast_present(self._slave, self._addr)
        )

    @property
    def is_on(self) -> bool:
        _, arc = self.coordinator.dali_block0[self._slave][self._addr]
        return arc > 0

    @property
    def brightness(self) -> int:
        _, arc = self.coordinator.dali_block0[self._slave][self._addr]
        return round(arc * 255 / 254) if arc > 0 else 0

    @property
    def extra_state_attributes(self) -> dict[str, Any]:
        status, arc = self.coordinator.dali_block0[self._slave][self._addr]
        max_lvl, min_lvl = self.coordinator.dali_block1[self._slave][self._addr]
        power_on, sys_fail = self.coordinator.dali_block2[self._slave][self._addr]
        fade_rate, fade_time = self.coordinator.dali_block3[self._slave][self._addr]
        return {
            "dali_slave":    self._slave,
            "dali_address":  self._addr,
            "arc_level":     arc,
            "status_byte":   f"0x{status:02X}",
            "lamp_failure":  bool(status & 0x02),
            "fade_running":  bool(status & 0x10),
            "max_level":     max_lvl,
            "min_level":     min_lvl,
            "power_on_level": power_on,
            "sys_fail_level": sys_fail,
            "fade_rate":     fade_rate,
            "fade_time":     fade_time,
        }

    async def async_turn_on(self, **kwargs: Any) -> None:
        if ATTR_BRIGHTNESS in kwargs:
            arc = max(1, round(kwargs[ATTR_BRIGHTNESS] * 254 / 255))
            await self.coordinator.async_dali_dapc(self._slave, self._addr, arc)
        else:
            await self.coordinator.async_dali_on(self._slave, self._addr)
        self.async_write_ha_state()

    async def async_turn_off(self, **kwargs: Any) -> None:
        await self.coordinator.async_dali_off(self._slave, self._addr)
        self.async_write_ha_state()


# ── Relay-backed light entity ─────────────────────────────────────────────────

class RelayLight(CoordinatorEntity[LightingModbusCoordinator], LightEntity):
    """A relay channel wired to a light — its own HA device, via_device to relay board."""

    _attr_color_mode = ColorMode.ONOFF
    _attr_supported_color_modes = {ColorMode.ONOFF}
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
        self._attr_unique_id   = f"{entry.entry_id}_relay_light_{channel}"
        self._attr_device_info = relay_light_device(
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
