"""
Coordinator for the Lighting Modbus integration.

Owns the pymodbus client and all cached state.  Extended from the previous
version to handle:
  - Button events from the Pico (fired to ButtonEventEntity instances)
  - Provisioning state/message/count (read by sensor entities)
  - Provisioning command dispatch (called by button entities)
  - Gateway device entities on the gateway DeviceInfo (not per-ballast)
"""

from __future__ import annotations

import asyncio
import logging
from datetime import timedelta
from typing import Any, TYPE_CHECKING

from pymodbus.client import AsyncModbusSerialClient
from pymodbus.exceptions import ModbusException

from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.update_coordinator import DataUpdateCoordinator, UpdateFailed

from .const import (
    CONF_BAUDRATE,
    CONF_DALI_BALLASTS,
    CONF_DALI_SLAVES,
    CONF_RELAY_SLAVE,
    CONF_SERIAL_PORT,
    DALI_BLOCK0_COUNT,
    DALI_BLOCK0_START,
    DALI_BLOCK1_COUNT,
    DALI_BLOCK1_START,
    DALI_BLOCK2_COUNT,
    DALI_BLOCK2_START,
    DALI_BLOCK3_COUNT,
    DALI_BLOCK3_START,
    DALI_MAX_REGS_PER_READ,
    DALI_PRESENT_BITMASK_COUNT,
    DALI_PRESENT_BITMASK_START,
    DALI_REG_DAPC,
    DALI_REG_GOTO_LAST_ACTIVE,
    DALI_REG_OFF,
    DALI_REG_RESCAN_INTERVAL,
    DALI_REG_SCAN_TRIGGER,
    DOMAIN,
    FAST_POLL_INTERVAL,
    FULL_SCAN_INTERVAL,
    RELAY_COIL_COUNT,
    RELAY_COIL_START,
    BTN_EVENT_PRESS,
    BTN_EVENT_RELEASE,
    BTN_EVENT_LONG_PRESS_START,
    BTN_EVENT_LONG_PRESS_REPEAT,
)

if TYPE_CHECKING:
    from .event import ButtonEventEntity

_LOGGER = logging.getLogger(__name__)

# Map serial protocol button event strings to HA event type constants
_BTN_EVENT_MAP = {
    "press":             BTN_EVENT_PRESS,
    "release":           BTN_EVENT_RELEASE,
    "long_press_start":  BTN_EVENT_LONG_PRESS_START,
    "long_press_repeat": BTN_EVENT_LONG_PRESS_REPEAT,
}


def _bitmask_from_registers(registers: list[int]) -> int:
    result = 0
    for i, reg in enumerate(registers):
        result |= reg << (i * 16)
    return result


def _read_block_chunked(client, start, count, slave, chunk_size=DALI_MAX_REGS_PER_READ):
    for offset in range(0, count, chunk_size):
        n = min(chunk_size, count - offset)
        yield offset, client.read_input_registers(start + offset, count=n, slave=slave)


class LightingModbusCoordinator(DataUpdateCoordinator):
    """Central coordinator — owns the serial bus and all cached state."""

    def __init__(self, hass: HomeAssistant, entry: ConfigEntry) -> None:
        super().__init__(
            hass,
            _LOGGER,
            name=DOMAIN,
            update_interval=timedelta(seconds=FAST_POLL_INTERVAL),
        )
        self._entry = entry
        self._lock  = asyncio.Lock()
        self._full_scan_counter = 0

        cfg = entry.data
        self._relay_slave:  int       = cfg[CONF_RELAY_SLAVE]
        self._dali_slaves:  list[int] = cfg[CONF_DALI_SLAVES]

        self._client = AsyncModbusSerialClient(
            port=cfg[CONF_SERIAL_PORT],
            baudrate=cfg[CONF_BAUDRATE],
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.5,
        )

        # ── Lighting state cache ──────────────────────────────────────────
        self.relay_coils: list[bool] = [False] * RELAY_COIL_COUNT
        self.dali_present: dict[int, int] = {s: 0 for s in self._dali_slaves}
        self.dali_block0:  dict[int, list[tuple[int, int]]] = {
            s: [(0, 0)] * 64 for s in self._dali_slaves
        }
        self.dali_block1:  dict[int, list[tuple[int, int]]] = {
            s: [(0, 0)] * 64 for s in self._dali_slaves
        }
        self.dali_block2:  dict[int, list[tuple[int, int]]] = {
            s: [(0, 0)] * 64 for s in self._dali_slaves
        }
        self.dali_block3:  dict[int, list[tuple[int, int]]] = {
            s: [(0, 0)] * 64 for s in self._dali_slaves
        }

        # ── Provisioning state (read by sensor/button/number entities) ────
        self.provision_state:  dict[int, str] = {s: "idle" for s in self._dali_slaves}
        self.provision_msg:    dict[int, str] = {s: "Idle" for s in self._dali_slaves}
        self.provision_count:  dict[int, int] = {s: 0     for s in self._dali_slaves}
        self.provision_target: dict[int, int] = {s: 0     for s in self._dali_slaves}

        # ── Button event entity map — populated by event.py after setup ───
        # Key: (fixture, button), Value: ButtonEventEntity instance
        self._button_entities: dict[tuple[int, int], "ButtonEventEntity"] = {}

    # ── Lifecycle ─────────────────────────────────────────────────────────

    async def async_connect(self) -> bool:
        connected = await self._client.connect()
        if not connected:
            return False
        async with self._lock:
            for slave in self._dali_slaves:
                try:
                    await self._client.write_register(
                        DALI_REG_RESCAN_INTERVAL, 0, slave=slave
                    )
                except ModbusException as exc:
                    _LOGGER.warning(
                        "Could not disable auto-rescan on slave %d: %s", slave, exc
                    )
        return True

    async def async_disconnect(self) -> None:
        self._client.close()

    # ── Button entity registration ─────────────────────────────────────────

    def register_button_entities(
        self,
        entity_map: dict[tuple[int, int], "ButtonEventEntity"],
    ) -> None:
        """Called by event.py after entities are created."""
        self._button_entities = entity_map

    def fire_button_event(
        self, fixture: int, button: int, event_type: str
    ) -> None:
        """Called when a button event arrives from the Pico over serial.

        Maps the raw event type string to the HA event type constant and
        fires it on the correct entity.  Silently ignores unknown fixtures,
        buttons, or event types — they may arrive before entities are ready.
        """
        ha_event = _BTN_EVENT_MAP.get(event_type)
        if ha_event is None:
            _LOGGER.debug("Unknown button event type: %s", event_type)
            return
        entity = self._button_entities.get((fixture, button))
        if entity is None:
            _LOGGER.debug(
                "No entity for fixture %d button %d", fixture, button
            )
            return
        entity.fire(ha_event)

    # ── Provisioning command dispatch ──────────────────────────────────────

    async def async_provision_start(
        self,
        slave: int,
        mode: str,
        target: int = 0,
    ) -> None:
        """Called by button entities to start a provisioning run."""
        # The coordinator relays this to the Pico via the serial connection.
        # In the Modbus-only integration (no Pico serial) this would issue
        # the DALI commissioning sequence directly.  For now we log it and
        # update provision state optimistically.
        _LOGGER.info(
            "Provision start: slave=%d mode=%s target=%d", slave, mode, target
        )
        self.provision_state[slave] = "starting"
        self.provision_msg[slave]   = f"Starting {mode} commission..."
        self.provision_count[slave] = 0
        self.async_update_listeners()

    async def async_provision_abort(self, slave: int) -> None:
        """Called by the Abort button."""
        _LOGGER.info("Provision abort: slave=%d", slave)
        self.provision_state[slave] = "idle"
        self.provision_msg[slave]   = "Aborted"
        self.async_update_listeners()

    def handle_provision_status(
        self,
        slave: int,
        state: str,
        msg: str,
        devices_found: int,
    ) -> None:
        """Called when a provision_status message arrives from the Pico."""
        self.provision_state[slave] = state
        self.provision_msg[slave]   = msg
        self.provision_count[slave] = devices_found
        self.async_update_listeners()

    # ── DataUpdateCoordinator entry point ──────────────────────────────────

    async def _async_update_data(self) -> dict[str, Any]:
        async with self._lock:
            await self._fast_poll()
            self._full_scan_counter += 1
            if self._full_scan_counter >= FULL_SCAN_INTERVAL:
                self._full_scan_counter = 0
                await self._full_scan()
        return self._snapshot()

    def _snapshot(self) -> dict[str, Any]:
        return {
            "relay_coils":   list(self.relay_coils),
            "dali_present":  dict(self.dali_present),
            "dali_block0":   {s: list(v) for s, v in self.dali_block0.items()},
            "dali_block1":   {s: list(v) for s, v in self.dali_block1.items()},
            "dali_block2":   {s: list(v) for s, v in self.dali_block2.items()},
            "dali_block3":   {s: list(v) for s, v in self.dali_block3.items()},
            "provision_state": dict(self.provision_state),
            "provision_msg":   dict(self.provision_msg),
            "provision_count": dict(self.provision_count),
        }

    # ── Fast poll ─────────────────────────────────────────────────────────

    async def _fast_poll(self) -> None:
        await self._poll_relays()
        for slave in self._dali_slaves:
            await self._poll_dali_block0(slave)

    async def _poll_relays(self) -> None:
        try:
            result = await self._client.read_coils(
                RELAY_COIL_START, count=RELAY_COIL_COUNT, slave=self._relay_slave
            )
            if not result.isError():
                self.relay_coils = list(result.bits[:RELAY_COIL_COUNT])
        except ModbusException as exc:
            _LOGGER.debug("Relay poll failed: %s", exc)

    async def _poll_dali_block0(self, slave: int) -> None:
        new_block: list[tuple[int, int]] = list(self.dali_block0[slave])
        for offset, coro in _read_block_chunked(
            self._client, DALI_BLOCK0_START, DALI_BLOCK0_COUNT, slave
        ):
            try:
                result = await coro
                if result.isError():
                    continue
                for i, reg in enumerate(result.registers):
                    new_block[offset + i] = ((reg >> 8) & 0xFF, reg & 0xFF)
            except ModbusException as exc:
                _LOGGER.debug("Block0 slave %d offset %d: %s", slave, offset, exc)
        self.dali_block0[slave] = new_block

    # ── Full scan ─────────────────────────────────────────────────────────

    async def _full_scan(self) -> None:
        for slave in self._dali_slaves:
            await self._trigger_firmware_scan(slave)
            await self._poll_present_bitmask(slave)
            await self._poll_dali_block0(slave)
            await self._poll_dali_block(
                slave, DALI_BLOCK1_START, DALI_BLOCK1_COUNT,
                "block1", self.dali_block1
            )
            await self._poll_dali_block(
                slave, DALI_BLOCK2_START, DALI_BLOCK2_COUNT,
                "block2", self.dali_block2
            )
            await self._poll_dali_block(
                slave, DALI_BLOCK3_START, DALI_BLOCK3_COUNT,
                "block3", self.dali_block3
            )

    async def _trigger_firmware_scan(self, slave: int) -> None:
        try:
            await self._client.write_register(
                DALI_REG_SCAN_TRIGGER, 0x0001, slave=slave
            )
        except ModbusException as exc:
            _LOGGER.debug("Scan trigger failed slave %d: %s", slave, exc)

    async def _poll_present_bitmask(self, slave: int) -> None:
        try:
            result = await self._client.read_input_registers(
                DALI_PRESENT_BITMASK_START,
                count=DALI_PRESENT_BITMASK_COUNT,
                slave=slave,
            )
            if not result.isError():
                self.dali_present[slave] = _bitmask_from_registers(
                    result.registers
                )
        except ModbusException as exc:
            _LOGGER.debug("Present bitmask slave %d: %s", slave, exc)

    async def _poll_dali_block(
        self, slave, start, count, name, store
    ) -> None:
        new_block: list[tuple[int, int]] = list(store[slave])
        for offset, coro in _read_block_chunked(
            self._client, start, count, slave
        ):
            try:
                result = await coro
                if result.isError():
                    continue
                for i, reg in enumerate(result.registers):
                    new_block[offset + i] = ((reg >> 8) & 0xFF, reg & 0xFF)
            except ModbusException as exc:
                _LOGGER.debug(
                    "%s slave %d offset %d: %s", name, slave, offset, exc
                )
        store[slave] = new_block

    # ── Write helpers ──────────────────────────────────────────────────────

    async def async_set_relay(self, channel: int, state: bool) -> None:
        async with self._lock:
            try:
                value  = 0xFF00 if state else 0x0000
                result = await self._client.write_coil(
                    RELAY_COIL_START + channel, value, slave=self._relay_slave
                )
                if not result.isError():
                    self.relay_coils[channel] = state
            except ModbusException as exc:
                _LOGGER.error("Relay write channel %d: %s", channel, exc)
                raise

    async def async_dali_dapc(
        self, slave: int, dali_addr: int, arc_level: int
    ) -> None:
        addr_byte = (dali_addr & 0x3F) << 1
        value     = (addr_byte << 8) | (arc_level & 0xFF)
        async with self._lock:
            try:
                result = await self._client.write_register(
                    DALI_REG_DAPC, value, slave=slave
                )
                if not result.isError():
                    status, _ = self.dali_block0[slave][dali_addr]
                    self.dali_block0[slave][dali_addr] = (status, arc_level)
            except ModbusException as exc:
                _LOGGER.error(
                    "DAPC slave %d addr %d arc %d: %s",
                    slave, dali_addr, arc_level, exc
                )
                raise

    async def async_dali_on(self, slave: int, dali_addr: int) -> None:
        addr_byte = ((dali_addr & 0x3F) << 1) | 0x01
        async with self._lock:
            try:
                await self._client.write_register(
                    DALI_REG_GOTO_LAST_ACTIVE, addr_byte, slave=slave
                )
                status, arc = self.dali_block0[slave][dali_addr]
                if arc == 0:
                    self.dali_block0[slave][dali_addr] = (status, 1)
            except ModbusException as exc:
                _LOGGER.error(
                    "DALI on slave %d addr %d: %s", slave, dali_addr, exc
                )
                raise

    async def async_dali_off(self, slave: int, dali_addr: int) -> None:
        addr_byte = ((dali_addr & 0x3F) << 1) | 0x01
        async with self._lock:
            try:
                result = await self._client.write_register(
                    DALI_REG_OFF, addr_byte, slave=slave
                )
                if not result.isError():
                    status, _ = self.dali_block0[slave][dali_addr]
                    self.dali_block0[slave][dali_addr] = (status, 0)
            except ModbusException as exc:
                _LOGGER.error(
                    "DALI off slave %d addr %d: %s", slave, dali_addr, exc
                )
                raise

    # ── Convenience ───────────────────────────────────────────────────────

    def is_ballast_present(self, slave: int, dali_addr: int) -> bool:
        return bool(self.dali_present.get(slave, 0) & (1 << dali_addr))

    def present_ballasts(self, slave: int) -> list[int]:
        mask = self.dali_present.get(slave, 0)
        return [i for i in range(64) if mask & (1 << i)]
