"""
Event platform — button press/release events from switch plates.

One event entity per physical button (fixture × button).
All four event types share the same entity — HA automations filter by
trigger.event.event_type.

Device: Switch Plate N (fixture 0–23).
        Named "Switch Plate 01" etc — user renames and assigns to rooms
        in the HA device registry UI.

Event types:
  press              — button tapped and released within long-press threshold
  release            — button released after any hold
  long_press_start   — hold threshold crossed; dimming begins
  long_press_repeat  — emitted at dim_repeat_ms rate while held (~5 Hz)

The coordinator receives button events from the Pico via the serial protocol
and calls async_fire_event() on the relevant entity.
"""

from __future__ import annotations

import logging

from homeassistant.components.event import EventEntity
from homeassistant.config_entries import ConfigEntry
from homeassistant.core import HomeAssistant
from homeassistant.helpers.entity_platform import AddEntitiesCallback

from .const import (
    BUTTON_EVENT_TYPES,
    DOMAIN,
    MAX_BUTTONS,
    MAX_FIXTURES,
)
from .coordinator import LightingModbusCoordinator
from .device_registry import switch_plate_device

_LOGGER = logging.getLogger(__name__)


async def async_setup_entry(
    hass: HomeAssistant,
    entry: ConfigEntry,
    async_add_entities: AddEntitiesCallback,
) -> None:
    coordinator: LightingModbusCoordinator = hass.data[DOMAIN][entry.entry_id]
    entities: list[ButtonEventEntity] = []

    for fixture in range(MAX_FIXTURES):
        for button in range(MAX_BUTTONS):
            entities.append(
                ButtonEventEntity(coordinator, entry, fixture, button)
            )

    async_add_entities(entities)

    # Register the entity map with the coordinator so it can fire events
    coordinator.register_button_entities(
        {(e.fixture, e.button): e for e in entities}
    )


class ButtonEventEntity(EventEntity):
    """A single physical button on a switch plate.

    Does not hold state — fires an event each time the button changes.
    The entity lives on the Switch Plate device, which users name and
    assign to a room/area in the HA device registry.
    """

    _attr_should_poll = False
    _attr_event_types = BUTTON_EVENT_TYPES

    def __init__(
        self,
        coordinator: LightingModbusCoordinator,
        entry: ConfigEntry,
        fixture: int,
        button: int,
    ) -> None:
        self._coordinator = coordinator
        self.fixture   = fixture
        self.button    = button

        # Name: "Button N" within the switch plate device — keeps it terse
        # since the device name already conveys the plate identity.
        self._attr_name      = f"Button {button + 1}"
        self._attr_unique_id = (
            f"{entry.entry_id}_switch_{fixture:02d}_btn_{button}"
        )

        # All buttons for a fixture live on the same switch plate device
        self._attr_device_info = switch_plate_device(entry.entry_id, fixture)

        self._attr_icon = "mdi:gesture-tap-button"

    def fire(self, event_type: str) -> None:
        """Called by the coordinator when a button event arrives from the Pico."""
        self._trigger_event(
            event_type,
            {
                "fixture": self.fixture,
                "button":  self.button,
            },
        )
        self.async_write_ha_state()
