"""
mqtt_bridge.py — serial-to-MQTT bridge for the switch gateway Pico.

DEPLOYMENT MODEL
----------------
Used when the Pico 2 is the Modbus master.  The Pico owns all RS-485
devices and publishes state over USB CDC serial as line-delimited JSON.
This bridge translates that to MQTT and publishes HA auto-discovery payloads.

AVAILABILITY MODEL
------------------
System availability (Pico connected / disconnected):
    lighting/availability  →  "online" / "offline"

Per-ballast availability (DALI presence bitmask):
    lighting/dali/<slave>/<addr>/availability  →  "online" / "offline"

Each DALI light discovery payload lists BOTH topics with availability_mode "all",
so a ballast is only shown as available in HA when:
  - The Pico is connected, AND
  - The ballast responded in the most recent DALI scan

On hello:
  - All 64 addresses per DALI slave are published (discovery + offline)
  - scan_complete then sets present ones to online, absent ones stay offline
  - Relay channels and switch plates use only the system availability topic

On serial disconnect:
  - System topic goes offline — all entities become unavailable immediately

NAMING
------
All friendly names and room assignments are done in Home Assistant's device
registry after discovery.  Relay channels default to switch entities — the
user can override to light or fan in HA's entity registry UI.

USAGE
-----
  python3 mqtt_bridge.py \\
      --serial /dev/serial/by-id/usb-Raspberry_Pi_Pico_2-if00 \\
      --broker localhost \\
      [--port 1883] [--user U] [--password P] [--debug]
"""

from __future__ import annotations

import argparse
import json
import logging
import logging.handlers
import os
import re
import threading
import time
from dataclasses import dataclass
from typing import Any

import serial
import paho.mqtt.client as mqtt

try:
    from dotenv import load_dotenv
except ImportError:  # optional dependency; env vars still work without a .env file
    def load_dotenv(*_a, **_k):  # type: ignore
        return False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
log = logging.getLogger("bridge")

BUTTON_EVENT_TYPES = ["press", "release", "long_press_start", "long_press_repeat"]
MAX_FIXTURES       = 24
MAX_BUTTONS        = 7
RELAY_CHANNELS     = 32   # coils the controller reads per relay board
DALI_ADDR_COUNT    = 64


# ── Config ────────────────────────────────────────────────────────────────────

@dataclass
class BridgeConfig:
    serial_port:        str
    serial_baud:        int
    mqtt_broker:        str
    mqtt_port:          int
    mqtt_user:          str | None
    mqtt_password:      str | None
    topic_prefix:       str
    availability_topic: str
    serial_log_path:    str | None = None


# ── HA entity namespace ─────────────────────────────────────────────────────────
# NODE prefixes every HA unique_id + topic ("lighting_dali_2_1",
# "lighting/dali/2/1/state"). One lighting system per broker, so it's a fixed
# constant, not configurable. LABEL is kept (empty) so device names stay clean.
NODE  = "lighting"
LABEL = ""


# ── Device info builders ───────────────────────────────────────────────────────

def _dev_relay_board() -> dict:
    return {
        "identifiers":  [f"{NODE}_relay_board"],
        "name":         f"{LABEL}Relay Board",
        "manufacturer": "Waveshare",
        "model":        "Modbus RTU Relay 32CH",
    }


def _dev_relay_channel(channel: int) -> dict:
    return {
        "identifiers":  [f"{NODE}_relay_{channel}"],
        "name":         f"{LABEL}Relay Channel {channel}",
        "manufacturer": "Waveshare",
        "model":        "Relay Channel",
        "via_device":   f"{NODE}_relay_board",
    }


def _dev_dali_gateway(slave: int) -> dict:
    return {
        "identifiers":  [f"{NODE}_dali_gw_{slave}"],
        "name":         f"{LABEL}DALI Gateway {slave}",
        "manufacturer": "Mechination",
        "model":        "TinyModbus ATtiny1604 DALI Gateway",
    }


def _dev_dali_ballast(slave: int, addr: int) -> dict:
    return {
        "identifiers":  [f"{NODE}_dali_{slave}_{addr}"],
        "name":         f"{LABEL}DALI {slave}.{addr:02d}",
        "manufacturer": "Mechination",
        "model":        "DALI Ballast",
        "via_device":   f"{NODE}_dali_gw_{slave}",
    }


def _dev_switch_plate(fixture: int) -> dict:
    return {
        "identifiers":  [f"{NODE}_switch_plate_{fixture:02d}"],
        "name":         f"{LABEL}Fixture {fixture + 1:02d}",
        "manufacturer": "Mechination",
        "model":        "Switch Gateway Button Matrix",
        "via_device":   f"{NODE}_controller",
    }


def _dev_dali_group(slave: int, group: int) -> dict:
    return {
        "identifiers":  [f"{NODE}_group_{slave}_{group}"],
        "name":         f"{LABEL}DALI Group {slave}.{group}",
        "manufacturer": "Mechination",
        "model":        "DALI Group",
        "via_device":   f"{NODE}_dali_gw_{slave}",
    }


def _dev_controller(fw_version: str | None = None) -> dict:
    d = {
        "identifiers":  [f"{NODE}_controller"],
        "name":         f"{LABEL}Switch Gateway Controller",
        "manufacturer": "Mechination",
        "model":        "Raspberry Pi Pico 2 Controller",
    }
    if fw_version:
        d["sw_version"] = fw_version
    return d


# Per-ballast DALI config fields exposed as HA number entities.
#   (key, friendly name, min, max)  — 0..254 levels, 0..15 fade rate/time
DALI_CONFIG_FIELDS = [
    ("min",       "Min Level",  0, 254),
    ("max",       "Max Level",  0, 254),
    ("power_on",  "Power-On Level",   0, 254),
    ("sys_fail",  "System-Failure Level", 0, 254),
    ("fade_rate", "Fade Rate",  0, 15),
    ("fade_time", "Fade Time",  0, 15),
]

# Controller timing config exposed as HA number entities.
CONTROLLER_CONFIG_FIELDS = [
    ("long_press_ms",      "Long-Press (ms)",      100, 5000, 50),
    ("dim_repeat_ms",      "Dim Repeat (ms)",       50, 2000, 10),
    ("reversal_window_ms", "Reversal Window (ms)",  50, 2000, 10),
]

# A binding is a comma list of "<slave>:<addr>" (ballast) or "<slave>:g<group>"
# targets, or empty to clear.  This regex is BOTH the HA text-entity client-side
# validation pattern and (mirrored by _parse_binding_str) the bridge-side parser.
BINDING_PATTERN = r"^\s*(\d{1,3}\s*:\s*[gG]?\d{1,3}(\s*,\s*\d{1,3}\s*:\s*[gG]?\d{1,3})*)?\s*$"


def _binding_to_str(targets: list) -> str:
    """[{slave,addr}|{slave,group}] → 's:a,s:gN' string for the HA text entity."""
    out = []
    for t in targets:
        if "group" in t:
            out.append(f"{t['slave']}:g{t['group']}")
        else:
            out.append(f"{t['slave']}:{t.get('addr', 0)}")
    return ",".join(out)


def _parse_binding_str(s: str) -> list | None:
    """'s:a,s:gN' → [{slave,addr}|{slave,group}].  None if malformed (empty=[])."""
    s = s.strip()
    if not s:
        return []
    targets: list = []
    for part in s.split(","):
        sl, _, ad = part.strip().partition(":")
        sl, ad = sl.strip(), ad.strip()
        if not sl.isdigit() or not ad:
            return None
        try:
            if ad[0] in "gG":
                targets.append({"slave": int(sl), "group": int(ad[1:])})
            else:
                targets.append({"slave": int(sl), "addr": int(ad)})
        except ValueError:
            return None
    return targets


# ── Bridge ────────────────────────────────────────────────────────────────────

class Bridge:
    """
    Translates between Pico serial JSON and MQTT.

    Thread model:
      Main thread      — paho loop_forever() (MQTT network + callbacks)
      serial-reader    — reads serial, calls dispatch_from_pico()
      serial_lock      — guards all serial writes
    """

    def __init__(self, cfg: BridgeConfig) -> None:
        self._cfg         = cfg
        self._serial_lock = threading.Lock()
        self._serial: serial.Serial | None = None
        self._cmd_id      = 0
        self._mqtt: mqtt.Client

        # Tracks whether discovery has been published for each entity.
        # Cleared on hello so reconnect republishes cleanly.
        self._disc_relays:      set[int]             = set()
        self._relay_slaves:     list[int]            = []   # relay board slave ids (from hello)
        self._disc_ballasts:    set[tuple[int, int]] = set()
        self._disc_gateways:    set[int]             = set()
        self._disc_ballast_cfg: set[tuple[int, int]] = set()
        self._disc_group:       set[tuple[int, int]] = set()
        self._disc_controller:  bool                 = False
        self._fw_version:       str                  = "?"

        # Per-slave target address for "Commission Address" button
        self._prov_target: dict[int, int] = {}

        # Pre-compile MQTT command topic patterns from configured prefix
        p = re.escape(cfg.topic_prefix)
        self._RE: list[tuple[re.Pattern, str]] = [
            (re.compile(rf"^{p}/relay/(\d+)/set$"),                    "_cmd_relay"),
            (re.compile(rf"^{p}/dali/(\d+)/(\d+)/config/([a-z_]+)/set$"), "_cmd_dali_config"),
            (re.compile(rf"^{p}/dali/(\d+)/(\d+)/set$"),              "_cmd_dali"),
            (re.compile(rf"^{p}/group/(\d+)/(\d+)/members/set$"),     "_cmd_group_members"),
            (re.compile(rf"^{p}/group/(\d+)/(\d+)/set$"),             "_cmd_group"),
            (re.compile(rf"^{p}/provision/(\d+)/start/all$"),         "_cmd_prov_all"),
            (re.compile(rf"^{p}/provision/(\d+)/start/unaddressed$"), "_cmd_prov_unaddressed"),
            (re.compile(rf"^{p}/provision/(\d+)/start/target$"),      "_cmd_prov_target"),
            (re.compile(rf"^{p}/provision/(\d+)/start/address$"),     "_cmd_prov_address"),
            (re.compile(rf"^{p}/provision/(\d+)/abort$"),             "_cmd_prov_abort"),
            (re.compile(rf"^{p}/mapping/(\d+)/(\d+)/set$"),           "_cmd_set_mapping"),
            (re.compile(rf"^{p}/mapping/(\d+)/(\d+)/delete$"),        "_cmd_del_mapping"),
            (re.compile(rf"^{p}/binding/(\d+)/(\d+)/set$"),           "_cmd_binding"),
            (re.compile(rf"^{p}/config/([a-z_]+)/set$"),             "_cmd_cfg"),
            (re.compile(rf"^{p}/config/set$"),                         "_cmd_set_config"),
        ]

        # Optional: tee the controller's "# ..." serial log lines to a rotating
        # file so the user can `tail -F` and debug along. Disabled if path empty.
        self._serial_log: logging.Logger | None = None
        if cfg.serial_log_path:
            slog = logging.getLogger("controller-serial")
            slog.setLevel(logging.INFO)
            slog.propagate = False
            if not slog.handlers:
                h = logging.handlers.RotatingFileHandler(
                    cfg.serial_log_path, maxBytes=2_000_000, backupCount=3)
                h.setFormatter(logging.Formatter("%(asctime)s %(message)s"))
                slog.addHandler(h)
            self._serial_log = slog
            log.info("Controller serial log → %s", cfg.serial_log_path)

    # ── Helpers ───────────────────────────────────────────────────────────

    def _next_id(self) -> int:
        self._cmd_id = (self._cmd_id + 1) & 0xFFFF
        return self._cmd_id

    def _t(self, *parts: str) -> str:
        return "/".join([self._cfg.topic_prefix, *parts])

    def _pub(self, topic: str, payload: Any, retained: bool = False) -> None:
        if not isinstance(payload, str):
            payload = json.dumps(payload)
        self._mqtt.publish(topic, payload, retain=retained)
        log.debug("MQTT ← %s : %.120s", topic, payload)

    def _send(self, msg: dict) -> None:
        line = json.dumps(msg) + "\n"
        with self._serial_lock:
            if self._serial and self._serial.is_open:
                try:
                    self._serial.write(line.encode())
                    log.debug("SERIAL → %s", line.strip())
                except serial.SerialException as exc:
                    log.error("Serial write failed: %s", exc)

    def _ballast_avail_topic(self, slave: int, addr: int) -> str:
        return self._t("dali", str(slave), str(addr), "availability")

    def _gateway_avail_topic(self, slave: int) -> str:
        return self._t("gateway", str(slave), "availability")

    def _gw_avail(self, slave: int, *extra: str) -> dict:
        """HA availability fragment gating on system + gateway (+ extra) topics,
        all required online."""
        topics = [self._cfg.availability_topic, self._gateway_avail_topic(slave),
                  *extra]
        return {"availability": [{"topic": t} for t in topics],
                "availability_mode": "all"}

    def _set_ballast_availability(self, slave: int, addr: int,
                                   online: bool) -> None:
        self._pub(
            self._ballast_avail_topic(slave, addr),
            "online" if online else "offline",
            retained=True,
        )

    # ── Discovery ─────────────────────────────────────────────────────────

    def _pub_all_relays_for_slave(self, slave: int) -> None:
        """Publish all 32 relay channels for a board, gated on its reachability, so
        the channels appear (unavailable) even when the board never reports state —
        mirroring how all 64 DALI addresses are seeded per gateway."""
        for ch in range(RELAY_CHANNELS):
            self._pub_relay_discovery(ch, slave)

    def _pub_relay_discovery(self, channel: int, slave=None) -> None:
        """Publish discovery for one relay channel as a switch (once per session)."""
        if channel in self._disc_relays:
            return
        self._disc_relays.add(channel)
        if slave is None and self._relay_slaves:
            slave = self._relay_slaves[0]
        uid = f"{NODE}_relay_{channel}"
        disc = {
            "unique_id":            uid,
            # name=None → HA uses device name; user can rename in device registry
            "name":                 None,
            "state_topic":          self._t("relay", str(channel), "state"),
            "command_topic":        self._t("relay", str(channel), "set"),
            "state_value_template": "{{ value_json.state }}",
            "payload_on":           '{"state":"ON"}',
            "payload_off":          '{"state":"OFF"}',
            "device":               _dev_relay_channel(channel),
        }
        # Gate on system + the relay board's reachability (its state_gateway topic),
        # so a channel goes unavailable when the board is absent — like a ballast.
        if slave is not None:
            disc["availability"] = [
                {"topic": self._cfg.availability_topic},
                {"topic": self._gateway_avail_topic(slave)},
            ]
            disc["availability_mode"] = "all"
        else:
            disc["availability_topic"] = self._cfg.availability_topic
        self._pub(f"homeassistant/switch/{uid}/config", disc, retained=True)

    def _pub_ballast_discovery(self, slave: int, addr: int) -> None:
        """
        Publish discovery for one DALI ballast (once per session).

        Uses two availability topics with mode "all":
          1. System topic  — goes offline when Pico disconnects
          2. Per-ballast   — goes offline when ballast stops responding

        The ballast is seeded as offline; scan_complete / presence sets it online.
        """
        if (slave, addr) in self._disc_ballasts:
            return
        self._disc_ballasts.add((slave, addr))

        uid      = f"{NODE}_dali_{slave}_{addr}"
        avail_t  = self._ballast_avail_topic(slave, addr)

        self._pub(
            f"homeassistant/light/{uid}/config",
            {
                "unique_id":                 uid,
                "name":                      None,
                "schema":                    "json",
                "state_topic":               self._t("dali", str(slave), str(addr), "state"),
                "command_topic":             self._t("dali", str(slave), str(addr), "set"),
                # Modern brightness declaration (the standalone "brightness" flag
                # no longer reliably renders a slider). JSON-schema lights read
                # state/color_mode/brightness from the JSON state payload.
                "supported_color_modes":     ["brightness"],
                "brightness_scale":          254,
                # System + gateway + per-ballast must all be online.
                **self._gw_avail(slave, avail_t),
                "device":                    _dev_dali_ballast(slave, addr),
            },
            retained=True,
        )

        # Seed as offline — scan_complete will flip to online if present
        self._set_ballast_availability(slave, addr, False)

    def _pub_all_ballasts_for_slave(self, slave: int) -> None:
        """
        Publish discovery for all 64 addresses on a DALI slave, all offline.
        Called on hello so every possible ballast exists in HA immediately.
        scan_complete then brings present ones online.
        """
        for addr in range(DALI_ADDR_COUNT):
            self._pub_ballast_discovery(slave, addr)

    def _pub_all_groups_for_slave(self, slave: int) -> None:
        """
        Publish discovery for all 16 DALI groups on a slave, seeded empty/off.
        DALI groups are a fixed 0-15 set per gateway, so every group is always
        present (gated on gateway availability) — which lets the mapper populate
        an empty group. The controller's state_group then fills in the ones that
        actually have members.
        """
        for group in range(16):
            self._pub_group_discovery(slave, group)
            self._pub(self._t("group", str(slave), str(group), "state"),
                      {"state": "OFF"}, retained=True)
            self._pub(self._t("group", str(slave), str(group), "members", "state"),
                      "", retained=True)

    def _pub_gateway_discovery(self, slave: int) -> None:
        """Publish provisioning entities for one DALI gateway (once per session)."""
        if slave in self._disc_gateways:
            return
        self._disc_gateways.add(slave)

        device   = _dev_dali_gateway(slave)
        uid_base = f"{NODE}_prov_{slave}"
        status_t = self._t("provision", str(slave), "status")

        # Diagnostic sensors
        for suffix, name, icon, tmpl in [
            ("state", "Commission State",  "mdi:lightning-bolt-circle",
             "{{ value_json.state }}"),
            ("msg",   "Commission Status", "mdi:text-box-outline",
             "{{ value_json.msg }}"),
            ("count", "Devices Found",     "mdi:counter",
             "{{ value_json.devices_found }}"),
        ]:
            self._pub(
                f"homeassistant/sensor/{uid_base}_{suffix}/config",
                {
                    "unique_id":          f"{uid_base}_{suffix}",
                    "name":               name,
                    "state_topic":        status_t,
                    "value_template":     tmpl,
                    **self._gw_avail(slave),
                    "icon":               icon,
                    "entity_category":    "diagnostic",
                    "device":             device,
                },
                retained=True,
            )

        # Config buttons
        for suffix, name, icon, cmd in [
            ("all",         "Commission All",         "mdi:lightbulb-group",
             self._t("provision", str(slave), "start", "all")),
            ("unaddressed", "Commission New Devices", "mdi:lightbulb-plus",
             self._t("provision", str(slave), "start", "unaddressed")),
            ("address",     "Commission Address",     "mdi:lightbulb-on",
             self._t("provision", str(slave), "start", "address")),
            ("abort",       "Abort Commission",       "mdi:stop-circle",
             self._t("provision", str(slave), "abort")),
        ]:
            self._pub(
                f"homeassistant/button/{uid_base}_{suffix}/config",
                {
                    "unique_id":          f"{uid_base}_{suffix}",
                    "name":               name,
                    "command_topic":      cmd,
                    "payload_press":      "PRESS",
                    **self._gw_avail(slave),
                    "icon":               icon,
                    "entity_category":    "config",
                    "device":             device,
                },
                retained=True,
            )

        # Target address number
        self._pub(
            f"homeassistant/number/{uid_base}_target/config",
            {
                "unique_id":          f"{uid_base}_target",
                "name":               "Target Address",
                "command_topic":      self._t("provision", str(slave), "start", "target"),
                "state_topic":        self._t("provision", str(slave), "target_state"),
                "min":                0,
                "max":                63,
                "step":               1,
                "mode":               "box",
                **self._gw_avail(slave),
                "icon":               "mdi:numeric",
                "entity_category":    "config",
                "device":             device,
            },
            retained=True,
        )

        # Seed retained state topics
        self._pub(
            self._t("provision", str(slave), "target_state"), "0", retained=True
        )
        self._pub(
            status_t,
            {"state": "idle", "msg": "Idle", "devices_found": 0},
            retained=True,
        )

    def _pub_switch_plate_discovery(self, fixture: int) -> None:
        device = _dev_switch_plate(fixture)
        for btn in range(MAX_BUTTONS):
            uid = f"{NODE}_switch_{fixture:02d}_btn_{btn}"
            self._pub(
                f"homeassistant/event/{uid}/config",
                {
                    "unique_id":          uid,
                    "name":               f"Button {btn + 1}",
                    "state_topic":        self._t("button", str(fixture), str(btn)),
                    "event_types":        BUTTON_EVENT_TYPES,
                    "value_template":     "{{ value_json.event_type }}",
                    "availability_topic": self._cfg.availability_topic,
                    "icon":               "mdi:gesture-tap-button",
                    "device":             device,
                },
                retained=True,
            )
            # Editable binding for this button: a text entity holding the targets
            # as "<slave>:<addr>" / "<slave>:g<group>" comma list (regex-validated).
            buid = f"{NODE}_binding_{fixture:02d}_btn_{btn}"
            self._pub(
                f"homeassistant/text/{buid}/config",
                {
                    "unique_id":          buid,
                    "name":               f"Button {btn + 1} Binding",
                    "command_topic":      self._t("binding", str(fixture), str(btn), "set"),
                    "state_topic":        self._t("binding", str(fixture), str(btn), "state"),
                    "pattern":            BINDING_PATTERN,
                    "availability_topic": self._cfg.availability_topic,
                    "entity_category":    "config",
                    "icon":               "mdi:link-variant",
                    "device":             device,
                },
                retained=True,
            )
            self._pub(self._t("binding", str(fixture), str(btn), "state"), "", retained=True)

    def _pub_ballast_config_discovery(self, slave: int, addr: int) -> None:
        """Number entities for a present ballast's DALI config (config category)."""
        if (slave, addr) in self._disc_ballast_cfg:
            return
        self._disc_ballast_cfg.add((slave, addr))
        device  = _dev_dali_ballast(slave, addr)
        state_t = self._t("dali", str(slave), str(addr), "config")
        gwa     = self._gw_avail(slave, self._ballast_avail_topic(slave, addr))
        for key, name, lo, hi in DALI_CONFIG_FIELDS:
            uid = f"{NODE}_dali_{slave}_{addr}_{key}"
            self._pub(
                f"homeassistant/number/{uid}/config",
                {
                    "unique_id":          uid,
                    "name":               name,
                    "command_topic":      self._t("dali", str(slave), str(addr),
                                                  "config", key, "set"),
                    "state_topic":        state_t,
                    "value_template":     f"{{{{ value_json.{key} }}}}",
                    "min": lo, "max": hi, "step": 1, "mode": "box",
                    **gwa,
                    "entity_category":    "config",
                    "device":             device,
                },
                retained=True,
            )

    def _pub_group_discovery(self, slave: int, group: int) -> None:
        """A DALI group as a light (on/bri) + an editable members text entity."""
        if (slave, group) in self._disc_group:
            return
        self._disc_group.add((slave, group))
        device = _dev_dali_group(slave, group)
        uid    = f"{NODE}_group_{slave}_{group}"
        self._pub(
            f"homeassistant/light/{uid}/config",
            {
                "unique_id":                 uid,
                "name":                      None,
                "schema":                    "json",
                "state_topic":               self._t("group", str(slave), str(group), "state"),
                "command_topic":             self._t("group", str(slave), str(group), "set"),
                "supported_color_modes":     ["brightness"],
                "brightness_scale":          254,
                **self._gw_avail(slave),
                "device":                    device,
            },
            retained=True,
        )
        muid = f"{uid}_members"
        self._pub(
            f"homeassistant/text/{muid}/config",
            {
                "unique_id":          muid,
                "name":               "Members",
                "command_topic":      self._t("group", str(slave), str(group), "members", "set"),
                "state_topic":        self._t("group", str(slave), str(group), "members", "state"),
                "pattern":            r"^\s*(\d{1,2}(\s*,\s*\d{1,2})*)?\s*$",
                **self._gw_avail(slave),
                "entity_category":    "config",
                "icon":               "mdi:format-list-bulleted",
                "device":             device,
            },
            retained=True,
        )

    def _pub_controller_discovery(self, v: dict) -> None:
        """Controller device: timing number entities + fw/baud sensors."""
        if self._disc_controller:
            return
        self._disc_controller = True
        device  = _dev_controller(v.get("fw_version"))
        state_t = self._t("config", "state")
        for key, name, lo, hi, step in CONTROLLER_CONFIG_FIELDS:
            uid = f"{NODE}_cfg_{key}"
            self._pub(
                f"homeassistant/number/{uid}/config",
                {
                    "unique_id":          uid,
                    "name":               name,
                    "command_topic":      self._t("config", key, "set"),
                    "state_topic":        state_t,
                    "value_template":     f"{{{{ value_json.{key} }}}}",
                    "min": lo, "max": hi, "step": step, "mode": "box",
                    "availability_topic": self._cfg.availability_topic,
                    "entity_category":    "config",
                    "device":             device,
                },
                retained=True,
            )
        for key, name, icon in [
            ("fw_version", "Firmware",    "mdi:chip"),
            ("baud",       "Modbus Baud", "mdi:speedometer"),
        ]:
            uid = f"{NODE}_cfg_{key}"
            self._pub(
                f"homeassistant/sensor/{uid}/config",
                {
                    "unique_id":          uid,
                    "name":               name,
                    "state_topic":        state_t,
                    "value_template":     f"{{{{ value_json.{key} }}}}",
                    "availability_topic": self._cfg.availability_topic,
                    "entity_category":    "diagnostic",
                    "icon":               icon,
                    "device":             device,
                },
                retained=True,
            )

    # ── Inbound from Pico ─────────────────────────────────────────────────

    def dispatch_from_pico(self, line: str) -> None:
        line = line.strip()
        if not line.startswith("{"):
            # Controller text log line ("# ...") or blank — not an event. Tee it
            # to the serial log file (if configured) so the user can follow along.
            if line and self._serial_log:
                self._serial_log.info(line)
            return
        try:
            msg = json.loads(line)
        except json.JSONDecodeError as exc:
            log.warning("Bad JSON from Pico: %.80s — %s", line, exc)
            return
        t = msg.get("t", "")
        handler = getattr(self, f"_pico_{t}", None)
        if handler:
            handler(msg)
        else:
            log.debug("Unknown message type from Pico: %r", t)

    def _pico_hello(self, msg: dict) -> None:
        d = msg.get("d", {})
        dali_slaves = d.get("dali_slaves", [])
        # Accept the new "relay_slaves":[...] array, falling back to the legacy
        # scalar "relay_slave":N for older firmware builds.
        relay_slaves = d.get("relay_slaves")
        if relay_slaves is None:
            legacy = d.get("relay_slave")
            relay_slaves = [legacy] if legacy is not None else []
        self._fw_version = d.get("fw_version", "?")
        log.info(
            "Pico hello: relay_slaves=%s dali_slaves=%s fw=%s",
            relay_slaves, dali_slaves, self._fw_version,
        )

        # Clear session state so reconnect republishes fresh discovery
        self._disc_relays.clear()
        self._disc_ballasts.clear()
        self._disc_gateways.clear()
        self._disc_ballast_cfg.clear()
        self._disc_group.clear()
        self._disc_controller = False

        self._pub(self._cfg.availability_topic, "online", retained=True)

        # Switch plates — static, 24 × 7 = 168 entities, published immediately
        for fixture in range(MAX_FIXTURES):
            self._pub_switch_plate_discovery(fixture)

        # Per DALI slave: gateway provisioning entities + all 64 ballasts (offline)
        # scan_complete will bring responding ballasts online
        for slave in dali_slaves:
            self._pub_gateway_discovery(slave)
            self._pub_all_ballasts_for_slave(slave)
            self._pub_all_groups_for_slave(slave)   # all 16 groups, always present
            # Seed gateway reachable (optimistic). The controller pushes
            # state_gateway offline/online if it stops/starts responding.
            self._pub(self._gateway_avail_topic(slave), "online", retained=True)

        # Relay board(s): seed reachable + publish all channels gated on that
        # reachability, so they show unavailable when the board is absent. The
        # controller's relay poll flips the gateway topic the same as for DALI.
        self._relay_slaves = list(relay_slaves)
        for slave in relay_slaves:
            self._pub(self._gateway_avail_topic(slave), "online", retained=True)
        if relay_slaves:
            self._pub_all_relays_for_slave(relay_slaves[0])

        # Pull device config (→ state_cfg) and the full mapping/group set
        # (→ config_mapping / state_group) so HA repopulates after a reconnect.
        self._send({"t": "get_cfg",      "id": self._next_id()})
        self._send({"t": "get_mappings", "id": self._next_id()})

    def _pico_state_relay(self, msg: dict) -> None:
        ch = msg.get("d", {}).get("ch")
        on = msg.get("v", {}).get("on", False)
        if ch is None:
            return
        self._pub_relay_discovery(ch, self._relay_slaves[0] if self._relay_slaves else None)
        self._pub(
            self._t("relay", str(ch), "state"),
            {"state": "ON" if on else "OFF"},
            retained=True,
        )

    def _pico_state_dali(self, msg: dict) -> None:
        d     = msg.get("d", {})
        v     = msg.get("v", {})
        slave = d.get("slave")
        addr  = d.get("addr")
        if slave is None or addr is None:
            return
        # Discovery should already exist from hello, but publish defensively
        self._pub_ballast_discovery(slave, addr)
        st = {"state": "ON"} if v.get("on") else {"state": "OFF"}
        if v.get("on"):
            st["color_mode"] = "brightness"
            st["brightness"] = v.get("bri", 0)
        self._pub(self._t("dali", str(slave), str(addr), "state"), st, retained=True)
        # Per-ballast DALI config (min/max/power_on/sys_fail/fade_*) → number
        # entities. Only present ballasts emit state_dali, so config entities are
        # created lazily for the ones that actually exist.
        if any(k in v for k, *_ in DALI_CONFIG_FIELDS):
            self._pub_ballast_config_discovery(slave, addr)
            self._pub(
                self._t("dali", str(slave), str(addr), "config"),
                {k: v.get(k, 0) for k, *_ in DALI_CONFIG_FIELDS},
                retained=True,
            )

    def _pico_state_group(self, msg: dict) -> None:
        d     = msg.get("d", {})
        v     = msg.get("v", {})
        slave = d.get("slave")
        group = d.get("group")
        if slave is None or group is None:
            return
        self._pub_group_discovery(slave, group)
        gst = {"state": "ON"} if v.get("on") else {"state": "OFF"}
        if v.get("on"):
            gst["color_mode"] = "brightness"
            gst["brightness"] = v.get("bri", 0)
        self._pub(self._t("group", str(slave), str(group), "state"), gst, retained=True)
        self._pub(
            self._t("group", str(slave), str(group), "members", "state"),
            ",".join(str(a) for a in v.get("members", [])),
            retained=True,
        )

    def _pico_state_cfg(self, msg: dict) -> None:
        v = msg.get("v", {})
        if not v.get("fw_version"):
            v = {**v, "fw_version": self._fw_version}
        self._pub_controller_discovery(v)
        self._pub(self._t("config", "state"), v, retained=True)

    def _pico_state_gateway(self, msg: dict) -> None:
        slave = msg.get("d", {}).get("slave")
        if slave is None:
            return
        online = msg.get("v", {}).get("online", True)
        self._pub(self._gateway_avail_topic(slave),
                  "online" if online else "offline", retained=True)
        log.info("Gateway %s reachability: %s", slave,
                 "online" if online else "OFFLINE")

    def _pico_button(self, msg: dict) -> None:
        d          = msg.get("d", {})
        v          = msg.get("v", {})
        fixture    = d.get("fixture")
        button     = d.get("button")
        event_type = v.get("action")
        if fixture is None or button is None or event_type is None:
            return
        # Not retained — button events are momentary
        self._pub(
            self._t("button", str(fixture), str(button)),
            {"event_type": event_type},
        )

    def _pico_scan_complete(self, msg: dict) -> None:
        """
        Authoritative presence update for one DALI slave.

        Sets present addresses online, all others offline.
        This is the primary driver of per-ballast availability.
        """
        d       = msg.get("d", {})
        v       = msg.get("v", {})
        slave   = d.get("slave")
        present = set(v.get("present", []))
        if slave is None:
            return

        log.info(
            "Scan complete: slave=%d present=%s (%d/%d)",
            slave, sorted(present), len(present), DALI_ADDR_COUNT,
        )

        for addr in range(DALI_ADDR_COUNT):
            self._set_ballast_availability(slave, addr, addr in present)

    def _pico_presence(self, msg: dict) -> None:
        """
        Incremental presence update — appeared / disappeared since last scan.

        Updates only the changed addresses rather than the full 64.
        """
        d           = msg.get("d", {})
        v           = msg.get("v", {})
        slave       = d.get("slave")
        appeared    = v.get("appeared", [])
        disappeared = v.get("disappeared", [])
        if slave is None:
            return

        if appeared:
            log.info("Slave %d: appeared %s", slave, appeared)
            for addr in appeared:
                self._pub_ballast_discovery(slave, addr)   # may be new
                self._set_ballast_availability(slave, addr, True)

        if disappeared:
            log.warning("Slave %d: disappeared %s", slave, disappeared)
            for addr in disappeared:
                self._set_ballast_availability(slave, addr, False)

    def _pico_provision_status(self, msg: dict) -> None:
        d     = msg.get("d", {})
        v     = msg.get("v", {})
        slave = d.get("slave")
        if slave is None:
            return
        self._pub(
            self._t("provision", str(slave), "status"),
            {
                "state":         v.get("state", "unknown"),
                "msg":           v.get("msg", ""),
                "devices_found": v.get("devices_found", 0),
            },
            retained=True,
        )
        log.info(
            "Provision slave %d [%s] found=%d: %s",
            slave, v.get("state"), v.get("devices_found", 0), v.get("msg"),
        )

    def _pico_fault(self, msg: dict) -> None:
        d = msg.get("d", {})
        v = msg.get("v", {})
        log.warning(
            "Fault: slave=%s addr=%s lamp_fail=%s power_cycle=%s",
            d.get("slave"), d.get("addr"),
            v.get("lamp_fail"), v.get("power_cycle"),
        )

    def _pico_ack(self, msg: dict) -> None:
        d = msg.get("d", {})
        if not d.get("ok", True):
            log.warning("NAK id=%s err=%s", msg.get("id"), d.get("err"))

    def _pico_config_mapping(self, msg: dict) -> None:
        """Relay a button's binding to retained MQTT topics. Published by firmware
        after set_/del_mapping and on get_mappings/resync. Drives both the raw
        mapping topic and the per-button binding text entity (string form)."""
        d       = msg.get("d", {})
        v       = msg.get("v", {})
        fixture = d.get("fixture")
        button  = d.get("button")
        if fixture is None or button is None:
            return
        targets = v.get("targets", [])
        self._pub(
            self._t("mapping", str(fixture), str(button)),
            {"fixture": fixture, "button": button, "targets": targets},
            retained=True,
        )
        self._pub(
            self._t("binding", str(fixture), str(button), "state"),
            _binding_to_str(targets),
            retained=True,
        )

    # ── Inbound from MQTT → Pico ──────────────────────────────────────────

    def dispatch_from_mqtt(self, topic: str, payload: str) -> None:
        log.debug("MQTT → %s : %.80s", topic, payload)
        for pattern, method_name in self._RE:
            m = pattern.match(topic)
            if m:
                getattr(self, method_name)(m, payload)
                return
        log.debug("Unhandled MQTT topic: %s", topic)

    def _cmd_relay(self, m: re.Match, payload: str) -> None:
        channel = int(m.group(1))
        try:
            on = str(json.loads(payload).get("state", "OFF")).upper() == "ON"
        except (json.JSONDecodeError, AttributeError):
            on = payload.strip().upper() == "ON"
        self._send({
            "t": "set_relay", "id": self._next_id(),
            "d": {"ch": channel}, "v": {"on": on},
        })

    def _cmd_dali(self, m: re.Match, payload: str) -> None:
        slave = int(m.group(1))
        addr  = int(m.group(2))
        try:
            body = json.loads(payload)
        except json.JSONDecodeError:
            log.warning("Bad DALI payload: %s", payload)
            return
        out: dict[str, Any] = {
            "t": "set_dali", "id": self._next_id(),
            "d": {"slave": slave, "addr": addr},
            "v": {},
        }
        if "brightness" in body:
            # bri sets the level directly (0 = off); the controller derives on.
            out["v"]["bri"] = int(body["brightness"])
        elif "state" in body:
            out["v"]["on"] = str(body["state"]).upper() == "ON"
        else:
            log.warning("DALI command missing brightness and state")
            return
        self._send(out)

    def _cmd_prov_all(self, m: re.Match, payload: str) -> None:
        self._send({
            "t": "provision_start", "id": self._next_id(),
            "d": {"slave": int(m.group(1)), "mode": "all"},
        })

    def _cmd_prov_unaddressed(self, m: re.Match, payload: str) -> None:
        self._send({
            "t": "provision_start", "id": self._next_id(),
            "d": {"slave": int(m.group(1)), "mode": "unaddressed"},
        })

    def _cmd_prov_target(self, m: re.Match, payload: str) -> None:
        slave = int(m.group(1))
        try:
            addr = int(payload.strip())
            if 0 <= addr <= 63:
                self._prov_target[slave] = addr
                self._pub(
                    self._t("provision", str(slave), "target_state"),
                    str(addr), retained=True,
                )
            else:
                log.warning("Provision target out of range: %s", payload)
        except ValueError:
            log.warning("Provision target not a number: %s", payload)

    def _cmd_prov_address(self, m: re.Match, payload: str) -> None:
        slave  = int(m.group(1))
        target = self._prov_target.get(slave, 0)
        self._send({
            "t": "provision_start", "id": self._next_id(),
            "d": {"slave": slave, "mode": "address", "target": target},
        })

    def _cmd_prov_abort(self, m: re.Match, payload: str) -> None:
        self._send({"t": "provision_abort", "id": self._next_id()})

    # fix #10: mapping and config commands ──────────────────────────────────

    def _cmd_set_mapping(self, m: re.Match, payload: str) -> None:
        """lighting/mapping/<fixture>/<button>/set — payload is JSON targets array."""
        fixture = int(m.group(1))
        button  = int(m.group(2))
        try:
            targets = json.loads(payload)
            if not isinstance(targets, list):
                raise ValueError("targets must be a list")
        except (json.JSONDecodeError, ValueError) as exc:
            log.warning("Bad set_mapping payload: %s — %s", payload, exc)
            return
        self._send({
            "t": "set_mapping", "id": self._next_id(),
            "d": {"fixture": fixture, "button": button},
            "v": {"targets": targets},
        })

    def _cmd_del_mapping(self, m: re.Match, payload: str) -> None:
        """lighting/mapping/<fixture>/<button>/delete"""
        fixture = int(m.group(1))
        button  = int(m.group(2))
        self._send({
            "t": "del_mapping", "id": self._next_id(),
            "d": {"fixture": fixture, "button": button},
        })

    def _cmd_set_config(self, m: re.Match, payload: str) -> None:
        """lighting/config/set — payload: {"long_press_ms":N, "dim_repeat_ms":N, ...}"""
        try:
            body = json.loads(payload)
        except json.JSONDecodeError as exc:
            log.warning("Bad set_config payload: %s — %s", payload, exc)
            return
        v = {key: int(body[key])
             for key in ("long_press_ms", "dim_repeat_ms", "reversal_window_ms")
             if key in body}
        self._send({"t": "set_cfg", "id": self._next_id(), "v": v})

    def _cmd_cfg(self, m: re.Match, payload: str) -> None:
        """config/<field>/set — one controller config field from a number entity."""
        field = m.group(1)
        try:
            val = int(float(payload.strip()))
        except ValueError:
            log.warning("Bad config value for %s: %s", field, payload)
            return
        self._send({"t": "set_cfg", "id": self._next_id(), "v": {field: val}})

    def _cmd_dali_config(self, m: re.Match, payload: str) -> None:
        """dali/<slave>/<addr>/config/<field>/set — one DALI config field (number)."""
        slave, addr, field = int(m.group(1)), int(m.group(2)), m.group(3)
        try:
            val = int(float(payload.strip()))
        except ValueError:
            log.warning("Bad dali config value for %s: %s", field, payload)
            return
        self._send({
            "t": "set_dali", "id": self._next_id(),
            "d": {"slave": slave, "addr": addr}, "v": {field: val},
        })

    def _cmd_group(self, m: re.Match, payload: str) -> None:
        """group/<slave>/<group>/set — JSON-schema light command for a DALI group."""
        slave, group = int(m.group(1)), int(m.group(2))
        try:
            body = json.loads(payload)
        except json.JSONDecodeError:
            log.warning("Bad group payload: %s", payload)
            return
        out: dict[str, Any] = {
            "t": "set_group", "id": self._next_id(),
            "d": {"slave": slave, "group": group}, "v": {},
        }
        if "brightness" in body:
            out["v"]["bri"] = int(body["brightness"])
        elif "state" in body:
            out["v"]["on"] = str(body["state"]).upper() == "ON"
        else:
            return
        self._send(out)

    def _cmd_group_members(self, m: re.Match, payload: str) -> None:
        """group/<slave>/<group>/members/set — comma list of short addresses."""
        slave, group = int(m.group(1)), int(m.group(2))
        s = payload.strip()
        members = [int(x) for x in s.split(",") if x.strip().isdigit()] if s else []
        self._send({
            "t": "set_group", "id": self._next_id(),
            "d": {"slave": slave, "group": group}, "v": {"members": members},
        })

    def _cmd_binding(self, m: re.Match, payload: str) -> None:
        """binding/<fixture>/<button>/set — a button binding as a targets string."""
        fixture, button = int(m.group(1)), int(m.group(2))
        targets = _parse_binding_str(payload)
        if targets is None:
            log.warning("Bad binding string for F%d.B%d: %r", fixture, button, payload)
            return
        self._send({
            "t": "set_mapping", "id": self._next_id(),
            "d": {"fixture": fixture, "button": button}, "v": {"targets": targets},
        })

    # ── MQTT subscriptions ────────────────────────────────────────────────

    def _subscriptions(self) -> list[str]:
        p = self._cfg.topic_prefix
        return [
            f"{p}/relay/+/set",
            f"{p}/dali/+/+/set",
            f"{p}/dali/+/+/config/+/set",
            f"{p}/group/+/+/set",
            f"{p}/group/+/+/members/set",
            f"{p}/provision/+/start/all",
            f"{p}/provision/+/start/unaddressed",
            f"{p}/provision/+/start/target",
            f"{p}/provision/+/start/address",
            f"{p}/provision/+/abort",
            f"{p}/mapping/+/+/set",
            f"{p}/mapping/+/+/delete",
            f"{p}/binding/+/+/set",
            f"{p}/config/set",
            f"{p}/config/+/set",
        ]

    # ── Serial reader ──────────────────────────────────────────────────────

    def _serial_reader_loop(self) -> None:
        """
        Runs in a daemon thread forever.  Opens the serial port, reads lines,
        calls dispatch_from_pico().  On any serial error, closes the port,
        marks the system offline, and retries until the port reopens.
        """
        buf = b""
        while True:
            # Open / reopen
            while True:
                try:
                    port = serial.Serial(
                        self._cfg.serial_port,
                        self._cfg.serial_baud,
                        timeout=0.1,
                        exclusive=True,   # fail fast if another bridge holds the port
                    )
                    with self._serial_lock:
                        self._serial = port
                    log.info(
                        "Serial opened: %s @ %d baud",
                        self._cfg.serial_port, self._cfg.serial_baud,
                    )
                    # Pull a fresh full state (hello, all device state, groups,
                    # mappings) so HA repopulates even if the Pico didn't reboot.
                    self._send({"t": "resync", "id": self._next_id()})
                    buf = b""
                    break
                except serial.SerialException as exc:
                    log.warning(
                        "Cannot open %s: %s — retry in 3 s",
                        self._cfg.serial_port, exc,
                    )
                    time.sleep(3)

            # Read
            try:
                while True:
                    chunk = port.read(port.in_waiting or 1)
                    if not chunk:
                        continue
                    buf += chunk
                    while b"\n" in buf:
                        raw, buf = buf.split(b"\n", 1)
                        line = raw.strip()
                        if line:
                            self.dispatch_from_pico(
                                line.decode(errors="replace")
                            )
            except serial.SerialException as exc:
                log.error("Serial lost: %s — reconnecting...", exc)
                with self._serial_lock:
                    self._serial = None
                try:
                    port.close()
                except Exception:
                    pass
                # System offline — HA will show all entities as unavailable.
                # Per-ballast topics are already retained from last scan.
                self._pub(
                    self._cfg.availability_topic, "offline", retained=True
                )
                time.sleep(1)

    # ── Run ───────────────────────────────────────────────────────────────

    def run(self) -> None:
        cfg = self._cfg

        self._mqtt = mqtt.Client()
        if cfg.mqtt_user:
            self._mqtt.username_pw_set(cfg.mqtt_user, cfg.mqtt_password)
        self._mqtt.will_set(cfg.availability_topic, "offline", retain=True)
        self._mqtt.on_connect = lambda c, u, f, rc: (
            log.info("MQTT connected (rc=%d)", rc),
            [c.subscribe(t) for t in self._subscriptions()],
        )
        self._mqtt.on_disconnect = lambda c, u, rc: (
            log.warning("MQTT disconnected (rc=%d)", rc),
        )
        self._mqtt.on_message = (
            lambda c, u, m: self.dispatch_from_mqtt(
                m.topic,
                m.payload.decode(errors="replace").strip(),
            )
        )

        log.info("Connecting to MQTT %s:%d", cfg.mqtt_broker, cfg.mqtt_port)
        self._mqtt.connect(cfg.mqtt_broker, cfg.mqtt_port, keepalive=60)
        self._mqtt.loop_start()

        reader = threading.Thread(
            target=self._serial_reader_loop,
            daemon=True,
            name="serial-reader",
        )
        reader.start()

        log.info("Bridge running — Ctrl-C to stop.")
        try:
            reader.join()
        except KeyboardInterrupt:
            log.info("Shutting down.")
            self._pub(cfg.availability_topic, "offline", retained=True)
            self._mqtt.loop_stop()
            self._mqtt.disconnect()
            with self._serial_lock:
                if self._serial:
                    self._serial.close()


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    # Load a .env next to this script. CLI flags override env vars.
    load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))

    def env(key: str, default: str | None = None) -> str | None:
        return os.environ.get(key, default)

    parser = argparse.ArgumentParser(
        description="Switch gateway MQTT bridge. Config via flags or env / .env "
                    "(SERIAL_PORT, MQTT_BROKER, MQTT_PORT, MQTT_USER, MQTT_PASSWORD). "
                    "One lighting system per broker.",
    )
    parser.add_argument(
        "--serial", default=env("SERIAL_PORT", "/dev/ttyUSB0"),
        help="Serial port. Prefer /dev/serial/by-id/... symlink for stability.",
    )
    parser.add_argument("--baud",     type=int, default=int(env("MQTT_BAUD", "115200")))
    parser.add_argument("--broker",   default=env("MQTT_BROKER", "localhost"))
    parser.add_argument("--port",     type=int, default=int(env("MQTT_PORT", "1883")))
    parser.add_argument("--user",     default=env("MQTT_USER"))      # unset → anonymous
    parser.add_argument("--password", default=env("MQTT_PASSWORD"))
    parser.add_argument(
        "--serial-log", default=env("SERIAL_LOG", "controller.log"),
        help="File to tee the controller's '# ...' serial log lines to (rotating). "
             "Empty to disable.",
    )
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    cfg = BridgeConfig(
        serial_port=args.serial,
        serial_baud=args.baud,
        mqtt_broker=args.broker,
        mqtt_port=args.port,
        mqtt_user=args.user,
        mqtt_password=args.password,
        topic_prefix=NODE,
        availability_topic=f"{NODE}/availability",
        serial_log_path=args.serial_log or None,
    )

    log.info("Serial: %s @ %d baud", cfg.serial_port, cfg.serial_baud)
    log.info("MQTT:   %s:%d  user=%s", cfg.mqtt_broker, cfg.mqtt_port,
             cfg.mqtt_user or "(anonymous)")

    Bridge(cfg).run()


if __name__ == "__main__":
    main()
