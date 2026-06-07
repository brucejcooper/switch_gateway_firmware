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
      [--port 1883] [--user U] [--password P] [--prefix lighting] [--debug]
"""

from __future__ import annotations

import argparse
import json
import logging
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


# ── Per-instance identity ───────────────────────────────────────────────────────
# NODE prefixes every HA unique_id + device identifier so multiple bridges (e.g.
# a test rig and the main system) can share one broker without colliding. LABEL
# optionally prefixes the human-readable device names. Both are set in main()
# from --prefix / --node-name. Default "lighting"/"" → the main system is
# byte-for-byte unchanged.
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
        "name":         f"{LABEL}Switch Plate {fixture + 1:02d}",
        "manufacturer": "Mechination",
        "model":        "Switch Gateway Button Matrix",
    }


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
        self._disc_relays:   set[int]            = set()
        self._disc_ballasts: set[tuple[int, int]] = set()
        self._disc_gateways: set[int]             = set()

        # Per-slave target address for "Commission Address" button
        self._prov_target: dict[int, int] = {}

        # Pre-compile MQTT command topic patterns from configured prefix
        p = re.escape(cfg.topic_prefix)
        self._RE: list[tuple[re.Pattern, str]] = [
            (re.compile(rf"^{p}/relay/(\d+)/set$"),                    "_cmd_relay"),
            (re.compile(rf"^{p}/dali/(\d+)/(\d+)/set$"),              "_cmd_dali"),
            (re.compile(rf"^{p}/provision/(\d+)/start/all$"),         "_cmd_prov_all"),
            (re.compile(rf"^{p}/provision/(\d+)/start/unaddressed$"), "_cmd_prov_unaddressed"),
            (re.compile(rf"^{p}/provision/(\d+)/start/target$"),      "_cmd_prov_target"),
            (re.compile(rf"^{p}/provision/(\d+)/start/address$"),     "_cmd_prov_address"),
            (re.compile(rf"^{p}/provision/(\d+)/abort$"),             "_cmd_prov_abort"),
            # fix #10: mapping and config commands
            (re.compile(rf"^{p}/mapping/(\d+)/(\d+)/set$"),           "_cmd_set_mapping"),
            (re.compile(rf"^{p}/mapping/(\d+)/(\d+)/delete$"),        "_cmd_del_mapping"),
            (re.compile(rf"^{p}/config/set$"),                         "_cmd_set_config"),
        ]

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

    def _set_ballast_availability(self, slave: int, addr: int,
                                   online: bool) -> None:
        self._pub(
            self._ballast_avail_topic(slave, addr),
            "online" if online else "offline",
            retained=True,
        )

    # ── Discovery ─────────────────────────────────────────────────────────

    def _pub_relay_discovery(self, channel: int) -> None:
        """Publish discovery for one relay channel as a switch (once per session)."""
        if channel in self._disc_relays:
            return
        self._disc_relays.add(channel)
        uid = f"{NODE}_relay_{channel}"
        self._pub(
            f"homeassistant/switch/{uid}/config",
            {
                "unique_id":            uid,
                # name=None → HA uses device name; user can rename in device registry
                "name":                 None,
                "state_topic":          self._t("relay", str(channel), "state"),
                "command_topic":        self._t("relay", str(channel), "set"),
                "state_value_template": "{{ value_json.state }}",
                "payload_on":           '{"state":"ON"}',
                "payload_off":          '{"state":"OFF"}',
                # Relay channels use only the system availability topic.
                # There is no per-channel presence concept on the Waveshare board.
                "availability_topic":   self._cfg.availability_topic,
                "device":               _dev_relay_channel(channel),
            },
            retained=True,
        )

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
                "brightness":                True,
                "brightness_scale":          254,
                "state_value_template":      "{{ value_json.on | lower }}",
                "brightness_value_template": "{{ value_json.bri }}",
                # Both topics must be "online" for entity to be available.
                "availability": [
                    {"topic": self._cfg.availability_topic},
                    {"topic": avail_t},
                ],
                "availability_mode":         "all",
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
                    "availability_topic": self._cfg.availability_topic,
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
                    "availability_topic": self._cfg.availability_topic,
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
                "availability_topic": self._cfg.availability_topic,
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

    # ── Inbound from Pico ─────────────────────────────────────────────────

    def dispatch_from_pico(self, line: str) -> None:
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
        log.info(
            "Pico hello: relay_slaves=%s dali_slaves=%s fw=%s",
            relay_slaves, dali_slaves, d.get("fw_version", "?"),
        )

        # Clear session state so reconnect publishes fresh discovery
        self._disc_relays.clear()
        self._disc_ballasts.clear()
        self._disc_gateways.clear()

        self._pub(self._cfg.availability_topic, "online", retained=True)

        # Switch plates — static, 24 × 7 = 168 entities, published immediately
        for fixture in range(MAX_FIXTURES):
            self._pub_switch_plate_discovery(fixture)

        # Per DALI slave: gateway provisioning entities + all 64 ballasts (offline)
        # scan_complete will bring responding ballasts online
        for slave in dali_slaves:
            self._pub_gateway_discovery(slave)
            self._pub_all_ballasts_for_slave(slave)

    def _pico_state_relay(self, msg: dict) -> None:
        ch = msg.get("d", {}).get("ch")
        on = msg.get("v", {}).get("on", False)
        if ch is None:
            return
        self._pub_relay_discovery(ch)
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
        self._pub(
            self._t("dali", str(slave), str(addr), "state"),
            {
                "on":        v.get("on", False),
                "bri":       v.get("bri", 0),
                "fading":    v.get("fading", False),
                "lamp_fail": v.get("lamp_fail", False),
            },
            retained=True,
        )

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
        """fix #9: Relay current button-to-light mapping to retained MQTT topic.
        Published by firmware after set_mapping / del_mapping, and on get_config."""
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

    # ── MQTT subscriptions ────────────────────────────────────────────────

    def _subscriptions(self) -> list[str]:
        p = self._cfg.topic_prefix
        return [
            f"{p}/relay/+/set",
            f"{p}/dali/+/+/set",
            f"{p}/provision/+/start/all",
            f"{p}/provision/+/start/unaddressed",
            f"{p}/provision/+/start/target",
            f"{p}/provision/+/start/address",
            f"{p}/provision/+/abort",
            # fix #10: mapping and config
            f"{p}/mapping/+/+/set",
            f"{p}/mapping/+/+/delete",
            f"{p}/config/set",
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
                    )
                    with self._serial_lock:
                        self._serial = port
                    log.info(
                        "Serial opened: %s @ %d baud",
                        self._cfg.serial_port, self._cfg.serial_baud,
                    )
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
    global NODE, LABEL

    # Load a .env next to this script (so secrets like MQTT_PASSWORD stay out of
    # the command line / process list). CLI flags still override env vars.
    load_dotenv(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".env"))

    def env(key: str, default: str | None = None) -> str | None:
        return os.environ.get(key, default)

    parser = argparse.ArgumentParser(
        description="Switch gateway MQTT bridge. Config via flags or env / .env "
                    "(SERIAL_PORT, MQTT_BROKER, MQTT_PORT, MQTT_USER, "
                    "MQTT_PASSWORD, MQTT_PREFIX, NODE_NAME).",
    )
    parser.add_argument(
        "--serial", default=env("SERIAL_PORT", "/dev/ttyUSB0"),
        help="Serial port. Prefer /dev/serial/by-id/... symlink for stability.",
    )
    parser.add_argument("--baud",     type=int, default=int(env("MQTT_BAUD", "115200")))
    parser.add_argument("--broker",   default=env("MQTT_BROKER", "mqtt.8bitcloud.com"))
    parser.add_argument("--port",     type=int, default=int(env("MQTT_PORT", "1883")))
    parser.add_argument("--user",     default=env("MQTT_USER", "lights"))
    parser.add_argument("--password", default=env("MQTT_PASSWORD"))
    parser.add_argument(
        "--prefix", default=env("MQTT_PREFIX", "lighting"),
        help="Topic prefix AND HA unique_id/device prefix — the instance id. Use a "
             "distinct value (e.g. 'testrig') to co-exist with the main system.",
    )
    parser.add_argument(
        "--node-name", default=env("NODE_NAME", ""),
        help="Human label prefixed to HA device names (e.g. 'Test Rig'), to tell "
             "instances apart in the HA UI.",
    )
    parser.add_argument("--debug", action="store_true")
    args = parser.parse_args()

    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)

    # Instance identity — must be set before any discovery is built.
    NODE  = args.prefix
    LABEL = f"{args.node_name} " if args.node_name else ""

    if not args.password:
        log.warning("No MQTT password set (MQTT_PASSWORD in .env or --password). "
                    "Continuing — broker may reject the connection.")

    cfg = BridgeConfig(
        serial_port=args.serial,
        serial_baud=args.baud,
        mqtt_broker=args.broker,
        mqtt_port=args.port,
        mqtt_user=args.user,
        mqtt_password=args.password,
        topic_prefix=args.prefix,
        availability_topic=f"{args.prefix}/availability",
    )

    log.info("Serial: %s @ %d baud", cfg.serial_port, cfg.serial_baud)
    log.info("MQTT:   %s:%d  user=%s  prefix=%s  label=%r",
             cfg.mqtt_broker, cfg.mqtt_port, cfg.mqtt_user, cfg.topic_prefix, LABEL)

    Bridge(cfg).run()


if __name__ == "__main__":
    main()
