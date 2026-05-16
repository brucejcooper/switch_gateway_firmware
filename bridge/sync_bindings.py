#!/usr/bin/env python3
"""sync_bindings.py — wipe and replace button bindings from a spreadsheet.

Reads "Switch wiring.xlsx" (Switch Positions + Lights tabs) and replaces
the firmware's entire binding set in one transaction via the
replace_mappings_begin / _add / _commit JSON commands over USB CDC.

Dry-run by default. Pass --apply to actually send commands.

Dependencies: openpyxl, pyserial.

The MQTT bridge must be stopped while running this tool — it would consume
the same acks the tool is waiting for.
"""

import argparse
import json
import sys
import time
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import openpyxl
import serial


# ── Firmware constants (must match src/mapping.h) ────────────────────────────
MAX_FIXTURES        = 24
MAX_BUTTONS         = 8       # 7 physical + 1 spare
MAX_TARGETS_PER_BTN = 8

@dataclass
class Target:
    slave: int
    addr: int

    def to_json(self) -> dict:
        return {"slave": self.slave, "addr": self.addr}


# ── Spreadsheet parsing ──────────────────────────────────────────────────────

def load_remap(wb) -> tuple[dict, list[str]]:
    """Read Switch Positions tab into a remap dict.

    Returns ({(fixture_id, fix_name): [phys_at_logical_0, ...]}, errors).
    Rows whose `Patch Panel Location` (col B) is not an integer are skipped
    silently — they're commentary rows.
    """
    ws = wb["Switch Positions"]
    remap: dict[tuple[int, str], list[int]] = {}
    errors: list[str] = []

    for row_idx, row in enumerate(ws.iter_rows(values_only=True), start=1):
        if row_idx == 1:
            continue  # header
        name, fix_id, *order_cells = row[:7]
        if not isinstance(fix_id, int):
            continue  # commentary / blank
        if not (0 <= fix_id < MAX_FIXTURES):
            errors.append(f"Switch Positions row {row_idx}: fixture {fix_id} "
                          f"out of range 0..{MAX_FIXTURES - 1}")
            continue

        order: list[int] = []
        for col_idx, cell in enumerate(order_cells):
            if cell is None or cell == "":
                continue
            try:
                phys = int(cell)
            except (TypeError, ValueError):
                errors.append(f"Switch Positions row {row_idx} col {col_idx + 3}: "
                              f"non-integer button {cell!r}")
                continue
            if not (0 <= phys < MAX_BUTTONS):
                errors.append(f"Switch Positions row {row_idx} col {col_idx + 3}: "
                              f"physical button {phys} out of range")
                continue
            order.append(phys)

        key = (fix_id, name)
        if key in remap:
            errors.append(f"Switch Positions row {row_idx}: duplicate "
                          f"(fixture {fix_id}, name {name!r})")
            continue
        remap[key] = order

    # Disjoint check for shared fixture IDs.
    by_fix: dict[int, dict[str, list[int]]] = defaultdict(dict)
    for (fix_id, name), order in remap.items():
        by_fix[fix_id][name] = order
    for fix_id, plates in by_fix.items():
        if len(plates) <= 1:
            continue
        seen: dict[int, str] = {}
        for name, order in plates.items():
            for phys in order:
                if phys in seen and seen[phys] != name:
                    errors.append(f"Fixture {fix_id}: physical button {phys} "
                                  f"claimed by both {seen[phys]!r} and {name!r}")
                seen[phys] = name

    return remap, errors


def load_bindings(wb, remap: dict) -> tuple[dict, list[str]]:
    """Read Lights tab and invert into per-(fixture, physical_button) bindings.

    Returns ({(fixture_id, phys_button): [Target, ...]}, errors).

    The fixture ID for each binding is looked up by name in `remap` — the
    spreadsheet only stores the name and logical button per binding. Plate
    names are unique even when two plates share a fixture ID. """
    ws = wb["Lights"]
    bindings: dict[tuple[int, int], list[Target]] = defaultdict(list)
    errors: list[str] = []

    # name → fixture_id from Switch Positions. Plate names are globally
    # unique, so name disambiguates between plates that share a fixture ID.
    name_to_fix_id: dict[str, int] = {}
    name_collisions: list[str] = []
    for (fix_id, name) in remap:
        if name in name_to_fix_id and name_to_fix_id[name] != fix_id:
            name_collisions.append(
                f"Switch Positions: name {name!r} maps to both fixture "
                f"{name_to_fix_id[name]} and {fix_id}")
        else:
            name_to_fix_id[name] = fix_id
    errors.extend(name_collisions)

    for row_idx, row in enumerate(ws.iter_rows(values_only=True), start=1):
        if row_idx == 1:
            continue  # header
        # Columns: A Light, B Location, C Device, D Address,
        #          E Fix Name, F Btn1, G Fix Name 2, H Btn2,
        #          I Fix name 3, J Btn3, K Note
        if len(row) < 10:
            continue
        light_name = row[0]
        device     = row[2]
        address    = row[3]

        # Skip rows where the light isn't fully wired up — placeholders with
        # missing Device or Address can't drive a binding either way, so
        # silently ignore rather than blocking the sync.
        if device is None or address is None:
            continue

        if not isinstance(device, int) or not (1 <= device <= 247):
            errors.append(f"Lights row {row_idx} ({light_name!r}): "
                          f"invalid Device {device!r}")
            continue
        slave = device
        if not isinstance(address, int) or not (0 <= address < 64):
            errors.append(f"Lights row {row_idx} ({light_name!r}): "
                          f"invalid Address {address!r} (must be 0..63)")
            continue

        for triplet_idx, base in enumerate((4, 6, 8)):
            fix_name = row[base]
            log_btn  = row[base + 1]
            if fix_name is None and log_btn is None:
                continue
            # Partial pair (name without button or vice versa) means the row
            # is mid-edit — silently skip. A fully-set but invalid pair
            # (e.g. unknown fix_name) still errors below.
            if fix_name is None or not isinstance(log_btn, int):
                continue
            fix_id = name_to_fix_id.get(fix_name)
            if fix_id is None:
                errors.append(f"Lights row {row_idx} ({light_name!r}) "
                              f"binding {triplet_idx + 1}: no Switch Positions "
                              f"entry for plate {fix_name!r}")
                continue
            order = remap[(fix_id, fix_name)]
            if not (0 <= log_btn < len(order)):
                errors.append(f"Lights row {row_idx} ({light_name!r}) "
                              f"binding {triplet_idx + 1}: logical button "
                              f"{log_btn} out of range 0..{len(order) - 1} "
                              f"for {fix_name!r}")
                continue
            phys_btn = order[log_btn]
            tgt = Target(slave=slave, addr=address)
            existing = bindings[(fix_id, phys_btn)]
            if any(t.slave == tgt.slave and t.addr == tgt.addr for t in existing):
                continue  # duplicate target, silently dedup
            existing.append(tgt)

    for (fix_id, btn), targets in bindings.items():
        if len(targets) > MAX_TARGETS_PER_BTN:
            errors.append(f"Fixture {fix_id} button {btn}: {len(targets)} "
                          f"targets exceeds limit {MAX_TARGETS_PER_BTN}")

    return dict(bindings), errors


# ── Serial protocol ──────────────────────────────────────────────────────────

class FirmwareClient:
    def __init__(self, port: serial.Serial, verbose: bool = False):
        self.port = port
        self.verbose = verbose
        self._next_id = 1

    def _ack_id(self) -> int:
        i = self._next_id
        self._next_id += 1
        return i

    def _send(self, msg: dict, timeout: float = 3.0) -> dict:
        cmd_id = self._ack_id()
        msg["id"] = cmd_id
        line = json.dumps(msg, separators=(",", ":")) + "\n"
        if self.verbose:
            print(f"→ {line.rstrip()}")
        self.port.write(line.encode())
        self.port.flush()

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.port.readline()
            if not raw:
                continue
            text = raw.decode(errors="replace").strip()
            if not text.startswith("{"):
                if self.verbose:
                    print(f"  (skip) {text}")
                continue
            try:
                evt = json.loads(text)
            except json.JSONDecodeError:
                continue
            if evt.get("t") == "ack" and evt.get("id") == cmd_id:
                if self.verbose:
                    print(f"← {text}")
                return evt
            # not our ack — could be a config_mapping echo, ignore
        raise TimeoutError(f"no ack for id={cmd_id}: {msg!r}")

    def begin(self) -> None:
        ack = self._send({"t": "replace_mappings_begin"})
        if not ack.get("d", {}).get("ok"):
            raise RuntimeError(f"begin rejected: {ack}")

    def add(self, fixture: int, button: int, targets: list[Target]) -> None:
        msg = {
            "t": "replace_mapping_add",
            "d": {"fixture": fixture, "button": button},
            "v": {"targets": [t.to_json() for t in targets]},
        }
        ack = self._send(msg)
        if not ack.get("d", {}).get("ok"):
            err = ack.get("d", {}).get("err", "?")
            raise RuntimeError(f"add F{fixture}.B{button} rejected: {err}")

    def commit(self) -> None:
        ack = self._send({"t": "replace_mappings_commit"})
        if not ack.get("d", {}).get("ok"):
            err = ack.get("d", {}).get("err", "?")
            raise RuntimeError(f"commit rejected: {err}")

    def abort(self) -> None:
        try:
            self._send({"t": "replace_mappings_abort"})
        except TimeoutError:
            pass


# ── Driver ───────────────────────────────────────────────────────────────────

def print_plan(remap: dict, bindings: dict) -> None:
    print(f"Plates:   {len({k[0] for k in remap})} fixture IDs, "
          f"{len(remap)} switch plates")
    print(f"Bindings: {len(bindings)} (fixture, button) entries, "
          f"{sum(len(v) for v in bindings.values())} targets total")
    print()
    for (fix_id, btn) in sorted(bindings):
        names = sorted({n for (f, n) in remap if f == fix_id})
        label = " / ".join(names) or "?"
        tgts = ", ".join(f"{t.slave}:{t.addr}" for t in bindings[(fix_id, btn)])
        print(f"  F{fix_id:02d}.B{btn} [{label}] → {tgts}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--workbook", required=True, type=Path,
                    help='Path to "Switch wiring.xlsx"')
    ap.add_argument("--serial", type=Path,
                    help="USB CDC device (required with --apply)")
    ap.add_argument("--apply", action="store_true",
                    help="Actually send replace_mappings to the firmware. "
                         "Default is dry-run.")
    ap.add_argument("--verbose", "-v", action="store_true")
    args = ap.parse_args()

    if not args.workbook.exists():
        print(f"workbook not found: {args.workbook}", file=sys.stderr)
        return 2

    wb = openpyxl.load_workbook(args.workbook, data_only=True, read_only=True)
    remap, errs1 = load_remap(wb)
    bindings, errs2 = load_bindings(wb, remap)

    all_errs = errs1 + errs2
    if all_errs:
        print("Validation errors:", file=sys.stderr)
        for e in all_errs:
            print(f"  ! {e}", file=sys.stderr)
        return 1

    print_plan(remap, bindings)

    if not args.apply:
        print("\n(dry-run — pass --apply to send)")
        return 0

    if not args.serial:
        print("--apply requires --serial", file=sys.stderr)
        return 2

    print(f"\nApplying to {args.serial} ...")
    with serial.Serial(str(args.serial), 115200, timeout=0.2) as port:
        # Drain any boot-time chatter so our ack matcher doesn't see stale lines.
        port.reset_input_buffer()
        client = FirmwareClient(port, verbose=args.verbose)
        try:
            client.begin()
            for (fix_id, btn), targets in sorted(bindings.items()):
                client.add(fix_id, btn, targets)
            client.commit()
        except (RuntimeError, TimeoutError) as e:
            print(f"FAILED: {e}", file=sys.stderr)
            client.abort()
            return 1

    print("Done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
