#!/usr/bin/env python3
"""Inventory a DALI slave's ballasts and detect short-address collisions.

Drives the controller CLI `identify <slave> <addr>` (which reads GTIN + serial
from each ballast's memory bank 0 via raw DALI commands).  An address shared by
two or more gear answers the serial read with conflicting bytes, which the
gateway reports as a garbled/MALFORMED frame -> `identify` prints "collision".
The serial number is unique per device, so the collision is guaranteed
detectable even for two identical (same-GTIN) fixtures.

By default it scans the addresses the gateway reports present (fast); pass
--all to probe every short address 0..63.

    .venv/bin/python test/dali_inventory.py 2
    .venv/bin/python test/dali_inventory.py 2 --all
"""
import argparse
import glob
import re
import sys
import time

import serial


def resolve_port(preferred):
    """Return `preferred` if given, else the first available USB-CDC port.
    Exits with a clear message if none is found, rather than raising an
    IndexError on an empty glob that a caller might mistake for a parsing bug."""
    if preferred:
        return preferred
    cands = sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))
    if not cands:
        sys.exit("no controller serial port found (no /dev/tty.usbmodem* or "
                  "/dev/ttyACM*) — pass --port explicitly")
    return cands[0]


def main():
    ap = argparse.ArgumentParser(description="DALI ballast inventory + collision scan")
    ap.add_argument("slave", type=int, help="DALI gateway Modbus slave id")
    ap.add_argument("--port", default=None, help="controller USB-CDC port (auto if omitted)")
    ap.add_argument("--all", action="store_true", help="probe all 64 addresses, not just present")
    cfg = ap.parse_args()

    port = resolve_port(cfg.port)
    ser = serial.Serial(port, 115200, timeout=0.2)
    time.sleep(0.5)
    ser.reset_input_buffer()

    def drain(wait):
        """Read until `wait` s pass with no more bytes arriving (extending the
        deadline by 0.15 s each time bytes show up), rather than a flat sleep.
        Much faster on quick replies (log warn, show present) and stops a slow
        reply (identify, up to 8 s) from being truncated mid-frame into a bogus
        'absent'/'?' row when it lands right at the edge of a fixed sleep."""
        buf = b""
        deadline = time.time() + wait
        while time.time() < deadline:
            chunk = ser.read(4096)
            if chunk:
                buf += chunk
                deadline = time.time() + 0.15
        return buf

    def cmd(c, wait):
        ser.reset_input_buffer()
        ser.write((c + "\n").encode())
        ser.flush()
        return [l for l in drain(wait).decode("utf-8", "replace").splitlines()
                if l.strip() and not l.startswith("#") and not l.startswith("{")]

    cmd("log warn", 0.3)

    if cfg.all:
        addrs = list(range(64))
    else:
        out = " ".join(cmd(f"show present {cfg.slave}", 0.6))
        m = re.search(r"present \d+:\s*(.*)", out)
        addrs = [int(x) for x in m.group(1).split()] if (m and m.group(1).strip()) else []
        print(f"present addresses: {addrs or '(none)'}")

    print(f"\n{'addr':>4}  {'gtin':>12}  {'serial':>16}  status")
    print(f"{'----':>4}  {'------------':>12}  {'----------------':>16}  ------")
    collisions = []
    inventory = {}
    for a in addrs:
        line = (cmd(f"identify {cfg.slave} {a}", 8.0) or [""])[-1]
        m = re.search(r"gtin=([0-9A-Fa-f]+) serial=([0-9A-Fa-f]+)", line)
        if m:
            gtin, serial_no = m.group(1).upper(), m.group(2).upper()
            inventory[a] = (gtin, serial_no)
            print(f"{a:>4}  {gtin:>12}  {serial_no:>16}  ok")
        elif "collision" in line:
            collisions.append(a)
            print(f"{a:>4}  {'':>12}  {'':>16}  COLLISION (2+ devices)")
        elif "no_response" in line:
            if not cfg.all:
                print(f"{a:>4}  {'':>12}  {'':>16}  absent")
        else:
            print(f"{a:>4}  {'':>12}  {'':>16}  ? ({line})")

    print()
    print(f"{len(inventory)} device(s) inventoried.")
    if collisions:
        print(f"COLLISIONS on addresses: {collisions}")
        print("  -> re-provision these to give each colliding device a unique address")
    else:
        print("no collisions detected")
    ser.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt as e:
        sys.exit(str(e) or "interrupted")
