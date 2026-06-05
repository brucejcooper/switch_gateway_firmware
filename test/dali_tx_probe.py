#!/usr/bin/env python3
"""Wire-level DALI transmit probe.

Answers one question with a logic-analyzer trace: when the TinyModbus gateway is
told to send a raw DALI command, does it actually drive the DALI TX line?

It captures the bench lines with the Saleae (see saleae_capture.py) while it
fires one raw DALI command, then reports per-line edge activity.
fires a single raw DALI query at the gateway, then reports the per-line edge
activity.  A healthy gateway shows edges on D0 (DALI TX) — and usually D1 (DALI
RX) if a ballast answers.  A dead transmit path shows D0 flat even though the
Modbus exchange (D4/D5) is clearly present.

Why the elaborate timing: the Saleae capture has a multi-second *arming latency*
after start — traffic that happens in that gap is lost.  So this probe runs
continuous background Modbus traffic for the whole window (which anchors and
proves the overlap) and fires the one DALI command in the middle, past arming.

Usage (from the controller dir, with the venv):
    .venv/bin/python test/dali_tx_probe.py            # slave 2, addr 0
    .venv/bin/python test/dali_tx_probe.py --slave 2 --addr 0 --opcode 0x91

Requires: Saleae Logic app running with the automation server enabled, the probe
connected (D0 DALI-TX, D1 DALI-RX, D4 RS485-RX, D5 RS485-TX, D6 DE), and the
controller on USB-CDC.
"""

import argparse
import glob
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit("/", 1)[0])  # allow `import saleae_capture`
import serial
from saleae_capture import SaleaeCapture, DALI_TX, DALI_RX, SER_RX, SER_TX, SER_DE


def short_addr_command_frame(short_addr, opcode):
    addr_byte = ((short_addr & 0x3F) << 1) | 0x01  # command form
    return (addr_byte << 8) | (opcode & 0xFF)


def main():
    ap = argparse.ArgumentParser(description="Probe whether the gateway drives DALI TX on a raw command")
    ap.add_argument("--port", default=None, help="controller USB-CDC port (auto-detect if omitted)")
    ap.add_argument("--slave", type=int, default=2)
    ap.add_argument("--addr", type=int, default=0, help="DALI short address to query")
    ap.add_argument("--opcode", type=lambda x: int(x, 0), default=0x91,
                    help="DALI opcode (default 0x91 QUERY CONTROL GEAR PRESENT)")
    ap.add_argument("--window", type=float, default=6.0, help="capture window seconds")
    ap.add_argument("--settle", type=float, default=2.5, help="delay after arm before firing (> arming latency)")
    ap.add_argument("--saleae-port", type=int, default=10430)
    cfg = ap.parse_args()

    port = cfg.port or sorted(glob.glob("/dev/tty.usbmodem*"))[0]
    ser = serial.Serial(port, 115200, timeout=0.1)
    lock = threading.Lock()

    def send(cmd):
        with lock:
            ser.write((cmd + "\n").encode())
            ser.flush()

    # Continuous, harmless Modbus traffic anchors the capture overlap (and proves
    # in the trace that the window covered the moment we fired the DALI command).
    stop = threading.Event()

    def anchor():
        while not stop.is_set():
            send(f"get_input {cfg.slave} 0x0104 1")
            time.sleep(0.05)

    threading.Thread(target=anchor, daemon=True).start()

    frame = short_addr_command_frame(cfg.addr, cfg.opcode)
    cap = SaleaeCapture(duration=cfg.window, port=cfg.saleae_port)
    try:
        cap.start()
    except Exception as e:
        stop.set(); ser.close()
        sys.exit(f"Saleae capture unavailable: {e}")

    print(f"Capture armed ({cfg.window:.0f}s @ {cap.sample_rate} S/s) with Modbus anchor traffic.")
    time.sleep(1.0)
    send(f"gw_reboot {cfg.slave}")                 # clear any stuck-PENDING raw channel
    time.sleep(cfg.settle)                          # reboot + settle, past arming latency
    print(f"Firing raw DALI 0x{frame:04X} (addr {cfg.addr}, opcode 0x{cfg.opcode:02X}) mid-window…")
    send(f"dali_raw {cfg.slave} 0x{frame:04X}")     # the single DALI-TX attempt to observe
    time.sleep(1.5)

    summary = cap.stop_and_export("dali_tx_probe")
    stop.set()
    ser.close()
    cap.close()

    lines = summary.get("lines", {})

    def info(idx):
        for nm, v in lines.items():
            digits = "".join(c for c in nm if c.isdigit())
            if digits and int(digits) == idx:
                return v
        return {"edges": 0, "active_span_s": 0.0}

    tx, rx = info(DALI_TX), info(DALI_RX)
    srx, stx, de = info(SER_RX), info(SER_TX), info(SER_DE)
    mb = summary.get("modbus", {})

    print("\n=== wire activity ===")
    print(f"  D{DALI_TX} DALI TX : {tx['edges']:5d} edges")
    print(f"  D{DALI_RX} DALI RX : {rx['edges']:5d} edges")
    print(f"  D{SER_RX} RS485 RX: {srx['edges']:5d} edges  ({len(mb.get('modbus_rx', []))} Modbus frames)")
    print(f"  D{SER_TX} RS485 TX: {stx['edges']:5d} edges  ({len(mb.get('modbus_tx', []))} Modbus frames)")
    print(f"  D{SER_DE} DE      : {de['edges']:5d} edges")

    overlap_ok = srx["edges"] > 4  # anchor traffic proves the window covered the test
    print("\n=== verdict ===")
    if not overlap_ok:
        print("  ⚠ INCONCLUSIVE: no RS-485 activity captured — the window did not overlap the bus")
        print("    (Saleae arming latency / probe wiring). Re-run; check the probe + automation server.")
    elif tx["edges"] > 2:
        print("  ✔ Gateway DRIVES DALI TX — forward frame present on D0.")
        print("    " + ("ballast response seen on D1." if rx["edges"] > 2 else "no D1 response (no/again ballast)."))
    else:
        print("  ✘ Gateway does NOT drive DALI TX (D0 flat) despite accepting the Modbus command.")
        print("    The DALI transmit path is dead — forward frame never goes out, so no ballast")
        print("    can answer, presence stays 0, and raw_cmd_result wedges PENDING (→ stuck busy).")
        print("    Look at the gateway's DALI TX firmware path / TX driver+pin.")
    if summary.get("sal"):
        print(f"\n  saved capture: {summary['sal']}  (open in Saleae Logic to inspect)")


if __name__ == "__main__":
    main()
