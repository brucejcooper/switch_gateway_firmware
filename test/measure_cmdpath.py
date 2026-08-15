#!/usr/bin/env python3
"""Ground-truth measurement of the controller command path (task 10).

Four windows, each measured from the CLI (`mbq`, `busload`, `scan`, `dali`,
`soak`) and reported as transactions per DALI command plus RS-485 busload:

  M1  idle steady state (background sweep + cfg rotation)   — 60 s
  M2  full interrogation scan (dali_state producer)         — until scan idle
  M3  user commands + verification reads                    — 20 commands
  M4  soak path=user depth=4 (kept-full backlog)            — 600 pairs

WHAT THE RATIO DIVIDES BY, and why it is not `enqueues`. A transaction is
either an enqueue-carrying request or a bare ack/poll, and v4 exists to drive
(enqueues + polls) / DALI-commands to 1.0. `enqueues` is NOT that denominator:
an enqueue refused with A_FULL costs a transaction and carries no command, so
counting it as one flatters any change that converts polls into re-offers —
which is exactly what raising a queue depth does. The honest denominator is the
number of commands that REACHED A TERMINAL OUTCOME in the window, i.e. the sum
of the client's result counters (complete + failed + lost + result_lost; a NAK
lands in `lost`, so absent-address probes are counted, as they must be — they
each cost a DALI frame). Both ratios are printed: `txns/cmd` is the real one,
`txns/enq` is the one the 2026-08-15 baseline log quotes, kept so the two runs
can be compared line for line. They agree exactly when A_FULL is 0.

Read `busload` IMMEDIATELY after a window closes: it reports one rolling window
and the next command starts overwriting it.
"""
import argparse
import re
import sys
import time

import serial


def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", default="/dev/cu.usbmodem112201",
                   help="controller USB-CDC CLI (default: %(default)s)")
    p.add_argument("--slave", type=int, default=3,
                   help="DALI gateway slave id (default: %(default)s)")
    p.add_argument("--addr", type=int, default=45,
                   help="dimmable ballast short address for M3/M4 (default: %(default)s)")
    p.add_argument("--idle-s", type=float, default=60.0,
                   help="M1 window length in seconds (default: %(default)s)")
    p.add_argument("--cmds", type=int, default=20,
                   help="M3 user commands (default: %(default)s)")
    p.add_argument("--pairs", type=int, default=600,
                   help="M4 soak pairs (default: %(default)s)")
    p.add_argument("-o", "--out", default="measure.log", help="log file")
    return p


class Rig:
    def __init__(self, port, slave, addr, log):
        self.s = serial.Serial(port, 115200, timeout=0.2)
        self.slave = slave
        self.addr = addr
        self.log = log

    def drain(self, t=0.3):
        out = b""
        end = time.time() + t
        while time.time() < end:
            b = self.s.read(4096)
            if b:
                out += b
                end = time.time() + 0.15
        return out.decode(errors="replace")

    def cmd(self, c, wait=0.4):
        self.s.write((c + "\n").encode())
        time.sleep(0.05)
        return self.drain(wait)

    def rec(self, tag, text):
        self.log.write(f"\n===== {tag} =====\n{text}\n")
        self.log.flush()
        print(f"--- {tag} ---")
        print(text)

    # ── counters ─────────────────────────────────────────────────────────────
    TXN_RE = re.compile(r"txns: (\d+) enqueue \+ (\d+) poll \((\d+) recovery\), "
                        r"(\d+) timeout,\s*(\d+) A_FULL, (\d+) resend")
    # Every terminal outcome, because `cmds` below is the denominator: a class
    # left off this line stops being counted as a DALI command and inflates
    # txns/cmd. `nak` split out of `lost` in task 11 for exactly that reason.
    RES_RE = re.compile(r"results: complete=(\d+) failed=(\d+) lost=(\d+) "
                        r"result_lost=(\d+) nak=(\d+)")

    def counters(self):
        """(enqueues, polls, a_full, commands) — commands = terminal outcomes."""
        out = self.cmd(f"mbq {self.slave}")
        t = self.TXN_RE.search(out)
        r = self.RES_RE.search(out)
        if not t or not r:
            print("PARSE FAIL mbq:", out)
            return None, out
        enq, poll, _rec, _to, afull, _rs = (int(x) for x in t.groups())
        cmds = sum(int(x) for x in r.groups())
        return (enq, poll, afull, cmds), out

    def busload(self):
        return self.cmd("busload", 0.6)

    def summarise(self, tag, before, after, extra=""):
        """Print the window's transaction accounting."""
        if not before or not after:
            return
        enq, poll, afull, cmds = (a - b for a, b in zip(after, before))
        txns = enq + poll
        self.rec(f"{tag} SUMMARY",
                 f"{extra}{enq} enqueue ({afull} A_FULL) + {poll} poll = {txns} txns, "
                 f"{cmds} DALI commands\n"
                 f"  txns/cmd {txns / max(cmds, 1):.2f}   "
                 f"txns/enq {txns / max(enq, 1):.2f}")


def bus_pct(text):
    m = re.search(r"bus busy (\d+)%", text)
    return m.group(1) + "%" if m else "?"


def main():
    args = build_parser().parse_args()
    log = open(args.out, "w")
    rig = Rig(args.port, args.slave, args.addr, log)

    rig.drain(1.0)
    rig.cmd("echo off")
    # `cfg` bare is a get, and reports the read-only fw_version — which is what
    # a run's log has to record about the firmware it measured.
    rig.rec("hello", rig.cmd("cfg") + rig.cmd("log warn"))

    verdicts = []

    # M1: idle steady state.
    c0, raw = rig.counters()
    rig.rec("M1 mbq before", raw)
    rig.cmd("busload reset")
    time.sleep(args.idle_s)
    bl = rig.busload()
    rig.rec(f"M1 busload ({args.idle_s:.0f}s idle)", bl)
    c1, raw = rig.counters()
    rig.rec("M1 mbq after", raw)
    rig.summarise("M1", c0, c1, f"idle {args.idle_s:.0f}s: ")
    verdicts.append(("M1 idle", c0, c1, bus_pct(bl), args.idle_s))

    # M2: a full interrogation scan, timed to `dstate` reporting scan=idle.
    c0, _ = rig.counters()
    rig.cmd("busload reset")
    rig.rec("M2 scan start", rig.cmd(f"scan {args.slave}"))
    t0 = time.time()
    while time.time() - t0 < 90:
        if "scan=idle" in rig.cmd(f"dstate {args.slave}", 0.3):
            break
        time.sleep(1.0)
    scan_s = time.time() - t0
    bl = rig.busload()
    rig.rec(f"M2 busload (scan, {scan_s:.1f}s)", bl)
    c1, raw = rig.counters()
    rig.rec("M2 mbq after", raw)
    rig.summarise("M2", c0, c1, f"scan {scan_s:.1f}s: ")
    verdicts.append(("M2 scan", c0, c1, bus_pct(bl), scan_s))

    # M3: user commands, each followed by its verification read.
    c0, _ = rig.counters()
    rig.cmd("busload reset")
    t0 = time.time()
    for i in range(args.cmds):
        rig.cmd(f"dali {args.slave} {args.addr} bri={200 if i % 2 == 0 else 50}", 0.1)
        time.sleep(0.6)      # let the verify reads land between commands
    time.sleep(2.0)          # drain trailing verify/fade reads
    m3_s = time.time() - t0
    bl = rig.busload()
    rig.rec(f"M3 busload ({args.cmds} user cmds, {m3_s:.1f}s)", bl)
    c1, raw = rig.counters()
    rig.rec("M3 mbq after", raw)
    rig.summarise("M3", c0, c1, f"{args.cmds} user cmds (incl. verify reads): ")
    verdicts.append(("M3 user+verify", c0, c1, bus_pct(bl), m3_s))

    # M4: the saturated path. The soak keeps its own accounting (it knows how
    # many DALI commands it issued), so its report is the record; the busload
    # read must follow it immediately.
    rig.rec("M4 soak start",
            rig.cmd(f"soak {args.slave} {args.addr} n={args.pairs} depth=4 path=user"))
    t0 = time.time()
    final = ""
    while time.time() - t0 < 180:
        time.sleep(5)
        out = rig.cmd("soak", 1.0)
        if "VERDICT" in out or "pairs/s" in out:
            final = out
            if "running" not in out.lower():
                break
    bl = rig.busload()
    rig.rec("M4 soak report", final)
    rig.rec("M4 busload (read immediately)", bl)

    rig.rec("final dq", rig.cmd(f"dq {args.slave}", 0.8))
    rig.rec("final mbq gw", rig.cmd(f"mbq gw {args.slave}", 0.8))

    rows = ["window          wall    cmds   enq  A_FULL   poll  txns/cmd  bus"]
    for tag, c0, c1, bus, wall in verdicts:
        if not c0 or not c1:
            continue
        enq, poll, afull, cmds = (a - b for a, b in zip(c1, c0))
        rows.append(f"{tag:<15} {wall:5.1f}s {cmds:6d} {enq:5d} {afull:7d} {poll:6d}"
                    f"  {(enq + poll) / max(cmds, 1):8.2f}  {bus:>4}")
    rig.rec("TABLE", "\n".join(rows))
    log.close()
    print("DONE")
    return 0


if __name__ == "__main__":
    sys.exit(main())
