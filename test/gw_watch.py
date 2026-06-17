#!/usr/bin/env python3
"""Watch a TinyModbus gateway for receive-path wedges via the forensic register.

The instrumented gateway firmware latches the state at the first moment its
receive path wedges (modbus_idle stuck false > ~250 ms) into FC04 register
0x0201, counts every occurrence, and self-recovers.  This tool polls that
register through the controller CLI and prints/decodes any wedge it sees, so we
can confirm what actually triggers the deafness — without needing a manual reset
(which would wipe the latch).

Leave it running while the system operates normally (the controller's background
polling of slaves 1/2/3 + the DALI scanner is what should provoke the wedge):

    .venv/bin/python test/gw_watch.py            # slave 2, poll every 1s

forensic 0x0201 layout:
    reg0 = wedge count
    reg1 = (buf[0] << 8) | buf[1]   leading slave addr + function code at wedge
    reg2 = (rx_sz   << 8) | flags   flags bit0 frame_ready, bit1 RXCIE-on
    reg3 = (silence << 8) | silence_threshold

Reading buf[0] is the smoking gun: == our slave id => our own frame stalled;
another slave's id (1/3) => foreign-frame buffer poisoning.
"""

import argparse
import glob
import re
import sys
import time

import serial


def main():
    ap = argparse.ArgumentParser(description="Watch a gateway for receive-path wedges")
    ap.add_argument("--port", default=None, help="controller USB-CDC port (auto-detect if omitted)")
    ap.add_argument("--slave", type=int, default=2)
    ap.add_argument("--interval", type=float, default=1.0, help="poll interval seconds")
    cfg = ap.parse_args()

    port = cfg.port or sorted(glob.glob("/dev/tty.usbmodem*") + glob.glob("/dev/ttyACM*"))[0]
    ser = serial.Serial(port, 115200, timeout=0.3)
    time.sleep(0.4)
    ser.reset_input_buffer()

    def cmd(c, wait=0.35):
        ser.write((c + "\n").encode())
        ser.flush()
        time.sleep(wait)
        return ser.read(3000).decode("utf-8", "replace")

    def read_regs(c):
        out = cmd(c)
        body = "\n".join(l for l in out.splitlines() if not l.startswith("#"))
        if "timeout" in body or "err" in body:
            return None
        return [int(x, 16) for x in re.findall(r"0x([0-9A-Fa-f]{4})", body)]

    print(f"Watching slave {cfg.slave} on {port} (Ctrl-C to stop)…")
    last_count = 0
    down_since = None
    while True:
        regs = read_regs(f"get_input {cfg.slave} 0x0201 8")
        now = time.strftime("%H:%M:%S")
        if regs is None or len(regs) < 4:
            if down_since is None:
                down_since = time.time()
            print(f"[{now}] gateway NOT responding ({time.time()-down_since:.0f}s) "
                  f"— wedged hard or not booted")
        else:
            if down_since is not None:
                print(f"[{now}] gateway back up after {time.time()-down_since:.0f}s")
                down_since = None
            count = regs[0]
            if count != last_count:
                b0, b1 = regs[1] >> 8, regs[1] & 0xFF
                rxsz, flags = regs[2] >> 8, regs[2] & 0xFF
                sil, sthr = regs[3] >> 8, regs[3] & 0xFF
                verdict = ("OUR frame stalled" if b0 == cfg.slave
                           else f"FOREIGN frame poisoning (addr {b0})" if b0 not in (0,)
                           else "broadcast")
                print(f"[{now}] *** WEDGE #{count} ***  leading buf[0]=0x{b0:02X} fc=0x{b1:02X} "
                      f"rx_sz={rxsz} frame_ready={flags&1} rxcie_on={(flags>>1)&1} "
                      f"silence={sil}/{sthr}  => {verdict}")
                last_count = count
            else:
                print(f"[{now}] ok  (wedges so far: {count})")
        time.sleep(cfg.interval)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nstopped")
    except IndexError:
        sys.exit("no controller serial port found")
