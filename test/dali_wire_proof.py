#!/usr/bin/env python3
"""Capture the DALI bus (D0 TX, D1 RX) over repeated off->on cycles and DECODE the
Manchester frames, so we can see exactly what byte the ballast returns to
QUERY ACTUAL LEVEL after GOTO_LAST_ACTIVE — hard proof, not inference.

Self-validating: the forward frames on D0 are known (GLA=03 0A, QSTATUS=03 90,
QACTUAL=03 A0, DAPC etc.). If D0 decodes to those, the decoder is trusted and the
D1 backward bytes are real.
"""
import sys, os, time, glob, csv, re, threading
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import serial
from saleae_capture import SaleaeCapture

TBIT = 833.333e-6
TE   = 416.667e-6

def chan_transitions(csv_path):
    """Return {ch_name: [(t, level), ...]} recording only changes; tolerant of
    sparse (blank) cells."""
    with open(csv_path, newline="") as fh:
        r = csv.reader(fh); header = next(r)
        cols = [c.strip() for c in header[1:]]
        last = [None]*len(cols); out = {c: [] for c in cols}
        for row in r:
            if not row: continue
            try: t = float(row[0])
            except ValueError: continue
            for i, cell in enumerate(row[1:len(cols)+1]):
                v = cell.strip()
                if v == "": continue
                lvl = 1 if v in ("1","1.0","True","true") else 0
                if last[i] is None or lvl != last[i]:
                    out[cols[i]].append((t, lvl)); last[i] = lvl
    return out

def level_at(trans, t):
    """Level just before time t (trans is sorted (time, level), level holds until next)."""
    lo, hi, lvl = 0, len(trans), 0
    while lo < hi:
        mid = (lo+hi)//2
        if trans[mid][0] <= t: lvl = trans[mid][1]; lo = mid+1
        else: hi = mid
    return lvl

def frames(trans, gap=1.1e-3):
    fr, cur = [], []
    for i, (t, _) in enumerate(trans):
        if cur and t - trans[i-1][0] > gap:
            fr.append(cur); cur = []
        cur.append((t, trans[i][1]))
    if cur: fr.append(cur)
    return fr

def decode_frame(trans, fr, n_data):
    """Decode n_data Manchester bits, MSB first. Calibrate the '1' level from the
    start bit's 2nd half so it's polarity-agnostic. Returns int or None."""
    t0 = fr[0][0]
    ref = level_at(trans, t0 + 1.5*TE)          # start bit = logical 1
    val = 0
    for k in range(n_data):
        s = level_at(trans, t0 + (k+1)*TBIT + 1.5*TE)
        bit = 1 if s == ref else 0
        val = (val << 1) | bit
    # frame must actually span ~ (1+n_data) bits of transitions
    span = fr[-1][0] - fr[0][0]
    if span < (n_data-1)*TBIT*0.6: return None
    return val

# ---- run capture while driving off->on cycles ----
port = sorted(glob.glob("/dev/tty.usbmodem*"))[0]
ser = serial.Serial(port, 115200, timeout=0.1); time.sleep(1.5); ser.reset_input_buffer()
def sc(c, w):
    ser.write((c+"\n").encode()); ser.flush(); time.sleep(w)
sc("log dbg", 0.6); sc("dali 2 1 fade_time=4", 2); sc("dali 2 1 bri=137", 3); sc("dali 2 1 on=false", 3)

cap = SaleaeCapture(channels=[0,1], duration=15, out_dir="/tmp/dali_proof")
cap.connect().start()
ctrl_log = []
def drive():
    time.sleep(3)
    for i in range(3):
        ser.reset_input_buffer()
        sc("dali 2 1 on=true", 3.5)
        ctrl_log.append(ser.read(40000).decode("utf-8","replace"))
        sc("dali 2 1 on=false", 0.2); time.sleep(0.4)
th = threading.Thread(target=drive); th.start()
summary = cap.stop_and_export("gla")   # waits for the 15s timed capture
th.join(); ser.close()

# controller-side: what arc did it read?
log = "\n".join(ctrl_log)
arcs = re.findall(r'ballast 2\.1 arc=(\d+) status=0x([0-9A-Fa-f]+)', log)
ff   = len(re.findall(r'ballast 2\.1 .*arc=0xFF', log))
print("controller valid arc reads (arc,status):", [(int(a),s) for a,s in arcs][:30])
print("controller arc=0xFF reads (guard-dropped):", ff)

dig_dir = summary.get("digital_dir")
csvs = glob.glob(os.path.join(dig_dir, "*.csv")) if dig_dir else []
if not csvs:
    print("NO digital CSV:", summary); sys.exit(1)
tr = chan_transitions(csvs[0])
names = list(tr.keys())
tx = tr[names[0]]; rx = tr[names[1]]   # D0 = forward (TX), D1 = backward (RX)
print(f"channels: {names}  (TX edges={len(tx)}, RX edges={len(rx)})")

OPC = {0x0A:"GOTO_LAST_ACTIVE",0x90:"QUERY_STATUS",0xA0:"QUERY_ACTUAL",0x05:"RECALL_MAX",0x00:"OFF"}
fwd = []
for fr in frames(tx):
    v = decode_frame(tx, fr, 16)
    if v is None: continue
    addr, opc = (v>>8)&0xFF, v&0xFF
    fwd.append((fr[0][0], addr, opc))
print(f"\n=== {len(fwd)} forward frames on D0 (validation: addr should be 0x03) ===")
for t,a,o in fwd[:24]:
    print(f"  t={t:7.4f}  {a:02X} {o:02X}  {OPC.get(o,'?')}")

bwd = []
for fr in frames(rx):
    v = decode_frame(rx, fr, 8)
    if v is not None: bwd.append((fr[0][0], v))
print(f"\n=== backward frames on D1 (ballast responses) ===")
for t,v in bwd[:30]:
    print(f"  t={t:7.4f}  byte=0x{v:02X} ({v})" + ("   <<< 0xFF / 255!" if v==0xFF else ""))

# correlate: each QUERY_ACTUAL forward -> the next backward frame
print(f"\n=== QUERY_ACTUAL (D0 03 A0) -> ballast's reply (D1) ===")
for t,a,o in fwd:
    if o != 0xA0: continue
    reply = next((bv for bt,bv in bwd if bt > t and bt < t+3e-3), None)
    print(f"  QUERY_ACTUAL @ t={t:7.4f} -> reply=" +
          (f"0x{reply:02X} ({reply})" if reply is not None else "none") +
          ("   *** 255 ON THE WIRE ***" if reply==0xFF else ""))
