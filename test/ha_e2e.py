#!/usr/bin/env python3
"""Full HA e2e: MQTT set -> DALI -> verify read -> MQTT state, timed.

Publishes JSON-schema light commands to lighting/dali/3/45/set and times the
retained state topic reporting the MEASURED brightness (the controller only
publishes what the verify read saw, so a matching state IS the round trip).
"""
import json, subprocess, sys, time, threading, queue

BROKER = "192.168.1.113"
CMD_T   = "lighting/dali/3/45/set"
STATE_T = "lighting/dali/3/45/state"
AVAIL   = ["lighting/bridge/status", "lighting/gateway/3/availability",
           "lighting/dali/3/45/availability"]

def sub_lines(topics, q):
    p = subprocess.Popen(["mosquitto_sub", "-h", BROKER, "-v"] +
                         sum((["-t", t] for t in topics), []),
                         stdout=subprocess.PIPE, text=True, bufsize=1)
    for line in p.stdout:
        q.put((time.monotonic(), line.strip()))

q = queue.Queue()
threading.Thread(target=sub_lines, args=([STATE_T] + AVAIL, q), daemon=True).start()
time.sleep(1.0)

# retained baseline: availability + current state
seen = {}
try:
    while True:
        t, line = q.get(timeout=1.0)
        top, _, pay = line.partition(" ")
        seen[top] = pay
except queue.Empty:
    pass
print("availability baseline:")
for t in AVAIL:
    print(f"  {t}: {seen.get(t, 'MISSING')}")
print(f"  state baseline: {seen.get(STATE_T, 'MISSING')}")

def pub(payload):
    subprocess.run(["mosquitto_pub", "-h", BROKER, "-t", CMD_T, "-m", payload], check=True)

# 7-point round trip; values above the ballast's min clamp (~85)
targets = [100, 150, 200, 254, 120, 180, 90]
results = []
for tgt in targets:
    # drain
    try:
        while True: q.get_nowait()
    except queue.Empty:
        pass
    t0 = time.monotonic()
    pub(json.dumps({"state": "ON", "brightness": tgt}))
    got = None
    while time.monotonic() - t0 < 5.0:
        try:
            t, line = q.get(timeout=0.1)
        except queue.Empty:
            continue
        top, _, pay = line.partition(" ")
        if top != STATE_T: continue
        try: d = json.loads(pay)
        except json.JSONDecodeError: continue
        if d.get("state") == "ON" and d.get("brightness") == tgt:
            got = t - t0
            break
    results.append((tgt, got))
    print(f"  bri={tgt}: {'%.3f s' % got if got is not None else 'TIMEOUT (5 s)'}")
    time.sleep(0.5)

# OFF then ON round trip
for st in ("OFF", "ON"):
    try:
        while True: q.get_nowait()
    except queue.Empty:
        pass
    t0 = time.monotonic()
    pub(json.dumps({"state": st}))
    got = None
    while time.monotonic() - t0 < 5.0:
        try:
            t, line = q.get(timeout=0.1)
        except queue.Empty:
            continue
        top, _, pay = line.partition(" ")
        if top != STATE_T: continue
        try: d = json.loads(pay)
        except json.JSONDecodeError: continue
        if d.get("state") == st:
            got = t - t0
            break
    print(f"  state={st}: {'%.3f s' % got if got is not None else 'TIMEOUT (5 s)'}")
    results.append((st, got))
    time.sleep(0.5)

ok = sum(1 for _, g in results if g is not None)
lat = [g for _, g in results if g is not None]
print(f"\nRESULT: {ok}/{len(results)} round trips; latency avg "
      f"{sum(lat)/len(lat):.3f} s, max {max(lat):.3f} s" if lat else "RESULT: all failed")
