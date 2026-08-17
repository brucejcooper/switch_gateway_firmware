#!/usr/bin/env python3
"""Full HA e2e: MQTT set -> DALI -> verify read -> MQTT state, timed.

Publishes JSON-schema light commands to lighting/dali/3/45/set and times the
retained state topic reporting the MEASURED brightness (the controller only
publishes what the verify read saw, so a matching state IS the round trip).

Exits 0 only if every round trip completed AND all three availability topics
read "online" in the baseline; nonzero otherwise, so this can gate CI/a release
instead of needing its output eyeballed.

Usage: ha_e2e.py [broker-ip]   (default: 192.168.1.113)
"""
import json, subprocess, sys, time, threading, queue

BROKER = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.113"
CMD_T   = "lighting/dali/3/45/set"
STATE_T = "lighting/dali/3/45/state"
AVAIL   = ["lighting/availability", "lighting/gateway/3/availability",
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

results = []

def round_trip(label, payload, matches, timeout=5.0):
    """Drain stale queue entries, publish `payload`, and time how long until a
    STATE_T message satisfies matches(dict). Records + prints (label, latency)
    -- latency is None on timeout -- in one place, so the two round-trip loops
    below can't drift from each other the way the old copy-pasted ones did."""
    try:
        while True: q.get_nowait()
    except queue.Empty:
        pass
    t0 = time.monotonic()
    pub(payload)
    got = None
    while time.monotonic() - t0 < timeout:
        try:
            t, line = q.get(timeout=0.1)
        except queue.Empty:
            continue
        top, _, pay = line.partition(" ")
        if top != STATE_T: continue
        try: d = json.loads(pay)
        except json.JSONDecodeError: continue
        if matches(d):
            got = t - t0
            break
    results.append((label, got))
    print(f"  {label}: {f'{got:.3f} s' if got is not None else f'TIMEOUT ({timeout:.0f} s)'}")
    return got

# 7-point round trip; values above the ballast's min clamp (~85)
targets = [100, 150, 200, 254, 120, 180, 90]
for tgt in targets:
    round_trip(f"bri={tgt}", json.dumps({"state": "ON", "brightness": tgt}),
               lambda d, tgt=tgt: d.get("state") == "ON" and d.get("brightness") == tgt)
    time.sleep(0.5)

# OFF then ON round trip
for st in ("OFF", "ON"):
    round_trip(f"state={st}", json.dumps({"state": st}),
               lambda d, st=st: d.get("state") == st)
    time.sleep(0.5)

avail_ok = all(seen.get(t) == "online" for t in AVAIL)
if not avail_ok:
    print("FAIL: not all availability topics read 'online' in the baseline")

ok = sum(1 for _, g in results if g is not None)
lat = [g for _, g in results if g is not None]
if lat:
    print(f"\nRESULT: {ok}/{len(results)} round trips; latency avg "
          f"{sum(lat)/len(lat):.3f} s, max {max(lat):.3f} s")
else:
    print("\nRESULT: all failed")

sys.exit(0 if (avail_ok and ok == len(results)) else 1)
