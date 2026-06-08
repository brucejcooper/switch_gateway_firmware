#!/usr/bin/env python3
"""Migrate `Switch wiring.xlsx` into Home Assistant via the lightswitch_mapper
integration (device renames + floorplan positions + fixture button-orders) and the
bridge's binding text entities.

Data model (confirmed with the owner):
  - Lights sheet: each light has Device (gateway Modbus slave), Address (DALI short
    addr), a Light name, Location, Map X/Y, and up to 3 (Fix Name, Btn) bindings.
    **Btn is the physical button POSITION on the plate (0-indexed).**
  - Switch Positions sheet: each fixture -> Patch Panel Location (= the physical
    fixture, HA `lighting_switch_plate_<LL>`), a "Button order" = the WIRING
    `order[position] = controller pin`, and Map X/Y. HA binds the controller pin, so
    a light at (Fix Name, Btn) binds pin = order[Btn].
  - Two fixtures may share a patch location (a physical plate split into two logical
    fixtures). That becomes one HA fixture with a split button_order "ordersA+ordersB"
    and a second ("ext") marker placed from the second fixture's Map X/Y.

What it writes:
  - DALI light device renamed to its Light name; placed at Map X/Y.
  - Fixture device renamed; button_order set (the wiring string); placed at Map X/Y
    (+ the ext marker at F#2). Split fixture name = "A + B".
  - Each fixture pin's binding text entity set to the targets that map to it.

Missing/short/empty button orders fall back to identity (position == pin) and are
FLAGGED. Run --dry-run first (default); --apply writes. Creds: --token <long-lived>
or --dev (dev rig testy/testy login). --url defaults to http://localhost:8123.
"""
import argparse
import json
import os
import re
import sys
import urllib.parse
import urllib.request
from collections import defaultdict

import openpyxl

XLSX = os.path.join(os.path.dirname(__file__), "..", "Switch wiring.xlsx")


# ---------------------------------------------------------------- spreadsheet ---
def _ints(cells):
    """Coerce a row slice to ints, tolerant of text-formatted cells (the first
    button-order cell is stored as text while the rest are numbers)."""
    out = []
    for c in cells:
        try:
            out.append(int(float(c)))
        except (ValueError, TypeError):
            pass
    return out


def _f(c):
    try:
        return float(c)
    except (ValueError, TypeError):
        return None


def load_plan(path):
    wb = openpyxl.load_workbook(path, data_only=True)

    # Switch Positions: name -> {loc, order(list of pins in physical order), x, y}
    fixtures = {}
    byloc = defaultdict(list)
    for r in list(wb["Switch Positions"].iter_rows(values_only=True))[1:]:
        if r[0] is None or r[1] is None:
            continue
        name = str(r[0]).strip()
        loc = int(r[1])
        order = _ints(r[2:9])
        fixtures[name] = {"loc": loc, "order": order, "x": _f(r[9]), "y": _f(r[10])}
        byloc[loc].append(name)

    # Lights: identity, name, position, and (fix, position) bindings
    lights = []
    used_pos = defaultdict(set)  # fixture name -> set of positions referenced
    for r in list(wb["Lights"].iter_rows(values_only=True))[1:]:
        if r[2] is None or r[3] is None:
            continue
        L = {
            "name": str(r[0]).strip() if r[0] else "",
            "location": str(r[1]).strip() if r[1] else "",
            "slave": int(r[2]),
            "addr": int(r[3]),
            "x": _f(r[11]),
            "y": _f(r[12]),
            "binds": [],
        }
        for fn, bt in ((r[4], r[5]), (r[6], r[7]), (r[8], r[9])):
            if fn and bt is not None:
                fn = str(fn).strip()
                L["binds"].append((fn, int(bt)))
                used_pos[fn].add(int(bt))
        lights.append(L)
    return fixtures, byloc, lights, used_pos


def effective_order(fx_name, fixtures, used_pos):
    """Pins in physical-position order; fall back to identity for missing positions.
    Returns (order_list, flags[])."""
    order = list(fixtures.get(fx_name, {}).get("order", []))
    flags = []
    need = max(used_pos.get(fx_name, {-1}), default=-1) + 1
    if not order:
        order = sorted(used_pos.get(fx_name, set())) or [0]
        flags.append("empty order → identity from used positions")
    elif need > len(order):
        for p in range(len(order), need):
            order.append(p)  # identity fallback for uncovered positions
        flags.append(f"order shorter than used positions → extended with identity to len {need}")
    if len(set(order)) != len(order):
        flags.append(f"pin collision in order {order}")
    return order, flags


def pin_for(fx_name, pos, fixtures, used_pos):
    order, _ = effective_order(fx_name, fixtures, used_pos)
    return order[pos] if pos < len(order) else pos


def build(fixtures, byloc, lights, used_pos):
    """Produce the write plan + anomaly list, independent of HA."""
    plan = {"lights": [], "fixtures": [], "bindings": defaultdict(set), "warn": []}

    for L in lights:
        plan["lights"].append(L)
        if L["x"] is None or L["y"] is None:
            plan["warn"].append(f"light {L['slave']}.{L['addr']} '{L['name']}' has no Map X/Y")

    for loc in sorted(byloc):
        names = byloc[loc]
        fxid = f"lighting_switch_plate_{loc:02d}"
        parts = []
        for n in names:
            order, flags = effective_order(n, fixtures, used_pos)
            for f in flags:
                plan["warn"].append(f"fixture '{n}' (loc {loc}): {f}")
            parts.append({"name": n, "order": order, "x": fixtures[n]["x"], "y": fixtures[n]["y"]})
        # collision across the split (two logical fixtures sharing pins on one plate)
        if len(parts) > 1:
            allpins = [p for pt in parts for p in pt["order"]]
            if len(set(allpins)) != len(allpins):
                plan["warn"].append(f"loc {loc}: split fixtures share pins {sorted(allpins)} — collision")
        plan["fixtures"].append({
            "id": fxid, "loc": loc, "parts": parts,
            "name": " + ".join(p["name"] for p in parts),
            "button_order": "+".join("".join(str(p) for p in pt["order"]) for pt in parts),
        })

    # bindings: (fixture id, pin) -> set of light targets
    for L in lights:
        for fn, pos in L["binds"]:
            fx = fixtures.get(fn)
            if not fx:
                plan["warn"].append(f"binding to unknown fixture '{fn}' (light {L['name']})")
                continue
            fxid = f"lighting_switch_plate_{fx['loc']:02d}"
            pin = pin_for(fn, pos, fixtures, used_pos)
            plan["bindings"][(fxid, pin)].add(f"{L['slave']}:{L['addr']}")
    return plan


def print_plan(plan):
    print(f"\n=== LIGHTS ({len(plan['lights'])}) — rename + place ===")
    for L in plan["lights"][:6]:
        print(f"  {L['slave']}.{L['addr']:<2} '{L['name']}'  @({L['x']},{L['y']})")
    print(f"  … ({max(0,len(plan['lights'])-6)} more)")

    print(f"\n=== FIXTURES ({len(plan['fixtures'])}) — rename + button_order + place ===")
    for F in plan["fixtures"]:
        split = "  [SPLIT]" if len(F["parts"]) > 1 else ""
        print(f"  {F['id']}  order={F['button_order']!r}  '{F['name']}'{split}")

    nb = sum(len(v) for v in plan["bindings"].values())
    print(f"\n=== BINDINGS ({len(plan['bindings'])} fixture-pins, {nb} targets) ===")
    for (fxid, pin), tgts in sorted(plan["bindings"].items())[:10]:
        print(f"  {fxid} pin {pin} → {sorted(tgts)}")
    print(f"  … ({max(0,len(plan['bindings'])-10)} more fixture-pins)")

    print(f"\n=== ANOMALIES ({len(plan['warn'])}) ===")
    for w in plan["warn"]:
        print(f"  ! {w}")


# ---------------------------------------------------------------------- HA -----
class HA:
    def __init__(self, url, token):
        self.url = url.rstrip("/")
        self.token = token
        self._inv = None

    def _post(self, path, body, form=False, auth=True):
        data = (urllib.parse.urlencode(body) if form else json.dumps(body)).encode()
        h = {"Content-Type": "application/x-www-form-urlencoded" if form else "application/json"}
        if auth:
            h["Authorization"] = "Bearer " + self.token
        with urllib.request.urlopen(urllib.request.Request(self.url + path, data, h)) as r:
            return json.loads(r.read() or "{}")

    @classmethod
    def dev_login(cls, url):
        cid = url.rstrip("/") + "/"
        post = lambda p, b, f=False: cls(url, "")._post(p, b, f, auth=False)
        fl = post("/auth/login_flow", {"client_id": cid, "handler": ["homeassistant", None], "redirect_uri": cid})
        r = post(f"/auth/login_flow/{fl['flow_id']}", {"username": "testy", "password": "testy", "client_id": cid})
        tok = post("/auth/token", {"grant_type": "authorization_code", "code": r["result"], "client_id": cid}, True)
        return cls(url, tok["access_token"])

    def ws_batch(self, msgs):
        """Run a list of (type, kwargs) over one websocket; return list of results."""
        import asyncio
        import websockets

        async def go():
            out = []
            async with websockets.connect(self.url.replace("http", "ws") + "/api/websocket", max_size=None) as w:
                await w.recv()
                await w.send(json.dumps({"type": "auth", "access_token": self.token}))
                await w.recv()
                for i, (t, kw) in enumerate(msgs, 1):
                    await w.send(json.dumps({"id": i, "type": t, **kw}))
                    out.append(json.loads(await w.recv()))
            return out
        return asyncio.run(go())

    def inventory(self):
        if self._inv is None:
            self._inv = self.ws_batch([("lightswitch_mapper/get_inventory", {})])[0]["result"]
        return self._inv


def apply_plan(ha, plan, do):
    inv = ha.inventory()
    by_target = {it["target"]: it for it in inv["lights"] if it.get("target")}
    fx_by_id = {it["id"]: it for it in inv["fixtures"]}
    msgs = []  # websocket ops
    svc = []   # text.set_value ops (REST)
    miss = []

    for L in plan["lights"]:
        it = by_target.get(f"{L['slave']}:{L['addr']}")
        if not it:
            miss.append(f"light {L['slave']}.{L['addr']} '{L['name']}' not in HA"); continue
        if it.get("device_id") and L["name"]:
            msgs.append(("config/device_registry/update", {"device_id": it["device_id"], "name_by_user": L["name"]}))
        if L["x"] is not None and L["y"] is not None:
            msgs.append(("lightswitch_mapper/set_position", {"uid": it["id"], "x": float(L["x"]), "y": float(L["y"])}))

    for F in plan["fixtures"]:
        it = fx_by_id.get(F["id"])
        if not it:
            miss.append(f"fixture {F['id']} '{F['name']}' not in HA"); continue
        if it.get("device_id"):
            msgs.append(("config/device_registry/update", {"device_id": it["device_id"], "name_by_user": F["name"]}))
        msgs.append(("lightswitch_mapper/set_button_order", {"fixture": F["id"], "order": F["button_order"]}))
        if F["parts"][0]["x"] is not None:
            msgs.append(("lightswitch_mapper/set_position",
                         {"uid": F["id"], "x": float(F["parts"][0]["x"]), "y": float(F["parts"][0]["y"])}))
        for k, pt in enumerate(F["parts"][1:], start=2):
            if pt["x"] is not None:
                msgs.append(("lightswitch_mapper/set_position",
                             {"uid": f"{F['id']}#{k}", "x": float(pt["x"]), "y": float(pt["y"])}))

    for (fxid, pin), tgts in plan["bindings"].items():
        it = fx_by_id.get(fxid)
        if not it:
            continue  # already flagged as missing fixture
        ent = next((b["entity_id"] for b in it.get("bindings", []) if b.get("button") == pin), None)
        if not ent:
            miss.append(f"{fxid} has no binding entity for pin {pin}"); continue
        svc.append((ent, ",".join(sorted(tgts))))

    print(f"\n=== APPLY: {len(msgs)} ws ops + {len(svc)} bindings; {len(miss)} not matched in HA ===")
    for m in miss[:20]:
        print(f"  ? {m}")
    if not do:
        print("(dry-run — pass --apply to write)")
        return
    ha.ws_batch(msgs)
    for ent, val in svc:
        ha._post("/api/services/text/set_value", {"entity_id": ent, "value": val})
    print("APPLIED.")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xlsx", default=XLSX)
    ap.add_argument("--url", default="http://localhost:8123")
    ap.add_argument("--token")
    ap.add_argument("--dev", action="store_true", help="use the dev rig testy/testy login")
    ap.add_argument("--apply", action="store_true", help="write to HA (default: dry-run)")
    ap.add_argument("--offline", action="store_true", help="just build+print the plan from xlsx, no HA")
    args = ap.parse_args()

    fixtures, byloc, lights, used_pos = load_plan(args.xlsx)
    plan = build(fixtures, byloc, lights, used_pos)
    print_plan(plan)
    if args.offline:
        return
    ha = HA.dev_login(args.url) if args.dev else HA(args.url, args.token)
    if not args.dev and not args.token:
        print("\n(no --token/--dev — stopping before HA match. add creds to see the apply plan)")
        return
    apply_plan(ha, plan, args.apply)


if __name__ == "__main__":
    main()
