// decode.js — the single wire-format boundary.
//
// /api/state is organised the way the domain is: relays as a boolean array
// indexed by channel, and DALI organised per gateway slave. This is the ONLY
// place that knows that layout; it flattens the wire into the one named model
// every tab consumes (flat ballast/group/gateway lists). Keep it in lock-step
// with the renderer in src/net/web_state.c.
//
// Wire shape:
//   relays: [bool|null, ...]                    index = channel; null = unread
//   dali: { "<slave>": {
//             online: bool,
//             ballasts: [ {on,bri,min,max,power_on,sys_fail,fade_rate,fade_time,fading,lamp_fail} | null, ... ],  // index = address
//             groups:   [ {on,bri,members:[addr,...]} | null, ... ]  index = group
//                       null = no members yet; decoded to an entry with
//                       empty:true rather than dropped, so all 16 are listed
//          }, ... }
//   mappings: [ {fixture,button,targets:[{slave,addr}|{slave,group}]}, ... ]
//   provision: [ {slave,state,state_text,count,msg}, ... ]
//   cfg: {...}   seq: N

export function decodeState(s) {
  s = s || {};
  const ballasts = [];
  const groups = [];
  // Gateways/boards available on the bus — relay boards first, then DALI
  // gateways — each tagged with its kind so the dashboard can label them.
  const gateways = (s.relay_boards || []).map((b) => ({
    slave: b.slave, online: !!b.online, kind: 'relay',
  }));

  const dali = s.dali || {};
  for (const key of Object.keys(dali)) {
    const slave = Number(key);
    const sd = dali[key] || {};
    gateways.push({ slave, online: !!sd.online, kind: 'dali' });
    (sd.ballasts || []).forEach((b, addr) => {
      if (!b) return; // null = unavailable
      ballasts.push({
        slave, addr, present: true,
        on: !!b.on, bri: b.bri,
        min: b.min, max: b.max, power_on: b.power_on, sys_fail: b.sys_fail,
        fade_rate: b.fade_rate, fade_time: b.fade_time,
        fading: !!b.fading, lamp_fail: !!b.lamp_fail,
      });
    });
    /* Keep EVERY group slot, including the ones the device has never emitted.
     * A null here means "no members yet", not "does not exist" — DALI has 16
     * groups per bus, always, and there is no create step: you make a group by
     * adding members to it. Dropping the nulls meant the UI only listed groups
     * that already had members, so there was no way to build a new one. */
    (sd.groups || []).forEach((g, group) => {
      groups.push({
        slave, group,
        on: !!(g && g.on),
        bri: g ? g.bri : 0,
        members: (g && g.members) || [],
        empty: !g || !(g.members || []).length,
      });
    });
  }

  return {
    // on === null means the channel has never been read (see web_state.c
    // SEC_RELAY). Keep that distinct from false rather than collapsing it with
    // !!on, so the UI can show "unread" instead of a confident "off".
    relays: (s.relays || []).map((on, ch) => ({ ch, on: on === null ? null : !!on,
                                                known: on !== null })),
    ballasts,
    groups,
    gateways,
    mappings: s.mappings || [],
    provision: s.provision || [],
    cfg: s.cfg || {},
    seq: s.seq,
  };
}
