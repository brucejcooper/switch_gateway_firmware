import { h } from '../dom.js';
import { saver, badge, num } from '../util.js';

// Provision tab: per-gateway commissioning controls. State/count come from the
// poll; this view is re-rendered on poll so progress updates live.
// The label comes from the DEVICE (state_text), not a table here. The table
// that used to live here was { 0:idle, 1:running, 2:complete, 3:error } — the
// GATEWAY's old 4-value commission status word. Provisioning moved to the
// controller and its prov_run_state_t has 12 states, so those numbers stopped
// meaning what this claimed: a run at state 2 is "scanning" and was shown as a
// green "complete", state 3 is "terminating" and was shown as a red "error",
// and state 11 (the real error) fell off the end as a bare "state 11".
//
// Only the colour is decided here, from the text, so a new state added on the
// device shows up correctly without touching this file.
const kindFor = (t) => (t === 'error' ? 'bad'
                      : t === 'complete' ? 'ok'
                      : t === 'idle' || t === 'unknown' ? 'muted'
                      : 'warn');

export function renderProvision(state) {
  const gateways = state.gateways || [];
  const prov = state.provision || [];
  if (!gateways.length) return h('div', { class: 'card sub' }, 'No DALI gateways.');

  return gateways.map((g) => {
    const p = prov.find((x) => x.slave === g.slave) || {};
    const st = num(p.state, 0);

    const addrInput = h('input', { type: 'number', min: '0', max: '63', value: '0', style: { width: '70px' } });

    const all = h('button', { class: 'btn small primary' }, 'Commission all');
    all.addEventListener('click', saver('/api/provision', () => ({ slave: g.slave, mode: 'all' }), { okMsg: 'Started' }));

    const fresh = h('button', { class: 'btn small' }, 'Commission new');
    fresh.addEventListener('click', saver('/api/provision', () => ({ slave: g.slave, mode: 'new' }), { okMsg: 'Started' }));

    const reprov = h('button', { class: 'btn small' }, 'Re-provision');
    reprov.addEventListener('click', saver('/api/provision',
      () => ({ slave: g.slave, mode: 'address', target: num(addrInput.value) }), { okMsg: 'Started' }));

    const abort = h('button', { class: 'btn small' }, 'Abort');
    abort.addEventListener('click', saver('/api/provision', () => ({ slave: g.slave, abort: true }), { okMsg: 'Aborting' }));

    return h('div', { class: 'card' },
      h('div', { class: 'row between' },
        h('strong', {}, 'Gateway slave ', String(g.slave)),
        h('div', { class: 'row' },
          badge(p.state_text || ('state ' + st), kindFor(p.state_text)),
          p.count != null ? badge('count ' + p.count) : null,
        ),
      ),
      // The device's own explanation — "Error: no ballasts found" says what a
      // red badge alone cannot.
      p.msg ? h('div', { class: 'sub', style: { marginTop: '6px' } }, p.msg) : null,
      h('div', { class: 'row', style: { marginTop: '10px' } }, all, fresh, abort),
      h('div', { class: 'row', style: { marginTop: '8px' } },
        h('span', { class: 'muted' }, 'Address'), addrInput, reprov),
    );
  });
}
