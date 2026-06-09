import { h } from '../dom.js';
import { saver, badge, num } from '../util.js';

// Provision tab: per-gateway commissioning controls. State/count come from the
// poll; this view is re-rendered on poll so progress updates live.
const STATE_LABEL = { 0: 'idle', 1: 'running', 2: 'complete', 3: 'error' };
const STATE_KIND = { 0: 'muted', 1: 'warn', 2: 'ok', 3: 'bad' };

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
          badge(STATE_LABEL[st] || ('state ' + st), STATE_KIND[st] || 'muted'),
          p.count != null ? badge('count ' + p.count) : null,
        ),
      ),
      h('div', { class: 'row', style: { marginTop: '10px' } }, all, fresh, abort),
      h('div', { class: 'row', style: { marginTop: '8px' } },
        h('span', { class: 'muted' }, 'Address'), addrInput, reprov),
    );
  });
}
