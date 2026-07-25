import { h, mount } from '../dom.js';
import { onToggle, badge } from '../util.js';

// Relays tab. Live: each channel's toggle is patched in place on every poll so
// on/off tracks the device without rebuilding the DOM. If the channel count
// changes (rare — config), we ask for a full re-render (return false).
export function renderRelays(state) {
  const relays = state.relays || [];
  if (!relays.length) return h('div', { class: 'card sub' }, 'No relays known yet.');

  // Relay-board reachability (decode tags boards with kind === 'relay').
  const boardsOffline = (st) => {
    const b = (st.gateways || []).filter((g) => g.kind === 'relay');
    return b.length > 0 && b.every((g) => !g.online);
  };

  const rows = new Map();
  const sig = relays.length;

  const cells = relays.map((r) => { const row = relayRow(r); rows.set(r.ch, row); return row.el; });
  const head = h('div', { class: 'row between' });
  const renderHead = (off) => mount(head,
    h('h3', {}, 'Relays'),
    off ? badge('board offline', 'bad') : null,
  );
  renderHead(boardsOffline(state));

  const grid = h('div', {
    style: { display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(150px, 1fr))', gap: '8px' },
  }, cells);

  const nodes = h('div', { class: 'card' }, head, grid);

  const update = (st) => {
    const list = st.relays || [];
    if (list.length !== sig) return false; // channel count changed
    renderHead(boardsOffline(st));
    for (const r of list) { const row = rows.get(r.ch); if (row) row.update(r); }
    return true;
  };
  return { nodes, update };
}

function relayRow(r) {
  const toggle = onToggle(!!r.on, '/api/relay', { ch: r.ch });
  const el = h('div', { class: 'row between', style: { background: 'var(--bg)', padding: '8px 10px', borderRadius: '6px' } },
    h('strong', {}, 'CH ', String(r.ch)),
    toggle,
  );
  const update = (rr) => { toggle.setAttribute('aria-pressed', rr.on ? 'true' : 'false'); };
  return { el, update };
}
