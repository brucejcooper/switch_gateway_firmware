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
  // Re-render signature: length AND which channels are known — a channel going
  // from unread to known swaps a badge for a toggle, which patching cannot do.
  const sig = relays.length + ':' + relays.map((r) => (r.known ? '1' : '0')).join('');

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
    // Recompute the same signature: a full re-render is needed if the channel
    // count changed OR if any channel crossed between unread and known, since
    // that swaps a badge for a toggle and patching cannot do it.
    const cur = list.length + ':' + list.map((r) => (r.known ? '1' : '0')).join('');
    if (cur !== sig) return false;
    renderHead(boardsOffline(st));
    for (const r of list) { const row = rows.get(r.ch); if (row) row.update(r); }
    return true;
  };
  return { nodes, update };
}

function relayRow(r) {
  // An unread channel gets no toggle: the controller refuses a toggle it cannot
  // base on a measured value, so offering the control would just produce an
  // error. Show why instead.
  const toggle = r.known ? onToggle(!!r.on, '/api/relay', { ch: r.ch })
                         : badge('unread', 'warn');
  const el = h('div', { class: 'row between', style: { background: 'var(--bg)', padding: '8px 10px', borderRadius: '6px' } },
    h('strong', {}, 'CH ', String(r.ch)),
    toggle,
  );
  const update = (rr) => {
    if (!rr.known || !r.known) return;   // known-ness changed: ask for a re-render
    toggle.setAttribute('aria-pressed', rr.on ? 'true' : 'false');
  };
  return { el, update };
}
