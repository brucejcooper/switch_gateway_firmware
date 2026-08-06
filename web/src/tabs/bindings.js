import { h, mount } from '../dom.js';
import { saver, num } from '../util.js';

// Bindings tab: a fixtures x buttons grid. Clicking a cell opens a small target
// editor below the grid (so the grid stays compact and 360px-friendly).
//
// The grid also doubles as a LIVE BUTTON MONITOR. A button's index is which of
// the 8 column inputs it is wired to, not its position on the plate — the two
// orders differ per fitting — so the only reliable way to learn a plate's
// indices has been to press each button and read a serial log. Here, a press
// lights its own cell. /api/state carries a short newest-first history
// (`buttons`), because a press is instantaneous and a 1.5 s poll asking for
// "current" state would miss nearly all of them.
const LIVE_MS = 2500;      // how long a cell stays lit after its press

export function renderBindings(state, reload) {
  const cfg = state.cfg || {};
  const maxFix = num(cfg.max_fixtures, 0);
  const maxBtn = num(cfg.max_buttons, 0);
  const maxTgt = num(cfg.max_targets, 8);
  const mappings = state.mappings || [];

  if (!maxFix || !maxBtn) return h('div', { class: 'card sub' }, 'Binding grid size unknown.');

  const find = (f, b) =>
    mappings.find((m) => m.fixture === f && m.button === b);

  const editorHost = h('div', {});
  const cellOf = new Map();      // "f:b" -> cell element, for the live monitor
  const lastPress = h('div', { class: 'sub' }, 'Press a button to identify it.');

  // The payload's age_ms is resolved device-side at render, so it only advances
  // when the poll actually returns new data — an unchanged poll answers 304 and
  // freezes it. Stamp arrival per seq and add locally-elapsed time on top, so
  // ages keep counting between changes instead of sticking.
  let stampSeq = null, stampAt = 0;
  const agesFrom = (st) => {
    if (st.seq !== stampSeq) { stampSeq = st.seq; stampAt = Date.now(); }
    const extra = Date.now() - stampAt;
    return (st.buttons || []).map((e) => ({ ...e, age: num(e.age_ms, 0) + extra }));
  };

  const grid = h('div', { class: 'card' },
    h('h3', {}, 'Bindings — tap a cell to edit'),
    h('div', { class: 'sub', style: { marginBottom: '2px' } },
      maxFix + ' fixtures × ' + maxBtn + ' buttons, up to ' + maxTgt + ' targets each'),
    h('div', { style: { marginBottom: '8px' } }, lastPress),
  );

  for (let f = 0; f < maxFix; f++) {
    const cells = [];
    for (let b = 0; b < maxBtn; b++) {
      const m = find(f, b);
      const tgts = (m && m.targets) || [];
      const cell = h('div', { class: 'bind-cell' + (tgts.length ? ' has' : ''), title: 'Fixture ' + f + ' button ' + b },
        h('div', { class: 'ttl' }, 'B' + b),
        h('div', { class: 'tgts' }, tgts.length ? tgts.map(fmtTgt).join(', ') : '—'),
      );
      cell.addEventListener('click', () => {
        mount(editorHost, targetEditor(f, b, tgts, maxTgt, reload));
        editorHost.scrollIntoView({ behavior: 'smooth', block: 'nearest' });
      });
      cellOf.set(f + ':' + b, cell);
      cells.push(cell);
    }
    grid.appendChild(h('div', { style: { marginBottom: '10px' } },
      h('div', { class: 'sub', style: { marginBottom: '4px' } }, 'Fixture ' + f),
      h('div', { class: 'bind-grid', style: { gridTemplateColumns: 'repeat(' + Math.min(maxBtn, 4) + ', 1fr)' } }, cells),
    ));
  }

  // Signature of what the STRUCTURE depends on. If bindings themselves change,
  // ask for a full re-render; the monitor only ever patches classes and text.
  const sig = JSON.stringify(mappings) + '|' + maxFix + 'x' + maxBtn;

  const update = (st) => {
    if (JSON.stringify(st.mappings || []) + '|' +
        num((st.cfg || {}).max_fixtures, 0) + 'x' +
        num((st.cfg || {}).max_buttons, 0) !== sig) return false;

    const events = agesFrom(st);
    const lit = new Map();
    for (const e of events) {
      if (e.age > LIVE_MS) continue;
      const k = e.fixture + ':' + e.button;
      if (!lit.has(k) || e.age < lit.get(k)) lit.set(k, e.age);
    }
    for (const [k, cell] of cellOf) cell.classList.toggle('live', lit.has(k));

    const top = events[0];
    lastPress.textContent = top
      ? 'Last press: Fixture ' + top.fixture + ' · B' + top.button +
        ' (' + top.action + ') ' + fmtAge(top.age)
      : 'Press a button to identify it.';
    lastPress.classList.toggle('accent', !!top && top.age <= LIVE_MS);
    return true;
  };
  update(state);

  return { nodes: [grid, editorHost], update };
}

function fmtAge(ms) {
  if (ms < 1500) return 'just now';
  if (ms < 60000) return Math.round(ms / 1000) + ' s ago';
  if (ms < 3600000) return Math.round(ms / 60000) + ' min ago';
  return 'a while ago';
}

function fmtTgt(t) {
  if (t.group != null) return t.slave + ':g' + t.group;
  return t.slave + ':' + t.addr;
}

function targetEditor(fixture, button, targets, maxTgt, reload) {
  const rows = [];
  const rowsHost = h('div', {});

  const addRow = (t) => {
    if (rows.length >= maxTgt) return;
    const kind = h('select', {},
      h('option', { value: 'addr', selected: !(t && t.group != null) }, 'Ballast'),
      h('option', { value: 'group', selected: t && t.group != null }, 'Group'),
    );
    const slave = h('input', { type: 'number', min: '0', value: t ? t.slave : 2, placeholder: 'slave' });
    const addr = h('input', { type: 'number', min: '0', value: t ? (t.group != null ? t.group : t.addr) : 0, placeholder: 'addr' });
    const del = h('button', { class: 'btn small', type: 'button' }, '✕');
    const row = h('div', { class: 'tgt-row' }, kind, slave, addr, del);
    const rec = { kind, slave, addr, row };
    del.addEventListener('click', () => {
      const i = rows.indexOf(rec);
      if (i >= 0) rows.splice(i, 1);
      row.remove();
    });
    rows.push(rec);
    rowsHost.appendChild(row);
  };

  (targets.length ? targets : []).forEach(addRow);

  const addBtn = h('button', { class: 'btn small', type: 'button', style: { marginTop: '4px' } }, '+ target');
  addBtn.addEventListener('click', () => {
    if (rows.length >= maxTgt) return;
    addRow(null);
  });

  const save = h('button', { class: 'btn primary small' }, 'Save binding');
  save.addEventListener('click', saver('/api/mapping', () => ({
    fixture, button,
    targets: rows.map((r) => {
      const slave = num(r.slave.value);
      const v = num(r.addr.value);
      return r.kind.value === 'group' ? { slave, group: v } : { slave, addr: v };
    }),
  }), { okMsg: 'Binding saved', after: reload }));

  const clear = h('button', { class: 'btn small' }, 'Clear (delete)');
  clear.addEventListener('click', saver('/api/mapping', () => ({ fixture, button, targets: [] }),
    { okMsg: 'Binding deleted', after: reload }));

  return h('div', { class: 'card' },
    h('h3', {}, 'Fixture ', String(fixture), ' · Button ', String(button)),
    rowsHost,
    addBtn,
    h('div', { class: 'row', style: { marginTop: '10px' } }, save, clear),
  );
}
