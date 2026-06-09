import { h, mount } from '../dom.js';
import { saver, num } from '../util.js';

// Bindings tab: a fixtures x buttons grid. Clicking a cell opens a small target
// editor below the grid (so the grid stays compact and 360px-friendly).
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

  const grid = h('div', { class: 'card' },
    h('h3', {}, 'Bindings — tap a cell to edit'),
    h('div', { class: 'sub', style: { marginBottom: '8px' } },
      maxFix + ' fixtures × ' + maxBtn + ' buttons, up to ' + maxTgt + ' targets each'),
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
      cells.push(cell);
    }
    grid.appendChild(h('div', { style: { marginBottom: '10px' } },
      h('div', { class: 'sub', style: { marginBottom: '4px' } }, 'Fixture ' + f),
      h('div', { class: 'bind-grid', style: { gridTemplateColumns: 'repeat(' + Math.min(maxBtn, 4) + ', 1fr)' } }, cells),
    ));
  }

  return [grid, editorHost];
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
