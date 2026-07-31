import { h, mount } from '../dom.js';
import { post } from '../api.js';
import { toastErr } from '../toast.js';
import { onToggle, saver, badge, numField, num } from '../util.js';

// Lights tab. Live: each ballast row patches its toggle/slider/badges in place
// on every poll so on/off and brightness track the device, without rebuilding
// the DOM (so a slider drag or an open Advanced panel is never clobbered). If the
// set of present ballasts changes, we ask for a full re-render (return false).
/* Which gateways are showing their unused addresses. Module-level so the choice
 * survives the re-renders that a poll triggers — it is view state, not device
 * state, so it deliberately does not live in `state`. */
const showAll = new Set();

export function renderLights(state, reload) {
  const sortB = (l) => l.slice().sort((a, b) => (a.slave - b.slave) || (a.addr - b.addr));
  const ballasts = sortB(state.ballasts || []);
  if (!ballasts.length) return h('div', { class: 'card sub' }, 'No ballasts known yet.');

  const keyOf = (b) => b.slave + ':' + b.addr;
  const rows = new Map();

  /* The device reports all 64 short addresses per bus, present or not. Filtering
   * is a PRESENTATION choice and belongs here: by default show only addresses
   * that answered, with a per-gateway toggle to reveal the rest — which is what
   * you need when commissioning, and noise the rest of the time. */
  const visible = (b) => b.present !== false || showAll.has(b.slave);

  /* The signature covers what is DRAWN, so flipping a toggle (or a ballast
   * appearing) forces a full re-render rather than being patched in place —
   * update() only refreshes existing rows and cannot add or remove them. */
  const sig = ballasts.filter(visible).map(keyOf).join('|');

  // group by slave — every configured gateway gets a card, even with nothing on it
  const bySlave = new Map();
  for (const b of ballasts) {
    if (!bySlave.has(b.slave)) bySlave.set(b.slave, []);
    bySlave.get(b.slave).push(b);
  }
  const nodes = [];
  for (const [slave, list] of bySlave) {
    const shown = list.filter(visible);
    const hidden = list.length - shown.length;
    const rowEls = shown.map((b) => { const r = ballastRow(b); rows.set(keyOf(b), r); return r.el; });
    /* Rescan: re-interrogate this gateway now instead of waiting for the
     * background sweep to come round. A newly powered ballast is otherwise
     * only noticed when the rotation reaches its address. Asynchronous on the
     * device — the row appears on a later poll, not when this returns, which is
     * why the toast says "started" rather than claiming a result. */
    const scan = h('button', { class: 'btn small' }, 'Rescan');
    scan.addEventListener('click', saver('/api/scan', () => ({ slave }),
                                         { okMsg: 'Scan started' }));
    const toggle = h('button', { class: 'btn small' },
                     showAll.has(slave) ? 'Hide unused' : 'Show all 64');
    toggle.addEventListener('click', () => {
      if (showAll.has(slave)) showAll.delete(slave); else showAll.add(slave);
      reload();   // structure changes, so redraw rather than patch
    });
    nodes.push(h('div', { class: 'card' },
      h('div', { class: 'row between' },
        h('h3', {}, 'Slave ', String(slave)),
        h('div', { class: 'row' },
          hidden ? h('span', { class: 'muted' }, String(hidden) + ' unused') : null,
          toggle, scan)),
      rowEls));
  }

  const update = (st) => {
    const list = sortB(st.ballasts || []);
    if (list.filter(visible).map(keyOf).join('|') !== sig) return false; // structure changed
    for (const b of list) { const r = rows.get(keyOf(b)); if (r) r.update(b); }
    return true;
  };
  return { nodes, update };
}

function ballastRow(b) {
  const path = '/api/dali';
  const id = { slave: b.slave, addr: b.addr };
  let dragging = false;

  const briVal = h('span', { class: 'muted', style: { fontVariantNumeric: 'tabular-nums', minWidth: '34px', textAlign: 'right' } }, String(b.bri ?? 0));
  const slider = h('input', {
    type: 'range', min: '0', max: '254', value: String(b.bri ?? 0),
    disabled: b.present === false,
  });
  slider.addEventListener('pointerdown', () => { dragging = true; });
  slider.addEventListener('input', () => { briVal.textContent = slider.value; });
  slider.addEventListener('change', async () => {
    dragging = false;
    try { await post(path, { ...id, bri: num(slider.value) }); }
    catch (e) { toastErr(e.unauthorized ? 'Not authorized' : 'Failed'); }
  });

  const toggle = onToggle(!!b.on, path, id);

  const head = h('div', { class: 'row' });
  const renderHead = (bb) => {
    const badges = [];
    if (bb.present === false) badges.push(badge('absent', 'muted'));
    if (bb.lamp_fail) badges.push(badge('lamp fail', 'bad'));
    if (bb.fading) badges.push(badge('fading', 'warn'));
    mount(head, h('strong', {}, 'A', String(bb.addr)), badges);
  };
  renderHead(b);

  const el = h('div', { class: 'card', style: { background: 'var(--bg)' } },
    h('div', { class: 'row between' }, head, toggle),
    h('div', { class: 'row', style: { marginTop: '8px' } }, h('span', { class: 'grow' }, slider), briVal),
    advanced(b, id, path),
  );

  const update = (bb) => {
    toggle.setAttribute('aria-pressed', bb.on ? 'true' : 'false');
    slider.disabled = bb.present === false;
    if (!dragging) {                       // don't fight an in-progress drag
      slider.value = String(bb.bri ?? 0);
      briVal.textContent = String(bb.bri ?? 0);
    }
    renderHead(bb);
  };
  return { el, update };
}

function advanced(b, id, path) {
  const fields = {};
  const mk = (key, label, max = 254) => {
    const f = numField(label, b[key], { min: '0', max: String(max) });
    fields[key] = f.input;
    return f.wrap;
  };

  const body = h('div', { class: 'grid2' },
    mk('min', 'Min'),
    mk('max', 'Max'),
    mk('power_on', 'Power-on level'),
    mk('sys_fail', 'System-fail level'),
    mk('fade_rate', 'Fade rate', 15),
    mk('fade_time', 'Fade time', 15),
  );

  const save = h('button', { class: 'btn primary small', style: { marginTop: '10px' } }, 'Save settings');
  save.addEventListener('click', saver(path, () => {
    const out = { ...id };
    for (const [k, input] of Object.entries(fields)) {
      if (input.value !== '') out[k] = num(input.value);
    }
    return out;
  }, { okMsg: 'Settings saved' }));

  return h('details', { class: 'adv' },
    h('summary', {}, 'Advanced (min/max/fade/…)'),
    body,
    save,
  );
}
