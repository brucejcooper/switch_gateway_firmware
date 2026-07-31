import { h } from '../dom.js';
import { post } from '../api.js';
import { toastErr } from '../toast.js';
import { onToggle, saver, num, badge } from '../util.js';

// Groups tab. Live: each group card patches its toggle/slider in place on poll
// (the members editor is left alone so edits aren't clobbered). Full re-render
// if the set of groups changes.
export function renderGroups(state) {
  const sortG = (l) => l.slice().sort((a, b) => (a.slave - b.slave) || (a.group - b.group));
  const groups = sortG(state.groups || []);
  if (!groups.length) return h('div', { class: 'card sub' }, 'No DALI gateways.');

  const keyOf = (g) => g.slave + ':' + g.group;
  /* The signature includes emptiness, not just identity: the set of groups is
   * now fixed at 16 per bus so the keys never change, but a group gaining its
   * first member (or losing its last) adds or removes the "empty" badge, and
   * update() only patches the toggle/slider/checkboxes. Without this the badge
   * would go stale the moment you built a group. */
  const sigOf = (l) => l.map((g) => keyOf(g) + (g.empty ? 'e' : '')).join('|');
  const sig = sigOf(groups);
  const cards = new Map();
  const nodes = groups.map((g) => { const c = groupCard(g); cards.set(keyOf(g), c); return c.el; });

  const update = (st) => {
    const list = sortG(st.groups || []);
    if (sigOf(list) !== sig) return false;
    for (const g of list) { const c = cards.get(keyOf(g)); if (c) c.update(g); }
    return true;
  };
  return { nodes, update };
}

function groupCard(g) {
  const path = '/api/group';
  const id = { slave: g.slave, group: g.group };
  let dragging = false;

  const briVal = h('span', { class: 'muted', style: { minWidth: '34px', textAlign: 'right' } }, String(g.bri ?? 0));
  const slider = h('input', { type: 'range', min: '0', max: '254', value: String(g.bri ?? 0) });
  slider.addEventListener('pointerdown', () => { dragging = true; });
  slider.addEventListener('input', () => { briVal.textContent = slider.value; });
  slider.addEventListener('change', async () => {
    dragging = false;
    try { await post(path, { ...id, bri: num(slider.value) }); }
    catch (e) { toastErr(e.unauthorized ? 'Not authorized' : 'Failed'); }
  });

  const toggle = onToggle(!!g.on, path, id);

  const members = new Set(g.members || []);
  const boxes = [];
  const grid = h('div', { class: 'members' },
    Array.from({ length: 64 }, (_, i) => {
      const cb = h('input', { type: 'checkbox', checked: members.has(i) });
      boxes.push({ i, cb });
      return h('label', {}, cb, String(i));
    }),
  );

  const saveMembers = h('button', { class: 'btn primary small', style: { marginTop: '8px' } }, 'Save members');
  saveMembers.addEventListener('click', saver(path, () => ({
    ...id,
    members: boxes.filter((b) => b.cb.checked).map((b) => b.i),
  }), { okMsg: 'Members saved' }));

  const el = h('div', { class: 'card' },
    h('div', { class: 'row between' },
      h('strong', {}, 'Slave ', String(g.slave), ' · Group ', String(g.group)),
      /* All 16 groups per bus are listed whether or not they have members —
       * ticking a member here is what creates one. Flag the empty ones so the
       * populated groups are still easy to pick out of the list. */
      g.empty ? badge('empty', 'muted') : null,
      toggle,
    ),
    h('div', { class: 'row', style: { marginTop: '8px' } }, h('span', { class: 'grow' }, slider), briVal),
    h('details', { class: 'adv' },
      h('summary', {}, 'Members (' + members.size + ')'),
      grid,
      saveMembers,
    ),
  );

  const update = (gg) => {
    toggle.setAttribute('aria-pressed', gg.on ? 'true' : 'false');
    if (!dragging) {
      slider.value = String(gg.bri ?? 0);
      briVal.textContent = String(gg.bri ?? 0);
    }
  };
  return { el, update };
}
