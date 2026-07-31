import './style.css';
import { h, mount } from './dom.js';
import { setAfterWrite, getState } from './api.js';
import { renderDashboard } from './tabs/dashboard.js';
import { renderLights } from './tabs/lights.js';
import { renderGroups } from './tabs/groups.js';
import { renderRelays } from './tabs/relays.js';
import { renderBindings } from './tabs/bindings.js';
import { renderController } from './tabs/controller.js';
import { renderNetwork } from './tabs/network.js';
import { renderMqtt } from './tabs/mqtt.js';
import { renderProvision } from './tabs/provision.js';
import { renderFirmware } from './tabs/firmware.js';
import { renderPassword } from './tabs/password.js';

const POLL_MS = 1500;

// Tab definitions. `live: true` views are re-rendered on every poll (read-only
// / progress views); `live: false` config forms render once on tab-open/reload
// so they are not clobbered while the user is editing.
const TABS = [
  { id: 'dashboard', label: 'Dashboard', render: renderDashboard, live: true },
  { id: 'lights', label: 'Lights', render: renderLights, live: true },
  { id: 'groups', label: 'Groups', render: renderGroups, live: true },
  { id: 'relays', label: 'Relays', render: renderRelays, live: true },
  { id: 'bindings', label: 'Bindings', render: renderBindings, live: false },
  { id: 'controller', label: 'Controller', render: renderController, live: false },
  { id: 'network', label: 'Network', render: renderNetwork, live: false },
  { id: 'mqtt', label: 'MQTT', render: renderMqtt, live: false },
  { id: 'provision', label: 'Provision', render: renderProvision, live: true },
  { id: 'firmware', label: 'Firmware', render: renderFirmware, live: false },
  { id: 'password', label: 'Password', render: renderPassword, live: false },
];

const app = {
  state: null,
  active: null,
  el: {},
};

function init() {
  const root = document.getElementById('app');

  const tabBar = h('nav', { class: 'tabs' });
  const banner = h('div', {});
  const content = h('main', {});
  const fw = h('span', { class: 'fw' }, '');

  const tabButtons = TABS.map((t) => {
    const b = h('button', { type: 'button' }, t.label);
    b.addEventListener('click', () => setActive(t.id, true));
    return b;
  });
  tabBar.append(...tabButtons);

  mount(root,
    h('header', { class: 'top' },
      h('div', { class: 'brand' }, h('h1', {}, 'Light Switch'), fw),
      tabBar,
    ),
    banner,
    content,
  );

  app.el = { content, banner, fw, tabButtons };

  setActive(fromHash() || 'dashboard', false);
  window.addEventListener('hashchange', () => setActive(fromHash() || 'dashboard', false));

  poll();
  setInterval(poll, POLL_MS);

  /* After a successful write, read back promptly rather than waiting up to
   * POLL_MS. Two reads: one now, which usually already carries the new value
   * because the device is sub-second, and one a little later to catch a command
   * whose verification read had not landed yet (a DALI transaction is ~70 ms,
   * and a ballast mid-fade keeps reporting until it settles).
   *
   * Costs two extra requests per user action, not a faster background poll for
   * everyone — which matters here because the W5100S has only two sockets spare
   * for the web server. */
  setAfterWrite(() => { poll(); setTimeout(poll, 400); });
}

function fromHash() {
  const id = location.hash.replace(/^#/, '');
  return TABS.some((t) => t.id === id) ? id : null;
}

function setActive(id, viaClick) {
  app.active = id;
  if (viaClick && location.hash !== '#' + id) {
    history.replaceState(null, '', '#' + id);
  }
  TABS.forEach((t, i) => {
    app.el.tabButtons[i].classList.toggle('active', t.id === id);
  });
  renderActive();
}

// Full render of the active tab. Used on tab-open, reload, and (for live tabs)
// on each poll.
function renderActive() {
  const tab = TABS.find((t) => t.id === app.active);
  if (!tab) return;
  if (!app.state) {
    mount(app.el.content, h('div', { class: 'card sub' }, 'Loading…'));
    return;
  }
  const reload = () => renderActive();
  const result = tab.render(app.state, reload);
  // A tab may return either DOM node(s) or { nodes, update } — the latter lets a
  // live tab patch its existing DOM each poll (preserving slider drags, focus,
  // and open expanders) instead of being rebuilt. update(state) returns false if
  // the structure changed and a full re-render is needed.
  if (result && typeof result.update === 'function' && 'nodes' in result) {
    app.liveUpdate = result.update;
    mount(app.el.content, result.nodes);
  } else {
    app.liveUpdate = null;
    mount(app.el.content, result);
  }
}

// On poll, refresh the active tab if it is "live": prefer an in-place update
// (no clobbering of inputs being edited); fall back to a full re-render.
function refreshLive() {
  const tab = TABS.find((t) => t.id === app.active);
  if (!tab || !tab.live) return;
  if (app.liveUpdate) {
    if (app.liveUpdate(app.state) === false) renderActive();
  } else {
    renderActive();
  }
}

/* Single-flight. Three things now ask for a poll — the 1.5 s interval and the
 * two after-write reads — and without this they can overlap: a slow response
 * could land after a newer one and overwrite fresher state with staler, and
 * concurrent requests compete for the two W5100S sockets the web server has.
 * A request arriving mid-poll sets a flag and runs once the current one
 * finishes, so requests coalesce instead of stacking. */
let polling = false;
let pollAgain = false;

async function poll() {
  if (polling) { pollAgain = true; return; }
  polling = true;
  try {
    const s = await getState();
    const wasEmpty = app.state == null;
    app.state = s;
    setBanner(null);
    app.el.fw.textContent = (s.cfg && s.cfg.fw_version) || '';
    if (wasEmpty) renderActive(); // first data: render whatever tab is open
    else refreshLive();
  } catch (e) {
    if (e.unauthorized) {
      setBanner('Not authorized — reload to log in.', '');
    } else {
      setBanner('Device unreachable — retrying…', 'warn');
    }
  } finally {
    polling = false;
    if (pollAgain) { pollAgain = false; poll(); }
  }
}

function setBanner(text, kind) {
  if (!text) { mount(app.el.banner); return; }
  mount(app.el.banner, h('div', { class: 'banner' + (kind ? ' ' + kind : '') }, text));
}

init();
