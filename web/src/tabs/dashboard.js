import { h } from '../dom.js';
import { badge } from '../util.js';

// Read-only overview. Re-rendered on every poll, so no user-editable inputs here.
export function renderDashboard(state) {
  const cfg = state.cfg || {};
  const net = cfg.net || {};
  const mqtt = cfg.mqtt || {};
  const gateways = state.gateways || [];
  const ballasts = state.ballasts || [];
  const groups = state.groups || [];

  return [
    h('div', { class: 'card' },
      h('h3', {}, 'Gateways & boards'),
      gateways.length
        ? gateways.map((g) =>
            h('div', { class: 'row between', style: { padding: '4px 0' } },
              h('span', {}, (g.kind === 'relay' ? 'Relay board ' : 'DALI gateway ') + 'slave ' + g.slave),
              badge(g.online ? 'online' : 'offline', g.online ? 'ok' : 'bad'),
            ),
          )
        : h('div', { class: 'sub' }, 'No gateways reported'),
    ),

    h('div', { class: 'card' },
      h('h3', {}, 'Summary'),
      stat('Ballasts', String(ballasts.filter((b) => b.present !== false).length) + ' present / ' + ballasts.length + ' known'),
      stat('Groups', String(groups.length)),
      stat('Firmware', cfg.fw_version || '?'),
      stat('Update seq', state.seq != null ? String(state.seq) : '?'),
    ),

    h('div', { class: 'card' },
      h('h3', {}, 'Network'),
      stat('Mode', net.dhcp ? 'DHCP' : 'Static'),
      stat('IP', net.current_ip || net.ip || '?'),
      stat('Gateway', net.gw || '?'),
      stat('DNS', net.dns || '?'),
    ),

    h('div', { class: 'card' },
      h('h3', {}, 'MQTT'),
      stat('Broker', (mqtt.host || '?') + ':' + (mqtt.port ?? '?')),
      stat('User', mqtt.user || '(none)'),
      stat('Prefix', mqtt.prefix || '?'),
      stat('Password set', mqtt.have_pass ? 'yes' : 'no'),
    ),
  ];
}

function stat(k, v) {
  return h('div', { class: 'stat' }, h('span', { class: 'k' }, k), h('span', {}, v));
}
