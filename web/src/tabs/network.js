import { h } from '../dom.js';
import { saver, textField } from '../util.js';

// Network config form. Static fields hide when DHCP is on.
export function renderNetwork(state) {
  const net = (state.cfg && state.cfg.net) || {};

  const dhcp = h('input', { type: 'checkbox', checked: !!net.dhcp });
  const ip = textField('IP address', net.ip, { placeholder: '192.168.1.50' });
  const mask = textField('Subnet mask', net.mask, { placeholder: '255.255.255.0' });
  const gw = textField('Gateway', net.gw, { placeholder: '192.168.1.1' });
  const dns = textField('DNS', net.dns, { placeholder: '192.168.1.1' });

  const staticBox = h('div', {}, ip.wrap, mask.wrap, gw.wrap, dns.wrap);
  const sync = () => { staticBox.style.display = dhcp.checked ? 'none' : ''; };
  dhcp.addEventListener('change', sync);
  sync();

  const save = h('button', { class: 'btn primary' }, 'Save network');
  save.addEventListener('click', saver('/api/net', () => {
    const body = { dhcp: dhcp.checked };
    if (!dhcp.checked) {
      body.ip = ip.input.value.trim();
      body.mask = mask.input.value.trim();
      body.gw = gw.input.value.trim();
      body.dns = dns.input.value.trim();
    }
    return body;
  }, { okMsg: 'Network saved' }));

  return h('div', { class: 'card' },
    h('h3', {}, 'Network'),
    h('div', { class: 'muted' }, 'Current address: ' + (net.current_ip || net.ip || '?')),
    h('label', {}, 'Use DHCP'),
    h('div', { class: 'row' }, dhcp, h('span', { class: 'muted' }, 'Automatic addressing')),
    staticBox,
    h('div', { class: 'note' }, 'Applies after reboot.'),
    h('div', { style: { marginTop: '12px' } }, save),
  );
}
