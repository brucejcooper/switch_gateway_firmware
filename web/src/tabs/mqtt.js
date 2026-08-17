import { h } from '../dom.js';
import { saver, textField, numField, num } from '../util.js';

// MQTT config form. Password is never shown; blank means "leave unchanged".
export function renderMqtt(state) {
  const m = (state.cfg && state.cfg.mqtt) || {};

  // maxlength mirrors the firmware field sizes (config.h); the device rejects
  // an over-long value with a 400 naming the limit, this just stops you typing
  // past it in the first place.
  const host = textField('Broker host', m.host, { placeholder: '192.168.1.10', maxlength: 63 });
  const port = numField('Port', m.port ?? 1883, { min: '1', max: '65535' });
  const user = textField('Username', m.user, { placeholder: '(optional)', maxlength: 47 });
  const prefix = textField('Topic prefix', m.prefix, { placeholder: 'lighting', maxlength: 23 });
  const pass = h('input', { type: 'password', maxlength: 63, placeholder: m.have_pass ? '•••••• (unchanged)' : '(none)' });

  const save = h('button', { class: 'btn primary' }, 'Save MQTT');
  save.addEventListener('click', saver('/api/mqtt', () => {
    const body = {
      host: host.input.value.trim(),
      port: num(port.input.value, 1883),
      user: user.input.value,
      prefix: prefix.input.value.trim(),
    };
    if (pass.value !== '') body.pass = pass.value;
    return body;
  }, { okMsg: 'MQTT saved' }));

  return h('div', { class: 'card' },
    h('h3', {}, 'MQTT broker'),
    host.wrap, port.wrap, user.wrap,
    h('label', {}, 'Password'),
    pass,
    h('div', { class: 'sub', style: { marginTop: '2px' } }, 'Leave blank to keep the existing password.'),
    prefix.wrap,
    h('div', { class: 'note' }, 'Applies after reboot.'),
    h('div', { style: { marginTop: '12px' } }, save),
  );
}
