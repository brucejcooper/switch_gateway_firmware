import { h } from '../dom.js';
import { saver, numField, num } from '../util.js';

// Controller config form. Populated once on render (tab-open / reload).
export function renderController(state) {
  const cfg = state.cfg || {};

  const lp = numField('Long-press (ms)', cfg.long_press_ms, { min: '0' });
  const dr = numField('Dim repeat (ms)', cfg.dim_repeat_ms, { min: '0' });
  const rw = numField('Reversal window (ms)', cfg.reversal_window_ms, { min: '0' });
  const gd = numField('Gateway diag poll (ms, 0=off)', cfg.gw_diag_interval_ms, { min: '0' });
  const sw = numField('DALI background sweep (ms, 0=off)', cfg.dali_sweep_interval_ms, { min: '0' });
  const baud = numField('RS-485 baud', cfg.baud, { min: '0' });

  const de = h('select', {},
    h('option', { value: '0', selected: num(cfg.de_mode) === 0 }, 'When transmitting'),
    h('option', { value: '1', selected: num(cfg.de_mode) === 1 }, 'Always on'),
  );

  const byteswap = h('input', { type: 'checkbox', checked: num(cfg.relay_byteswap) === 1 });

  const save = h('button', { class: 'btn primary' }, 'Save controller config');
  save.addEventListener('click', saver('/api/cfg', () => ({
    long_press_ms: num(lp.input.value),
    dim_repeat_ms: num(dr.input.value),
    reversal_window_ms: num(rw.input.value),
    gw_diag_interval_ms: num(gd.input.value),
    dali_sweep_interval_ms: num(sw.input.value),
    baud: num(baud.input.value),
    de_mode: num(de.value),
    relay_byteswap: byteswap.checked ? 1 : 0,
  }), { okMsg: 'Controller config saved' }));

  return h('div', { class: 'card' },
    h('h3', {}, 'Controller'),
    h('div', { class: 'grid2' }, lp.wrap, dr.wrap, rw.wrap, gd.wrap, sw.wrap, baud.wrap),
    h('label', {}, 'DE / driver-enable mode'),
    de,
    h('label', {}, 'Relay coil byte-swap'),
    h('div', { class: 'row' }, byteswap,
      h('span', { class: 'muted' }, 'Reverse coil read-back byte order (Waveshare 32CH)')),
    h('div', { class: 'note' }, 'Changing baud applies after reboot.'),
    h('div', { style: { marginTop: '12px' } }, save),
  );
}
