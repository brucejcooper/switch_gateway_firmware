// Small shared UI helpers used across tabs.
import { h } from './dom.js';
import { post } from './api.js';
import { toastOk, toastErr } from './toast.js';

// An on/off toggle that POSTs `body` with on:"toggle".
export function onToggle(isOn, path, body) {
  const btn = h('button', {
    class: 'toggle',
    type: 'button',
    'aria-pressed': isOn ? 'true' : 'false',
    title: 'Toggle',
  });
  btn.addEventListener('click', async () => {
    btn.disabled = true;
    // Read the CURRENT displayed state, not the value captured at render time
    // (the row's live updater keeps aria-pressed in sync with the device).
    const cur = btn.getAttribute('aria-pressed') === 'true';
    try {
      await post(path, { ...body, on: 'toggle' });
      btn.setAttribute('aria-pressed', cur ? 'false' : 'true'); // optimistic; poll confirms
    } catch (e) {
      toastErr(e.unauthorized ? 'Not authorized' : 'Failed');
    } finally {
      btn.disabled = false;
    }
  });
  return btn;
}

// Wrap an async save action with toast feedback. Returns a handler.
export function saver(path, getBody, opts = {}) {
  return async (ev) => {
    const btn = ev && ev.currentTarget;
    if (btn) btn.disabled = true;
    try {
      await post(path, getBody());
      toastOk(opts.okMsg || 'Saved');
      if (opts.after) opts.after();
    } catch (e) {
      toastErr(e.unauthorized ? 'Not authorized' : (e.message || 'Failed'));
    } finally {
      if (btn) btn.disabled = false;
    }
  };
}

export function badge(text, kind) {
  return h('span', { class: 'badge' + (kind ? ' ' + kind : '') }, text);
}

// Numeric input with a label, returns {wrap, input}.
export function numField(label, value, attrs = {}) {
  const input = h('input', { type: 'number', value: value ?? '', ...attrs });
  const wrap = h('div', {}, h('label', {}, label), input);
  return { wrap, input };
}

export function textField(label, value, attrs = {}) {
  const input = h('input', { type: 'text', value: value ?? '', ...attrs });
  const wrap = h('div', {}, h('label', {}, label), input);
  return { wrap, input };
}

export function num(v, d = 0) {
  const n = Number(v);
  return Number.isFinite(n) ? n : d;
}
