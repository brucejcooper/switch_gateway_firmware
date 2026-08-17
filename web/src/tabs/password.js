import { h } from '../dom.js';
import { post } from '../api.js';
import { toastOk, toastErr } from '../toast.js';

// Change the web UI Basic-auth credentials.
export function renderPassword(state) {
  const cur = (state.cfg && state.cfg.web_user) || '';

  // maxlength mirrors web_user[32]/web_pass[48] in config.h. Worth stopping
  // here rather than at the device: a credential that did not fit used to be
  // truncated, saved, reported success — and then would not let you back in.
  const user = h('input', { type: 'text', value: cur, placeholder: 'admin', maxlength: 31 });
  const p1 = h('input', { type: 'password', placeholder: 'New password', maxlength: 47 });
  const p2 = h('input', { type: 'password', placeholder: 'Confirm password', maxlength: 47 });
  const msg = h('div', { class: 'note', style: { color: 'var(--ok)', display: 'none' } });

  const save = h('button', { class: 'btn primary' }, 'Change credentials');
  save.addEventListener('click', async () => {
    if (!user.value.trim()) { toastErr('Username required'); return; }
    if (!p1.value) { toastErr('Password required'); return; }
    if (p1.value !== p2.value) { toastErr('Passwords do not match'); return; }
    save.disabled = true;
    try {
      await post('/api/auth', { user: user.value.trim(), pass: p1.value });
      msg.textContent = 'Password changed — reload and log in again with the new credentials.';
      msg.style.display = '';
      p1.value = ''; p2.value = '';
      toastOk('Credentials changed');
    } catch (e) {
      toastErr(e.unauthorized ? 'Not authorized' : (e.message || 'Failed'));
    } finally {
      save.disabled = false;
    }
  });

  return h('div', { class: 'card' },
    h('h3', {}, 'Web UI password'),
    h('label', {}, 'Username'), user,
    h('label', {}, 'New password'), p1,
    h('label', {}, 'Confirm password'), p2,
    h('div', { style: { marginTop: '12px' } }, save),
    msg,
  );
}
