import { h } from '../dom.js';
import { post } from '../api.js';
import { toastOk, toastErr } from '../toast.js';

// Change the web UI Basic-auth credentials.
export function renderPassword(state) {
  const cur = (state.cfg && state.cfg.web_user) || '';

  const user = h('input', { type: 'text', value: cur, placeholder: 'admin' });
  const p1 = h('input', { type: 'password', placeholder: 'New password' });
  const p2 = h('input', { type: 'password', placeholder: 'Confirm password' });
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
