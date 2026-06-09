// Transient toast notifications.

let host = null;

function ensureHost() {
  if (!host) {
    host = document.createElement('div');
    host.id = 'toasts';
    document.body.appendChild(host);
  }
  return host;
}

export function toast(msg, kind = '') {
  const el = document.createElement('div');
  el.className = 'toast' + (kind ? ' ' + kind : '');
  el.textContent = msg;
  ensureHost().appendChild(el);
  // force reflow then show
  requestAnimationFrame(() => el.classList.add('show'));
  setTimeout(() => {
    el.classList.remove('show');
    setTimeout(() => el.remove(), 250);
  }, 2200);
}

export function toastOk(msg = 'Saved') { toast(msg, 'ok'); }
export function toastErr(msg = 'Error') { toast(msg, 'err'); }
