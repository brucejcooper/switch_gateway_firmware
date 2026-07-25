// Same-origin API helpers. HTTP Basic auth is handled by the browser; we never
// set Authorization ourselves. The browser includes cached credentials on
// same-origin requests automatically.

import { decodeState } from './decode.js';

export async function getState() {
  const res = await fetch('/api/state', { headers: { Accept: 'application/json' } });
  if (res.status === 401) {
    const e = new Error('unauthorized');
    e.unauthorized = true;
    throw e;
  }
  if (!res.ok) throw new Error('HTTP ' + res.status);
  // The wire format is compact positional arrays; decodeState() is the single
  // boundary that turns it into the named model the tabs consume.
  return decodeState(await res.json());
}

// Firmware/OTA status (GET /api/ota) — installed/latest/available/state/pct.
export async function getOta() {
  const res = await fetch('/api/ota', { headers: { Accept: 'application/json' } });
  if (res.status === 401) {
    const e = new Error('unauthorized');
    e.unauthorized = true;
    throw e;
  }
  if (!res.ok) throw new Error('HTTP ' + res.status);
  return res.json();
}

// POST a JSON body to `path`. Resolves to the parsed response (or {ok:true}
// when the body is empty) and rejects on non-2xx / 401.
export async function post(path, obj) {
  const res = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(obj),
  });
  if (res.status === 401) {
    const e = new Error('unauthorized');
    e.unauthorized = true;
    throw e;
  }
  let data = null;
  try { data = await res.json(); } catch { /* empty / non-JSON body */ }
  if (!res.ok || (data && data.ok === false)) {
    throw new Error((data && data.error) || 'HTTP ' + res.status);
  }
  return data || { ok: true };
}
