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

// Called after every SUCCESSFUL write. main.js uses it to re-read /api/state
// immediately instead of waiting out the poll interval.
//
// The device applies a command and republishes the result in well under a
// second (measured: a group toggle's ballast read and all three derived group
// states land in the same second as the request). The visible lag was purely
// this UI's 1.5 s poll. So look sooner — do NOT predict the new value locally.
// Everything rendered here still comes from a device read; the only thing that
// changes is when we ask.
let afterWrite = null;
export function setAfterWrite(fn) { afterWrite = fn; }

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
  if (afterWrite) afterWrite();   // success only — a rejected write changed nothing
  return data || { ok: true };
}
