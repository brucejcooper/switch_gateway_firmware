import { h, mount } from '../dom.js';
import { getOta, post } from '../api.js';
import { toastOk, toastErr } from '../toast.js';

// Firmware / OTA tab. Fetches its own status from GET /api/ota (not part of the
// /api/state poll), so it's registered `live: false` and self-refreshes: after a
// Check, and repeatedly while an Install is in flight until the device reboots.
export function renderFirmware() {
  const body = h('div', {}, h('div', { class: 'muted' }, 'Loading…'));
  const card = h('div', { class: 'card' }, h('h3', {}, 'Firmware'), body);

  const kv = (k, v) => h('div', { class: 'row' },
    h('span', { class: 'muted' }, k + ': '), h('span', {}, v));

  const draw = (o) => {
    const avail = !!o.available;
    const running = o.state === 'running';

    const check = h('button', { class: 'btn' }, 'Check for Updates');
    check.addEventListener('click', async (ev) => {
      ev.currentTarget.disabled = true;
      try {
        await post('/api/ota/check', {});
        toastOk('Checking…');
        setTimeout(refresh, 2500);   // give the device a moment to fetch the manifest
      } catch (e) {
        toastErr(e.unauthorized ? 'Not authorized' : (e.message || 'Failed'));
      } finally {
        if (ev.currentTarget) ev.currentTarget.disabled = false;
      }
    });

    const install = h('button', { class: 'btn primary' }, 'Install Update');
    install.disabled = !avail || running;
    install.addEventListener('click', async (ev) => {
      if (!window.confirm('Install firmware ' + (o.latest || '') +
                          '?\nThe controller will download, verify and reboot.')) return;
      ev.currentTarget.disabled = true;
      try {
        await post('/api/ota/install', {});
        toastOk('Update started');
        pollProgress();
      } catch (e) {
        toastErr(e.message || 'Failed');
        ev.currentTarget.disabled = false;
      }
    });

    const stateLine =
      running ? h('div', { class: 'note' }, 'Updating… ' + (o.pct || 0) + '%')
      : o.state === 'success' ? h('div', { class: 'note' }, 'Update complete')
      : o.state === 'fail'
        ? h('div', { class: 'note err' }, 'Update failed' + (o.ota_error ? ': ' + o.ota_error : ''))
      : null;

    mount(body,
      kv('Installed', o.installed || '?'),
      kv('Latest', o.latest || '—'),
      avail
        ? h('div', { class: 'note' }, 'Update available' + (o.title ? ': ' + o.title : ''))
        : h('div', { class: 'muted' }, 'Up to date'),
      o.notes ? h('div', { class: 'muted', style: { marginTop: '6px' } }, o.notes) : null,
      o.release_url
        ? h('div', { style: { marginTop: '6px' } },
            h('a', { href: o.release_url, target: '_blank', rel: 'noreferrer' }, 'Release notes'))
        : null,
      stateLine,
      h('div', { class: 'row', style: { marginTop: '12px', gap: '8px' } }, check, install),
      h('div', { class: 'note', style: { marginTop: '8px' } },
        'Checks automatically every hour and on reconnect.'),
    );
  };

  const refresh = async () => {
    if (!card.isConnected) return;   // tab switched away
    try {
      draw(await getOta());
    } catch (e) {
      mount(body, h('div', { class: 'muted' },
        e.unauthorized ? 'Not authorized' : 'Firmware status unavailable'));
    }
  };

  // Keep polling while an install runs; the device reboots to apply, so a failed
  // fetch is the expected "it's rebooting" signal rather than an error.
  const pollProgress = () => {
    if (!card.isConnected) return;
    getOta()
      .then((o) => {
        draw(o);
        if (o.state === 'running') setTimeout(pollProgress, 2000);
      })
      .catch(() => {
        mount(body, h('div', { class: 'note' },
          'Controller is rebooting to apply the update — this page will reconnect shortly.'));
      });
  };

  /* Deferred, NOT called directly. renderFirmware() returns `card` and main.js
   * mounts it only after that return, so a synchronous refresh() ran while the
   * card was still detached — its `if (!card.isConnected) return` guard fired
   * every time and the "Loading…" placeholder was never replaced. This tab could
   * not have loaded at all.
   *
   * setTimeout(0) puts the first fetch in the next task, by which point the
   * mount above has happened and the guard passes. The guard is still worth
   * keeping: it stops a late response from a tab the user has already navigated
   * away from drawing into a detached node. */
  setTimeout(refresh, 0);
  return card;
}
