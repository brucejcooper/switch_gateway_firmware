import { h } from '../dom.js';

/* Docs tab: how to drive this device from outside the web UI.
 *
 * Written for someone who has just plugged the box in. The topic and route
 * lists are transcribed from the firmware itself — ha_bridge.c's subscribe
 * table and publish sites, and http_server.c's route table — not from memory,
 * so they describe what the device actually answers to. `<prefix>` is the MQTT
 * prefix from the MQTT tab (default `lighting`).
 */

const sec = (title, ...kids) =>
  h('div', { class: 'card' }, h('h3', {}, title), ...kids);

const p = (...t) => h('p', { style: { margin: '6px 0' } }, ...t);
const mut = (t) => h('div', { class: 'sub', style: { margin: '4px 0 10px' } }, t);
const code = (t) => h('code', {
  style: { background: 'var(--bg)', padding: '1px 5px', borderRadius: '4px',
           fontSize: '13px', wordBreak: 'break-all' },
}, t);

/* Rows scroll inside their own container: topic strings are long and a phone
 * must not end up scrolling the whole page sideways. */
const table = (head, rows) => h('div', { style: { overflowX: 'auto', margin: '8px 0' } },
  h('table', { style: { borderCollapse: 'collapse', width: '100%', fontSize: '13px' } },
    h('thead', {}, h('tr', {}, ...head.map((x) =>
      h('th', { style: { textAlign: 'left', padding: '4px 8px 4px 0',
                         borderBottom: '1px solid var(--border)',
                         whiteSpace: 'nowrap' } }, x)))),
    h('tbody', {}, ...rows.map((r) => h('tr', {}, ...r.map((cell, i) =>
      h('td', { style: { padding: '4px 8px 4px 0', verticalAlign: 'top',
                         borderBottom: '1px solid var(--border)',
                         whiteSpace: i === 0 ? 'nowrap' : 'normal' } },
        i === 0 ? code(cell) : cell)))))));

export function renderDocs() {
  return [
    sec('How this device is controlled',
      p('Three interfaces, all driving the same state: this web UI, MQTT (which is ' +
        'how Home Assistant talks to it), and a REST API. There is also a USB serial ' +
        'console for local diagnostics.'),
      p('The controller is the ', h('strong', {}, 'source of truth for the bus'), '. ' +
        'It only ever reports values it has read from the hardware — it never publishes ' +
        'a requested level as though it were confirmed. Ask for brightness 5 on a ballast ' +
        'whose minimum is 85 and you will see 85 reported, because that is what the ' +
        'ballast actually did.'),
      p('A device that has not answered yet is published as ', code('unavailable'),
        ' rather than guessed at, so entities appear as the bus is discovered.')),

    sec('MQTT — commands you send',
      mut('Replace <prefix> with the MQTT prefix from the MQTT tab (default "lighting"). ' +
          'Home Assistant discovers everything automatically; these are for scripts, ' +
          'Node-RED, or anything else on the broker.'),
      table(['Topic', 'Payload'], [
        ['<prefix>/dali/<slave>/<addr>/set', '{"state":"ON"} · {"state":"OFF"} · {"state":"ON","brightness":0-254}'],
        ['<prefix>/dali/<slave>/<addr>/config/<field>/set', 'A number. Fields: min, max, power_on, sys_fail (0-255, 255 = MASK/keep current), fade_rate, fade_time (0-15)'],
        ['<prefix>/group/<slave>/<group>/set', 'Same shape as a ballast. Groups are 0-15.'],
        ['<prefix>/group/<slave>/<group>/members/set', 'Comma-separated short addresses, e.g. "0,45". Empty clears the group.'],
        ['<prefix>/relay/<ch>/set', '"1" / "0". Channels 0-31.'],
        ['<prefix>/binding/<fixture>/<button>/set', 'Targets for a wall button.'],
        ['<prefix>/mapping/<fixture>/<button>/set', 'Button-to-target mapping; /delete removes one.'],
        ['<prefix>/config/set', 'Controller settings; <prefix>/config/<key>/set for one key.'],
        ['<prefix>/provision/<slave>/start/all', 'Commission every ballast (renumbers the bus).'],
        ['<prefix>/provision/<slave>/start/unaddressed', 'Commission only gear with no address.'],
        ['<prefix>/provision/<slave>/start/address', 'Re-provision one address; set it via .../start/target first.'],
        ['<prefix>/provision/<slave>/abort', 'Stop a run in progress.'],
        ['<prefix>/update/check', '"CHECK" — re-read the firmware manifest.'],
        ['<prefix>/update/install', '"INSTALL" — download, verify and reboot.'],
        ['<prefix>/system/reboot', 'Reboot the controller.'],
      ])),

    sec('MQTT — state you receive',
      mut('All retained, so a subscriber gets the current picture on connect.'),
      table(['Topic', 'Meaning'], [
        ['<prefix>/availability', 'The controller itself: "online" / "offline" (also its last will).'],
        ['<prefix>/gateway/<slave>/availability', 'One DALI gateway or relay board. Offline until it answers.'],
        ['<prefix>/dali/<slave>/<addr>/availability', 'One ballast. Offline until it answers a scan.'],
        ['<prefix>/dali/<slave>/<addr>/state', '{"state":"ON","brightness":N} — the MEASURED level, including each step of a fade.'],
        ['<prefix>/dali/<slave>/<addr>/config', 'min, max, power_on, sys_fail, fade_rate, fade_time. Only published once read from the gear.'],
        ['<prefix>/group/<slave>/<group>/state', 'Derived: on if ANY member is on, brightness = the MAXIMUM across members.'],
        ['<prefix>/group/<slave>/<group>/members/state', 'Current membership, read back from the ballasts.'],
        ['<prefix>/relay/<ch>/state', 'One relay channel.'],
        ['<prefix>/button/<fixture>/<button>', 'Wall button events: press, release, click, long_press.'],
        ['<prefix>/provision/<slave>/status', 'Commissioning progress and result.'],
        ['<prefix>/gateway/<slave>/diag', 'Gateway uptime, reboot count, reset reason.'],
        ['<prefix>/update/state', 'Installed and available firmware versions.'],
      ])),

    sec('Home Assistant',
      p('Nothing to configure in HA beyond pointing this device at the same broker. ' +
        'It publishes MQTT auto-discovery on startup and on every reconnect, so the ' +
        'entities appear on their own.'),
      p('Discovery is published to ', code('homeassistant/<component>/<object_id>/config'),
        '. The device also subscribes to ', code('homeassistant/status'),
        ' — HA\u2019s birth message — and re-publishes everything when HA restarts, so ' +
        'entities survive an HA restart without touching the controller.'),
      mut('Entities are created for the whole address space, not just what is present: ' +
          'all 64 short addresses per gateway, all 16 groups, all 32 relay channels. ' +
          'Ones that have not answered show as unavailable, so a ballast you plug in ' +
          'later simply becomes available rather than needing a re-discovery.'),
      table(['Entity', 'HA type · object id'], [
        ['Ballast', 'light · lighting_dali_<slave>_<addr>'],
        ['Ballast setting', 'number · lighting_dali_<slave>_<addr>_<field> (min, max, power_on, sys_fail, fade_rate, fade_time)'],
        ['Group', 'light · lighting_group_<slave>_<group>'],
        ['Group members', 'text · lighting_group_<slave>_<group>_members'],
        ['Relay channel', 'switch · lighting_relay_<ch>'],
        ['Wall button', 'event · press, release, click, long_press'],
        ['Button binding', 'text · lighting_binding_<fixture>_<button>'],
        ['Button stuck', 'binary_sensor · per plate'],
        ['Commissioning', 'sensor · lighting_prov_<slave>_state / _msg / _count, plus a target number'],
        ['Gateway diagnostics', 'sensor · lighting_gwdiag_<slave>_uptime / _reboots / _reset'],
        ['Firmware', 'update · lighting_fw_update, with a lighting_fw_check button'],
        ['Controller settings', 'number/switch · lighting_cfg_<key>'],
        ['Reboot', 'button · lighting_reboot'],
      ]),
      p(h('strong', {}, 'Devices in HA. '),
        'Entities are grouped under one device per physical thing: ',
        code('lighting_controller'), ' for the controller itself, ',
        code('lighting_dali_gw_<slave>'), ' for each DALI gateway (its ballasts, ' +
        'groups, commissioning and diagnostics hang off it), and one per wall plate.'),
      p(h('strong', {}, 'Availability is hierarchical. '),
        'A ballast is shown available only if the controller, its gateway AND the ' +
        'ballast itself are all online — HA is told to require all three. So pulling ' +
        'a gateway greys out everything behind it rather than leaving stale entities ' +
        'looking live.'),
      p(h('strong', {}, 'Toggling from HA. '),
        'HA resolves light.toggle itself from the state we publish and sends an ' +
        'explicit ON or OFF, so a toggle is only as correct as the reported state — ' +
        'which is why levels are read back from the gear rather than assumed.')),

    sec('REST API',
      mut('HTTP Basic auth on every route — set the password on the Password tab. ' +
          'Bodies are JSON. Reads are GET, everything else POST.'),
      table(['Route', 'Body / result'], [
        ['GET /api/state', 'Everything: relays, ballasts, groups, gateways, mappings, config.'],
        ['GET /api/ota', 'Installed vs available firmware, and install progress.'],
        ['GET /api/log', 'Recent device log.'],
        ['POST /api/dali', '{"slave":3,"addr":0,"on":true} · {"slave":3,"addr":0,"bri":128} · "on":"toggle"'],
        ['POST /api/group', '{"slave":3,"group":1,"on":true} · {"members":[0,45]}'],
        ['POST /api/relay', '{"ch":0,"on":true} · "on":"toggle"'],
        ['POST /api/scan', '{"slave":3} — re-interrogate a gateway now.'],
        ['POST /api/provision', '{"slave":3,"mode":"all"|"new"|"address","target":N} · {"abort":true}'],
        ['POST /api/cfg', 'Controller settings.'],
        ['POST /api/net', 'Network settings (applied on reboot).'],
        ['POST /api/mqtt', 'Broker settings (applied on reboot).'],
        ['POST /api/auth', 'Change the web password.'],
        ['POST /api/mapping', 'Button-to-target bindings.'],
        ['POST /api/ota/check', 'Re-read the firmware manifest.'],
        ['POST /api/ota/install', 'Download, verify and reboot into the new firmware.'],
        ['POST /api/reboot', 'Reboot the controller.'],
      ]),
      p('Example: ', code('curl -u admin:PASSWORD -X POST http://DEVICE/api/dali ' +
                          '-d \'{"slave":3,"addr":0,"on":"toggle"}\''))),

    sec('Concepts worth knowing',
      table(['Term', 'What it means'], [
        ['slave', 'A board on the RS-485 bus — a DALI gateway or a relay board — identified by its Modbus address.'],
        ['short address', 'A ballast’s DALI address, 0-63. Assigned by commissioning, not by wiring order.'],
        ['group', 'DALI groups 0-15 per bus. A ballast can be in several. There is no "create": you add members.'],
        ['commissioning', 'Assigning short addresses. "All" renumbers everything; "unaddressed" only touches new gear.'],
        ['fixture / button', 'A wall plate and one of its buttons, bound to targets on the Bindings tab.'],
        ['fade', 'The ballast ramps to a new level over its configured fade time. Intermediate levels are real readings and are published as they happen.'],
      ])),
  ];
}
