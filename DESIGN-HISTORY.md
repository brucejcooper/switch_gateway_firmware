# Controller — design history

Everything the controller **used to be**, kept here so the source and README can
describe only what it **is**.

Nothing in this file is current. It exists so a surprising design choice can be
traced to the thing it replaced. If it contradicts the code, the code wins.

---

## The gateway used to hold the bus state

The DALI gateway once cached per-ballast levels, presence and dirty bitmaps,
group masks, and ran commissioning itself. The controller read that cache over
Modbus register banks.

A gateway reboot lost the cache, and anything derived from it — most visibly
group membership, which made a group toggle behave as though every light were
already off.

**Now:** the controller owns all of it and reads the gear directly. The gateway
is a pipe for opaque commands.

| Was read from the gateway | Now |
|---|---|
| BLOCK0 `0x0000` arc/status | Raw DALI queries (`dali_state.c`) |
| BLOCK1-3 config banks `0x0040`/`0x0080`/`0x00C0` | Raw DTR0+SET pairs and QUERY reads |
| Presence `0x0100`, dirty `0x0104` | Controller's own presence model |
| Group masks `0x0110+` | Controller diffs and issues ADD/REMOVE FROM GROUP |
| Commission register `0x1F04` | `dali_commission.c` runs the search |
| `SCAN_TRIGGER` `0x1F00`, `RESCAN_INTERVAL` `0x1F01` | Controller's sweep |

`0x1F02`/`0x1F03` (slave address, baud index) were relocated from `0x1012`/`0x1013`.

## The command queue was an id ledger

v1 inferred a command's fate from accept/complete counters and a hashed result
ring, reconstructed after any disruption. Every recovery-corruption bug was that
reconstruction being subtly wrong. Replaced by self-describing slots: the master
reads a command's fate rather than inferring it. No ledger, no
resync/rebuild/rollback.

## Predicted state, and why it all went

The controller used to write what it had *asked for* into its cache and publish
it as fact:

- **Brightness.** A set wrote the requested level and emitted it. Ballasts clamp
  to their own min/max and fade there, so asking for 5 on a ballast with min 85
  showed 5, then jumped to 85.
- **On/off.** An OFF wrote `arc = 0` before the lamp had finished fading down.
- **Relays.** The CLI and intercore paths wrote the requested state; a *failed*
  write left it there until a coil poll disagreed, up to 2 s later.
- **Group membership.** `dali_group_set_members` recorded the requested mask;
  a write that silently failed sat in the cache and in HA.
- **Group on/off.** Emitted group state computed from *pre-command* member
  levels.
- **`commanded_on`.** A per-ballast "intent" flag, plus a 0.7 s settle window
  during which a real read was not allowed to overwrite it. It existed so a
  rapid second toggle read intent rather than a stale level.

**Now:** nothing predicts. Every published value comes from a read. All three
toggle paths ask the gear (`QUERY LAMP POWER ON`) instead of consulting a cache,
which is what made `commanded_on` unnecessary.

### Retained-config warm start

`IC_CMD_SEED_DALI_CFG` subscribed to our own retained `dali/+/+/config` topic and
adopted it at boot, so HA's config numbers were populated before the rotation
read them. That meant ingesting our own output as truth — a ballast swapped
while we were off came back with its old config. Removed; the boot config sweep
reads the real values instead.

## High-water-mark truncation

Three arrays were cut at the highest index ever seen, so they could express
"nothing here yet" but never "nothing here":

- **Relays** — truncated at the highest channel reported, and channels below it
  that had never been read rendered `false`.
- **Groups** — `group_max()` stopped at the highest group seen, so a bus using
  G0/G1/G7/G9 served exactly ten entries and groups 10-15 were unreachable.
  Since DALI has no "create group", an unlisted group could not be built.
- **Ballasts** — `ballast_max()` listed only addresses that had answered.

## Optimistic availability

Gateways were seeded `online` at discovery and `reachable = true` at init, then
corrected later. Because ballast entities declare `availability_mode: all` over
[controller, gateway, ballast], one optimistic gateway made everything under it
look available with whatever state was retained from the previous run.

## Hardcoded discovery ids

Discovery object_ids, unique_ids and device identifiers were literal
`lighting_...` while topics already followed `mqtt_app_prefix()`. Two controllers
could be given separate topic namespaces and still collide in
`homeassistant/.../lighting_dali_3_0/config`.

## Things that were silently wrong

- **CLI printed nothing on success.** `cmd_dali`/`cmd_relay`/`cmd_cfg` emitted to
  core 1 for MQTT and returned — a hangover from when a Python bridge consumed
  the CLI's JSON off the serial wire.
- **`dali_state.c` picked between two priorities that were the same class**, under
  a comment promising verification reads outranked the sweep.
- **HA group toggle meant OFF** — `IC_TRI_TOGGLE` compared unequal to `IC_TRI_ON`.
- **Firmware tab could never load** — its first fetch ran before the card was
  mounted and hit an `isConnected` guard.
- **Provisioning state labels in the UI** were the gateway's old 4-value status
  word, so a run at state 2 ("scanning") showed a green "complete".
- **Incremental provisioning reported an error** when it correctly found nothing
  to do on a fully-addressed bus.
- **Watchdog covered only the main loop.** `main()` disarmed the bootloader's
  watchdog first thing and armed its own at main-loop entry, leaving boot
  unprotected.

## Blocking operations made async

The boot scan and the `scan` CLI verb both blocked until the interrogation
finished — 53 s with an absent gateway, and on a live system the buttons stopped
responding for the duration.

## Lua bindings

Button-to-target bindings were once Lua coroutines. Replaced by the typed
mapping table.
