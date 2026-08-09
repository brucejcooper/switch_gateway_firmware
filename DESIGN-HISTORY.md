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

## The command queue was an id ledger, then a slot mirror

**v1** inferred a command's fate from accept/complete counters and a hashed
result ring, reconstructed after any disruption. Every recovery-corruption bug
was that reconstruction being subtly wrong.

**v2** replaced it with self-describing slots the master could read directly —
no ledger, no resync/rebuild/rollback. But FC23 makes the master *name* the
register it writes, so it could only submit into a slot it already knew was
free, and it learned that from a read. Every DALI frame cost a poll that
carried no work, then a submit: measured **1.73 Modbus transactions per command
and 99 % RS-485 occupancy** for one saturated gateway. The client also kept an
*optimistic mirror* of the gateway's slots, and reconciling that mirror against
reality after a reboot or a lost reply is where task 04 spent three fix cycles.

**v4** (task 08) removes both the poll and the mirror. The gateway chooses the
slot and retains each result until the master acks it, so one transaction acks
the previous result, collects the next and enqueues one command — **1.03
transactions per command, 37 % occupancy**, at unchanged throughput.
Correlation is a gateway-assigned sequence carried in the reply, so there is
nothing on the master to reconstruct: `mbq_client` holds no mirror, and
`resolve_ambiguous()` and the whole "idle means gone" inference are gone with
it. Recovery is five rules (R1/R1′/R2/R3/R4) applied to observations, not a
rebuild.

`docs/modbus-command-queue.md`, the standalone specification of v2's generic
slot pattern, was deleted in task 08 — its subject no longer exists. The
normative reference for both halves is now `../TinyModbus/QUEUE-PROTOCOL.md`.

### What the v4 bring-up cost, and the lesson

Three separate defects shipped a **clean soak verdict at full throughput**
while being badly wrong: the client polled instead of pacing (2.04
transactions per command — worse than the v2 it replaced); the pacing estimate
was anchored at reply-parse time rather than send time, costing 32 % of
throughput; and a gateway reboot cascaded into thousands of spurious reboot
declarations. **Correctness alone does not pass a queue change** — measure the
transaction count and the bus occupancy, or a regression hides in a green run.

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
