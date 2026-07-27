# Modbus command queue — a generic, self-describing slot protocol

A reliable, ordered command pipe from a master to a slave over Modbus RTU, with
**no payload knowledge**. It carries **opaque 32-bit commands** to the slave and
returns an **8-bit response + a lifecycle status** per command. It is used here to
drive a DALI gateway, but this document describes ONLY the generic pattern — the
meaning of the 32 command bits, of the response byte, and of any failure code is
defined entirely by the payload adapter layered on top (see the DALI adapter docs),
never here.

## Why self-describing slots

The predecessor inferred a command's fate from a running id-ledger (accept/complete
counters, a hashed result ring) that had to be *reconstructed* after any disruption
(slave reboot, transport glitch). Every recovery-corruption bug was that
reconstruction being subtly wrong. This protocol removes the class **by
construction**: each of the slave's slots directly reports its own state, so the
master reads a command's fate rather than inferring it. There is no ledger, no id
hashing, and no resync/rebuild/rollback.

## Slots and lifecycle

The slave exposes **8 slots**. Each slot is an independent, self-describing command
with a lifecycle:

```
0  idle       free; writable by the master
1  pending    accepted, waiting to execute
2  executing  in flight
3  complete   terminal; response8 = result value
4  failed     terminal; response8 = failure code
```

The **slave drives** `pending → executing → complete|failed`. The **master drives**
`idle|terminal → pending` by writing a command into the slot. No other transitions
exist. In particular:

> **Invariant (load-bearing): the slave returns a slot to `idle` ONLY at init /
> reboot, and does so for ALL slots together. It never individually clears one slot
> to idle.** Therefore any single slot that the master's cache believed active
> (pending/executing) or held an uncollected terminal, but reads back `idle`, is an
> unambiguous whole-slave reboot signal — it cannot be produced by any per-slot path.
> This is what makes "one blank ⇒ the slave rebooted" sound (see *Recovery*).

## Register map

Two blocks, 8 slots each. Addresses are the reference layout; a deployment may
relocate the bases but keeps the shape.

**Command block — master-write (FC16 / FC23-write range):**

| Reg | Meaning |
|---|---|
| `0x1000 + 2*slot` (2 regs) | the **opaque 32-bit command** for `slot`. The queue stores it verbatim and hands it to the payload executor unmodified. |

**Status block — slave-write (served under FC03 holding AND FC04 input at the same
addresses, so an FC23 read range can target it):**

| Reg | Layout |
|---|---|
| `0x1010 + slot` (1 reg) | `[status:8 | response8:8]` — `status` = lifecycle 0..4; `response8` = value on `complete`, code on `failed`, else 0. |

There are no aggregate counters and no shared result ring — each slot is complete on
its own.

## Ordering

Execution order is the **slave-assigned accept sequence**: when a slot goes
`idle → pending`, the slave stamps an internal monotonic sequence number; it executes
the lowest-sequence `pending` slot. The sequence is **wrap-aware** over the small live
window (≤ 8 outstanding), never placed on the wire — this keeps the 32-bit command
purely payload. For a single master transaction that writes several contiguous slots,
the slave stamps them in **ascending slot-index order**, so if the master allocates
slots in its intended order, accept order == submit order.

At a shallow in-flight depth (2–3) the accept order is simply submit order.

## Slave (server) rules

- **Execute** the lowest accept-sequence `pending` slot; wrap-aware compare.
- **Drive the lifecycle** and, on finishing, set `response8` then `status` (see the
  atomicity rule below).
- **Status write atomicity (load-bearing).** `[status | response8]` is one 16-bit
  register but a small MCU stores it as two bytes. A concurrent master read must
  NEVER observe `status = complete|failed` with a stale `response8`. Write
  `response8` **before** `status` (or write the whole 16-bit register) **under the
  same fence** the status-read path uses. This is the one ordering the whole protocol
  rests on — the analog of "publish the value before you publish that it's ready."
- **Reject a write to an ACTIVE slot.** A master write (FC16 or FC23-write) targeting
  a slot that is `pending` or `executing` is refused **all-or-nothing**: if ANY
  targeted slot is active, reject the WHOLE write with exception `0x06`
  (SLAVE_DEVICE_BUSY) and change nothing. `idle` and terminal (`complete`/`failed`)
  slots are writable; writing re-arms the slot to `pending` with a fresh sequence.
  This is defensive — a correct master avoids it via its cache, but the slave must
  never let a stale-cache write clobber an in-flight command.
- **Reboot** leaves all slots `idle` (init zeroes them; no extra work). The slave
  keeps no cross-reboot queue state.

## Master (client) rules

- **Optimistic slot cache.** The master keeps its own view of each slot, assumes the
  happy path, and verifies every read-back against it.
- **Submit = FC23 write+read, only when there is a new command.** Target an `idle`
  (or collected-terminal) slot: FC23 writes that slot's command and reads the status
  block back in one transaction; the read-back shows the slot `pending`. With no new
  command, a plain FC03/FC04 read polls.
- **Poll** each active slot every few ms until it is terminal; resolve the result
  from the slot's own `status` + `response8`. **Never** deliver anything but a slot's
  own result.
- **Deliver-once.** Track a "collected" flag per slot in the cache. A terminal slot is
  delivered to the caller exactly once; re-reading a still-terminal (not-yet-rewritten)
  slot must not double-deliver.
- **Reconcile from the 8 statuses on every read-back.** Any owned slot that the cache
  expected active or holding an uncollected terminal, but reads `idle`, is a blank ⇒
  by the invariant above, the slave rebooted ⇒ **fail the whole outstanding set**
  (deliver a synthesized *lost* failure) and **re-submit the idempotent ones**;
  non-idempotent commands are reported failed, never blind-resubmitted. Idempotency is
  a generic per-command attribute the caller sets when submitting — the queue does not
  interpret *why* a command is idempotent, only the flag.
- **ABA guard.** Because FC23 writes before it reads, only reuse a slot whose terminal
  result was collected in a PRIOR read — the "collected" flag enforces this, so a
  freshly-written slot is never mistaken for a stale terminal.

## Failure / loss model

- `failed` carries an opaque 8-bit code that only the payload adapter interprets.
- A **lost** command (the "one blank ⇒ reboot" case) is a failure **synthesized by
  the master**, not a slave-provided code — the slave that rebooted has no memory of
  it. The master surfaces it to the caller as a failure just like a slave-reported
  `failed`.
- Diagnostics such as a reboot counter may exist elsewhere on the slave (e.g. a
  sysinfo block) for monitoring, but the queue does **not** use them for recovery —
  recovery is driven solely by reading the slots.

## What is NOT in this layer (the payload adapter's job)

- The layout and meaning of the 32-bit command.
- The meaning of `response8` values and of `failed` codes.
- Which commands are idempotent (the adapter sets the per-command flag).
- Any multi-frame / indivisible operation must be packed into a **single slot's**
  32-bit command so it runs as one lifecycle — the queue must never require two slots
  to be executed adjacently, because the accept-sequence and reject-active rules make
  no adjacency guarantee across slots.
