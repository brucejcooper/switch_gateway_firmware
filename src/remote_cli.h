#pragma once
/* remote_cli — the CLI, reachable over the network.
 *
 * One command interface: the same CLI dispatcher (cli.c) that serves USB-CDC
 * also serves HTTP (POST /api/cli {"cmd":"..."}) and MQTT (<prefix>/cli/cmd ->
 * <prefix>/cli/result). Same 39 commands, same output, wherever they are typed.
 *
 * The CLI must run on core 0 (it drives DALI/Modbus directly); HTTP and MQTT
 * live on core 1. This is a single-slot mailbox between them: core 1 posts one
 * command line, core 0 runs it with stdout captured into a buffer, core 1 reads
 * the output back and frees the slot. One command at a time — a second request
 * while one is in flight is refused (the caller answers "busy").
 *
 * Only SYNCHRONOUS output is captured: what the command prints before it
 * returns. A command that starts something asynchronous (`scan`, `provision`)
 * prints its acknowledgement; its later progress goes to the log (/api/log).
 */
#include <stdbool.h>
#include <stdint.h>

#define RCLI_OUT_MAX 8192u

typedef enum { RCLI_OWNER_HTTP = 1, RCLI_OWNER_MQTT = 2 } rcli_owner_t;

/* ── core 1 ── */

/* Post a command line. Returns false if the slot is busy or the line is empty
 * or too long. On success *ticket identifies the request. */
bool rcli_submit(rcli_owner_t owner, const char *line, uint32_t *ticket);

/* True once `ticket` has finished; `out` and `len` then give the captured output
 * (valid until rcli_release). *truncated is set if the output overflowed
 * RCLI_OUT_MAX. */
bool rcli_result(uint32_t ticket, const char **out, uint16_t *len, bool *truncated);

/* Done with `ticket` — finished or not. If it has not finished yet (the HTTP
 * client hung up, a timeout), the slot frees itself when core 0 completes it. */
void rcli_release(uint32_t ticket);

/* The ticket currently held by `owner`, or 0. Lets MQTT poll for its result. */
uint32_t rcli_ticket_of(rcli_owner_t owner);

/* ── core 0 ── */

void rcli_init(void);      /* claims a hardware spinlock; call before core 1 launch */
void rcli_service(void);   /* main loop: run a posted command, if any */
