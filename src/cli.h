#pragma once

/*
 * cli.h — public interface to the human USB-CDC text CLI (cli.c).
 *
 * See cli.c's own file-top comment for the split rationale (this used to be
 * one third of serial_protocol.c). ic_seam.h is the other half.
 */

#include <stdbool.h>
#include "intercore.h"   /* ic_cmd_t — for cli_ping_on_reply() below */

/* Initialise the CLI (nothing to do — stdio_usb handles CDC init) */
void cli_init(void);

/* Call from the main loop — reads USB CDC input, echoes/line-edits it, and
 * dispatches complete lines through the cli_commands[] verb table. */
void cli_poll(void);

#define CLI_LINE_MAX 256

/* Run one command line through the same dispatcher USB uses. Output goes to
 * stdout. Core 0 only — remote_cli.c calls it with stdout captured. */
void cli_exec_line(const char *line);

/* ── Functions ic_seam.c reaches into cli.c for ─────────────────────────────
 * Each of these is a CLI feature at heart (the verb that starts it lives in
 * cli.c's dispatch table) that the intercore seam also drives, either because
 * a remote peer can trigger the same action (reboot) or because the seam is
 * how the asynchronous answer arrives (a `net ping` reply travels core 1 ->
 * core 0 as an IC_CMD_PING_REPLY). Keeping the state and console-printing
 * logic in one place (here) beat a second copy in ic_seam.c. */

/* Shared reboot path: the local `reboot` CLI verb and a remote IC_CMD_REBOOT
 * (MQTT/web, dispatched by ic_seam.c) both land here. `src` names the trigger
 * for the log. Never returns. */
void controller_reboot(const char *src);

/* Feed one IC_CMD_PING_REPLY (drained by ic_seam.c's ic_dispatch_cmd) into
 * the running `net ping` request and print the outcome. */
void cli_ping_on_reply(const ic_cmd_t *c);

/* Bail out of a stuck `net ping` run if core 1 never replies. Call once per
 * ic_seam_pump() tick, AFTER the intercore command drain — a reply landing
 * this tick must update the ping run (via cli_ping_on_reply above) before
 * this deadline check runs, or a reply that already arrived this tick could
 * be reported as a timeout. */
void cli_ping_poll(void);
