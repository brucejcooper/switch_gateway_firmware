#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>   /* size_t, for config_hostname_set() */

/* Slaves are addressed by their real Modbus address. The relay (Waveshare) and
 * DALI (TinyModbus) gateways are kept in separate config lists so the firmware
 * can dispatch operations to the right protocol via slave_kind(). Default
 * hardware: relay at slave 1, DALI gateways at slaves 2 and 3. */
#define MAX_RELAY_SLAVES 2
#define MAX_DALI_SLAVES  2

typedef struct {
    uint16_t long_press_ms;        /* default 500 */
    uint16_t dim_repeat_ms;        /* default 200 */
    uint16_t reversal_window_ms;   /* default 400 */
    /* How often the controller reads each DALI gateway's FC04 0x0310 sysinfo
     * block (uptime / boot count / reset reason) and publishes it to HA as
     * diagnostic sensors. Piggybacks on the existing dirty-poll loop. 0 = off. */
    uint32_t gw_diag_interval_ms;  /* default 30000 */
    /* Background DALI sweep: interrogate ONE ballast per slave this often.
     * 0 disables it. This is a backstop for changes nothing tells us about (a
     * ballast that lost power, gear changed outside this controller) — every
     * command the system issues is verified immediately regardless, so a slow
     * sweep costs nothing user-visible. Default 10000. */
    uint32_t dali_sweep_interval_ms;
    uint8_t  relay_slave_ids[MAX_RELAY_SLAVES];
    uint8_t  relay_slave_count;
    uint8_t  dali_slave_ids[MAX_DALI_SLAVES];
    uint8_t  dali_slave_count;

    /* Pending relay_slaves/dali_slaves rewrite, staged by `cfg relay_slaves=`/
     * `dali_slaves=` (CLI). These lists are persist-only, reboot-to-apply: every
     * consumer above binds slave_id -> array index once at boot (dali_state.c,
     * dirty_poll.c, dali_queue.c keep their own frozen slave_id/idx tables) while
     * state_cache_slave_idx() recomputes the index live from
     * relay_slave_ids/dali_slave_ids on every call. Writing a rewrite straight
     * into the live arrays made the frozen tables and the live lookup disagree
     * mid-session, so e.g. removing slave 2 from a {2,3} list re-numbered slave
     * 3 to index 0 and caused emit_state_dali to publish slave 2's stale cached
     * state under slave 3's name while dirty_poll kept polling the removed
     * slave. config_save() persists these pending values in place of the live
     * ones when a rewrite is pending; the live arrays only pick it up on the
     * next config_load() at boot. */
    bool     relay_slaves_pending;
    uint8_t  pending_relay_slave_ids[MAX_RELAY_SLAVES];
    uint8_t  pending_relay_slave_count;
    bool     dali_slaves_pending;
    uint8_t  pending_dali_slave_ids[MAX_DALI_SLAVES];
    uint8_t  pending_dali_slave_count;

    /* Modbus bus settings (migrated from /modbus_baud.txt + /modbus_config.txt). */
    uint32_t baud;                 /* default 19200 */
    uint8_t  de_mode;              /* modbus_de_mode_t, default 0 (when-tx) */

    /* Some Modbus relay boards (e.g. Waveshare Modbus RTU Relay 32CH) return the
     * FC01 coil-status data field with its bytes in REVERSED order (byte0 =
     * highest channels), contrary to the Modbus spec. When set, the firmware
     * reverses the coil-data bytes before decoding so relay read-back matches the
     * physical state. Default 0 (spec-compliant). */
    bool     relay_byteswap;       /* default 0 */

    /* MQTT broker. host is a dotted-quad IP (hostname/DNS is a follow-up).
     * Compile defaults are placeholders only — never commit secrets. */
    char     mqtt_host[64];
    uint16_t mqtt_port;            /* default 1883 */
    char     mqtt_user[48];
    char     mqtt_pass[64];
    char     mqtt_prefix[24];      /* default "lighting" */

    /* Network. dhcp default on; static fields used only when dhcp == 0. */
    uint8_t  net_dhcp;             /* default 1 */
    uint8_t  net_ip[4], net_mask[4], net_gw[4], net_dns[4];
    /* DHCP option-12 hostname (default "lights"). Sent in DISCOVER/REQUEST so the
     * router registers it in local DNS — e.g. reach the board as "lights". DNS
     * label charset (letters/digits/hyphen); reboot to apply. Always stored
     * lowercased — see config_hostname_set(). */
    char     net_hostname[32];     /* default "lights" */

    /* Embedded config web UI (core 1). Basic-auth credentials; default
     * admin/admin. Plain HTTP (no TLS on the W5100S) — LAN-only fallback. */
    char     web_user[32];         /* default "admin" */
    char     web_pass[48];         /* default "admin" */

    /* OTA manifest URL override. Empty = the compiled-in default
     * (FWUPDATE_MANIFEST_URL, fwupdate.c). Settable via CLI `cfg fw_url=…`
     * — lets a bench device poll a local HTTP server instead of the
     * production host, and production point at a staging server. */
    char     fw_url[128];
} global_config_t;

extern global_config_t g_config;

/* ── DHCP/DNS hostname validation ─────────────────────────────────────────
 * One RFC-1123 label: 1..31 chars, letters/digits/hyphen, not leading or
 * trailing a hyphen. Rejecting anything else keeps an illegal label off the
 * wire (some DHCP servers drop those silently).
 *
 * Lives here rather than in serial_protocol.c (where it started, private and
 * core-0-only) because all three write paths need the same rule: the CLI, the
 * web/API command handler on core 1, and config_load() itself — which used to
 * skip validation entirely and let anything in /config.json reach
 * netif_init(), and so HOST_NAME[] and the wire. Pure functions, no shared
 * state, safe to call from either core. */
bool config_hostname_ok(const char *s);

/* Validate `src` and write its LOWERCASED form into dst[cap]; returns false
 * and leaves dst untouched if `src` is not a valid label. Uppercase is legal
 * in a label (labels are case-insensitive, RFC 4343) but is normalised on the
 * way in so the stored value, the DHCP option-12 bytes and whatever the router
 * shows in local DNS are all the same string. */
bool config_hostname_set(char *dst, size_t cap, const char *src);

typedef enum {
    SLAVE_KIND_UNKNOWN = 0,
    SLAVE_KIND_RELAY,
    SLAVE_KIND_DALI,
} slave_kind_t;

/* Resolve a Modbus slave id to its protocol kind via the configured lists.
 * Returns SLAVE_KIND_UNKNOWN for slave 0 (broadcast) and unconfigured ids. */
slave_kind_t slave_kind(uint8_t slave_id);

/* Load from /config.json; fills defaults if absent or corrupt.
 * Uses stdio fopen() — works with pico-vfs. Accepts both the legacy
 * "relay_slave":N scalar and the new "relay_slaves":[...] array shape. */
bool config_load(void);

/* Persist current g_config to /config.json */
bool config_save(void);

void config_defaults(void);
