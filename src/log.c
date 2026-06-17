#include "log.h"
#include "cdc_out.h"
#include "pico/time.h"
#include <stdint.h>
#include <stdio.h>
#include <strings.h>

log_level_t g_log_level = LOG_LEVEL_INFO;

/* Longest single log line we emit. Lines longer than this are truncated (the
 * trailing newline is always preserved). The hex-dump debug lines are the
 * longest in practice and sit well under this. */
#define LOG_LINE_MAX 192

/* Format the whole line into a stack buffer and hand it to the non-blocking
 * CDC ring as ONE unit, so a line is never half-written and the calling core
 * never blocks on the USB host (the old per-call printf/vprintf/putchar both
 * blocked on a full FIFO and let two cores' lines interleave). */
void log_emit(log_level_t lvl, const char *level_char,
              const char *module, const char *fmt, ...) {
    if (lvl > g_log_level) return;
    char line[LOG_LINE_MAX];
    uint32_t ts = to_ms_since_boot(get_absolute_time());
    int n = snprintf(line, sizeof line, "# %lu %s %s: ",
                     (unsigned long)ts, level_char, module);
    if (n < 0) return;
    if (n > (int)sizeof line - 1) n = (int)sizeof line - 1;   /* header clamp */
    va_list ap;
    va_start(ap, fmt);
    int m = vsnprintf(line + n, (size_t)((int)sizeof line - n), fmt, ap);
    va_end(ap);
    if (m > 0) n += m;
    if (n > (int)sizeof line - 1) n = (int)sizeof line - 1;   /* leave room for '\n' */
    line[n++] = '\n';
    cdc_out_write(line, n);
}

const char *log_level_name(log_level_t lvl) {
    switch (lvl) {
    case LOG_LEVEL_OFF:  return "off";
    case LOG_LEVEL_ERR:  return "err";
    case LOG_LEVEL_WARN: return "warn";
    case LOG_LEVEL_INFO: return "info";
    case LOG_LEVEL_DBG:  return "dbg";
    }
    return "?";
}

bool log_level_parse(const char *s, log_level_t *out) {
    if (!s || !*s) return false;
    if (strcasecmp(s, "off")  == 0) { *out = LOG_LEVEL_OFF;  return true; }
    if (strcasecmp(s, "err")  == 0) { *out = LOG_LEVEL_ERR;  return true; }
    if (strcasecmp(s, "warn") == 0) { *out = LOG_LEVEL_WARN; return true; }
    if (strcasecmp(s, "info") == 0) { *out = LOG_LEVEL_INFO; return true; }
    if (strcasecmp(s, "dbg")  == 0 || strcasecmp(s, "debug") == 0) {
        *out = LOG_LEVEL_DBG;
        return true;
    }
    if (s[0] >= '0' && s[0] <= '4' && s[1] == '\0') {
        *out = (log_level_t)(s[0] - '0');
        return true;
    }
    return false;
}
