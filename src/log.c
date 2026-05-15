#include "log.h"
#include "pico/time.h"
#include <stdint.h>
#include <strings.h>

log_level_t g_log_level = LOG_LEVEL_INFO;

void log_emit(log_level_t lvl, const char *level_char,
              const char *module, const char *fmt, ...) {
    if (lvl > g_log_level) return;
    uint32_t ts = to_ms_since_boot(get_absolute_time());
    printf("# %lu %s %s: ", (unsigned long)ts, level_char, module);
    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);
    putchar('\n');
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
