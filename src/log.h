#pragma once
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

typedef enum {
    LOG_LEVEL_OFF  = 0,
    LOG_LEVEL_ERR  = 1,
    LOG_LEVEL_WARN = 2,
    LOG_LEVEL_INFO = 3,   /* default */
    LOG_LEVEL_DBG  = 4,
} log_level_t;

extern log_level_t g_log_level;

void log_emit(log_level_t lvl, const char *level_char,
              const char *module, const char *fmt, ...);

#define LOG_ERR(module, ...)  log_emit(LOG_LEVEL_ERR,  "E", module, __VA_ARGS__)
#define LOG_WARN(module, ...) log_emit(LOG_LEVEL_WARN, "W", module, __VA_ARGS__)
#define LOG_INFO(module, ...) log_emit(LOG_LEVEL_INFO, "I", module, __VA_ARGS__)
#define LOG_DBG(module, ...)  log_emit(LOG_LEVEL_DBG,  "D", module, __VA_ARGS__)

bool log_level_parse(const char *s, log_level_t *out);
const char *log_level_name(log_level_t lvl);
