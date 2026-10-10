#pragma once
/* Host-test stand-in for the Pico SDK's pico/time.h — just enough for modules
 * whose only SDK use is a blocking deadline loop the tests never exercise
 * (dali_queue.c's dq_submit_sync). A fixed clock: such a loop would spin, so a
 * test must not call it. */
#include <stdint.h>
typedef uint64_t absolute_time_t;
static inline absolute_time_t get_absolute_time(void) { return 0; }
static inline absolute_time_t make_timeout_time_ms(uint32_t ms) { return (absolute_time_t)ms * 1000u; }
static inline int64_t absolute_time_diff_us(absolute_time_t from, absolute_time_t to) {
    return (int64_t)(to - from);
}
