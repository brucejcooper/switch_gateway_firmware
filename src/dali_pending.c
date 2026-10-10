/*
 * dali_pending.c — latest-wins desired-state table. See dali_pending.h.
 */
#include "dali_pending.h"
#include <string.h>

void dp_init(dp_table_t *t) {
    memset(t, 0, sizeof *t);
    t->next_stamp = 1u;
}

void dp_write(dp_table_t *t, bool is_group, uint8_t target, dp_op_t op, uint8_t arc) {
    dp_slot_t *s = &t->slot[dp_index(is_group, target)];
    if (s->dirty) t->c_overwritten++;
    else          t->n_dirty++;
    s->dirty = true;
    s->op    = (uint8_t)op;
    s->arc   = (op == DP_LEVEL) ? arc : 0u;
    s->stamp = t->next_stamp++;
    t->c_written++;
}

void dp_clear(dp_table_t *t, bool is_group, uint8_t target) {
    dp_slot_t *s = &t->slot[dp_index(is_group, target)];
    if (!s->dirty) return;
    s->dirty = false;
    t->n_dirty--;
}

bool dp_button(dp_table_t *t, bool is_group, uint8_t target, dp_op_t op, uint8_t arc) {
    bool own = t->slot[dp_index(is_group, target)].dirty;
    uint8_t others = (uint8_t)(t->n_dirty - (own ? 1u : 0u));
    if (others == 0u) {
        if (own) { dp_clear(t, is_group, target); t->c_button++; }
        return false;
    }
    /* Not counted as a written command: it is the button's, re-asserted. */
    dp_write(t, is_group, target, op, arc);
    t->c_written--;
    if (own) t->c_overwritten--;
    t->c_button++;
    return true;
}

bool dp_next(const dp_table_t *t, uint8_t *idx) {
    if (t->n_dirty == 0u) return false;
    bool found = false;
    uint32_t best = 0;
    for (uint8_t i = 0; i < DP_SLOTS; i++) {
        const dp_slot_t *s = &t->slot[i];
        if (!s->dirty) continue;
        if (!found || s->stamp < best) { best = s->stamp; *idx = i; found = true; }
    }
    return found;
}

void dp_take(dp_table_t *t, uint8_t idx) {
    dp_slot_t *s = &t->slot[idx];
    if (!s->dirty) return;
    s->dirty = false;
    t->n_dirty--;
    t->c_shipped++;
}

void dp_frame(uint8_t idx, const dp_slot_t *s, uint8_t *addr, uint8_t *data) {
    bool    grp = idx >= DP_BALLASTS;
    uint8_t n   = grp ? (uint8_t)(idx - DP_BALLASTS) : idx;
    /* DAPC form has bit 0 clear, command form has it set; a group address has
     * bit 7 set. */
    uint8_t dapc = grp ? (uint8_t)(0x80u | ((n & 0x0Fu) << 1))
                       : (uint8_t)((n & 0x3Fu) << 1);
    switch ((dp_op_t)s->op) {
    case DP_ON:
        *addr = (uint8_t)(dapc | 0x01u); *data = 0x0Au;   /* GOTO LAST ACTIVE LEVEL */
        break;
    case DP_LEVEL:
        *addr = dapc; *data = s->arc;
        break;
    case DP_OFF:
    default:
        if (grp) { *addr = (uint8_t)(dapc | 0x01u); *data = 0x00u; }  /* OFF command */
        else     { *addr = dapc;                    *data = 0x00u; }  /* DAPC 0 */
        break;
    }
}
