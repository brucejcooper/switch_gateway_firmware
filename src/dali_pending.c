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

void dp_members(dp_table_t *t, uint8_t group, uint64_t add, uint64_t remove) {
    uint8_t g = group & 0x0Fu;
    bool was = (t->m_add[g] | t->m_rem[g]) != 0u;
    t->m_add[g] = (t->m_add[g] & ~remove) | add;
    t->m_rem[g] = (t->m_rem[g] & ~add) | remove;
    bool now = (t->m_add[g] | t->m_rem[g]) != 0u;
    if (!was && now) { t->n_dirty++; t->m_stamp[g] = t->next_stamp++; }
    for (uint64_t m = add | remove; m; m &= m - 1u) t->c_member_ops++;
}

void dp_note_intent(dp_table_t *t, uint8_t group, uint64_t members) {
    uint8_t g = group & 0x0Fu;
    t->m_intent[g] = members;
    t->m_intent_valid = (uint16_t)(t->m_intent_valid | (1u << g));
}

bool dp_intent(const dp_table_t *t, uint8_t group, uint64_t *members) {
    uint8_t g = group & 0x0Fu;
    if (!(t->m_intent_valid & (1u << g))) return false;
    *members = t->m_intent[g];
    return true;
}

static uint8_t lowest_bit(uint64_t m) {
    uint8_t b = 0;
    while (!(m & 1u)) { m >>= 1; b++; }
    return b;
}

bool dp_next(const dp_table_t *t, uint8_t *idx) {
    if (t->n_dirty == 0u) return false;
    bool found = false;
    uint32_t best = 0;
    for (uint8_t i = 0; i < DP_ALL; i++) {
        uint32_t stamp;
        if (i < DP_SLOTS) {
            if (!t->slot[i].dirty) continue;
            stamp = t->slot[i].stamp;
        } else {
            uint8_t g = (uint8_t)(i - DP_MEMBER_BASE);
            if (!(t->m_add[g] | t->m_rem[g])) continue;
            stamp = t->m_stamp[g];
        }
        if (!found || stamp < best) { best = stamp; *idx = i; found = true; }
    }
    return found;
}

void dp_member_frame(const dp_table_t *t, uint8_t idx, uint8_t *addr, uint8_t *data,
                     uint8_t *ballast) {
    uint8_t g = (uint8_t)(idx - DP_MEMBER_BASE);
    uint64_t pend = t->m_add[g] | t->m_rem[g];
    uint8_t  b    = pend ? lowest_bit(pend) : 0u;
    bool     add  = (t->m_add[g] >> b) & 1u;
    *ballast = b;
    *addr    = (uint8_t)(((b & 0x3Fu) << 1) | 0x01u);
    *data    = (uint8_t)((add ? 0x60u : 0x70u) | g);
}

void dp_take(dp_table_t *t, uint8_t idx) {
    if (idx >= DP_MEMBER_BASE) {
        uint8_t g = (uint8_t)(idx - DP_MEMBER_BASE);
        uint64_t pend = t->m_add[g] | t->m_rem[g];
        if (!pend) return;
        uint64_t bit = (uint64_t)1 << lowest_bit(pend);
        t->m_add[g] &= ~bit;
        t->m_rem[g] &= ~bit;
        if (!(t->m_add[g] | t->m_rem[g])) t->n_dirty--;
        t->c_shipped++;
        return;
    }
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
