#include "mapping.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

mapping_record_t g_mappings[MAX_FIXTURES][MAX_BUTTONS];

static mapping_record_t s_txn_scratch[MAX_FIXTURES][MAX_BUTTONS];
static bool             s_txn_active;
static bool             s_txn_errored;

/* Mappings persist as line-delimited JSON: a version header line, then one
 * self-describing object per binding.  Explicit (de)serialisation keeps the
 * on-disk format decoupled from the in-memory struct — unlike the old raw
 * fwrite() of mapping_record_t, a layout change can no longer silently corrupt
 * the file, missing/extra fields are tolerated, and it's human-inspectable. */
#define MAPPINGS_PATH        "/mappings.json"
#define MAPPINGS_PATH_LEGACY "/mappings.bin"
#define MAPPINGS_VERSION     1

/* Serialise a record's targets as a JSON array (ballast → {slave,addr},
 * group → {slave,group}).  Shared by mapping_save and mapping_serialise_all. */
static int serialise_targets(char *buf, int size, const mapping_record_t *r) {
    int p = snprintf(buf, size, "[");
    for (int t = 0; t < r->target_count; t++) {
        if (TARGET_IS_GROUP(r->targets[t]))
            p += snprintf(buf + p, size - p, "%s{\"slave\":%u,\"group\":%u}",
                          t ? "," : "", r->targets[t].slave,
                          TARGET_GROUP_NUM(r->targets[t]));
        else
            p += snprintf(buf + p, size - p, "%s{\"slave\":%u,\"addr\":%u}",
                          t ? "," : "", r->targets[t].slave, r->targets[t].addr);
    }
    p += snprintf(buf + p, size - p, "]");
    return p;
}

/* Find "key":<int> in s (parsing our own well-formed output); def if absent. */
static long json_int_in(const char *s, const char *key, long def) {
    char needle[24];
    snprintf(needle, sizeof needle, "\"%s\"", key);
    const char *p = strstr(s, needle);
    if (!p) return def;
    p += strlen(needle);
    while (*p == ' ' || *p == ':') p++;
    return strtol(p, NULL, 10);
}

static uint8_t parse_targets_json(const char *line, mapping_target_t *out) {
    const char *p = strstr(line, "\"targets\"");
    if (!p || !(p = strchr(p, '['))) return 0;
    const char *arr_end = strchr(p, ']');
    uint8_t count = 0;
    while (count < MAX_TARGETS_PER_BTN && (p = strchr(p, '{'))) {
        if (arr_end && p > arr_end) break;
        const char *obj_end = strchr(p, '}');
        if (!obj_end) break;
        char obj[64];
        int len = (int)(obj_end - p) + 1;
        if (len >= (int)sizeof obj) len = (int)sizeof obj - 1;
        memcpy(obj, p, (size_t)len);
        obj[len] = '\0';
        long slave = json_int_in(obj, "slave", -1);
        long group = json_int_in(obj, "group", -1);
        long addr  = json_int_in(obj, "addr",  -1);
        if (slave >= 0) {
            out[count].slave = (uint8_t)slave;
            out[count].addr  = group >= 0 ? TARGET_MAKE_GROUP((uint8_t)group)
                             : addr  >= 0 ? (uint8_t)addr : 0u;
            count++;
        }
        p = obj_end + 1;
    }
    return count;
}

/* Legacy relay targets stored slave=0; rewrite to the first configured relay
 * slave id and persist if anything changed. */
static void migrate_relay_slave0(void) {
    if (g_config.relay_slave_count == 0) return;
    uint8_t real_relay = g_config.relay_slave_ids[0];
    bool changed = false;
    for (int f = 0; f < MAX_FIXTURES; f++)
        for (int b = 0; b < MAX_BUTTONS; b++) {
            mapping_record_t *r = &g_mappings[f][b];
            for (int t = 0; t < r->target_count; t++)
                if (r->targets[t].slave == 0) { r->targets[t].slave = real_relay; changed = true; }
        }
    if (changed) mapping_save();
}

/* One-time migration from the old raw-struct file (same struct, so a valid
 * legacy file still reads cleanly) → re-saved as JSON. */
static bool mapping_load_legacy(void) {
    FILE *fp = fopen(MAPPINGS_PATH_LEGACY, "rb");
    if (!fp) return false;
    mapping_record_t rec;
    bool any = false;
    while (fread(&rec, sizeof(rec), 1, fp) == 1)
        if (rec.fixture < MAX_FIXTURES && rec.button < MAX_BUTTONS &&
            rec.target_count <= MAX_TARGETS_PER_BTN) {
            g_mappings[rec.fixture][rec.button] = rec;
            any = true;
        }
    fclose(fp);
    if (any) mapping_save();   /* convert to JSON for next boot */
    return any;
}

bool mapping_load(void) {
    memset(g_mappings, 0, sizeof(g_mappings));
    FILE *fp = fopen(MAPPINGS_PATH, "r");
    if (!fp) {
        mapping_load_legacy();
    } else {
        char line[512];
        while (fgets(line, sizeof line, fp)) {
            if (!strstr(line, "\"fixture\"")) continue;   /* header / blank lines */
            long f = json_int_in(line, "fixture", -1);
            long b = json_int_in(line, "button", -1);
            if (f < 0 || f >= MAX_FIXTURES || b < 0 || b >= MAX_BUTTONS) continue;
            mapping_record_t *r = &g_mappings[f][b];
            r->fixture      = (uint8_t)f;
            r->button       = (uint8_t)b;
            r->target_count = parse_targets_json(line, r->targets);
        }
        fclose(fp);
    }
    migrate_relay_slave0();
    return true;
}

bool mapping_save(void) {
    FILE *fp = fopen(MAPPINGS_PATH, "w");
    if (!fp) return false;
    fprintf(fp, "{\"format\":\"mappings\",\"version\":%d}\n", MAPPINGS_VERSION);
    char tbuf[256];
    bool ok = true;
    for (int f = 0; f < MAX_FIXTURES && ok; f++)
        for (int b = 0; b < MAX_BUTTONS && ok; b++) {
            const mapping_record_t *r = &g_mappings[f][b];
            if (r->target_count == 0) continue;
            serialise_targets(tbuf, sizeof tbuf, r);
            if (fprintf(fp, "{\"fixture\":%u,\"button\":%u,\"targets\":%s}\n",
                        r->fixture, r->button, tbuf) < 0) ok = false;
        }
    fclose(fp);
    return ok;
}

const mapping_record_t *mapping_lookup(uint8_t fixture, uint8_t button) {
    if (fixture >= MAX_FIXTURES || button >= MAX_BUTTONS) return NULL;
    const mapping_record_t *r = &g_mappings[fixture][button];
    return r->target_count > 0 ? r : NULL;
}

bool mapping_set(uint8_t fixture, uint8_t button,
                 const mapping_target_t *targets, uint8_t count) {
    if (fixture >= MAX_FIXTURES || button >= MAX_BUTTONS) return false;
    if (count > MAX_TARGETS_PER_BTN) count = MAX_TARGETS_PER_BTN;
    mapping_record_t *r = &g_mappings[fixture][button];
    r->fixture      = fixture;
    r->button       = button;
    r->target_count = count;
    memcpy(r->targets, targets, count * sizeof(mapping_target_t));
    return mapping_save();
}

bool mapping_delete(uint8_t fixture, uint8_t button) {
    if (fixture >= MAX_FIXTURES || button >= MAX_BUTTONS) return false;
    memset(&g_mappings[fixture][button], 0, sizeof(mapping_record_t));
    return mapping_save();
}

/* ── Batch replace transaction ─────────────────────────────────────────── */

void mapping_txn_begin(void) {
    memset(s_txn_scratch, 0, sizeof(s_txn_scratch));
    s_txn_active  = true;
    s_txn_errored = false;
}

bool mapping_txn_add(uint8_t fixture, uint8_t button,
                     const mapping_target_t *targets, uint8_t count) {
    if (!s_txn_active) return false;
    if (fixture >= MAX_FIXTURES || button >= MAX_BUTTONS ||
        count > MAX_TARGETS_PER_BTN) {
        s_txn_errored = true;
        return false;
    }
    mapping_record_t *r = &s_txn_scratch[fixture][button];
    r->fixture      = fixture;
    r->button       = button;
    r->target_count = count;
    memcpy(r->targets, targets, count * sizeof(mapping_target_t));
    return true;
}

bool mapping_txn_commit(void) {
    if (!s_txn_active || s_txn_errored) return false;
    memcpy(g_mappings, s_txn_scratch, sizeof(g_mappings));
    s_txn_active = false;
    return mapping_save();
}

void mapping_txn_abort(void) {
    s_txn_active  = false;
    s_txn_errored = false;
}

bool mapping_txn_active(void)  { return s_txn_active; }
bool mapping_txn_errored(void) { return s_txn_errored; }

static bool targets_equal(const mapping_record_t *a, const mapping_record_t *b) {
    if (a->target_count != b->target_count) return false;
    for (int i = 0; i < a->target_count; i++) {
        if (a->targets[i].slave != b->targets[i].slave ||
            a->targets[i].addr  != b->targets[i].addr) return false;
    }
    return true;
}

void mapping_txn_for_each_change(mapping_diff_cb_t cb, void *user) {
    if (!s_txn_active) return;
    for (int f = 0; f < MAX_FIXTURES; f++) {
        for (int b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *old_r = &g_mappings[f][b];
            const mapping_record_t *new_r = &s_txn_scratch[f][b];
            if (targets_equal(old_r, new_r)) continue;
            cb((uint8_t)f, (uint8_t)b, new_r->targets, new_r->target_count, user);
        }
    }
}

int mapping_serialise_all(char *buf, int buf_len) {
    int pos = 0;
    for (int f = 0; f < MAX_FIXTURES; f++) {
        for (int b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *r = &g_mappings[f][b];
            if (r->target_count == 0) continue;

            char targets[256];
            serialise_targets(targets, sizeof(targets), r);

            int n = snprintf(buf + pos, buf_len - pos,
                             "{\"t\":\"config_mapping\","
                             "\"d\":{\"fixture\":%u,\"button\":%u},"
                             "\"v\":{\"targets\":%s}}\n",
                             r->fixture, r->button, targets);
            if (n < 0 || n >= buf_len - pos) return -1;
            pos += n;
        }
    }
    return pos;
}
