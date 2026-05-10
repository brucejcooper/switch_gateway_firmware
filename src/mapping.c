#include "mapping.h"
#include <stdio.h>
#include <string.h>

mapping_record_t g_mappings[MAX_FIXTURES][MAX_BUTTONS];

bool mapping_load(void) {
    memset(g_mappings, 0, sizeof(g_mappings));
    FILE *fp = fopen("/mappings.bin", "rb");
    if (!fp) return false;

    mapping_record_t rec;
    while (fread(&rec, sizeof(rec), 1, fp) == 1) {
        if (rec.fixture < MAX_FIXTURES && rec.button < MAX_BUTTONS) {
            g_mappings[rec.fixture][rec.button] = rec;
        }
    }
    fclose(fp);
    return true;
}

bool mapping_save(void) {
    FILE *fp = fopen("/mappings.bin", "wb");
    if (!fp) return false;

    bool ok = true;
    for (int f = 0; f < MAX_FIXTURES && ok; f++) {
        for (int b = 0; b < MAX_BUTTONS && ok; b++) {
            if (g_mappings[f][b].target_count == 0) continue;
            ok = fwrite(&g_mappings[f][b], sizeof(mapping_record_t), 1, fp) == 1;
        }
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

int mapping_serialise_all(char *buf, int buf_len) {
    int pos = 0;
    for (int f = 0; f < MAX_FIXTURES; f++) {
        for (int b = 0; b < MAX_BUTTONS; b++) {
            const mapping_record_t *r = &g_mappings[f][b];
            if (r->target_count == 0) continue;

            char targets[256];
            int  tp = snprintf(targets, sizeof(targets), "[");
            for (int t = 0; t < r->target_count; t++) {
                tp += snprintf(targets + tp, sizeof(targets) - tp,
                               "%s{\"slave\":%u,\"addr\":%u}",
                               t > 0 ? "," : "",
                               r->targets[t].slave, r->targets[t].addr);
            }
            tp += snprintf(targets + tp, sizeof(targets) - tp, "]");

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
