/* savemig: versioned save files with a stepwise migration chain.
 *
 * File layout: header { magic, version, payload_len, crc32 } followed by the
 * version-specific fixed part and a variable-size blob. crc32 covers fixed
 * part + blob; a mismatch rejects the load before any migration runs.
 *
 * Loading an old save walks the migration chain one version at a time
 * (v1 -> v2 -> v3 -> v4). Each step is a pure struct-to-struct transform
 * that copies every payload byte O(1) times, so total migration work is
 * linear in save size. A missing chain step aborts with LOAD_ERR_MISSING_STEP
 * instead of guessing.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define SAVE_MAGIC 0x45564153u /* 'SAVE' */
#define SAVE_VERSION_CURRENT 4
#define NAME_CAP_V1 16
#define NAME_CAP 32
#define MAX_ITEMS 64
#define MAX_BLOB (256u << 20) /* sanity cap for corrupt headers */

/* ---- crc32 (zlib polynomial) ---- */

static uint32_t crc32_update(uint32_t crc, const void* data, size_t n) {
    static uint32_t table[256];
    static int ready = 0;
    if (!ready) {
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1;
            table[i] = c;
        }
        ready = 1;
    }
    const unsigned char* p = data;
    crc = ~crc;
    while (n--) crc = table[(crc ^ *p++) & 0xFF] ^ (crc >> 8);
    return ~crc;
}

/* ---- on-disk layouts, one per version (fixed part; blob follows) ---- */

typedef struct { uint32_t magic, version, payload_len, crc; } SaveHeader;

typedef struct {
    int32_t hp; uint32_t gold; char name[NAME_CAP_V1]; uint32_t blob_len;
} DiskV1;

typedef struct { /* v2: wider name, new level field */
    int32_t hp; uint32_t gold; uint32_t level; char name[NAME_CAP]; uint32_t blob_len;
} DiskV2;

typedef struct { /* v3: 64-bit gold, inventory */
    int32_t hp; uint64_t gold; uint32_t level; char name[NAME_CAP];
    uint32_t item_count; uint32_t items[MAX_ITEMS]; uint32_t blob_len;
} DiskV3;

typedef struct { /* v4 (current): difficulty + playtime */
    int32_t hp; uint64_t gold; uint32_t level; uint32_t difficulty;
    double playtime; char name[NAME_CAP];
    uint32_t item_count; uint32_t items[MAX_ITEMS]; uint32_t blob_len;
} DiskV4;

/* in-memory saves: fixed part + owned blob pointer */
typedef struct { DiskV1 d; unsigned char* blob; } SaveV1;
typedef struct { DiskV2 d; unsigned char* blob; } SaveV2;
typedef struct { DiskV3 d; unsigned char* blob; } SaveV3;
typedef struct { DiskV4 d; unsigned char* blob; } SaveV4;

typedef struct {
    int version;
    union { SaveV1 v1; SaveV2 v2; SaveV3 v3; SaveV4 v4; } u;
} AnySave;

static uint32_t fixed_size(uint32_t version) {
    switch (version) {
    case 1: return sizeof(DiskV1);
    case 2: return sizeof(DiskV2);
    case 3: return sizeof(DiskV3);
    case 4: return sizeof(DiskV4);
    default: return 0;
    }
}

static unsigned char** any_blob_slot(AnySave* s) {
    switch (s->version) {
    case 1: return &s->u.v1.blob;
    case 2: return &s->u.v2.blob;
    case 3: return &s->u.v3.blob;
    default: return &s->u.v4.blob;
    }
}

static uint32_t any_blob_len(const AnySave* s) {
    switch (s->version) {
    case 1: return s->u.v1.d.blob_len;
    case 2: return s->u.v2.d.blob_len;
    case 3: return s->u.v3.d.blob_len;
    default: return s->u.v4.d.blob_len;
    }
}

/* ---- migration steps: each is O(payload) and independently testable ---- */

static unsigned long long g_mig_bytes; /* bytes copied by migrations (linearity probe) */

static unsigned char* copy_blob(const unsigned char* src, uint32_t n) {
    unsigned char* dst = malloc(n ? n : 1);
    if (dst && n) memcpy(dst, src, n);
    return dst;
}

static int migrate_1_to_2(const void* in_, void* out_) {
    const SaveV1* in = in_;
    SaveV2* out = out_;
    memset(&out->d, 0, sizeof out->d);
    out->d.hp = in->d.hp;
    out->d.gold = in->d.gold;
    out->d.level = 1; /* new in v2: legacy saves start at level 1 */
    memcpy(out->d.name, in->d.name, NAME_CAP_V1);
    out->d.blob_len = in->d.blob_len;
    out->blob = copy_blob(in->blob, in->d.blob_len);
    if (!out->blob) return -1;
    g_mig_bytes += sizeof out->d + in->d.blob_len;
    return 0;
}

static int migrate_2_to_3(const void* in_, void* out_) {
    const SaveV2* in = in_;
    SaveV3* out = out_;
    memset(&out->d, 0, sizeof out->d);
    out->d.hp = in->d.hp;
    out->d.gold = in->d.gold; /* widened u32 -> u64 */
    out->d.level = in->d.level;
    memcpy(out->d.name, in->d.name, NAME_CAP);
    out->d.item_count = 0; /* new in v3: empty inventory */
    out->d.blob_len = in->d.blob_len;
    out->blob = copy_blob(in->blob, in->d.blob_len);
    if (!out->blob) return -1;
    g_mig_bytes += sizeof out->d + in->d.blob_len;
    return 0;
}

static int migrate_3_to_4(const void* in_, void* out_) {
    const SaveV3* in = in_;
    SaveV4* out = out_;
    memset(&out->d, 0, sizeof out->d);
    out->d.hp = in->d.hp;
    out->d.gold = in->d.gold;
    out->d.level = in->d.level;
    out->d.difficulty = 1; /* new in v4: normal */
    out->d.playtime = 0.0; /* new in v4 */
    memcpy(out->d.name, in->d.name, NAME_CAP);
    out->d.item_count = in->d.item_count;
    memcpy(out->d.items, in->d.items, sizeof in->d.items);
    out->d.blob_len = in->d.blob_len;
    out->blob = copy_blob(in->blob, in->d.blob_len);
    if (!out->blob) return -1;
    g_mig_bytes += sizeof out->d + in->d.blob_len;
    return 0;
}

/* ---- migration chain registry: step[v-1] migrates v -> v+1 ---- */

typedef int (*MigStep)(const void* in, void* out);

typedef struct { MigStep step[SAVE_VERSION_CURRENT - 1]; } Migrator;

static void migrator_default(Migrator* m) {
    m->step[0] = migrate_1_to_2;
    m->step[1] = migrate_2_to_3;
    m->step[2] = migrate_3_to_4;
}

/* ---- load / write ---- */

typedef enum {
    LOAD_OK = 0, LOAD_ERR_IO, LOAD_ERR_MAGIC, LOAD_ERR_VERSION,
    LOAD_ERR_CORRUPT, LOAD_ERR_CHECKSUM, LOAD_ERR_MISSING_STEP, LOAD_ERR_NOMEM
} LoadErr;

static const char* load_err_str(LoadErr e) {
    switch (e) {
    case LOAD_OK: return "ok";
    case LOAD_ERR_IO: return "io error";
    case LOAD_ERR_MAGIC: return "bad magic";
    case LOAD_ERR_VERSION: return "unsupported version";
    case LOAD_ERR_CORRUPT: return "corrupt payload";
    case LOAD_ERR_CHECKSUM: return "checksum mismatch";
    case LOAD_ERR_MISSING_STEP: return "missing migration step";
    case LOAD_ERR_NOMEM: return "out of memory";
    default: return "?";
    }
}

/* On success *out holds the current-version save; caller frees out->blob. */
static LoadErr save_load(const char* path, const Migrator* mig, SaveV4* out) {
    FILE* f = fopen(path, "rb");
    if (!f) return LOAD_ERR_IO;
    SaveHeader h;
    if (fread(&h, sizeof h, 1, f) != 1) { fclose(f); return LOAD_ERR_IO; }
    if (h.magic != SAVE_MAGIC) { fclose(f); return LOAD_ERR_MAGIC; }
    if (h.version < 1 || h.version > SAVE_VERSION_CURRENT) { fclose(f); return LOAD_ERR_VERSION; }
    uint32_t fixed = fixed_size(h.version);

    AnySave cur;
    memset(&cur, 0, sizeof cur);
    cur.version = (int)h.version;
    if (fread(&cur.u, fixed, 1, f) != 1) { fclose(f); return LOAD_ERR_CORRUPT; }
    uint32_t blen = any_blob_len(&cur);
    if (blen > MAX_BLOB || h.payload_len != fixed + blen) { fclose(f); return LOAD_ERR_CORRUPT; }

    unsigned char* blob = malloc(blen ? blen : 1);
    if (!blob) { fclose(f); return LOAD_ERR_NOMEM; }
    if (blen && fread(blob, blen, 1, f) != 1) { free(blob); fclose(f); return LOAD_ERR_CORRUPT; }
    fclose(f);

    uint32_t crc = crc32_update(0, &cur.u, fixed);
    crc = crc32_update(crc, blob, blen);
    if (crc != h.crc) { free(blob); return LOAD_ERR_CHECKSUM; }
    *any_blob_slot(&cur) = blob;

    while (cur.version < SAVE_VERSION_CURRENT) {
        MigStep s = mig ? mig->step[cur.version - 1] : NULL;
        if (!s) { free(*any_blob_slot(&cur)); return LOAD_ERR_MISSING_STEP; }
        AnySave next;
        memset(&next, 0, sizeof next);
        next.version = cur.version + 1;
        if (s(&cur.u, &next.u) != 0) { free(*any_blob_slot(&cur)); return LOAD_ERR_NOMEM; }
        free(*any_blob_slot(&cur));
        cur = next;
    }
    *out = cur.u.v4;
    return LOAD_OK;
}

/* Serialize `save` (SaveV1..SaveV4 matching `version`); used to write the
 * current version and by tests to fabricate old saves. */
static int save_write(const char* path, uint32_t version, const void* save) {
    uint32_t fixed = fixed_size(version);
    if (!fixed) return -1;
    const unsigned char* blob = NULL;
    uint32_t blen = 0;
    switch (version) {
    case 1: blen = ((const SaveV1*)save)->d.blob_len; blob = ((const SaveV1*)save)->blob; break;
    case 2: blen = ((const SaveV2*)save)->d.blob_len; blob = ((const SaveV2*)save)->blob; break;
    case 3: blen = ((const SaveV3*)save)->d.blob_len; blob = ((const SaveV3*)save)->blob; break;
    case 4: blen = ((const SaveV4*)save)->d.blob_len; blob = ((const SaveV4*)save)->blob; break;
    }
    SaveHeader h;
    h.magic = SAVE_MAGIC;
    h.version = version;
    h.payload_len = fixed + blen;
    h.crc = crc32_update(0, save, fixed);
    h.crc = crc32_update(h.crc, blob, blen);
    FILE* f = fopen(path, "wb");
    if (!f) return -1;
    int ok = fwrite(&h, sizeof h, 1, f) == 1
          && fwrite(save, fixed, 1, f) == 1
          && (blen == 0 || fwrite(blob, blen, 1, f) == 1);
    fclose(f);
    return ok ? 0 : -1;
}

/* ---- self tests ---- */

static int failures = 0;

static void check(const char* name, int ok) {
    if (!ok) { printf("FAIL %s\n", name); failures++; }
}

static void fill_blob(unsigned char* b, uint32_t n) {
    for (uint32_t i = 0; i < n; i++) b[i] = (unsigned char)(i * 31u + 7u);
}

static SaveV1 make_v1(uint32_t blob_len) {
    SaveV1 s;
    memset(&s.d, 0, sizeof s.d);
    s.d.hp = 85;
    s.d.gold = 123456789u;
    memcpy(s.d.name, "hero", 5);
    s.d.blob_len = blob_len;
    s.blob = malloc(blob_len ? blob_len : 1);
    fill_blob(s.blob, blob_len);
    return s;
}

/* the v4 a legacy v1 save must migrate into, field for field */
static SaveV4 make_v4_from_legacy(const SaveV1* in) {
    SaveV4 s;
    memset(&s.d, 0, sizeof s.d);
    s.d.hp = in->d.hp;
    s.d.gold = in->d.gold;
    s.d.level = 1;
    s.d.difficulty = 1;
    s.d.playtime = 0.0;
    memcpy(s.d.name, in->d.name, NAME_CAP_V1);
    s.d.item_count = 0;
    s.d.blob_len = in->d.blob_len;
    s.blob = copy_blob(in->blob, in->d.blob_len);
    return s;
}

static int v4_equal(const SaveV4* a, const SaveV4* b) {
    const DiskV4* x = &a->d;
    const DiskV4* y = &b->d;
    return x->hp == y->hp && x->gold == y->gold && x->level == y->level
        && x->difficulty == y->difficulty && x->playtime == y->playtime
        && memcmp(x->name, y->name, NAME_CAP) == 0
        && x->item_count == y->item_count
        && memcmp(x->items, y->items, sizeof(uint32_t) * x->item_count) == 0
        && x->blob_len == y->blob_len
        && (x->blob_len == 0 || memcmp(a->blob, b->blob, x->blob_len) == 0);
}

static void flip_byte(const char* path, long offset_from_end) {
    FILE* f = fopen(path, "r+b");
    if (!f) return;
    fseek(f, offset_from_end, SEEK_END);
    int c = fgetc(f);
    fseek(f, offset_from_end, SEEK_END);
    fputc(c ^ 0xFF, f);
    fclose(f);
}

#define TMP_SAVE "savemig_test.tmp"

int main(void) {
    Migrator mig;
    migrator_default(&mig);

    /* crc32 known vector */
    check("crc32_vector", crc32_update(0, "123456789", 9) == 0xCBF43926u);

    /* current-version roundtrip */
    {
        SaveV1 legacy = make_v1(1000);
        SaveV4 want = make_v4_from_legacy(&legacy);
        check("write_v4", save_write(TMP_SAVE, 4, &want) == 0);
        SaveV4 got;
        check("load_v4", save_load(TMP_SAVE, &mig, &got) == LOAD_OK);
        check("roundtrip_v4", v4_equal(&want, &got));
        free(got.blob);

        /* checksum: corrupt one payload byte -> reject */
        flip_byte(TMP_SAVE, -1);
        check("checksum_reject", save_load(TMP_SAVE, &mig, &got) == LOAD_ERR_CHECKSUM);

        /* bad magic -> reject */
        check("write_v4_again", save_write(TMP_SAVE, 4, &want) == 0);
        flip_byte(TMP_SAVE, -(long)(sizeof(SaveHeader) + want.d.blob_len + sizeof(DiskV4)));
        check("magic_reject", save_load(TMP_SAVE, &mig, &got) == LOAD_ERR_MAGIC);

        /* future version -> reject */
        check("write_v4_third", save_write(TMP_SAVE, 4, &want) == 0);
        FILE* f = fopen(TMP_SAVE, "r+b");
        uint32_t v = SAVE_VERSION_CURRENT + 1;
        fseek(f, 4, SEEK_SET);
        fwrite(&v, 4, 1, f);
        fclose(f);
        check("future_version_reject", save_load(TMP_SAVE, &mig, &got) == LOAD_ERR_VERSION);

        free(want.blob);
        free(legacy.blob);
    }

    /* each migration step individually */
    {
        SaveV1 v1 = make_v1(64);
        SaveV2 v2;
        check("step_1_to_2", migrate_1_to_2(&v1, &v2) == 0);
        check("step12_hp", v2.d.hp == v1.d.hp);
        check("step12_gold", v2.d.gold == v1.d.gold);
        check("step12_level_default", v2.d.level == 1);
        check("step12_name", memcmp(v2.d.name, v1.d.name, NAME_CAP_V1) == 0);
        check("step12_blob", memcmp(v2.blob, v1.blob, 64) == 0);

        SaveV3 v3;
        check("step_2_to_3", migrate_2_to_3(&v2, &v3) == 0);
        check("step23_gold_wide", v3.d.gold == (uint64_t)v2.d.gold);
        check("step23_level", v3.d.level == v2.d.level);
        check("step23_items_empty", v3.d.item_count == 0);
        check("step23_blob", memcmp(v3.blob, v2.blob, 64) == 0);

        v3.d.item_count = 2;
        v3.d.items[0] = 42; v3.d.items[1] = 77;
        SaveV4 v4;
        check("step_3_to_4", migrate_3_to_4(&v3, &v4) == 0);
        check("step34_difficulty_default", v4.d.difficulty == 1);
        check("step34_playtime_default", v4.d.playtime == 0.0);
        check("step34_items", v4.d.item_count == 2 && v4.d.items[0] == 42 && v4.d.items[1] == 77);
        check("step34_blob", memcmp(v4.blob, v3.blob, 64) == 0);

        free(v1.blob); free(v2.blob); free(v3.blob); free(v4.blob);
    }

    /* full chain v1 -> v4 must equal a directly written current-version save */
    {
        SaveV1 legacy = make_v1(5000);
        SaveV4 direct = make_v4_from_legacy(&legacy);
        check("write_v1", save_write(TMP_SAVE, 1, &legacy) == 0);
        SaveV4 migrated;
        check("load_v1_migrates", save_load(TMP_SAVE, &mig, &migrated) == LOAD_OK);
        check("chain_matches_direct", v4_equal(&migrated, &direct));
        free(migrated.blob);
        free(legacy.blob);
        free(direct.blob);
    }

    /* missing chain step -> explicit error, not a guess */
    {
        SaveV1 legacy = make_v1(16);
        SaveV2 v2;
        migrate_1_to_2(&legacy, &v2);
        check("write_v2", save_write(TMP_SAVE, 2, &v2) == 0);
        Migrator broken;
        migrator_default(&broken);
        broken.step[1] = NULL; /* v2 -> v3 missing */
        SaveV4 got;
        LoadErr err = save_load(TMP_SAVE, &broken, &got);
        check("missing_step_err", err == LOAD_ERR_MISSING_STEP);
        printf("missing_step_err=%s\n", load_err_str(err));
        broken.step[0] = NULL; /* v1 -> v2 also missing */
        check("write_v1_again", save_write(TMP_SAVE, 1, &legacy) == 0);
        check("missing_first_step_err", save_load(TMP_SAVE, &broken, &got) == LOAD_ERR_MISSING_STEP);
        /* current version needs no steps at all */
        SaveV4 cur;
        memset(&cur.d, 0, sizeof cur.d);
        cur.d.blob_len = 0;
        cur.blob = NULL;
        Migrator empty = {{ NULL, NULL, NULL }};
        check("write_v4_fourth", save_write(TMP_SAVE, 4, &cur) == 0);
        check("current_needs_no_chain", save_load(TMP_SAVE, &empty, &got) == LOAD_OK);
        free(got.blob);
        free(legacy.blob);
        free(v2.blob);
    }

    /* migration cost is linear in save size: bytes copied == 3 * (fixed + n),
     * no quadratic term; wall clock printed for the eyeball check */
    {
        SaveV1 a = make_v1(1u << 16);
        SaveV1 b = make_v1(1u << 18);
        unsigned long long expect_a =
            (unsigned long long)(sizeof(DiskV2) + sizeof(DiskV3) + sizeof(DiskV4)) + 3ull * a.d.blob_len;
        unsigned long long expect_b =
            (unsigned long long)(sizeof(DiskV2) + sizeof(DiskV3) + sizeof(DiskV4)) + 3ull * b.d.blob_len;

        g_mig_bytes = 0;
        SaveV2 t2; SaveV3 t3; SaveV4 t4;
        migrate_1_to_2(&a, &t2); migrate_2_to_3(&t2, &t3); migrate_3_to_4(&t3, &t4);
        unsigned long long bytes_a = g_mig_bytes;
        free(t2.blob); free(t3.blob); free(t4.blob);

        g_mig_bytes = 0;
        migrate_1_to_2(&b, &t2); migrate_2_to_3(&t2, &t3); migrate_3_to_4(&t3, &t4);
        unsigned long long bytes_b = g_mig_bytes;
        free(t2.blob); free(t3.blob); free(t4.blob);

        check("linear_exact_a", bytes_a == expect_a);
        check("linear_exact_b", bytes_b == expect_b);
        check("linear_no_quadratic", bytes_b - bytes_a == 3ull * (b.d.blob_len - a.d.blob_len));

        SaveV1 big = make_v1(16u << 20);
        clock_t t0 = clock();
        migrate_1_to_2(&big, &t2); migrate_2_to_3(&t2, &t3); migrate_3_to_4(&t3, &t4);
        double ms = 1000.0 * (clock() - t0) / CLOCKS_PER_SEC;
        printf("migrate_16mb=%.1fms\n", ms);
        free(big.blob); free(t2.blob); free(t3.blob); free(t4.blob);
        free(a.blob); free(b.blob);
    }

    remove(TMP_SAVE);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
