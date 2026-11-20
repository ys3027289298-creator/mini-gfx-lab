
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold.

   Two build modes:
   - default (LOD_ENABLE_LOAD defined): on-disk variant load with structural
     validation, per-level diagnostics and fallback to the nearest usable
     level. The selection math itself is unchanged.
   - -ULOD_ENABLE_LOAD: no load path at all; the program is the original
     runtime-computed selector, byte for byte. */
#include <stdio.h>
#include <math.h>

#define LEVELS 4

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
} MeshLod;

typedef struct {
    int level;
    int switches;
    int thrash;            /* switches within one unit of distance change */
} State;

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* pick coarsest level whose projected error is below tolerance */
static int select_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                        float tolerance_px, State* st) {
    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    int prev = st->level;
    if (want != prev) {
        /* hysteresis: require the candidate to beat the threshold by tau */
        float e_new = screen_error(m->error[want], distance, fov_y, screen_h);
        float e_old = screen_error(m->error[prev], distance, fov_y, screen_h);
        if (want > prev && e_new > tolerance_px * m->tau) return prev;
        if (want < prev && e_old < tolerance_px / m->tau) return prev;
        st->switches++;
        st->level = want;
    }
    return st->level;
}

#ifdef LOD_ENABLE_LOAD

#include <stdint.h>
#include <string.h>
#include <stdlib.h>

/* ---- on-disk format (little endian, fixed layout) -----------------------
   header (52 bytes):
     u32 magic "LOD1", u32 version, u32 levels, u32 reserved
     f32 radius, tau
     f32 bbox_min[3], bbox_max[3]            mesh-wide bounds
     u32 header_crc                          CRC32 over the preceding 48 B
   per level (36 bytes):
     u32 face_count
     f32 error, bbox_min[3], bbox_max[3]
     u32 record_crc                          CRC32 over the preceding 32 B */
#define LOD_MAGIC   0x31444f4cu            /* "LOD1" */
#define LOD_VERSION 1u
#define HDR_SIZE    52u
#define LVL_SIZE    36u
#define LOD_FILE_SIZE (HDR_SIZE + (unsigned)LEVELS * LVL_SIZE)

typedef struct {
    uint32_t face_count;
    float error;
    float bbox_min[3];
    float bbox_max[3];
} LodLevelMeta;

typedef enum {
    LOD_OK = 0,
    LOD_E_IO,              /* cannot open/read/write */
    LOD_E_TRUNCATED,       /* file shorter than the fixed layout */
    LOD_E_SIZE,            /* file longer than the fixed layout */
    LOD_E_MAGIC,
    LOD_E_VERSION,
    LOD_E_CHECKSUM,        /* CRC mismatch; value looked otherwise legal */
    LOD_E_NOT_FINITE,      /* NaN/Inf in a float field */
    LOD_E_RANGE,           /* scalar outside its legal range */
    LOD_E_FACE_COUNT,      /* face counts not monotonically non-increasing */
    LOD_E_BBOX,            /* min > max, or level bbox outside mesh bbox */
    LOD_E_NO_FALLBACK      /* level 0 unusable: nothing to fall back to */
} LodError;

typedef struct {
    MeshLod base;
    LodLevelMeta meta[LEVELS];
    int valid[LEVELS];            /* 1 if the level passed every check */
    int header_valid;             /* magic/version/bounds/header CRC all good */
    LodError load_error;          /* first error seen; LOD_OK if clean */
    int error_level;              /* -1 for header/file errors */
    char error_field[48];         /* e.g. "level[2].face_count" */
} LodAsset;

/* ---- little-endian codec ------------------------------------------------ */

static void put_u32(unsigned char* p, uint32_t v) {
    p[0] = (unsigned char)(v);
    p[1] = (unsigned char)(v >> 8);
    p[2] = (unsigned char)(v >> 16);
    p[3] = (unsigned char)(v >> 24);
}

static uint32_t get_u32(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void put_f32(unsigned char* p, float f) {
    uint32_t bits;
    memcpy(&bits, &f, 4);
    put_u32(p, bits);
}

static float get_f32(const unsigned char* p) {
    uint32_t bits = get_u32(p);
    float f;
    memcpy(&f, &bits, 4);
    return f;
}

static uint32_t f32_bits(float f) {
    uint32_t bits;
    memcpy(&bits, &f, 4);
    return bits;
}

static int f32_is_finite(float f) {
    uint32_t bits = f32_bits(f);
    return ((bits >> 23) & 0xffu) != 0xffu;   /* exponent all ones = NaN/Inf */
}

/* CRC32/IEEE 802.3, incremental so header and records share one routine */
static uint32_t crc32_ieee(const unsigned char* p, size_t n, uint32_t crc) {
    crc = ~crc;
    for (size_t i = 0; i < n; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xedb88320u & (uint32_t)(-(int32_t)(crc & 1u)));
    }
    return ~crc;
}

static void lod_set_error(LodAsset* a, LodError e, int level, const char* field) {
    if (a->load_error == LOD_OK) {                 /* keep the first failure */
        a->load_error = e;
        a->error_level = level;
        if (level < 0) snprintf(a->error_field, sizeof(a->error_field), "%s", field);
        else snprintf(a->error_field, sizeof(a->error_field), "level[%d].%s", level, field);
    }
}

/* Reference serializer: bakes runtime-computed variants into the canonical
   byte layout. Returns LOD_OK or LOD_E_IO. */
static LodError lod_save(const char* path, const MeshLod* m,
                         const LodLevelMeta meta[LEVELS]) {
    unsigned char* buf = (unsigned char*)calloc(LOD_FILE_SIZE, 1);
    if (!buf) return LOD_E_IO;

    put_u32(buf + 0, LOD_MAGIC);
    put_u32(buf + 4, LOD_VERSION);
    put_u32(buf + 8, (uint32_t)LEVELS);
    put_u32(buf + 12, 0);
    put_f32(buf + 16, m->radius);
    put_f32(buf + 20, m->tau);
    for (int k = 0; k < 3; k++) {
        put_f32(buf + 24 + 4 * k, meta[0].bbox_min[k]);
        put_f32(buf + 36 + 4 * k, meta[0].bbox_max[k]);
    }
    put_u32(buf + 48, crc32_ieee(buf, 48, 0));

    for (int i = 0; i < LEVELS; i++) {
        unsigned char* r = buf + HDR_SIZE + (unsigned)i * LVL_SIZE;
        put_u32(r + 0, meta[i].face_count);
        put_f32(r + 4, m->error[i]);
        for (int k = 0; k < 3; k++) {
            put_f32(r + 8 + 4 * k, meta[i].bbox_min[k]);
            put_f32(r + 20 + 4 * k, meta[i].bbox_max[k]);
        }
        put_u32(r + 32, crc32_ieee(r, 32, 0));
    }

    FILE* f = fopen(path, "wb");
    if (!f) { free(buf); return LOD_E_IO; }
    size_t w = fwrite(buf, 1, LOD_FILE_SIZE, f);
    int close_rc = fclose(f);
    free(buf);
    return (w == LOD_FILE_SIZE && close_rc == 0) ? LOD_OK : LOD_E_IO;
}

static unsigned char* read_whole_file(const char* path, size_t* out_n) {
    *out_n = 0;
    FILE* f = fopen(path, "rb");
    if (!f) return NULL;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long sz = ftell(f);
    if (sz < 0 || fseek(f, 0, SEEK_SET) != 0) { fclose(f); return NULL; }
    unsigned char* buf = (unsigned char*)malloc((size_t)sz + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t n = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (n != (size_t)sz) { free(buf); return NULL; }
    *out_n = n;
    return buf;
}

/* Load and validate. On a per-level failure the level is marked invalid and
   validation continues, so callers can fall back to coarser levels; the
   first error code is returned. Only an unusable level 0 yields
   LOD_E_NO_FALLBACK. */
static LodError lod_load(LodAsset* a, const char* path) {
    memset(a, 0, sizeof(*a));
    a->load_error = LOD_OK;
    a->error_level = -1;
    for (int i = 0; i < LEVELS; i++) a->valid[i] = 0;

    size_t n = 0;
    unsigned char* buf = read_whole_file(path, &n);
    if (!buf) { lod_set_error(a, LOD_E_IO, -1, "file"); return LOD_E_IO; }
    if (n != LOD_FILE_SIZE) {
        LodError e = (n < LOD_FILE_SIZE) ? LOD_E_TRUNCATED : LOD_E_SIZE;
        lod_set_error(a, e, -1, "file_size");
        free(buf);
        return e;
    }

    /* ---- header: structural checks first, checksum last ---------------- */
    int header_ok = 1;
    if (get_u32(buf + 0) != LOD_MAGIC) {
        lod_set_error(a, LOD_E_MAGIC, -1, "header.magic");
        header_ok = 0;
    }
    if (get_u32(buf + 4) != LOD_VERSION) {
        lod_set_error(a, LOD_E_VERSION, -1, "header.version");
        header_ok = 0;
    }
    if (get_u32(buf + 8) != (uint32_t)LEVELS) {
        lod_set_error(a, LOD_E_RANGE, -1, "header.levels");
        header_ok = 0;
    }
    if (get_u32(buf + 12) != 0u) {
        lod_set_error(a, LOD_E_RANGE, -1, "header.reserved");
        header_ok = 0;
    }

    float hdr_radius = get_f32(buf + 16);
    float hdr_tau = get_f32(buf + 20);
    float hdr_min[3], hdr_max[3];
    for (int k = 0; k < 3; k++) {
        hdr_min[k] = get_f32(buf + 24 + 4 * k);
        hdr_max[k] = get_f32(buf + 36 + 4 * k);
    }
    if (!f32_is_finite(hdr_radius) || !f32_is_finite(hdr_tau) ||
        !f32_is_finite(hdr_min[0]) || !f32_is_finite(hdr_min[1]) ||
        !f32_is_finite(hdr_min[2]) || !f32_is_finite(hdr_max[0]) ||
        !f32_is_finite(hdr_max[1]) || !f32_is_finite(hdr_max[2])) {
        lod_set_error(a, LOD_E_NOT_FINITE, -1, "header");
        header_ok = 0;
    } else if (hdr_radius <= 0.0f || hdr_tau <= 1.0f) {
        lod_set_error(a, LOD_E_RANGE, -1, "header.radius_or_tau");
        header_ok = 0;
    } else {
        for (int k = 0; k < 3; k++)
            if (!(hdr_min[k] <= hdr_max[k])) {
                lod_set_error(a, LOD_E_BBOX, -1, "header.bbox");
                header_ok = 0;
                break;
            }
    }
    if (crc32_ieee(buf, 48, 0) != get_u32(buf + 48)) {
        lod_set_error(a, LOD_E_CHECKSUM, -1, "header.checksum");
        header_ok = 0;
    }

    /* header fields are usable even if some per-level records fail: a
       degraded asset must still select with the real radius/tau */
    if (header_ok) {
        a->base.radius = hdr_radius;
        a->base.tau = hdr_tau;
    }
    a->header_valid = header_ok;

    /* Even with a bad header the per-level pass runs so valid[] is complete
       and the diagnostics point at every independently broken record. */
    for (int i = 0; i < LEVELS; i++) {
        const unsigned char* r = buf + HDR_SIZE + (unsigned)i * LVL_SIZE;
        LodLevelMeta* lv = &a->meta[i];
        int level_ok = 1;

        lv->face_count = get_u32(r + 0);
        lv->error = get_f32(r + 4);
        for (int k = 0; k < 3; k++) {
            lv->bbox_min[k] = get_f32(r + 8 + 4 * k);
            lv->bbox_max[k] = get_f32(r + 20 + 4 * k);
        }
        a->base.error[i] = lv->error;

        /* semantic checks first so a broken field reports its own name;
           the CRC then catches corruption that happens to stay legal */
        if (!f32_is_finite(lv->error)) {
            lod_set_error(a, LOD_E_NOT_FINITE, i, "error");
            level_ok = 0;
        }
        for (int k = 0; k < 3; k++) {
            if (!f32_is_finite(lv->bbox_min[k]) || !f32_is_finite(lv->bbox_max[k])) {
                lod_set_error(a, LOD_E_NOT_FINITE, i, "bbox");
                level_ok = 0;
            }
        }
        if (lv->face_count == 0) {
            lod_set_error(a, LOD_E_RANGE, i, "face_count");
            level_ok = 0;
        }
        if (i > 0 && lv->face_count > a->meta[i - 1].face_count) {
            lod_set_error(a, LOD_E_FACE_COUNT, i, "face_count");
            level_ok = 0;
        }
        if (i > 0 && f32_is_finite(lv->error) && f32_is_finite(a->base.error[i - 1]) &&
            lv->error < a->base.error[i - 1]) {
            lod_set_error(a, LOD_E_RANGE, i, "error");
            level_ok = 0;
        }
        for (int k = 0; k < 3 && f32_is_finite(lv->bbox_min[k]) &&
                           f32_is_finite(lv->bbox_max[k]); k++) {
            if (!(lv->bbox_min[k] <= lv->bbox_max[k])) {
                lod_set_error(a, LOD_E_BBOX, i, "bbox_max");
                level_ok = 0;
                break;
            }
            if (f32_is_finite(hdr_min[k]) && f32_is_finite(hdr_max[k]) &&
                (lv->bbox_min[k] < hdr_min[k] || lv->bbox_max[k] > hdr_max[k])) {
                lod_set_error(a, LOD_E_BBOX, i, "bbox_outside_mesh_bounds");
                level_ok = 0;
                break;
            }
        }
        if (crc32_ieee(r, 32, 0) != get_u32(r + 32)) {
            lod_set_error(a, LOD_E_CHECKSUM, i, "checksum");
            level_ok = 0;
        }
        a->valid[i] = level_ok;
    }

    free(buf);

    /* A broken header means the records cannot be trusted as a set: force
       total rejection rather than handing selectors a half-meaningful mesh.
       Keep the header's own diagnostic; only pin the level coordinate. */
    if (!header_ok) {
        for (int i = 0; i < LEVELS; i++) a->valid[i] = 0;
    }
    if (!a->valid[0]) {
        if (a->error_level < 0) a->error_level = 0;
        return LOD_E_NO_FALLBACK;
    }
    return a->load_error;
}

/* Selection over a loaded asset: identical selector math, then walk down to
   the nearest valid level. Level 0 is guaranteed valid when load returned
   anything other than LOD_E_NO_FALLBACK. */
static int asset_select_level(const LodAsset* a, float distance, float fov_y,
                              int screen_h, float tolerance_px, State* st) {
    int lv = select_level(&a->base, distance, fov_y, screen_h, tolerance_px, st);
    while (lv > 0 && !a->valid[lv]) lv--;
    st->level = lv;                  /* hysteresis state must track a real level */
    return lv;
}

/* ---- test harness ------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const char* TMP_GOOD = "/tmp/lod_variant_good.bin";

static LodLevelMeta test_meta[LEVELS] = {
    { 12000, 0.0f,  {-10.0f, -10.0f, -10.0f}, {10.0f, 10.0f, 10.0f} },
    { 3000,  1.0f,  { -9.5f,  -9.5f,  -9.5f}, { 9.5f,  9.5f,  9.5f} },
    { 750,   4.0f,  { -9.0f,  -9.0f,  -9.0f}, { 9.0f,  9.0f,  9.0f} },
    { 188,  16.0f,  { -8.0f,  -8.0f,  -8.0f}, { 8.0f,  8.0f,  8.0f} }
};

static int write_bytes(const char* path, const unsigned char* buf, size_t n) {
    FILE* f = fopen(path, "wb");
    if (!f) return -1;
    size_t w = fwrite(buf, 1, n, f);
    return (fclose(f) == 0 && w == n) ? 0 : -1;
}

/* Reload a mutated copy of the good file; must diagnose, never crash. */
static LodError reload_mutated(const char* tmp, const unsigned char* good, size_t n,
                               size_t off, uint32_t new_u32, LodAsset* a) {
    unsigned char* buf = (unsigned char*)malloc(n);
    memcpy(buf, good, n);
    put_u32(buf + off, new_u32);
    if (write_bytes(tmp, buf, n) != 0) { free(buf); return LOD_E_IO; }
    free(buf);
    return lod_load(a, tmp);
}

static int field_prefix(const LodAsset* a, int level, const char* tail) {
    char want[48];
    if (level < 0) snprintf(want, sizeof(want), "%s", tail);
    else snprintf(want, sizeof(want), "level[%d].%s", level, tail);
    return strncmp(a->error_field, want, sizeof(want)) == 0;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };

    /* ---- bake + clean load --------------------------------------------- */
    check("save_ok", lod_save(TMP_GOOD, &m, test_meta) == LOD_OK);

    LodAsset a;
    check("load_ok", lod_load(&a, TMP_GOOD) == LOD_OK);
    int bits_equal = f32_bits(a.base.radius) == f32_bits(m.radius) &&
                     f32_bits(a.base.tau) == f32_bits(m.tau);
    for (int i = 0; i < LEVELS; i++)
        bits_equal = bits_equal &&
                     f32_bits(a.base.error[i]) == f32_bits(m.error[i]) &&
                     a.meta[i].face_count == test_meta[i].face_count &&
                     f32_bits(a.meta[i].bbox_min[0]) == f32_bits(test_meta[i].bbox_min[0]) &&
                     f32_bits(a.meta[i].bbox_max[2]) == f32_bits(test_meta[i].bbox_max[2]);
    check("loaded_bits_match_runtime", bits_equal);
    check("all_levels_valid", a.valid[0] && a.valid[1] && a.valid[2] && a.valid[3]);

    /* ---- frame-by-frame parity with the runtime selector --------------- */
    State st_rt = {0, 0, 0}, st_ld = {0, 0, 0};
    float fov = 1.0f;
    int h = 1080, parity = 1;
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;
        int lv_rt = select_level(&m, d, fov, h, 50.0f, &st_rt);
        int lv_ld = asset_select_level(&a, d, fov, h, 50.0f, &st_ld);
        if (lv_rt != lv_ld) parity = 0;
    }
    check("selection_parity_40_frames", parity);
    check("selection_switch_count_parity", st_rt.switches == st_ld.switches);

    /* original demo output, kept so the no-load build diff stays meaningful */
    int hist[LEVELS] = {0};
    State st = {0, 0, 0};
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;
        hist[select_level(&m, d, fov, h, 50.0f, &st)]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    printf("hysteresis_ok=%d\n", st.switches <= 8);

    /* ---- header/file corruption ---------------------------------------- */
    size_t gn = 0;
    unsigned char* good = read_whole_file(TMP_GOOD, &gn);
    check("good_file_size", good && gn == LOD_FILE_SIZE);
    const char* tmp = "/tmp/lod_variant_bad.bin";
    LodAsset bad;

    /* header errors keep their first-cause code/field but end in total
       rejection, since no record can be trusted without a valid header */
    check("bad_magic", reload_mutated(tmp, good, gn, 0, 0xdeadbeef, &bad) == LOD_E_NO_FALLBACK &&
                       bad.header_valid == 0 &&
                       field_prefix(&bad, -1, "header.magic"));
    check("bad_version", reload_mutated(tmp, good, gn, 4, 99u, &bad) == LOD_E_NO_FALLBACK &&
                         bad.header_valid == 0 &&
                         field_prefix(&bad, -1, "header.version"));

    /* tau 1.1 -> 1.15 stays in range, so only the header checksum catches it */
    uint32_t tau_115;
    float f115 = 1.15f;
    memcpy(&tau_115, &f115, 4);
    check("header_checksum", reload_mutated(tmp, good, gn, 20, tau_115, &bad) ==
                             LOD_E_NO_FALLBACK && bad.header_valid == 0 &&
                             field_prefix(&bad, -1, "header.checksum"));

    check("truncated", (write_bytes(tmp, good, 100) == 0 &&
                        lod_load(&bad, tmp) == LOD_E_TRUNCATED && bad.error_level == -1));
    unsigned char* bigger = (unsigned char*)malloc(gn + 4);
    memcpy(bigger, good, gn);
    memset(bigger + gn, 0, 4);
    check("oversized", (write_bytes(tmp, bigger, gn + 4) == 0 &&
                        lod_load(&bad, tmp) == LOD_E_SIZE));
    free(bigger);

    /* ---- per-level corruption with level+field diagnostics ------------- */
    size_t l2 = HDR_SIZE + 2u * LVL_SIZE;       /* level 2 face_count */
    check("face_count_monotonic",
          reload_mutated(tmp, good, gn, l2, 5000u, &bad) == LOD_E_FACE_COUNT &&
          bad.error_level == 2 && field_prefix(&bad, 2, "face_count") &&
          bad.valid[2] == 0 && bad.valid[0] && bad.valid[1] && bad.valid[3]);

    size_t l1_max = HDR_SIZE + 1u * LVL_SIZE + 20;   /* level 1 bbox_max.x */
    check("bbox_order",
          reload_mutated(tmp, good, gn, l1_max, f32_bits(-9.9f), &bad) == LOD_E_BBOX &&
          bad.error_level == 1 && field_prefix(&bad, 1, "bbox_max"));

    size_t l3_err = HDR_SIZE + 3u * LVL_SIZE + 4;    /* level 3 error -> +Inf */
    check("not_finite",
          reload_mutated(tmp, good, gn, l3_err, 0x7f800000u, &bad) == LOD_E_NOT_FINITE &&
          bad.error_level == 3 && field_prefix(&bad, 3, "error"));

    /* legal bbox shrink: semantics pass, the record checksum must fire */
    size_t l1_crc_target = HDR_SIZE + 1u * LVL_SIZE + 20;
    check("level_checksum",
          reload_mutated(tmp, good, gn, l1_crc_target, f32_bits(0.99f * 9.5f), &bad) ==
          LOD_E_CHECKSUM && bad.error_level == 1 && field_prefix(&bad, 1, "checksum"));

    /* ---- fallback: level 2 dead, selector must use level 1 instead ----- */
    LodAsset degraded;
    reload_mutated(tmp, good, gn, l2, 5000u, &degraded);
    State st_dg = {0, 0, 0};
    int fallback_ok = 1, saw_fallback = 0;
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;
        int lv_dg = asset_select_level(&degraded, d, fov, h, 50.0f, &st_dg);
        if (!degraded.valid[lv_dg]) fallback_ok = 0;       /* never return a dead level */
        if (lv_dg == 1 && degraded.valid[2] == 0) saw_fallback = 1;
        if (lv_dg > 1) fallback_ok = 0;                   /* level 3 is coarser than 2 */
    }
    check("fallback_uses_valid_level", fallback_ok && saw_fallback);

    /* ---- level 0 corruption: no fallback possible ---------------------- */
    size_t l0_face = HDR_SIZE;
    check("no_fallback_level0",
          reload_mutated(tmp, good, gn, l0_face, 0u, &bad) == LOD_E_NO_FALLBACK &&
          bad.error_level == 0 && field_prefix(&bad, 0, "face_count") && bad.valid[0] == 0);

    /* ---- header corruption: every level is distrusted, first cause kept - */
    check("header_failure_rejects_all",
          reload_mutated(tmp, good, gn, 0, 0xdeadbeef, &bad) == LOD_E_NO_FALLBACK &&
          bad.header_valid == 0 &&
          bad.valid[0] == 0 && bad.valid[1] == 0 && bad.valid[2] == 0 &&
          bad.valid[3] == 0 && field_prefix(&bad, -1, "header.magic") &&
          bad.base.radius == 0.0f && bad.base.tau == 0.0f);

    /* a per-level failure still leaves the header's radius/tau in place */
    LodAsset level_only;
    check("per_level_keeps_header_fields",
          reload_mutated(tmp, good, gn, HDR_SIZE + 2u * LVL_SIZE, 5000u,
                         &level_only) == LOD_E_FACE_COUNT &&
          f32_bits(level_only.base.radius) == f32_bits(m.radius) &&
          f32_bits(level_only.base.tau) == f32_bits(m.tau) &&
          level_only.header_valid == 1);

    free(good);
    remove(TMP_GOOD);
    remove(tmp);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}

#else  /* !LOD_ENABLE_LOAD: original runtime-only build, untouched behavior */

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    State st = {0, 0, 0};
    float fov = 1.0f; int h = 1080;
    int hist[LEVELS] = {0};
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int lv = select_level(&m, d, fov, h, 50.0f, &st);
        hist[lv]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    /* 迟滞应当让切换次数远小于采样次数 */
    printf("hysteresis_ok=%d\n", st.switches <= 8);
    return 0;
}

#endif
