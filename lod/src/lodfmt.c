/* lodfmt: encoding, decoding and validation of the LOD pack format.
   One implementation serves the baker and the runtime so the two can never
   drift apart: the baker validates the bytes it just built before writing. */
#include "lodfmt.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOD_ALIGN 8u
#define LOD_MESH_FLAG_RESERVED 0u

static uint64_t align_up(uint64_t v) {
    return (v + (LOD_ALIGN - 1u)) & ~(uint64_t)(LOD_ALIGN - 1u);
}

const char* lod_error_str(LodError e) {
    switch (e) {
    case LOD_OK: return "ok";
    case LOD_ERR_IO: return "io";
    case LOD_ERR_SIZE: return "size";
    case LOD_ERR_MAGIC: return "magic";
    case LOD_ERR_VERSION: return "version";
    case LOD_ERR_HEADER: return "header";
    case LOD_ERR_TABLE: return "table";
    case LOD_ERR_CRC: return "crc";
    case LOD_ERR_MESH: return "mesh";
    case LOD_ERR_LEVEL: return "level";
    case LOD_ERR_COUNTS: return "counts";
    case LOD_ERR_RANGE: return "range";
    case LOD_ERR_BBOX: return "bbox";
    case LOD_ERR_MONOTONE: return "monotone";
    case LOD_ERR_INDEX: return "index";
    default: return "unknown";
    }
}

/* ---- little-endian primitives ------------------------------------------- */

uint32_t lod_get_u32(const uint8_t* p) {
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 |
           (uint32_t)p[3] << 24;
}
uint64_t lod_get_u64(const uint8_t* p) {
    return (uint64_t)lod_get_u32(p) | (uint64_t)lod_get_u32(p + 4) << 32;
}
float lod_get_f32(const uint8_t* p) {
    uint32_t bits = lod_get_u32(p);
    float f;
    memcpy(&f, &bits, sizeof f);
    return f;
}
void lod_put_u32(uint8_t* p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
void lod_put_u64(uint8_t* p, uint64_t v) {
    lod_put_u32(p, (uint32_t)v);
    lod_put_u32(p + 4, (uint32_t)(v >> 32));
}
void lod_put_f32(uint8_t* p, float f) {
    uint32_t bits;
    memcpy(&bits, &f, sizeof bits);
    lod_put_u32(p, bits);
}

uint32_t lod_crc32(const void* data, size_t len) {
    const uint8_t* p = (const uint8_t*)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int k = 0; k < 8; k++)
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

/* ---- bounds ------------------------------------------------------------- */

void lod_bbox_init(LodBBox* b) {
    b->min[0] = b->min[1] = b->min[2] = 1e30f;
    b->max[0] = b->max[1] = b->max[2] = -1e30f;
}
void lod_bbox_add(LodBBox* b, float x, float y, float z) {
    if (x < b->min[0]) b->min[0] = x;
    if (y < b->min[1]) b->min[1] = y;
    if (z < b->min[2]) b->min[2] = z;
    if (x > b->max[0]) b->max[0] = x;
    if (y > b->max[1]) b->max[1] = y;
    if (z > b->max[2]) b->max[2] = z;
}
int lod_bbox_is_valid(const LodBBox* b) {
    for (int i = 0; i < 3; i++) {
        if (!isfinite(b->min[i]) || !isfinite(b->max[i])) return 0;
        if (b->min[i] > b->max[i]) return 0;
    }
    return 1;
}
/* outer contains inner; a small epsilon absorbs float round-off from baking */
int lod_bbox_contains(const LodBBox* outer, const LodBBox* inner) {
    if (!lod_bbox_is_valid(outer) || !lod_bbox_is_valid(inner)) return 0;
    for (int i = 0; i < 3; i++) {
        float pad = 1e-4f * (fabsf(inner->max[i]) + fabsf(inner->min[i]) + 1.0f);
        if (inner->min[i] < outer->min[i] - pad) return 0;
        if (inner->max[i] > outer->max[i] + pad) return 0;
    }
    return 1;
}
LodBBox lod_bbox_union(const LodBBox* a, const LodBBox* b) {
    LodBBox u;
    lod_bbox_init(&u);
    if (lod_bbox_is_valid(a)) {
        lod_bbox_add(&u, a->min[0], a->min[1], a->min[2]);
        lod_bbox_add(&u, a->max[0], a->max[1], a->max[2]);
    }
    if (lod_bbox_is_valid(b)) {
        lod_bbox_add(&u, b->min[0], b->min[1], b->min[2]);
        lod_bbox_add(&u, b->max[0], b->max[1], b->max[2]);
    }
    return u;
}

static void write_bbox(uint8_t* p, const LodBBox* b) {
    for (int i = 0; i < 3; i++) lod_put_f32(p + 4u * (size_t)i, b->min[i]);
    for (int i = 0; i < 3; i++) lod_put_f32(p + 12u + 4u * (size_t)i, b->max[i]);
}
static LodBBox read_bbox(const uint8_t* p) {
    LodBBox b;
    for (int i = 0; i < 3; i++) b.min[i] = lod_get_f32(p + 4u * (size_t)i);
    for (int i = 0; i < 3; i++) b.max[i] = lod_get_f32(p + 12u + 4u * (size_t)i);
    return b;
}

/* ---- header ------------------------------------------------------------- */

LodError lod_read_header(const uint8_t* buf, size_t size, LodHeader* out) {
    if (size < LOD_HEADER_BYTES) return LOD_ERR_SIZE;
    if (memcmp(buf + LOD_HDR_OFF_MAGIC, LOD_FORMAT_MAGIC, 8) != 0) return LOD_ERR_MAGIC;
    out->version = lod_get_u32(buf + LOD_HDR_OFF_VERSION);
    if (out->version != LOD_FORMAT_VERSION) return LOD_ERR_VERSION;
    if (lod_get_u32(buf + LOD_HDR_OFF_HEADER_BYTES) != LOD_HEADER_BYTES)
        return LOD_ERR_HEADER;
    out->mesh_count = lod_get_u32(buf + LOD_HDR_OFF_MESH_COUNT);
    out->level_count = lod_get_u32(buf + LOD_HDR_OFF_LEVEL_COUNT);
    out->table_off = lod_get_u64(buf + LOD_HDR_OFF_TABLE_OFF);
    out->table_bytes = lod_get_u64(buf + LOD_HDR_OFF_TABLE_BYTES);
    out->data_off = lod_get_u64(buf + LOD_HDR_OFF_DATA_OFF);
    out->file_bytes = lod_get_u64(buf + LOD_HDR_OFF_FILE_BYTES);
    out->payload_crc = lod_get_u32(buf + LOD_HDR_OFF_PAYLOAD_CRC);
    out->flags = lod_get_u32(buf + LOD_HDR_OFF_FLAGS);
    return LOD_OK;
}

/* ---- validation --------------------------------------------------------- */

static int u64_add_ok(uint64_t a, uint64_t b, uint64_t* out) {
    if (a > UINT64_MAX - b) return 0;
    *out = a + b;
    return 1;
}

LodError lod_pack_validate(const uint8_t* buf, size_t size,
                           uint32_t* bad_mesh, uint32_t* bad_level) {
    if (bad_mesh) *bad_mesh = UINT32_MAX;
    if (bad_level) *bad_level = UINT32_MAX;

    LodHeader h;
    LodError e = lod_read_header(buf, size, &h);
    if (e != LOD_OK) return e;

    if (h.mesh_count == 0 || h.mesh_count > LOD_MAX_MESHES) return LOD_ERR_COUNTS;
    if (h.level_count == 0 || h.level_count > LOD_MAX_MESHES * LOD_MAX_LEVELS_PER_MESH)
        return LOD_ERR_COUNTS;
    if (h.table_off < LOD_HEADER_BYTES) return LOD_ERR_HEADER;
    uint64_t expect_table = (uint64_t)h.mesh_count * LOD_MESH_REC_BYTES +
                            (uint64_t)h.level_count * LOD_LEVEL_REC_BYTES;
    if (h.table_bytes != expect_table) return LOD_ERR_TABLE;
    uint64_t table_end;
    if (!u64_add_ok(h.table_off, h.table_bytes, &table_end)) return LOD_ERR_TABLE;
    if (table_end > size) return LOD_ERR_SIZE;
    if (h.data_off < table_end || h.data_off > size) return LOD_ERR_HEADER;
    if (h.file_bytes != size) return LOD_ERR_SIZE;
    if (h.flags != 0) return LOD_ERR_HEADER;

    /* payload integrity covers every byte after the header */
    if (lod_crc32(buf + LOD_HEADER_BYTES, size - LOD_HEADER_BYTES) != h.payload_crc)
        return LOD_ERR_CRC;

    const uint8_t* mt = buf + h.table_off;
    const uint8_t* lt = mt + (uint64_t)h.mesh_count * LOD_MESH_REC_BYTES;

    uint32_t level_cursor = 0;
    for (uint32_t mi = 0; mi < h.mesh_count; mi++) {
        const uint8_t* m = mt + (uint64_t)mi * LOD_MESH_REC_BYTES;
        uint32_t level_count = lod_get_u32(m + 4);
        uint32_t level_base = lod_get_u32(m + 8);
        uint64_t levels_off = lod_get_u64(m + 48);
        uint64_t rec_off = h.table_off + (uint64_t)h.mesh_count * LOD_MESH_REC_BYTES +
                           (uint64_t)level_base * LOD_LEVEL_REC_BYTES;
        LodBBox source = read_bbox(m + 24);

        if (level_count == 0 || level_count > LOD_MAX_LEVELS_PER_MESH) {
            if (bad_mesh) *bad_mesh = mi;
            return LOD_ERR_COUNTS;
        }
        if (level_base != level_cursor ||
            (uint64_t)level_cursor + level_count > h.level_count) {
            if (bad_mesh) *bad_mesh = mi;
            return LOD_ERR_COUNTS;
        }
        if (levels_off != rec_off ||
            lod_get_u32(m + 12) == 0 || lod_get_u32(m + 16) == 0 ||
            lod_get_u32(m + 20) != 0 ||
            !lod_bbox_is_valid(&source)) {
            if (bad_mesh) *bad_mesh = mi;
            return LOD_ERR_MESH;
        }
        /* mesh ids must be unique so runtime lookup is unambiguous */
        for (uint32_t mj = 0; mj < mi; mj++) {
            if (lod_get_u32(mt + (uint64_t)mj * LOD_MESH_REC_BYTES) ==
                lod_get_u32(m)) {
                if (bad_mesh) *bad_mesh = mi;
                return LOD_ERR_MESH;
            }
        }

        float prev_error = -1.0f;
        for (uint32_t li = 0; li < level_count; li++) {
            const uint8_t* l = lt + (uint64_t)(level_base + li) * LOD_LEVEL_REC_BYTES;
            uint32_t level = lod_get_u32(l + 0);
            uint32_t vertex_count = lod_get_u32(l + 4);
            uint32_t tri_count = lod_get_u32(l + 8);
            float geom_error = lod_get_f32(l + 12);
            LodBBox bounds = read_bbox(l + 16);
            uint64_t verts_off = lod_get_u64(l + 40);
            uint64_t idx_off = lod_get_u64(l + 48);
            uint32_t verts_bytes = lod_get_u32(l + 56);
            uint32_t idx_bytes = lod_get_u32(l + 60);

            uint32_t err_mesh = mi, err_level = level_base + li;
            if (level != li || vertex_count == 0 || tri_count == 0) {
                if (bad_mesh) *bad_mesh = err_mesh;
                if (bad_level) *bad_level = err_level;
                return LOD_ERR_LEVEL;
            }
            if (!isfinite(geom_error) || geom_error < 0.0f ||
                geom_error <= prev_error) {
                if (bad_mesh) *bad_mesh = err_mesh;
                if (bad_level) *bad_level = err_level;
                return LOD_ERR_MONOTONE;
            }
            prev_error = geom_error;

            /* the simplification must never shrink outside the original mesh */
            if (!lod_bbox_is_valid(&bounds) ||
                !lod_bbox_contains(&bounds, &source)) {
                if (bad_mesh) *bad_mesh = err_mesh;
                if (bad_level) *bad_level = err_level;
                return LOD_ERR_BBOX;
            }

            uint64_t want_v = (uint64_t)vertex_count * 12u;
            uint64_t want_i = (uint64_t)tri_count * 12u;
            uint64_t v_end, i_end;
            if (verts_bytes != want_v || idx_bytes != want_i ||
                !u64_add_ok(verts_off, verts_bytes, &v_end) ||
                !u64_add_ok(idx_off, idx_bytes, &i_end) ||
                verts_off < h.data_off || idx_off < h.data_off ||
                v_end > size || i_end > size ||
                (verts_off & (LOD_ALIGN - 1u)) || (idx_off & (LOD_ALIGN - 1u))) {
                if (bad_mesh) *bad_mesh = err_mesh;
                if (bad_level) *bad_level = err_level;
                return LOD_ERR_RANGE;
            }

            const uint8_t* idx = buf + idx_off;
            for (uint32_t t = 0; t < tri_count * 3u; t++) {
                if (lod_get_u32(idx + (size_t)t * 4u) >= vertex_count) {
                    if (bad_mesh) *bad_mesh = err_mesh;
                    if (bad_level) *bad_level = err_level;
                    return LOD_ERR_INDEX;
                }
            }
        }
        level_cursor += level_count;
    }
    if (level_cursor != h.level_count) return LOD_ERR_COUNTS;
    return LOD_OK;
}

/* ---- writing ------------------------------------------------------------ */

LodError lod_pack_write(const char* path, const LodMeshInput* meshes,
                        uint32_t mesh_count, uint64_t* out_bytes,
                        char* err, size_t errlen) {
    if (out_bytes) *out_bytes = 0;
    if (mesh_count == 0 || mesh_count > LOD_MAX_MESHES ||
        meshes == NULL) {
        if (err) snprintf(err, errlen, "bad mesh list");
        return LOD_ERR_MESH;
    }

    uint32_t total_levels = 0;
    uint64_t data_bytes = 0;
    for (uint32_t i = 0; i < mesh_count; i++) {
        const LodMeshInput* m = &meshes[i];
        if (m->level_count == 0 || m->level_count > LOD_MAX_LEVELS_PER_MESH ||
            m->levels == NULL ||
            total_levels > LOD_MAX_MESHES * LOD_MAX_LEVELS_PER_MESH - m->level_count) {
            if (err) snprintf(err, errlen, "mesh %u has invalid level count", i);
            return LOD_ERR_COUNTS;
        }
        total_levels += m->level_count;
        for (uint32_t l = 0; l < m->level_count; l++) {
            const LodLevelInput* lv = &m->levels[l];
            if (lv->vertex_count == 0 || lv->tri_count == 0 ||
                lv->verts == NULL || lv->indices == NULL) {
                if (err) snprintf(err, errlen, "mesh %u level %u is empty", i, l);
                return LOD_ERR_LEVEL;
            }
            data_bytes = (uint64_t)align_up(data_bytes) +
                         (uint64_t)lv->vertex_count * 12u;
            data_bytes = (uint64_t)align_up(data_bytes) +
                         (uint64_t)lv->tri_count * 12u;
        }
    }

    uint64_t table_off = LOD_HEADER_BYTES;
    uint64_t table_bytes = (uint64_t)mesh_count * LOD_MESH_REC_BYTES +
                           (uint64_t)total_levels * LOD_LEVEL_REC_BYTES;
    uint64_t data_off = table_off + table_bytes;
    uint64_t file_bytes = data_off + data_bytes;
    if (file_bytes > (uint64_t)SIZE_MAX) {
        if (err) snprintf(err, errlen, "pack too large");
        return LOD_ERR_SIZE;
    }

    uint8_t* buf = (uint8_t*)calloc(1, (size_t)file_bytes);
    if (!buf) {
        if (err) snprintf(err, errlen, "out of memory");
        return LOD_ERR_SIZE;
    }

    memcpy(buf + LOD_HDR_OFF_MAGIC, LOD_FORMAT_MAGIC, 8);
    lod_put_u32(buf + LOD_HDR_OFF_VERSION, LOD_FORMAT_VERSION);
    lod_put_u32(buf + LOD_HDR_OFF_HEADER_BYTES, LOD_HEADER_BYTES);
    lod_put_u32(buf + LOD_HDR_OFF_MESH_COUNT, mesh_count);
    lod_put_u32(buf + LOD_HDR_OFF_LEVEL_COUNT, total_levels);
    lod_put_u64(buf + LOD_HDR_OFF_TABLE_OFF, table_off);
    lod_put_u64(buf + LOD_HDR_OFF_TABLE_BYTES, table_bytes);
    lod_put_u64(buf + LOD_HDR_OFF_DATA_OFF, data_off);
    lod_put_u64(buf + LOD_HDR_OFF_FILE_BYTES, file_bytes);
    lod_put_u32(buf + LOD_HDR_OFF_FLAGS, LOD_MESH_FLAG_RESERVED);

    uint8_t* mt = buf + table_off;
    uint8_t* lt = mt + (uint64_t)mesh_count * LOD_MESH_REC_BYTES;
    uint64_t cursor = data_off;
    uint32_t level_base = 0;

    for (uint32_t i = 0; i < mesh_count; i++) {
        const LodMeshInput* m = &meshes[i];
        uint8_t* mrec = mt + (uint64_t)i * LOD_MESH_REC_BYTES;
        lod_put_u32(mrec + 0, m->mesh_id);
        lod_put_u32(mrec + 4, m->level_count);
        lod_put_u32(mrec + 8, level_base);
        lod_put_u32(mrec + 12, m->source_vert_count);
        lod_put_u32(mrec + 16, m->source_tri_count);
        lod_put_u32(mrec + 20, 0);
        write_bbox(mrec + 24, &m->source_bounds);
        lod_put_u64(mrec + 48, table_off + (uint64_t)mesh_count * LOD_MESH_REC_BYTES +
                                  (uint64_t)level_base * LOD_LEVEL_REC_BYTES);
        lod_put_u64(mrec + 56, 0);

        for (uint32_t l = 0; l < m->level_count; l++) {
            const LodLevelInput* lv = &m->levels[l];
            uint32_t gi = level_base + l;
            uint8_t* lrec = lt + (uint64_t)gi * LOD_LEVEL_REC_BYTES;
            cursor = align_up(cursor);
            uint64_t verts_off = cursor;
            cursor += (uint64_t)lv->vertex_count * 12u;
            cursor = align_up(cursor);
            uint64_t idx_off = cursor;
            cursor += (uint64_t)lv->tri_count * 12u;

            lod_put_u32(lrec + 0, l);
            lod_put_u32(lrec + 4, lv->vertex_count);
            lod_put_u32(lrec + 8, lv->tri_count);
            lod_put_f32(lrec + 12, lv->geom_error);
            write_bbox(lrec + 16, &lv->bounds);
            lod_put_u64(lrec + 40, verts_off);
            lod_put_u64(lrec + 48, idx_off);
            lod_put_u32(lrec + 56, lv->vertex_count * 12u);
            lod_put_u32(lrec + 60, lv->tri_count * 12u);

            for (uint32_t v = 0; v < lv->vertex_count * 3u; v++)
                lod_put_f32(buf + verts_off + (size_t)v * 4u, lv->verts[v]);
            for (uint32_t k = 0; k < lv->tri_count * 3u; k++)
                lod_put_u32(buf + idx_off + (size_t)k * 4u, lv->indices[k]);
        }
        level_base += m->level_count;
    }

    lod_put_u32(buf + LOD_HDR_OFF_PAYLOAD_CRC,
                lod_crc32(buf + LOD_HEADER_BYTES,
                          (size_t)(file_bytes - LOD_HEADER_BYTES)));

    uint32_t bad_mesh = 0, bad_level = 0;
    LodError e = lod_pack_validate(buf, (size_t)file_bytes, &bad_mesh, &bad_level);
    if (e != LOD_OK) {
        if (err) {
            if (bad_mesh != UINT32_MAX) {
                snprintf(err, errlen, "bake rejected (%s) at mesh %u level %u",
                         lod_error_str(e), bad_mesh,
                         bad_level == UINT32_MAX ? 0u : bad_level);
            } else {
                snprintf(err, errlen, "bake rejected (%s)", lod_error_str(e));
            }
        }
        free(buf);
        return e;
    }

    FILE* f = fopen(path, "wb");
    if (!f) {
        free(buf);
        if (err) snprintf(err, errlen, "cannot open %s", path);
        return LOD_ERR_IO;
    }
    size_t wrote = fwrite(buf, 1, (size_t)file_bytes, f);
    int close_rc = fclose(f);
    free(buf);
    if (wrote != (size_t)file_bytes || close_rc != 0) {
        if (err) snprintf(err, errlen, "short write to %s", path);
        return LOD_ERR_IO;
    }
    if (out_bytes) *out_bytes = file_bytes;
    return LOD_OK;
}

/* ---- runtime decoding --------------------------------------------------- */

LodError lod_pack_load(const char* path, LodPack* out, char* err, size_t errlen) {
    memset(out, 0, sizeof *out);
    FILE* f = fopen(path, "rb");
    if (!f) {
        if (err) snprintf(err, errlen, "cannot open %s", path);
        return LOD_ERR_IO;
    }
    if (fseek(f, 0, SEEK_END) != 0) {
        fclose(f);
        if (err) snprintf(err, errlen, "seek failed");
        return LOD_ERR_IO;
    }
    long sz = ftell(f);
    if (sz < 0 || fseek(f, 0, SEEK_SET) != 0) {
        fclose(f);
        if (err) snprintf(err, errlen, "seek failed");
        return LOD_ERR_IO;
    }
    uint8_t* buf = (uint8_t*)malloc((size_t)sz ? (size_t)sz : 1u);
    if (!buf) {
        fclose(f);
        if (err) snprintf(err, errlen, "out of memory");
        return LOD_ERR_SIZE;
    }
    size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) {
        free(buf);
        if (err) snprintf(err, errlen, "short read from %s", path);
        return LOD_ERR_IO;
    }

    uint32_t bad_mesh = 0, bad_level = 0;
    LodError e = lod_pack_validate(buf, (size_t)sz, &bad_mesh, &bad_level);
    if (e != LOD_OK) {
        free(buf);
        if (err) {
            if (bad_mesh != UINT32_MAX) {
                snprintf(err, errlen, "reject %s (%s) mesh %u level %u", path,
                         lod_error_str(e), bad_mesh,
                         bad_level == UINT32_MAX ? 0u : bad_level);
            } else {
                snprintf(err, errlen, "reject %s (%s)", path, lod_error_str(e));
            }
        }
        return e;
    }

    out->buf = buf;
    out->size = (size_t)sz;
    lod_read_header(buf, (size_t)sz, &out->hdr);
    return LOD_OK;
}

void lod_pack_free(LodPack* p) {
    free(p->buf);
    memset(p, 0, sizeof *p);
}

int lod_pack_find_mesh(const LodPack* p, uint32_t mesh_id) {
    if (!p->buf) return -1;
    const uint8_t* mt = p->buf + p->hdr.table_off;
    for (uint32_t i = 0; i < p->hdr.mesh_count; i++) {
        if (lod_get_u32(mt + (uint64_t)i * LOD_MESH_REC_BYTES) == mesh_id)
            return (int)i;
    }
    return -1;
}

int lod_pack_mesh(const LodPack* p, uint32_t mesh_index, LodMeshView* out) {
    if (!p->buf || mesh_index >= p->hdr.mesh_count) return -1;
    const uint8_t* m = p->buf + p->hdr.table_off +
                       (uint64_t)mesh_index * LOD_MESH_REC_BYTES;
    out->mesh_id = lod_get_u32(m + 0);
    out->level_count = lod_get_u32(m + 4);
    out->source_vert_count = lod_get_u32(m + 12);
    out->source_tri_count = lod_get_u32(m + 16);
    out->source_bounds = read_bbox(m + 24);
    out->levels_off = lod_get_u64(m + 48);
    return 0;
}

int lod_pack_level(const LodPack* p, uint32_t mesh_index, uint32_t level_index,
                   LodLevelView* out) {
    LodMeshView mv;
    if (lod_pack_mesh(p, mesh_index, &mv) != 0) return -1;
    if (level_index >= mv.level_count) return -1;
    const uint8_t* l = p->buf + mv.levels_off +
                       (uint64_t)level_index * LOD_LEVEL_REC_BYTES;
    out->level = lod_get_u32(l + 0);
    out->vertex_count = lod_get_u32(l + 4);
    out->tri_count = lod_get_u32(l + 8);
    out->geom_error = lod_get_f32(l + 12);
    out->bounds = read_bbox(l + 16);
    out->verts_off = lod_get_u64(l + 40);
    out->idx_off = lod_get_u64(l + 48);
    out->verts_bytes = lod_get_u32(l + 56);
    out->idx_bytes = lod_get_u32(l + 60);
    return 0;
}
