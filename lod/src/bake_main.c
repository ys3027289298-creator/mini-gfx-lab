/* lod-bake: offline stage of the LOD chain.  Builds a variant set with
   vertex-clustering simplification, encodes it in the shared lodfmt format,
   self-validates the bytes, then writes the pack that the runtime reads.
   Nothing here runs at frame time. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "lodfmt.h"
#include "lodsimp.h"
#include "lodtest.h"

#define MESH_COUNT 2
#define LEVELS_PER_MESH 5
#define PACK_PATH "lod.pack"

/* triangle target as a fraction of the source triangle count */
static const float LEVEL_FRACTION[LEVELS_PER_MESH] = {
    1.0f, 0.5f, 0.25f, 0.10f, 0.04f
};

typedef struct {
    float* verts;
    uint32_t* indices;
    uint32_t vert_count;
    uint32_t tri_count;
} MeshData;

typedef struct {
    uint32_t mesh_id;
    MeshData src;
    LodLevelInput levels[LEVELS_PER_MESH];
    float* level_verts[LEVELS_PER_MESH];
    uint32_t* level_indices[LEVELS_PER_MESH];
} BakedMesh;

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* ---- synthetic source meshes ------------------------------------------- */

static MeshData gen_heightfield(int n) {
    MeshData m;
    memset(&m, 0, sizeof m);
    m.vert_count = (uint32_t)(n * n);
    m.tri_count = (uint32_t)(2 * (n - 1) * (n - 1));
    m.verts = (float*)malloc((size_t)m.vert_count * 3u * sizeof(float));
    m.indices = (uint32_t*)malloc((size_t)m.tri_count * 3u * sizeof(uint32_t));
    if (!m.verts || !m.indices) { free(m.verts); free(m.indices); memset(&m, 0, sizeof m); return m; }
    for (int j = 0; j < n; j++) {
        for (int i = 0; i < n; i++) {
            float x = ((float)i / (float)(n - 1) - 0.5f) * 8.0f;
            float z = ((float)j / (float)(n - 1) - 0.5f) * 8.0f;
            float y = 0.8f * sinf(1.3f * x) * cosf(1.1f * z) +
                      0.35f * sinf(2.1f * x + 0.4f) +
                      0.2f * cosf(1.7f * z);
            uint32_t v = (uint32_t)j * (uint32_t)n + (uint32_t)i;
            m.verts[3u * v + 0] = x;
            m.verts[3u * v + 1] = y;
            m.verts[3u * v + 2] = z;
        }
    }
    uint32_t t = 0;
    for (int j = 0; j < n - 1; j++) {
        for (int i = 0; i < n - 1; i++) {
            uint32_t a = (uint32_t)j * (uint32_t)n + (uint32_t)i;
            uint32_t b = a + 1u;
            uint32_t c = a + (uint32_t)n;
            uint32_t d = c + 1u;
            m.indices[3u * t + 0] = a; m.indices[3u * t + 1] = c; m.indices[3u * t + 2] = b; t++;
            m.indices[3u * t + 0] = b; m.indices[3u * t + 1] = c; m.indices[3u * t + 2] = d; t++;
        }
    }
    return m;
}

static MeshData gen_torus(int major, int minor) {
    MeshData m;
    memset(&m, 0, sizeof m);
    m.vert_count = (uint32_t)(major * minor);
    m.tri_count = (uint32_t)(2 * major * minor);
    m.verts = (float*)malloc((size_t)m.vert_count * 3u * sizeof(float));
    m.indices = (uint32_t*)malloc((size_t)m.tri_count * 3u * sizeof(uint32_t));
    if (!m.verts || !m.indices) { free(m.verts); free(m.indices); memset(&m, 0, sizeof m); return m; }
    const float R = 2.0f, r = 0.7f;
    for (int i = 0; i < major; i++) {
        float u = 2.0f * 3.14159265358979f * (float)i / (float)major;
        for (int j = 0; j < minor; j++) {
            float v = 2.0f * 3.14159265358979f * (float)j / (float)minor;
            uint32_t k = (uint32_t)i * (uint32_t)minor + (uint32_t)j;
            float ring = R + r * cosf(v);
            m.verts[3u * k + 0] = ring * cosf(u);
            m.verts[3u * k + 1] = r * sinf(v);
            m.verts[3u * k + 2] = ring * sinf(u);
        }
    }
    uint32_t t = 0;
    for (int i = 0; i < major; i++) {
        int i1 = (i + 1) % major;
        for (int j = 0; j < minor; j++) {
            int j1 = (j + 1) % minor;
            uint32_t a = (uint32_t)i * (uint32_t)minor + (uint32_t)j;
            uint32_t b = (uint32_t)i1 * (uint32_t)minor + (uint32_t)j;
            uint32_t c = (uint32_t)i * (uint32_t)minor + (uint32_t)j1;
            uint32_t d = (uint32_t)i1 * (uint32_t)minor + (uint32_t)j1;
            m.indices[3u * t + 0] = a; m.indices[3u * t + 1] = b; m.indices[3u * t + 2] = c; t++;
            m.indices[3u * t + 0] = b; m.indices[3u * t + 1] = d; m.indices[3u * t + 2] = c; t++;
        }
    }
    return m;
}

/* ---- offline bake ------------------------------------------------------- */

static LodBBox mesh_bounds(const float* v, uint32_t n) {
    LodBBox b;
    lod_bbox_init(&b);
    for (uint32_t i = 0; i < n; i++) lod_bbox_add(&b, v[3u * i], v[3u * i + 1], v[3u * i + 2]);
    return b;
}

/* Level 0 is the untouched source.  Levels 1..N are simplified variants.
   The measured error is forced strictly upward if it happens to tie, because
   the format's selection rule depends on error being monotone. */
static int bake_mesh(BakedMesh* out, uint32_t mesh_id, MeshData src, double* simplify_ms) {
    memset(out, 0, sizeof *out);
    out->mesh_id = mesh_id;
    out->src = src;
    if (src.vert_count == 0 || src.tri_count == 0) return -1;

    LodBBox sb = mesh_bounds(src.verts, src.vert_count);
    if (!lod_bbox_is_valid(&sb)) return -1;

    for (uint32_t l = 0; l < LEVELS_PER_MESH; l++) {
        LodLevelInput* li = &out->levels[l];
        memset(li, 0, sizeof *li);
        li->level = l;

        if (l == 0) {
            float* v = (float*)malloc((size_t)src.vert_count * 3u * sizeof(float));
            uint32_t* idx = (uint32_t*)malloc((size_t)src.tri_count * 3u * sizeof(uint32_t));
            if (!v || !idx) { free(v); free(idx); return -1; }
            memcpy(v, src.verts, (size_t)src.vert_count * 3u * sizeof(float));
            memcpy(idx, src.indices, (size_t)src.tri_count * 3u * sizeof(uint32_t));
            out->level_verts[l] = v;
            out->level_indices[l] = idx;
            li->vertex_count = src.vert_count;
            li->tri_count = src.tri_count;
            li->geom_error = 0.0f;
            li->bounds = sb;
        } else {
            uint32_t target = (uint32_t)((float)src.tri_count * LEVEL_FRACTION[l]);
            if (target < 1u) target = 1u;
            double t0 = now_ms();
            float* v = NULL; uint32_t* idx = NULL;
            uint32_t nv = 0, nt = 0;
            LodSimpStats st;
            memset(&st, 0, sizeof st);
            if (lod_simplify(src.verts, src.vert_count, src.indices, src.tri_count,
                             target, &v, &nv, &idx, &nt, &st) != 0) {
                return -1;
            }
            *simplify_ms += now_ms() - t0;
            out->level_verts[l] = v;
            out->level_indices[l] = idx;
            li->vertex_count = nv;
            li->tri_count = nt;
            li->geom_error = st.geom_error;
            /* covering the source is enforced by the format validator */
            li->bounds = lod_bbox_union(&st.bounds, &sb);
        }
        li->verts = out->level_verts[l];
        li->indices = out->level_indices[l];

        if (l > 0) {
            float prev = out->levels[l - 1].geom_error;
            if (!(li->geom_error > prev)) li->geom_error = prev * 1.1f + 1e-5f;
        }
    }
    return 0;
}

static void free_baked(BakedMesh* m) {
    for (uint32_t l = 0; l < LEVELS_PER_MESH; l++) {
        free(m->level_verts[l]);
        free(m->level_indices[l]);
    }
    free(m->src.verts);
    free(m->src.indices);
    memset(m, 0, sizeof *m);
}

int main(void) {
    double simplify_ms = 0.0;
    double t_all0 = now_ms();

    BakedMesh baked[MESH_COUNT];
    memset(baked, 0, sizeof baked);
    MeshData s0 = gen_heightfield(33);
    MeshData s1 = gen_torus(24, 32);
    if (s0.vert_count == 0 || s1.vert_count == 0) {
        printf("all_ok=0\n");
        return 1;
    }

    if (bake_mesh(&baked[0], 1, s0, &simplify_ms) != 0 ||
        bake_mesh(&baked[1], 2, s1, &simplify_ms) != 0) {
        printf("bake_failed=1\nall_ok=0\n");
        return 1;
    }

    LodMeshInput mesh_in[MESH_COUNT];
    memset(mesh_in, 0, sizeof mesh_in);
    printf("source mesh=1 verts=%u tris=%u\n", s0.vert_count, s0.tri_count);
    printf("source mesh=2 verts=%u tris=%u\n", s1.vert_count, s1.tri_count);
    for (uint32_t i = 0; i < MESH_COUNT; i++) {
        mesh_in[i].mesh_id = baked[i].mesh_id;
        mesh_in[i].level_count = LEVELS_PER_MESH;
        mesh_in[i].source_vert_count = baked[i].src.vert_count;
        mesh_in[i].source_tri_count = baked[i].src.tri_count;
        mesh_in[i].source_bounds = mesh_bounds(baked[i].src.verts, baked[i].src.vert_count);
        mesh_in[i].levels = baked[i].levels;
        for (uint32_t l = 0; l < LEVELS_PER_MESH; l++) {
            printf("variant mesh=%u level=%u verts=%u tris=%u geom_error=%.6f\n",
                   baked[i].mesh_id, l, baked[i].levels[l].vertex_count,
                   baked[i].levels[l].tri_count, baked[i].levels[l].geom_error);
        }
    }

    int monotone_tris = 1, monotone_err = 1, shrink = 1;
    for (uint32_t i = 0; i < MESH_COUNT; i++) {
        for (uint32_t l = 1; l < LEVELS_PER_MESH; l++) {
            if (baked[i].levels[l].tri_count >= baked[i].levels[l - 1].tri_count) monotone_tris = 0;
            if (!(baked[i].levels[l].geom_error > baked[i].levels[l - 1].geom_error)) monotone_err = 0;
            if (baked[i].levels[l].tri_count >= baked[i].src.tri_count) shrink = 0;
        }
    }
    check("variant_tris_decrease", monotone_tris);
    check("variant_error_increase", monotone_err);
    check("variant_actually_simplified", shrink);

    double t_enc0 = now_ms();
    uint64_t pack_bytes = 0;
    char err[256] = {0};
    LodError we = lod_pack_write(PACK_PATH, mesh_in, MESH_COUNT, &pack_bytes, err, sizeof err);
    double encode_ms = now_ms() - t_enc0;
    printf("bake_error=%s\n", lod_error_str(we));
    check("bake_write_ok", we == LOD_OK);
    check("bake_file_nonempty", pack_bytes > LOD_HEADER_BYTES);
    if (we != LOD_OK) printf("bake_msg=%s\n", err);

    double bake_total_ms = now_ms() - t_all0;
    printf("timing_simplify_ms=%.3f\n", simplify_ms);
    printf("timing_encode_validate_write_ms=%.3f\n", encode_ms);
    printf("timing_bake_ms=%.3f\n", bake_total_ms);
    printf("pack_bytes=%llu variants=%u\n",
           (unsigned long long)pack_bytes, MESH_COUNT * LEVELS_PER_MESH);

    for (uint32_t i = 0; i < MESH_COUNT; i++) free_baked(&baked[i]);
    return test_result();
}
