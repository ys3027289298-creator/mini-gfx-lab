
/* instancing: one mesh, 100 objects, one transform per object.

   Two submission paths drive the same software rasterizer:

     uniform   - one draw call per object, the transform arrives as a
                 per-draw uniform (what the pre-instancing path did).
     instanced - objects are bucketed by material; each bucket is one draw
                 call and the transform is fetched from the instance buffer
                 with the instance index handed to the vertex stage
                 (gl_InstanceIndex).

   The two paths must produce a bit-identical framebuffer. The instance
   buffer can be rebuilt from scratch every frame or patched incrementally
   for the objects that moved; both are timed, and the incremental buffer is
   checked byte-for-byte against a full rebuild and against the picture.
*/
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include <time.h>

#define NUM_OBJECTS   100
#define NUM_MATERIALS 4
#define MAX_BATCHES   NUM_MATERIALS
#define IB_CAP        (NUM_OBJECTS + 8)

#define FB_W 160
#define FB_H 120

#define FNV_OFFSET 2166136261u
#define FNV_PRIME  16777619u

typedef struct { float x, y, z; } Vec3;

/* One instance = 3 x vec4 transform rows (translation in .w) + one vec4 of
   custom data. 64 bytes, the stride the vertex stage indexes by instance. */
typedef struct {
    float row[3][4];
    int id;            /* custom data: stable object id, breaks depth ties */
    float pad[3];
} InstanceXform;

typedef struct {
    Vec3 p[8];
    unsigned char tri[12][3];
} Mesh;

typedef struct { unsigned char r, g, b; } Material;

typedef struct {
    Vec3 pos, rot;
    float scale;
    int material;
    int id;
    int slot;          /* fixed slot in the batch-ordered instance buffer */
} Object;

/* A batch is a contiguous run of instance slots sharing one material. */
typedef struct {
    int material;
    int first;
    int count;
} Batch;

typedef struct {
    InstanceXform* data;
    int capacity;
    int count;
} InstanceBuffer;

typedef struct { int calls, instances; } DrawStats;

typedef struct { float x, y, invz; } ScreenVert;

typedef struct {
    unsigned char rgb[FB_H][FB_W][3];
    float invz[FB_H][FB_W];   /* 1/z, larger is closer */
    int id[FB_H][FB_W];
} Framebuffer;

/* ---------------- scene ---------------- */

static void mesh_box(Mesh* m) {
    const float h = 0.35f;
    static const Vec3 corners[8] = {
        { -h, -h, -h }, {  h, -h, -h }, {  h,  h, -h }, { -h,  h, -h },
        { -h, -h,  h }, {  h, -h,  h }, {  h,  h,  h }, { -h,  h,  h },
    };
    memcpy(m->p, corners, sizeof(corners));
    static const unsigned char tris[12][3] = {
        { 0, 2, 1 }, { 0, 3, 2 },   /* -Z */
        { 4, 5, 6 }, { 4, 6, 7 },   /* +Z */
        { 0, 1, 5 }, { 0, 5, 4 },   /* -Y */
        { 3, 7, 6 }, { 3, 6, 2 },   /* +Y */
        { 0, 4, 7 }, { 0, 7, 3 },   /* -X */
        { 1, 2, 6 }, { 1, 6, 5 },   /* +X */
    };
    memcpy(m->tri, tris, sizeof(tris));
}

static void init_materials(Material* mats) {
    static const unsigned char rgb[NUM_MATERIALS][3] = {
        { 214, 74, 58 }, { 62, 148, 92 }, { 58, 104, 186 }, { 208, 170, 58 },
    };
    for (int i = 0; i < NUM_MATERIALS; i++) {
        mats[i].r = rgb[i][0];
        mats[i].g = rgb[i][1];
        mats[i].b = rgb[i][2];
    }
}

static void init_objects(Object* objs) {
    for (int i = 0; i < NUM_OBJECTS; i++) {
        Object* o = &objs[i];
        memset(o, 0, sizeof(*o));
        int col = i % 10, row = i / 10;
        o->pos = (Vec3){ (col - 4.5f) * 0.26f, (row - 4.5f) * 0.26f,
                         0.25f * sinf((float)i * 0.7f) };
        o->rot = (Vec3){ 0.3f * sinf((float)i * 1.1f),
                         0.4f * cosf((float)i * 0.9f),
                         0.2f * sinf((float)i * 0.37f) };
        o->scale = 0.75f + 0.2f * (float)(i % 7) / 6.0f;
        /* material id deliberately not aligned with submission order, so a
           batch really has to collect instances rather than take a run */
        o->material = (i * 7 + i / 10) % NUM_MATERIALS;
        o->id = i;
        o->slot = -1;
    }
}

static void animate(Object* o) {
    o->rot.y += 0.05f;
    o->rot.x = 0.30f * sinf(o->rot.y * 0.7f);
    o->pos.z = 0.25f * sinf(o->rot.y * 1.3f);
}

static void compose(const Object* o, InstanceXform* xf) {
    float cx = cosf(o->rot.x), sx = sinf(o->rot.x);
    float cy = cosf(o->rot.y), sy = sinf(o->rot.y);
    float cz = cosf(o->rot.z), sz = sinf(o->rot.z);
    float s = o->scale;
    memset(xf, 0, sizeof(*xf));
    /* row-major Rz * Ry * Rx * scale, translation in the last column */
    xf->row[0][0] = (cz * cy) * s;
    xf->row[0][1] = (cz * sy * sx - sz * cx) * s;
    xf->row[0][2] = (cz * sy * cx + sz * sx) * s;
    xf->row[0][3] = o->pos.x;
    xf->row[1][0] = (sz * cy) * s;
    xf->row[1][1] = (sz * sy * sx + cz * cx) * s;
    xf->row[1][2] = (sz * sy * cx - cz * sx) * s;
    xf->row[1][3] = o->pos.y;
    xf->row[2][0] = (-sy) * s;
    xf->row[2][1] = (cy * sx) * s;
    xf->row[2][2] = (cy * cx) * s;
    xf->row[2][3] = o->pos.z;
    xf->id = o->id;
}

/* Collect same-material instances into contiguous batches. Slot order inside
   a batch is object order; the collection is stable across frames because
   material assignment is static. */
static int build_batches(Batch* batches, Object* objs) {
    int n_batch = 0, slot = 0;
    for (int m = 0; m < NUM_MATERIALS; m++) {
        Batch* b = &batches[n_batch++];
        b->material = m;
        b->first = slot;
        b->count = 0;
        for (int i = 0; i < NUM_OBJECTS; i++) {
            if (objs[i].material != m) continue;
            objs[i].slot = slot++;
            b->count++;
        }
    }
    return n_batch;
}

/* ---------------- instance buffer ---------------- */

static void ib_rebuild(InstanceBuffer* ib, const Object* objs) {
    for (int i = 0; i < NUM_OBJECTS; i++)
        compose(&objs[i], &ib->data[objs[i].slot]);
    ib->count = NUM_OBJECTS;
}

static void ib_update(InstanceBuffer* ib, const Object* objs,
                      const int* changed, int n_changed) {
    for (int k = 0; k < n_changed; k++) {
        const Object* o = &objs[changed[k]];
        compose(o, &ib->data[o->slot]);
    }
}

/* ---------------- rasterizer ---------------- */

static void fb_clear(Framebuffer* fb) {
    memset(fb->rgb, 0, sizeof(fb->rgb));
    memset(fb->invz, 0, sizeof(fb->invz));
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++) fb->id[y][x] = INT_MAX;
}

static void project(const Vec3* p, ScreenVert* v) {
    const float cam_z = 4.5f, focal = 170.0f;
    float z = p->z + cam_z;
    if (z < 0.05f) z = 0.05f;
    v->invz = 1.0f / z;
    v->x = FB_W * 0.5f + focal * p->x * v->invz;
    v->y = FB_H * 0.5f - focal * p->y * v->invz;
}

static inline float edge(float ax, float ay, float bx, float by, float px, float py) {
    return (bx - ax) * (py - ay) - (by - ay) * (px - ax);
}

static void raster_triangle(Framebuffer* fb, const ScreenVert* a,
                            const ScreenVert* b, const ScreenVert* c,
                            unsigned char r, unsigned char g, unsigned char bl,
                            int id) {
    float area = edge(a->x, a->y, b->x, b->y, c->x, c->y);
    if (fabsf(area) < 1e-9f) return;
    float inv_area = 1.0f / area;

    float minx = fminf(a->x, fminf(b->x, c->x));
    float maxx = fmaxf(a->x, fmaxf(b->x, c->x));
    float miny = fminf(a->y, fminf(b->y, c->y));
    float maxy = fmaxf(a->y, fmaxf(b->y, c->y));
    int x0 = (int)floorf(minx), x1 = (int)ceilf(maxx);
    int y0 = (int)floorf(miny), y1 = (int)ceilf(maxy);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > FB_W - 1) x1 = FB_W - 1;
    if (y1 > FB_H - 1) y1 = FB_H - 1;

    for (int y = y0; y <= y1; y++) {
        float py = (float)y + 0.5f;
        for (int x = x0; x <= x1; x++) {
            float px = (float)x + 0.5f;
            float w0 = edge(b->x, b->y, c->x, c->y, px, py) * inv_area;
            float w1 = edge(c->x, c->y, a->x, a->y, px, py) * inv_area;
            float w2 = 1.0f - w0 - w1;
            if (w0 < 0.0f || w1 < 0.0f || w2 < 0.0f) continue;
            float invz = w0 * a->invz + w1 * b->invz + w2 * c->invz;
            if (invz < fb->invz[y][x]) continue;
            if (invz == fb->invz[y][x] && id >= fb->id[y][x]) continue;
            fb->invz[y][x] = invz;
            fb->id[y][x] = id;
            fb->rgb[y][x][0] = r;
            fb->rgb[y][x][1] = g;
            fb->rgb[y][x][2] = bl;
        }
    }
}

/* Vertex + fragment stage for one instance. The transform passed in is the
   only per-instance input, exactly like a real instanced vertex shader. */
static void draw_instance(Framebuffer* fb, const Mesh* mesh,
                          const InstanceXform* xf, const Material* mat) {
    Vec3 world[8];
    ScreenVert sv[8];
    for (int i = 0; i < 8; i++) {
        const Vec3* p = &mesh->p[i];
        world[i].x = xf->row[0][0] * p->x + xf->row[0][1] * p->y + xf->row[0][2] * p->z + xf->row[0][3];
        world[i].y = xf->row[1][0] * p->x + xf->row[1][1] * p->y + xf->row[1][2] * p->z + xf->row[1][3];
        world[i].z = xf->row[2][0] * p->x + xf->row[2][1] * p->y + xf->row[2][2] * p->z + xf->row[2][3];
        project(&world[i], &sv[i]);
    }

    static const Vec3 L = { 0.3f, 0.5f, -0.81f };
    for (int t = 0; t < 12; t++) {
        const unsigned char* v = mesh->tri[t];
        Vec3 e1 = { world[v[1]].x - world[v[0]].x,
                    world[v[1]].y - world[v[0]].y,
                    world[v[1]].z - world[v[0]].z };
        Vec3 e2 = { world[v[2]].x - world[v[0]].x,
                    world[v[2]].y - world[v[0]].y,
                    world[v[2]].z - world[v[0]].z };
        Vec3 n = { e1.y * e2.z - e1.z * e2.y,
                   e1.z * e2.x - e1.x * e2.z,
                   e1.x * e2.y - e1.y * e2.x };
        float len = sqrtf(n.x * n.x + n.y * n.y + n.z * n.z);
        if (len > 1e-12f) { n.x /= len; n.y /= len; n.z /= len; }
        float ndl = fabsf(n.x * L.x + n.y * L.y + n.z * L.z);
        float shade = 0.32f + 0.68f * ndl;
        unsigned char r = (unsigned char)((float)mat->r * shade);
        unsigned char g = (unsigned char)((float)mat->g * shade);
        unsigned char b = (unsigned char)((float)mat->b * shade);
        raster_triangle(fb, &sv[v[0]], &sv[v[1]], &sv[v[2]], r, g, b, xf->id);
    }
}

/* Path A: one draw call per object, transform as a per-draw uniform. */
static void submit_uniform(Framebuffer* fb, const Mesh* mesh, const Object* objs,
                           const Material* mats, DrawStats* st) {
    for (int i = 0; i < NUM_OBJECTS; i++) {
        InstanceXform uni;
        compose(&objs[i], &uni);
        st->calls++;
        st->instances++;
        draw_instance(fb, mesh, &uni, &mats[objs[i].material]);
    }
}

/* Path B: one draw call per material batch; the vertex stage receives an
   instance index and fetches that instance's transform from the buffer.
   index_bias exists only for the negative control that proves the pixel
   comparison notices a wrong index. */
static void submit_instanced(Framebuffer* fb, const Mesh* mesh,
                             const InstanceBuffer* ib, const Batch* batches,
                             int n_batch, const Material* mats,
                             int index_bias, DrawStats* st) {
    for (int b = 0; b < n_batch; b++) {
        const Batch* batch = &batches[b];
        st->calls++;
        for (int k = 0; k < batch->count; k++) {
            /* this is the whole point: transform by instance index */
            const InstanceXform* xf = &ib->data[batch->first + k + index_bias];
            draw_instance(fb, mesh, xf, &mats[batch->material]);
        }
        st->instances += batch->count;
    }
}

/* ---------------- framebuffer comparison ---------------- */

static unsigned fb_hash(const Framebuffer* fb) {
    unsigned h = FNV_OFFSET;
    const unsigned char* p = &fb->rgb[0][0][0];
    for (size_t i = 0; i < sizeof(fb->rgb); i++) {
        h ^= p[i];
        h *= FNV_PRIME;
    }
    return h;
}

static int fb_diff_pixels(const Framebuffer* a, const Framebuffer* b) {
    int n = 0;
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++)
            if (memcmp(a->rgb[y][x], b->rgb[y][x], 3) != 0) n++;
    return n;
}

static int fb_painted(const Framebuffer* fb) {
    int n = 0;
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++)
            if (fb->rgb[y][x][0] || fb->rgb[y][x][1] || fb->rgb[y][x][2]) n++;
    return n;
}

/* ---------------- timing ---------------- */

static double now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec * 1e-3;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---------------- rebuild vs incremental ---------------- */

/* Animate the quarter of the scene that moves on this frame; returns how many
   objects need to be re-extracted. */
static int step_scene(Object* objs, int frame, int* changed) {
    int n = 0;
    for (int i = frame % 4; i < NUM_OBJECTS; i += 4) {
        animate(&objs[i]);
        changed[n++] = i;
    }
    return n;
}

int main(void) {
    static Mesh mesh;
    static Material mats[NUM_MATERIALS];
    static Object objs[NUM_OBJECTS];
    static Batch batches[MAX_BATCHES];
    static InstanceXform full_data[IB_CAP];
    static InstanceXform inc_data[IB_CAP];
    static Framebuffer fb_ref, fb_inst, fb_bad;

    mesh_box(&mesh);
    init_materials(mats);
    init_objects(objs);

    InstanceBuffer ib_full = { full_data, IB_CAP, 0 };
    InstanceBuffer ib_inc  = { inc_data,  IB_CAP, 0 };

    int n_batch = build_batches(batches, objs);
    ib_rebuild(&ib_full, objs);
    ib_rebuild(&ib_inc, objs);

    /* every object must land in exactly one slot and each slot once */
    static unsigned char slot_seen[NUM_OBJECTS];
    int slots_ok = 1, slots_total = 0;
    for (int b = 0; b < n_batch; b++) {
        if (batches[b].first != slots_total) slots_ok = 0;
        slots_total += batches[b].count;
        for (int i = 0; i < NUM_OBJECTS; i++) {
            if (objs[i].material != batches[b].material) continue;
            if (objs[i].slot < batches[b].first ||
                objs[i].slot >= batches[b].first + batches[b].count) slots_ok = 0;
            if (slot_seen[objs[i].slot]++) slots_ok = 0;
        }
    }
    if (slots_total != NUM_OBJECTS) slots_ok = 0;
    for (int i = 0; i < NUM_OBJECTS; i++) if (slot_seen[i] != 1) slots_ok = 0;

    /* ---- picture: uniform vs instanced ---- */
    DrawStats st_uni = { 0, 0 }, st_inst = { 0, 0 };
    fb_clear(&fb_ref);
    submit_uniform(&fb_ref, &mesh, objs, mats, &st_uni);
    fb_clear(&fb_inst);
    submit_instanced(&fb_inst, &mesh, &ib_inc, batches, n_batch, mats, 0, &st_inst);

    int diff = fb_diff_pixels(&fb_ref, &fb_inst);
    int painted = fb_painted(&fb_ref);
    unsigned hash = fb_hash(&fb_ref);

    /* negative control: the same batched path, reading the neighbouring
       instance slot. If the comparison could not see that, it would be
       worthless as a proof of correctness. */
    DrawStats st_bad = { 0, 0 };
    fb_clear(&fb_bad);
    submit_instanced(&fb_bad, &mesh, &ib_inc, batches, n_batch, mats, 1, &st_bad);
    int bad_diff = fb_diff_pixels(&fb_ref, &fb_bad);

    /* ---- rebuild vs incremental instance data ----
       Each phase is timed as a block of REPS identical calls so the clock
       pair itself is negligible, then divided back down to one frame. */
    enum { WARMUP = 200, FRAMES = 2000, REPS = 16 };
    double us_rebuild = 0.0, us_update = 0.0, us_full = 0.0;
    int changed[NUM_OBJECTS];

    for (int f = 0; f < WARMUP + FRAMES; f++) {
        int n_changed = step_scene(objs, f, changed);
        if (f < WARMUP) {
            ib_rebuild(&ib_full, objs);
            ib_update(&ib_inc, objs, changed, n_changed);
            continue;
        }
        double t0 = now_us();
        for (int r = 0; r < REPS; r++) ib_rebuild(&ib_full, objs);
        double t1 = now_us();
        for (int r = 0; r < REPS; r++) ib_update(&ib_inc, objs, changed, n_changed);
        double t2 = now_us();
        us_rebuild += (t1 - t0) / REPS;
        us_update += (t2 - t1) / REPS;
    }
    int moved_per_frame = NUM_OBJECTS / 4;

    int data_match = memcmp(full_data, inc_data,
                            sizeof(InstanceXform) * NUM_OBJECTS) == 0;

    /* all-dirty incremental: same slot writes, no gathering saving */
    {
        static InstanceXform scratch_data[IB_CAP];
        InstanceBuffer ib_scratch = { scratch_data, IB_CAP, 0 };
        int all[NUM_OBJECTS];
        for (int i = 0; i < NUM_OBJECTS; i++) all[i] = i;
        double t0 = now_us();
        for (int f = 0; f < FRAMES; f++)
            for (int r = 0; r < REPS; r++)
                ib_update(&ib_scratch, objs, all, NUM_OBJECTS);
        us_full = (now_us() - t0) / (FRAMES * REPS);
        if (memcmp(scratch_data, inc_data, sizeof(InstanceXform) * NUM_OBJECTS) != 0)
            failures++;
    }

    /* the incrementally patched buffer must still draw the reference frame */
    DrawStats st_uni2 = { 0, 0 }, st_inst2 = { 0, 0 };
    fb_clear(&fb_ref);
    submit_uniform(&fb_ref, &mesh, objs, mats, &st_uni2);
    fb_clear(&fb_inst);
    submit_instanced(&fb_inst, &mesh, &ib_inc, batches, n_batch, mats, 0, &st_inst2);
    int diff_inc = fb_diff_pixels(&fb_ref, &fb_inst);

    size_t stride = sizeof(InstanceXform);
    printf("instances=%d\n", NUM_OBJECTS);
    printf("materials=%d\n", NUM_MATERIALS);
    printf("batches=%d\n", n_batch);
    printf("batch_sizes=");
    for (int b = 0; b < n_batch; b++) printf("%d%s", batches[b].count, b + 1 < n_batch ? "," : "\n");
    printf("draw_calls_uniform=%d\n", st_uni.calls);
    printf("draw_calls_instanced=%d\n", st_inst.calls);
    printf("draw_call_reduction=%.2fx\n", (double)st_uni.calls / (double)st_inst.calls);
    printf("instances_submitted_uniform=%d\n", st_uni.instances);
    printf("instances_submitted_instanced=%d\n", st_inst.instances);
    printf("instance_stride_bytes=%zu\n", stride);
    printf("pixels_painted=%d\n", painted);
    printf("pixels_differing=%d\n", diff);
    printf("framebuffer_hash=%08x\n", hash);
    printf("misindex_diff_pixels=%d\n", bad_diff);
    printf("rebuild_us_per_frame=%.4f\n", us_rebuild / FRAMES);
    printf("rebuild_bytes_per_frame=%zu\n", stride * NUM_OBJECTS);
    printf("incremental_us_per_frame=%.4f\n", us_update / FRAMES);
    printf("incremental_dirty_instances=%d\n", moved_per_frame);
    printf("incremental_bytes_per_frame=%zu\n", stride * (size_t)moved_per_frame);
    printf("incremental_speedup=%.2fx\n",
           us_update > 0.0 ? us_rebuild / us_update : 0.0);
    printf("incremental_all_dirty_us_per_frame=%.4f\n", us_full);
    printf("incremental_matches_rebuild=%d\n", data_match);
    printf("incremental_render_identical=%d\n", diff_inc == 0);

    check("slots_exactly_once", slots_ok);
    check("uniform_vs_instanced_identical", diff == 0);
    check("framebuffer_nonempty", painted > 1000);
    check("misindex_detected", bad_diff > 0);
    check("instance_counts_match",
          st_uni.instances == NUM_OBJECTS && st_inst.instances == NUM_OBJECTS);
    check("draw_calls_reduced", st_inst.calls < st_uni.calls &&
                                st_uni.calls == NUM_OBJECTS);
    check("incremental_matches_rebuild", data_match);
    check("incremental_render_identical", diff_inc == 0);
    check("incremental_faster", us_update < us_rebuild);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
