#define _POSIX_C_SOURCE 199309L

/* skinning: CPU linear-blend skinning (LBS).

   Per-vertex pipeline:
     1. truncate influences to the 4-bone limit, keeping the strongest ones,
        and record every cut in SkinStats;
     2. normalize the surviving weights -- a zero total weight falls back to
        the identity pose, so no 0/0 division (and no NaN) is produced;
     3. blend the bone matrices by weight and transform position and normal.
   The blended-matrix path is cross-checked against a hand-written per-vertex
   weighted-sum path and against shuffled influence orders. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define SKIN_BONE_LIMIT 4
#define SKIN_MAX_BONES 32
#define SKIN_MAX_INFL 8
#define SKIN_TRUNC_LOG 256
#define BENCH_VERTS 10000
#define BENCH_FRAMES 200

/* ---------- row-major 4x4 ---------- */

typedef struct { float m[16]; } Mat4;

static void mat4_zero(Mat4* a) { memset(a, 0, sizeof(*a)); }

static void mat4_identity(Mat4* a) {
    mat4_zero(a);
    a->m[0] = a->m[5] = a->m[10] = a->m[15] = 1.0f;
}

static void mat4_translation(Mat4* a, float x, float y, float z) {
    mat4_identity(a);
    a->m[3] = x; a->m[7] = y; a->m[11] = z;
}

static void mat4_rot_z(Mat4* a, float t) {
    mat4_identity(a);
    float c = cosf(t), s = sinf(t);
    a->m[0] = c;  a->m[1] = -s;
    a->m[4] = s;  a->m[5] = c;
}

static void mat4_mul(Mat4* out, const Mat4* a, const Mat4* b) {
    Mat4 r;
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++) {
            float s = 0.0f;
            for (int k = 0; k < 4; k++)
                s += a->m[i * 4 + k] * b->m[k * 4 + j];
            r.m[i * 4 + j] = s;
        }
    *out = r;
}

static void mat4_transform_point(const Mat4* a, const float* p, float* q) {
    float x = p[0], y = p[1], z = p[2];
    float w = a->m[12] * x + a->m[13] * y + a->m[14] * z + a->m[15];
    q[0] = a->m[0] * x + a->m[1] * y + a->m[2] * z + a->m[3];
    q[1] = a->m[4] * x + a->m[5] * y + a->m[6] * z + a->m[7];
    q[2] = a->m[8] * x + a->m[9] * y + a->m[10] * z + a->m[11];
    if (w != 0.0f && w != 1.0f) { q[0] /= w; q[1] /= w; q[2] /= w; }
}

static void mat4_transform_dir(const Mat4* a, const float* n, float* out) {
    float x = n[0], y = n[1], z = n[2];
    out[0] = a->m[0] * x + a->m[1] * y + a->m[2] * z;
    out[1] = a->m[4] * x + a->m[5] * y + a->m[6] * z;
    out[2] = a->m[8] * x + a->m[9] * y + a->m[10] * z;
}

static void vec3_normalize(float* v) {
    float len = sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0.0f) { v[0] /= len; v[1] /= len; v[2] /= len; }
}

/* ---------- skinning data ---------- */

typedef struct { int bone; float weight; } Influ;

/* raw binding straight from the asset, up to SKIN_MAX_INFL entries */
typedef struct {
    Influ infl[SKIN_MAX_INFL];
    int count;
} SkinVertex;

/* binding after truncation + normalization, ready for the per-frame pose */
typedef struct {
    Influ infl[SKIN_BONE_LIMIT];
    int count;
    int zero;       /* all input weights were zero: pose with identity */
} CompactVertex;

typedef struct { int vertex, before, after; } TruncEntry;

typedef struct {
    int truncated_verts;
    int dropped_influences;
    int zero_weight_verts;
    TruncEntry log[SKIN_TRUNC_LOG];
    int log_count;
} SkinStats;

/* higher weight first; bone index ascending breaks ties deterministically */
static int infl_before(const SkinVertex* s, int a, int b) {
    if (s->infl[a].weight != s->infl[b].weight)
        return s->infl[a].weight > s->infl[b].weight;
    return s->infl[a].bone < s->infl[b].bone;
}

/* truncate to the 4-bone limit, record the cut, then normalize.
   normalization runs after truncation: it is a monotone scale, so the
   strongest-4 selection is unaffected by when it happens. */
static void skin_compact(const SkinVertex* in, int nverts,
                         CompactVertex* out, SkinStats* st) {
    memset(st, 0, sizeof(*st));
    for (int v = 0; v < nverts; v++) {
        const SkinVertex* s = &in[v];
        CompactVertex* c = &out[v];
        memset(c, 0, sizeof(*c));

        int idx[SKIN_MAX_INFL];
        int valid = 0;
        for (int i = 0; i < s->count; i++)
            if (s->infl[i].bone >= 0 && s->infl[i].bone < SKIN_MAX_BONES)
                idx[valid++] = i;

        /* insertion sort of the valid indices, descending weight */
        for (int i = 1; i < valid; i++) {
            int key = idx[i], j = i - 1;
            while (j >= 0 && infl_before(s, key, idx[j])) {
                idx[j + 1] = idx[j];
                j--;
            }
            idx[j + 1] = key;
        }

        int keep = valid < SKIN_BONE_LIMIT ? valid : SKIN_BONE_LIMIT;
        if (valid > SKIN_BONE_LIMIT) {
            st->truncated_verts++;
            st->dropped_influences += valid - SKIN_BONE_LIMIT;
            if (st->log_count < SKIN_TRUNC_LOG)
                st->log[st->log_count++] = (TruncEntry){v, valid, SKIN_BONE_LIMIT};
        }

        float sum = 0.0f;
        for (int k = 0; k < keep; k++) sum += s->infl[idx[k]].weight;

        if (sum <= 0.0f) {
            /* zero-weight vertex: identity fallback, never a 0/0 */
            c->zero = 1;
            st->zero_weight_verts++;
            continue;
        }
        c->count = keep;
        for (int k = 0; k < keep; k++) {
            c->infl[k].bone = s->infl[idx[k]].bone;
            c->infl[k].weight = s->infl[idx[k]].weight / sum;
        }
    }
}

/* fast path: build the blended matrix once, then transform. */
static void skin_pose(const CompactVertex* cvs, const Mat4* bones, int nverts,
                      const float* pos_in, float* pos_out,
                      const float* nrm_in, float* nrm_out) {
    for (int v = 0; v < nverts; v++) {
        const CompactVertex* c = &cvs[v];
        Mat4 blend;
        mat4_zero(&blend);
        if (c->zero) {
            mat4_identity(&blend);
        } else {
            for (int i = 0; i < c->count; i++) {
                const Mat4* b = &bones[c->infl[i].bone];
                float w = c->infl[i].weight;
                for (int k = 0; k < 16; k++) blend.m[k] += w * b->m[k];
            }
        }
        mat4_transform_point(&blend, &pos_in[v * 3], &pos_out[v * 3]);
        mat4_transform_dir(&blend, &nrm_in[v * 3], &nrm_out[v * 3]);
        vec3_normalize(&nrm_out[v * 3]);
    }
}

/* reference path: per-vertex hand weighting. each bone transforms the
   vertex on its own and the results are summed -- no blended matrix is
   ever formed, so agreement here proves the matrix blend is correct. */
static void skin_pose_manual(const CompactVertex* cvs, const Mat4* bones, int nverts,
                             const float* pos_in, float* pos_out,
                             const float* nrm_in, float* nrm_out) {
    for (int v = 0; v < nverts; v++) {
        const CompactVertex* c = &cvs[v];
        if (c->zero) {
            memcpy(&pos_out[v * 3], &pos_in[v * 3], 3 * sizeof(float));
            memcpy(&nrm_out[v * 3], &nrm_in[v * 3], 3 * sizeof(float));
            continue;
        }
        float acc_p[3] = {0.0f, 0.0f, 0.0f};
        float acc_n[3] = {0.0f, 0.0f, 0.0f};
        for (int i = 0; i < c->count; i++) {
            float q[3], d[3];
            float w = c->infl[i].weight;
            mat4_transform_point(&bones[c->infl[i].bone], &pos_in[v * 3], q);
            mat4_transform_dir(&bones[c->infl[i].bone], &nrm_in[v * 3], d);
            for (int k = 0; k < 3; k++) {
                acc_p[k] += w * q[k];
                acc_n[k] += w * d[k];
            }
        }
        vec3_normalize(acc_n);
        memcpy(&pos_out[v * 3], acc_p, 3 * sizeof(float));
        memcpy(&nrm_out[v * 3], acc_n, 3 * sizeof(float));
    }
}

/* ---------- test helpers ---------- */

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int close3(const float* a, const float* b, float eps) {
    for (int i = 0; i < 3; i++)
        if (fabsf(a[i] - b[i]) > eps) return 0;
    return 1;
}

static int all_finite(const float* a, int n) {
    for (int i = 0; i < n; i++)
        if (!isfinite(a[i])) return 0;
    return 1;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1000.0 + (double)ts.tv_nsec / 1e6;
}

/* ---------- benchmark mesh (file scope to stay off the stack) ---------- */

static SkinVertex bench_in[BENCH_VERTS];
static CompactVertex bench_cv[BENCH_VERTS];
static float bench_pos_in[BENCH_VERTS * 3];
static float bench_pos_out[BENCH_VERTS * 3];
static float bench_pos_ref[BENCH_VERTS * 3];
static float bench_nrm_in[BENCH_VERTS * 3];
static float bench_nrm_out[BENCH_VERTS * 3];
static float bench_nrm_ref[BENCH_VERTS * 3];

static unsigned g_seed = 0x12345678u;

static float rngf(void) {
    g_seed = g_seed * 1103515245u + 12345u;
    return (float)((g_seed >> 8) & 0xffffff) / (float)0x1000000;
}

int main(void) {
    /* --- hand-computed two-bone blend: 0.25*T(1) + 0.75*T(3) = T(2.5) --- */
    Mat4 bones[SKIN_MAX_BONES];
    for (int i = 0; i < SKIN_MAX_BONES; i++) mat4_identity(&bones[i]);
    mat4_translation(&bones[0], 1.0f, 0.0f, 0.0f);
    mat4_translation(&bones[1], 3.0f, 0.0f, 0.0f);
    Mat4 rz; mat4_rot_z(&rz, 1.5707963f);
    mat4_mul(&bones[2], &bones[1], &rz);       /* T(3,0,0) then rotate 90 deg */

    SkinVertex sv[2];
    memset(sv, 0, sizeof(sv));
    sv[0].count = 2;
    sv[0].infl[0] = (Influ){0, 0.25f};
    sv[0].infl[1] = (Influ){1, 0.75f};
    float pin[6] = {0.0f, 0.0f, 0.0f,  1.0f, 0.0f, 0.0f};
    float nin[6]  = {1.0f, 0.0f, 0.0f,  0.0f, 1.0f, 0.0f};
    float pout[6], nout[6], pref[6], nref[6];

    CompactVertex cv[2];
    SkinStats st;
    skin_compact(sv, 1, cv, &st);
    skin_pose(cv, bones, 1, pin, pout, nin, nout);
    skin_pose_manual(cv, bones, 1, pin, pref, nin, nref);

    float expect[3] = {2.5f, 0.0f, 0.0f};
    check("hand_weighted_value", close3(pout, expect, 1e-5f));
    check("matches_manual_pos", close3(pout, pref, 1e-6f));
    check("matches_manual_nrm", close3(nout, nref, 1e-6f));
    check("normalized_sums_one", fabsf(cv[0].infl[0].weight + cv[0].infl[1].weight - 1.0f) < 1e-6f);

    /* --- truncation: 6 influences must be cut to the strongest 4 --- */
    SkinVertex tv;
    tv.count = 6;
    float raw_w[6] = {0.40f, 0.05f, 0.30f, 0.10f, 0.08f, 0.07f};
    for (int i = 0; i < 6; i++) tv.infl[i] = (Influ){i, raw_w[i]};
    SkinVertex tv2 = tv;
    SkinVertex tmesh[2] = {tv, tv2};
    CompactVertex tcv[2];
    SkinStats tst;
    skin_compact(tmesh, 2, tcv, &tst);

    int kept_ok = tcv[0].count == 4;
    int kept_bones[4] = {tcv[0].infl[0].bone, tcv[0].infl[1].bone,
                         tcv[0].infl[2].bone, tcv[0].infl[3].bone};
    int want_bones[4] = {0, 2, 3, 4};       /* .40, .30, .10, .08 */
    for (int i = 0; i < 4; i++) if (kept_bones[i] != want_bones[i]) kept_ok = 0;
    float wsum = 0.0f;
    for (int i = 0; i < 4; i++) wsum += tcv[0].infl[i].weight;
    check("truncate_to_four", kept_ok);
    check("truncate_normalized", fabsf(wsum - 1.0f) < 1e-6f);
    check("truncate_top_weight", fabsf(tcv[0].infl[0].weight - 0.40f / 0.88f) < 1e-6f);
    check("truncate_recorded", tst.truncated_verts == 2 && tst.dropped_influences == 4
                               && tst.log_count == 2
                               && tst.log[0].vertex == 0 && tst.log[0].before == 6
                               && tst.log[0].after == 4
                               && tst.log[1].vertex == 1);

    /* --- order independence at the pipeline level: shuffled raw bindings --- */
    SkinVertex shuffled;
    shuffled.count = 6;
    int perm[6] = {5, 2, 0, 4, 1, 3};
    for (int i = 0; i < 6; i++) shuffled.infl[i] = (Influ){perm[i], raw_w[perm[i]]};
    CompactVertex scv;
    SkinStats sst;
    skin_compact(&shuffled, 1, &scv, &sst);
    float sp_out[3], sn_out[3];
    skin_pose(&scv, bones, 1, pin, sp_out, nin, sn_out);
    float tp_out[3], tn_out[3];
    skin_pose(&tcv[0], bones, 1, pin, tp_out, nin, tn_out);
    check("order_independent_pipeline", close3(sp_out, tp_out, 1e-6f)
                                        && close3(sn_out, tn_out, 1e-6f));

    /* --- order independence at the blend level: reversed accumulation --- */
    CompactVertex fwd, rev;
    fwd.count = rev.count = 3;
    fwd.infl[0] = (Influ){0, 0.5f}; fwd.infl[1] = (Influ){1, 0.3f}; fwd.infl[2] = (Influ){2, 0.2f};
    rev.infl[0] = (Influ){2, 0.2f}; rev.infl[1] = (Influ){1, 0.3f}; rev.infl[2] = (Influ){0, 0.5f};
    float fp[3], fn[3], rp[3], rn[3];
    skin_pose(&fwd, bones, 1, pin, fp, nin, fn);
    skin_pose(&rev, bones, 1, pin, rp, nin, rn);
    check("order_independent_blend", close3(fp, rp, 1e-6f) && close3(fn, rn, 1e-6f));

    /* --- zero-weight vertex: identity fallback, no NaN, position kept --- */
    SkinVertex zv;
    zv.count = 2;
    zv.infl[0] = (Influ){0, 0.0f};
    zv.infl[1] = (Influ){1, 0.0f};
    CompactVertex zcv;
    SkinStats zst;
    skin_compact(&zv, 1, &zcv, &zst);
    float zp[3], zn[3];
    float zpos[3] = {7.0f, -3.0f, 2.0f};
    float znrm[3] = {0.0f, 0.0f, 1.0f};
    skin_pose(&zcv, bones, 1, zpos, zp, znrm, zn);
    check("zero_weight_flagged", zcv.zero == 1 && zst.zero_weight_verts == 1);
    check("zero_weight_keeps_pose", close3(zp, zpos, 1e-7f) && close3(zn, znrm, 1e-7f));
    check("zero_weight_finite", all_finite(zp, 3) && all_finite(zn, 3));

    /* --- normal transform: a full-weight 90-degree Z rotation --- */
    SkinVertex nv;
    nv.count = 1;
    nv.infl[0] = (Influ){2, 1.0f};
    CompactVertex ncv;
    SkinStats nst;
    skin_compact(&nv, 1, &ncv, &nst);
    float npp[3], nnn[3];
    skin_pose(&ncv, bones, 1, pin, npp, nin, nnn);
    float want_n[3] = {0.0f, 1.0f, 0.0f};
    check("normal_rotated", close3(nnn, want_n, 1e-5f));

    /* --- benchmark: 10k vertices, single-frame timing + 200-frame average --- */
    /* every bone the mesh can reference (indices 0..7) gets a real pose:
       translation by (i,0,0) composed with a per-bone Z rotation */
    for (int i = 0; i < 8; i++) {
        Mat4 t, r;
        mat4_translation(&t, (float)i, 0.0f, 0.0f);
        mat4_rot_z(&r, 0.13f * (float)i);
        mat4_mul(&bones[i], &t, &r);
    }
    for (int v = 0; v < BENCH_VERTS; v++) {
        for (int k = 0; k < 3; k++) {
            bench_pos_in[v * 3 + k] = rngf() * 20.0f - 10.0f;
            bench_nrm_in[v * 3 + k] = rngf() * 2.0f - 1.0f;
        }
        vec3_normalize(&bench_nrm_in[v * 3]);
        if (v % 97 == 0) {
            /* scattered zero-weight vertices to prove the guard at scale */
            bench_in[v].count = 2;
            bench_in[v].infl[0] = (Influ){0, 0.0f};
            bench_in[v].infl[1] = (Influ){1, 0.0f};
        } else {
            int cnt = 1 + (v % SKIN_BONE_LIMIT);           /* 1..4 influences */
            bench_in[v].count = cnt;
            for (int k = 0; k < cnt; k++) {
                bench_in[v].infl[k] = (Influ){(v + k) % 8, rngf() + 0.01f};
            }
        }
    }

    double t0 = now_ms();
    skin_compact(bench_in, BENCH_VERTS, bench_cv, &st);
    double compact_ms = now_ms() - t0;
    check("bench_zero_verts_counted", st.zero_weight_verts == (BENCH_VERTS + 96) / 97);

    t0 = now_ms();
    skin_pose(bench_cv, bones, BENCH_VERTS, bench_pos_in, bench_pos_out,
              bench_nrm_in, bench_nrm_out);
    double single_ms = now_ms() - t0;

    t0 = now_ms();
    for (int f = 0; f < BENCH_FRAMES; f++)
        skin_pose(bench_cv, bones, BENCH_VERTS, bench_pos_in, bench_pos_out,
                  bench_nrm_in, bench_nrm_out);
    double avg_ms = (now_ms() - t0) / (double)BENCH_FRAMES;

    skin_pose_manual(bench_cv, bones, BENCH_VERTS, bench_pos_in, bench_pos_ref,
                     bench_nrm_in, bench_nrm_ref);

    int mismatch = 0;
    for (int i = 0; i < BENCH_VERTS * 3; i++) {
        if (fabsf(bench_pos_out[i] - bench_pos_ref[i]) > 1e-4f) mismatch++;
        if (fabsf(bench_nrm_out[i] - bench_nrm_ref[i]) > 1e-4f) mismatch++;
    }
    check("bench_matches_manual", mismatch == 0);
    check("bench_no_nan", all_finite(bench_pos_out, BENCH_VERTS * 3)
                          && all_finite(bench_nrm_out, BENCH_VERTS * 3)
                          && all_finite(bench_pos_ref, BENCH_VERTS * 3));

    double checksum = 0.0;
    for (int i = 0; i < BENCH_VERTS; i++) checksum += bench_pos_out[i * 3];

    printf("bench_verts=%d frames=%d compact_ms=%.4f single_frame_ms=%.4f avg_frame_ms=%.4f\n",
           BENCH_VERTS, BENCH_FRAMES, compact_ms, single_ms, avg_ms);
    printf("bench_checksum=%.3f\n", checksum);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
