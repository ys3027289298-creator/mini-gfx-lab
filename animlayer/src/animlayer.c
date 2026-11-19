
/* animlayer: layered skeleton animation with per-bone masks.
   Upper/lower body clips are overlaid per bone; per-bone weights are
   normalized before blending and transitions interpolate every bone
   (quaternion slerp + translation lerp). When every mask contributing to
   a bone is zero the base layer passes through, so normalization never
   divides by zero and no NaN is produced. */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_BONES 32
#define MAX_LAYERS 4
#define EPS 1e-8f

static const float PI = 3.14159265358979323846f;

typedef struct {
    float q[4];   /* local rotation quaternion, x,y,z,w */
    float t[3];   /* local translation */
} BonePose;

typedef struct {
    BonePose bone[MAX_BONES];
} Pose;

typedef struct {
    float weight;           /* layer weight, >= 0 */
    float mask[MAX_BONES];  /* per-bone mask, 0..1 */
    const Pose* pose;       /* sampled clip pose */
} Layer;

/* ---- quaternion math ---- */

static float q_dot(const float* a, const float* b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

static void q_normalize(float* q) {
    float n = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n <= EPS) { q[0] = q[1] = q[2] = 0.0f; q[3] = 1.0f; return; }
    float inv = 1.0f / n;
    for (int i = 0; i < 4; i++) q[i] *= inv;
}

static void q_axis_angle(float* q, float x, float y, float z, float angle) {
    float h = angle * 0.5f;
    float s = sinf(h);
    q[0] = x * s;
    q[1] = y * s;
    q[2] = z * s;
    q[3] = cosf(h);
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* spherical interpolation; falls back to normalized lerp near parallel */
static void q_slerp(const float* a, const float* b, float u, float* out) {
    float aa[4], bb[4];
    memcpy(aa, a, sizeof(aa));
    memcpy(bb, b, sizeof(bb));
    q_normalize(aa);
    q_normalize(bb);
    float d = q_dot(aa, bb);
    if (d < 0.0f) {                 /* keep quaternions in one hemisphere */
        d = -d;
        for (int i = 0; i < 4; i++) bb[i] = -bb[i];
    }
    if (d > 0.9995f) {
        for (int i = 0; i < 4; i++) out[i] = aa[i] + (bb[i] - aa[i]) * u;
        q_normalize(out);
        return;
    }
    float theta = acosf(clampf(d, -1.0f, 1.0f));
    float sin_th = sinf(theta);
    float w0 = sinf((1.0f - u) * theta) / sin_th;
    float w1 = sinf(u * theta) / sin_th;
    for (int i = 0; i < 4; i++) out[i] = w0 * aa[i] + w1 * bb[i];
    q_normalize(out);
}

/* ---- layered evaluation ---- */

/* For each bone, effective layer weights are weight*mask and normalized by
   their sum. A zero sum means no layer claims the bone: the base layer pose
   passes through unchanged instead of dividing by zero. */
static void pose_layered(const Layer* layers, int n, int bones, Pose* out) {
    for (int b = 0; b < bones; b++) {
        float wsum = 0.0f;
        for (int l = 0; l < n; l++)
            wsum += layers[l].weight * layers[l].mask[b];

        BonePose* o = &out->bone[b];
        if (wsum <= EPS) {
            *o = layers[0].pose->bone[b];
            continue;
        }

        float acc_q[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
        float acc_t[3] = { 0.0f, 0.0f, 0.0f };
        const float* ref = layers[0].pose->bone[b].q;
        for (int l = 0; l < n; l++) {
            float e = layers[l].weight * layers[l].mask[b] / wsum;
            const BonePose* s = &layers[l].pose->bone[b];
            float flip = q_dot(ref, s->q) < 0.0f ? -1.0f : 1.0f;
            for (int i = 0; i < 4; i++) acc_q[i] += e * s->q[i] * flip;
            for (int i = 0; i < 3; i++) acc_t[i] += e * s->t[i];
        }
        q_normalize(acc_q);
        memcpy(o->q, acc_q, sizeof(acc_q));
        memcpy(o->t, acc_t, sizeof(acc_t));
    }
}

/* ---- per-bone transition ---- */

typedef struct {
    Pose from;
    float alpha;      /* 0..1 */
    float duration;
    int active;
} Transition;

static void trans_start(Transition* tr, const Pose* from, float duration) {
    tr->from = *from;
    tr->alpha = 0.0f;
    tr->duration = duration > 0.0f ? duration : 0.0f;
    tr->active = 1;
}

static void trans_advance(Transition* tr, float dt) {
    if (!tr->active) return;
    if (tr->duration <= EPS) {
        tr->alpha = 1.0f;
        tr->active = 0;
        return;
    }
    tr->alpha += dt / tr->duration;
    if (tr->alpha >= 1.0f) {
        tr->alpha = 1.0f;
        tr->active = 0;
    }
}

/* Blends the cached start pose toward the current target per bone. Once the
   transition has finished the target is copied verbatim, i.e. the result is
   exactly the single evaluated layer stack. */
static void trans_apply(const Transition* tr, const Pose* target,
                        int bones, Pose* out) {
    if (!tr->active || tr->alpha >= 1.0f) {
        *out = *target;
        return;
    }
    for (int b = 0; b < bones; b++) {
        q_slerp(tr->from.bone[b].q, target->bone[b].q,
                tr->alpha, out->bone[b].q);
        for (int i = 0; i < 3; i++)
            out->bone[b].t[i] = tr->from.bone[b].t[i]
                + (target->bone[b].t[i] - tr->from.bone[b].t[i]) * tr->alpha;
    }
}

/* ---- test helpers ---- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void fill_pose(Pose* p, int bones, float seed) {
    for (int b = 0; b < bones; b++) {
        float ang = seed + 0.13f * (float)b;
        q_axis_angle(p->bone[b].q, 0.0f, 1.0f, 0.0f, ang);
        p->bone[b].t[0] = seed * 1.5f + (float)b;
        p->bone[b].t[1] = seed * 0.7f - (float)b * 0.5f;
        p->bone[b].t[2] = seed + (float)b * 0.25f;
    }
}

static int vec_close(const float* a, const float* b, int n, float tol) {
    for (int i = 0; i < n; i++)
        if (fabsf(a[i] - b[i]) > tol) return 0;
    return 1;
}

/* rotations match if quaternions coincide up to sign */
static int bone_close(const BonePose* a, const BonePose* b, float tol) {
    float d = fabsf(q_dot(a->q, b->q));
    return fabsf(d - 1.0f) < tol && vec_close(a->t, b->t, 3, tol);
}

static int pose_close(const Pose* a, const Pose* b, int bones, float tol) {
    for (int i = 0; i < bones; i++)
        if (!bone_close(&a->bone[i], &b->bone[i], tol)) return 0;
    return 1;
}

static int pose_finite(const Pose* p, int bones) {
    for (int b = 0; b < bones; b++) {
        for (int i = 0; i < 4; i++) if (!isfinite(p->bone[b].q[i])) return 0;
        for (int i = 0; i < 3; i++) if (!isfinite(p->bone[b].t[i])) return 0;
    }
    return 1;
}

static int q_unit(const float* q) {
    return fabsf(q_dot(q, q) - 1.0f) < 1e-6f;
}

/* independent hand-written two-layer per-bone blend */
static void manual_blend2(const Layer* l0, const Layer* l1,
                          int b, BonePose* out) {
    float e0 = l0->weight * l0->mask[b];
    float e1 = l1->weight * l1->mask[b];
    float s = e0 + e1;
    e0 /= s;
    e1 /= s;
    const BonePose* a = &l0->pose->bone[b];
    const BonePose* c = &l1->pose->bone[b];
    float flip = q_dot(a->q, c->q) < 0.0f ? -1.0f : 1.0f;
    for (int i = 0; i < 4; i++)
        out->q[i] = e0 * a->q[i] + e1 * c->q[i] * flip;
    q_normalize(out->q);
    for (int i = 0; i < 3; i++)
        out->t[i] = e0 * a->t[i] + e1 * c->t[i];
}

int main(void) {
    enum { BONES = 7 };
    /* 0 pelvis, 1 spine, 2 head, 3 armL, 4 armR, 5 legL, 6 legR */
    Pose base, over, third, out;
    fill_pose(&base, BONES, 0.3f);
    fill_pose(&over, BONES, 2.1f);
    fill_pose(&third, BONES, 4.7f);

    Layer layers[3];
    memset(layers, 0, sizeof(layers));
    layers[0].weight = 1.0f;
    layers[0].pose = &base;
    for (int b = 0; b < BONES; b++) layers[0].mask[b] = 1.0f;
    layers[1].weight = 0.7f;
    layers[1].pose = &over;
    float upper[BONES];
    for (int b = 0; b < BONES; b++) upper[b] = 0.0f;
    upper[1] = upper[2] = upper[3] = upper[4] = 1.0f;  /* spine up, no legs/pelvis */
    for (int b = 0; b < BONES; b++) layers[1].mask[b] = upper[b];
    layers[2].weight = 0.5f;
    layers[2].pose = &third;                         /* arm-only extra layer */
    layers[2].mask[3] = 1.0f;

    /* layered output must equal a hand-written per-bone blend */
    pose_layered(layers, 2, BONES, &out);
    int manual_ok = 1;
    for (int b = 0; b < BONES; b++) {
        BonePose expect;
        manual_blend2(&layers[0], &layers[1], b, &expect);
        if (!bone_close(&out.bone[b], &expect, 1e-5f)) manual_ok = 0;
    }
    check("layer_matches_manual", manual_ok);

    /* masked-out bones keep the base pose exactly (upper/lower no longer fight) */
    int lower_ok = 1;
    for (int b = 0; b < BONES; b++)
        if (upper[b] == 0.0f && !bone_close(&out.bone[b], &base.bone[b], 1e-6f))
            lower_ok = 0;
    check("lower_body_untouched", lower_ok);

    /* all-zero mask: no NaN/Inf, base layer passes through */
    Layer zeroed[2];
    memcpy(zeroed, layers, sizeof(zeroed));
    for (int l = 0; l < 2; l++)
        for (int b = 0; b < BONES; b++) zeroed[l].mask[b] = 0.0f;
    pose_layered(zeroed, 2, BONES, &out);
    check("zero_mask_no_nan", pose_finite(&out, BONES));
    check("zero_mask_base_passthrough", pose_close(&out, &base, BONES, 1e-7f));

    /* weights need not sum to 1: normalization makes the result invariant */
    layers[0].weight = 2.0f;
    layers[1].weight = 3.0f;
    pose_layered(layers, 2, BONES, &out);
    int norm_ok = 1;
    for (int b = 0; b < BONES; b++) {
        BonePose expect;
        manual_blend2(&layers[0], &layers[1], b, &expect);
        if (!bone_close(&out.bone[b], &expect, 1e-5f)) norm_ok = 0;
        if (!q_unit(out.bone[b].q)) norm_ok = 0;
    }
    check("normalized_weights_match_manual", norm_ok);
    layers[0].weight = 1.0f;
    layers[1].weight = 0.7f;

    /* three contributors on one bone, normalized against their sum */
    pose_layered(layers, 3, BONES, &out);
    {
        int b = 3;
        float e0 = layers[0].weight * layers[0].mask[b];
        float e1 = layers[1].weight * layers[1].mask[b];
        float e2 = layers[2].weight * layers[2].mask[b];
        float s = e0 + e1 + e2;
        const BonePose* p0 = &layers[0].pose->bone[b];
        const BonePose* p1 = &layers[1].pose->bone[b];
        const BonePose* p2 = &layers[2].pose->bone[b];
        float f1 = q_dot(p0->q, p1->q) < 0.0f ? -1.0f : 1.0f;
        float f2 = q_dot(p0->q, p2->q) < 0.0f ? -1.0f : 1.0f;
        BonePose expect;
        for (int i = 0; i < 4; i++)
            expect.q[i] = (e0 * p0->q[i] + e1 * p1->q[i] * f1
                           + e2 * p2->q[i] * f2) / s;
        q_normalize(expect.q);
        for (int i = 0; i < 3; i++)
            expect.t[i] = (e0 * p0->t[i] + e1 * p1->t[i] + e2 * p2->t[i]) / s;
        check("three_layer_manual", bone_close(&out.bone[b], &expect, 1e-5f));
    }

    /* slerp at u=0.5 of identity and 90deg is 45deg */
    {
        float qa[4] = { 0.0f, 0.0f, 0.0f, 1.0f };
        float qb[4], qm[4];
        q_axis_angle(qb, 0.0f, 1.0f, 0.0f, PI * 0.5f);
        q_slerp(qa, qb, 0.5f, qm);
        check("slerp_half_angle",
              fabsf(qm[1] - sinf(PI / 8.0f)) < 1e-6f &&
              fabsf(qm[3] - cosf(PI / 8.0f)) < 1e-6f &&
              fabsf(qm[0]) < 1e-6f && fabsf(qm[2]) < 1e-6f);
    }

    /* transition: per-bone slerp/lerp, exact endpoints, single layer at end */
    Pose from, target;
    for (int b = 0; b < BONES; b++) {
        from.bone[b].q[0] = from.bone[b].q[1] = from.bone[b].q[2] = 0.0f;
        from.bone[b].q[3] = 1.0f;
        for (int i = 0; i < 3; i++) from.bone[b].t[i] = 0.0f;
        q_axis_angle(target.bone[b].q, 0.0f, 1.0f, 0.0f, PI * 0.5f);
        target.bone[b].t[0] = 2.0f;
        target.bone[b].t[1] = 4.0f;
        target.bone[b].t[2] = 6.0f;
    }
    Transition tr;
    trans_start(&tr, &from, 1.0f);
    trans_apply(&tr, &target, BONES, &out);
    check("trans_alpha0_is_from", pose_close(&out, &from, BONES, 1e-6f));

    trans_advance(&tr, 0.5f);
    trans_apply(&tr, &target, BONES, &out);
    int half_ok = 1;
    for (int b = 0; b < BONES; b++) {
        if (fabsf(out.bone[b].q[1] - sinf(PI / 8.0f)) > 1e-6f ||
            fabsf(out.bone[b].q[3] - cosf(PI / 8.0f)) > 1e-6f)
            half_ok = 0;
        if (!vec_close(out.bone[b].t, (float[3]){ 1.0f, 2.0f, 3.0f }, 3, 1e-6f))
            half_ok = 0;
    }
    check("trans_half_per_bone", half_ok);
    check("trans_no_nan", pose_finite(&out, BONES));

    trans_advance(&tr, 0.5f);
    check("transition_finishes_flag", tr.active == 0 && tr.alpha == 1.0f);
    trans_apply(&tr, &target, BONES, &out);
    check("trans_end_is_target", pose_close(&out, &target, BONES, 1e-6f));
    trans_apply(&tr, &target, BONES, &out);
    check("trans_reapply_stable", pose_close(&out, &target, BONES, 1e-7f));

    /* overlay fades out: at the end only the base (single) layer remains */
    layers[1].weight = 1.0f;
    pose_layered(layers, 2, BONES, &from);
    Transition fade;
    trans_start(&fade, &from, 1.0f);
    layers[1].weight = 0.0f;
    pose_layered(layers, 2, BONES, &target);
    check("fadeout_eval_is_base", pose_close(&target, &base, BONES, 1e-6f));
    trans_advance(&fade, 1.0f);
    trans_apply(&fade, &target, BONES, &out);
    check("transition_returns_to_single_layer",
          fade.active == 0 && pose_close(&out, &base, BONES, 1e-6f));

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
