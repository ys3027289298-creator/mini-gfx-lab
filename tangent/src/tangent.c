
/* mini-tangent: per-vertex tangent reconstruction for normal-mapped surfaces.

   Each triangle contributes a Lengyel tangent/bitangent divided by its UV
   determinant, so flipped-winding faces keep the correct frame; per-vertex
   averages are Gram-Schmidt orthonormalized against the geometric normal and
   stored as a 3-channel unit vector. A side array keeps the handedness sign w
   (bitangent = cross(n, t) * w). Triangles whose UVs collapse (zero UV area)
   are flagged; vertices that receive no usable tangent get a screen-space
   fallback.

   The offline renderer below shades every face interior against a hand-derived
   reference tangent frame, reports per-face error and grazing-angle darkening,
   and treats same-UV vertices on disconnected surface components as a hard
   error. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define MAX_VERTS 4096
#define MAX_FACES 8192
#define EPS_F 1e-8f
#define DEGEN_DET 1e-10f

typedef struct { float x, y, z; } Vec3;
typedef struct { float u, v; } Vec2;

typedef struct {
    Vec3 pos[MAX_VERTS];
    Vec3 nrm[MAX_VERTS];
    Vec2 uv[MAX_VERTS];
    int face[MAX_FACES][3];
    int nv, nf;
} Mesh;

/* result of tangent_build: tangents are 3-channel; sign[] is the w channel */
typedef struct {
    Vec3 tangent[MAX_VERTS];
    float sign[MAX_VERTS];          /* handedness, +1 or -1 */
    unsigned char fallback[MAX_VERTS]; /* 1 = screen-space approximation used */
    unsigned char degen_face[MAX_FACES]; /* 1 = UV area collapsed on this face */
    int n_fallback;
    int n_degen_face;
} TangentData;

#define TAN_OK 0
#define TAN_ERR_BAD_MESH -1
#define TAN_ERR_UV_DUP -2

static Vec3 vsub(Vec3 a, Vec3 b) { return (Vec3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static Vec3 vadd(Vec3 a, Vec3 b) { return (Vec3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static Vec3 vmul(Vec3 a, float s) { return (Vec3){a.x*s, a.y*s, a.z*s}; }
static float vdot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static Vec3 vcross(Vec3 a, Vec3 b) {
    return (Vec3){a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
static float vlen(Vec3 a) { return sqrtf(vdot(a, a)); }
static Vec3 vnorm(Vec3 a) {
    float l = vlen(a);
    return l > EPS_F ? vmul(a, 1.0f / l) : (Vec3){0, 0, 0};
}

/* hard check: disconnected surface points mapped to the exact same UV.
   Co-facial vertices and duplicated seam vertices at identical positions are
   unioned first, so mirrored UV seams (one connected surface) pass while two
   separate patches stacked onto one UV island are reported with indices.
   Exact float equality on both UVs and positions. */
static int uf_find(int* parent, int x) {
    while (parent[x] != x) { parent[x] = parent[parent[x]]; x = parent[x]; }
    return x;
}
static void uf_union(int* parent, int a, int b) {
    a = uf_find(parent, a); b = uf_find(parent, b);
    if (a != b) parent[b] = a;
}

static int uv_dup_find(const Mesh* m, int* groups, int* gsize, int* ngroups) {
    int parent[MAX_VERTS];
    for (int i = 0; i < m->nv; i++) parent[i] = i;
    for (int f = 0; f < m->nf; f++)
        for (int a = 0; a < 3; a++)
            for (int b = a + 1; b < 3; b++)
                uf_union(parent, m->face[f][a], m->face[f][b]);
    for (int i = 0; i < m->nv; i++)
        for (int j = i + 1; j < m->nv; j++) {
            Vec3 d = vsub(m->pos[i], m->pos[j]);
            if (d.x == 0.0f && d.y == 0.0f && d.z == 0.0f)
                uf_union(parent, i, j);
        }
    int label[MAX_VERTS];
    for (int i = 0; i < m->nv; i++) label[i] = -1;
    int ng = 0;
    for (int i = 0; i < m->nv; i++) {
        if (label[i] >= 0) continue;
        label[i] = ng;
        int n = 1;
        for (int j = i + 1; j < m->nv; j++) {
            if (label[j] < 0 &&
                uf_find(parent, i) != uf_find(parent, j) &&
                m->uv[j].u == m->uv[i].u && m->uv[j].v == m->uv[i].v) {
                label[j] = ng;
                n++;
            }
        }
        if (n > 1) {
            int k = 0;
            for (int j = i; j < m->nv && k < 8; j++)
                if (label[j] == ng) groups[ng * 8 + k++] = j;
            gsize[ng] = k;          /* indices stored; larger groups are truncated */
            ng++;
        } else {
            label[i] = -1;   /* singleton: release the label */
        }
    }
    *ngroups = ng;
    return ng > 0 ? TAN_ERR_UV_DUP : TAN_OK;
}

/* face tangent in Lengyel form: divide by the UV determinant so faces whose
   winding flips the sign of r still contribute a correctly oriented frame */
static void face_tangent(const Mesh* m, int f, Vec3* t, Vec3* b, float* det) {
    const int* ix = m->face[f];
    Vec3 p0 = m->pos[ix[0]], p1 = m->pos[ix[1]], p2 = m->pos[ix[2]];
    Vec2 w0 = m->uv[ix[0]], w1 = m->uv[ix[1]], w2 = m->uv[ix[2]];
    Vec3 e1 = vsub(p1, p0), e2 = vsub(p2, p0);
    float du1 = w1.u - w0.u, dv1 = w1.v - w0.v;
    float du2 = w2.u - w0.u, dv2 = w2.v - w0.v;
    float r = du1 * dv2 - du2 * dv1;
    *det = r;
    if (fabsf(r) <= DEGEN_DET) { *t = *b = (Vec3){0, 0, 0}; return; }
    *t = vmul(vsub(vmul(e1, dv2), vmul(e2, dv1)), 1.0f / r);
    *b = vmul(vsub(vmul(e2, du1), vmul(e1, du2)), 1.0f / r);
}

/* orthonormalize an accumulated tangent against the vertex normal */
static Vec3 ortho_tangent(Vec3 t, Vec3 n) {
    Vec3 g = vsub(t, vmul(n, vdot(n, t)));       /* Gram-Schmidt */
    return vnorm(g);
}

/* screen-space fallback: world direction of increasing screen-x, projected
   onto the tangent plane; screen-y if that collapses (edge-on view) */
static Vec3 screen_fallback_tangent(Vec3 n, Vec3 cam_right, Vec3 cam_up) {
    Vec3 t = vsub(cam_right, vmul(n, vdot(n, cam_right)));
    if (vlen(t) <= EPS_F) t = vsub(cam_up, vmul(n, vdot(n, cam_up)));
    return vnorm(t);
}

static int mesh_validate(const Mesh* m) {
    if (m->nv <= 0 || m->nv > MAX_VERTS || m->nf <= 0 || m->nf > MAX_FACES) return -1;
    for (int f = 0; f < m->nf; f++)
        for (int k = 0; k < 3; k++)
            if (m->face[f][k] < 0 || m->face[f][k] >= m->nv) return -1;
    return 0;
}

static int tangent_build(const Mesh* m, Vec3 cam_right, Vec3 cam_up, TangentData* out) {
    if (mesh_validate(m) != 0) return TAN_ERR_BAD_MESH;
    int groups[MAX_VERTS * 8], gsize[MAX_VERTS], ngroups;
    int rc = uv_dup_find(m, groups, gsize, &ngroups);
    if (rc == TAN_ERR_UV_DUP) {
        fprintf(stderr, "error: %d vertex group(s) share identical UVs:\n", ngroups);
        for (int g = 0; g < ngroups; g++) {
            fprintf(stderr, "  group %d (uv %.6f,%.6f) indices:", g,
                    m->uv[groups[g * 8]].u, m->uv[groups[g * 8]].v);
            for (int k = 0; k < gsize[g]; k++) fprintf(stderr, " %d", groups[g * 8 + k]);
            fputc('\n', stderr);
        }
        return TAN_ERR_UV_DUP;
    }
    memset(out, 0, sizeof(*out));

    Vec3 tacc[MAX_VERTS] = {0}, bacc[MAX_VERTS] = {0};
    for (int f = 0; f < m->nf; f++) {
        Vec3 t, b; float det;
        face_tangent(m, f, &t, &b, &det);
        if (fabsf(det) <= DEGEN_DET) {           /* collapsed UV triangle */
            out->degen_face[f] = 1;
            out->n_degen_face++;
            continue;
        }
        for (int k = 0; k < 3; k++) {
            int vi = m->face[f][k];
            tacc[vi] = vadd(tacc[vi], t);
            bacc[vi] = vadd(bacc[vi], b);
        }
    }

    for (int i = 0; i < m->nv; i++) {
        if (vlen(tacc[i]) <= EPS_F) {            /* no usable UV gradient */
            out->fallback[i] = 1;
            out->n_fallback++;
            out->tangent[i] = screen_fallback_tangent(m->nrm[i], cam_right, cam_up);
            out->sign[i] = 1.0f;
            continue;
        }
        Vec3 t = ortho_tangent(tacc[i], m->nrm[i]);
        Vec3 b = vcross(m->nrm[i], t);
        /* w = sign(dot(b, accumulated b)); normalized b is unit length */
        out->sign[i] = vdot(b, vnorm(bacc[i])) < 0.0f ? -1.0f : 1.0f;
        out->tangent[i] = t;
    }
    return TAN_OK;
}

/* ---- offline software shading: reconstructed vs hand-derived frame -------- */

typedef struct { Vec3 origin, right, up, forward; } Camera;

/* procedural tangent-space normal map: a fixed slanted bump so a wrong tangent
   rotates the resulting world normal visibly */
static Vec3 sample_normal_map(Vec2 uv) {
    float tx = 0.30f * cosf(uv.u * 6.2831853f);
    float ty = 0.30f * sinf(uv.v * 6.2831853f);
    return vnorm((Vec3){tx, ty, 1.0f});
}

static Vec3 world_normal_from_frame(Vec3 t, Vec3 b, Vec3 n, Vec3 tsn) {
    return vnorm(vadd(vadd(vmul(t, tsn.x), vmul(b, tsn.y)), vmul(n, tsn.z)));
}

typedef struct {
    float shade_sum, shade_max;
    float nrm_sum, nrm_max;
    int samples;
    int grazing;                    /* samples seen edge-on (n dot v < 0.25) */
    float shade_grazing_sum;
    float ref_grazing_sum;
} FaceError;
/* reference frame supplied per test mesh: hand-derived analytic tangent */
typedef Vec3 (*RefFrameFn)(const Mesh*, int vi, Vec3* out_b);

/* shade one mesh, comparing reconstructed tangents against ref_fn per sample.
   Points are sampled on a barycentric grid inside every face. */
static void render_compare(const Mesh* m, const TangentData* td,
                           const Camera* cam, Vec3 light, RefFrameFn ref_fn,
                           FaceError* fe) {
    memset(fe, 0, sizeof(FaceError) * (size_t)m->nf);
    const int GRID = 8;
    for (int f = 0; f < m->nf; f++) {
        const int* ix = m->face[f];
        for (int ia = 0; ia <= GRID; ia++) {
            for (int ib = 0; ib <= GRID - ia; ib++) {
                float l1 = ia / (float)GRID;
                float l2 = ib / (float)GRID;
                float l0 = 1.0f - l1 - l2;
                if (l0 < 0.0f) continue;
                Vec3 p = vadd(vadd(vmul(m->pos[ix[0]], l0), vmul(m->pos[ix[1]], l1)),
                              vmul(m->pos[ix[2]], l2));
                Vec3 n = vnorm(vadd(vadd(vmul(m->nrm[ix[0]], l0),
                                         vmul(m->nrm[ix[1]], l1)),
                                  vmul(m->nrm[ix[2]], l2)));
                Vec2 uv = { m->uv[ix[0]].u * l0 + m->uv[ix[1]].u * l1 + m->uv[ix[2]].u * l2,
                            m->uv[ix[0]].v * l0 + m->uv[ix[1]].v * l1 + m->uv[ix[2]].v * l2 };

                Vec3 t0 = vadd(vadd(vmul(td->tangent[ix[0]], l0),
                                    vmul(td->tangent[ix[1]], l1)),
                               vmul(td->tangent[ix[2]], l2));
                t0 = ortho_tangent(t0, n);
                float w = td->sign[ix[0]] * l0 + td->sign[ix[1]] * l1 + td->sign[ix[2]] * l2;
                Vec3 b0 = vmul(vcross(n, t0), w < 0.0f ? -1.0f : 1.0f);

                /* per-vertex analytic frames, barycentrically blended like the
                   reconstructed ones */
                Vec3 rt_a, rb_a, rt_b, rb_b, rt_c, rb_c;
                rt_a = ref_fn(m, ix[0], &rb_a);
                rt_b = ref_fn(m, ix[1], &rb_b);
                rt_c = ref_fn(m, ix[2], &rb_c);
                Vec3 rt0 = vadd(vadd(vmul(rt_a, l0), vmul(rt_b, l1)), vmul(rt_c, l2));
                Vec3 rb0 = vadd(vadd(vmul(rb_a, l0), vmul(rb_b, l1)), vmul(rb_c, l2));
                rt0 = ortho_tangent(rt0, n);
                rb0 = vnorm(vsub(rb0, vmul(n, vdot(n, rb0))));

                Vec3 tsn = sample_normal_map(uv);
                Vec3 n_got = world_normal_from_frame(t0, b0, n, tsn);
                Vec3 n_ref = world_normal_from_frame(rt0, rb0, n, tsn);

                Vec3 view = vnorm(vsub(cam->origin, p));
                float l_got = fmaxf(vdot(n_got, light), 0.0f);
                float l_ref = fmaxf(vdot(n_ref, light), 0.0f);
                float de = fabsf(l_got - l_ref);
                float ne = acosf(fminf(1.0f, fmaxf(-1.0f, vdot(n_got, n_ref))));

                fe[f].shade_sum += de;
                fe[f].shade_max = fmaxf(fe[f].shade_max, de);
                fe[f].nrm_sum += ne;
                fe[f].nrm_max = fmaxf(fe[f].nrm_max, ne);
                fe[f].samples++;
                float ndv = fmaxf(vdot(n, view), 0.0f);
                if (ndv < 0.25f) {
                    fe[f].grazing++;
                    fe[f].shade_grazing_sum += l_got;
                    fe[f].ref_grazing_sum += l_ref;
                }
            }
        }
    }
}

/* control experiment: shuffle tangents to reproduce the "black at grazing
   angle" symptom from garbled tangent directions */
static void render_garbled(const Mesh* m, const TangentData* good,
                           const Camera* cam, Vec3 light, RefFrameFn ref_fn,
                           FaceError* fe) {
    TangentData bad = *good;
    unsigned int seed = 123456789u;
    for (int i = 0; i < m->nv; i++) {
        seed = seed * 1103515245u + 12345u;
        float ang = (seed >> 8) / 4294967296.0f * 6.2831853f;
        Vec3 n = m->nrm[i];
        Vec3 helper = fabsf(n.z) < 0.9f ? (Vec3){0, 0, 1} : (Vec3){1, 0, 0};
        Vec3 t = vnorm(vcross(n, helper));
        Vec3 b = vcross(n, t);
        bad.tangent[i] = vnorm(vadd(vmul(t, cosf(ang)), vmul(b, sinf(ang))));
        bad.sign[i] = 1.0f;
        bad.fallback[i] = 0;
    }
    render_compare(m, &bad, cam, light, ref_fn, fe);
}

/* ---- test meshes ---------------------------------------------------------- */

/* flat grid in the XY plane, normal +z, UV = position. analytic tangent +x */
static void build_plane(Mesh* m, int cols, int rows, float size) {
    memset(m, 0, sizeof(*m));
    for (int j = 0; j <= rows; j++) {
        for (int i = 0; i <= cols; i++) {
            int vi = m->nv;
            m->pos[vi] = (Vec3){-size + 2*size*i/cols, -size + 2*size*j/rows, 0};
            m->nrm[vi] = (Vec3){0, 0, 1};
            m->uv[vi] = (Vec2){(float)i / cols, (float)j / rows};
            m->nv++;
        }
    }
    for (int j = 0; j < rows; j++) {
        for (int i = 0; i < cols; i++) {
            int a = j * (cols + 1) + i, b = a + 1, c = a + cols + 1, d = c + 1;
            m->face[m->nf][0] = a; m->face[m->nf][1] = c; m->face[m->nf][2] = b; m->nf++;
            m->face[m->nf][0] = b; m->face[m->nf][1] = c; m->face[m->nf][2] = d; m->nf++;
        }
    }
}

static Vec3 ref_plane(const Mesh* m, int vi, Vec3* out_b) {
    (void)m; (void)vi;
    if (out_b) *out_b = (Vec3){0, 1, 0};
    return (Vec3){1, 0, 0};
}

/* two quads sharing the x=0 edge as split (duplicated) vertices. The right
   quad has mirrored UVs, so seam-adjacent faces disagree on tangent direction
   while the duplicated seam positions carry different UVs: a genuine seam. */
static void build_seam_plane(Mesh* m) {
    memset(m, 0, sizeof(*m));
    /* 0,1 left quad edge pair; 2,3 seam pair (left side); 4,5 seam pair
       (right side); 6,7 right outer edge */
    m->nv = 8;
    m->pos[0] = (Vec3){-1, -1, 0}; m->uv[0] = (Vec2){0, 0};
    m->pos[1] = (Vec3){-1,  1, 0}; m->uv[1] = (Vec2){0, 1};
    m->pos[2] = (Vec3){ 0, -1, 0}; m->uv[2] = (Vec2){1, 0};
    m->pos[3] = (Vec3){ 0,  1, 0}; m->uv[3] = (Vec2){1, 1};
    m->pos[4] = (Vec3){ 0, -1, 0}; m->uv[4] = (Vec2){1, 0};
    m->pos[5] = (Vec3){ 0,  1, 0}; m->uv[5] = (Vec2){1, 1};
    m->pos[6] = (Vec3){ 1, -1, 0}; m->uv[6] = (Vec2){0, 0};
    m->pos[7] = (Vec3){ 1,  1, 0}; m->uv[7] = (Vec2){0, 1};
    for (int i = 0; i < m->nv; i++) m->nrm[i] = (Vec3){0, 0, 1};
    m->face[0][0] = 0; m->face[0][1] = 1; m->face[0][2] = 2; m->nf = 1;
    m->face[1][0] = 1; m->face[1][1] = 3; m->face[1][2] = 2; m->nf = 2;
    m->face[2][0] = 4; m->face[2][1] = 5; m->face[2][2] = 6; m->nf = 3;
    m->face[3][0] = 5; m->face[3][1] = 7; m->face[3][2] = 6; m->nf = 4;
}

/* cylinder around the z-axis; analytic frame from the angle theta = 2*pi*u */
static void build_cylinder(Mesh* m, int seg, float h) {
    memset(m, 0, sizeof(*m));
    float r = 1.0f;
    for (int ring = 0; ring < 2; ring++) {
        for (int s = 0; s <= seg; s++) {
            float th = 6.2831853f * s / seg;
            int vi = m->nv;
            m->pos[vi] = (Vec3){r * cosf(th), r * sinf(th), ring ? h : -h};
            m->nrm[vi] = (Vec3){cosf(th), sinf(th), 0};
            m->uv[vi] = (Vec2){(float)s / seg, (float)ring};
            m->nv++;
        }
    }
    int base = seg + 1;
    for (int s = 0; s < seg; s++) {
        int a = s, b = s + 1, c = base + s, d = base + s + 1;
        m->face[m->nf][0] = a; m->face[m->nf][1] = c; m->face[m->nf][2] = b; m->nf++;
        m->face[m->nf][0] = b; m->face[m->nf][1] = c; m->face[m->nf][2] = d; m->nf++;
    }
}

static Vec3 ref_cylinder(const Mesh* m, int vi, Vec3* out_b) {
    Vec3 n = m->nrm[vi];
    /* hand-derived frame: t = dP/du along theta, b = dP/dv along +z, and
       cross(n, t) = +z so handedness is +1 */
    if (out_b) *out_b = (Vec3){0, 0, 1};
    return (Vec3){-n.y, n.x, 0};
}

/* one good triangle plus one whose UVs collapse: vertex 3 is pinned onto the
   same UV as vertex 1, giving face 1 zero UV area without a topology error
   (1 and 3 are never co-facial in a valid triangle) */
static void build_degen_uv(Mesh* m) {
    memset(m, 0, sizeof(*m));
    m->nv = 4;
    m->pos[0] = (Vec3){-1, -1, 0}; m->nrm[0] = (Vec3){0,0,1}; m->uv[0] = (Vec2){0,0};
    m->pos[1] = (Vec3){ 0, -1, 0}; m->nrm[1] = (Vec3){0,0,1}; m->uv[1] = (Vec2){1,0};
    m->pos[2] = (Vec3){ 0,  1, 0}; m->nrm[2] = (Vec3){0,0,1}; m->uv[2] = (Vec2){0,1};
    m->pos[3] = (Vec3){ 1,  1, 0}; m->nrm[3] = (Vec3){0,0,1}; m->uv[3] = (Vec2){1,0};
    m->face[0][0] = 0; m->face[0][1] = 2; m->face[0][2] = 1; m->nf = 1;
    m->face[1][0] = 1; m->face[1][1] = 2; m->face[1][2] = 3; m->nf = 2;
}

/* two disconnected triangles; vertex 4 sits at a different position but is
   pinned to the exact same UV as vertex 0 -> hard error */
static void build_dup_uv(Mesh* m) {
    memset(m, 0, sizeof(*m));
    m->nv = 6;
    m->pos[0] = (Vec3){-1, -1, 0}; m->uv[0] = (Vec2){0.0f, 0.0f};
    m->pos[1] = (Vec3){ 0, -1, 0}; m->uv[1] = (Vec2){1.0f, 0.0f};
    m->pos[2] = (Vec3){-1,  1, 0}; m->uv[2] = (Vec2){0.0f, 1.0f};
    m->pos[3] = (Vec3){ 1,  1, 0}; m->uv[3] = (Vec2){1.0f, 1.0f};
    m->pos[4] = (Vec3){ 2,  1, 0}; m->uv[4] = (Vec2){0.0f, 0.0f};  /* dup of 0 */
    m->pos[5] = (Vec3){ 2,  2, 0}; m->uv[5] = (Vec2){1.0f, 0.0f};
    for (int i = 0; i < m->nv; i++) m->nrm[i] = (Vec3){0, 0, 1};
    m->face[0][0] = 0; m->face[0][1] = 2; m->face[0][2] = 1; m->nf = 1;
    m->face[1][0] = 3; m->face[1][1] = 4; m->face[1][2] = 5; m->nf = 2;
}

/* ---- test harness --------------------------------------------------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void report_faces(const char* tag, const Mesh* m, const FaceError* fe,
                         float* max_mean, int* seam_faces) {
    *max_mean = 0.0f; *seam_faces = 0;
    for (int f = 0; f < m->nf; f++) {
        float mean = fe[f].shade_sum / (float)fe[f].samples;
        if (mean > *max_mean) *max_mean = mean;
        int seam = fe[f].nrm_max > 0.05f;     /* >~2.9 deg normal spread */
        if (seam) (*seam_faces)++;
        if (seam || f < 2)
            printf("  %s face %d: shade_mean=%.5f shade_max=%.5f nrm_max_deg=%.3f%s\n",
                   tag, f, mean, fe[f].shade_max,
                   fe[f].nrm_max * 57.29578f, seam ? " SEAM" : "");
    }
}

/* aggregate-only report for noisy control experiments */
static void report_summary(const char* tag, const Mesh* m, const FaceError* fe,
                           float* max_mean) {
    *max_mean = 0.0f;
    float worst_nrm = 0.0f;
    for (int f = 0; f < m->nf; f++) {
        float mean = fe[f].shade_sum / (float)fe[f].samples;
        if (mean > *max_mean) *max_mean = mean;
        if (fe[f].nrm_max > worst_nrm) worst_nrm = fe[f].nrm_max;
    }
    printf("  %s summary: worst_shade_mean=%.5f worst_nrm_deg=%.3f\n",
           tag, *max_mean, worst_nrm * 57.29578f);
}

int main(void) {
    Camera cam = {
        .origin = (Vec3){0, 0, 6}, .right = (Vec3){1, 0, 0},
        .up = (Vec3){0, 1, 0}, .forward = (Vec3){0, 0, -1}
    };
    Vec3 light = vnorm((Vec3){0.4f, 0.5f, 0.75f});

    /* 1. clean plane: reconstruction must match the hand-derived frame */
    Mesh plane; build_plane(&plane, 4, 4, 1.0f);
    TangentData td;
    int rc = tangent_build(&plane, cam.right, cam.up, &td);
    check("plane_build_ok", rc == TAN_OK);
    check("plane_no_fallback", td.n_fallback == 0 && td.n_degen_face == 0);
    int unit_ok = 1, ortho_ok = 1;
    for (int i = 0; i < plane.nv; i++) {
        if (fabsf(vlen(td.tangent[i]) - 1.0f) > 1e-6f) unit_ok = 0;
        if (fabsf(vdot(td.tangent[i], plane.nrm[i])) > 1e-6f) ortho_ok = 0;
    }
    check("plane_tangent_unit", unit_ok);
    check("plane_tangent_orthonormal", ortho_ok);
    FaceError fe[MAX_FACES];
    render_compare(&plane, &td, &cam, light, ref_plane, fe);
    float max_mean; int seam;
    report_faces("plane", &plane, fe, &max_mean, &seam);
    check("plane_pixel_match", max_mean < 1e-5f);
    check("plane_no_seam", seam == 0);

    /* 2. UV seam plane: seam-adjacent faces must report the tangent split */
    Mesh sp; build_seam_plane(&sp);
    TangentData tsd;
    rc = tangent_build(&sp, cam.right, cam.up, &tsd);
    check("seam_build_ok", rc == TAN_OK);
    FaceError sfe[MAX_FACES];
    render_compare(&sp, &tsd, &cam, light, ref_plane, sfe);
    report_faces("seam", &sp, sfe, &max_mean, &seam);
    check("seam_faces_reported", seam >= 2);

    /* 3. cylinder: curved surface, grazing-angle view must stay lit */
    Mesh cyl; build_cylinder(&cyl, 12, 1.0f);
    TangentData ctd;
    rc = tangent_build(&cyl, cam.right, cam.up, &ctd);
    check("cyl_build_ok", rc == TAN_OK);
    FaceError cfe[MAX_FACES];
    render_compare(&cyl, &ctd, &cam, light, ref_cylinder, cfe);
    report_faces("cyl", &cyl, cfe, &max_mean, &seam);
    check("cyl_pixel_match", max_mean < 1e-4f);
    int grazed = 0;
    float got_g = 0.0f, ref_g = 0.0f;
    for (int f = 0; f < cyl.nf; f++) {
        if (cfe[f].grazing > 0) {
            grazed++;
            got_g += cfe[f].shade_grazing_sum;
            ref_g += cfe[f].ref_grazing_sum;
        }
    }
    printf("  cyl grazing faces=%d got=%.4f ref=%.4f\n",
           grazed, got_g, ref_g);
    check("cyl_grazing_not_black", grazed > 0 && got_g > 0.05f &&
          fabsf(got_g - ref_g) / ref_g < 0.02f);

    /* garbled-tangent control: same mesh, scrambled tangents go dark/astray */
    FaceError gfe[MAX_FACES];
    render_garbled(&cyl, &ctd, &cam, light, ref_cylinder, gfe);
    float gmax = 0.0f;
    report_summary("garbled", &cyl, gfe, &gmax);
    check("garbled_tangent_mismatch", gmax > 0.1f && gmax > max_mean * 10.0f);

    /* 4. degenerate UV face: flagged, its vertices fall back to screen space */
    Mesh dg; build_degen_uv(&dg);
    TangentData dtd;
    rc = tangent_build(&dg, cam.right, cam.up, &dtd);
    check("degen_build_ok", rc == TAN_OK);
    check("degen_face_flagged", dtd.n_degen_face == 1 && dtd.degen_face[1] == 1);
    /* only vertex 3 belongs to no valid-UV face; 1 and 2 are shared with
       the good face and keep regular tangents */
    check("degen_vertex_fallback", dtd.fallback[3] && !dtd.fallback[0] &&
          !dtd.fallback[1] && !dtd.fallback[2]);
    int fb_unit = 1, fb_ortho = 1, fb_axis = 1;
    for (int i = 0; i < dg.nv; i++) {
        if (!dtd.fallback[i]) continue;
        if (fabsf(vlen(dtd.tangent[i]) - 1.0f) > 1e-6f) fb_unit = 0;
        if (fabsf(vdot(dtd.tangent[i], dg.nrm[i])) > 1e-6f) fb_ortho = 0;
        /* camera looks down -z at a +z wall: screen-right is world +x */
        if (fabsf(dtd.tangent[i].x - 1.0f) > 1e-6f) fb_axis = 0;
    }
    check("fallback_unit_orthonormal", fb_unit && fb_ortho);
    check("fallback_screen_right", fb_axis);

    /* 5. identical UVs: hard error with vertex indices listed */
    Mesh dup; build_dup_uv(&dup);
    TangentData dud;
    rc = tangent_build(&dup, cam.right, cam.up, &dud);
    check("dup_uv_rejected", rc == TAN_ERR_UV_DUP);

    /* 6. malformed mesh: out-of-range face index */
    Mesh bad = plane;
    bad.face[0][0] = plane.nv + 5;
    TangentData bud;
    check("bad_mesh_rejected",
          tangent_build(&bad, cam.right, cam.up, &bud) == TAN_ERR_BAD_MESH);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
