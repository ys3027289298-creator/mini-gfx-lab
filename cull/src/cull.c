#define _POSIX_C_SOURCE 199309L
/* screen-cull: screen-space particle culling with a per-frame render budget.
   Pass 1 drops particles that are off-screen or sub-visible (projected
   size x alpha cannot move any pixel by half an LSB). Pass 2 sheds the
   least important survivors via quickselect when over budget. Every drop
   is counted per reason. Self-test renders culled vs full sets into a
   software framebuffer and requires <1% differing pixels, and times the
   cull pass against a 0.2 ms/frame budget. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define VIEW_W 320
#define VIEW_H 180
#define MAX_PARTICLES 32768
#define RENDER_BUDGET 5000
#define MIN_CONTRIB (0.5f / 255.0f)  /* peak pixel contribution below half an LSB */
#define DIFF_TOL (4.0f / 255.0f)
#define DIFF_BUDGET_PCT 1.0
#define CULL_TIME_BUDGET_MS 0.2

typedef struct { float x, y, r, alpha, priority; } Particle;
typedef struct { float x, y, zoom; } Camera;

typedef struct {
    int total, rendered;
    int culled_offscreen, culled_subvisible, dropped_budget;
} CullStats;

static int g_idx[MAX_PARTICLES];
static float g_imp[MAX_PARTICLES];
static unsigned char g_keep[MAX_PARTICLES];

static int cull_frame(const Particle* ps, int n, const Camera* cam, int budget,
                      CullStats* st) {
    memset(st, 0, sizeof(*st));
    st->total = n;
    memset(g_keep, 0, (size_t)n);
    const float hw = VIEW_W * 0.5f, hh = VIEW_H * 0.5f;
    int nc = 0;
    for (int i = 0; i < n; i++) {
        const Particle* p = &ps[i];
        float pr = p->r * cam->zoom;                       /* projected radius, px */
        float sx = (p->x - cam->x) * cam->zoom + hw;
        float sy = (p->y - cam->y) * cam->zoom + hh;
        if (sx + pr < 0.0f || sx - pr >= (float)VIEW_W ||
            sy + pr < 0.0f || sy - pr >= (float)VIEW_H) {
            st->culled_offscreen++;
            continue;
        }
        /* sub-visible: even a perfect hit moves no pixel by half an LSB */
        if (p->alpha * (pr + 0.5f) < MIN_CONTRIB) {
            st->culled_subvisible++;
            continue;
        }
        g_idx[nc] = i;
        g_imp[nc] = p->priority * p->alpha * pr * pr;      /* importance ~ lit area */
        nc++;
    }
    if (nc > budget) {
        /* quickselect: partition so the `budget` largest importances lead,
           O(n) average — no full sort on the hot path */
        int lo = 0, hi = nc - 1, k = budget;
        while (lo < hi) {
            float piv = g_imp[(lo + hi) >> 1];
            int i = lo, j = hi;
            do {
                while (g_imp[i] > piv) i++;
                while (g_imp[j] < piv) j--;
                if (i <= j) {
                    float tf = g_imp[i]; g_imp[i] = g_imp[j]; g_imp[j] = tf;
                    int ti = g_idx[i]; g_idx[i] = g_idx[j]; g_idx[j] = ti;
                    i++; j--;
                }
            } while (i <= j);
            if (j < k) lo = i;
            if (k < i) hi = j;
        }
        st->dropped_budget = nc - budget;
        nc = budget;
    }
    for (int c = 0; c < nc; c++) g_keep[g_idx[c]] = 1;
    st->rendered = nc;
    return nc;
}

/* software renderer: alpha-blended circles toward white, linear AA coverage */
static void render(const Particle* ps, int n, const Camera* cam,
                   const unsigned char* mask, float* fb) {
    const float hw = VIEW_W * 0.5f, hh = VIEW_H * 0.5f;
    for (int i = 0; i < n; i++) {
        if (mask && !mask[i]) continue;
        const Particle* p = &ps[i];
        float pr = p->r * cam->zoom;
        float sx = (p->x - cam->x) * cam->zoom + hw;
        float sy = (p->y - cam->y) * cam->zoom + hh;
        int x0 = (int)floorf(sx - pr), x1 = (int)ceilf(sx + pr);
        int y0 = (int)floorf(sy - pr), y1 = (int)ceilf(sy + pr);
        if (x1 < 0 || x0 >= VIEW_W || y1 < 0 || y0 >= VIEW_H) continue;
        if (x0 < 0) x0 = 0;
        if (x1 >= VIEW_W) x1 = VIEW_W - 1;
        if (y0 < 0) y0 = 0;
        if (y1 >= VIEW_H) y1 = VIEW_H - 1;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++) {
                float dx = x + 0.5f - sx, dy = y + 0.5f - sy;
                float cov = pr + 0.5f - sqrtf(dx * dx + dy * dy);
                if (cov <= 0.0f) continue;
                if (cov > 1.0f) cov = 1.0f;
                float a = p->alpha * cov;
                float* d = &fb[y * VIEW_W + x];
                *d += (1.0f - *d) * a;
            }
    }
}

static unsigned g_rng = 0x9e3779b9u;
static float frand(void) {
    g_rng ^= g_rng << 13; g_rng ^= g_rng >> 17; g_rng ^= g_rng << 5;
    return (float)(g_rng >> 8) * (1.0f / 16777216.0f);
}
static float frange(float a, float b) { return a + (b - a) * frand(); }

int main(void) {
    static Particle ps[MAX_PARTICLES];
    int n = 0;
    /* heroes: large, bright, on-screen — must survive every cut */
    for (int i = 0; i < 4000; i++)
        ps[n++] = (Particle){ frange(-160, 160), frange(-90, 90),
                              frange(2, 8), frange(0.5f, 1.0f), frange(1, 3) };
    /* off-screen crowd: pure waste if rendered */
    for (int i = 0; i < 8000; i++) {
        float x, y;
        do { x = frange(-2000, 2000); y = frange(-2000, 2000); }
        while (x > -180 && x < 180 && y > -110 && y < 110);
        ps[n++] = (Particle){ x, y, frange(1, 4), frange(0.5f, 1.0f), 1.0f };
    }
    /* sub-visible dust: on-screen but below half an LSB */
    for (int i = 0; i < 6000; i++)
        ps[n++] = (Particle){ frange(-160, 160), frange(-90, 90),
                              frange(0.02f, 0.2f), frange(0.0005f, 0.002f), 1.0f };
    /* filler haze: each moves a pixel by <=1 LSB — first to go over budget */
    for (int i = 0; i < 4000; i++)
        ps[n++] = (Particle){ frange(-160, 160), frange(-90, 90),
                              frange(1.0f, 1.5f), frange(0.0015f, 0.002f), 1.0f };

    Camera cam = { 0.0f, 0.0f, 1.0f };
    CullStats st;
    cull_frame(ps, n, &cam, RENDER_BUDGET, &st);
    printf("total=%d rendered=%d offscreen=%d subvisible=%d budget_dropped=%d\n",
           st.total, st.rendered, st.culled_offscreen, st.culled_subvisible,
           st.dropped_budget);

    static float fb_full[VIEW_W * VIEW_H], fb_cull[VIEW_W * VIEW_H];
    render(ps, n, &cam, NULL, fb_full);
    render(ps, n, &cam, g_keep, fb_cull);
    int diff = 0;
    for (int i = 0; i < VIEW_W * VIEW_H; i++)
        if (fabsf(fb_full[i] - fb_cull[i]) > DIFF_TOL) diff++;
    double pct = 100.0 * diff / (VIEW_W * VIEW_H);
    printf("diff_pixels=%d diff_pct=%.4f\n", diff, pct);

    /* timing: cull pass only, many frames, worst frame must fit the budget */
    int frames = 400;
    double total_ms = 0.0, max_ms = 0.0;
    int sink = 0;
    for (int f = 0; f < frames; f++) {
        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        cull_frame(ps, n, &cam, RENDER_BUDGET, &st);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        double ms = (double)(t1.tv_sec - t0.tv_sec) * 1e3 +
                    (double)(t1.tv_nsec - t0.tv_nsec) / 1e6;
        total_ms += ms;
        if (ms > max_ms) max_ms = ms;
        sink += st.rendered;
    }
    double avg_ms = total_ms / frames;
    printf("cull_avg_ms=%.4f cull_max_ms=%.4f frames=%d sink=%d\n",
           avg_ms, max_ms, frames, sink);

    int counted = st.culled_offscreen > 0 && st.culled_subvisible > 0 &&
                  st.dropped_budget > 0;
    printf("drops_counted=%d\n", counted);
    printf("pixel_match_ok=%d\n", pct < DIFF_BUDGET_PCT);
    printf("time_ok=%d\n", max_ms < CULL_TIME_BUDGET_MS);
    int ok = counted && pct < DIFF_BUDGET_PCT && max_ms < CULL_TIME_BUDGET_MS;
    printf("ok=%d\n", ok);
    return ok ? 0 : 1;
}
