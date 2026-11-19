
/* particle-pool: fixed-capacity particle pool with per-frame spawn budget.

   Quota allocation is three stages, every stage O(n) in emitter count:
     1. priority-weighted base quota (demand-capped, rotating leftover pass)
     2. per-emitter minimum reserve + aging boost for starved emitters
     3. inter-frame EMA smoothing + hard burst clamp (<= 105% of budget) */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MAX_PARTICLES 4096
#define MAX_EMITTERS 64
#define MIN_QUOTA 1
#define BURST_PCT 105        /* per-frame total may not exceed 105% of budget */
#define WAIT_HIST 8          /* starvation wait histogram buckets (clamped) */
#define FP 256               /* fixed-point scale for the smoothing EMA */

typedef struct { float x, y, vx, vy, life, age; int alive; int tag; } Particle;

typedef struct {
    int id;
    int quota;             /* particles wanted per frame */
    int priority;          /* base scheduling weight */
    int spawned;           /* actually spawned this frame */
    int grant;             /* quota granted this frame */
    int starve;            /* consecutive zero-grant frames */
    long smooth;           /* EMA of grant, fixed-point xFP */
    long total;            /* total granted over the run */
    int max_wait;          /* longest zero-grant run so far */
    int wait_hist[WAIT_HIST + 1];
} Emitter;

typedef struct {
    Particle pool[MAX_PARTICLES];
    int alive_count, high_water;
    Emitter emitters[MAX_EMITTERS];
    int emitter_count;
    int rr_cursor;         /* rotating cursor: fair tie-break without sorting */
} World;

static void world_init(World* w) {
    memset(w, 0, sizeof(*w));
    w->emitter_count = 0;
}

static int emitter_add(World* w, int id, int quota, int priority) {
    if (w->emitter_count >= MAX_EMITTERS) return -1;
    Emitter* e = &w->emitters[w->emitter_count++];
    memset(e, 0, sizeof(*e));
    e->id = id; e->quota = quota; e->priority = priority;
    return e->id;
}

static int pool_find_free(World* w) {
    for (int i = 0; i < MAX_PARTICLES; i++) if (!w->pool[i].alive) return i;
    return -1;
}

static int spawn_one(World* w, int emitter_id) {
    int idx = pool_find_free(w);
    if (idx < 0) return -1;
    Particle* p = &w->pool[idx];
    p->alive = 1; p->age = 0.0f;
    p->x = 0.0f; p->y = 0.0f;
    p->vx = 1.0f; p->vy = 0.5f;
    p->life = 2.0f;
    p->tag = emitter_id;
    w->alive_count++;
    if (w->alive_count > w->high_water) w->high_water = w->alive_count;
    return idx;
}

/* Three-stage quota allocation. Fills Emitter.grant, returns total granted.
   No sorting, no nested scans: strictly linear in emitter count. */
static int alloc_frame(World* w, int budget) {
    int n = w->emitter_count;
    int g[MAX_EMITTERS];
    int reserved[MAX_EMITTERS];
    long wt[MAX_EMITTERS];
    long wsum = 0;
    for (int i = 0; i < n; i++) {
        Emitter* e = &w->emitters[i];
        /* stage 2's aging boost feeds back into the stage 1 weights:
           the longer an emitter waited, the heavier it weighs */
        wt[i] = e->quota > 0 ? (long)e->priority * (1 + e->starve) : 0;
        wsum += wt[i];
        g[i] = 0; reserved[i] = 0;
    }
    if (wsum > 0 && budget > 0 && n > 0) {
        int total = 0;
        int start = w->rr_cursor % n;

        /* ---- stage 1: priority-weighted base quota ---- */
        for (int i = 0; i < n; i++) {
            long b = (long)budget * wt[i] / wsum;
            if (b > w->emitters[i].quota) b = w->emitters[i].quota;
            g[i] = (int)b;
            total += g[i];
        }
        /* leftover from flooring: round-robin +1 passes from the cursor */
        int left = budget - total;
        while (left > 0) {
            int progressed = 0;
            for (int k = 0; k < n && left > 0; k++) {
                int i = (start + k) % n;
                if (g[i] < w->emitters[i].quota) { g[i]++; left--; total++; progressed = 1; }
            }
            if (!progressed) break;
        }

        /* ---- stage 2: minimum reserve for zero-grant emitters ---- */
        for (int k = 0; k < n && total < budget; k++) {
            int i = (start + k) % n;
            Emitter* e = &w->emitters[i];
            if (g[i] == 0 && e->quota > 0) {
                int r = e->quota < MIN_QUOTA ? e->quota : MIN_QUOTA;
                if (r > budget - total) r = budget - total;
                g[i] += r; total += r; reserved[i] = 1;
            }
        }
        w->rr_cursor = (start + 1) % n;

        /* ---- stage 3: EMA smoothing + burst clamp ---- */
        total = 0;
        for (int i = 0; i < n; i++) {
            /* per-emitter jump cap: at most 2x its smoothed share + MIN_QUOTA */
            long cap = ((w->emitters[i].smooth * 2) >> 8) + MIN_QUOTA;
            if (g[i] > cap) g[i] = (int)cap;
            total += g[i];
        }
        long limit = (long)budget * BURST_PCT / 100;
        if (total > limit) {
            /* scale down proportionally, but never below a reserved minimum */
            int locked = 0, rest = 0;
            for (int i = 0; i < n; i++) {
                if (reserved[i]) locked += g[i]; else rest += g[i];
            }
            long avail = limit - locked; if (avail < 0) avail = 0;
            total = locked;
            if (rest > 0) {
                for (int i = 0; i < n; i++) if (!reserved[i]) {
                    g[i] = (int)((long)g[i] * avail / rest);
                    total += g[i];
                }
                /* hand back rounding headroom, rotating */
                int rem = (int)(limit - total);
                for (int k = 0; k < n && rem > 0; k++) {
                    int i = (start + k) % n;
                    if (!reserved[i] && g[i] < w->emitters[i].quota) { g[i]++; total++; rem--; }
                }
            }
        }
    }
    /* commit: starvation bookkeeping + EMA update */
    int total = 0;
    for (int i = 0; i < n; i++) {
        Emitter* e = &w->emitters[i];
        e->grant = g[i];
        total += g[i];
        if (g[i] > 0) {
            if (e->starve > 0) {
                int b = e->starve < WAIT_HIST ? e->starve : WAIT_HIST;
                e->wait_hist[b]++;
            }
            e->starve = 0;
        } else {
            e->starve++;
        }
        if (e->starve > e->max_wait) e->max_wait = e->starve;
        e->smooth += ((long)g[i] * FP - e->smooth) / 4;
        e->total += g[i];
    }
    return total;
}

static int spawn_frame(World* w, int budget) {
    int granted = alloc_frame(w, budget);
    for (int i = 0; i < w->emitter_count; i++) {
        Emitter* e = &w->emitters[i];
        e->spawned = 0;
        for (int k = 0; k < e->grant; k++) {
            if (spawn_one(w, e->id) < 0) break;
            e->spawned++;
        }
    }
    return granted;
}

static void update(World* w, float dt) {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        p->x += p->vx * dt; p->y += p->vy * dt;
        if (p->age >= p->life) { p->alive = 0; w->alive_count--; }
    }
}

static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* 10 emitters x 1000 frames: budget dips + demand bursts, worst-case
   appetite layout (lowest priority asks for the most). */
static int run_fairness_sim(void) {
    World w; world_init(&w);
    const int N = 10;
    for (int i = 0; i < N; i++)
        emitter_add(&w, i + 1, 60 - i * 5, i + 1);   /* quota 60..15, prio 1..10 */

    double worst_ratio = 0.0;
    long burst_alloc = 0, burst_budget = 0;
    for (int frame = 0; frame < 1000; frame++) {
        int budget = (frame % 149 < 3) ? 6 : 40;     /* periodic budget dips */
        int burst = (frame % 211 < 4);               /* periodic demand spikes */
        for (int i = 0; i < N; i++)
            w.emitters[i].quota = (60 - i * 5) * (burst ? 4 : 1);
        int got = spawn_frame(&w, budget);
        double r = (double)got / budget;
        if (r > worst_ratio) worst_ratio = r;
        if (burst) { burst_alloc += got; burst_budget += budget; }
        update(&w, 0.1f);
    }

    /* flush in-flight starvation runs into the histograms */
    int agg[WAIT_HIST + 1] = {0};
    for (int i = 0; i < N; i++) {
        Emitter* e = &w.emitters[i];
        if (e->starve > 0) {
            int b = e->starve < WAIT_HIST ? e->starve : WAIT_HIST;
            e->wait_hist[b]++;
        }
        for (int b = 0; b <= WAIT_HIST; b++) agg[b] += e->wait_hist[b];
    }

    printf("== 10 emitters x 1000 frames (budget 40, dips to 6, demand bursts x4) ==\n");
    printf("emitter  prio  demand  granted  max_wait  wait=1  wait=2  wait=3+\n");
    int all_got = 1, max_wait = 0;
    for (int i = 0; i < N; i++) {
        Emitter* e = &w.emitters[i];
        int w3 = 0;
        for (int b = 3; b <= WAIT_HIST; b++) w3 += e->wait_hist[b];
        printf("%3d     %3d   %4d   %7ld  %4d     %5d  %5d  %5d\n",
               e->id, e->priority, 60 - i * 5, e->total, e->max_wait,
               e->wait_hist[1], e->wait_hist[2], w3);
        if (e->total == 0) all_got = 0;
        if (e->max_wait > max_wait) max_wait = e->max_wait;
    }

    printf("starvation wait distribution (zero-grant run length, all emitters):\n");
    for (int b = 1; b <= WAIT_HIST; b++)
        if (agg[b] || b <= 3)
            printf("  wait=%s%d  %d\n", b == WAIT_HIST ? ">=" : "", b, agg[b]);

    double burst_ratio = burst_budget ? (double)burst_alloc / burst_budget : 0.0;
    int ok = all_got && max_wait <= 3 && worst_ratio <= 1.05 + 1e-9;
    printf("checks:\n");
    printf("  all_emitters_got_quota        %s\n", all_got ? "PASS" : "FAIL");
    printf("  max_starvation_wait=%d (<=3)   %s\n", max_wait, max_wait <= 3 ? "PASS" : "FAIL");
    printf("  worst_frame_total/budget=%.3f (<=1.05)  %s\n",
           worst_ratio, worst_ratio <= 1.05 + 1e-9 ? "PASS" : "FAIL");
    printf("  burst_frames_total=%ld/%ld budget (=%.3fx, <=1.05)  %s\n",
           burst_alloc, burst_budget, burst_ratio,
           burst_ratio <= 1.05 + 1e-9 ? "PASS" : "FAIL");
    printf("alive=%d high_water=%d\n", w.alive_count, w.high_water);
    return ok;
}

/* allocation cost must scale linearly with emitter count */
static void run_scaling_bench(void) {
    printf("== allocation cost vs emitter count ==\n");
    for (int n = 8; n <= MAX_EMITTERS; n *= 2) {
        World w; world_init(&w);
        for (int i = 0; i < n; i++)
            emitter_add(&w, i + 1, 1000, (i % 10) + 1);
        for (int f = 0; f < 200; f++) alloc_frame(&w, 100);   /* warm up EMA */
        const int iters = 200000;
        long sink = 0;
        long long t0 = now_ns();
        for (int it = 0; it < iters; it++) sink += alloc_frame(&w, 100);
        long long t1 = now_ns();
        double ns = (double)(t1 - t0) / iters;
        printf("n=%2d  ns/frame=%8.1f  ns/emitter=%6.2f  (sink=%ld)\n",
               n, ns, ns / n, sink);
    }
}

int main(void) {
    /* smoke: original 3-emitter scenario */
    World w; world_init(&w);
    emitter_add(&w, 1, 40, 2);
    emitter_add(&w, 2, 30, 9);
    emitter_add(&w, 3, 20, 5);
    for (int frame = 0; frame < 10; frame++) {
        spawn_frame(&w, 50);
        update(&w, 0.1f);
    }
    printf("smoke: alive=%d high_water=%d\n", w.alive_count, w.high_water);

    int ok = run_fairness_sim();
    run_scaling_bench();
    printf("overall: %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
