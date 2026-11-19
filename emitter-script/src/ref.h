/* ref: hand-written reference implementations of the ten example scripts.

   Each ref_NN function mirrors examples/NN_*.emitters but is written by hand:
   constants are folded, the curve is inlined, and there is no parser, no
   generic scheduler and no shared spawn loop. test.c cross-checks the scripted
   run against these. Keeping them literal is the point: if both sides shared
   code, agreement would prove nothing. */
#ifndef REF_H
#define REF_H

#include <math.h>
#include <string.h>

#include "ems.h"

/* ---- example 01: single constant emitter ---- */
static void ref_01(EmsResult* out) {
    enum { POOL = 128, TICKS = 5, BUDGET = 10, QUOTA = 2 };
    const float dt = 0.1f, life = 20.0f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        int left = BUDGET;
        int want = QUOTA < left ? QUOTA : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f; px[idx] = 0.0f; py[idx] = 0.0f;
            alive++; out->spawned[0]++; left--;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; px[i] += 1.0f * dt; py[i] += 0.5f * dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 02: three emitters, budget 50 shared by priority ---- */
static void ref_02(EmsResult* out) {
    enum { POOL = 4096, TICKS = 20, BUDGET = 50 };
    const float dt = 0.1f, life = 0.5f;
    /* script index: 0 spark(p1,q40), 1 jet(p9,q30), 2 haze(p5,q20) */
    static const int visit[3] = { 1, 2, 0 };
    static const int quota[3] = { 40, 30, 20 };
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL];
    static int owner[POOL], next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        int left = BUDGET;
        for (int k = 0; k < 3 && left > 0; k++) {
            int e = visit[k];
            int want = quota[e] < left ? quota[e] : left;
            for (int i = 0; i < want && free_head != -1; i++) {
                int idx = free_head;
                free_head = next_free[idx];
                live[idx] = 1; age[idx] = 0.0f; px[idx] = 0.0f; py[idx] = 0.0f;
                owner[idx] = e;
                alive++; out->spawned[e]++; left--;
            }
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; px[i] += 1.0f * dt; py[i] += 0.5f * dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[owner[i]]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->alive[owner[i]]++;
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 03: linear quota ramp 0..4 per unit time ---- */
static void ref_03(EmsResult* out) {
    enum { POOL = 256, TICKS = 4, BUDGET = 100 };
    const float dt = 0.25f, life = 10.0f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    double carry = 0.0;
    for (int tick = 0; tick < TICKS; tick++) {
        float t = (float)tick * dt;
        float phase = t - floorf(t);
        float q = 0.0f + (4.0f - 0.0f) * phase;
        if (q > (float)BUDGET) q = (float)BUDGET;
        carry += q;
        int want = (int)floor(carry);
        if (want > BUDGET) want = BUDGET;
        carry -= want;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f; px[idx] = 0.0f; py[idx] = 0.0f;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; px[i] += 0.25f * dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 04: velocity x follows quad_in curve -2..2 ---- */
static void ref_04(EmsResult* out) {
    enum { POOL = 256, TICKS = 8, BUDGET = 100, QUOTA = 2 };
    const float dt = 0.125f, life = 10.0f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL], vx[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        float t = (float)tick * dt;
        float phase = t - floorf(t);
        float u = phase * phase;                 /* quad_in */
        float v = -2.0f + (2.0f - -2.0f) * u;
        int left = BUDGET;
        int want = QUOTA < left ? QUOTA : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f;
            px[idx] = 0.0f; py[idx] = 0.0f; vx[idx] = v;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; px[i] += vx[i] * dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 05: life shrinks linearly 4..1 ---- */
static void ref_05(EmsResult* out) {
    enum { POOL = 256, TICKS = 8, BUDGET = 100, QUOTA = 2 };
    const float dt = 0.125f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL], life[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        float t = (float)tick * dt;
        float phase = t - floorf(t);
        float lf = 4.0f + (1.0f - 4.0f) * phase;
        int left = BUDGET;
        int want = QUOTA < left ? QUOTA : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f;
            px[idx] = 0.0f; py[idx] = 0.0f; life[idx] = lf;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; py[i] += 1.0f * dt;
            if (age[i] >= life[i]) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 06: step gate 0 -> 3 at t >= 1 ---- */
static void ref_06(EmsResult* out) {
    enum { POOL = 256, TICKS = 6, BUDGET = 100 };
    const float dt = 0.5f, life = 10.0f;
    static unsigned char live[POOL];
    static float age[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    double carry = 0.0;
    for (int tick = 0; tick < TICKS; tick++) {
        float t = (float)tick * dt;
        int want = t >= 1.0f ? 3 : 0;
        int left = BUDGET;
        if (want > left) want = left;
        (void)carry;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) out->sum_age += age[i];
}

/* ---- example 07: x position sweep -5..5, cubic_io ---- */
static void ref_07(EmsResult* out) {
    enum { POOL = 256, TICKS = 8, BUDGET = 100, QUOTA = 1 };
    const float dt = 0.125f, life = 10.0f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        float t = (float)tick * dt;
        float u = t - floorf(t);
        float e = u < 0.5f ? 4.0f * u * u * u
                           : 1.0f - powf(-2.0f * u + 2.0f, 3.0f) / 2.0f;
        float x = -5.0f + (5.0f - -5.0f) * e;
        int left = BUDGET;
        int want = QUOTA < left ? QUOTA : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f; px[idx] = x;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->sum_x += px[i]; out->sum_y += 0.0f; out->sum_age += age[i];
    }
}

/* ---- example 08: pool 10 saturates; greedy(5,8) then trickle(1,5) ---- */
static void ref_08(EmsResult* out) {
    enum { POOL = 10, TICKS = 5, BUDGET = 100 };
    const float dt = 0.1f, life = 100.0f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL];
    static int owner[POOL], next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        int left = BUDGET;
        /* greedy: priority 5, quota 8, origin (0,0) */
        int want = 8 < left ? 8 : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f; px[idx] = 0.0f; py[idx] = 0.0f;
            owner[idx] = 0;
            alive++; out->spawned[0]++; left--;
        }
        /* trickle: priority 1, quota 5, origin (1,1) */
        want = 5 < left ? 5 : left;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f; px[idx] = 1.0f; py[idx] = 1.0f;
            owner[idx] = 1;
            alive++; out->spawned[1]++; left--;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[owner[i]]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->alive[owner[i]]++;
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

/* ---- example 09: quota 0.25 per tick accumulates ---- */
static void ref_09(EmsResult* out) {
    enum { POOL = 64, TICKS = 8, BUDGET = 10 };
    const float dt = 1.0f, life = 100.0f;
    static unsigned char live[POOL];
    static float age[POOL];
    static int next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    double carry = 0.0;
    for (int tick = 0; tick < TICKS; tick++) {
        carry += 0.25;
        int want = (int)floor(carry);
        carry -= want;
        if (want > BUDGET) want = BUDGET;
        for (int i = 0; i < want && free_head != -1; i++) {
            int idx = free_head;
            free_head = next_free[idx];
            live[idx] = 1; age[idx] = 0.0f;
            alive++; out->spawned[0]++;
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[0]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    out->alive[0] = alive;
    for (int i = 0; i < POOL; i++) if (live[i]) out->sum_age += age[i];
}

/* ---- example 10: budget 7 split a(3,5), b(1,5), c(0,5); life 0.5 ---- */
static void ref_10(EmsResult* out) {
    enum { POOL = 256, TICKS = 10, BUDGET = 7 };
    const float dt = 0.2f, life = 0.5f;
    static unsigned char live[POOL];
    static float age[POOL], px[POOL], py[POOL];
    static int owner[POOL], next_free[POOL];
    memset(out, 0, sizeof(*out));
    memset(live, 0, sizeof(live));
    for (int i = 0; i < POOL; i++) next_free[i] = i + 1 < POOL ? i + 1 : -1;
    int free_head = 0, alive = 0;
    for (int tick = 0; tick < TICKS; tick++) {
        int left = BUDGET;
        for (int e = 0; e < 3 && left > 0; e++) {   /* a, b, c already in priority order */
            int want = 5 < left ? 5 : left;
            for (int i = 0; i < want && free_head != -1; i++) {
                int idx = free_head;
                free_head = next_free[idx];
                live[idx] = 1; age[idx] = 0.0f;
                px[idx] = 0.0f; py[idx] = (float)e;
                owner[idx] = e;
                alive++; out->spawned[e]++; left--;
            }
        }
        if (alive > out->high_water) out->high_water = alive;
        for (int i = 0; i < POOL; i++) {
            if (!live[i]) continue;
            age[i] += dt; px[i] += 1.0f * dt;
            if (age[i] >= life) {
                live[i] = 0; next_free[i] = free_head; free_head = i;
                alive--; out->expired[owner[i]]++;
            }
        }
        out->frame_alive[tick] = alive;
    }
    out->frames = TICKS;
    for (int i = 0; i < POOL; i++) if (live[i]) {
        out->alive[owner[i]]++;
        out->sum_x += px[i]; out->sum_y += py[i]; out->sum_age += age[i];
    }
}

#endif
