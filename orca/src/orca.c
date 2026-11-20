/* mini-orca: ORCA local avoidance (RVO2-style reciprocal velocity obstacles).

   - per-neighbour velocity obstacle -> feasible half-plane (Line)
   - 2D linear program (lp1/lp2/lp3) solves the velocity closest to the
     preferred one inside all half-planes and the max-speed disc
   - head-to-head against a naive "push straight through" avoidance
   - checks: zero collisions, mean arrival within 20% of the ideal
     straight-line time, per-frame cost measured for N = 10..100 agents */
#define _POSIX_C_SOURCE 199309L /* clock_gettime */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_AGENTS 128
#define PARALLEL_EPS 1e-7
#define COLLISION_EPS 1e-6
#define ARRIVE_TOL 0.5

enum { MODE_ORCA = 0, MODE_NAIVE = 1 };

typedef struct { double x, y; } V2;

static V2 v2(double x, double y) { V2 v = { x, y }; return v; }
static V2 v2add(V2 a, V2 b) { return v2(a.x + b.x, a.y + b.y); }
static V2 v2sub(V2 a, V2 b) { return v2(a.x - b.x, a.y - b.y); }
static V2 v2scale(V2 a, double s) { return v2(a.x * s, a.y * s); }
static double v2dot(V2 a, V2 b) { return a.x * b.x + a.y * b.y; }
static double v2det(V2 a, V2 b) { return a.x * b.y - a.y * b.x; }
static double v2len(V2 a) { return sqrt(v2dot(a, a)); }
static V2 v2norm(V2 a) { double l = v2len(a); return l > 1e-12 ? v2scale(a, 1.0 / l) : v2(0, 0); }
static V2 v2rot(V2 a, double ang) { double c = cos(ang), s = sin(ang); return v2(c * a.x - s * a.y, s * a.x + c * a.y); }

/* feasible side: det(dir, v - point) <= 0 */
typedef struct { V2 point, dir; } Line;

typedef struct {
    V2 pos, vel, pref, goal;
    double radius, pref_speed, max_speed;
    int arrived;
    double arrive_time;
} Agent;

/* project the optimal velocity onto line `no` clipped by lines 0..no-1
   and the speed disc; returns 0 when the feasible segment is empty */
static int lp1(const Line* lines, int no, double radius, V2 opt, int dir_opt, V2* res) {
    double dot = v2dot(lines[no].point, lines[no].dir);
    double disc = dot * dot + radius * radius - v2dot(lines[no].point, lines[no].point);
    if (disc < 0.0) return 0;
    double sq = sqrt(disc);
    double tl = -dot - sq, tr = -dot + sq;
    for (int i = 0; i < no; i++) {
        double denom = v2det(lines[no].dir, lines[i].dir);
        double numer = v2det(lines[i].dir, v2sub(lines[no].point, lines[i].point));
        if (fabs(denom) <= PARALLEL_EPS) {
            if (numer < 0.0) return 0;
            continue;
        }
        double t = numer / denom;
        if (denom >= 0.0) { if (t < tr) tr = t; }
        else              { if (t > tl) tl = t; }
        if (tl > tr) return 0;
    }
    if (dir_opt) {
        double t = v2dot(opt, lines[no].dir) > 0.0 ? tr : tl;
        *res = v2add(lines[no].point, v2scale(lines[no].dir, t));
    } else {
        double t = v2dot(lines[no].dir, v2sub(opt, lines[no].point));
        if (t < tl) t = tl; else if (t > tr) t = tr;
        *res = v2add(lines[no].point, v2scale(lines[no].dir, t));
    }
    return 1;
}

/* solve the full LP; returns the failing line index, or n on success */
static int lp2(const Line* lines, int n, double radius, V2 opt, int dir_opt, V2* res) {
    if (dir_opt) *res = v2scale(opt, radius);
    else if (v2dot(opt, opt) > radius * radius) *res = v2scale(v2norm(opt), radius);
    else *res = opt;
    for (int i = 0; i < n; i++) {
        if (v2det(lines[i].dir, v2sub(lines[i].point, *res)) > 0.0) {
            V2 tmp = *res;
            if (!lp1(lines, i, radius, opt, dir_opt, res)) { *res = tmp; return i; }
        }
    }
    return n;
}

/* infeasible fallback: relax lines begin..n-1 by penetration depth */
static void lp3(const Line* lines, int n, int begin, double radius, V2* res) {
    double distance = 0.0;
    Line proj[MAX_AGENTS];
    for (int i = begin; i < n; i++) {
        if (v2det(lines[i].dir, v2sub(lines[i].point, *res)) > distance) {
            int m = 0;
            for (int j = 0; j < i; j++) {
                Line line;
                double determinant = v2det(lines[i].dir, lines[j].dir);
                if (fabs(determinant) <= PARALLEL_EPS) {
                    if (v2dot(lines[i].dir, lines[j].dir) > 0.0) continue;
                    line.point = v2scale(v2add(lines[i].point, lines[j].point), 0.5);
                } else {
                    double t = v2det(lines[j].dir, v2sub(lines[i].point, lines[j].point)) / determinant;
                    line.point = v2add(lines[i].point, v2scale(lines[i].dir, t));
                }
                line.dir = v2norm(v2sub(lines[j].dir, lines[i].dir));
                proj[m++] = line;
            }
            V2 tmp = *res;
            V2 dir = v2(-lines[i].dir.y, lines[i].dir.x);
            if (lp2(proj, m, radius, dir, 1, res) < m) *res = tmp;
            distance = v2det(lines[i].dir, v2sub(lines[i].point, *res));
        }
    }
}

/* ORCA: build one half-plane per neighbour, solve for the new velocity */
static V2 orca_velocity(const Agent* agents, int n, int self, double tau, double dt, Line* lines) {
    const Agent* a = &agents[self];
    int nl = 0;
    for (int j = 0; j < n; j++) {
        if (j == self || agents[j].arrived) continue;
        V2 rel_pos = v2sub(agents[j].pos, a->pos);
        V2 rel_vel = v2sub(a->vel, agents[j].vel);
        double dist_sq = v2dot(rel_pos, rel_pos);
        double comb_r = a->radius + agents[j].radius;
        double comb_r_sq = comb_r * comb_r;
        Line line;
        V2 u;
        if (dist_sq > comb_r_sq) {
            /* not overlapping: truncate the VO cone at t = tau */
            V2 w = v2sub(rel_vel, v2scale(rel_pos, 1.0 / tau));
            double w_len_sq = v2dot(w, w);
            double dp1 = v2dot(w, rel_pos);
            if (dp1 < 0.0 && dp1 * dp1 > comb_r_sq * w_len_sq) {
                /* project on the cut-off circle */
                double w_len = sqrt(w_len_sq);
                V2 unit_w = v2scale(w, 1.0 / w_len);
                line.dir = v2(unit_w.y, -unit_w.x);
                u = v2scale(unit_w, comb_r / tau - w_len);
            } else {
                /* project on the VO legs */
                double leg = sqrt(dist_sq - comb_r_sq);
                if (v2det(rel_pos, w) > 0.0)
                    line.dir = v2scale(v2(rel_pos.x * leg - rel_pos.y * comb_r,
                                          rel_pos.x * comb_r + rel_pos.y * leg), 1.0 / dist_sq);
                else
                    line.dir = v2scale(v2(-(rel_pos.x * leg + rel_pos.y * comb_r),
                                          rel_pos.x * comb_r - rel_pos.y * leg), 1.0 / dist_sq);
                double dp2 = v2dot(rel_vel, line.dir);
                u = v2sub(v2scale(line.dir, dp2), rel_vel);
            }
        } else {
            /* already overlapping: escape within one timestep */
            double inv_dt = 1.0 / dt;
            V2 w = v2sub(rel_vel, v2scale(rel_pos, inv_dt));
            double w_len = v2len(w);
            V2 unit_w = w_len > 1e-12 ? v2scale(w, 1.0 / w_len) : v2(1, 0);
            line.dir = v2(unit_w.y, -unit_w.x);
            u = v2scale(unit_w, comb_r * inv_dt - w_len);
        }
        line.point = v2add(a->vel, v2scale(u, 0.5)); /* reciprocal: share half the correction */
        lines[nl++] = line;
    }
    V2 result;
    int fail = lp2(lines, nl, a->max_speed, a->pref, 0, &result);
    if (fail < nl) lp3(lines, nl, fail, a->max_speed, &result);
    return result;
}

/* naive baseline: steer at the goal and push away from close neighbours */
static V2 naive_velocity(const Agent* agents, int n, int self) {
    const Agent* a = &agents[self];
    V2 v = a->pref;
    for (int j = 0; j < n; j++) {
        if (j == self || agents[j].arrived) continue;
        V2 d = v2sub(a->pos, agents[j].pos);
        double dist = v2len(d);
        double safe = a->radius + agents[j].radius + 0.6;
        if (dist < safe && dist > 1e-9) {
            double push = (safe - dist) / safe * 3.0;
            v = v2add(v, v2scale(v2norm(d), push * a->pref_speed));
        }
    }
    double l = v2len(v);
    if (l > a->max_speed) v = v2scale(v, a->max_speed / l);
    return v;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

typedef struct {
    long collisions;   /* pair-frames with overlap beyond COLLISION_EPS */
    double min_gap;    /* smallest (distance - r_i - r_j) ever observed */
    double avg_arrive; /* mean arrival time over arrived agents (s) */
    double ideal;      /* straight-line time at preferred speed (s) */
    int arrived;       /* how many agents reached their goal */
    double frame_ms;   /* avg compute+integrate cost per frame (ms) */
} SimResult;

/* n agents on a circle, each heading for the antipodal point.
   agent_r is the true body radius; ORCA plans with agent_r + pad so the
   discrete-time integration never lets true bodies overlap. */
static SimResult simulate(int n, int mode, double circle_r, double pref_speed,
                          double agent_r, double pad, double dt, double tau, double bias) {
    Agent agents[MAX_AGENTS];
    Line* lines = (Line*)malloc(sizeof(Line) * (size_t)n);
    V2* new_vel = (V2*)malloc(sizeof(V2) * (size_t)n);
    SimResult r = { 0, 1e30, 0.0, 2.0 * circle_r / pref_speed, 0, 0.0 };

    for (int i = 0; i < n; i++) {
        double ang = 2.0 * M_PI * i / n;
        agents[i].pos = v2(circle_r * cos(ang), circle_r * sin(ang));
        agents[i].goal = v2scale(agents[i].pos, -1.0);
        agents[i].vel = v2(0, 0);
        agents[i].radius = agent_r + pad;
        agents[i].pref_speed = pref_speed;
        agents[i].max_speed = 1.5 * pref_speed;
        agents[i].arrived = 0;
        agents[i].arrive_time = -1.0;
    }

    double t = 0.0, t_max = 4.0 * r.ideal, compute_ms = 0.0;
    long frames = 0;
    int remaining = n;
    while (remaining > 0 && t < t_max) {
        double t0 = now_ms();
        for (int i = 0; i < n; i++) {
            if (agents[i].arrived) continue;
            V2 to = v2sub(agents[i].goal, agents[i].pos);
            double d = v2len(to);
            V2 pref = d > 1e-9 ? v2scale(to, pref_speed / d) : v2(0, 0);
            /* deterministic per-agent symmetry break, else perfectly
               head-on pairs freeze in a reciprocal standoff */
            agents[i].pref = v2rot(pref, bias * sin(i * 12.9898));
            new_vel[i] = (mode == MODE_ORCA) ? orca_velocity(agents, n, i, tau, dt, lines)
                                             : naive_velocity(agents, n, i);
        }
        for (int i = 0; i < n; i++) {
            if (agents[i].arrived) continue;
            agents[i].vel = new_vel[i];
            agents[i].pos = v2add(agents[i].pos, v2scale(agents[i].vel, dt));
        }
        compute_ms += now_ms() - t0;
        frames++;
        t += dt;
        for (int i = 0; i < n; i++) {
            if (agents[i].arrived) continue;
            if (v2len(v2sub(agents[i].goal, agents[i].pos)) < ARRIVE_TOL) {
                agents[i].arrived = 1;
                agents[i].arrive_time = t;
                r.avg_arrive += t;
                remaining--;
            }
        }
        for (int i = 0; i < n; i++) {
            if (agents[i].arrived) continue;
            for (int j = i + 1; j < n; j++) {
                if (agents[j].arrived) continue;
                /* true bodies, padding excluded */
                double gap = v2len(v2sub(agents[i].pos, agents[j].pos)) - 2.0 * agent_r;
                if (gap < r.min_gap) r.min_gap = gap;
                if (gap < -COLLISION_EPS) r.collisions++;
            }
        }
    }
    r.arrived = n - remaining;
    if (r.arrived > 0) r.avg_arrive /= r.arrived;
    if (frames > 0) r.frame_ms = compute_ms / frames;
    free(lines);
    free(new_vel);
    return r;
}

int main(void) {
    const double circle_r = 50.0, pref_speed = 2.0, agent_r = 0.5;
    const double pad = 0.05, dt = 0.05, tau = 4.0, bias = 0.25;
    int fail = 0;

    printf("== head-to-head, N=40 ==\n");
    SimResult orca = simulate(40, MODE_ORCA, circle_r, pref_speed, agent_r, pad, dt, tau, bias);
    SimResult naive = simulate(40, MODE_NAIVE, circle_r, pref_speed, agent_r, pad, dt, tau, bias);
    printf("orca : collisions=%ld min_gap=%+.4f avg_arrive=%.2fs (ideal %.2fs, %+.1f%%) arrived=%d/40\n",
           orca.collisions, orca.min_gap, orca.avg_arrive, orca.ideal,
           100.0 * (orca.avg_arrive / orca.ideal - 1.0), orca.arrived);
    printf("naive: collisions=%ld min_gap=%+.4f avg_arrive=%.2fs (ideal %.2fs, %+.1f%%) arrived=%d/40\n",
           naive.collisions, naive.min_gap, naive.avg_arrive, naive.ideal,
           100.0 * (naive.avg_arrive / naive.ideal - 1.0), naive.arrived);
    if (orca.collisions != 0 || orca.min_gap < -COLLISION_EPS) { printf("FAIL: orca collided\n"); fail = 1; }
    if (orca.arrived != 40) { printf("FAIL: orca agents stuck\n"); fail = 1; }
    if (orca.avg_arrive > 1.2 * orca.ideal) { printf("FAIL: orca arrival > 120%% of ideal\n"); fail = 1; }

    printf("== orca sweep, N=10..100 ==\n");
    for (int n = 10; n <= 100; n += 10) {
        SimResult s = simulate(n, MODE_ORCA, circle_r, pref_speed, agent_r, pad, dt, tau, bias);
        double over = 100.0 * (s.avg_arrive / s.ideal - 1.0);
        printf("N=%3d  frame=%7.4f ms  collisions=%ld  min_gap=%+.4f  avg_arrive=%6.2fs (%+.1f%%)  arrived=%d/%d\n",
               n, s.frame_ms, s.collisions, s.min_gap, s.avg_arrive, over, s.arrived, n);
        if (s.collisions != 0 || s.min_gap < -COLLISION_EPS) { printf("FAIL: collision at N=%d\n", n); fail = 1; }
        if (s.arrived != n) { printf("FAIL: stuck agents at N=%d\n", n); fail = 1; }
        if (s.avg_arrive > 1.2 * s.ideal) { printf("FAIL: arrival > 120%% of ideal at N=%d\n", n); fail = 1; }
    }

    printf("%s\n", fail ? "FAIL" : "OK");
    return fail;
}
