
/* mini-dubins: curvature-constrained shortest paths for vehicle-like units.
   Closed-form shortest Dubins path between two poses under a minimum turning
   radius, sampling into a configurable number of followable waypoints, and a
   self-check that the analytic length matches an independent numerical
   optimization within 5%. */
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef struct { double x, y, theta; } Pose;

typedef enum { SEG_L = 0, SEG_S = 1, SEG_R = 2 } SegType;

typedef struct {
    SegType seg[3];  /* word, e.g. {L,S,L} */
    double param[3]; /* normalized segment lengths (unit turning radius) */
    double rho;      /* minimum turning radius */
    double rot;      /* frame rotation of the normalized problem */
    double length;   /* total path length, world units */
} DubinsPath;

static double mod2pi(double a) {
    a = fmod(a, 2.0 * M_PI);
    if (a < 0) a += 2.0 * M_PI;
    return a;
}

static double angdiff(double a, double b) {
    double d = fmod(a - b + M_PI, 2.0 * M_PI);
    if (d < 0) d += 2.0 * M_PI;
    return d - M_PI;
}

/* ---- the six Dubins words, normalized to unit radius ----
   start (0,0,alpha), goal (d,0,beta); out = {t, p, q}, -1 if infeasible. */

static int word_lsl(double d, double a, double b, double* o) {
    double p2 = 2 + d * d - 2 * cos(a - b) + 2 * d * (sin(a) - sin(b));
    if (p2 < 0) return -1;
    double tmp = atan2(cos(b) - cos(a), d + sin(a) - sin(b));
    o[0] = mod2pi(-a + tmp);
    o[1] = sqrt(p2);
    o[2] = mod2pi(b - tmp);
    return 0;
}

static int word_rsr(double d, double a, double b, double* o) {
    double p2 = 2 + d * d - 2 * cos(a - b) + 2 * d * (sin(b) - sin(a));
    if (p2 < 0) return -1;
    double tmp = atan2(cos(a) - cos(b), d - sin(a) + sin(b));
    o[0] = mod2pi(a - tmp);
    o[1] = sqrt(p2);
    o[2] = mod2pi(-b + tmp);
    return 0;
}

static int word_lsr(double d, double a, double b, double* o) {
    double p2 = -2 + d * d + 2 * cos(a - b) + 2 * d * (sin(a) + sin(b));
    if (p2 < 0) return -1;
    double p = sqrt(p2);
    double tmp = atan2(-cos(a) - cos(b), d + sin(a) + sin(b)) - atan2(-2.0, p);
    o[0] = mod2pi(-a + tmp);
    o[1] = p;
    o[2] = mod2pi(-mod2pi(b) + tmp);
    return 0;
}

static int word_rsl(double d, double a, double b, double* o) {
    double p2 = -2 + d * d + 2 * cos(a - b) - 2 * d * (sin(a) + sin(b));
    if (p2 < 0) return -1;
    double p = sqrt(p2);
    double tmp = atan2(cos(a) + cos(b), d - sin(a) - sin(b)) - atan2(2.0, p);
    o[0] = mod2pi(a - tmp);
    o[1] = p;
    o[2] = mod2pi(b - tmp);
    return 0;
}

static int word_rlr(double d, double a, double b, double* o) {
    double tmp = (6.0 - d * d + 2 * cos(a - b) + 2 * d * (sin(a) - sin(b))) / 8.0;
    if (fabs(tmp) > 1) return -1;
    double p = mod2pi(2 * M_PI - acos(tmp));
    o[0] = mod2pi(a - atan2(cos(a) - cos(b), d - sin(a) + sin(b)) + p / 2.0);
    o[1] = p;
    o[2] = mod2pi(a - b - o[0] + p);
    return 0;
}

static int word_lrl(double d, double a, double b, double* o) {
    double tmp = (6.0 - d * d + 2 * cos(a - b) + 2 * d * (sin(b) - sin(a))) / 8.0;
    if (fabs(tmp) > 1) return -1;
    double p = mod2pi(2 * M_PI - acos(tmp));
    o[0] = mod2pi(-a + atan2(-cos(a) + cos(b), d + sin(a) - sin(b)) + p / 2.0);
    o[1] = p;
    o[2] = mod2pi(mod2pi(b) - a - o[0] + p);
    return 0;
}

typedef int (*WordFn)(double, double, double, double*);

static const struct { WordFn fn; SegType seg[3]; } WORDS[6] = {
    { word_lsl, { SEG_L, SEG_S, SEG_L } },
    { word_rsr, { SEG_R, SEG_S, SEG_R } },
    { word_lsr, { SEG_L, SEG_S, SEG_R } },
    { word_rsl, { SEG_R, SEG_S, SEG_L } },
    { word_rlr, { SEG_R, SEG_L, SEG_R } },
    { word_lrl, { SEG_L, SEG_R, SEG_L } },
};

/* shortest curvature-bounded path from s to g with min radius rho (> 0) */
static int dubins_shortest(Pose s, Pose g, double rho, DubinsPath* out) {
    if (rho <= 0 || !out) return -1;
    double dx = g.x - s.x, dy = g.y - s.y;
    double rot = atan2(dy, dx);
    double d = hypot(dx, dy) / rho;
    double alpha = mod2pi(s.theta - rot);
    double beta = mod2pi(g.theta - rot);
    double best = -1;
    int bestw = -1;
    double tpq[3] = { 0, 0, 0 };
    for (int w = 0; w < 6; w++) {
        double cand[3];
        if (WORDS[w].fn(d, alpha, beta, cand) != 0) continue;
        double len = cand[0] + cand[1] + cand[2];
        if (bestw < 0 || len < best) {
            best = len;
            bestw = w;
            tpq[0] = cand[0]; tpq[1] = cand[1]; tpq[2] = cand[2];
        }
    }
    if (bestw < 0) return -1;
    for (int i = 0; i < 3; i++) {
        out->seg[i] = WORDS[bestw].seg[i];
        out->param[i] = tpq[i];
    }
    out->rho = rho;
    out->rot = rot;
    out->length = best * rho;
    return 0;
}

/* advance pose q along one segment by normalized length phi */
static void seg_step(SegType s, double phi, Pose* q) {
    if (s == SEG_S) {
        q->x += phi * cos(q->theta);
        q->y += phi * sin(q->theta);
    } else if (s == SEG_L) {
        double nx = q->x + sin(q->theta + phi) - sin(q->theta);
        double ny = q->y - cos(q->theta + phi) + cos(q->theta);
        q->x = nx; q->y = ny;
        q->theta += phi;
    } else {
        double nx = q->x - sin(q->theta - phi) + sin(q->theta);
        double ny = q->y + cos(q->theta - phi) - cos(q->theta);
        q->x = nx; q->y = ny;
        q->theta -= phi;
    }
}

/* discretize path into n waypoints (n >= 2, endpoints included).
   out[i] carries x, y and heading theta so a unit can steer toward them. */
static int dubins_sample(const DubinsPath* path, Pose start, Pose* out, int n) {
    if (!path || !out || n < 2) return -1;
    double total = path->param[0] + path->param[1] + path->param[2];
    double cr = cos(path->rot), sr = sin(path->rot);
    double alpha = mod2pi(start.theta - path->rot);
    for (int i = 0; i < n; i++) {
        double s = total * (double)i / (double)(n - 1);
        Pose q = { 0, 0, alpha }; /* normalized frame */
        double rem = s;
        for (int k = 0; k < 3 && rem > 0; k++) {
            double step = rem < path->param[k] ? rem : path->param[k];
            seg_step(path->seg[k], step, &q);
            rem -= step;
        }
        out[i].x = start.x + path->rho * (cr * q.x - sr * q.y);
        out[i].y = start.y + path->rho * (sr * q.x + cr * q.y);
        out[i].theta = q.theta + path->rot;
    }
    return 0;
}

/* ---- independent numerical optimizer (cross-check only) ----
   Direct transcription: segment headings th[0..M-1] (first/last pinned to the
   boundary headings) and segment lengths len[0..M-1]; positions integrated
   forward. Minimize total length plus penalties for terminal pose error and
   curvature-limit violation with Adam. No shared code with the analytic
   solution above, so agreement within 5% is meaningful. */

#define OPT_MMAX 160

/* Gauss-Seidel projection of heading jumps onto |dth| <= lim, up to maxpass
   sweeps over the chain (th[0] and th[m-1] stay pinned) */
static void project_curvature(double* th, int m, double lim, int maxpass) {
    for (int pass = 0; pass < maxpass; pass++) {
        double worst = 0;
        for (int j = 0; j + 1 < m; j++) {
            double dth = angdiff(th[j + 1], th[j]);
            double over = fabs(dth) - lim;
            if (over <= 0) continue;
            if (over > worst) worst = over;
            double sgn = dth > 0 ? 1.0 : -1.0;
            int f0 = j > 0, f1 = j + 1 < m - 1;
            if (f0 && f1) { th[j] += sgn * over / 2; th[j + 1] -= sgn * over / 2; }
            else if (f1) th[j + 1] -= sgn * over;
            else if (f0) th[j] += sgn * over;
        }
        if (worst <= 1e-12) break;
    }
}

/* one Adam run at resolution m (equal-length segments, scalar total length L).
   Decision variables: interior headings th[1..m-2] (endpoints pinned) and L.
   Curvature |dth| <= (L/m)*kmax is enforced exactly by projection.
   Warm start: th_in/m_in/L_in from a coarser level, else sweep-based init
   (0 = shortest-arc lerp, 1 = long-way lerp, 2 = aim-at-goal). */
static double opt_run(Pose s, Pose g, double rho, int m, int sweep,
                      const double* th_in, int m_in, double L_in,
                      int iters, double* th_out, double* out_L,
                      double* out_pos_err, double* out_max_curv) {
    double th[OPT_MMAX];
    double mom[OPT_MMAX], vel[OPT_MMAX];
    double momL = 0, velL = 0;
    double dist = hypot(g.x - s.x, g.y - s.y);
    double dir = dist > 1e-6 ? atan2(g.y - s.y, g.x - s.x) : s.theta;
    double L;
    if (th_in) {
        L = L_in;
        for (int j = 0; j < m; j++) {
            double u = (j + 0.5) * m_in / m - 0.5;
            int j0 = (int)floor(u);
            double f = u - j0;
            if (j0 < 0) { j0 = 0; f = 0; }
            if (j0 >= m_in - 1) { j0 = m_in - 2; f = 1; }
            th[j] = th_in[j0] + angdiff(th_in[j0 + 1], th_in[j0]) * f;
        }
    } else {
        L = dist > 1e-6 ? dist : 2 * M_PI * rho;
        double dth_init = angdiff(g.theta, s.theta);
        if (sweep == 1) dth_init += dth_init > 0 ? -2 * M_PI : 2 * M_PI;
        for (int j = 0; j < m; j++)
            th[j] = sweep == 2 ? dir
                               : s.theta + dth_init * (double)j / (m - 1);
    }
    th[0] = s.theta;
    th[m - 1] = g.theta;
    for (int i = 0; i < OPT_MMAX; i++) mom[i] = vel[i] = 0;

    const double WE = 100.0;   /* terminal position penalty */
    const double WA = 50.0;    /* terminal heading penalty (soft, on top of pin) */
    const double WK = 1000.0;  /* soft curvature penalty; projection enforces the hard limit */
    const double LR = 0.03, B1 = 0.9, B2 = 0.999, EPS = 1e-8;
    double kmax = 1.0 / rho;

    for (int it = 1; it <= iters; it++) {
        double ds = L / m;
        double gth[OPT_MMAX];
        double gL = 1.0; /* minimize L */
        for (int j = 0; j < m; j++) gth[j] = 0;
        double X = s.x, Y = s.y;
        for (int j = 0; j < m; j++) {
            X += ds * cos(th[j]);
            Y += ds * sin(th[j]);
        }
        double ex = X - g.x, ey = Y - g.y;
        for (int j = 0; j < m; j++) {
            gth[j] += WE * (2 * ex * (-ds * sin(th[j])) + 2 * ey * (ds * cos(th[j])));
            gL += WE * (2 * ex * cos(th[j]) + 2 * ey * sin(th[j])) / m;
        }
        double ea = angdiff(th[m - 1], g.theta);
        gth[m - 1] += WA * 2 * ea;
        for (int j = 0; j + 1 < m; j++) {
            double dth = angdiff(th[j + 1], th[j]);
            double viol = fabs(dth) - ds * kmax;
            if (viol > 0) {
                double sgn = dth > 0 ? 1.0 : -1.0;
                double c = WK * 2 * viol;
                gth[j] -= c * sgn;
                gth[j + 1] += c * sgn;
                gL -= c * kmax / m; /* growing L relaxes the violation */
            }
        }
        for (int j = 1; j < m - 1; j++) {
            mom[j] = B1 * mom[j] + (1 - B1) * gth[j];
            vel[j] = B2 * vel[j] + (1 - B2) * gth[j] * gth[j];
            double mh = mom[j] / (1 - pow(B1, it));
            double vh = vel[j] / (1 - pow(B2, it));
            th[j] -= LR * mh / (sqrt(vh) + EPS);
        }
        momL = B1 * momL + (1 - B1) * gL;
        velL = B2 * velL + (1 - B2) * gL * gL;
        L -= LR * (momL / (1 - pow(B1, it))) / (sqrt(velL / (1 - pow(B2, it))) + EPS);
        if (L < 1e-3) L = 1e-3;
        /* keep the chain near-feasible during optimization (cheap) */
        project_curvature(th, m, L / m * kmax, 20);
    }
    /* exact hard curvature limit for the reported solution */
    project_curvature(th, m, L / m * kmax, 2000);
    double ds = L / m;
    double X = s.x, Y = s.y, maxk = 0;
    for (int j = 0; j < m; j++) {
        X += ds * cos(th[j]);
        Y += ds * sin(th[j]);
    }
    for (int j = 0; j + 1 < m; j++) {
        double k = fabs(angdiff(th[j + 1], th[j])) / ds;
        if (k > maxk) maxk = k;
    }
    for (int j = 0; j < m; j++) th_out[j] = th[j];
    if (out_L) *out_L = L;
    if (out_pos_err) *out_pos_err = hypot(X - g.x, Y - g.y);
    if (out_max_curv) *out_max_curv = maxk;
    return L;
}

/* multi-start, coarse-to-fine numerical optimization; returns the best
   feasible length found */
static double numeric_opt_length(Pose s, Pose g, double rho, int iters,
                                 double* out_pos_err, double* out_max_curv) {
    static const int LEVELS[] = { 16, 48, OPT_MMAX };
    double best_l = -1, best_perr = 0, best_maxk = 0;
    double kmax = 1.0 / rho;
    for (int sweep = 0; sweep < 3; sweep++) {
        double th[OPT_MMAX];
        double perr = 0, maxk = 0, l = 0, L_in = 0;
        for (int lv = 0; lv < 3; lv++) {
            int m = LEVELS[lv];
            l = opt_run(s, g, rho, m, sweep,
                        lv ? th : 0, lv ? LEVELS[lv - 1] : 0, L_in,
                        iters, th, &L_in, &perr, &maxk);
        }
        int feasible = perr < 0.02 * l + 1e-3 && maxk <= kmax * 1.005;
        if (feasible && (best_l < 0 || l < best_l)) {
            best_l = l; best_perr = perr; best_maxk = maxk;
        }
    }
    if (best_l < 0) { /* none feasible: report the first sweep anyway */
        double th[OPT_MMAX], L_in = 0;
        for (int lv = 0; lv < 3; lv++)
            best_l = opt_run(s, g, rho, LEVELS[lv], 0,
                             lv ? th : 0, lv ? LEVELS[lv - 1] : 0, L_in,
                             iters, th, &L_in, &best_perr, &best_maxk);
    }
    if (out_pos_err) *out_pos_err = best_perr;
    if (out_max_curv) *out_max_curv = best_maxk;
    return best_l;
}

/* ---- self tests ---- */

static int failures = 0;

static void check(const char* name, int ok) {
    if (ok) printf("ok %s\n", name);
    else { printf("FAIL %s\n", name); failures++; }
}

static int pose_near(Pose a, Pose b, double tol) {
    return fabs(a.x - b.x) < tol && fabs(a.y - b.y) < tol &&
           fabs(angdiff(a.theta, b.theta)) < tol;
}

/* max |curvature| of uniformly arc-spaced samples, by finite differences
   (ds is the arc-length step, so arcs measure exactly 1/rho) */
static double sample_max_curv(const Pose* p, int n, double ds) {
    double maxk = 0;
    for (int i = 1; i + 1 < n; i++) {
        double k = fabs(angdiff(p[i + 1].theta, p[i - 1].theta)) / (2 * ds);
        if (k > maxk) maxk = k;
    }
    return maxk;
}

int main(void) {
    /* straight line: no turning needed */
    {
        DubinsPath p;
        Pose s = { 0, 0, 0 }, g = { 10, 0, 0 };
        check("straight_solvable", dubins_shortest(s, g, 1.0, &p) == 0);
        check("straight_length", fabs(p.length - 10.0) < 1e-9);
    }

    /* exact semicircle of radius R */
    {
        DubinsPath p;
        Pose s = { 0, 0, M_PI / 2 }, g = { 4, 0, -M_PI / 2 };
        check("semicircle_solvable", dubins_shortest(s, g, 2.0, &p) == 0);
        check("semicircle_length", fabs(p.length - 2.0 * M_PI) < 1e-9);
    }

    /* rigid-transform invariance: rotate + translate both poses */
    {
        DubinsPath p1, p2;
        Pose s = { 1, 2, 0.7 }, g = { 8, -3, -1.9 };
        check("invariance_solvable", dubins_shortest(s, g, 1.5, &p1) == 0);
        double a = 0.9, ca = cos(a), sa = sin(a), tx = -5, ty = 11;
        Pose s2 = { tx + ca * s.x - sa * s.y, ty + sa * s.x + ca * s.y, s.theta + a };
        Pose g2 = { tx + ca * g.x - sa * g.y, ty + sa * g.x + ca * g.y, g.theta + a };
        check("invariance_solvable2", dubins_shortest(s2, g2, 1.5, &p2) == 0);
        check("invariance_length", fabs(p1.length - p2.length) < 1e-9);
    }

    /* sampling: configurable count, exact endpoints, curvature bounded */
    {
        Pose s = { 0, 0, 0.3 }, g = { 12, 7, 2.4 };
        double rho = 2.5;
        DubinsPath p;
        check("sample_solvable", dubins_shortest(s, g, rho, &p) == 0);
        int counts[] = { 2, 7, 33, 500 };
        for (int ci = 0; ci < 4; ci++) {
            int n = counts[ci];
            Pose pts[500];
            char name[64];
            snprintf(name, sizeof name, "sample_count_%d", n);
            check(name, dubins_sample(&p, s, pts, n) == 0);
            snprintf(name, sizeof name, "sample_endpoints_%d", n);
            check(name, pose_near(pts[0], s, 1e-9) && pose_near(pts[n - 1], g, 1e-9));
        }
        Pose dense[2000];
        check("sample_dense", dubins_sample(&p, s, dense, 2000) == 0);
        check("sample_curvature_bound",
              sample_max_curv(dense, 2000, p.length / 1999) <= 1.0 / rho * (1 + 1e-9));
        check("sample_reject_bad_count", dubins_sample(&p, s, dense, 1) == -1);
    }

    /* analytic length vs independent numerical optimization, within 5% */
    {
        Pose cases[][2] = {
            { { 0, 0, 0 }, { 10, 5, 0.5 } },     /* long cruise + bend */
            { { 0, 0, 0 }, { 4, 0, M_PI } },     /* U-turn (CCC territory) */
            { { 0, 0, 1.2 }, { -6, 3, -2.0 } },  /* reverse-ish swing */
        };
        double rhos[] = { 2.0, 1.0, 1.5 };
        for (int c = 0; c < 3; c++) {
            DubinsPath p;
            char name[64];
            snprintf(name, sizeof name, "opt_solvable_%d", c);
            check(name, dubins_shortest(cases[c][0], cases[c][1], rhos[c], &p) == 0);
            double perr, maxk;
            double lnum = numeric_opt_length(cases[c][0], cases[c][1], rhos[c],
                                             20000, &perr, &maxk);
            snprintf(name, sizeof name, "opt_converged_%d", c);
            check(name, perr < 0.05 * p.length + 1e-6 &&
                        maxk <= 1.0 / rhos[c] * 1.02);
            snprintf(name, sizeof name, "opt_length_within_5pct_%d", c);
            check(name, fabs(lnum - p.length) <= 0.05 * p.length);
        }
    }

    printf(failures ? "FAILURES: %d\n" : "all tests passed\n", failures);
    return failures ? 1 : 0;
}
