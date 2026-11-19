
/* lod-selector: screen-space-error LOD selection with hysteresis, a bounded
   switch history (direction + magnitude) and a rolling per-frame error window
   with threshold alerts. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define LEVELS 4
#define HISTORY_CAP 16   /* retained switch events, oldest overwritten */
#define WINDOW_CAP 64    /* retained per-frame samples, oldest overwritten */

typedef struct {
    float error[LEVELS];   /* geometric error per level, world units */
    float radius;          /* bounding radius */
    float tau;             /* hysteresis ratio, e.g. 1.1 */
} MeshLod;

typedef struct {
    int level;
    int switches;
    int thrash;            /* switches within one unit of distance change */
} State;

typedef struct {
    int level;             /* selected level this frame */
    float error_px;        /* selected level's projected error, px */
    float prev_error_px;   /* previous level's projected error, px */
    int changed;           /* 1 if the level changed this frame */
} LodChoice;

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* pick coarsest level whose projected error is below tolerance */
static LodChoice lod_select(const MeshLod* m, float distance, float fov_y, int screen_h,
                            float tolerance_px, State* st) {
    LodChoice out;
    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    int prev = st->level;
    int next = prev;
    if (want != prev) {
        /* hysteresis: require the candidate to beat the threshold by tau */
        float e_new = screen_error(m->error[want], distance, fov_y, screen_h);
        float e_old = screen_error(m->error[prev], distance, fov_y, screen_h);
        int blocked = (want > prev && e_new > tolerance_px * m->tau) ||
                      (want < prev && e_old < tolerance_px / m->tau);
        if (!blocked) {
            next = want;
            st->switches++;
        }
    }
    out.level = next;
    out.error_px = screen_error(m->error[next], distance, fov_y, screen_h);
    out.prev_error_px = screen_error(m->error[prev], distance, fov_y, screen_h);
    out.changed = (next != prev);
    st->level = next;
    return out;
}

/* ---- switch history ---------------------------------------------------- */

typedef struct {
    unsigned long frame;
    double t_ms;
    int from;
    int to;
    int dir;          /* +1 refine (finer level), -1 coarsen */
    int levels;       /* |to - from|: how many levels were crossed */
    float error_from; /* projected error before the switch, px */
    float error_to;   /* projected error after the switch, px */
} LodSwitch;

/* ---- per-frame error window -------------------------------------------- */

typedef struct {
    double t_ms;
    int level;
    float error_px;   /* error of the level that was actually selected */
    int switched;
} LodSample;

typedef struct {
    int samples;
    int switches;
    double sum_error;   /* accumulated selected-level error, px */
    double span_ms;     /* time from first to last sample in the window */
    float mean_error;
    float max_error;
} LodWindowStats;

typedef struct {
    /* bounded switch history ring */
    LodSwitch hist[HISTORY_CAP];
    int hist_head;              /* next write slot */
    int hist_count;             /* retained events, <= HISTORY_CAP */
    unsigned long hist_total;   /* switches ever seen */

    /* bounded per-frame sample ring */
    LodSample win[WINDOW_CAP];
    int win_head;
    int win_count;

    /* window / alert policy */
    double window_ms;
    float alert_px;
    double alert_cooldown_ms;
    double last_alert_ms;
    unsigned long alerts;
} LodTrace;

static void trace_init(LodTrace* tr, double window_ms, float alert_px, double cooldown_ms) {
    memset(tr, 0, sizeof *tr);
    tr->window_ms = window_ms;
    tr->alert_px = alert_px;
    tr->alert_cooldown_ms = cooldown_ms;
    tr->last_alert_ms = -1e30;
}

static const LodSwitch* hist_at(const LodTrace* tr, int i) {
    int idx = (tr->hist_head - tr->hist_count + i + HISTORY_CAP) % HISTORY_CAP;
    return &tr->hist[idx];
}

static const LodSample* win_at(const LodTrace* tr, int i) {
    int idx = (tr->win_head - tr->win_count + i + WINDOW_CAP) % WINDOW_CAP;
    return &tr->win[idx];
}

/* Aggregate the retained samples inside the time window. Samples older than
   window_ms are ignored; at high frame rates the ring capacity (WINDOW_CAP)
   is the binding limit, so memory stays constant. */
static LodWindowStats trace_window(const LodTrace* tr, double now_ms) {
    LodWindowStats st;
    memset(&st, 0, sizeof st);
    double cutoff = now_ms - tr->window_ms;
    int have = 0;
    double t_first = 0.0, t_last = 0.0;

    for (int i = 0; i < tr->win_count; i++) {
        const LodSample* s = win_at(tr, i);
        if (s->t_ms < cutoff) continue;
        if (!have) { t_first = s->t_ms; have = 1; }
        t_last = s->t_ms;
        st.samples++;
        st.switches += s->switched;
        st.sum_error += (double)s->error_px;
        if (s->error_px > st.max_error) st.max_error = s->error_px;
    }
    if (st.samples > 0) {
        st.mean_error = (float)(st.sum_error / st.samples);
        st.span_ms = t_last - t_first;
    }
    return st;
}

/* Record one frame: append the switch event (if any) and the per-frame error,
   then raise an alert when the window mean exceeds the threshold. Returns 1
   when an alert was raised on this frame. */
static int trace_record(LodTrace* tr, unsigned long frame, double t_ms,
                        int prev_level, LodChoice c) {
    if (c.changed) {
        LodSwitch ev;
        memset(&ev, 0, sizeof ev);
        ev.frame = frame;
        ev.t_ms = t_ms;
        ev.from = prev_level;
        ev.to = c.level;
        ev.dir = (c.level < prev_level) ? +1 : -1;
        ev.levels = (c.level > prev_level) ? c.level - prev_level : prev_level - c.level;
        ev.error_from = c.prev_error_px;
        ev.error_to = c.error_px;

        tr->hist[tr->hist_head] = ev;
        tr->hist_head = (tr->hist_head + 1) % HISTORY_CAP;
        if (tr->hist_count < HISTORY_CAP) tr->hist_count++;
        tr->hist_total++;
    }

    LodSample* s = &tr->win[tr->win_head];
    s->t_ms = t_ms;
    s->level = c.level;
    s->error_px = c.error_px;
    s->switched = c.changed;
    tr->win_head = (tr->win_head + 1) % WINDOW_CAP;
    if (tr->win_count < WINDOW_CAP) tr->win_count++;

    LodWindowStats w = trace_window(tr, t_ms);
    if (w.samples > 0 && w.mean_error > tr->alert_px &&
        (t_ms - tr->last_alert_ms) >= tr->alert_cooldown_ms) {
        tr->alerts++;
        tr->last_alert_ms = t_ms;
        return 1;
    }
    return 0;
}

/* ---- test harness ------------------------------------------------------ */

typedef struct {
    double t_ms;
    int level;
    float error_px;
    int switched;
} RawFrame;

/* Independent recomputation from the raw frame log. It applies the same two
   limits as the ring: at most WINDOW_CAP newest frames, none older than the
   time window. */
static LodWindowStats reference_window(const RawFrame* raw, int n,
                                       double now_ms, double window_ms) {
    LodWindowStats st;
    memset(&st, 0, sizeof st);
    int start = (n > WINDOW_CAP) ? n - WINDOW_CAP : 0;
    double cutoff = now_ms - window_ms;
    while (start < n && raw[start].t_ms < cutoff) start++;

    int have = 0;
    double t_first = 0.0, t_last = 0.0;
    for (int i = start; i < n; i++) {
        if (!have) { t_first = raw[i].t_ms; have = 1; }
        t_last = raw[i].t_ms;
        st.samples++;
        st.switches += raw[i].switched;
        st.sum_error += (double)raw[i].error_px;
        if (raw[i].error_px > st.max_error) st.max_error = raw[i].error_px;
    }
    if (st.samples > 0) {
        st.mean_error = (float)(st.sum_error / st.samples);
        st.span_ms = t_last - t_first;
    }
    return st;
}

static int stats_equal(const LodWindowStats* a, const LodWindowStats* b) {
    if (a->samples != b->samples || a->switches != b->switches) return 0;
    double tol = 1e-9;
    if (fabs(a->sum_error - b->sum_error) > tol * (1.0 + fabs(a->sum_error))) return 0;
    if (fabs(a->span_ms - b->span_ms) > tol * (1.0 + fabs(a->span_ms))) return 0;
    if (fabsf(a->mean_error - b->mean_error) > 1e-4f) return 0;
    if (fabsf(a->max_error - b->max_error) > 1e-4f) return 0;
    return 1;
}

/* log-triangle camera path: 5 -> 800 -> 5 world units, 200 frames/cycle */
static float camera_distance(int i) {
    int k = i % 200;
    double half = (k < 100) ? k / 100.0 : (200 - k) / 100.0;
    return (float)(5.0 * pow(160.0, half));
}

static int switches_equal(const LodSwitch* a, const LodSwitch* b) {
    return a->frame == b->frame && a->t_ms == b->t_ms &&
           a->from == b->from && a->to == b->to &&
           a->dir == b->dir && a->levels == b->levels &&
           a->error_from == b->error_from && a->error_to == b->error_to;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    State st = {0, 0, 0};
    const float fov = 1.0f;
    const int screen_h = 1080;
    const float tolerance_px = 50.0f;
    const double dt_ms = 5.0;          /* 200 fps: ring capacity binds */
    const double window_ms = 500.0;
    enum { frames = 1000 };

    LodTrace tr;
    trace_init(&tr, window_ms, 20.0f, 100.0);

    static RawFrame raw[frames];
    static LodSwitch raw_sw[frames];
    int raw_n = 0, raw_sw_n = 0;
    int consistent = 1, win_ok = 1, max_samples = 0, switches = 0;
    int lv_hist[LEVELS] = {0};
    double first_alert_t = -1.0;

    double t = 0.0;
    for (int i = 0; i < frames; i++) {
        float d = camera_distance(i);
        int prev = st.level;
        LodChoice c = lod_select(&m, d, fov, screen_h, tolerance_px, &st);

        if (trace_record(&tr, (unsigned long)i, t, prev, c)) {
            if (first_alert_t < 0.0) first_alert_t = t;
        }
        if (c.changed) switches++;
        lv_hist[c.level]++;

        /* per-frame recomputation from the selected level, independent of the
           value used inside the selector */
        RawFrame rf;
        rf.t_ms = t;
        rf.level = c.level;
        rf.error_px = screen_error(m.error[c.level], d, fov, screen_h);
        rf.switched = c.changed;
        raw[raw_n++] = rf;

        LodWindowStats inc = trace_window(&tr, t);
        LodWindowStats ref = reference_window(raw, raw_n, t, window_ms);
        if (!stats_equal(&inc, &ref)) consistent = 0;
        if (inc.samples > WINDOW_CAP) win_ok = 0;
        if (inc.samples > max_samples) max_samples = inc.samples;

        if (c.changed) {
            LodSwitch sw;
            memset(&sw, 0, sizeof sw);
            sw.frame = (unsigned long)i;
            sw.t_ms = t;
            sw.from = prev;
            sw.to = c.level;
            sw.dir = (c.level < prev) ? +1 : -1;
            sw.levels = (c.level > prev) ? c.level - prev : prev - c.level;
            sw.error_from = screen_error(m.error[prev], d, fov, screen_h);
            sw.error_to = rf.error_px;
            raw_sw[raw_sw_n++] = sw;
        }
        t += dt_ms;
    }

    /* retained history must be exactly the most recent HISTORY_CAP events */
    int hist_ok = tr.hist_total == (unsigned long)raw_sw_n;
    int keep = (raw_sw_n < HISTORY_CAP) ? raw_sw_n : HISTORY_CAP;
    if (tr.hist_count != keep) hist_ok = 0;
    if (hist_ok) {
        for (int i = 0; i < tr.hist_count; i++) {
            if (!switches_equal(hist_at(&tr, i), &raw_sw[raw_sw_n - tr.hist_count + i]))
                hist_ok = 0;
        }
    }

    /* direction/magnitude bookkeeping and the net level change */
    int dir_ok = 1, err_ok = 1, net = 0;
    for (int i = 0; i < raw_sw_n; i++) {
        const LodSwitch* s = &raw_sw[i];
        int mag = (s->to > s->from) ? s->to - s->from : s->from - s->to;
        int dir = (s->to < s->from) ? +1 : -1;
        if (s->levels != mag || s->dir != dir) dir_ok = 0;
        net += s->dir * s->levels;
        if (s->dir > 0 && !(s->error_to < s->error_from)) err_ok = 0;   /* refine */
        if (s->dir < 0 && !(s->error_to > s->error_from)) err_ok = 0;   /* coarsen */
    }
    if (net != st.level - 0) dir_ok = 0;

    int levels_ok = 1;
    for (int i = 0; i < LEVELS; i++) if (lv_hist[i] == 0) levels_ok = 0;

    int mem_ok = tr.hist_count <= HISTORY_CAP && tr.win_count <= WINDOW_CAP &&
                 raw_sw_n > HISTORY_CAP;   /* overflow path was exercised */
    int cap_bound = (max_samples == WINDOW_CAP);

    /* alerts must stay silent when the threshold cannot be reached */
    LodTrace calm;
    trace_init(&calm, window_ms, 1e9f, 100.0);
    State cs = {0, 0, 0};
    int calm_alerts = 0;
    t = 0.0;
    for (int i = 0; i < 200; i++) {
        LodChoice c = lod_select(&m, camera_distance(i), fov, screen_h, tolerance_px, &cs);
        calm_alerts += trace_record(&calm, (unsigned long)i, t, cs.level, c);
        t += dt_ms;
    }

    int alert_ok = tr.alerts > 0 && calm_alerts == 0;

    printf("levels=%d,%d,%d,%d switches=%d\n",
           lv_hist[0], lv_hist[1], lv_hist[2], lv_hist[3], switches);
    printf("hist_total=%lu hist_kept=%d hist_cap=%d\n",
           tr.hist_total, tr.hist_count, HISTORY_CAP);
    printf("history_ok=%d direction_ok=%d error_sign_ok=%d\n", hist_ok, dir_ok, err_ok);
    printf("window_ms=%.0f window_cap=%d max_samples=%d span_ms=%.1f\n",
           window_ms, WINDOW_CAP, max_samples, trace_window(&tr, t - dt_ms).span_ms);
    printf("window_consistency_ok=%d window_bound_ok=%d\n", consistent, win_ok);
    printf("alert_px=%.0f alerts=%lu first_alert_t=%.0f alert_ok=%d\n",
           tr.alert_px, tr.alerts, first_alert_t, alert_ok);
    printf("trace_bytes=%zu history_bytes=%zu\n",
           sizeof(LodTrace), sizeof(LodSwitch) * HISTORY_CAP);
    printf("memory_ok=%d cap_bound=%d levels_ok=%d\n", mem_ok, cap_bound, levels_ok);

    int ok = hist_ok && dir_ok && err_ok && consistent && win_ok && mem_ok &&
             cap_bound && levels_ok && alert_ok;
    printf("all_ok=%d\n", ok);
    return ok ? 0 : 1;
}
