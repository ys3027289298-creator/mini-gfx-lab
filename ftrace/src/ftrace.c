/* ftrace: frame timeline tracer.
   Records per-frame, per-stage durations into a fixed ring buffer with no
   heap allocation on the capture path, keeps online log-scale histograms,
   and builds a single report object that both the console printer and the
   CSV/JSON exporters render, so exported numbers cannot diverge from the
   runtime printout. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <inttypes.h>
#include <time.h>

#define FT_MAX_STAGES   16
#define FT_MAX_FRAMES   4096
#define FT_HIST_BUCKETS 41      /* bucket 0: zero; 1..39: [2^(b-1), 2^b) ns; 40: >= 2^39 ns */
#define FT_NAME_MAX     31
#define FT_NPCT         4
static const int FT_PERCENTILES[FT_NPCT] = { 50, 90, 95, 99 };

typedef struct {
    const char* name;
    uint64_t hist[FT_HIST_BUCKETS];
    uint64_t observed_cnt;      /* every observed frame, even ones rotated out of the ring */
} FtStageStat;

typedef struct {
    uint64_t index;
    uint64_t stage_ns[FT_MAX_STAGES];
    uint64_t total_ns;
    uint8_t  used[FT_MAX_STAGES];
} FtFrame;

typedef struct {
    /* scalar capture state first; the fixed arrays live at the tail so ring
       wrap-around can never alias the metadata above. */
    int          stage_count;
    uint32_t     head;
    uint64_t     frames_seen;
    int          wrapped;
    uint64_t     frame_start_ns;
    volatile uint64_t blackhole;   /* sink so capture writes cannot be optimized away */
    uint64_t     open_ns[FT_MAX_STAGES];
    uint8_t      open[FT_MAX_STAGES];
    FtFrame      cur;
    FtStageStat  st[FT_MAX_STAGES];
    uint64_t     sort_scratch[FT_MAX_FRAMES];
    FtFrame      ring[FT_MAX_FRAMES];
} FtTrace;

typedef struct {
    const char* name;
    uint64_t n;                 /* samples this row is based on (retained window) */
    uint64_t min_ns, mean_ns, max_ns;
    uint64_t pct[FT_NPCT];
    uint64_t hist[FT_HIST_BUCKETS];
    uint64_t hist_count;        /* frames behind the histogram, which never rotates out */
} FtStageReport;

typedef struct {
    int wrapped;
    uint64_t observed_frames;
    uint64_t retained_frames;
    FtStageReport stage[FT_MAX_STAGES];
    FtStageReport frame_total;
} FtReport;

/* ---------- time source: vdso-backed monotonic clock, no syscall on modern Linux */

static inline uint64_t ft_now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

/* log2 bucket: v in [2^(b-1), 2^b) lands in b; v == 0 lands in bucket 0 */
static int ft_hist_bucket(uint64_t v) {
    if (v == 0) return 0;
    int b = 64 - __builtin_clzll((unsigned long long)v);
    return b < FT_HIST_BUCKETS ? b : FT_HIST_BUCKETS - 1;
}

static void ft_hist_edges(int b, uint64_t* low, uint64_t* high) {
    if (b == 0) { *low = 0; *high = 1; return; }
    *low = 1ull << (b - 1);
    *high = b >= FT_HIST_BUCKETS - 1 ? UINT64_MAX : (1ull << b);
}

/* ---------- capture API: allocation-free, O(stages) per frame at worst ---------- */

static void ft_init(FtTrace* t) {
    memset(t, 0, sizeof(*t));
}

/* names must be stable strings (literals) without commas, quotes or spaces */
static int ft_stage(FtTrace* t, const char* name) {
    if (!t || !name) return -1;
    size_t len = strlen(name);
    if (len == 0 || len > FT_NAME_MAX) return -1;
    for (size_t i = 0; i < len; i++)
        if (name[i] == ',' || name[i] == '"' || name[i] == ' ' || name[i] == '\t')
            return -1;
    for (int i = 0; i < t->stage_count; i++)
        if (strcmp(t->st[i].name, name) == 0) return i;
    if (t->stage_count >= FT_MAX_STAGES) return -1;
    int id = t->stage_count++;
    t->st[id].name = name;
    return id;
}

static void ft_frame_begin(FtTrace* t) {
    if (!t) return;
    memset(&t->cur, 0, sizeof(t->cur));
    memset(t->open, 0, sizeof(t->open));
    t->cur.index = t->frames_seen;
    t->frame_start_ns = ft_now_ns();
}

static void ft_begin(FtTrace* t, int id) {
    if (!t || id < 0 || id >= FT_MAX_STAGES || id >= t->stage_count || t->open[id]) return;
    t->open[id] = 1;
    t->open_ns[id] = ft_now_ns();
}

static void ft_end(FtTrace* t, int id) {
    if (!t || id < 0 || id >= FT_MAX_STAGES || id >= t->stage_count || !t->open[id]) return;
    t->open[id] = 0;
    uint64_t d = ft_now_ns() - t->open_ns[id];
    t->cur.used[id] = 1;
    t->cur.stage_ns[id] += d;
}

static void ft_frame_end(FtTrace* t) {
    if (!t) return;
    /* a stage left open is closed at the frame boundary */
    for (int id = 0; id < t->stage_count; id++) {
        if (!t->open[id]) continue;
        t->open[id] = 0;
        uint64_t d = ft_now_ns() - t->open_ns[id];
        t->cur.used[id] = 1;
        t->cur.stage_ns[id] += d;
    }

    uint64_t total = 0;
    for (int id = 0; id < t->stage_count; id++) {
        if (!t->cur.used[id]) continue;
        uint64_t d = t->cur.stage_ns[id];
        total += d;
        FtStageStat* s = &t->st[id];
        s->observed_cnt++;
        s->hist[ft_hist_bucket(d)]++;
    }
    t->cur.total_ns = total ? total : ft_now_ns() - t->frame_start_ns;

    t->ring[t->head] = t->cur;
    t->blackhole += t->cur.total_ns;
    t->head++;
    if (t->head == FT_MAX_FRAMES) { t->head = 0; t->wrapped = 1; }
    t->frames_seen++;
}

/* test seam: inject a frame with fixed durations, bypassing the clock entirely */
static void ft_inject_frame(FtTrace* t, const uint64_t* stage_ns, int count) {
    if (!t) return;
    memset(&t->cur, 0, sizeof(t->cur));
    memset(t->open, 0, sizeof(t->open));
    t->cur.index = t->frames_seen;
    if (count > t->stage_count) count = t->stage_count;
    for (int id = 0; id < count; id++) {
        t->cur.used[id] = 1;              /* injection makes it an explicit sample, even at 0 ns */
        t->cur.stage_ns[id] = stage_ns[id];
    }
    ft_frame_end(t);
}

/* ---------- aggregation: one report object, rendered by every exporter ---------- */

static int ft_cmp_u64(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
    return (x > y) - (x < y);
}

/* nearest-rank, 1-indexed: rank = ceil(p/100 * n), result is the rank-th smallest */
static uint64_t ft_percentile(const uint64_t* sorted, uint64_t n, int p) {
    if (n == 0) return 0;
    uint64_t rank = ((uint64_t)p * n + 99) / 100;
    if (rank < 1) rank = 1;
    if (rank > n) rank = n;
    return sorted[rank - 1];
}

static void ft_build_stage_row(FtTrace* t, FtStageReport* r, const char* name,
                               const FtStageStat* s, int is_total) {
    memset(r, 0, sizeof(*r));
    r->name = name;
    if (s) {
        memcpy(r->hist, s->hist, sizeof(r->hist));
        r->hist_count = s->observed_cnt;
    }
    uint64_t* vals = t->sort_scratch;   /* shared scratch, rows are built one at a time */
    uint64_t n = 0, sum = 0;
    uint64_t retained = t->wrapped ? FT_MAX_FRAMES : t->frames_seen;
    for (uint64_t f = 0; f < retained; f++) {
        const FtFrame* fr = &t->ring[f];
        uint64_t v = is_total ? fr->total_ns
                   : (fr->used[s - t->st] ? fr->stage_ns[s - t->st] : 0);
        if (is_total || fr->used[s - t->st]) {
            vals[n++] = v;
            sum += v;
        }
    }
    r->n = n;
    if (n == 0) return;
    qsort(vals, n, sizeof(uint64_t), ft_cmp_u64);
    r->min_ns = vals[0];
    r->max_ns = vals[n - 1];
    r->mean_ns = sum / n;
    for (int i = 0; i < FT_NPCT; i++)
        r->pct[i] = ft_percentile(vals, n, FT_PERCENTILES[i]);
}

static FtReport ft_report_build(FtTrace* t) {
    FtReport rep;
    memset(&rep, 0, sizeof(rep));
    rep.wrapped = t->wrapped;
    rep.observed_frames = t->frames_seen;
    rep.retained_frames = t->wrapped ? FT_MAX_FRAMES : t->frames_seen;
    for (int id = 0; id < t->stage_count; id++)
        ft_build_stage_row(t, &rep.stage[id], t->st[id].name, &t->st[id], 0);
    ft_build_stage_row(t, &rep.frame_total, "frame.total", NULL, 1);
    return rep;
}

/* ---------- renderers: console and CSV consume the same FtReport rows ---------- */

#define FT_STATS_HEADER "name,n,min_ns,mean_ns,p50_ns,p90_ns,p95_ns,p99_ns,max_ns"

static void ft_row_write_csv(FILE* f, const FtStageReport* r) {
    fprintf(f, "%s,%" PRIu64 ",%" PRIu64 ",%" PRIu64,
            r->name, r->n, r->min_ns, r->mean_ns);
    for (int i = 0; i < FT_NPCT; i++) fprintf(f, ",%" PRIu64, r->pct[i]);
    fprintf(f, ",%" PRIu64 "\n", r->max_ns);
}

static void ft_report_print(const FtReport* rep, FILE* f) {
    fprintf(f, "observed_frames=%" PRIu64 " retained=%" PRIu64 " wrapped=%d\n",
            rep->observed_frames, rep->retained_frames, rep->wrapped);
    fprintf(f, "%-12s %8s %10s %10s %10s %10s %10s %10s %10s\n",
            "name", "n", "min_ns", "mean_ns", "p50_ns", "p90_ns", "p95_ns", "p99_ns", "max_ns");
    for (int id = 0; id < FT_MAX_STAGES && rep->stage[id].name; id++) {
        const FtStageReport* r = &rep->stage[id];
        fprintf(f, "%-12s %8" PRIu64 " %10" PRIu64 " %10" PRIu64,
                r->name, r->n, r->min_ns, r->mean_ns);
        for (int i = 0; i < FT_NPCT; i++) fprintf(f, " %10" PRIu64, r->pct[i]);
        fprintf(f, " %10" PRIu64 "\n", r->max_ns);
    }
    const FtStageReport* r = &rep->frame_total;
    fprintf(f, "%-12s %8" PRIu64 " %10" PRIu64 " %10" PRIu64,
            r->name, r->n, r->min_ns, r->mean_ns);
    for (int i = 0; i < FT_NPCT; i++) fprintf(f, " %10" PRIu64, r->pct[i]);
    fprintf(f, " %10" PRIu64 "\n", r->max_ns);
}

static void ft_stats_write_csv(const FtReport* rep, FILE* f) {
    fprintf(f, FT_STATS_HEADER "\n");
    for (int id = 0; id < FT_MAX_STAGES && rep->stage[id].name; id++)
        ft_row_write_csv(f, &rep->stage[id]);
    ft_row_write_csv(f, &rep->frame_total);
}

static void ft_hist_write_csv(const FtReport* rep, FILE* f) {
    fprintf(f, "name,bucket,low_ns,high_ns,count\n");
    for (int id = 0; id < FT_MAX_STAGES && rep->stage[id].name; id++) {
        const FtStageReport* r = &rep->stage[id];
        for (int b = 0; b < FT_HIST_BUCKETS; b++) {
            if (r->hist[b] == 0) continue;
            uint64_t low, high;
            ft_hist_edges(b, &low, &high);
            fprintf(f, "%s,%d,%" PRIu64 ",%" PRIu64 ",%" PRIu64 "\n",
                    r->name, b, low, high, r->hist[b]);
        }
    }
}

static uint32_t ft_ring_slot(const FtTrace* t, uint64_t k) {
    /* oldest-first: when wrapped, head points at the oldest slot; before the
       first wrap the occupied slots are simply 0..frames_seen-1. */
    return t->wrapped ? (uint32_t)((t->head + k) % FT_MAX_FRAMES) : (uint32_t)k;
}

static void ft_timeline_write_csv(const FtTrace* t, FILE* f) {
    fprintf(f, "frame_index,kind,name,duration_ns\n");
    uint64_t retained = t->wrapped ? FT_MAX_FRAMES : t->frames_seen;
    for (uint64_t k = 0; k < retained; k++) {
        const FtFrame* fr = &t->ring[ft_ring_slot(t, k)];
        for (int id = 0; id < t->stage_count; id++) {
            if (!fr->used[id]) continue;
            fprintf(f, "%" PRIu64 ",stage,%s,%" PRIu64 "\n",
                    fr->index, t->st[id].name, fr->stage_ns[id]);
        }
        fprintf(f, "%" PRIu64 ",total,frame.total,%" PRIu64 "\n", fr->index, fr->total_ns);
    }
}

/* Chrome Trace Event format, loadable by chrome://tracing and Perfetto.
   Frame index is used as the trace timestamp base; values are integer us. */
static void ft_trace_write_json(const FtTrace* t, FILE* f) {
    fprintf(f, "[\n");
    uint64_t retained = t->wrapped ? FT_MAX_FRAMES : t->frames_seen;
    int first = 1;
    for (uint64_t k = 0; k < retained; k++) {
        const FtFrame* fr = &t->ring[ft_ring_slot(t, k)];
        uint64_t ts_us = fr->index * 1000;   /* one synthetic frame == 1 ms of trace time */
        uint64_t cursor = ts_us;
        for (int id = 0; id < t->stage_count; id++) {
            if (!fr->used[id]) continue;
            uint64_t dur_us = fr->stage_ns[id] / 1000;
            if (!first) fputs(",\n", f);
            first = 0;
            fprintf(f,
                "{\"name\":\"%s\",\"cat\":\"ftrace\",\"ph\":\"X\",\"ts\":%" PRIu64
                ",\"dur\":%" PRIu64 ",\"pid\":1,\"tid\":1}",
                t->st[id].name, cursor, dur_us);
            cursor += dur_us;
        }
        if (!first) fputs(",\n", f);
        first = 0;
        fprintf(f,
            "{\"name\":\"frame.total\",\"cat\":\"ftrace\",\"ph\":\"I\",\"ts\":%" PRIu64
            ",\"pid\":1,\"tid\":1}", ts_us);
    }
    fputs("\n]\n", f);
}

/* ---------- self tests ---------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* parse the whitespace table printed by ft_report_print: rows are
   "name n min mean p50 p90 p95 p99 max"; the pseudo stage is frame.total */
static int text_row(FILE* f, const char* want, uint64_t out[9]) {
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char name[40];
        if (sscanf(line, "%39s %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64
                          " %" SCNu64 " %" SCNu64 " %" SCNu64 " %" SCNu64,
                   name, &out[1], &out[2], &out[3], &out[4], &out[5],
                   &out[6], &out[7], &out[8]) == 9) {
            if (strcmp(name, want) == 0) { out[0] = 1; return 1; }
        }
    }
    return 0;
}

static int csv_row(FILE* f, const char* want, uint64_t out[9]) {
    char line[256];
    while (fgets(line, sizeof(line), f)) {
        char* comma = strchr(line, ',');
        if (!comma) continue;
        *comma = '\0';
        if (strcmp(line, want) != 0) continue;
        char* p = comma + 1;
        int k = 1;
        for (; k < 9 && *p; k++) {
            out[k] = strtoull(p, &p, 10);
            if (*p == ',') p++;
        }
        out[0] = 1;
        return k == 9;
    }
    return 0;
}

static void test_deterministic(void) {
    FtTrace t; ft_init(&t);
    int a = ft_stage(&t, "update");
    int b = ft_stage(&t, "draw");
    check("stage_ids", a == 0 && b == 1);
    check("stage_reject_bad_name", ft_stage(&t, "has space") == -1
                                 && ft_stage(NULL, "x") == -1);

    /* 100 frames: update fixed 7us, draw alternates 1us / 3us */
    uint64_t vals[2];
    for (int i = 0; i < 100; i++) {
        vals[0] = 7000;
        vals[1] = (i & 1) ? 3000 : 1000;
        ft_inject_frame(&t, vals, 2);
    }
    FtReport rep = ft_report_build(&t);
    check("retained_full", rep.observed_frames == 100 && rep.retained_frames == 100
                        && rep.wrapped == 0);

    FtStageReport* u = &rep.stage[0];
    check("update_n_mean", u->n == 100 && u->mean_ns == 7000 && u->min_ns == 7000
                        && u->max_ns == 7000 && u->pct[0] == 7000 && u->pct[3] == 7000);
    FtStageReport* d = &rep.stage[1];
    check("draw_n_minmax", d->n == 100 && d->min_ns == 1000 && d->max_ns == 3000
                        && d->mean_ns == 2000);
    check("draw_p50_p99", d->pct[0] == 1000 && d->pct[3] == 3000);
    /* 90th nearest-rank of 100 samples: rank 90; sorted 50x1000 then 50x3000 -> 3000 */
    check("draw_p90", d->pct[1] == 3000 && d->pct[2] == 3000);

    check("frame_total_stats", rep.frame_total.n == 100
                        && rep.frame_total.min_ns == 8000
                        && rep.frame_total.max_ns == 10000
                        && rep.frame_total.mean_ns == 9000);

    /* histogram: 7000 is in [4096,8192) -> bucket 13; every frame contributes */
    check("hist_bucket_known", ft_hist_bucket(0) == 0 && ft_hist_bucket(1) == 1
                        && ft_hist_bucket(4096) == 13 && ft_hist_bucket(8192) == 14
                        && ft_hist_bucket(1ull << 40) == FT_HIST_BUCKETS - 1);
    check("hist_count_never_rotates", u->hist_count == 100 && u->hist[13] == 100);
}

static void test_ring_wrap(void) {
    FtTrace t; ft_init(&t);
    int a = ft_stage(&t, "s0");
    (void)a;
    uint64_t vals[1];
    for (uint64_t i = 0; i < FT_MAX_FRAMES + 500; i++) {
        vals[0] = i + 1;                 /* 1ns .. 4596ns, strictly increasing */
        ft_inject_frame(&t, vals, 1);
    }
    FtReport rep = ft_report_build(&t);
    check("wrap_flag_window", rep.wrapped == 1
                        && rep.observed_frames == FT_MAX_FRAMES + 500
                        && rep.retained_frames == FT_MAX_FRAMES);
    /* retained window holds the last 4096 injected values: 501..4596 */
    check("wrap_retained_range", rep.stage[0].min_ns == 501
                        && rep.stage[0].max_ns == FT_MAX_FRAMES + 500
                        && rep.stage[0].n == FT_MAX_FRAMES);
    /* histogram never rotates: all 4596 samples accounted for */
    uint64_t hsum = 0;
    for (int b = 0; b < FT_HIST_BUCKETS; b++) hsum += rep.stage[0].hist[b];
    check("wrap_hist_full_history", hsum == FT_MAX_FRAMES + 500
                        && rep.stage[0].hist_count == FT_MAX_FRAMES + 500);
}

static void test_print_equals_export(void) {
    FtTrace t; ft_init(&t);
    ft_stage(&t, "update");
    ft_stage(&t, "draw");
    uint64_t vals[2];
    for (int i = 0; i < 250; i++) {
        vals[0] = 5000 + (uint64_t)(i * 7) % 9000;
        vals[1] = 2000 + (uint64_t)(i * 3) % 4000;
        ft_inject_frame(&t, vals, 2);
    }
    FtReport rep = ft_report_build(&t);

    FILE* tf = tmpfile();
    FILE* cf = tmpfile();
    if (!tf || !cf) { check("print_equals_export_open", 0); return; }
    ft_report_print(&rep, tf);
    ft_stats_write_csv(&rep, cf);
    rewind(tf); rewind(cf);

    const char* rows[3] = { "update", "draw", "frame.total" };
    int ok = 1;
    for (int i = 0; i < 3; i++) {
        uint64_t tr[9] = {0}, cr[9] = {0};
        int got_t = text_row(tf, rows[i], tr);
        int got_c = csv_row(cf, rows[i], cr);
        if (!got_t || !got_c) { ok = 0; continue; }
        for (int k = 1; k < 9; k++)
            if (tr[k] != cr[k]) ok = 0;
    }
    check("print_equals_export", ok);
    fclose(tf); fclose(cf);
}

static void test_csv_json_shapes(void) {
    FtTrace t; ft_init(&t);
    ft_stage(&t, "physics");
    uint64_t vals[1] = { 4096 };
    for (int i = 0; i < 10; i++) ft_inject_frame(&t, vals, 1);
    FtReport rep = ft_report_build(&t);

    FILE* hf = tmpfile();
    ft_hist_write_csv(&rep, hf);
    rewind(hf);
    char hdr[128]; int hdr_ok = fgets(hdr, sizeof(hdr), hf) != NULL
                              && strcmp(hdr, "name,bucket,low_ns,high_ns,count\n") == 0;
    int bucket_found = 0;
    char line[256];
    while (fgets(line, sizeof(line), hf)) {
        char name[40]; int b; uint64_t low, high, cnt;
        char* p = line;
        char* c1 = strchr(p, ','); if (!c1) continue;
        size_t nl = (size_t)(c1 - p);
        if (nl >= sizeof(name)) continue;
        memcpy(name, p, nl); name[nl] = '\0';
        p = c1 + 1;
        b = (int)strtol(p, &p, 10); if (*p++ != ',') continue;
        low = strtoull(p, &p, 10); if (*p++ != ',') continue;
        high = strtoull(p, &p, 10); if (*p++ != ',') continue;
        cnt = strtoull(p, &p, 10);
        if (strcmp(name, "physics") == 0 && b == 13 && low == 4096
            && high == 8192 && cnt == 10) bucket_found = 1;
    }
    check("hist_csv_shape", hdr_ok && bucket_found);
    fclose(hf);

    FILE* lf = tmpfile();
    ft_timeline_write_csv(&t, lf);
    rewind(lf);
    int stage_lines = 0, total_lines = 0;
    if (!fgets(line, sizeof(line), lf)) { fclose(lf); check("timeline_csv_shape", 0); return; }
    while (fgets(line, sizeof(line), lf)) {
        /* frame_index,kind,name,duration_ns */
        char* p = line;
        uint64_t idx = strtoull(p, &p, 10); if (*p++ != ',') continue;
        char* kind = p;
        char* c2 = strchr(p, ','); if (!c2) continue; *c2 = '\0'; p = c2 + 1;
        char* name = p;
        char* c3 = strchr(p, ','); if (!c3) continue; *c3 = '\0'; p = c3 + 1;
        uint64_t dur = strtoull(p, &p, 10);
        (void)idx;
        if (strcmp(kind, "stage") == 0 && strcmp(name, "physics") == 0 && dur == 4096)
            stage_lines++;
        if (strcmp(kind, "total") == 0) total_lines++;
    }
    check("timeline_csv_shape", stage_lines == 10 && total_lines == 10);
    fclose(lf);

    FILE* jf = tmpfile();
    ft_trace_write_json(&t, jf);
    rewind(jf);
    int events = 0, has_x = 0, has_i = 0;
    while (fgets(line, sizeof(line), jf)) {
        if (strstr(line, "\"ph\":\"X\"")) { events++; has_x = 1; }
        if (strstr(line, "\"ph\":\"I\"")) { events++; has_i = 1; }
    }
    check("trace_json_shape", events == 20 && has_x && has_i);
    fclose(jf);
}

static void test_live_path(void) {
    FtTrace t; ft_init(&t);
    int a = ft_stage(&t, "a");
    int b = ft_stage(&t, "b");
    for (int i = 0; i < 200; i++) {
        ft_frame_begin(&t);
        ft_begin(&t, a);
        struct timespec sp = { 0, 200000 };  /* 0.2 ms */
        nanosleep(&sp, NULL);
        ft_end(&t, a);
        ft_begin(&t, b);                     /* left open: auto-closed at frame end */
        ft_frame_end(&t);
    }
    FtReport rep = ft_report_build(&t);
    check("live_captured", rep.stage[0].n == 200 && rep.stage[1].n == 200
                        && rep.stage[0].min_ns >= 150000
                        && rep.frame_total.n == 200
                        && rep.frame_total.min_ns >= rep.stage[0].min_ns);
    check("live_end_without_begin_ignored", (ft_end(&t, 99), 1));
}

/* ---- overhead bench -------------------------------------------------------
   The budget is "under 2% of total frame time", so it is measured against a
   realistic ~50 us frame (a 60 fps frame slice), not a toy loop. Rather than
   timing the two arms in long separate windows -- where one scheduler hit
   pollutes a whole window -- each measurement is a back-to-back pair of an
   off frame and an on frame. Both share the noise of that moment, so their
   difference is almost pure tracer cost. Pair order alternates to cancel
   systematic bias; the median of the per-pair deltas is the estimator. */

static volatile uint64_t g_sink;

static void workload_once(uint64_t iters) {
    volatile uint64_t acc = 0;
    for (uint64_t i = 1; i <= iters; i++) acc = acc + i * 2654435761ull;
    g_sink = acc;
}

static void frame_body(FtTrace* t, int sa, int sb, uint64_t iters, int capture) {
    if (capture) ft_frame_begin(t);
    if (capture) ft_begin(t, sa);
    workload_once(iters / 2);
    if (capture) ft_end(t, sa);
    if (capture) ft_begin(t, sb);
    workload_once(iters - iters / 2);
    if (capture) ft_end(t, sb);
    if (capture) ft_frame_end(t);
}

static void test_overhead(void) {
    enum { PAIRS = 4096 };
    const uint64_t iters = 300000;           /* ~50 us of work per frame */
    uint64_t deltas[PAIRS];
    uint64_t off_sum = 0;
    FtTrace t; ft_init(&t);
    int sa = ft_stage(&t, "work_a");
    int sb = ft_stage(&t, "work_b");

    for (int k = 0; k < PAIRS; k++) {
        uint64_t t0, t1, t2, off_ns, on_ns;
        if (k & 1) {                          /* alternate which arm leads */
            t0 = ft_now_ns();
            frame_body(&t, sa, sb, iters, 0);
            t1 = ft_now_ns();
            frame_body(&t, sa, sb, iters, 1);
            t2 = ft_now_ns();
            off_ns = t1 - t0; on_ns = t2 - t1;
        } else {
            t0 = ft_now_ns();
            frame_body(&t, sa, sb, iters, 1);
            t1 = ft_now_ns();
            frame_body(&t, sa, sb, iters, 0);
            t2 = ft_now_ns();
            on_ns = t1 - t0; off_ns = t2 - t1;
        }
        off_sum += off_ns;
        deltas[k] = on_ns > off_ns ? on_ns - off_ns : 0;
    }

    qsort(deltas, PAIRS, sizeof(uint64_t), ft_cmp_u64);
    uint64_t med_delta = deltas[PAIRS / 2];
    uint64_t per_frame_off = off_sum / PAIRS;
    double pct = 100.0 * (double)med_delta / (double)per_frame_off;

    /* raw clock-read cost, for the report */
    uint64_t c0 = ft_now_ns();
    enum { CLOCK_N = 10000 };
    for (int i = 0; i < CLOCK_N; i++) (void)ft_now_ns();
    uint64_t clock_ns = (ft_now_ns() - c0) / CLOCK_N;

    printf("bench_pairs=%d iters_per_frame=%" PRIu64 "\n", PAIRS, iters);
    printf("bench_frame_work_ns=%" PRIu64 "\n", per_frame_off);
    printf("bench_tracer_ns_per_frame=%" PRIu64 "\n", med_delta);
    printf("bench_overhead_pct=%.4f\n", pct);
    printf("clock_read_ns=%" PRIu64 "\n", clock_ns);
    check("overhead_under_2pct", pct < 2.0);
}


int main(void) {
    test_deterministic();
    test_ring_wrap();
    test_print_equals_export();
    test_csv_json_shapes();
    test_live_path();
    test_overhead();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
