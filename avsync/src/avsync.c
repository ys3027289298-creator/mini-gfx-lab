/* avsync: present one visual action per gameplay event on the frame grid
   and measure the audio/video deviation frame by frame. Frame k reaches
   the screen at its present time (k+0.5)/fps; the audio event timeline
   (schema avsync-events/1, written by mini-audio-lab's avsync module) is
   compared against those present times. Writes the frame timestamps
   (frames.csv) and the per-frame deviation (deviation.csv) and fails if
   any deviation exceeds one frame. --align must match the mode recorded
   in the timeline; --align=off with a legacy timeline reproduces the
   current numbers. */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define SCHEMA "avsync-events/1"
#define COLUMNS "event,target_s,scheduled_s,dev_s,dev_frames"

typedef struct {
    int sr, buffer, fps, align; /* parsed from the csv header */
} Hdr;

typedef struct {
    int event;
    double target_s, sched_s, dev_s, dev_frames;
} Ev;

/* 0 ok, -1 bad schema, -2 bad header, -3 io/parse */
static int load_events(FILE* f, Hdr* h, Ev** out, int* nout) {
    char line[512];
    if (!fgets(line, sizeof line, f)) return -1;
    line[strcspn(line, "\r\n")] = 0;
    char want[64];
    snprintf(want, sizeof want, "# %s", SCHEMA);
    if (strcmp(line, want)) return -1;
    if (!fgets(line, sizeof line, f)) return -2;
    char am[8] = { 0 };
    if (sscanf(line, "# sr=%d buffer=%d fps=%d align=%7s",
               &h->sr, &h->buffer, &h->fps, am) != 4)
        return -2;
    if (!strcmp(am, "on")) h->align = 1;
    else if (!strcmp(am, "off")) h->align = 0;
    else return -2;
    if (!fgets(line, sizeof line, f)) return -2;
    if (strncmp(line, COLUMNS, sizeof(COLUMNS) - 1)) return -2;

    int cap = 128, n = 0;
    Ev* ev = malloc(sizeof(Ev) * (size_t)cap);
    if (!ev) return -3;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        if (n == cap) {
            cap *= 2;
            Ev* p = realloc(ev, sizeof(Ev) * (size_t)cap);
            if (!p) { free(ev); return -3; }
            ev = p;
        }
        Ev* e = &ev[n];
        if (sscanf(line, "%d,%lf,%lf,%lf,%lf",
                   &e->event, &e->target_s, &e->sched_s, &e->dev_s, &e->dev_frames) != 5) {
            free(ev);
            return -3;
        }
        n++;
    }
    *out = ev;
    *nout = n;
    return 0;
}

static int load_path(const char* path, Hdr* h, Ev** ev, int* n) {
    FILE* f = fopen(path, "r");
    if (!f) return -3;
    int rc = load_events(f, h, ev, n);
    fclose(f);
    return rc;
}

/* frame whose present time is t: present(k) = (k+0.5)/fps */
static int frame_of(double t, int fps) {
    return (int)floor(t * fps + 1e-9);
}

/* 0 ok, 1 budget exceeded, -3 config mismatch, -4 io */
static int report(const Hdr* h, const Ev* ev, int n, int align, int fps,
                  const char* frames_out, const char* dev_out) {
    if (align != h->align) {
        fprintf(stderr, "align mismatch: timeline has align=%s; "
                "rerun both sides with the same --align\n",
                h->align ? "on" : "off");
        return -3;
    }
    if (fps != h->fps) {
        fprintf(stderr, "fps mismatch: timeline has %d, got --fps=%d\n", h->fps, fps);
        return -3;
    }
    if (n <= 0) {
        fprintf(stderr, "no events\n");
        return -3;
    }

    int nframes = 0;
    for (int i = 0; i < n; i++) {
        int fr = frame_of(ev[i].target_s, fps);
        if (fr + 1 > nframes) nframes = fr + 1;
    }

    FILE* ff = fopen(frames_out, "w");
    if (!ff) return -4;
    fprintf(ff, "frame,present_s\n");
    for (int k = 0; k < nframes; k++)
        fprintf(ff, "%d,%.9f\n", k, (k + 0.5) / fps);
    fclose(ff);

    FILE* df = fopen(dev_out, "w");
    if (!df) return -4;
    fprintf(df, "frame,event,dev_s,dev_frames\n");
    double mn = 1e300, mx = -1e300, mean = 0.0, maxf = 0.0;
    for (int i = 0; i < n; i++) {
        int fr = frame_of(ev[i].target_s, fps);
        double d = ev[i].sched_s - (fr + 0.5) / fps;
        fprintf(df, "%d,%d,%.9f,%.6f\n", fr, ev[i].event, d, d * fps);
        if (d < mn) mn = d;
        if (d > mx) mx = d;
        mean += d;
        double a = fabs(d) * fps;
        if (a > maxf) maxf = a;
    }
    fclose(df);
    mean /= n;

    printf("frames=%d events=%d fps=%d align=%s\n",
           nframes, n, fps, h->align ? "on" : "off");
    printf("dev_min_ms=%.3f dev_mean_ms=%.3f dev_max_ms=%.3f\n",
           mn * 1e3, mean * 1e3, mx * 1e3);
    printf("dev_maxabs_frames=%.6f\n", maxf);
    printf("wrote=%s,%s\n", frames_out, dev_out);
    int ok = maxf <= 1.0 + 1e-9;
    printf("within_1_frame=%d\n", ok);
    return ok ? 0 : 1;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int count_rows(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return -1;
    int n = 0;
    char line[512];
    while (fgets(line, sizeof line, f))
        if (line[0] >= '0' && line[0] <= '9') n++;
    fclose(f);
    return n;
}

static double maxabs_frames(const Ev* ev, int n, int fps) {
    double m = 0.0;
    for (int i = 0; i < n; i++) {
        double d = ev[i].sched_s - (frame_of(ev[i].target_s, fps) + 0.5) / fps;
        double a = fabs(d) * fps;
        if (a > m) m = a;
    }
    return m;
}

static int selftest(void) {
    Hdr ha, hl;
    Ev *ea = NULL, *el = NULL;
    int na = 0, nl = 0;

    int ra = load_path("test/events_aligned.csv", &ha, &ea, &na);
    int rl = load_path("test/events_legacy.csv", &hl, &el, &nl);
    check("load_aligned", ra == 0 && na == 120);
    check("load_legacy", rl == 0 && nl == 120);
    if (ra != 0 || rl != 0) {
        free(ea);
        free(el);
        printf("all_ok=%d\n", 0);
        return 1;
    }

    check("hdr_aligned", ha.align == 1 && ha.fps == 60 && ha.sr == 48000 && ha.buffer == 256);
    check("aligned_within_1_frame", maxabs_frames(ea, na, 60) <= 1.0);
    check("aligned_near_zero", maxabs_frames(ea, na, 60) < 0.01);
    check("aligned_report_ok",
          report(&ha, ea, na, 1, 60, "avsync_selftest_frames.csv",
                 "avsync_selftest_dev.csv") == 0);
    check("frames_rows", count_rows("avsync_selftest_frames.csv") == 120);
    check("dev_rows", count_rows("avsync_selftest_dev.csv") == 120);

    check("hdr_legacy", hl.align == 0);
    /* legacy timeline must reproduce the current half-frame offset */
    check("legacy_le_half_frame", maxabs_frames(el, nl, 60) <= 0.5 + 1e-6);
    check("legacy_within_1_frame", maxabs_frames(el, nl, 60) < 1.0);
    check("legacy_report_ok",
          report(&hl, el, nl, 0, 60, "avsync_selftest_frames.csv",
                 "avsync_selftest_dev.csv") == 0);

    /* mismatched modes are a config error, not a silent pass */
    check("align_mismatch_rejected",
          report(&ha, ea, na, 0, 60, "avsync_selftest_frames.csv",
                 "avsync_selftest_dev.csv") == -3);
    check("fps_mismatch_rejected",
          report(&ha, ea, na, 1, 30, "avsync_selftest_frames.csv",
                 "avsync_selftest_dev.csv") == -3);

    /* unknown schema versions are rejected */
    char bad[] = "# avsync-events/2\n# sr=48000 buffer=256 fps=60 align=on\n"
                 "event,target_s,scheduled_s,dev_s,dev_frames\n";
    FILE* bf = fmemopen(bad, sizeof bad - 1, "r");
    check("bad_schema_fixture", bf != NULL);
    if (bf) {
        Hdr bh;
        Ev* bev;
        int bn;
        check("bad_schema_rejected", load_events(bf, &bh, &bev, &bn) == -1);
        fclose(bf);
    }

    free(ea);
    free(el);
    remove("avsync_selftest_frames.csv");
    remove("avsync_selftest_dev.csv");

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}

int main(int argc, char** argv) {
    const char* events = NULL;
    const char* frames_out = "frames.csv";
    const char* dev_out = "deviation.csv";
    int align = 1, fps = 60;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--selftest")) return selftest();
        if (!strncmp(argv[i], "--events=", 9)) { events = argv[i] + 9; continue; }
        if (!strncmp(argv[i], "--frames-out=", 13)) { frames_out = argv[i] + 13; continue; }
        if (!strncmp(argv[i], "--dev-out=", 10)) { dev_out = argv[i] + 10; continue; }
        if (sscanf(argv[i], "--fps=%d", &fps) == 1) continue;
        if (!strncmp(argv[i], "--align=", 8)) {
            const char* v = argv[i] + 8;
            if (!strcmp(v, "on")) align = 1;
            else if (!strcmp(v, "off")) align = 0;
            else { fprintf(stderr, "bad --align value: %s\n", v); return 2; }
            continue;
        }
        fprintf(stderr, "unknown arg: %s\n", argv[i]);
        return 2;
    }
    if (!events) {
        fprintf(stderr, "usage: avsync --events=events.csv [--align=on|off] "
                "[--fps=60] [--frames-out=frames.csv] [--dev-out=deviation.csv]\n");
        return 2;
    }
    Hdr h;
    Ev* ev;
    int n;
    int rc = load_path(events, &h, &ev, &n);
    if (rc == -1) { fprintf(stderr, "%s: unsupported schema (want %s)\n", events, SCHEMA); return 2; }
    if (rc != 0) { fprintf(stderr, "%s: unreadable or malformed timeline\n", events); return 2; }
    rc = report(&h, ev, n, align, fps, frames_out, dev_out);
    free(ev);
    return rc < 0 ? 2 : rc;
}
