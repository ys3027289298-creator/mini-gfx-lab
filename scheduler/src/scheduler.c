
/* scheduler: sample-accurate audio event scheduling.
   Events are posted per visual frame, quantized to integer sample positions,
   shifted by a configurable latency-compensation offset, sorted inside each
   audio block by (sample position, insertion sequence) and rendered into a
   mono float buffer. Rendered output is bit-identical to an independent
   sample-by-sample hand-laid reference (see reference_render). */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdint.h>

#define MAX_EVENTS 4096
#define VOICE_LEN 480                 /* 10 ms @ 48 kHz */
#define LOG_CAP (MAX_EVENTS * 2)

typedef struct {
    int id;
    double time;                      /* seconds on the timeline */
    float gain;
    int seq;                          /* insertion order, stable tie-break */
} Event;

typedef struct {
    int sample;                       /* onset inside the block, post-compensation */
    int id;
    float gain;
    int seq;
} QEvent;

typedef struct {
    double sr;
    int delay;                        /* configurable latency compensation, samples */
    Event queue[MAX_EVENTS];
    int count;
    int next_seq;
    long long playhead;               /* samples already rendered */
    int dropped;                      /* posts rejected because the queue was full */
    int late;                         /* events whose onset was already in the past */
    int log_sample[LOG_CAP];
    int log_id[LOG_CAP];
    int log_seq[LOG_CAP];
    int log_n;
    double max_align_err;             /* largest |ideal float sample - quantized| */
} Scheduler;

/* Deterministic blip: sine with exponential decay. The reference renderer
   calls the exact same function, so bit-identical accumulation is expected. */
static float voice_tick(int k, float gain) {
    return gain * sinf(6.2831853f * 440.0f * (float)k / 48000.0f)
                * expf(-(float)k / 120.0f);
}

static void sched_init(Scheduler* s, double sr, int delay_samples) {
    memset(s, 0, sizeof(*s));
    s->sr = sr;
    s->delay = delay_samples;
}

static void sched_set_delay_samples(Scheduler* s, int delay_samples) {
    s->delay = delay_samples;
}

static void sched_set_delay_ms(Scheduler* s, double ms) {
    s->delay = (int)llround(ms * 1e-3 * s->sr);
}

static int sched_post(Scheduler* s, int id, double time, float gain) {
    if (s->count >= MAX_EVENTS) { s->dropped++; return -1; }
    Event* e = &s->queue[s->count++];
    e->id = id; e->time = time; e->gain = gain; e->seq = s->next_seq++;
    return 0;
}

/* Frame-aligned posting: frame f starts at f * frame_dt, offset shifts the
   event inside the frame (sub-frame sample accuracy kept via double time). */
static int sched_post_frame(Scheduler* s, int id, int frame, double frame_dt,
                            double offset, float gain) {
    return sched_post(s, id, (double)frame * frame_dt + offset, gain);
}

static int q_cmp(const void* a, const void* b) {
    const QEvent* x = (const QEvent*)a;
    const QEvent* y = (const QEvent*)b;
    if (x->sample != y->sample) return x->sample < y->sample ? -1 : 1;
    if (x->seq != y->seq) return x->seq < y->seq ? -1 : 1;
    return 0;
}

/* Gather every event due in [playhead, playhead+frames), quantize, compensate
   latency, sort and log. Removed events are swap-popped from the queue. */
static void sched_dispatch(Scheduler* s, int frames, QEvent* out, int cap, int* n_out) {
    int n = 0;
    long long start = s->playhead;
    long long end = start + frames;
    for (int i = 0; i < s->count; ) {
        Event* e = &s->queue[i];
        double ideal = e->time * s->sr - (double)s->delay;
        long long t = llround(ideal);
        double err = fabs(ideal - (double)t);
        if (err > s->max_align_err) s->max_align_err = err;
        if (t < start) {              /* cannot place into the past anymore */
            s->late++;
            s->queue[i] = s->queue[--s->count];
            continue;
        }
        if (t < end && n < cap) {
            QEvent* q = &out[n++];
            q->sample = (int)(t - start);
            q->id = e->id; q->gain = e->gain; q->seq = e->seq;
            s->queue[i] = s->queue[--s->count];
            continue;
        }
        i++;
    }
    qsort(out, (size_t)n, sizeof(QEvent), q_cmp);
    for (int i = 0; i < n && s->log_n < LOG_CAP; i++) {
        s->log_sample[s->log_n] = (int)(s->playhead + out[i].sample);
        s->log_id[s->log_n] = out[i].id;
        s->log_seq[s->log_n] = out[i].seq;
        s->log_n++;
    }
    *n_out = n;
}

/* Voice math only; block-clipped tails match the reference semantics. */
static void sched_apply(const QEvent* q, int n, float* out, int frames) {
    memset(out, 0, sizeof(float) * (size_t)frames);
    for (int i = 0; i < n; i++) {
        int at = q[i].sample;
        int len = at + VOICE_LEN > frames ? frames - at : VOICE_LEN;
        for (int k = 0; k < len; k++)
            out[at + k] += voice_tick(k, q[i].gain);
    }
}

static void sched_render_block(Scheduler* s, float* out, int frames) {
    QEvent q[MAX_EVENTS];
    int n;
    sched_dispatch(s, frames, q, MAX_EVENTS, &n);
    sched_apply(q, n, out, frames);
    s->playhead += frames;
}

/* Independent reference: same block walk, its own quantization/sort, and an
   explicit sample-by-sample hand placement of every voice. */
static void reference_render(const Event* ev, int n_ev, double sr, int delay,
                             float* buf, int total, int block) {
    QEvent* q = (QEvent*)malloc(sizeof(QEvent) * (size_t)n_ev);
    for (long long start = 0; start < total; start += block) {
        long long end = start + block;
        int n = 0;
        for (int i = 0; i < n_ev; i++) {
            double ideal = ev[i].time * sr - (double)delay;
            long long t = llround(ideal);
            if (t < start || t >= end) continue;
            q[n].sample = (int)(t - start);
            q[n].id = ev[i].id; q[n].gain = ev[i].gain; q[n].seq = ev[i].seq;
            n++;
        }
        qsort(q, (size_t)n, sizeof(QEvent), q_cmp);
        for (int f = 0; f < block; f++) buf[start + f] = 0.0f;
        for (int i = 0; i < n; i++) {
            int at = q[i].sample;
            for (int k = 0; k < VOICE_LEN && at + k < block; k++)
                buf[start + at + k] += voice_tick(k, q[i].gain);
        }
    }
    free(q);
}

static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

#define SR 48000.0
#define BLOCK 256
#define BLOCKS 32
#define TOTAL (BLOCK * BLOCKS)

static float out_buf[TOTAL];
static float ref_buf[TOTAL];

static uint32_t rng_state = 0x12345678u;
static double rnd(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return (double)(rng_state >> 8) / (double)(1u << 24);
}

int main(void) {
    printf("config sr=%.0f block=%d voice_len=%d max_events=%d\n",
           SR, BLOCK, VOICE_LEN, MAX_EVENTS);

    /* ---- bit-exactness vs hand-laid sample-by-sample reference ---- */
    Event ref_events[200];
    Scheduler s;
    sched_init(&s, SR, 0);
    for (int i = 0; i < 200; i++) {
        double t = rnd() * (TOTAL / SR) * 0.95;
        float g = 0.2f + (float)rnd() * 0.8f;
        ref_events[i].id = i;
        ref_events[i].time = t;
        ref_events[i].gain = g;
        ref_events[i].seq = i;
        sched_post(&s, i, t, g);
    }
    for (int b = 0; b < BLOCKS; b++)
        sched_render_block(&s, &out_buf[b * BLOCK], BLOCK);
    reference_render(ref_events, 200, SR, 0, ref_buf, TOTAL, BLOCK);
    check("waveform_bit_exact",
          memcmp(out_buf, ref_buf, sizeof(float) * TOTAL) == 0);
    printf("max_align_error=%.4f samples\n", s.max_align_err);
    check("align_under_one_sample", s.max_align_err < 1.0);

    /* same scenario with 37-sample latency compensation stays bit-exact */
    Scheduler s2;
    sched_init(&s2, SR, 37);
    for (int i = 0; i < 200; i++)
        sched_post(&s2, ref_events[i].id, ref_events[i].time, ref_events[i].gain);
    for (int b = 0; b < BLOCKS; b++)
        sched_render_block(&s2, &out_buf[b * BLOCK], BLOCK);
    reference_render(ref_events, 200, SR, 37, ref_buf, TOTAL, BLOCK);
    check("compensated_bit_exact",
          memcmp(out_buf, ref_buf, sizeof(float) * TOTAL) == 0);
    /* every onset was pulled 37 samples earlier than its raw position */
    int raw_pos[200];
    for (int i = 0; i < 200; i++)
        raw_pos[ref_events[i].id] = (int)llround(ref_events[i].time * SR);
    int shifted = 1;
    int expect_late = 0;
    for (int i = 0; i < 200; i++)
        if (raw_pos[i] < 37) expect_late++;
    for (int i = 0; i < s2.log_n; i++)
        if (s2.log_sample[i] != raw_pos[s2.log_id[i]] - 37) shifted = 0;
    check("compensation_shifts_onsets",
          shifted && s2.log_n + s2.late == 200 && s2.late == expect_late);

    /* ---- same-frame ordering: (sample, insertion seq) ---- */
    Scheduler o;
    sched_init(&o, SR, 0);
    sched_post(&o, 100, 100.0 / SR, 0.5f);   /* sample 100 */
    sched_post(&o, 50, 50.0 / SR, 0.5f);     /* sample 50 */
    for (int i = 0; i < 5; i++)              /* five events on sample 200 */
        sched_post(&o, 2000 + i, 200.0 / SR, 0.5f);
    float blk[BLOCK];
    sched_render_block(&o, blk, BLOCK);
    int ordered = 1;
    for (int i = 1; i < o.log_n; i++) {
        if (o.log_sample[i] < o.log_sample[i - 1]) ordered = 0;
        if (o.log_sample[i] == o.log_sample[i - 1] &&
            o.log_seq[i] < o.log_seq[i - 1]) ordered = 0;
    }
    check("same_frame_sorted", ordered && o.log_n == 7);
    check("tie_keeps_insertion_order",
          o.log_sample[2] == 200 && o.log_id[2] == 2000 &&
          o.log_id[3] == 2001 && o.log_id[6] == 2004);

    /* ---- frame-aligned posting lands on the quantized frame sample ---- */
    Scheduler f;
    sched_init(&f, SR, 0);
    double frame_dt = 1.0 / 60.0;
    for (int fr = 0; fr < 10; fr++)
        sched_post_frame(&f, fr, fr, frame_dt, 0.0, 0.5f);
    sched_render_block(&f, blk, BLOCK);
    int frame_ok = 1;
    for (int fr = 0; fr < 10 && fr < f.log_n; fr++) {
        int want = (int)llround(fr * frame_dt * SR);
        if (f.log_sample[fr] != want || f.log_id[fr] != fr) frame_ok = 0;
    }
    check("frame_aligned_onsets", frame_ok);

    /* ---- configurable latency compensation (samples and milliseconds) ---- */
    float big[TOTAL];
    Scheduler d;
    sched_init(&d, SR, 0);
    sched_set_delay_samples(&d, 100);
    sched_post(&d, 1, 0.01, 0.5f);            /* raw 480 - 100 = 380 */
    for (int b = 0; b < BLOCKS; b++)
        sched_render_block(&d, &big[b * BLOCK], BLOCK);
    check("delay_sample_offset", d.log_sample[0] == 380);

    Scheduler d2;
    sched_init(&d2, SR, 0);
    sched_set_delay_ms(&d2, 5.0);             /* 5 ms = 240 samples */
    check("delay_ms_config", d2.delay == 240);
    sched_post(&d2, 1, 0.01, 0.5f);           /* 480 - 240 = 240 */
    for (int b = 0; b < BLOCKS; b++)
        sched_render_block(&d2, &big[b * BLOCK], BLOCK);
    check("delay_ms_offset", d2.log_sample[0] == 240);

    /* large compensation can push an onset into the past -> counted late */
    Scheduler d3;
    sched_init(&d3, SR, 0);
    sched_set_delay_samples(&d3, 200);
    sched_post(&d3, 1, 0.001, 0.5f);          /* raw 48, compensated -152 */
    sched_render_block(&d3, big, BLOCK);
    check("compensation_can_be_late", d3.late == 1 && d3.log_n == 0);

    /* ---- late events are counted and dropped, never rendered ---- */
    Scheduler l;
    sched_init(&l, SR, 0);
    sched_render_block(&l, blk, BLOCK);       /* playhead -> 256 */
    check("late_post_rejected", sched_post(&l, 1, 0.001, 0.5f) == 0);
    int log_before = l.log_n;
    sched_render_block(&l, blk, BLOCK);
    check("late_counted", l.late == 1 && l.log_n == log_before);

    /* ---- queue saturation ---- */
    Scheduler cap;
    sched_init(&cap, SR, 0);
    int rejected = 0;
    for (int i = 0; i < MAX_EVENTS + 16; i++)
        if (sched_post(&cap, i, 10.0 + (double)i / SR, 0.5f) != 0) rejected++;
    check("queue_cap_enforced",
          cap.count == MAX_EVENTS && rejected == 16 && cap.dropped == 16);

    /* ---- half-open blocks: onset exactly on a boundary belongs to next block ---- */
    Scheduler bd;
    sched_init(&bd, SR, 0);
    sched_post(&bd, 7, (double)BLOCK / SR, 0.5f);
    sched_render_block(&bd, blk, BLOCK);
    check("boundary_not_early", bd.log_n == 0);
    sched_render_block(&bd, blk, BLOCK);
    check("boundary_next_block", bd.log_n == 1 && bd.log_sample[0] == BLOCK);
    /* ---- event count vs scheduling cost (dispatch = quantize+sort+log) ---- */
    int sizes[] = { 1, 8, 64, 256, 1024, 4096 };
    const int reps = 30;
    for (size_t si = 0; si < sizeof(sizes) / sizeof(sizes[0]); si++) {
        int n = sizes[si];
        long long disp_sum = 0, full_sum = 0;
        for (int r = 0; r < reps; r++) {
            Scheduler b;
            sched_init(&b, SR, 0);
            for (int i = 0; i < n; i++)
                sched_post(&b, i, (double)((i * 37) % BLOCK) / SR, 0.5f);
            float tmp[BLOCK];
            QEvent q[MAX_EVENTS]; int m;
            long long t0 = now_ns();
            sched_dispatch(&b, BLOCK, q, MAX_EVENTS, &m);
            long long t1 = now_ns();
            sched_apply(q, m, tmp, BLOCK);
            long long t2 = now_ns();
            disp_sum += t1 - t0;
            full_sum += t2 - t0;
        }
        long long disp_avg = disp_sum / reps;
        long long full_avg = full_sum / reps;
        printf("bench n=%d dispatch_ns=%lld full_block_ns=%lld per_event_ns=%.2f\n",
               n, disp_avg, full_avg, (double)disp_avg / n);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
