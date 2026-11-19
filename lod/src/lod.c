/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold.

   Animation is decoupled from the render frame rate: the canonical playback
   phase is always integrated with the real frame delta, while the pose is
   sampled at a per-LOD cadence (1x/2x/4x/8x frame stride, i.e. 60/30/15/7.5
   Hz at 60 fps).  A level switch only changes the sampling cadence; it never
   touches the phase, so playback stays continuous across LOD changes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define LEVELS 4

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

static float screen_error(float geom_error, float distance, float fov_y, int screen_h) {
    if (distance <= 0.0f) return 1e9f;
    return geom_error * (float)screen_h / (2.0f * distance * tanf(fov_y * 0.5f));
}

/* pick coarsest level whose projected error is below tolerance */
static int select_level(const MeshLod* m, float distance, float fov_y, int screen_h,
                        float tolerance_px, State* st) {
    int want = 0;
    for (int i = LEVELS - 1; i >= 0; i--) {
        float e = screen_error(m->error[i], distance, fov_y, screen_h);
        if (e <= tolerance_px) { want = i; break; }
    }
    int prev = st->level;
    if (want != prev) {
        /* hysteresis: require the candidate to beat the threshold by tau */
        float e_new = screen_error(m->error[want], distance, fov_y, screen_h);
        float e_old = screen_error(m->error[prev], distance, fov_y, screen_h);
        if (want > prev && e_new > tolerance_px * m->tau) return prev;
        if (want < prev && e_old < tolerance_px / m->tau) return prev;
        st->switches++;
        st->level = want;
    }
    return st->level;
}

/* ---- animation LOD ---------------------------------------------------- */

#define ANIM_FPS 60.0f
#define ANIM_DT (1.0f / ANIM_FPS)
#define POSE_RADIUS 100.0f

/* pose sampling stride in render frames, indexed by LOD level */
static const int anim_stride[LEVELS] = {1, 2, 4, 8};

typedef struct {
    float duration;       /* clip length, seconds */
    float speed;          /* playback rate, loops per second */
    float phase;          /* canonical phase, seconds: always integrated */
    float sampled_phase;  /* phase of the pose currently on screen */
    float accum;          /* time since the last pose sample */
    int level;
    long samples;
    long samples_at[LEVELS];
    int switches;
} AnimState;

static void anim_init(AnimState* a, float duration, float speed, int level) {
    memset(a, 0, sizeof(*a));
    a->duration = duration;
    a->speed = speed;
    a->level = level;
}

/* Change cadence only: phase, sampled pose and accumulator are untouched so
   the switch cannot introduce a phase discontinuity. */
static void anim_set_level(AnimState* a, int level) {
    if (level < 0) level = 0;
    if (level >= LEVELS) level = LEVELS - 1;
    if (level == a->level) return;
    a->level = level;
    a->switches++;
}

static float wrap_phase(float phase, float duration) {
    if (duration <= 0.0f) return phase;
    while (phase >= duration) phase -= duration;
    while (phase < 0.0f) phase += duration;
    return phase;
}

static void anim_update(AnimState* a, float dt) {
    /* canonical time always advances at full temporal resolution */
    a->phase = wrap_phase(a->phase + dt * a->speed, a->duration);
    /* only the sampled pose follows the LOD cadence */
    a->accum += dt;
    float interval = (float)anim_stride[a->level] * ANIM_DT;
    if (a->accum + 1e-5f >= interval) {
        a->sampled_phase = a->phase;
        a->accum -= interval;                 /* keep remainder: no drift on jittery dt */
        if (a->accum >= interval) a->accum = 0.0f;   /* frame slower than cadence: no catch-up */
        a->samples++;
        a->samples_at[a->level]++;
    }
}

/* a simple 2D rig: one joint angle plus the endpoint it drives */
typedef struct { float angle_deg; float x, y; } Pose;

static Pose anim_pose(float phase, float duration) {
    float u = duration > 0.0f ? phase / duration : 0.0f;
    float theta = 6.28318530718f * u;
    Pose p;
    p.angle_deg = 360.0f * u;
    p.x = POSE_RADIUS * sinf(theta);
    p.y = POSE_RADIUS * (1.0f - cosf(theta));
    return p;
}

static float angle_delta_deg(float a, float b) {
    float d = fmodf(a - b, 360.0f);
    if (d > 180.0f) d -= 360.0f;
    if (d < -180.0f) d += 360.0f;
    return d;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    float fov = 1.0f; int h = 1080;
    const float tolerance_px = 50.0f;

    /* --- geometric LOD selection (unchanged behaviour) --- */
    State st = {0, 0, 0};
    int hist[LEVELS] = {0};
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int lv = select_level(&m, d, fov, h, tolerance_px, &st);
        hist[lv]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    /* 迟滞应当让切换次数远小于采样次数 */
    check("hysteresis_ok", st.switches <= 8);

    /* --- per-level cadence: farther LOD = lower animation update rate --- */
    {
        const int frames = 600;
        for (int lv = 0; lv < LEVELS; lv++) {
            AnimState a;
            anim_init(&a, 1.0f, 1.0f, lv);
            float max_lag = 0.0f;
            for (int f = 0; f < frames; f++) {
                anim_update(&a, ANIM_DT);
                float lag = wrap_phase(a.phase - a.sampled_phase, a.duration);
                if (lag > max_lag) max_lag = lag;
            }
            int expect = frames / anim_stride[lv];
            printf("anim_lod%d stride=%d hz=%.2f samples=%ld expect=%d max_lag_ms=%.2f\n",
                   lv, anim_stride[lv], ANIM_FPS / (float)anim_stride[lv],
                   a.samples, expect, max_lag * 1000.0f);
            char name[32];
            snprintf(name, sizeof name, "anim_lod%d_rate_ok", lv);
            check(name, labs(a.samples - expect) <= 1);
            snprintf(name, sizeof name, "anim_lod%d_lag_ok", lv);
            float bound = (float)(anim_stride[lv] - 1) * ANIM_DT + 1e-4f;
            check(name, max_lag <= bound);
        }
    }

    /* --- oscillating distance: many level switches, phase must stay put --- */
    {
        const int frames = 1200;
        AnimState anim, ref;
        anim_init(&anim, 1.0f, 1.0f, 0);
        anim_init(&ref, 1.0f, 1.0f, 0);      /* full-rate reference */
        State st2 = {0, 0, 0};
        int levels_visited = 0;
        int phase_discontinuities = 0;
        int switch_phase_errors = 0;
        long updates = 0;
        float pose_err_max = 0.0f, pose_err_sum = 0.0f;
        float angle_err_max = 0.0f, lag_max = 0.0f;

        for (int f = 0; f < frames; f++) {
            float t = (float)f / ANIM_FPS;
            /* 10 .. 400 world units, sweeps through all four levels */
            float d = 205.0f + 195.0f * sinf(6.28318530718f * t / 5.0f);
            int lv = select_level(&m, d, fov, h, tolerance_px, &st2);
            levels_visited |= 1 << lv;

            if (lv != anim.level) {
                float phase_before = anim.phase;
                float sampled_before = anim.sampled_phase;
                anim_set_level(&anim, lv);
                if (anim.phase != phase_before || anim.sampled_phase != sampled_before)
                    switch_phase_errors++;
            }

            float phase_before = anim.phase;
            long samples_before = anim.samples;
            anim_update(&anim, ANIM_DT);
            updates += (anim.samples != samples_before);

            /* canonical phase must advance by exactly dt * speed, wrap aside */
            float advanced = wrap_phase(anim.phase - phase_before, anim.duration);
            if (fabsf(advanced - ANIM_DT * anim.speed) > 1e-4f) phase_discontinuities++;

            anim_update(&ref, ANIM_DT);      /* full frame rate, level 0 */

            Pose pr = anim_pose(ref.phase, ref.duration);
            Pose pa = anim_pose(anim.sampled_phase, anim.duration);
            float dx = pr.x - pa.x, dy = pr.y - pa.y;
            float err = sqrtf(dx * dx + dy * dy);
            float aerr = fabsf(angle_delta_deg(pr.angle_deg, pa.angle_deg));
            pose_err_sum += err;
            if (err > pose_err_max) pose_err_max = err;
            if (aerr > angle_err_max) angle_err_max = aerr;

            float lag = wrap_phase(anim.phase - anim.sampled_phase, anim.duration);
            if (lag > lag_max) lag_max = lag;
        }

        float max_lag_frames = (float)(anim_stride[LEVELS - 1] - 1);
        float max_lag_angle = 6.28318530718f * anim.speed * max_lag_frames * ANIM_DT;
        float pose_bound = 2.0f * POSE_RADIUS * sinf(max_lag_angle * 0.5f) + 0.5f;

        printf("anim_switch_frames=%d switches=%d levels_mask=0x%x\n",
               anim.switches, st2.switches, levels_visited);
        printf("anim_updates=%ld/%d update_rate_pct=%.1f\n",
               updates, frames, 100.0f * (float)updates / (float)frames);
        printf("phase_discontinuities=%d switch_phase_errors=%d\n",
               phase_discontinuities, switch_phase_errors);
        printf("pose_err_max=%.3f pose_err_mean=%.3f pose_angle_err_max_deg=%.3f lag_max_ms=%.2f\n",
               pose_err_max, pose_err_sum / (float)frames, angle_err_max, lag_max * 1000.0f);

        check("anim_switches_ok", anim.switches >= 8);
        check("anim_levels_visited_ok", levels_visited == (1 << LEVELS) - 1);
        check("phase_continuity_ok", phase_discontinuities == 0 && switch_phase_errors == 0);
        check("pose_error_bounded", pose_err_max <= pose_bound);
        check("anim_update_rate_reduced", updates < frames / 2);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
