
/* camera: 2D follow camera.
 * - critically-damped spring follow with velocity lookahead (prediction)
 * - world bounds clamp + swept wall collision (axis-separated, substepped)
 * - stackable shake with total amplitude cap
 * - parameter switching is state-preserving: no position/velocity jump
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_SHAKES 8
#define MAX_SUBSTEPS 64

typedef int (*SolidFn)(float x, float y, void* user);

typedef struct {
    float smooth_time;   /* seconds to converge; smaller = snappier */
    float lead_time;     /* seconds of target velocity to look ahead */
    float max_lead;      /* clamp on lookahead distance */
    float max_shake;     /* cap on total shake offset magnitude */
} CamParams;

typedef struct {
    float amp, freq, decay, duration, t;
    float seed_x, seed_y;
} Shake;

typedef struct {
    float x, y;              /* camera center (base, without shake) */
    float ox, oy;            /* offset from target: spring state, target frame */
    float vx, vy;            /* offset-spring velocity state (target frame) */
    float lead_x, lead_y;    /* smoothed predictive lead */
    float px, py;            /* previous target, for velocity estimate */
    int has_target;
    CamParams cur, from, dst;
    float blend_t, blend_d;  /* param transition progress / duration */
    float min_x, min_y, max_x, max_y;
    int clamp_x, clamp_y;
    SolidFn solid;
    void* solid_user;
    float tile;              /* substep size basis for swept collision */
    Shake shakes[MAX_SHAKES];
    int shake_count;
    float shake_x, shake_y;  /* render offset: draw at (x+shake_x, y+shake_y) */
} Camera;

static void cam_init(Camera* c, CamParams p) {
    memset(c, 0, sizeof(*c));
    c->cur = c->from = c->dst = p;
    c->tile = 1.0f;
}

static void cam_set_bounds(Camera* c, float wx, float wy, float ww, float wh,
                           float view_w, float view_h) {
    c->min_x = wx + view_w * 0.5f;
    c->max_x = wx + ww - view_w * 0.5f;
    c->min_y = wy + view_h * 0.5f;
    c->max_y = wy + wh - view_h * 0.5f;
    if (c->max_x < c->min_x) c->min_x = c->max_x = wx + ww * 0.5f;
    if (c->max_y < c->min_y) c->min_y = c->max_y = wy + wh * 0.5f;
    c->clamp_x = c->clamp_y = 1;
}

static void cam_set_world(Camera* c, SolidFn fn, void* user, float tile) {
    c->solid = fn;
    c->solid_user = user;
    c->tile = tile > 0.0f ? tile : 1.0f;
}

/* Instant switch: only parameters change, position/velocity state is kept,
 * so the spring continues from where it was — no hiccup. */
static void cam_set_params(Camera* c, CamParams p) {
    c->cur = c->from = c->dst = p;
    c->blend_t = c->blend_d = 0.0f;
}

/* Blended switch: parameters lerp over blend_d seconds. */
static void cam_tween_params(Camera* c, CamParams p, float blend_d) {
    c->from = c->cur;
    c->dst = p;
    c->blend_d = blend_d;
    c->blend_t = 0.0f;
}

/* re-anchor without a pan: call after teleporting the camera or the target */
static void cam_snap(Camera* c, float tx, float ty) {
    c->px = tx; c->py = ty;
    c->has_target = 1;
    c->ox = c->x - tx;
    c->oy = c->y - ty;
    c->vx = c->vy = 0.0f;
    c->lead_x = c->lead_y = 0.0f;
}

static int cam_add_shake(Camera* c, float amp, float freq, float decay,
                         float duration, float seed) {
    if (c->shake_count >= MAX_SHAKES) return -1;
    if (amp <= 0.0f || duration <= 0.0f) return -1;
    Shake* s = &c->shakes[c->shake_count++];
    s->amp = amp; s->freq = freq; s->decay = decay;
    s->duration = duration; s->t = 0.0f;
    s->seed_x = seed * 1.37f + 0.5f;
    s->seed_y = seed * 2.11f + 3.7f;
    return 0;
}

/* Game Programming Gems 4, ch. 1.10: critically damped spring, dt-robust. */
static float smooth_damp(float from, float to, float* vel, float smooth_time, float dt) {
    float omega = 2.0f / smooth_time;
    float x = omega * dt;
    float exp = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
    float change = from - to;
    float temp = (*vel + omega * change) * dt;
    *vel = (*vel - omega * temp) * exp;
    return to + (change + temp) * exp;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Swept axis-separated move: substeps of <= tile/2 so walls can't be tunneled. */
static void move_with_collision(Camera* c, float nx, float ny) {
    if (!c->solid) { c->x = nx; c->y = ny; return; }
    float dx = nx - c->x, dy = ny - c->y;
    float dist = sqrtf(dx * dx + dy * dy);
    int steps = (int)ceilf(dist / (c->tile * 0.5f));
    if (steps < 1) steps = 1;
    if (steps > MAX_SUBSTEPS) steps = MAX_SUBSTEPS;
    float sx = dx / steps, sy = dy / steps;
    for (int i = 0; i < steps; i++) {
        float ox = c->x, oy = c->y;
        float px = ox + sx, py = oy + sy;
        if (!c->solid(px, py, c->solid_user)) { c->x = px; c->y = py; continue; }
        if (!c->solid(px, oy, c->solid_user)) { c->x = px; c->vy = 0.0f; continue; }
        if (!c->solid(ox, py, c->solid_user)) { c->y = py; c->vx = 0.0f; continue; }
        c->vx = c->vy = 0.0f;
        break;
    }
}

/* deterministic pseudo-noise in [-1, 1] */
static float shake_noise(float seed, float t) {
    return 0.6f * sinf(t * seed + seed) + 0.4f * sinf(t * seed * 2.7f + seed * 1.3f);
}

static void cam_update(Camera* c, float tx, float ty, float dt) {
    if (dt <= 0.0f) return;

    /* advance parameter blend */
    if (c->blend_d > 0.0f) {
        c->blend_t += dt;
        float u = clampf(c->blend_t / c->blend_d, 0.0f, 1.0f);
        CamParams* a = &c->from;
        CamParams* b = &c->dst;
        c->cur.smooth_time = a->smooth_time + (b->smooth_time - a->smooth_time) * u;
        c->cur.lead_time   = a->lead_time   + (b->lead_time   - a->lead_time)   * u;
        c->cur.max_lead    = a->max_lead    + (b->max_lead    - a->max_lead)    * u;
        c->cur.max_shake   = a->max_shake   + (b->max_shake   - a->max_shake)   * u;
        if (u >= 1.0f) c->blend_d = 0.0f;
    }

    /* target velocity estimate for lookahead */
    if (!c->has_target) cam_snap(c, tx, ty);
    float tvx = (tx - c->px) / dt;
    float tvy = (ty - c->py) / dt;
    c->px = tx; c->py = ty;

    /* desired predictive lead, clamped */
    float dlx = tvx * c->cur.lead_time;
    float dly = tvy * c->cur.lead_time;
    float lm = sqrtf(dlx * dlx + dly * dly);
    if (lm > c->cur.max_lead && lm > 0.0f) {
        float s = c->cur.max_lead / lm;
        dlx *= s; dly *= s;
    }
    /* smooth the applied lead: lead_time/param switches ramp in instead of
     * jumping the goal (no hiccup) */
    float k = 1.0f - expf(-dt * 5.0f);
    c->lead_x += (dlx - c->lead_x) * k;
    c->lead_y += (dly - c->lead_y) * k;

    /* the offset spring lives entirely in the target's frame: the state is
     * never derived from a stale target position, so a moving target is
     * tracked with zero steady-state lag and the net lead equals
     * target_vel * lead_time exactly */
    float nox = smooth_damp(c->ox, c->lead_x, &c->vx, c->cur.smooth_time, dt);
    float noy = smooth_damp(c->oy, c->lead_y, &c->vy, c->cur.smooth_time, dt);

    move_with_collision(c, tx + nox, ty + noy);

    if (c->clamp_x) c->x = clampf(c->x, c->min_x, c->max_x);
    if (c->clamp_y) c->y = clampf(c->y, c->min_y, c->max_y);

    /* re-anchor the offset on the final position so wall/bounds clamping
     * is picked up by the spring next frame */
    c->ox = c->x - tx;
    c->oy = c->y - ty;

    /* shakes: stack, decay, expire, then cap total magnitude */
    c->shake_x = c->shake_y = 0.0f;
    for (int i = 0; i < c->shake_count; i++) {
        Shake* s = &c->shakes[i];
        s->t += dt;
        if (s->t >= s->duration) {
            c->shakes[i] = c->shakes[--c->shake_count];
            i--;
            continue;
        }
        float u = s->t / s->duration;
        float env = powf(1.0f - u, s->decay);
        float a = s->amp * env;
        c->shake_x += a * shake_noise(s->seed_x, s->t * s->freq);
        c->shake_y += a * shake_noise(s->seed_y, s->t * s->freq);
    }
    float sm = sqrtf(c->shake_x * c->shake_x + c->shake_y * c->shake_y);
    float cap = c->cur.max_shake;
    if (cap >= 0.0f && sm > cap && sm > 0.0f) {
        float s = cap / sm;
        c->shake_x *= s;
        c->shake_y *= s;
    }
}

/* ---------------- self test ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef struct { const unsigned char* cells; int w, h; float tile; } Grid;

static int grid_solid(float x, float y, void* user) {
    Grid* g = (Grid*)user;
    int cx = (int)floorf(x / g->tile);
    int cy = (int)floorf(y / g->tile);
    if (cx < 0 || cy < 0 || cx >= g->w || cy >= g->h) return 1;
    return g->cells[cy * g->w + cx] != 0;
}

static CamParams default_params(void) {
    CamParams p;
    p.smooth_time = 0.3f;
    p.lead_time = 0.2f;
    p.max_lead = 2.0f;
    p.max_shake = 0.5f;
    return p;
}

int main(void) {
    const float dt = 1.0f / 60.0f;

    /* hand-computed reference: same formula, unrolled scalar, 5 frames */
    {
        Camera c;
        cam_init(&c, default_params());
        c.cur.lead_time = 0.0f;  /* disable lookahead for the reference */
        float rx = 0.0f, rv = 0.0f;      /* reference pos/vel, target = (10, 0) */
        int match = 1;
        for (int i = 0; i < 5; i++) {
            cam_update(&c, 10.0f, 0.0f, dt);
            float omega = 2.0f / 0.3f;
            float x = omega * dt;
            float e = 1.0f / (1.0f + x + 0.48f * x * x + 0.235f * x * x * x);
            float change = rx - 10.0f;
            float temp = (rv + omega * change) * dt;
            rv = (rv - omega * temp) * e;
            rx = 10.0f + (change + temp) * e;
            if (fabsf(c.x - rx) > 1e-5f || fabsf(c.y) > 1e-6f) match = 0;
        }
        check("hand_calc_5_frames", match);
    }

    /* follow converges onto a stationary target */
    {
        Camera c;
        cam_init(&c, default_params());
        for (int i = 0; i < 180; i++) cam_update(&c, 10.0f, 5.0f, dt);
        check("follow_converges", fabsf(c.x - 10.0f) < 0.01f && fabsf(c.y - 5.0f) < 0.01f);
    }

    /* lookahead: steady-state lead equals target_speed * lead_time */
    {
        Camera c;
        cam_init(&c, default_params());
        float tx = 0.0f;
        for (int i = 0; i < 240; i++) { tx += 5.0f * dt; cam_update(&c, tx, 0.0f, dt); }
        float lead = c.x - tx;
        check("lookahead_steady", fabsf(lead - 5.0f * 0.2f) < 0.05f);
    }

    /* lookahead clamped by max_lead */
    {
        Camera c;
        cam_init(&c, default_params());
        float tx = 0.0f;
        for (int i = 0; i < 300; i++) { tx += 30.0f * dt; cam_update(&c, tx, 0.0f, dt); }
        float lead = c.x - tx;
        check("lookahead_capped", lead <= 2.0f + 1e-4f && lead > 1.5f);
    }

    /* bounds: view stays inside world even when target sits in a corner */
    {
        Camera c;
        cam_init(&c, default_params());
        cam_set_bounds(&c, 0.0f, 0.0f, 20.0f, 20.0f, 4.0f, 4.0f);
        for (int i = 0; i < 240; i++) cam_update(&c, 0.0f, 0.0f, dt);
        check("bounds_corner_lo", fabsf(c.x - 2.0f) < 0.01f && fabsf(c.y - 2.0f) < 0.01f);
        for (int i = 0; i < 240; i++) cam_update(&c, 20.0f, 20.0f, dt);
        check("bounds_corner_hi", fabsf(c.x - 18.0f) < 0.01f && fabsf(c.y - 18.0f) < 0.01f);
        int inside = 1;
        for (int i = 0; i < 120; i++) {
            cam_update(&c, (i % 2) ? 0.0f : 20.0f, (i % 3) ? 0.0f : 20.0f, dt);
            if (c.x < 2.0f - 1e-4f || c.x > 18.0f + 1e-4f ||
                c.y < 2.0f - 1e-4f || c.y > 18.0f + 1e-4f) inside = 0;
        }
        check("bounds_always_inside", inside);
    }

    /* walls: target crosses a solid column; camera must never penetrate */
    {
        /* 20x20 grid, tile=1, solid column at x=5 except none */
        static unsigned char cells[400];
        memset(cells, 0, sizeof(cells));
        for (int y = 0; y < 20; y++) cells[y * 20 + 5] = 1;
        Grid g = { cells, 20, 20, 1.0f };

        Camera c;
        cam_init(&c, default_params());
        cam_set_world(&c, grid_solid, &g, 1.0f);
        c.x = 4.5f; c.y = 10.5f;
        cam_snap(&c, 4.5f, 10.5f);

        int penetrations = 0;
        float tx = 4.5f;
        for (int i = 0; i < 600; i++) {
            tx += 4.0f * dt;  /* target walks through the wall to x~44 */
            cam_update(&c, tx, 10.5f, dt);
            if (grid_solid(c.x, c.y, &g)) penetrations++;
        }
        check("wall_penetrations_zero", penetrations == 0);
        check("wall_stops_camera", c.x < 5.0f && c.x > 4.0f);

        /* teleport across the wall in one big frame: substepping must hold */
        Camera c2;
        cam_init(&c2, default_params());
        cam_set_world(&c2, grid_solid, &g, 1.0f);
        c2.x = 4.5f; c2.y = 10.5f;
        int pen2 = 0;
        for (int i = 0; i < 30; i++) {
            cam_update(&c2, 15.5f, 10.5f, 0.5f);  /* huge dt, target past wall */
            if (grid_solid(c2.x, c2.y, &g)) pen2++;
        }
        check("wall_no_tunnel_big_dt", pen2 == 0 && c2.x < 5.0f);
    }

    /* parameter switch: no position jump, no displacement spike */
    {
        Camera c;
        cam_init(&c, default_params());
        float tx = 0.0f;
        for (int i = 0; i < 60; i++) { tx += 3.0f * dt; cam_update(&c, tx, 2.0f, dt); }
        float before_x = c.x, before_y = c.y;
        CamParams fast = default_params();
        fast.smooth_time = 0.05f;
        fast.lead_time = 0.5f;
        cam_set_params(&c, fast);  /* hard switch mid-flight */
        check("switch_no_pos_jump", fabsf(c.x - before_x) < 1e-6f && fabsf(c.y - before_y) < 1e-6f);
        float prev_x = c.x, prev_y = c.y, max_step = 0.0f;
        for (int i = 0; i < 30; i++) {
            tx += 3.0f * dt;
            cam_update(&c, tx, 2.0f, dt);
            float step = sqrtf((c.x - prev_x) * (c.x - prev_x) + (c.y - prev_y) * (c.y - prev_y));
            if (step > max_step) max_step = step;
            prev_x = c.x; prev_y = c.y;
        }
        /* per-frame move stays in the same order as target motion (3*dt=0.05) */
        check("switch_no_spike", max_step < 0.15f);

        /* blended switch also stays continuous */
        cam_tween_params(&c, default_params(), 0.5f);
        float bx = c.x, by = c.y;
        cam_update(&c, tx, 2.0f, dt);
        float blend_step = sqrtf((c.x - bx) * (c.x - bx) + (c.y - by) * (c.y - by));
        check("blend_switch_smooth", blend_step < 0.15f);
    }

    /* shake: stacks, capped, expires */
    {
        Camera c;
        cam_init(&c, default_params());
        check("shake_add", cam_add_shake(&c, 0.4f, 7.0f, 1.0f, 1.0f, 1.0f) == 0);
        check("shake_stacks", cam_add_shake(&c, 0.4f, 11.0f, 1.0f, 1.0f, 2.0f) == 0);
        check("shake_stacks3", cam_add_shake(&c, 0.4f, 13.0f, 1.0f, 1.0f, 3.0f) == 0);
        int capped = 1, moved = 0;
        for (int i = 0; i < 55; i++) {  /* sum of amps = 1.2 > cap 0.5 */
            cam_update(&c, 0.0f, 0.0f, dt);
            float m = sqrtf(c.shake_x * c.shake_x + c.shake_y * c.shake_y);
            if (m > 0.5f + 1e-4f) capped = 0;
            if (m > 1e-3f) moved = 1;
        }
        check("shake_capped", capped);
        check("shake_nonzero", moved);
        for (int i = 0; i < 60; i++) cam_update(&c, 0.0f, 0.0f, dt);
        check("shake_expires", c.shake_count == 0 &&
              fabsf(c.shake_x) < 1e-6f && fabsf(c.shake_y) < 1e-6f);
        check("shake_reject_bad", cam_add_shake(&c, -1.0f, 1.0f, 1.0f, 1.0f, 0.0f) == -1);
    }

    /* determinism: identical inputs -> identical trajectory */
    {
        Camera a, b;
        cam_init(&a, default_params());
        cam_init(&b, default_params());
        cam_add_shake(&a, 0.3f, 9.0f, 1.5f, 2.0f, 4.0f);
        cam_add_shake(&b, 0.3f, 9.0f, 1.5f, 2.0f, 4.0f);
        int same = 1;
        float tx = 0.0f;
        for (int i = 0; i < 300; i++) {
            tx += (i % 7) * 0.01f;
            cam_update(&a, tx, sinf(i * 0.1f), dt);
            cam_update(&b, tx, sinf(i * 0.1f), dt);
            if (a.x != b.x || a.y != b.y || a.shake_x != b.shake_x) same = 0;
        }
        check("deterministic_replay", same);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
