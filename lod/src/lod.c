
/* lod-selector: screen-space-error LOD selection with hysteresis to avoid
   level thrashing when the camera hovers near a threshold. */
#include <stdio.h>
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

int main(void) {
    MeshLod m = { {0.0f, 1.0f, 4.0f, 16.0f}, 2.0f, 1.1f };
    State st = {0, 0, 0};
    float fov = 1.0f; int h = 1080;
    int hist[LEVELS] = {0};
    for (int i = 0; i < 40; i++) {
        float d = 5.0f + 5.0f * i;             /* receding */
        int lv = select_level(&m, d, fov, h, 50.0f, &st);
        hist[lv]++;
    }
    printf("levels=%d,%d,%d,%d switches=%d\n", hist[0], hist[1], hist[2], hist[3], st.switches);
    /* 迟滞应当让切换次数远小于采样次数 */
    printf("hysteresis_ok=%d\n", st.switches <= 8);
    return 0;
}
