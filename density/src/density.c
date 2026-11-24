
/* density-budget: screen-fraction particle budgeting.
   相机拉远时固定每格配额会让单格塞进上千粒；这里按每格在屏幕上的
   像素面积重新分配总配额，长焦/短焦各走一条密度曲线。比例系数是
   fov 与分辨率的闭式解，深度只作为每格自身的几何因子参与。 */
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ---------- 参考视口：在该视口下密度系数 K == 1 ---------- */
#define REF_FOV_Y     1.0472f   /* 60 度 */
#define REF_SCREEN_H  1080.0f
#define FOCAL_SPLIT   1.0472f   /* fov <= 该值为长焦段，否则为短焦段 */

/* 两条密度曲线：长焦保持恒定屏幕密度（面积一次方），
   广角让密度随面积缓慢稀释，避免远景粒子成片消失 */
#define EXP_TELE  1.0f
#define EXP_WIDE  0.6f

#define MAX_CELLS 4096
#define MAX_POOL  32768

typedef enum { FOCAL_TELE = 0, FOCAL_WIDE = 1 } FocalRange;

typedef struct {
    int   cells_side;       /* 方形网格边长（格） */
    float cell_world;       /* 每格世界边长 */
    int   budget;           /* 每帧全网格发射总预算 */
    int   cell_cap;         /* 单格存活粒子上限，压不住就退回均匀分配 */
    int   max_particles;    /* 粒子池容量 */
    float life;             /* 粒子寿命（秒） */
} Config;

typedef struct {
    float fov_y;            /* 垂直视场，弧度 */
    int   screen_h;         /* 垂直分辨率，像素 */
    int   enabled;          /* 密度分配开关，关时均匀分配 */
} Camera;

typedef struct { float depth; int alive, quota, high_water; } Cell;
typedef struct { int cell; float age; int alive; } Particle;

typedef struct {
    const Config* cfg;
    Cell* cells;
    Particle* pool;
    int n_cells, alive, cursor;
    int peak[2];            /* 两焦段各自的单格存活峰值，切换开关不重置 */
    int fallback;           /* 极端长焦压测失败后置位，永久均匀分配 */
} World;

/* 屏幕面积比例系数，只从视场和分辨率直接解出：
   参考平面上每世界单位占 screen_h / (2d tan(fov/2)) 像素，
   平方后与参考视口作比，深度 d 被约掉：
   K = (H/H0)^2 * (tan(fov0/2) / tan(fov/2))^2 */
static float footprint_k(float fov_y, int screen_h) {
    float s = tanf(REF_FOV_Y * 0.5f) / tanf(fov_y * 0.5f);
    float r = (float)screen_h / REF_SCREEN_H;
    return r * r * s * s;
}

static FocalRange focal_range(float fov_y) {
    return fov_y <= FOCAL_SPLIT ? FOCAL_TELE : FOCAL_WIDE;
}

/* 当前视锥覆盖的格数（每维），深度取最近格作为可视范围基准 */
static int visible_span(const Config* cfg, const Camera* cam, float depth_near) {
    float world_h = 2.0f * depth_near * tanf(cam->fov_y * 0.5f);
    int span = (int)ceilf(world_h / cfg->cell_world);
    if (span < 1) span = 1;
    if (span > cfg->cells_side) span = cfg->cells_side;
    return span;
}

/* 计算一帧每格发射配额。
   关闭：可见格均匀分总预算；
   回退（极端长焦压测失败）：全网格均匀，把预算摊到含视锥外的所有格，
   单格配额被 budget/N 硬封顶，这是 cell_cap 唯一可证明的保证；
   开启：权重 = (K / depth^2)^指数，按权重在可见格内分总预算，
   最大余数法补齐取整误差。 */
static void allocate(const World* w, const Camera* cam, int span) {
    const Config* cfg = w->cfg;
    int n_vis = span * span;
    int n = w->fallback ? w->n_cells : n_vis;   /* 回退时全网格参与分配 */
    float k = footprint_k(cam->fov_y, cam->screen_h);
    FocalRange fr = focal_range(cam->fov_y);
    float expo = fr == FOCAL_TELE ? EXP_TELE : EXP_WIDE;

    float weight[MAX_CELLS], frac[MAX_CELLS];
    float sum = 0.0f;
    for (int i = 0; i < n; i++) {
        Cell* c = &w->cells[i];
        if (!cam->enabled || w->fallback) {
            weight[i] = 1.0f;
        } else {
            float d2 = c->depth * c->depth;
            weight[i] = powf(k / d2, expo);
        }
        sum += weight[i];
    }

    int used = 0;
    for (int i = 0; i < n; i++) {
        float exact = sum > 0.0f ? cfg->budget * weight[i] / sum : 0.0f;
        int q = (int)exact;
        frac[i] = exact - (float)q;
        w->cells[i].quota = q;
        used += q;
    }
    int left = cfg->budget - used;
    while (left > 0) {
        int best = -1;
        float best_frac = -1.0f;
        for (int i = 0; i < n; i++)
            if (frac[i] >= 0.0f && frac[i] > best_frac) { best_frac = frac[i]; best = i; }
        if (best < 0) break;
        w->cells[best].quota++;
        frac[best] = -1.0f;       /* 本轮已补过 */
        left--;
    }
    for (int i = n; i < w->n_cells; i++) w->cells[i].quota = 0;
}

static void spawn(World* w, int span) {
    /* 回退时全网格都可能分到配额；正常模式只有可见格有非零配额 */
    int n = w->fallback ? w->n_cells : span * span;
    for (int i = 0; i < n; i++) {
        Cell* c = &w->cells[i];
        for (int s = 0; s < c->quota; s++) {
            int idx = -1;
            for (int t = 0; t < w->cfg->max_particles; t++) {
                int cand = (w->cursor + t) % w->cfg->max_particles;
                if (!w->pool[cand].alive) { idx = cand; break; }
            }
            if (idx < 0) return;           /* 池子打满，本帧停止发射 */
            w->cursor = (idx + 1) % w->cfg->max_particles;
            w->pool[idx].alive = 1;
            w->pool[idx].cell = i;
            w->pool[idx].age = 0.0f;
            w->alive++;
            c->alive++;
            if (c->alive > c->high_water) c->high_water = c->alive;
        }
    }
}

static void age_particles(World* w, float dt) {
    for (int i = 0; i < w->cfg->max_particles; i++) {
        Particle* p = &w->pool[i];
        if (!p->alive) continue;
        p->age += dt;
        if (p->age >= w->cfg->life) {
            p->alive = 0;
            w->cells[p->cell].alive--;
            w->alive--;
        }
    }
}

static int worst_cell(const World* w, int span) {
    int worst = 0;
    for (int i = 0; i < span * span; i++)
        if (w->cells[i].alive > worst) worst = w->cells[i].alive;
    return worst;
}

/* 推进一帧，并记录当前焦段的单格存活峰值；峰值只增不减，开关切换不清零 */
static void step(World* w, const Camera* cam, float dt) {
    int span = visible_span(w->cfg, cam, w->cells[0].depth);
    allocate(w, cam, span);
    spawn(w, span);
    age_particles(w, dt);
    FocalRange fr = focal_range(cam->fov_y);
    int worst = worst_cell(w, span);
    if (worst > w->peak[fr]) w->peak[fr] = worst;
}

static int cell_peak_worst(const Cell* cells, int n) {
    int worst = 0;
    for (int i = 0; i < n; i++)
        if (cells[i].high_water > worst) worst = cells[i].high_water;
    return worst;
}

/* 最极端长焦端压测：fov 收到最窄，跑到稳态后看单格峰值。
   压不住 cap 就置回退标志、清零峰值记录后再跑一个稳态周期，
   验证均匀分配能否把单格峰值压回 cap 内。 */
static int extreme_tele_test(const Config* cfg, Cell* cells, Particle* pool,
                             int min_fov, int screen_h, int* uniform_worst) {
    World w;
    memset(&w, 0, sizeof(w));
    w.cfg = cfg; w.cells = cells; w.pool = pool; w.n_cells = cfg->cells_side * cfg->cells_side;
    Camera cam = { (float)min_fov * 0.0174533f, screen_h, 1 };

    for (int f = 0; f < 240; f++) step(&w, &cam, 0.05f);   /* 超过 2s 寿命，达稳态 */
    int span = visible_span(cfg, &cam, cells[0].depth);
    int density_worst = cell_peak_worst(cells, span * span);
    int triggered = density_worst > cfg->cell_cap;

    if (triggered) {
        w.fallback = 1;
        /* 回退验证必须从空状态起步：密度期攒下的存量会污染均匀期的峰值记录 */
        memset(cells, 0, sizeof(Cell) * (size_t)w.n_cells);
        memset(pool, 0, sizeof(Particle) * (size_t)cfg->max_particles);
        w.alive = 0; w.cursor = 0;
        for (int f = 0; f < 240; f++) step(&w, &cam, 0.05f);
        *uniform_worst = cell_peak_worst(cells, span * span);
    } else {
        *uniform_worst = density_worst;
    }
    printf("extreme_tele fov=%d deg visible=%d density_worst=%d cap=%d -> %s",
           min_fov, span * span, density_worst, cfg->cell_cap,
           triggered ? "FALLBACK_UNIFORM" : "density_holds");
    if (triggered) printf(" | uniform_worst=%d", *uniform_worst);
    printf("\n");
    return triggered;
}

int main(void) {
    int fails = 0;

    /* ---- 1. 比例系数闭式解：只由 fov 与分辨率决定 ---- */
    printf("== footprint coefficient (closed form from fov + resolution) ==\n");
    float k_ref  = footprint_k(REF_FOV_Y, 1080);
    float k_tele = footprint_k(10.0f * 0.0174533f, 1080);
    float k_wide = footprint_k(120.0f * 0.0174533f, 1080);
    float k_4k   = footprint_k(REF_FOV_Y, 2160);
    printf("K(ref 60deg/1080p)=%.4f K(10deg)=%.3f K(120deg)=%.4f K(4K)=%.4f\n",
           k_ref, k_tele, k_wide, k_4k);
    if (fabsf(k_ref - 1.0f) > 1e-4f) { printf("FAIL reference K != 1\n"); fails++; }
    if (!(k_tele > 40.0f))            { printf("FAIL tele K should be large\n"); fails++; }
    if (!(k_wide < 0.3f))             { printf("FAIL wide K should shrink\n"); fails++; }
    if (fabsf(k_4k - 4.0f) > 1e-3f)   { printf("FAIL 4K K != 4\n"); fails++; }

    Config cfg = { 64, 2.0f, 200, 350, MAX_POOL, 2.0f };
    int n = cfg.cells_side * cfg.cells_side;
    Cell cells[MAX_CELLS];
    Particle pool[MAX_POOL];

    /* ---- 2. 正常参数 + 共面格网：极端长焦压测能压住，不回退 ---- */
    printf("\n== normal profile, coplanar grid: extreme telephoto stress ==\n");
    memset(cells, 0, sizeof(cells));
    memset(pool, 0, sizeof(pool));
    for (int i = 0; i < n; i++) cells[i].depth = 100.0f;
    int uniform_worst = 0;
    int fb_normal = extreme_tele_test(&cfg, cells, pool, 10, 1080, &uniform_worst);
    if (fb_normal) { printf("FAIL normal profile should hold density\n"); fails++; }

    /* ---- 3. 激进参数 + 深度分层：近端格独占配额 -> 回退均匀 ---- */
    printf("\n== aggressive profile, depth-layered grid: must fall back ==\n");
    Config hot = cfg; hot.budget = 600; hot.cell_cap = 350;
    memset(cells, 0, sizeof(cells));
    memset(pool, 0, sizeof(pool));
    int span9 = 9;                          /* 10 度视锥在 100 深度处覆盖 9x9 */
    for (int i = 0; i < n; i++)
        cells[i].depth = 100.0f + 25.0f * (float)(i % span9);
    int fb_hot = extreme_tele_test(&hot, cells, pool, 10, 1080, &uniform_worst);
    if (!fb_hot) { printf("FAIL aggressive profile should trigger fallback\n"); fails++; }
    if (uniform_worst > hot.cell_cap) {
        printf("FAIL uniform fallback worst %d > cap %d\n", uniform_worst, hot.cell_cap);
        fails++;
    }

    /* ---- 4. 两焦段各自记峰值，开关切换期间持续记录、峰值不重置 ---- */
    printf("\n== focal peaks survive on/off toggling ==\n");
    World w;
    memset(&w, 0, sizeof(w));
    w.cfg = &cfg; w.cells = cells; w.pool = pool; w.n_cells = n;
    memset(cells, 0, sizeof(cells));
    memset(pool, 0, sizeof(pool));
    for (int i = 0; i < n; i++) cells[i].depth = 100.0f;

    /* 4a. 广角段稳态（开）。每换焦段先沉降超过粒子寿命，
       让上一焦段的存量凋亡，避免峰值跨焦段污染 */
    Camera wide_cam = { 110.0f * 0.0174533f, 1080, 1 };
    for (int f = 0; f < 160; f++) step(&w, &wide_cam, 0.05f);   /* 8s > 2s 寿命 */
    int wide_peak = w.peak[FOCAL_WIDE];

    /* 4b. 切长焦段（开），沉降后达稳态 */
    Camera tele_cam = { 10.0f * 0.0174533f, 1080, 1 };
    for (int f = 0; f < 160; f++) step(&w, &tele_cam, 0.05f);
    int tele_peak_on = w.peak[FOCAL_TELE];

    /* 4c. 长焦段内关闭开关跑一段，再重新打开；峰值记录不能被清零 */
    Camera tele_off = { 10.0f * 0.0174533f, 1080, 0 };
    for (int f = 0; f < 80; f++) step(&w, &tele_off, 0.05f);
    int tele_peak_mid = w.peak[FOCAL_TELE];
    for (int f = 0; f < 80; f++) step(&w, &tele_cam, 0.05f);
    int tele_peak_end = w.peak[FOCAL_TELE];

    printf("peak_per_cell wide=%d tele(on)=%d tele(after off-on)=%d\n",
           wide_peak, tele_peak_on, tele_peak_end);
    if (wide_peak <= 0 || tele_peak_on <= 0) {
        printf("FAIL both focal ranges must record a peak\n"); fails++;
    }
    /* 长焦端可见格少、单格配额大，单格峰值必然高于广角端 */
    if (!(tele_peak_on > wide_peak)) {
        printf("FAIL telephoto peak %d should exceed wide peak %d\n",
               tele_peak_on, wide_peak);
        fails++;
    }
    if (tele_peak_mid < tele_peak_on || tele_peak_end < tele_peak_mid) {
        printf("FAIL peaks must not reset across toggles: %d -> %d -> %d\n",
               tele_peak_on, tele_peak_mid, tele_peak_end);
        fails++;
    }

    /* ---- 5. 拉远场景对照：固定配额旧法 vs 屏幕占比重分配 ---- */
    printf("\n== zoom-out comparison ==\n");
    Camera near_cam = { 10.0f * 0.0174533f, 1080, 1 };
    Camera far_cam  = { 60.0f * 0.0174533f, 1080, 1 };
    int span_near = visible_span(&cfg, &near_cam, 100.0f);
    int span_far  = visible_span(&cfg, &far_cam, 100.0f);
    printf("visible cells: near(10deg)=%d far(60deg)=%d\n",
           span_near * span_near, span_far * span_far);
    memset(&w, 0, sizeof(w)); w.cfg = &cfg; w.cells = cells; w.pool = pool; w.n_cells = n;
    memset(cells, 0, sizeof(cells));
    memset(pool, 0, sizeof(pool));
    for (int i = 0; i < n; i++) cells[i].depth = 100.0f;
    for (int f = 0; f < 60; f++) step(&w, &far_cam, 0.05f);
    int far_cell0 = cells[0].high_water;
    int far_quota0 = cells[0].quota;   /* 最后一帧的每格发射配额 */
    /* 旧法：近景每格配额原样套到拉远后的全部可见格上，总投放直接超预算 */
    int legacy_per_cell = cfg.budget / (span_near * span_near);
    int legacy_total = legacy_per_cell * span_far * span_far;
    printf("density mode: far cell0 per-frame quota=%d peak stock=%d, total per frame <= %d\n",
           far_quota0, far_cell0, cfg.budget);
    printf("legacy fixed quota: per-cell=%d/frame, total would be %d/frame (%.1fx over budget)\n",
           legacy_per_cell, legacy_total, (float)legacy_total / cfg.budget);
    if (far_quota0 >= legacy_per_cell) {
        printf("FAIL screen-fraction budget must cut per-cell quota when zoomed out\n");
        fails++;
    }

    printf("\n%s\n", fails == 0 ? "ALL TESTS PASSED" : "TESTS FAILED");
    return fails;
}
