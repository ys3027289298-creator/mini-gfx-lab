
/* syncheck: consistency validation between render tiles and collision bodies.
   Render data (solid tiles) and collision data (body rects) are edited
   separately and drift apart. validate() rasterizes bodies and compares
   per cell; a slow per-cell manual re-check verifies the fast pass;
   mismatches are aggregated per region and turned into fix suggestions. */
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MAX_W 128
#define MAX_H 128
#define MAX_BODIES 512
#define MAX_SUGGEST 1024
#define REGION 16               /* region summary granularity, in cells */

enum { KIND_OK = 0, KIND_RENDER_ONLY = 1, KIND_COLLISION_ONLY = 2 };

typedef struct {
    int w, h;
    unsigned char tile[MAX_H][MAX_W];   /* tile id per cell, 0 = empty */
    unsigned char solid[256];           /* tile id -> blocks movement */
} RenderMap;

typedef struct { int x0, y0, x1, y1; } Body;    /* inclusive cell rect */

typedef struct { Body b[MAX_BODIES]; int n; } CollisionSet;

typedef struct {
    int w, h;
    unsigned char kind[MAX_H][MAX_W];   /* KIND_* per cell */
    int render_only;                    /* solid tile, no collision */
    int collision_only;                 /* collision, no solid tile */
    int oob_bodies;                     /* bodies fully outside the map */
} Report;

static void render_init(RenderMap* rm, int w, int h) {
    memset(rm, 0, sizeof(*rm));
    rm->w = w; rm->h = h;
}

static void render_set(RenderMap* rm, int x, int y, unsigned char id) {
    if (x < 0 || y < 0 || x >= rm->w || y >= rm->h) return;
    rm->tile[y][x] = id;
}

static int body_add(CollisionSet* cs, int x0, int y0, int x1, int y1) {
    if (cs->n >= MAX_BODIES) return -1;
    cs->b[cs->n++] = (Body){ x0, y0, x1, y1 };
    return 0;
}

/* fast pass: rasterize bodies once, then one compare per cell */
static void validate(const RenderMap* rm, const CollisionSet* cs, Report* rep) {
    static unsigned char cover[MAX_H][MAX_W];
    memset(rep, 0, sizeof(*rep));
    memset(cover, 0, sizeof(cover));
    rep->w = rm->w; rep->h = rm->h;
    for (int i = 0; i < cs->n; i++) {
        Body b = cs->b[i];
        int x0 = b.x0 < 0 ? 0 : b.x0;
        int y0 = b.y0 < 0 ? 0 : b.y0;
        int x1 = b.x1 >= rm->w ? rm->w - 1 : b.x1;
        int y1 = b.y1 >= rm->h ? rm->h - 1 : b.y1;
        if (x0 > x1 || y0 > y1) { rep->oob_bodies++; continue; }
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                cover[y][x] = 1;
    }
    for (int y = 0; y < rm->h; y++)
        for (int x = 0; x < rm->w; x++) {
            int rs = rm->solid[rm->tile[y][x]];
            if (rs && !cover[y][x]) { rep->kind[y][x] = KIND_RENDER_ONLY; rep->render_only++; }
            else if (!rs && cover[y][x]) { rep->kind[y][x] = KIND_COLLISION_ONLY; rep->collision_only++; }
        }
}

/* reference: naive per-cell check against every body, no rasterization */
static int manual_cell(const RenderMap* rm, const CollisionSet* cs, int x, int y) {
    int rs = rm->solid[rm->tile[y][x]];
    int hit = 0;
    for (int i = 0; i < cs->n; i++)
        if (x >= cs->b[i].x0 && x <= cs->b[i].x1 &&
            y >= cs->b[i].y0 && y <= cs->b[i].y1) { hit = 1; break; }
    if (rs && !hit) return KIND_RENDER_ONLY;
    if (!rs && hit) return KIND_COLLISION_ONLY;
    return KIND_OK;
}

typedef struct { int rx, ry, render_only, collision_only; } RegionCount;

/* aggregate mismatches per REGION x REGION block; emits only non-empty regions */
static int region_summary(const Report* rep, RegionCount* out, int max_out) {
    int n = 0;
    for (int ry = 0; ry * REGION < rep->h; ry++)
        for (int rx = 0; rx * REGION < rep->w; rx++) {
            int x1 = (rx + 1) * REGION < rep->w ? (rx + 1) * REGION : rep->w;
            int y1 = (ry + 1) * REGION < rep->h ? (ry + 1) * REGION : rep->h;
            int ro = 0, co = 0;
            for (int y = ry * REGION; y < y1; y++)
                for (int x = rx * REGION; x < x1; x++) {
                    if (rep->kind[y][x] == KIND_RENDER_ONLY) ro++;
                    else if (rep->kind[y][x] == KIND_COLLISION_ONLY) co++;
                }
            if ((ro || co) && n < max_out) {
                out[n++] = (RegionCount){ rx, ry, ro, co };
            }
        }
    return n;
}

typedef struct { int type; Body rect; char text[96]; } Suggest;

/* merge same-type mismatch cells into maximal rects, one fix suggestion each */
static int suggest_fixes(const Report* rep, Suggest* out, int max_out) {
    static unsigned char done[MAX_H][MAX_W];
    memset(done, 0, sizeof(done));
    int n = 0;
    for (int y = 0; y < rep->h; y++)
        for (int x = 0; x < rep->w; x++) {
            int t = rep->kind[y][x];
            if (!t || done[y][x]) continue;
            int wdt = 0;
            while (x + wdt < rep->w && rep->kind[y][x + wdt] == t && !done[y][x + wdt]) wdt++;
            int hgt = 1;
            while (y + hgt < rep->h) {
                int ok = 1;
                for (int i = 0; i < wdt; i++)
                    if (rep->kind[y + hgt][x + i] != t || done[y + hgt][x + i]) { ok = 0; break; }
                if (!ok) break;
                hgt++;
            }
            for (int j = 0; j < hgt; j++)
                for (int i = 0; i < wdt; i++) done[y + j][x + i] = 1;
            if (n < max_out) {
                Suggest* s = &out[n++];
                s->type = t;
                s->rect = (Body){ x, y, x + wdt - 1, y + hgt - 1 };
                if (t == KIND_RENDER_ONLY)
                    snprintf(s->text, sizeof(s->text),
                        "add collision body (%d,%d)-(%d,%d), or clear solid tiles there",
                        x, y, x + wdt - 1, y + hgt - 1);
                else
                    snprintf(s->text, sizeof(s->text),
                        "remove collision (%d,%d)-(%d,%d), or paint solid tiles there",
                        x, y, x + wdt - 1, y + hgt - 1);
            }
        }
    return n;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    static RenderMap rm;
    static CollisionSet cs;
    static Report rep;
    render_init(&rm, 32, 24);
    rm.solid[1] = 1;

    /* matching area: solid tiles + identical body -> no issues */
    for (int y = 5; y <= 6; y++)
        for (int x = 20; x <= 23; x++) render_set(&rm, x, y, 1);
    body_add(&cs, 20, 5, 23, 6);

    /* planted drift A: 3 solid tiles with no collision */
    for (int x = 2; x <= 4; x++) render_set(&rm, x, 3, 1);

    /* planted drift B: collision body with no solid tiles */
    body_add(&cs, 10, 10, 12, 11);

    /* body fully outside the map */
    body_add(&cs, -5, -5, -1, -1);

    validate(&rm, &cs, &rep);
    check("render_only_count", rep.render_only == 3);
    check("collision_only_count", rep.collision_only == 6);
    check("oob_body_count", rep.oob_bodies == 1);
    check("matching_area_clean", rep.kind[5][20] == KIND_OK && rep.kind[6][23] == KIND_OK);

    /* fast pass must equal naive per-cell manual check on every cell */
    int manual_diff = 0;
    for (int y = 0; y < rm.h; y++)
        for (int x = 0; x < rm.w; x++)
            if (rep.kind[y][x] != manual_cell(&rm, &cs, x, y)) manual_diff++;
    check("matches_manual_per_cell", manual_diff == 0);

    /* region summary: totals must equal global counts */
    RegionCount regions[64];
    int rn = region_summary(&rep, regions, 64);
    int ro_sum = 0, co_sum = 0;
    for (int i = 0; i < rn; i++) { ro_sum += regions[i].render_only; co_sum += regions[i].collision_only; }
    check("region_totals_match", ro_sum == rep.render_only && co_sum == rep.collision_only);
    check("region_single_hot", rn == 1 && regions[0].rx == 0 && regions[0].ry == 0 &&
          regions[0].render_only == 3 && regions[0].collision_only == 6);

    /* suggestions: rects must partition the mismatch cells exactly, per type */
    static Suggest sug[MAX_SUGGEST];
    int sn = suggest_fixes(&rep, sug, MAX_SUGGEST);
    static unsigned char seen[2][MAX_H][MAX_W];
    memset(seen, 0, sizeof(seen));
    int sug_bad = 0;
    for (int i = 0; i < sn; i++) {
        for (int y = sug[i].rect.y0; y <= sug[i].rect.y1; y++)
            for (int x = sug[i].rect.x0; x <= sug[i].rect.x1; x++) {
                if (rep.kind[y][x] != sug[i].type) sug_bad++;       /* covers a clean cell */
                if (seen[sug[i].type - 1][y][x]) sug_bad++;         /* overlaps another rect */
                seen[sug[i].type - 1][y][x] = 1;
            }
    }
    for (int y = 0; y < rep.h; y++)
        for (int x = 0; x < rep.w; x++)
            if (rep.kind[y][x] && !seen[rep.kind[y][x] - 1][y][x]) sug_bad++; /* missed a cell */
    check("suggestions_exact_partition", sug_bad == 0);
    check("suggest_render_only_rect", sn >= 1 && sug[0].type == KIND_RENDER_ONLY &&
          sug[0].rect.x0 == 2 && sug[0].rect.y0 == 3 && sug[0].rect.x1 == 4 && sug[0].rect.y1 == 3);
    for (int i = 0; i < sn && i < 8; i++) printf("fix: %s\n", sug[i].text);

    /* performance + manual cross-check on 10000 cells */
    static RenderMap big;
    static CollisionSet bcs;
    static Report brep;
    render_init(&big, 100, 100);
    big.solid[1] = 1;
    for (int y = 0; y < 100; y++)
        for (int x = 0; x < 100; x++)
            if ((x * x + y * 7) % 11 < 3) render_set(&big, x, y, 1);
    for (int i = 0; i < 40; i++)
        body_add(&bcs, (i * 13) % 90, (i * 29) % 90, (i * 13) % 90 + 6, (i * 29) % 90 + 4);

    clock_t t0 = clock();
    for (int i = 0; i < 200; i++) validate(&big, &bcs, &brep);
    double avg_ms = 1000.0 * (double)(clock() - t0) / CLOCKS_PER_SEC / 200.0;
    printf("validate_10k_avg_ms=%.3f\n", avg_ms);
    check("validate_10k_cells", brep.w * brep.h == 10000);
    check("validate_10k_under_50ms", avg_ms < 50.0);

    int big_diff = 0;
    for (int y = 0; y < big.h; y++)
        for (int x = 0; x < big.w; x++)
            if (brep.kind[y][x] != manual_cell(&big, &bcs, x, y)) big_diff++;
    check("big_matches_manual_per_cell", big_diff == 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
