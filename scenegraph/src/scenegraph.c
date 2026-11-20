
/* scene-graph: iterative scene-graph traversal with inherited visibility,
   hierarchical view culling, and material/depth draw-item batching that is
   provably pixel-identical to immediate-mode per-object drawing.

   两条渲染路径：
   - naive  : DFS 遍历中每个可见物体立即设状态、立即画（逐物体直绘）
   - batched: 同一次遍历生成绘制项，按 (材质, 深度) 合批后再画
   两条路径写入各自的软件 framebuffer，逐字节比对。

   合批正确性：DFS 序中 y 在 x 之前且两者屏幕矩形相交时，x 处理前
   y 所在的异状态批一定已被 flush，故所有存在像素重叠的项严格保持
   直绘顺序；不相交的项重排只触及互不相交的像素集合，画面不变。 */
#include <stdio.h>
#include <string.h>
#include <time.h>

#define MAX_NODES    4096
#define MAX_CHILDREN 32
#define MAX_ITEMS    4096
#define MAX_BATCHES  256
#define MAT_COUNT    6
#define FB_W 128
#define FB_H 96

/* ---------------- scene graph ---------------- */

typedef struct {
    int parent;
    int child[MAX_CHILDREN];
    int child_n;
    int lx, ly;                 /* local translation (2D, integer) */
    int visible;                /* local visibility flag */
    int has_draw;
    int mat;                    /* material id */
    int depth;                  /* depth layer, part of the state key */
    int rx0, ry0, rx1, ry1;     /* local draw rect (if has_draw) */
    int bx0, by0, bx1, by1;     /* local subtree AABB */
    int sub_n;                  /* subtree node count */
} Node;

typedef struct {
    Node nodes[MAX_NODES];
    int n;
    unsigned mat_color[MAT_COUNT];
} Scene;

typedef struct { int x0, y0, x1, y1; } Rect;

typedef struct {
    int seq;                    /* DFS submission order */
    int mat, depth;             /* state key */
    int x0, y0, x1, y1;         /* world / screen rect */
    int node;
} DrawItem;

static void scene_init(Scene* s) {
    memset(s, 0, sizeof(*s));
    s->n = 1;
    s->nodes[0].visible = 1;
    static const unsigned colors[MAT_COUNT] = {
        0xFF2020, 0x20FF20, 0x2060FF, 0xFFFF20, 0xFF20FF, 0x20FFFF
    };
    memcpy(s->mat_color, colors, sizeof(colors));
}

static int node_add(Scene* s, int parent, int lx, int ly, int visible) {
    if (s->n >= MAX_NODES || parent < 0 || parent >= s->n) return -1;
    Node* p = &s->nodes[parent];
    if (p->child_n >= MAX_CHILDREN) return -1;
    int id = s->n++;
    memset(&s->nodes[id], 0, sizeof(Node));
    s->nodes[id].parent = parent;
    s->nodes[id].lx = lx; s->nodes[id].ly = ly;
    s->nodes[id].visible = visible;
    p->child[p->child_n++] = id;
    return id;
}

static void node_set_draw(Scene* s, int id, int mat, int depth, Rect r) {
    if (id < 0 || id >= s->n) return;
    Node* nd = &s->nodes[id];
    nd->has_draw = 1;
    nd->mat = mat; nd->depth = depth;
    nd->rx0 = r.x0; nd->ry0 = r.y0; nd->rx1 = r.x1; nd->ry1 = r.y1;
}

/* bottom-up: local subtree AABB and subtree size. Nodes are created in
   pre-order, so reverse index order visits every child before its parent. */
static void scene_finalize(Scene* s) {
    for (int i = s->n - 1; i >= 0; i--) {
        Node* nd = &s->nodes[i];
        int have = 0;
        if (nd->has_draw) {
            nd->bx0 = nd->rx0; nd->by0 = nd->ry0;
            nd->bx1 = nd->rx1; nd->by1 = nd->ry1;
            have = 1;
        }
        nd->sub_n = 1;
        for (int k = 0; k < nd->child_n; k++) {
            Node* c = &s->nodes[nd->child[k]];
            Rect cb = { c->bx0 + c->lx, c->by0 + c->ly,
                        c->bx1 + c->lx, c->by1 + c->ly };
            if (!have) {
                nd->bx0 = cb.x0; nd->by0 = cb.y0;
                nd->bx1 = cb.x1; nd->by1 = cb.y1;
                have = 1;
            } else {
                if (cb.x0 < nd->bx0) nd->bx0 = cb.x0;
                if (cb.y0 < nd->by0) nd->by0 = cb.y0;
                if (cb.x1 > nd->bx1) nd->bx1 = cb.x1;
                if (cb.y1 > nd->by1) nd->by1 = cb.y1;
            }
            nd->sub_n += c->sub_n;
        }
    }
}

static int rect_intersects(Rect a, Rect b) {
    return a.x0 < b.x1 && b.x0 < a.x1 && a.y0 < b.y1 && b.y0 < a.y1;
}

/* ---------------- software framebuffer ---------------- */

static void fb_clear(unsigned fb[FB_H][FB_W]) {
    memset(fb, 0, sizeof(unsigned) * FB_H * FB_W);
}

static void fb_fill(unsigned fb[FB_H][FB_W], Rect r, unsigned color) {
    int x0 = r.x0 > 0 ? r.x0 : 0;
    int y0 = r.y0 > 0 ? r.y0 : 0;
    int x1 = r.x1 < FB_W ? r.x1 : FB_W;
    int y1 = r.y1 < FB_H ? r.y1 : FB_H;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            fb[y][x] = color;
}

/* ---------------- traversal stats ---------------- */

typedef struct {
    int items;              /* visible, in-view drawables emitted */
    int hidden_nodes;       /* nodes skipped via inherited invisibility */
    int view_nodes;         /* nodes skipped via view cull */
    int draws;              /* draw calls issued */
    int switches;           /* state switches between consecutive draws */
    int pushes, pops;       /* structural linearity counters */
    int stack_hwm;          /* max stack depth seen */
} TravStats;

typedef struct { int node; int wx, wy; int vis; } Frame;

/* shared iterative DFS; collects draw items and cull statistics.
   stack is explicit, so node depth costs stack space, not recursion. */
static void collect(const Scene* s, Rect cam, DrawItem* out,
                    TravStats* st) {
    memset(st, 0, sizeof(*st));
    Frame stack[MAX_NODES];
    int sp = 0;
    stack[sp++] = (Frame){0, 0, 0, 1};
    st->pushes++;
    if (sp > st->stack_hwm) st->stack_hwm = sp;
    while (sp > 0) {
        Frame f = stack[--sp];
        st->pops++;
        const Node* nd = &s->nodes[f.node];
        int eff = f.vis && nd->visible;
        if (!eff) { st->hidden_nodes += nd->sub_n; continue; }
        Rect wb = { nd->bx0 + f.wx, nd->by0 + f.wy,
                    nd->bx1 + f.wx, nd->by1 + f.wy };
        if (!rect_intersects(wb, cam)) { st->view_nodes += nd->sub_n; continue; }
        if (nd->has_draw) {
            Rect wr = { nd->rx0 + f.wx, nd->ry0 + f.wy,
                        nd->rx1 + f.wx, nd->ry1 + f.wy };
            if (rect_intersects(wr, cam)) {
                DrawItem* it = &out[st->items++];
                it->seq = st->items;
                it->mat = nd->mat; it->depth = nd->depth;
                it->x0 = wr.x0; it->y0 = wr.y0; it->x1 = wr.x1; it->y1 = wr.y1;
                it->node = f.node;
            }
        }
        /* reverse push so child 0 is popped first: DFS pre-order */
        for (int k = nd->child_n - 1; k >= 0; k--) {
            int c = nd->child[k];
            stack[sp].node = c;
            stack[sp].wx = f.wx + s->nodes[c].lx;
            stack[sp].wy = f.wy + s->nodes[c].ly;
            stack[sp].vis = eff;
            sp++;
            st->pushes++;
            if (sp > st->stack_hwm) st->stack_hwm = sp;
        }
    }
}

/* path 1: immediate-mode per-object drawing; state is set per object,
   so every object whose (mat,depth) differs from the previous one costs
   a state switch. */
static void render_naive(const DrawItem* items, int n, const Scene* s,
                         unsigned fb[FB_H][FB_W], TravStats* st) {
    fb_clear(fb);
    int prev_mat = -1, prev_depth = -1;
    for (int i = 0; i < n; i++) {
        const DrawItem* it = &items[i];
        if (it->mat != prev_mat || it->depth != prev_depth) st->switches++;
        prev_mat = it->mat; prev_depth = it->depth;
        Rect r = { it->x0, it->y0, it->x1, it->y1 };
        fb_fill(fb, r, s->mat_color[it->mat]);
        st->draws++;
        st->items++;
    }
}

/* ---------------- batcher ---------------- */

typedef struct {
    int mat, depth;
    int open;
    int head, tail;         /* linked list over item indices */
    int ux0, uy0, ux1, uy1; /* union rect of contained items */
} Batch;
/* path 2: coalesce same-state items while preserving the exact order of
   every pair whose screen rects overlap. Open batches hold their linked
   list of items; an overlapping foreign-state batch is flushed first.
   st->items counts per-item fills; st->draws counts merged batch draws. */
static void render_batched(const DrawItem* items, int n, const Scene* s,
                           unsigned fb[FB_H][FB_W], TravStats* st) {
    fb_clear(fb);
    st->items = 0;      /* st->draws / switches already zeroed by caller */
    Batch batches[MAX_BATCHES];
    int next_item[MAX_ITEMS];
    int nb = 0;
    int prev_key = -1;       /* key of the previously flushed batch */

    for (int i = 0; i < n; i++) {
        const DrawItem* it = &items[i];
        Rect ir = { it->x0, it->y0, it->x1, it->y1 };
        int found = -1;
        for (int b = 0; b < nb; b++) {
            Batch* bp = &batches[b];
            if (!bp->open || bp->mat != it->mat || bp->depth != it->depth)
                continue;
            /* conservative union test: if the union misses the item, no
               member can overlap it -> safe to append with no reorder risk */
            Rect u = { bp->ux0, bp->uy0, bp->ux1, bp->uy1 };
            if (!rect_intersects(u, ir)) { found = b; break; }
            /* exact per-item test along the batch chain */
            int clash = 0;
            for (int j = bp->head; j >= 0; j = next_item[j]) {
                Rect jr = { items[j].x0, items[j].y0, items[j].x1, items[j].y1 };
                if (rect_intersects(jr, ir)) { clash = 1; break; }
            }
            if (!clash) { found = b; break; }
        }
        /* flush every overlapping open batch of a different state */
        for (int b = 0; b < nb; b++) {
            Batch* bp = &batches[b];
            if (!bp->open || (bp->mat == it->mat && bp->depth == it->depth))
                continue;
            Rect u = { bp->ux0, bp->uy0, bp->ux1, bp->uy1 };
            if (!rect_intersects(u, ir)) continue;
            int key = bp->mat * 1024 + bp->depth;
            if (key != prev_key) { st->switches++; prev_key = key; }
            st->draws++;
            for (int j = bp->head; j >= 0; j = next_item[j]) {
                Rect jr = { items[j].x0, items[j].y0, items[j].x1, items[j].y1 };
                fb_fill(fb, jr, s->mat_color[items[j].mat]);
                st->items++;
            }
            bp->open = 0;
        }
        if (found < 0) {
            if (nb >= MAX_BATCHES) { /* capacity guard: flush everything */
                for (int b = 0; b < nb; b++) {
                    Batch* bp = &batches[b];
                    if (!bp->open) continue;
                    int key = bp->mat * 1024 + bp->depth;
                    if (key != prev_key) { st->switches++; prev_key = key; }
                    st->draws++;
                    for (int j = bp->head; j >= 0; j = next_item[j]) {
                        Rect jr = { items[j].x0, items[j].y0,
                                    items[j].x1, items[j].y1 };
                        fb_fill(fb, jr, s->mat_color[items[j].mat]);
                        st->items++;
                    }
                    bp->open = 0;
                }
                nb = 0;
            }
            found = nb++;
            Batch* bp = &batches[found];
            bp->mat = it->mat; bp->depth = it->depth; bp->open = 1;
            bp->head = bp->tail = i;
            bp->ux0 = it->x0; bp->uy0 = it->y0;
            bp->ux1 = it->x1; bp->uy1 = it->y1;
        } else {
            Batch* bp = &batches[found];
            next_item[bp->tail] = i;
            bp->tail = i;
            if (it->x0 < bp->ux0) bp->ux0 = it->x0;
            if (it->y0 < bp->uy0) bp->uy0 = it->y0;
            if (it->x1 > bp->ux1) bp->ux1 = it->x1;
            if (it->y1 > bp->uy1) bp->uy1 = it->y1;
        }
        next_item[i] = -1;
    }
    /* drain remaining open batches in first-open order */
    for (int b = 0; b < nb; b++) {
        Batch* bp = &batches[b];
        if (!bp->open) continue;
        int key = bp->mat * 1024 + bp->depth;
        if (key != prev_key) { st->switches++; prev_key = key; }
        st->draws++;
        for (int j = bp->head; j >= 0; j = next_item[j]) {
            Rect jr = { items[j].x0, items[j].y0, items[j].x1, items[j].y1 };
            fb_fill(fb, jr, s->mat_color[items[j].mat]);
            st->items++;
        }
        bp->open = 0;
    }
}

/* ---------------- test harness ---------------- */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("  %s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* build a wide, interleaved scene: repeated mat pattern with real overlaps,
   one invisible subtree, one far-away subtree for view culling. */
static void build_main_scene(Scene* s) {
    scene_init(s);
    int root = 0;
    int g1 = node_add(s, root, 0, 0, 1);
    int g2 = node_add(s, root, 40, 0, 1);
    int g3 = node_add(s, root, 110, 78, 1);
    int ghidden = node_add(s, root, 10, 10, 0);     /* invisible subtree */
    int gfar = node_add(s, root, 4000, 4000, 1);    /* outside any view */

    for (int i = 0; i < 12; i++) {
        int a = node_add(s, g1, i * 6, i * 3, 1);
        node_set_draw(s, a, i % MAT_COUNT, i % 3,
                      (Rect){0, 0, 14, 10});
        /* overlapping neighbour of a different material: forces order
           preservation, so the batcher cannot fuse them */
        int b = node_add(s, g1, i * 6 + 5, i * 3 + 2, 1);
        node_set_draw(s, b, (i + 2) % MAT_COUNT, i % 3,
                      (Rect){0, 0, 14, 10});
    }
    for (int i = 0; i < 10; i++) {                     /* disjoint same-mat run:
                                                          should collapse */
        int a = node_add(s, g2, (i % 5) * 18, (i / 5) * 18, 1);
        node_set_draw(s, a, 1, 0, (Rect){0, 0, 12, 12});
    }
    for (int i = 0; i < 8; i++) {
        /* same state key as the disjoint g2 run, traversed after it, and
           placed in the bottom-right corner so no g1 item overlaps it:
           the batcher rejoins g2's open batch (zero extra switch), while
           the naive path pays a switch entering and leaving this run */
        int a = node_add(s, g3, 0, i * 2, 1);
        node_set_draw(s, a, 1, 0, (Rect){0, 0, 6, 2});
    }
    for (int i = 0; i < 4; i++) {
        int a = node_add(s, ghidden, i * 8, i * 8, 1);
        node_set_draw(s, a, 0, 0, (Rect){0, 0, 10, 10});
    }
    for (int i = 0; i < 4; i++) {
        int a = node_add(s, gfar, i * 8, i * 8, 1);
        node_set_draw(s, a, 4, 1, (Rect){0, 0, 10, 10});
    }

    /* A-B-A rejoin scenario, all three groups mutually disjoint. Push order
       is reversed by the stack, so draw order is gx1 -> gy -> gx2: two
       mat-A runs separated by a mat-B run. Naive pays A,B,A = 3 switches;
       the batcher keeps A open across the disjoint B run and gx2 rejoins
       it, paying only 2 switches at drain. */
    int gx2 = node_add(s, root, 0, 84, 1);
    int gy  = node_add(s, root, 70, 60, 1);
    int gx1 = node_add(s, root, 0, 60, 1);
    for (int i = 0; i < 3; i++) {
        int a = node_add(s, gx1, i * 10, 0, 1);
        node_set_draw(s, a, 4, 1, (Rect){0, 0, 8, 8});
        int b = node_add(s, gy, i * 10, 0, 1);
        node_set_draw(s, b, 5, 1, (Rect){0, 0, 8, 8});
        int c = node_add(s, gx2, i * 10, 0, 1);
        node_set_draw(s, c, 4, 1, (Rect){0, 0, 8, 8});
    }
    scene_finalize(s);
}

static unsigned fb_naive[FB_H][FB_W];
static unsigned fb_batch[FB_H][FB_W];

static void run_camera(const char* tag, const Scene* s, Rect cam) {
    static DrawItem items[MAX_ITEMS];
    TravStats tr, tn, tb;
    collect(s, cam, items, &tr);

    TravStats zero = {0};
    tn = zero; render_naive(items, tr.items, s, fb_naive, &tn);
    tb = zero; render_batched(items, tr.items, s, fb_batch, &tb);

    int mismatch = memcmp(fb_naive, fb_batch, sizeof(fb_naive)) != 0;
    printf("[%s] cam=(%d,%d,%d,%d) nodes=%d\n",
           tag, cam.x0, cam.y0, cam.x1, cam.y1, s->n);
    printf("  culled_hidden=%d culled_view=%d items=%d\n",
           tr.hidden_nodes, tr.view_nodes, tr.items);
    printf("  naive:   draw_calls=%d state_switches=%d\n", tn.draws, tn.switches);
    printf("  batched: draw_calls=%d state_switches=%d (merged=%d calls)\n",
           tb.draws, tb.switches, tn.draws - tb.draws);
    printf("  stack_hwm=%d pushes=%d pops=%d\n",
           tr.stack_hwm, tr.pushes, tr.pops);
    check("pixel_identical", !mismatch);
    /* every visible drawable is filled once per path; batching only
       collapses draw calls, never items */
    check("item_count_equal",
          tb.items == tr.items && tn.items == tr.items && tn.draws == tr.items);
    check("batch_no_worse_switches", tb.switches <= tn.switches);
    check("batch_fewer_switches",
          strcmp(tag, "full") != 0 || tb.switches < tn.switches);
    check("batch_merged_calls",
          strcmp(tag, "full") != 0 || tb.draws < tn.draws);
    check("hidden_subtree_skipped", tr.hidden_nodes >= 5);
    check("far_subtree_culled", tr.view_nodes >= 5);
}

/* deep single-child chain: depth == node count. Structural work must stay
   one push/pop per node regardless of depth (linear, no recursion). */
static double bench_chain(int depth) {
    static Scene chain;
    scene_init(&chain);
    int cur = 0;
    for (int i = 0; i < depth; i++) {
        /* zero local offset: whole chain stays inside the view, so every
           node is pushed once and the stack reaches full depth */
        int c = node_add(&chain, cur, 0, 0, 1);
        node_set_draw(&chain, c, i % MAT_COUNT, 0, (Rect){0, 0, 4, 4});
        cur = c;
    }
    scene_finalize(&chain);

    static DrawItem items[MAX_ITEMS];
    TravStats st;
    Rect cam = {0, 0, FB_W, FB_H};
    clock_t t0 = clock();
    int iters = 300;
    for (int k = 0; k < iters; k++) collect(&chain, cam, items, &st);
    clock_t t1 = clock();
    double us = (double)(t1 - t0) * 1e6 / CLOCKS_PER_SEC / iters;
    printf("  depth=%5d nodes=%5d items=%3d pushes=%5d pops=%5d "
           "stack_hwm=%5d  %.1f us/frame\n",
           depth, chain.n, st.items, st.pushes, st.pops, st.stack_hwm, us);
    check("chain_push_pop_linear", st.pushes == chain.n && st.pops == chain.n);
    /* single-child chains pop before pushing the only child, so explicit
       stack depth stays O(1) no matter how deep: no recursion, no overflow */
    check("chain_stack_bounded", st.stack_hwm <= 2);
    return us;
}

/* randomized fuzz: random attachment, state keys, visibility, overlap and
   camera per seed. Both paths must remain byte-identical every frame. */
static unsigned lcg_state;
static unsigned rnd(void) {
    lcg_state = lcg_state * 1103515245u + 12345u;
    return lcg_state >> 8;
}

static void fuzz_pixels(void) {
    printf("== randomized fuzz ==\n");
    static Scene fs;
    static DrawItem fitems[MAX_ITEMS];
    static unsigned fna[FB_H][FB_W], fba[FB_H][FB_W];
    int frames = 0, total_items = 0, total_saved = 0;
    int fuzz_fail = 0;
    for (unsigned seed = 1; seed <= 200; seed++) {
        lcg_state = seed * 2654435761u + 17u;
        scene_init(&fs);
        int nn = 40 + (int)(rnd() % 120);
        for (int i = 1; i < nn && fs.n < MAX_NODES; i++) {
            int parent = (int)(rnd() % (unsigned)i);
            int lx = (int)(rnd() % 60) - 10;
            int ly = (int)(rnd() % 60) - 10;
            int vis = (rnd() % 100) < 85;
            int id = node_add(&fs, parent, lx, ly, vis);
            if (id < 0) continue;
            if (rnd() % 100 < 75) {
                int mat = (int)(rnd() % MAT_COUNT);
                int depth = (int)(rnd() % 3);
                int w = 2 + (int)(rnd() % 18);
                int h = 2 + (int)(rnd() % 14);
                node_set_draw(&fs, id, mat, depth, (Rect){0, 0, w, h});
            }
        }
        scene_finalize(&fs);
        int cx0 = (int)(rnd() % 80) - 20;
        int cy0 = (int)(rnd() % 60) - 15;
        int cw = 20 + (int)(rnd() % (FB_W + 40));
        int ch = 15 + (int)(rnd() % (FB_H + 30));
        Rect cam = {cx0, cy0, cx0 + cw, cy0 + ch};

        TravStats tr, tn, tb;
        collect(&fs, cam, fitems, &tr);
        TravStats zero = {0};
        tn = zero; render_naive(fitems, tr.items, &fs, fna, &tn);
        tb = zero; render_batched(fitems, tr.items, &fs, fba, &tb);
        if (memcmp(fna, fba, sizeof(fna)) != 0) {
            printf("  fuzz_mismatch seed=%u\n", seed);
            fuzz_fail++; failures++;
        }
        if (tb.items != tr.items || tn.draws != tr.items) {
            printf("  fuzz_item_count seed=%u tr=%d tn=%d tb=%d\n",
                   seed, tr.items, tn.draws, tb.items);
            fuzz_fail++; failures++;
        }
        frames++;
        total_items += tr.items;
        total_saved += tn.switches - tb.switches;
    }
    printf("  frames=%d items=%d switches_saved=%d\n",
           frames, total_items, total_saved);
    check("fuzz_pixel_identical", fuzz_fail == 0);
    check("fuzz_batcher_never_worse", total_saved >= 0);
}

int main(void) {
    Scene s;
    build_main_scene(&s);

    printf("== full view ==\n");
    run_camera("full", &s, (Rect){0, 0, FB_W, FB_H});

    printf("== narrowed view (partial cull) ==\n");
    run_camera("narrow", &s, (Rect){30, 10, 90, 70});

    fuzz_pixels();

    printf("== deep-chain linearity ==\n");
    double t500 = bench_chain(500);
    double t1000 = bench_chain(1000);
    double t2000 = bench_chain(2000);
    double ratio = t2000 / t500;
    double ratio1k = t1000 / t500;
    printf("  time_ratio 1000/500 = %.2f, 2000/500 = %.2f (node ratios 2/4)\n",
           ratio1k, ratio);
    printf("  linear_timing=%d\n", ratio < 6.0 ? 1 : 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
