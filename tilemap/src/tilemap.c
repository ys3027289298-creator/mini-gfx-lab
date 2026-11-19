/* tilemap: layered tile map with 4-neighbour autotiling, per-layer culling,
   and an undoable edit log that can be persisted and replayed. */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 32
#define H 32
#define LAYERS 3
#define UNDO_CAP 8              /* bounded undo/redo stacks, oldest is dropped */
#define LOG_MAGIC "tilemap-log"
#define LOG_VERSION 1

typedef enum {
    ED_ERR_IO = -7,
    ED_ERR_NOMEM = -6,
    ED_ERR_REPLAY = -5,         /* malformed log or state mismatch */
    ED_ERR_STATE = -4,          /* stack and map disagree */
    ED_ERR_REDO_EMPTY = -3,
    ED_ERR_UNDO_EMPTY = -2,
    ED_ERR_RANGE = -1,          /* layer / x / y out of bounds */
    ED_OK = 0,
    ED_NO_CHANGE = 1            /* valid write that changed nothing */
} EdStatus;

static const char* ed_status_str(EdStatus s) {
    switch (s) {
    case ED_ERR_IO: return "io";
    case ED_ERR_NOMEM: return "nomem";
    case ED_ERR_REPLAY: return "replay";
    case ED_ERR_STATE: return "state";
    case ED_ERR_REDO_EMPTY: return "redo_empty";
    case ED_ERR_UNDO_EMPTY: return "undo_empty";
    case ED_ERR_RANGE: return "range";
    case ED_OK: return "ok";
    case ED_NO_CHANGE: return "no_change";
    }
    return "unknown";
}

typedef struct {
    unsigned char base[LAYERS][H][W];   /* 0 = empty, else tile id */
    int layer_visible[LAYERS];
} TileMap;

static int cell_in_bounds(int layer, int x, int y) {
    return layer >= 0 && layer < LAYERS && x >= 0 && x < W && y >= 0 && y < H;
}

static void map_init(TileMap* m) {
    memset(m, 0, sizeof(*m));
    for (int l = 0; l < LAYERS; l++) m->layer_visible[l] = 1;
}

/* Out-of-bounds writes report an error and leave the map untouched. */
static EdStatus map_set(TileMap* m, int layer, int x, int y, unsigned char id) {
    if (!cell_in_bounds(layer, x, y)) return ED_ERR_RANGE;
    m->base[layer][y][x] = id;
    return ED_OK;
}

static unsigned char map_get(const TileMap* m, int layer, int x, int y) {
    if (!cell_in_bounds(layer, x, y)) return 0;
    return m->base[layer][y][x];
}

static int map_equal(const TileMap* a, const TileMap* b) {
    return memcmp(a->base, b->base, sizeof(a->base)) == 0;
}

/* 4-bit mask: 1 = neighbour filled (N,E,S,W) */
static int autotile_mask(const TileMap* m, int layer, int x, int y) {
    int mask = 0;
    if (map_get(m, layer, x, y - 1)) mask |= 1;
    if (map_get(m, layer, x + 1, y)) mask |= 2;
    if (map_get(m, layer, x, y + 1)) mask |= 4;
    if (map_get(m, layer, x - 1, y)) mask |= 8;
    return mask;
}

/* autotile variant id for a filled cell: mask plus a base offset */
static int autotile_variant(const TileMap* m, int layer, int x, int y) {
    if (!map_get(m, layer, x, y)) return -1;
    return autotile_mask(m, layer, x, y);
}

/* count visible tiles inside a rect across visible layers */
static int cull_count(const TileMap* m, int x0, int y0, int x1, int y1) {
    int n = 0;
    for (int l = 0; l < LAYERS; l++) {
        if (!m->layer_visible[l]) continue;
        for (int y = y0; y <= y1; y++)
            for (int x = x0; x <= x1; x++)
                if (map_get(m, l, x, y)) n++;
    }
    return n;
}

/* ---- command log ------------------------------------------------------- */

/* 'e' = edit, 'u' = undo restored a value, 'r' = redo re-applied one.
   Undo/redo are journalled too, so replaying the whole log after any mix of
   operations reproduces the live map exactly. */
typedef struct {
    char kind;
    unsigned char layer, x, y;
    unsigned char old_id, new_id;
} Command;

typedef struct {
    TileMap map;
    Command undo[UNDO_CAP];
    int undo_head, undo_count;
    Command redo[UNDO_CAP];
    int redo_head, redo_count;
    Command* journal;           /* append-only, includes undo/redo effects */
    int journal_len, journal_cap;
} Editor;

static void editor_init(Editor* ed) {
    map_init(&ed->map);
    ed->undo_head = ed->undo_count = 0;
    ed->redo_head = ed->redo_count = 0;
    ed->journal = NULL;
    ed->journal_len = ed->journal_cap = 0;
}

static void editor_destroy(Editor* ed) {
    free(ed->journal);
    ed->journal = NULL;
    ed->journal_len = ed->journal_cap = 0;
}

static EdStatus journal_append(Editor* ed, Command c) {
    if (ed->journal_len == ed->journal_cap) {
        int cap = ed->journal_cap ? ed->journal_cap * 2 : 32;
        Command* p = realloc(ed->journal, (size_t)cap * sizeof(*p));
        if (!p) return ED_ERR_NOMEM;
        ed->journal = p;
        ed->journal_cap = cap;
    }
    ed->journal[ed->journal_len++] = c;
    return ED_OK;
}

/* Bounded ring push: when full, overwrite the oldest entry. */
static void stack_push(Command* buf, int* head, int* count, Command c) {
    if (*count == UNDO_CAP) {
        buf[*head] = c;                 /* oldest slot is the tail */
        *head = (*head + 1) % UNDO_CAP;
    } else {
        buf[(*head + *count) % UNDO_CAP] = c;
        (*count)++;
    }
}

static Command stack_top(const Command* buf, int head, int count) {
    return buf[(head + count - 1) % UNDO_CAP];
}

/* Validate, journal, then mutate. The old value is returned so the caller
   can build the matching undo entry. Journal append happens before the map
   write, so a failed allocation leaves both map and journal unchanged. */
static EdStatus apply_change(Editor* ed, char kind, int layer, int x, int y,
                             unsigned char new_id, unsigned char* old_out) {
    if (!cell_in_bounds(layer, x, y)) return ED_ERR_RANGE;

    unsigned char old = ed->map.base[layer][y][x];
    if (old_out) *old_out = old;
    if (old == new_id) return ED_NO_CHANGE;

    Command c = { kind, (unsigned char)layer, (unsigned char)x,
                  (unsigned char)y, old, new_id };
    EdStatus st = journal_append(ed, c);
    if (st != ED_OK) return st;

    ed->map.base[layer][y][x] = new_id;
    return ED_OK;
}

/* Edit a cell and push it onto the undo stack. A new edit invalidates redo. */
static EdStatus editor_set(Editor* ed, int layer, int x, int y,
                           unsigned char id) {
    unsigned char old = 0;
    EdStatus st = apply_change(ed, 'e', layer, x, y, id, &old);
    if (st != ED_OK) return st;

    Command c = { 'e', (unsigned char)layer, (unsigned char)x,
                  (unsigned char)y, old, id };
    stack_push(ed->undo, &ed->undo_head, &ed->undo_count, c);
    ed->redo_head = 0;
    ed->redo_count = 0;
    return ED_OK;
}

static EdStatus editor_undo(Editor* ed) {
    if (ed->undo_count == 0) return ED_ERR_UNDO_EMPTY;

    Command c = stack_top(ed->undo, ed->undo_head, ed->undo_count);
    if (map_get(&ed->map, c.layer, c.x, c.y) != c.new_id)
        return ED_ERR_STATE;

    unsigned char old = 0;
    EdStatus st = apply_change(ed, 'u', c.layer, c.x, c.y, c.old_id, &old);
    if (st != ED_OK) return st == ED_NO_CHANGE ? ED_ERR_STATE : st;

    ed->undo_count--;
    stack_push(ed->redo, &ed->redo_head, &ed->redo_count, c);
    return ED_OK;
}

static EdStatus editor_redo(Editor* ed) {
    if (ed->redo_count == 0) return ED_ERR_REDO_EMPTY;

    Command c = stack_top(ed->redo, ed->redo_head, ed->redo_count);
    if (map_get(&ed->map, c.layer, c.x, c.y) != c.old_id)
        return ED_ERR_STATE;

    unsigned char old = 0;
    EdStatus st = apply_change(ed, 'r', c.layer, c.x, c.y, c.new_id, &old);
    if (st != ED_OK) return st == ED_NO_CHANGE ? ED_ERR_STATE : st;

    ed->redo_count--;
    stack_push(ed->undo, &ed->undo_head, &ed->undo_count, c);
    return ED_OK;
}

/* ---- persistence ------------------------------------------------------- */

static uint64_t map_checksum(const TileMap* m) {
    /* FNV-1a over the grid only, so visibility flags do not affect replay. */
    uint64_t h = UINT64_C(14695981039346656037);
    const unsigned char* p = (const unsigned char*)m->base;
    for (size_t i = 0; i < sizeof(m->base); i++) {
        h ^= p[i];
        h *= UINT64_C(1099511628211);
    }
    return h;
}

static EdStatus editor_save_log(const Editor* ed, const char* path) {
    FILE* f = fopen(path, "w");
    if (!f) return ED_ERR_IO;

    int ok = fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H,
                     LAYERS) > 0;
    for (int i = 0; ok && i < ed->journal_len; i++) {
        const Command* c = &ed->journal[i];
        ok = fprintf(f, "%c %d %d %d %u %u\n", c->kind, c->layer, c->x, c->y,
                     (unsigned)c->old_id, (unsigned)c->new_id) > 0;
    }
    if (ok)
        ok = fprintf(f, "sum %016llx\n",
                     (unsigned long long)map_checksum(&ed->map)) > 0;
    if (fclose(f) != 0) ok = 0;
    return ok ? ED_OK : ED_ERR_IO;
}

static int line_is_blank(const char* s) {
    for (; *s; s++)
        if (*s != ' ' && *s != '\t' && *s != '\r' && *s != '\n') return 0;
    return 1;
}

/* Replay into a fresh map. Every record must name an in-range cell whose
   current value equals the recorded old value; a trailing checksum must match
   the reconstructed grid. Any deviation is ED_ERR_REPLAY. */
static EdStatus replay_file(const char* path, TileMap* out, int* applied) {
    FILE* f = fopen(path, "r");
    if (!f) return ED_ERR_IO;

    if (applied) *applied = 0;
    map_init(out);

    char line[512];
    char magic[32];
    int ver = 0, w = 0, h = 0, layers = 0;
    if (!fgets(line, sizeof(line), f) ||
        sscanf(line, "%31s %d %d %d %d", magic, &ver, &w, &h, &layers) != 5 ||
        strcmp(magic, LOG_MAGIC) != 0 || ver != LOG_VERSION ||
        w != W || h != H || layers != LAYERS) {
        fclose(f);
        return ED_ERR_REPLAY;
    }

    int saw_sum = 0;
    uint64_t want_sum = 0;
    EdStatus st = ED_OK;
    while (fgets(line, sizeof(line), f)) {
        if (line_is_blank(line)) continue;

        if (strncmp(line, "sum ", 4) == 0) {
            unsigned long long got = 0;
            int used = 0;
            const char* rest = line;
            if (sscanf(line, "sum %llx%n", &got, &used) != 1 ||
                !line_is_blank(rest + used)) {
                st = ED_ERR_REPLAY;
                break;
            }
            want_sum = (uint64_t)got;
            saw_sum = 1;
            break;                          /* nothing may follow the checksum */
        }

        char kind = 0;
        int l = 0, x = 0, y = 0, old_id = 0, new_id = 0;
        if (sscanf(line, " %c %d %d %d %d %d", &kind, &l, &x, &y, &old_id,
                   &new_id) != 6 ||
            (kind != 'e' && kind != 'u' && kind != 'r') ||
            !cell_in_bounds(l, x, y) || old_id < 0 || old_id > 255 ||
            new_id < 0 || new_id > 255 ||
            map_get(out, l, x, y) != (unsigned char)old_id) {
            st = ED_ERR_REPLAY;
            break;
        }
        map_set(out, l, x, y, (unsigned char)new_id);
        if (applied) (*applied)++;
    }

    if (st == ED_OK) {
        /* Only blank lines may remain after the checksum. */
        while (fgets(line, sizeof(line), f))
            if (!line_is_blank(line)) { st = ED_ERR_REPLAY; break; }
    }
    if (st == ED_OK &&
        (!saw_sum || map_checksum(out) != want_sum))
        st = ED_ERR_REPLAY;
    if (ferror(f) && st == ED_OK) st = ED_ERR_IO;

    fclose(f);
    return st;
}

/* ---- checks ------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int check_status(const char* name, EdStatus got, EdStatus want) {
    int ok = got == want;
    printf("%s=%d", name, ok ? 1 : 0);
    if (!ok) printf(" (got %s, want %s)", ed_status_str(got),
                    ed_status_str(want));
    printf("\n");
    if (!ok) failures++;
    return ok;
}

static int editor_count_undo(const Editor* ed) { return ed->undo_count; }
static int editor_count_journal(const Editor* ed) { return ed->journal_len; }

static void test_out_of_range(void) {
    Editor ed;
    editor_init(&ed);

    check_status("oob_x_neg", editor_set(&ed, 0, -1, 0, 1), ED_ERR_RANGE);
    check_status("oob_x_high", editor_set(&ed, 0, W, 0, 1), ED_ERR_RANGE);
    check_status("oob_y_neg", editor_set(&ed, 0, 0, -1, 1), ED_ERR_RANGE);
    check_status("oob_y_high", editor_set(&ed, 0, 0, H, 1), ED_ERR_RANGE);
    check_status("oob_layer_neg", editor_set(&ed, -1, 0, 0, 1), ED_ERR_RANGE);
    check_status("oob_layer_high", editor_set(&ed, LAYERS, 0, 0, 1), ED_ERR_RANGE);

    check("oob_no_write", map_get(&ed.map, 0, 0, 0) == 0);
    check("oob_no_journal", editor_count_journal(&ed) == 0);
    check("oob_no_undo", editor_count_undo(&ed) == 0);

    check_status("undo_empty", editor_undo(&ed), ED_ERR_UNDO_EMPTY);
    check_status("redo_empty", editor_redo(&ed), ED_ERR_REDO_EMPTY);

    editor_destroy(&ed);
}

static void test_undo_redo(void) {
    Editor ed;
    editor_init(&ed);

    check_status("edit_ok", editor_set(&ed, 0, 10, 10, 3), ED_OK);
    check("edit_applied", map_get(&ed.map, 0, 10, 10) == 3);

    check_status("undo_ok", editor_undo(&ed), ED_OK);
    check("undo_restores", map_get(&ed.map, 0, 10, 10) == 0);
    check_status("undo_empty_after", editor_undo(&ed), ED_ERR_UNDO_EMPTY);

    check_status("redo_ok", editor_redo(&ed), ED_OK);
    check("redo_reapplies", map_get(&ed.map, 0, 10, 10) == 3);
    check_status("redo_empty_after", editor_redo(&ed), ED_ERR_REDO_EMPTY);

    /* Same value is not a change: no undo entry, no journal entry. */
    int undo_before = editor_count_undo(&ed);
    int journal_before = editor_count_journal(&ed);
    check_status("noop_status", editor_set(&ed, 0, 10, 10, 3), ED_NO_CHANGE);
    check("noop_no_undo", editor_count_undo(&ed) == undo_before);
    check("noop_no_journal", editor_count_journal(&ed) == journal_before);

    /* A new edit after undo drops the redo branch. */
    check_status("redo_prep", editor_undo(&ed), ED_OK);
    check_status("new_edit", editor_set(&ed, 0, 11, 10, 4), ED_OK);
    check_status("redo_cleared", editor_redo(&ed), ED_ERR_REDO_EMPTY);

    /* Undo/redo are themselves journalled, so replay still reaches the
       live state even after mixed history. */
    check("journal_counts_all_mutations", editor_count_journal(&ed) == 5);

    editor_destroy(&ed);
}

static void test_bounded_undo(void) {
    Editor ed;
    editor_init(&ed);

    /* CAP + 3 edits: the oldest three undo entries must be evicted. */
    const int total = UNDO_CAP + 3;
    for (int i = 0; i < total; i++)
        check_status("bounded_edit", editor_set(&ed, 0, i, 0, 1), ED_OK);
    check("bounded_undo_depth", editor_count_undo(&ed) == UNDO_CAP);

    for (int i = 0; i < UNDO_CAP; i++)
        check_status("bounded_undo", editor_undo(&ed), ED_OK);
    check_status("bounded_undo_stops", editor_undo(&ed), ED_ERR_UNDO_EMPTY);

    /* Cells 0..2 still hold their evicted edits; 3..CAP are undone. */
    int kept = 0, undone = 0;
    for (int i = 0; i < total; i++) {
        if (map_get(&ed.map, 0, i, 0) == 1) kept++;
        else undone++;
    }
    check("bounded_keeps_oldest_edits", kept == 3);
    check("bounded_undid_newest", undone == UNDO_CAP);

    editor_destroy(&ed);
}

static void test_persistence(void) {
    const char* path = "tilemap_log.tmp";
    Editor ed;
    editor_init(&ed);

    /* A mixed history: edits, undo, redo, a no-op, an out-of-range failure. */
    editor_set(&ed, 0, 3, 4, 7);
    editor_set(&ed, 1, 3, 4, 9);
    editor_set(&ed, 2, 31, 31, 5);
    editor_undo(&ed);
    editor_redo(&ed);
    editor_set(&ed, 0, 4, 4, 7);
    editor_set(&ed, 0, 4, 4, 7);            /* no-op */
    editor_set(&ed, 0, -1, 0, 1);           /* range error */
    editor_set(&ed, 0, 20, 20, 255);

    check_status("save_ok", editor_save_log(&ed, path), ED_OK);

    TileMap replayed;
    int applied = -1;
    check_status("replay_ok", replay_file(path, &replayed, &applied), ED_OK);
    check("replay_map_equal", map_equal(&ed.map, &replayed));
    check("replay_all_records", applied == editor_count_journal(&ed));
    check("replay_layer_visible_default",
          replayed.layer_visible[0] == 1 && replayed.layer_visible[1] == 1 &&
          replayed.layer_visible[2] == 1);

    /* Missing file is an I/O error, not a replay error. */
    check_status("replay_missing", replay_file("tilemap_no_such.tmp", &replayed,
                                               NULL), ED_ERR_IO);

    remove(path);
    editor_destroy(&ed);
}

static void test_replay_rejects_corruption(void) {
    const char* path = "tilemap_bad.tmp";
    FILE* f = fopen(path, "w");
    check("bad_write", f != NULL);
    if (!f) return;

    /* Second record claims an old value that was never there. */
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H, LAYERS);
    fprintf(f, "e 0 1 1 0 4\n");
    fprintf(f, "e 0 1 1 1 5\n");
    fprintf(f, "sum %016llx\n", 0ULL);
    fclose(f);

    TileMap out;
    check_status("replay_bad_old", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);

    /* Out-of-range record is rejected before touching the map. */
    f = fopen(path, "w");
    check("range_write", f != NULL);
    if (!f) return;
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H, LAYERS);
    fprintf(f, "e 0 %d 0 0 1\n", W);
    fprintf(f, "sum %016llx\n", 0ULL);
    fclose(f);
    check_status("replay_bad_range", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);

    /* Wrong dimensions in the header are rejected. */
    f = fopen(path, "w");
    check("hdr_write", f != NULL);
    if (!f) return;
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W + 1, H, LAYERS);
    fclose(f);
    check_status("replay_bad_header", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);

    /* A structurally valid log with the wrong checksum is rejected. */
    f = fopen(path, "w");
    check("sum_write", f != NULL);
    if (!f) return;
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H, LAYERS);
    fprintf(f, "e 0 1 1 0 4\n");
    fprintf(f, "sum %016llx\n", 1ULL);
    fclose(f);
    check_status("replay_bad_checksum", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);

    /* Unknown record kinds and trailing content are rejected. */
    f = fopen(path, "w");
    check("kind_write", f != NULL);
    if (!f) return;
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H, LAYERS);
    fprintf(f, "x 0 1 1 0 4\n");
    fclose(f);
    check_status("replay_bad_kind", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);

    f = fopen(path, "w");
    check("trail_write", f != NULL);
    if (!f) return;
    fprintf(f, "%s %d %d %d %d\n", LOG_MAGIC, LOG_VERSION, W, H, LAYERS);
    fprintf(f, "sum %016llx\n", 0ULL);
    fprintf(f, "e 0 1 1 0 4\n");
    fclose(f);
    check_status("replay_trailing_data", replay_file(path, &out, NULL),
                 ED_ERR_REPLAY);
    remove(path);
}

int main(void) {
    TileMap m;
    map_init(&m);
    for (int x = 5; x < 10; x++) map_set(&m, 0, x, 5, 1);
    map_set(&m, 1, 5, 5, 2);

    check("single_mask", autotile_mask(&m, 0, 5, 5) == 2);          /* only E */
    check("middle_mask", autotile_mask(&m, 0, 7, 5) == (2 | 8));    /* E and W */
    check("end_mask", autotile_mask(&m, 0, 9, 5) == 8);
    check("isolated_mask", autotile_mask(&m, 0, 20, 20) == 0);
    check("empty_variant", autotile_variant(&m, 0, 20, 20) == -1);
    check("filled_variant", autotile_variant(&m, 0, 7, 5) == (2 | 8));

    check("cull_all", cull_count(&m, 0, 0, W - 1, H - 1) == 6);
    check("cull_sub", cull_count(&m, 5, 5, 9, 5) == 6);
    check("cull_narrow", cull_count(&m, 6, 5, 8, 5) == 3);

    m.layer_visible[0] = 0;
    check("cull_hidden_layer", cull_count(&m, 0, 0, W - 1, H - 1) == 1);
    m.layer_visible[0] = 1;

    check("out_of_range_set",
          (map_set(&m, 0, -1, 0, 1), map_get(&m, 0, -1, 0)) == 0);
    check("bad_layer", map_get(&m, 9, 0, 0) == 0);

    test_out_of_range();
    test_undo_redo();
    test_bounded_undo();
    test_persistence();
    test_replay_rejects_corruption();

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
