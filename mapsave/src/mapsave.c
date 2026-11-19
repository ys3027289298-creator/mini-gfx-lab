/* mapsave: versioned, checksummed run-length save format for layered tile maps.

   File layout (all integers little-endian):

     offset  size  field
     0       4     magic "MGMS"
     4       1     major version (readers require 1)
     5       1     minor version (unknown minors are accepted)
     6       1     flags (reserved; readers ignore bits they do not know)
     7       1     tile bits: 8 or 16
     8       2     layer count
     10      2     width
     12      2     height
     14      2     reserved, written as 0
     16      4     header checksum: FNV-1a over bytes 0..15
     20      ...   body: for each layer, u32 layer checksum followed by runs
     n-4     4     file checksum: FNV-1a over bytes 0..n-5

   Run records, repeated until width*height cells are produced:

     tag 0x00        literal run: u16 count, then count raw cells
     tag 0x01..0xFE  repeat run: count = tag, then one cell value
     tag 0xFF        extended repeat: u16 count, then one cell value

   A cell is one byte at 8 tile bits and two bytes at 16.  A repeat record
   stores its value once, so empty and solid regions cost a handful of bytes.
   A literal record copies cells verbatim, so noisy data grows only by the
   framing (3 bytes per 65535 cells).  All layers in a v1 file share the
   width/height from the header.

   Integrity: the header checksum catches damaged metadata (reported at the
   offending field when its value is impossible, else at 16), each layer
   checksum covers that layer's decoded cells (reported at the layer record),
   and the file checksum covers everything (reported at n-4).  A file is only
   returned once every check has passed, so a damaged file never produces a
   partially decoded map.

   Compatibility: a reader accepts any minor version of major 1, because
   minor revisions may only add self-delimiting data that v1 readers can
   ignore (new flags, for example).  Unknown flag bits are ignored.  A
   structural change must bump the major version; readers then refuse the
   file with an explicit message instead of misparsing it. */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#define SAVE_MAGIC "MGMS"
#define SAVE_MAJOR 1
#define SAVE_MINOR 0
#define HEADER_SIZE 20
#define TRAILER_SIZE 4
#define MAX_LAYERS 1024
#define MAX_CELLS (64ull * 1024ull * 1024ull)   /* across all layers */

#define RUN_LITERAL 0x00
#define RUN_EXTENDED 0xFF
#define RUN_MAX 0xFFFF

/* ------------------------------------------------------------------ */
/* map model: 16-bit tile ids, v1 layers all share one size            */
/* ------------------------------------------------------------------ */

typedef struct {
    int w, h;
    uint16_t* cells;
} Layer;

typedef struct {
    int layer_count;
    Layer* layers;
} SaveMap;

static SaveMap* map_new(int layer_count, int w, int h) {
    if (layer_count <= 0 || layer_count > MAX_LAYERS || w <= 0 || h <= 0) return NULL;
    if ((unsigned long long)w * (unsigned long long)h *
        (unsigned long long)layer_count > MAX_CELLS) return NULL;
    SaveMap* m = (SaveMap*)calloc(1, sizeof(*m));
    if (!m) return NULL;
    m->layer_count = layer_count;
    m->layers = (Layer*)calloc((size_t)layer_count, sizeof(Layer));
    if (!m->layers) { free(m); return NULL; }
    for (int l = 0; l < layer_count; l++) {
        m->layers[l].w = w;
        m->layers[l].h = h;
        m->layers[l].cells = (uint16_t*)calloc((size_t)w * (size_t)h, sizeof(uint16_t));
        if (!m->layers[l].cells) {
            for (int j = 0; j < l; j++) free(m->layers[j].cells);
            free(m->layers); free(m);
            return NULL;
        }
    }
    return m;
}

static void map_free(SaveMap* m) {
    if (!m) return;
    for (int l = 0; l < m->layer_count; l++) free(m->layers[l].cells);
    free(m->layers);
    free(m);
}

static void map_set(SaveMap* m, int l, int x, int y, uint16_t v) {
    if (!m || l < 0 || l >= m->layer_count) return;
    Layer* L = &m->layers[l];
    if (x < 0 || y < 0 || x >= L->w || y >= L->h) return;
    L->cells[(size_t)y * (size_t)L->w + (size_t)x] = v;
}

static uint16_t map_get(const SaveMap* m, int l, int x, int y) {
    if (!m || l < 0 || l >= m->layer_count) return 0;
    const Layer* L = &m->layers[l];
    if (x < 0 || y < 0 || x >= L->w || y >= L->h) return 0;
    return L->cells[(size_t)y * (size_t)L->w + (size_t)x];
}

static int map_equal(const SaveMap* a, const SaveMap* b) {
    if (!a || !b || a->layer_count != b->layer_count) return 0;
    for (int l = 0; l < a->layer_count; l++) {
        if (a->layers[l].w != b->layers[l].w || a->layers[l].h != b->layers[l].h) return 0;
        size_t n = (size_t)a->layers[l].w * (size_t)a->layers[l].h;
        if (memcmp(a->layers[l].cells, b->layers[l].cells, n * sizeof(uint16_t)) != 0) return 0;
    }
    return 1;
}

/* ------------------------------------------------------------------ */
/* checksums                                                           */
/* ------------------------------------------------------------------ */

#define FNV_SEED 2166136261u

static uint32_t fnv1a(const void* data, size_t n, uint32_t seed) {
    const unsigned char* p = (const unsigned char*)data;
    uint32_t h = seed;
    for (size_t i = 0; i < n; i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

/* Hash cells in canonical little-endian order, independent of the on-disk
   tile width, so changing the encoding does not change a layer's checksum. */
static uint32_t cells_checksum(const uint16_t* cells, size_t n) {
    uint32_t h = FNV_SEED;
    for (size_t i = 0; i < n; i++) {
        h ^= (uint32_t)(cells[i] & 0xFFu);
        h *= 16777619u;
        h ^= (uint32_t)(cells[i] >> 8);
        h *= 16777619u;
    }
    return h;
}

/* ------------------------------------------------------------------ */
/* writer                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    unsigned char* buf;
    size_t len, cap;
    int oom;
} Buf;

static int buf_put(Buf* b, const void* src, size_t n) {
    if (b->oom) return -1;
    if (b->len + n > b->cap) {
        size_t nc = b->cap ? b->cap : 256;
        while (nc < b->len + n) nc *= 2;
        unsigned char* p = (unsigned char*)realloc(b->buf, nc);
        if (!p) { b->oom = 1; return -1; }
        b->buf = p;
        b->cap = nc;
    }
    memcpy(b->buf + b->len, src, n);
    b->len += n;
    return 0;
}

static int buf_u8(Buf* b, unsigned v) {
    unsigned char c = (unsigned char)(v & 0xFFu);
    return buf_put(b, &c, 1);
}

static int buf_u16(Buf* b, unsigned v) {
    unsigned char c[2] = { (unsigned char)(v & 0xFFu), (unsigned char)((v >> 8) & 0xFFu) };
    return buf_put(b, c, 2);
}

static int buf_u32(Buf* b, uint32_t v) {
    unsigned char c[4] = {
        (unsigned char)(v & 0xFFu), (unsigned char)((v >> 8) & 0xFFu),
        (unsigned char)((v >> 16) & 0xFFu), (unsigned char)((v >> 24) & 0xFFu)
    };
    return buf_put(b, c, 4);
}

static void cell_u16(unsigned char out[2], unsigned v) {
    out[0] = (unsigned char)(v & 0xFFu);
    out[1] = (unsigned char)((v >> 8) & 0xFFu);
}

static uint16_t rd_u16(const unsigned char* p) {
    return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd_u32(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int choose_tile_bits(const SaveMap* m) {
    for (int l = 0; l < m->layer_count; l++) {
        size_t n = (size_t)m->layers[l].w * (size_t)m->layers[l].h;
        for (size_t i = 0; i < n; i++)
            if (m->layers[l].cells[i] > 0xFFu) return 16;
    }
    return 8;
}

static int emit_value(Buf* b, uint16_t v, int bpc) {
    if (bpc == 1) return buf_u8(b, v);
    unsigned char c[2];
    cell_u16(c, v);
    return buf_put(b, c, 2);
}

static int emit_repeat(Buf* b, uint16_t v, size_t run, int bpc) {
    while (run > 0) {
        size_t chunk = run > RUN_MAX ? RUN_MAX : run;
        if (chunk <= 0xFEu) {
            if (buf_u8(b, (unsigned)chunk) != 0) return -1;
        } else {
            if (buf_u8(b, RUN_EXTENDED) != 0) return -1;
            if (buf_u16(b, (unsigned)chunk) != 0) return -1;
        }
        if (emit_value(b, v, bpc) != 0) return -1;
        run -= chunk;
    }
    return 0;
}

static int emit_literal(Buf* b, const uint16_t* cells, size_t start, size_t count, int bpc) {
    while (count > 0) {
        size_t chunk = count > RUN_MAX ? RUN_MAX : count;
        if (buf_u8(b, RUN_LITERAL) != 0) return -1;
        if (buf_u16(b, (unsigned)chunk) != 0) return -1;
        for (size_t k = 0; k < chunk; k++) {
            if (emit_value(b, cells[start + k], bpc) != 0) return -1;
        }
        start += chunk;
        count -= chunk;
    }
    return 0;
}

static int write_layer(Buf* b, const Layer* L, int bpc) {
    size_t total = (size_t)L->w * (size_t)L->h;
    size_t i = 0, lit_start = 0, lit_len = 0;
    while (i < total) {
        size_t j = i + 1;
        while (j < total && L->cells[j] == L->cells[i]) j++;
        size_t run = j - i;
        if (run >= 2) {
            if (lit_len > 0) {
                if (emit_literal(b, L->cells, lit_start, lit_len, bpc) != 0) return -1;
                lit_len = 0;
            }
            if (emit_repeat(b, L->cells[i], run, bpc) != 0) return -1;
            i = j;
        } else {
            if (lit_len == 0) lit_start = i;
            lit_len++;
            i++;
            if (lit_len == RUN_MAX) {
                if (emit_literal(b, L->cells, lit_start, lit_len, bpc) != 0) return -1;
                lit_len = 0;
            }
        }
    }
    if (lit_len > 0 && emit_literal(b, L->cells, lit_start, lit_len, bpc) != 0) return -1;
    return 0;
}

static int validate_map_for_save(const SaveMap* m) {
    if (!m || m->layer_count <= 0 || m->layer_count > MAX_LAYERS) return -2;
    if (!m->layers) return -2;
    int w = m->layers[0].w, h = m->layers[0].h;
    if (w <= 0 || h <= 0 || w > 0xFFFF || h > 0xFFFF) return -2;
    if ((unsigned long long)w * (unsigned long long)h *
        (unsigned long long)m->layer_count > MAX_CELLS) return -2;
    for (int l = 0; l < m->layer_count; l++) {
        if (!m->layers[l].cells) return -2;
        if (m->layers[l].w != w || m->layers[l].h != h) return -2;   /* v1 is uniform */
    }
    return 0;
}

/* Serialize into a caller-owned malloc buffer.  On success *out_buf receives
   the bytes and *out_len the count; the caller frees *out_buf. */
static int save_to_bytes(const SaveMap* m, unsigned char** out_buf, size_t* out_len) {
    if (!out_buf || !out_len) return -1;
    int rc = validate_map_for_save(m);
    if (rc != 0) return rc;

    int bits = choose_tile_bits(m);
    int bpc = bits / 8;
    int w = m->layers[0].w, h = m->layers[0].h;

    Buf b = {0};
    if (buf_put(&b, SAVE_MAGIC, 4) != 0) goto fail;
    if (buf_u8(&b, SAVE_MAJOR) != 0) goto fail;
    if (buf_u8(&b, SAVE_MINOR) != 0) goto fail;
    if (buf_u8(&b, 0) != 0) goto fail;             /* flags */
    if (buf_u8(&b, (unsigned)bits) != 0) goto fail;
    if (buf_u16(&b, (unsigned)m->layer_count) != 0) goto fail;
    if (buf_u16(&b, (unsigned)w) != 0) goto fail;
    if (buf_u16(&b, (unsigned)h) != 0) goto fail;
    if (buf_u16(&b, 0) != 0) goto fail;             /* reserved */
    if (buf_u32(&b, fnv1a(b.buf, 16, FNV_SEED)) != 0) goto fail;

    for (int l = 0; l < m->layer_count; l++) {
        const Layer* L = &m->layers[l];
        size_t total = (size_t)L->w * (size_t)L->h;
        if (buf_u32(&b, cells_checksum(L->cells, total)) != 0) goto fail;
        if (write_layer(&b, L, bpc) != 0) goto fail;
    }

    if (buf_u32(&b, fnv1a(b.buf, b.len, FNV_SEED)) != 0) goto fail;

    *out_buf = b.buf;
    *out_len = b.len;
    return 0;
fail:
    free(b.buf);
    return -1;
}

static int save_write(const char* path, const SaveMap* m) {
    unsigned char* buf = NULL;
    size_t n = 0;
    int rc = save_to_bytes(m, &buf, &n);
    if (rc != 0) return rc;
    FILE* f = fopen(path, "wb");
    if (!f) { free(buf); return -3; }
    size_t wrote = fwrite(buf, 1, n, f);
    int closed = fclose(f);
    free(buf);
    if (wrote != n || closed != 0) return -3;
    return 0;
}

/* ------------------------------------------------------------------ */
/* reader                                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    int code;
    size_t offset;
    char msg[96];
} SaveError;

enum {
    ERR_OK = 0,
    ERR_BAD_ARG,
    ERR_IO,
    ERR_OOM,
    ERR_TRUNCATED,
    ERR_BAD_MAGIC,
    ERR_BAD_VERSION,
    ERR_BAD_TILE_BITS,
    ERR_BAD_DIMS,
    ERR_BAD_HEADER_CKSUM,
    ERR_BAD_RUN,
    ERR_BAD_LAYER_CKSUM,
    ERR_BAD_SIZE,
    ERR_BAD_FILE_CKSUM
};

static void set_err(SaveError* e, int code, size_t offset, const char* msg) {
    if (!e) return;
    e->code = code;
    e->offset = offset;
    snprintf(e->msg, sizeof(e->msg), "%s", msg);
}

static int read_literal(const unsigned char* buf, size_t* pos, size_t end, size_t count,
                        uint16_t* out, int bpc, SaveError* err) {
    size_t bytes = count * (size_t)bpc;
    if (*pos + bytes > end) {
        set_err(err, ERR_TRUNCATED, *pos, "truncated in literal payload");
        return -1;
    }
    for (size_t k = 0; k < count; k++) {
        out[k] = bpc == 1 ? buf[*pos + k] : rd_u16(buf + *pos + k * 2);
    }
    *pos += bytes;
    return 0;
}

static int read_layer(const unsigned char* buf, size_t* pos, size_t end, Layer* L,
                      int bpc, SaveError* err) {
    size_t total = (size_t)L->w * (size_t)L->h;
    size_t written = 0;
    while (written < total) {
        if (*pos >= end) {
            set_err(err, ERR_TRUNCATED, *pos, "truncated in run data");
            return -1;
        }
        size_t tag_off = *pos;
        unsigned tag = buf[*pos];
        *pos += 1;
        if (tag == RUN_LITERAL) {
            if (*pos + 2 > end) {
                set_err(err, ERR_TRUNCATED, *pos, "truncated literal count");
                return -1;
            }
            unsigned count = rd_u16(buf + *pos);
            *pos += 2;
            if (count == 0) {
                set_err(err, ERR_BAD_RUN, tag_off, "zero-length literal");
                return -1;
            }
            if (written + count > total) {
                set_err(err, ERR_BAD_RUN, tag_off, "literal overflows layer");
                return -1;
            }
            if (read_literal(buf, pos, end, count, L->cells + written, bpc, err) != 0) return -1;
            written += count;
        } else {
            unsigned count;
            if (tag == RUN_EXTENDED) {
                if (*pos + 2 > end) {
                    set_err(err, ERR_TRUNCATED, *pos, "truncated repeat count");
                    return -1;
                }
                count = rd_u16(buf + *pos);
                *pos += 2;
                if (count == 0) {
                    set_err(err, ERR_BAD_RUN, tag_off, "zero-length repeat");
                    return -1;
                }
            } else {
                count = tag;
            }
            if (written + count > total) {
                set_err(err, ERR_BAD_RUN, tag_off, "repeat overflows layer");
                return -1;
            }
            if (*pos + (size_t)bpc > end) {
                set_err(err, ERR_TRUNCATED, *pos, "truncated repeat value");
                return -1;
            }
            uint16_t v = bpc == 1 ? buf[*pos] : rd_u16(buf + *pos);
            *pos += (size_t)bpc;
            for (unsigned k = 0; k < count; k++) L->cells[written + k] = v;
            written += count;
        }
    }
    return 0;
}

/* Parse a complete file image.  Returns NULL on any error and leaves *err
   pointing at the byte offset where verification failed. */
static SaveMap* save_read_bytes(const unsigned char* buf, size_t n, SaveError* err) {
    if (err) { err->code = ERR_OK; err->offset = 0; err->msg[0] = 0; }
    if (!buf) { set_err(err, ERR_BAD_ARG, 0, "null buffer"); return NULL; }
    if (n < HEADER_SIZE + TRAILER_SIZE) {
        set_err(err, ERR_TRUNCATED, n, "shorter than header plus trailer");
        return NULL;
    }
    if (memcmp(buf, SAVE_MAGIC, 4) != 0) {
        set_err(err, ERR_BAD_MAGIC, 0, "bad magic");
        return NULL;
    }

    unsigned major = buf[4];
    unsigned minor = buf[5];
    unsigned flags = buf[6];
    unsigned bits = buf[7];
    (void)minor;   /* v1 readers accept additive minor revisions */
    (void)flags;   /* reserved: unknown advisory bits are ignored */
    if (major != SAVE_MAJOR) {
        set_err(err, ERR_BAD_VERSION, 4,
                major > SAVE_MAJOR ? "file needs a newer reader" : "invalid major version");
        return NULL;
    }
    if (bits != 8 && bits != 16) {
        set_err(err, ERR_BAD_TILE_BITS, 7, "tile bits must be 8 or 16");
        return NULL;
    }
    int bpc = (int)bits / 8;

    unsigned layer_count = rd_u16(buf + 8);
    unsigned w = rd_u16(buf + 10);
    unsigned h = rd_u16(buf + 12);
    if (rd_u32(buf + 16) != fnv1a(buf, 16, FNV_SEED)) {
        /* The checksum only proves the header is damaged.  Checked first, so
           a damaged dimension is reported as damage rather than as a bad
           dimension.  If a field holds an impossible value, name that field;
           otherwise the damage is in a free-form field (minor, flags) and the
           best we can point at is the checksum word. */
        size_t bad = 16;
        if (rd_u16(buf + 14) != 0) bad = 14;
        else if (w == 0) bad = 10;
        else if (h == 0) bad = 12;
        set_err(err, ERR_BAD_HEADER_CKSUM, bad, "header checksum mismatch");
        return NULL;
    }
    if (layer_count == 0 || layer_count > MAX_LAYERS) {
        set_err(err, ERR_BAD_DIMS, 8, "bad layer count");
        return NULL;
    }
    if (w == 0 || h == 0) {
        set_err(err, ERR_BAD_DIMS, w == 0 ? 10 : 12, "zero dimension");
        return NULL;
    }
    if ((unsigned long long)w * (unsigned long long)h *
        (unsigned long long)layer_count > MAX_CELLS) {
        set_err(err, ERR_BAD_DIMS, 10, "map exceeds cell limit");
        return NULL;
    }

    size_t end = n - TRAILER_SIZE;
    SaveMap* m = map_new((int)layer_count, (int)w, (int)h);
    if (!m) { set_err(err, ERR_OOM, 0, "out of memory"); return NULL; }

    size_t pos = HEADER_SIZE;
    for (unsigned l = 0; l < layer_count; l++) {
        size_t layer_off = pos;
        if (pos + 4 > end) {
            set_err(err, ERR_TRUNCATED, pos, "truncated layer record");
            goto fail;
        }
        uint32_t stored = rd_u32(buf + pos);
        pos += 4;
        if (read_layer(buf, &pos, end, &m->layers[l], bpc, err) != 0) goto fail;
        size_t total = (size_t)m->layers[l].w * (size_t)m->layers[l].h;
        if (cells_checksum(m->layers[l].cells, total) != stored) {
            set_err(err, ERR_BAD_LAYER_CKSUM, layer_off, "layer checksum mismatch");
            goto fail;
        }
    }
    if (pos != end) {
        set_err(err, ERR_BAD_SIZE, pos,
                pos < end ? "trailing bytes before checksum" : "missing bytes before checksum");
        goto fail;
    }
    if (rd_u32(buf + end) != fnv1a(buf, end, FNV_SEED)) {
        set_err(err, ERR_BAD_FILE_CKSUM, end, "file checksum mismatch");
        goto fail;
    }
    return m;

fail:
    map_free(m);
    return NULL;
}

static SaveMap* save_read_file(const char* path, SaveError* err) {
    FILE* f = fopen(path, "rb");
    if (!f) { set_err(err, ERR_IO, 0, "cannot open file"); return NULL; }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); set_err(err, ERR_IO, 0, "seek failed"); return NULL; }
    long sz = ftell(f);
    if (sz < 0) { fclose(f); set_err(err, ERR_IO, 0, "tell failed"); return NULL; }
    rewind(f);
    unsigned char* buf = (unsigned char*)malloc(sz > 0 ? (size_t)sz : 1);
    if (!buf) { fclose(f); set_err(err, ERR_OOM, 0, "out of memory"); return NULL; }
    size_t got = fread(buf, 1, (size_t)sz, f);
    fclose(f);
    if (got != (size_t)sz) { free(buf); set_err(err, ERR_IO, got, "short read"); return NULL; }
    SaveMap* m = save_read_bytes(buf, got, err);
    free(buf);
    return m;
}

/* ------------------------------------------------------------------ */
/* self-test                                                           */
/* ------------------------------------------------------------------ */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void show_err(const char* name, const SaveError* e) {
    printf("  %s: code=%d offset=%zu msg=%s\n", name, e->code, e->offset, e->msg);
}

static size_t count_cells(const SaveMap* m) {
    size_t n = 0;
    for (int l = 0; l < m->layer_count; l++)
        n += (size_t)m->layers[l].w * (size_t)m->layers[l].h;
    return n;
}

static SaveMap* make_sample(void) {
    SaveMap* m = map_new(3, 40, 30);
    if (!m) return NULL;
    for (int y = 5; y < 20; y++)
        for (int x = 8; x < 28; x++)
            map_set(m, 0, x, y, 1);
    for (int x = 0; x < 40; x++) { map_set(m, 1, x, 0, 7); map_set(m, 1, x, 29, 7); }
    for (int y = 1; y < 29; y++) { map_set(m, 1, 0, y, 7); map_set(m, 1, 39, y, 7); }
    for (int i = 0; i < 60; i++) map_set(m, 1, (i * 13) % 40, (i * 7) % 30, (uint16_t)(40 + i));
    for (int i = 0; i < 20; i++) map_set(m, 2, (i * 11) % 40, (i * 17) % 30, (uint16_t)(900 + i));
    return m;
}

static void put_u32le(unsigned char* p, uint32_t v) {
    p[0] = (unsigned char)(v & 0xFFu);
    p[1] = (unsigned char)((v >> 8) & 0xFFu);
    p[2] = (unsigned char)((v >> 16) & 0xFFu);
    p[3] = (unsigned char)((v >> 24) & 0xFFu);
}

/* Recompute both checksums after the test deliberately edits a header byte,
   so the edit reaches the specific validation under test. */
static void reseal(unsigned char* buf, size_t n) {
    put_u32le(buf + 16, fnv1a(buf, 16, FNV_SEED));
    put_u32le(buf + n - TRAILER_SIZE, fnv1a(buf, n - TRAILER_SIZE, FNV_SEED));
}

int main(void) {
    /* ---------- round trip ---------- */
    SaveMap* m = make_sample();
    if (!m) { printf("oom\n"); return 1; }

    unsigned char* bytes = NULL;
    size_t len = 0;
    check("save_ok", save_to_bytes(m, &bytes, &len) == 0 && bytes != NULL);

    SaveError err;
    SaveMap* back = save_read_bytes(bytes, len, &err);
    check("load_ok", back != NULL && err.code == ERR_OK);
    if (!back) show_err("load", &err);
    check("roundtrip_equal", back && map_equal(m, back));
    check("version_major", bytes && bytes[4] == SAVE_MAJOR && bytes[5] == SAVE_MINOR);
    check("auto_width_16", bytes && bytes[7] == 16);   /* ids above 255 */

    size_t cells = count_cells(m);
    size_t binary16 = cells * 2 + 12;
    size_t binary8 = cells + 12;
    printf("map=%dx%d layers=%d cells=%zu\n",
           m->layers[0].w, m->layers[0].h, m->layer_count, cells);
    printf("rle_bytes=%zu binary_u16=%zu binary_u8=%zu\n", len, binary16, binary8);
    printf("ratio_u16=%.3f saving_u16=%.1f%%\n",
           (double)len / (double)binary16, 100.0 * (1.0 - (double)len / (double)binary16));
    printf("ratio_u8=%.3f saving_u8=%.1f%%\n",
           (double)len / (double)binary8, 100.0 * (1.0 - (double)len / (double)binary8));

    /* per-cell comparison across every layer */
    int mismatches = 0;
    for (int l = 0; l < m->layer_count; l++)
        for (int y = 0; y < m->layers[l].h; y++)
            for (int x = 0; x < m->layers[l].w; x++)
                if (map_get(m, l, x, y) != map_get(back, l, x, y)) mismatches++;
    check("cell_by_cell", back != NULL && mismatches == 0);

    /* file round trip */
    const char* path = "mapsave_tmp.mgms";
    check("file_write", save_write(path, m) == 0);
    SaveMap* from_disk = save_read_file(path, &err);
    check("file_read_equal", from_disk && map_equal(m, from_disk));
    if (from_disk) map_free(from_disk);
    remove(path);

    /* ---------- 8-bit width and highly compressible maps ---------- */
    SaveMap* narrow = map_new(1, 16, 16);
    map_set(narrow, 0, 3, 3, 200);
    map_set(narrow, 0, 4, 3, 42);
    unsigned char* nrb = NULL;
    size_t nrl = 0;
    check("narrow_save", save_to_bytes(narrow, &nrb, &nrl) == 0 && nrb[7] == 8);
    SaveMap* nrback = save_read_bytes(nrb, nrl, &err);
    check("narrow_roundtrip", nrback && map_equal(narrow, nrback));

    SaveMap* empty = map_new(1, 128, 128);
    unsigned char* eb = NULL;
    size_t elen = 0;
    check("empty_save", save_to_bytes(empty, &eb, &elen) == 0);
    check("empty_small", elen < 40);   /* one repeat record, not 16384 bytes */
    SaveMap* eback = save_read_bytes(eb, elen, &err);
    check("empty_roundtrip", eback && map_equal(empty, eback));

    SaveMap* solid = map_new(1, 128, 128);
    for (size_t i = 0; i < 128u * 128u; i++) solid->layers[0].cells[i] = 3;
    unsigned char* sb = NULL;
    size_t slen = 0;
    check("solid_save", save_to_bytes(solid, &sb, &slen) == 0);
    check("solid_small", slen < 40);
    SaveMap* sback = save_read_bytes(sb, slen, &err);
    check("solid_roundtrip", sback && map_equal(solid, sback));

    SaveMap* noisy = map_new(1, 32, 32);
    for (size_t i = 0; i < 32u * 32u; i++) noisy->layers[0].cells[i] = (uint16_t)(i * 37u + 1u);
    unsigned char* nb = NULL;
    size_t nlen = 0;
    check("noisy_save", save_to_bytes(noisy, &nb, &nlen) == 0);
    check("noisy_overhead_bounded", nlen <= 32u * 32u * 2u + 64u);
    SaveMap* nback = save_read_bytes(nb, nlen, &err);
    check("noisy_roundtrip", nback && map_equal(noisy, nback));

    /* ---------- corruption handling ---------- */
    SaveError e2;
    unsigned char* tmp = (unsigned char*)malloc(len);
    memcpy(tmp, bytes, len);

    SaveMap* r = save_read_bytes(tmp, len - 10, &e2);
    check("reject_truncated", r == NULL && e2.code != ERR_OK && e2.offset <= len - 10);
    show_err("truncated", &e2);

    tmp[len / 2] ^= 0x01;
    r = save_read_bytes(tmp, len, &e2);
    check("reject_body_corrupt", r == NULL && e2.code != ERR_OK);
    show_err("body_corrupt", &e2);
    memcpy(tmp, bytes, len);

    tmp[0] = 'X';
    r = save_read_bytes(tmp, len, &e2);
    check("reject_magic", r == NULL && e2.code == ERR_BAD_MAGIC && e2.offset == 0);
    memcpy(tmp, bytes, len);

    tmp[4] = SAVE_MAJOR + 1;
    r = save_read_bytes(tmp, len, &e2);
    check("reject_future_major", r == NULL && e2.code == ERR_BAD_VERSION && e2.offset == 4);
    memcpy(tmp, bytes, len);

    tmp[5] = (unsigned char)(SAVE_MINOR + 1);   /* additive minor revision */
    reseal(tmp, len);
    r = save_read_bytes(tmp, len, &e2);
    check("accept_newer_minor", r != NULL && map_equal(m, r));
    if (r) map_free(r);
    memcpy(tmp, bytes, len);

    tmp[6] = 0x80;   /* unknown advisory flag */
    reseal(tmp, len);
    r = save_read_bytes(tmp, len, &e2);
    check("accept_unknown_flags", r != NULL && map_equal(m, r));
    if (r) map_free(r);
    memcpy(tmp, bytes, len);

    tmp[7] = 7;      /* neither 8 nor 16 tile bits */
    reseal(tmp, len);
    r = save_read_bytes(tmp, len, &e2);
    check("reject_tile_bits", r == NULL && e2.code == ERR_BAD_TILE_BITS && e2.offset == 7);
    memcpy(tmp, bytes, len);

    tmp[8] = 0;      /* zero layers */
    reseal(tmp, len);
    r = save_read_bytes(tmp, len, &e2);
    check("reject_layer_count", r == NULL && e2.code == ERR_BAD_DIMS && e2.offset == 8);
    memcpy(tmp, bytes, len);

    tmp[10] = tmp[11] = 0;   /* width of zero: damaged and impossible */
    r = save_read_bytes(tmp, len, &e2);
    check("reject_header_cksum", r == NULL && e2.code == ERR_BAD_HEADER_CKSUM && e2.offset == 10);
    memcpy(tmp, bytes, len);

    tmp[14] = 1;     /* reserved field must stay zero */
    r = save_read_bytes(tmp, len, &e2);
    check("reject_reserved", r == NULL && e2.code == ERR_BAD_HEADER_CKSUM && e2.offset == 14);
    memcpy(tmp, bytes, len);

    /* flip the stored file checksum itself: body decodes, final check fails */
    tmp[len - 1] ^= 0x01;
    r = save_read_bytes(tmp, len, &e2);
    check("reject_file_cksum", r == NULL && e2.code == ERR_BAD_FILE_CKSUM &&
          e2.offset == len - TRAILER_SIZE);
    memcpy(tmp, bytes, len);

    /* append a byte: structure decodes but the size no longer matches */
    {
        unsigned char* longer = (unsigned char*)malloc(len + 1);
        memcpy(longer, bytes, len);
        longer[len] = 0;
        r = save_read_bytes(longer, len + 1, &e2);
        check("reject_trailing_byte", r == NULL && e2.code == ERR_BAD_SIZE);
        free(longer);
    }

    /* blank map: a single repeat record; flip its value byte so the layer
       decodes structurally but the layer checksum disagrees */
    {
        unsigned char* ebuf = (unsigned char*)malloc(elen);
        memcpy(ebuf, eb, elen);
        /* header 20, layer checksum 4, 0xFF tag, u16 count, value */
        check("empty_layout", elen == HEADER_SIZE + 4 + 1 + 2 + 1 + TRAILER_SIZE);
        ebuf[HEADER_SIZE + 4 + 1 + 2] ^= 0x01;
        r = save_read_bytes(ebuf, elen, &e2);
        check("reject_layer_cksum", r == NULL && e2.code == ERR_BAD_LAYER_CKSUM &&
              e2.offset == HEADER_SIZE);
        show_err("layer_cksum", &e2);
        free(ebuf);
    }

    /* non-uniform layers are refused by the writer before serialization */
    {
        SaveMap* uneven = map_new(2, 8, 8);
        unsigned char* ub = NULL;
        size_t ulen = 0;
        uneven->layers[1].w = 4;
        check("reject_nonuniform_save", save_to_bytes(uneven, &ub, &ulen) == -2 && ub == NULL);
        uneven->layers[1].w = 8;
        map_free(uneven);
    }

    free(tmp);
    free(bytes); free(nrb); free(eb); free(sb); free(nb);
    map_free(m); map_free(back); map_free(narrow); map_free(nrback);
    map_free(empty); map_free(eback); map_free(solid); map_free(sback);
    map_free(noisy); map_free(nback);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
