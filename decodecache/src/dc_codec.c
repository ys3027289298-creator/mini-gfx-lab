/* decodecache: DCI1 codec + fixed-capacity decode cache.  See dc_codec.h. */
#define _POSIX_C_SOURCE 199309L
#include "dc_codec.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define FNV_OFFSET 1469598103934665603ull
#define FNV_PRIME  1099511628211ull
#define RLE_MAX 128                /* max run / literal length per packet */
#define HDR_LEN 14                 /* magic4 w2 h2 ch1 rsv1 payload_len4 BE */

enum { F_NONE = 0, F_SUB = 1, F_UP = 2, F_AVG = 3, F_PAETH = 4 };

static char g_err[160] = "no error";

static void set_err(const char* msg) {
    strncpy(g_err, msg, sizeof(g_err) - 1);
    g_err[sizeof(g_err) - 1] = '\0';
}
const char* dc_last_error(void) { return g_err; }

/* ------------------------------------------------------------- utilities */

uint64_t dc_fnv64(const unsigned char* data, size_t len) {
    uint64_t h = FNV_OFFSET;
    for (size_t i = 0; i < len; i++) { h ^= data[i]; h *= FNV_PRIME; }
    return h;
}

uint64_t dc_image_pixel_hash(int w, int h, int channels, const unsigned char* px) {
    uint64_t hsh = FNV_OFFSET;
    unsigned char hdr[12];
    int32_t ww = w, hh = h, cc = channels;
    memcpy(hdr + 0, &ww, 4);
    memcpy(hdr + 4, &hh, 4);
    memcpy(hdr + 8, &cc, 4);
    for (int i = 0; i < 12; i++) { hsh ^= hdr[i]; hsh *= FNV_PRIME; }
    size_t bytes = (size_t)w * (size_t)h * (size_t)channels;
    for (size_t i = 0; i < bytes; i++) { hsh ^= px[i]; hsh *= FNV_PRIME; }
    return hsh;
}

unsigned long dc_now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long)ts.tv_sec * 1000000ul + (unsigned long)ts.tv_nsec / 1000ul;
}

/* deterministic xorshift32 so the same seed paints the same image */
static uint32_t xs32(uint32_t* s) {
    uint32_t x = *s;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    *s = x;
    return x;
}

static int paeth(int a, int b, int c) {
    int p = a + b - c;
    int pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
    if (pa <= pb && pa <= pc) return a;
    if (pb <= pc) return b;
    return c;
}

/* ------------------------------------------------------------- RLE layer */

/* Compress `in` into `out`.  Returns bytes written, 0 if capacity exhausted. */
static size_t rle_encode(const unsigned char* in, size_t n,
                         unsigned char* out, size_t cap) {
    size_t o = 0, i = 0;
    while (i < n) {
        size_t run = 1;
        while (i + run < n && in[i + run] == in[i] && run < RLE_MAX) run++;
        if (run >= 3) {                                  /* worth a run packet */
            if (o + 2 > cap) return 0;
            out[o++] = (unsigned char)(0x80 | (run - 1));
            out[o++] = in[i];
            i += run;
        } else {
            size_t start = i;
            while (i < n) {                              /* literal packet */
                size_t r2 = 1;
                while (i + r2 < n && in[i + r2] == in[i] && r2 < RLE_MAX) r2++;
                if (r2 >= 3 || i - start >= RLE_MAX) break;
                i++;
            }
            size_t cnt = i - start;
            if (cnt == 0) { cnt = 1; i++; }
            if (o + 1 + cnt > cap) return 0;
            out[o++] = (unsigned char)(cnt - 1);
            memcpy(out + o, in + start, cnt);
            o += cnt;
        }
    }
    return o;
}

/* Expand RLE payload.  Returns 0 on success, -1 on a truncated packet. */
static int rle_decode(const unsigned char* in, size_t n,
                      unsigned char* out, size_t expect) {
    size_t i = 0, o = 0;
    while (i < n) {
        unsigned char ctrl = in[i++];
        size_t cnt = (size_t)(ctrl & 0x7f) + 1;
        if (ctrl & 0x80) {
            if (i >= n || o + cnt > expect) return -1;
            unsigned char v = in[i++];
            memset(out + o, v, cnt);
        } else {
            if (i + cnt > n || o + cnt > expect) return -1;
            memcpy(out + o, in + i, cnt);
            i += cnt;
        }
        o += cnt;
    }
    return o == expect ? 0 : -1;
}

/* ------------------------------------------------------------- PNG filters */

static void filter_row(int type, const unsigned char* cur,
                       const unsigned char* prev, int row_bytes,
                       unsigned char* out) {
    for (int x = 0; x < row_bytes; x++) {
        int a = x >= DC_CHANNELS ? cur[x - DC_CHANNELS] : 0;
        int b = prev ? prev[x] : 0;
        int c = (prev && x >= DC_CHANNELS) ? prev[x - DC_CHANNELS] : 0;
        switch (type) {
        case F_SUB: out[x] = (unsigned char)(cur[x] - a); break;
        case F_UP:  out[x] = (unsigned char)(cur[x] - b); break;
        case F_AVG: out[x] = (unsigned char)(cur[x] - ((a + b) / 2)); break;
        case F_PAETH: out[x] = (unsigned char)(cur[x] - paeth(a, b, c)); break;
        default:    out[x] = cur[x];
        }
    }
}

static void unfilter_row(int type, unsigned char* cur,
                         const unsigned char* prev, int row_bytes) {
    for (int x = 0; x < row_bytes; x++) {
        int a = x >= DC_CHANNELS ? cur[x - DC_CHANNELS] : 0;
        int b = prev ? prev[x] : 0;
        int c = (prev && x >= DC_CHANNELS) ? prev[x - DC_CHANNELS] : 0;
        int v = cur[x];
        switch (type) {
        case F_SUB: v += a; break;
        case F_UP:  v += b; break;
        case F_AVG: v += (a + b) / 2; break;
        case F_PAETH: v += paeth(a, b, c); break;
        default: break;
        }
        cur[x] = (unsigned char)v;
    }
}

/* ------------------------------------------------------------- DCI1 encode */

/* Header (14): magic4 w2 h2 ch1 rsv1 payload_len4(BE); then RLE payload;
   then FNV-1a u64 over header+payload. */

/* Filter each scanline, RLE the filtered stream and frame the DCI1 blob.
   Shared by the synthetic builder and the caller-pixel encoder. */
static size_t encode_pixels(int w, int h, const unsigned char* src,
                            unsigned char* out, size_t out_cap) {
    int row_bytes = w * DC_CHANNELS;
    size_t filt_bytes = (size_t)h * (size_t)(row_bytes + 1);
    size_t rle_cap = filt_bytes * 2 + 1;
    unsigned char* filtered = (unsigned char*)malloc(filt_bytes);
    unsigned char* payload = (unsigned char*)malloc(rle_cap);
    if (!filtered || !payload) {
        free(filtered); free(payload);
        set_err("encode: out of memory"); return 0;
    }

    /* pick the lowest absolute-sum filter independently for every scanline */
    unsigned char scratch[DC_MAX_DIM * DC_CHANNELS];
    for (int y = 0; y < h; y++) {
        const unsigned char* cur = src + (size_t)y * row_bytes;
        const unsigned char* prev = y ? src + (size_t)(y - 1) * row_bytes : NULL;
        int best = F_NONE; long best_cost = 1l << 60;
        for (int t = F_NONE; t <= F_PAETH; t++) {
            filter_row(t, cur, prev, row_bytes, scratch);
            long cost = 0;
            for (int x = 0; x < row_bytes; x++) cost += abs((signed char)scratch[x]);
            if (cost < best_cost) { best_cost = cost; best = t; }
        }
        filtered[(size_t)y * (row_bytes + 1)] = (unsigned char)best;
        filter_row(best, cur, prev, row_bytes,
                   filtered + (size_t)y * (row_bytes + 1) + 1);
    }

    size_t plen = rle_encode(filtered, filt_bytes, payload, rle_cap);
    free(filtered);
    if (plen == 0) { free(payload); set_err("encode: RLE compression failed"); return 0; }
    size_t total = HDR_LEN + plen + 8;
    if (total > out_cap) { free(payload); set_err("encode: output buffer too small"); return 0; }

    out[0] = DC_MAGIC0; out[1] = DC_MAGIC1; out[2] = DC_MAGIC2; out[3] = DC_MAGIC3;
    out[4] = (unsigned char)(w >> 8); out[5] = (unsigned char)(w & 0xff);
    out[6] = (unsigned char)(h >> 8); out[7] = (unsigned char)(h & 0xff);
    out[8] = DC_CHANNELS; out[9] = 0;
    out[10] = (unsigned char)(plen >> 24); out[11] = (unsigned char)(plen >> 16);
    out[12] = (unsigned char)(plen >> 8); out[13] = (unsigned char)(plen & 0xff);
    memcpy(out + HDR_LEN, payload, plen);
    free(payload);
    uint64_t csum = dc_fnv64(out, HDR_LEN + plen);
    for (int k = 0; k < 8; k++)
        out[HDR_LEN + plen + k] = (unsigned char)(csum >> (8 * k));
    return total;
}

size_t dc_encode_rgba(int w, int h, const unsigned char* pixels,
                      unsigned char* out, size_t out_cap) {
    if (w <= 0 || h <= 0 || w > DC_MAX_DIM || h > DC_MAX_DIM || !pixels || !out) {
        set_err("dc_encode_rgba: bad arguments"); return 0;
    }
    return encode_pixels(w, h, pixels, out, out_cap);
}

size_t dc_build_blob(uint32_t seed, int w, int h,
                     unsigned char* out, size_t out_cap) {
    if (w <= 0 || h <= 0 || w > DC_MAX_DIM || h > DC_MAX_DIM) {
        set_err("dc_build_blob: dimensions out of range"); return 0;
    }
    size_t px_bytes = (size_t)w * (size_t)h * DC_CHANNELS;
    unsigned char* src = (unsigned char*)malloc(px_bytes);
    if (!src) { set_err("dc_build_blob: out of memory"); return 0; }

    uint32_t st = seed ? seed : 0x9e3779b9u;
    for (size_t i = 0; i < px_bytes / DC_CHANNELS; i++) {
        int x = (int)(i % (size_t)w), y = (int)(i / (size_t)w);
        src[4*i + 0] = (unsigned char)(xs32(&st) & 0xff);
        src[4*i + 1] = (unsigned char)((x * 7 + y * 13) & 0xff);
        src[4*i + 2] = (unsigned char)((y * 5 + (xs32(&st) & 0x1f)) & 0xff);
        src[4*i + 3] = 255;
    }

    size_t total = encode_pixels(w, h, src, out, out_cap);
    free(src);
    return total;
}

/* ------------------------------------------------------------- DCI1 decode */

typedef struct {
    int w, h, channels;
    unsigned char* pixels;
    size_t bytes;
} RawImage;

static void raw_free(RawImage* r) {
    free(r->pixels);
    memset(r, 0, sizeof(*r));
}

/* Full decode path: validate, RLE expand, reverse every filter, checksum. */
static int decode_full(const unsigned char* blob, size_t len, RawImage* out) {
    memset(out, 0, sizeof(*out));
    if (len < HDR_LEN + 8) { set_err("decode: blob too short"); return -1; }
    if (blob[0] != DC_MAGIC0 || blob[1] != DC_MAGIC1 ||
        blob[2] != DC_MAGIC2 || blob[3] != DC_MAGIC3) {
        set_err("decode: bad magic"); return -1;
    }
    int w = (blob[4] << 8) | blob[5];
    int h = (blob[6] << 8) | blob[7];
    int ch = blob[8];
    size_t plen = ((size_t)blob[10] << 24) | ((size_t)blob[11] << 16) |
                  ((size_t)blob[12] << 8) | blob[13];
    if (w <= 0 || h <= 0 || w > DC_MAX_DIM || h > DC_MAX_DIM ||
        ch != DC_CHANNELS) { set_err("decode: bad header fields"); return -1; }
    if (HDR_LEN + plen + 8 != len) { set_err("decode: length mismatch"); return -1; }

    uint64_t want = 0;
    for (int k = 0; k < 8; k++)
        want |= (uint64_t)blob[HDR_LEN + plen + k] << (8 * k);
    if (dc_fnv64(blob, HDR_LEN + plen) != want) {
        set_err("decode: checksum mismatch"); return -1;
    }

    int row_bytes = w * ch;
    size_t expect = (size_t)h * (size_t)(row_bytes + 1);
    unsigned char* filtered = (unsigned char*)malloc(expect);
    unsigned char* pixels = (unsigned char*)malloc((size_t)w * h * ch);
    if (!filtered || !pixels) {
        free(filtered); free(pixels); set_err("decode: out of memory"); return -1;
    }
    if (rle_decode(blob + HDR_LEN, plen, filtered, expect) != 0) {
        free(filtered); free(pixels); set_err("decode: malformed RLE stream"); return -1;
    }
    for (int y = 0; y < h; y++) {
        int type = filtered[(size_t)y * (row_bytes + 1)];
        if (type > F_PAETH) {
            free(filtered); free(pixels); set_err("decode: unknown filter"); return -1;
        }
        unsigned char* cur = pixels + (size_t)y * row_bytes;
        const unsigned char* prev = y ? pixels + (size_t)(y - 1) * row_bytes : NULL;
        memcpy(cur, filtered + (size_t)y * (row_bytes + 1) + 1, row_bytes);
        unfilter_row(type, cur, prev, row_bytes);
    }
    free(filtered);
    out->w = w; out->h = h; out->channels = ch;
    out->pixels = pixels;
    out->bytes = (size_t)w * h * ch;
    return 0;
}

/* ------------------------------------------------------------------ cache */

typedef struct {
    int valid;
    uint64_t id;
    size_t blob_len;
    uint64_t source_hash;
    DcImage img;
} Slot;

struct DcCache {
    int enabled;
    int cursor;                    /* FIFO eviction cursor */
    Slot slots[DC_CACHE_CAP];
    unsigned char* scratch;          /* cache-off decode target */
    size_t scratch_cap;
    DcImage scratch_img;
    unsigned long decodes, hits, misses;
    char last_log[200];
};

DcCache* dc_cache_create(int enabled) {
    DcCache* c = (DcCache*)calloc(1, sizeof(*c));
    if (!c) { set_err("dc_cache_create: out of memory"); return NULL; }
    c->enabled = enabled != 0;
    return c;
}

void dc_cache_destroy(DcCache* c) {
    if (!c) return;
    for (int i = 0; i < DC_CACHE_CAP; i++) free((void*)c->slots[i].img.pixels);
    free(c->scratch);
    free(c);
}

int dc_cache_enabled(const DcCache* c) { return c && c->enabled; }
unsigned long dc_decodes(const DcCache* c) { return c ? c->decodes : 0; }
unsigned long dc_hits(const DcCache* c) { return c ? c->hits : 0; }
unsigned long dc_misses(const DcCache* c) { return c ? c->misses : 0; }
const char* dc_last_log(const DcCache* c) { return c ? c->last_log : "no cache"; }

static const DcImage* publish(Slot* s, uint64_t id, size_t blob_len,
                              uint64_t src_hash, RawImage* raw) {
    s->valid = 1; s->id = id; s->blob_len = blob_len; s->source_hash = src_hash;
    free((void*)s->img.pixels);                 /* replace any previous entry */
    s->img.w = raw->w; s->img.h = raw->h; s->img.channels = raw->channels;
    s->img.bytes = raw->bytes;
    s->img.pixels = raw->pixels;                /* take ownership */
    s->img.source_hash = src_hash;
    s->img.pixel_hash =
        dc_image_pixel_hash(raw->w, raw->h, raw->channels, raw->pixels);
    return &s->img;
}

const DcImage* dc_get(DcCache* c, uint64_t image_id,
                      const unsigned char* blob, size_t len) {
    if (!c || !blob || len == 0) { set_err("dc_get: bad arguments"); return NULL; }
    uint64_t src_hash = dc_fnv64(blob, len);
    unsigned long t0 = dc_now_us();

    if (c->enabled) {
        for (int i = 0; i < DC_CACHE_CAP; i++) {
            Slot* s = &c->slots[i];
            if (s->valid && s->id == image_id && s->blob_len == len &&
                s->source_hash == src_hash) {
                c->hits++;
                snprintf(c->last_log, sizeof(c->last_log),
                         "cache_hit id=%llu decodes=%lu hits=%lu misses=%lu",
                         (unsigned long long)image_id, c->decodes, c->hits, c->misses);
                return &s->img;
            }
        }
        /* miss: decode into a free slot, else FIFO-evict at the cursor */
        int idx = -1;
        for (int i = 0; i < DC_CACHE_CAP; i++)
            if (!c->slots[i].valid) { idx = i; break; }
        if (idx < 0) { idx = c->cursor; c->cursor = (c->cursor + 1) % DC_CACHE_CAP; }
        RawImage raw;
        if (decode_full(blob, len, &raw) != 0) return NULL;
        c->decodes++; c->misses++;
        const DcImage* img = publish(&c->slots[idx], image_id, len, src_hash, &raw);
        unsigned long us = dc_now_us() - t0;
        snprintf(c->last_log, sizeof(c->last_log),
                 "cache_miss id=%llu decode_us=%lu decodes=%lu",
                 (unsigned long long)image_id, us, c->decodes);
        return img;
    }

    /* cache disabled: the legacy path re-decodes every single call */
    RawImage raw;
    if (decode_full(blob, len, &raw) != 0) return NULL;
    if (raw.bytes > c->scratch_cap) {
        unsigned char* nb = (unsigned char*)realloc(c->scratch, raw.bytes);
        if (!nb) { raw_free(&raw); set_err("dc_get: out of memory"); return NULL; }
        c->scratch = nb; c->scratch_cap = raw.bytes;
    }
    memcpy(c->scratch, raw.pixels, raw.bytes);
    raw_free(&raw);
    c->decodes++;
    c->scratch_img.w = (blob[4] << 8) | blob[5];
    c->scratch_img.h = (blob[6] << 8) | blob[7];
    c->scratch_img.channels = DC_CHANNELS;
    c->scratch_img.bytes = (size_t)c->scratch_img.w * c->scratch_img.h * DC_CHANNELS;
    c->scratch_img.pixels = c->scratch;
    c->scratch_img.source_hash = 0;
    c->scratch_img.pixel_hash = dc_image_pixel_hash(
        c->scratch_img.w, c->scratch_img.h, DC_CHANNELS, c->scratch);
    unsigned long us = dc_now_us() - t0;
    snprintf(c->last_log, sizeof(c->last_log),
             "cache_off id=%llu decode_us=%lu decodes=%lu",
             (unsigned long long)image_id, us, c->decodes);
    return &c->scratch_img;
}
