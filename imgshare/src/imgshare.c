/* imgshare: a shared, reference-counted image buffer with an intern table, so
   consumers that each used to memcpy their own copy can all point at one
   allocation instead.

   Pixels are immutable through a shared handle.  Mutating consumers get a
   private copy via imgs_make_writable(); that is the only place this module
   ever memcpys pixels.  Everything is plain C11 with fixed-capacity storage so
   that the allocation policy is observable: exactly one allocation exists per
   interned image, whatever the number of consumers. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "imgshare.h"

/* ------------------------------------------------------------------ stats */

static ImgsStats g_stats;

void imgs_stats_reset(void) {
    memset(&g_stats, 0, sizeof(g_stats));
}

const ImgsStats* imgs_stats(void) {
    return &g_stats;
}

/* ------------------------------------------------------------------ bytes */

static size_t imgs_bytes_for(int w, int h, ImgsFormat fmt) {
    if (w <= 0 || h <= 0) return 0;
    size_t px = (size_t)w * (size_t)h;
    switch (fmt) {
    case IMGS_FMT_RGBA8: return px * 4u;
    case IMGS_FMT_RGB8:  return px * 3u;
    case IMGS_FMT_GRAY8: return px;
    default:             return 0;
    }
}

static int imgs_valid_size(int w, int h) {
    /* 0 is not a legal dimension; oversized images are rejected before the
       size_t multiply can overflow. */
    return w > 0 && h > 0 && w <= IMGS_MAX_DIM && h <= IMGS_MAX_DIM;
}

static int imgs_is_live(ImgsRef ref) {
    return ref != NULL && ref->magic == IMGS_MAGIC;
}

/* Bytes actually owned by a buffer, header included. */
static size_t imgs_owned(const ImgsBuffer* b) {
    return imgs_bytes_for(b->w, b->h, b->fmt);
}
/* ------------------------------------------------------------- allocation */

static size_t imgs_hdr_size(void) {
    return (sizeof(ImgsBuffer) + 15u) & ~(size_t)15u;
}

/* The single owning constructor.  Pixels sit directly after the handle in one
   malloc so creation is one exact, accountable allocation. */
static ImgsRef imgs_alloc(int w, int h, ImgsFormat fmt, const char* name) {
    size_t bytes = imgs_bytes_for(w, h, fmt);
    if (bytes == 0) return NULL;
    size_t hdr = imgs_hdr_size();
    unsigned char* raw = (unsigned char*)calloc(1, hdr + bytes);
    if (!raw) return NULL;

    ImgsBuffer* b = (ImgsBuffer*)(void*)raw;
    b->magic = IMGS_MAGIC;
    b->w = w;
    b->h = h;
    b->fmt = fmt;
    b->stride = (int)(bytes / ((size_t)w * (size_t)h)) * w;
    b->refs = 1;
    b->writable = 1;
    b->alloc_bytes = hdr + bytes;
    b->pixels = raw + hdr;
    if (name) {
        size_t n = strlen(name);
        if (n >= sizeof(b->name)) n = sizeof(b->name) - 1;
        memcpy(b->name, name, n);
        b->name[n] = '\0';
    }

    g_stats.live_buffers++;
    g_stats.live_bytes += b->alloc_bytes;
    if (g_stats.live_bytes > g_stats.peak_bytes)
        g_stats.peak_bytes = g_stats.live_bytes;
    return b;
}

/* ---------------------------------------------------------------- intern */

static ImgsRef* imgs_slot_for(ImgsImage* img, uint64_t key) {
    if (key == 0) return NULL;
    for (int i = 0; i < IMGS_MAX_INTERN; i++)
        if (img->table[i].key == key) return &img->table[i].ref;
    return NULL;
}

/* Reserve an intern slot for a new key.  Returns NULL when the table is full;
   *existing is set when the key is already present. */
static ImgsInternSlot* imgs_slot_reserve(ImgsImage* img, uint64_t key,
                                         ImgsRef** existing) {
    *existing = NULL;
    for (int i = 0; i < IMGS_MAX_INTERN; i++) {
        if (img->table[i].key == key) { *existing = &img->table[i].ref; return NULL; }
    }
    for (int i = 0; i < IMGS_MAX_INTERN; i++) {
        if (img->table[i].key == 0) {
            img->table[i].key = key;
            img->table[i].ref = NULL;
            return &img->table[i];
        }
    }
    return NULL;
}

static void imgs_table_forget(ImgsImage* img, ImgsRef ref) {
    for (int i = 0; i < IMGS_MAX_INTERN; i++)
        if (img->table[i].ref == ref) {
            img->table[i].key = 0;
            img->table[i].ref = NULL;
        }
}

static void imgs_directory_remove(ImgsImage* img, ImgsRef ref) {
    for (int i = 0; i < img->count; i++) {
        if (img->buffers[i] == ref) {
            memmove(&img->buffers[i], &img->buffers[i + 1],
                    sizeof(img->buffers[0]) * (size_t)(img->count - i - 1));
            img->count--;
            img->buffers[img->count] = NULL;
            return;
        }
    }
}

static int imgs_directory_add(ImgsImage* img, ImgsRef ref) {
    if (img->count >= IMGS_MAX_BUFFERS) return 0;
    img->buffers[img->count++] = ref;
    return 1;
}

/* --------------------------------------------------------------- image */

int imgs_image_init(ImgsImage* img) {
    if (img == NULL) { g_stats.illegal_calls++; return IMGS_EINVAL; }
    memset(img, 0, sizeof(*img));
    return IMGS_OK;
}

void imgs_image_destroy(ImgsImage* img) {
    if (img == NULL) { g_stats.illegal_calls++; return; }
    while (img->count > 0) {
        ImgsRef b = img->buffers[--img->count];
        if (b == NULL) continue;
        g_stats.live_bytes -= b->alloc_bytes;
        g_stats.live_buffers--;
        b->magic = 0;
        free(b);
    }
    memset(img, 0, sizeof(*img));
}

int imgs_image_has(const ImgsImage* img, ImgsRef ref) {
    if (img == NULL || ref == NULL) return 0;
    for (int i = 0; i < img->count; i++)
        if (img->buffers[i] == ref) return 1;
    return 0;
}

/* -------------------------------------------------------------- create */

ImgsRef imgs_create(ImgsImage* img, int w, int h, ImgsFormat fmt, uint64_t key,
                    const char* name) {
    if (img == NULL || !imgs_valid_size(w, h) ||
        imgs_bytes_for(w, h, fmt) == 0) {
        g_stats.illegal_calls++;
        return NULL;
    }
    if (key != 0) {
        ImgsRef* existing = NULL;
        ImgsInternSlot* slot = imgs_slot_reserve(img, key, &existing);
        if (existing != NULL) {
            /* re-creating an interned key would silently alias or replace a
               live image: refuse it. */
            g_stats.illegal_calls++;
            return NULL;
        }
        if (slot == NULL) {          /* intern table full */
            g_stats.illegal_calls++;
            return NULL;
        }
        if (img->count >= IMGS_MAX_BUFFERS) {
            slot->key = 0;           /* release the reservation */
            g_stats.illegal_calls++;
            return NULL;
        }
        ImgsRef b = imgs_alloc(w, h, fmt, name);
        if (b == NULL) {
            slot->key = 0;
            return NULL;
        }
        img->buffers[img->count++] = b;
        slot->ref = b;
        g_stats.creates++;
        return b;
    }

    if (img->count >= IMGS_MAX_BUFFERS) {
        g_stats.illegal_calls++;
        return NULL;
    }
    ImgsRef b = imgs_alloc(w, h, fmt, name);
    if (b == NULL) return NULL;
    img->buffers[img->count++] = b;
    g_stats.creates++;
    return b;
}

/* ------------------------------------------------------------- acquire */

ImgsRef imgs_acquire(ImgsImage* img, ImgsRef ref) {
    if (!imgs_is_live(ref) || !imgs_image_has(img, ref)) {
        g_stats.illegal_calls++;
        return NULL;
    }
    if (ref->refs >= IMGS_MAX_REFS) {
        g_stats.illegal_calls++;
        return NULL;
    }
    ref->refs++;
    /* As soon as a second handle exists the pixels are shared and must not be
       mutated in place; a writer has to copy first. */
    if (ref->refs > 1) ref->writable = 0;

    /* Every new share handle is one copy the old per-consumer path would have
       made: this is the avoided-copy signal. */
    g_stats.shared_uses++;
    g_stats.avoided_copies++;
    g_stats.avoided_copy_bytes += imgs_owned(ref);
    return ref;
}

int imgs_open(ImgsImage* img, uint64_t key, ImgsRef* out) {
    if (img == NULL || out == NULL || key == 0) {
        g_stats.illegal_calls++;
        return IMGS_EINVAL;
    }
    ImgsRef* found = imgs_slot_for(img, key);
    if (found == NULL || *found == NULL) return IMGS_ENOENT;
    ImgsRef got = imgs_acquire(img, *found);
    if (got == NULL) return IMGS_EINVAL;
    *out = got;
    return IMGS_OK;
}

/* -------------------------------------------------------- copy on write */

ImgsRef imgs_make_writable(ImgsImage* img, ImgsRef ref) {
    if (!imgs_is_live(ref) || !imgs_image_has(img, ref)) {
        g_stats.illegal_calls++;
        return NULL;
    }
    if (ref->refs == 1 && ref->writable) return ref;

    if (img->count >= IMGS_MAX_BUFFERS) {
        g_stats.illegal_calls++;
        return NULL;
    }
    ImgsRef fresh = imgs_alloc(ref->w, ref->h, ref->fmt, ref->name);
    if (fresh == NULL) return NULL;
    memcpy(fresh->pixels, ref->pixels, imgs_owned(ref));
    if (!imgs_directory_add(img, fresh)) {
        g_stats.live_bytes -= fresh->alloc_bytes;
        g_stats.live_buffers--;
        fresh->magic = 0;
        free(fresh);
        g_stats.illegal_calls++;
        return NULL;
    }
    /* The copy is private: it is deliberately never added to the intern table,
       so imgs_open keeps handing out the canonical shared buffer. */
    g_stats.writable_copies++;
    g_stats.copies++;
    g_stats.copy_bytes += imgs_owned(fresh);
    return fresh;
}

/* --------------------------------------------------------------- release */

int imgs_release(ImgsImage* img, ImgsRef ref) {
    if (!imgs_is_live(ref) || !imgs_image_has(img, ref)) {
        g_stats.illegal_calls++;
        return IMGS_EINVAL;
    }
    if (ref->refs <= 0) {         /* unreachable for live handles, defensive */
        g_stats.illegal_calls++;
        return IMGS_EINVAL;
    }
    if (ref->refs > 1) {
        ref->refs--;
        g_stats.releases++;
        return IMGS_OK;
    }

    /* last reference: drop from directory + intern table, then free */
    imgs_directory_remove(img, ref);
    imgs_table_forget(img, ref);
    g_stats.live_bytes -= ref->alloc_bytes;
    g_stats.live_buffers--;
    ref->magic = 0;
    free(ref);
    g_stats.releases++;
    return IMGS_OK;
}

/* ---------------------------------------------------------------- access */

int imgs_refs(const ImgsRef ref) {
    if (!imgs_is_live(ref)) { g_stats.illegal_calls++; return -1; }
    return ref->refs;
}

size_t imgs_bytes(const ImgsRef ref) {
    if (!imgs_is_live(ref)) { g_stats.illegal_calls++; return 0; }
    return imgs_owned(ref);
}

int imgs_pixel_fill(ImgsRef ref, int seed) {
    if (!imgs_is_live(ref)) { g_stats.illegal_calls++; return IMGS_EINVAL; }
    if (!ref->writable) { g_stats.illegal_calls++; return IMGS_EBUSY; }
    unsigned v = (unsigned)seed * 2654435761u + 17u;
    size_t bytes = imgs_owned(ref);
    for (size_t i = 0; i < bytes; i++) {
        v = v * 1664525u + 1013904223u;
        ref->pixels[i] = (unsigned char)(v >> 24);
    }
    return IMGS_OK;
}

uint64_t imgs_pixel_hash(const ImgsRef ref) {
    if (!imgs_is_live(ref)) { g_stats.illegal_calls++; return 0; }
    /* FNV-1a 64 over the meaningful pixels, with the geometry folded in first
       so two different shapes cannot collide on all-zero content. */
    uint64_t hsh = 1469598103934665603ull;
    unsigned char hdrb[12];
    int32_t w = ref->w, h = ref->h, fmt = (int32_t)ref->fmt;
    memcpy(hdrb + 0, &w, 4);
    memcpy(hdrb + 4, &h, 4);
    memcpy(hdrb + 8, &fmt, 4);
    for (int i = 0; i < 12; i++) {
        hsh ^= hdrb[i];
        hsh *= 1099511628211ull;
    }
    size_t bytes = imgs_owned(ref);
    for (size_t i = 0; i < bytes; i++) {
        hsh ^= ref->pixels[i];
        hsh *= 1099511628211ull;
    }
    return hsh;
}
