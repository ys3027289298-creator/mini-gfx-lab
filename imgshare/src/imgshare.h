/* imgshare: shared, reference-counted image buffers with an intern table.
   Multiple consumers point at one allocation instead of each memcpy'ing a
   private copy.  Pixels are immutable through a shared handle; mutators get a
   private buffer via copy-on-write. */
#ifndef IMGSHARE_H
#define IMGSHARE_H

#include <stddef.h>
#include <stdint.h>

#define IMGS_MAX_BUFFERS 64
#define IMGS_MAX_INTERN 64
#define IMGS_MAX_REFS 0x7fffffff
#define IMGS_MAX_DIM 8192
#define IMGS_NAME_MAX 32
#define IMGS_MAGIC 0x494d4753u /* "IMGS" */

typedef enum {
    IMGS_FMT_RGBA8 = 1,
    IMGS_FMT_RGB8  = 3,
    IMGS_FMT_GRAY8 = 4
} ImgsFormat;

typedef struct ImgsBuffer ImgsBuffer;
typedef ImgsBuffer* ImgsRef;

struct ImgsBuffer {
    uint32_t magic;
    int w, h, stride;          /* stride in bytes */
    ImgsFormat fmt;
    int refs;                  /* live handles, >= 1 */
    int writable;              /* private buffer that may be mutated in place */
    size_t alloc_bytes;        /* full malloc size, header included */
    unsigned char* pixels;     /* packed, no padding */
    char name[IMGS_NAME_MAX];
};

typedef struct {
    uint64_t key;
    ImgsRef ref;
} ImgsInternSlot;

typedef struct {
    ImgsRef buffers[IMGS_MAX_BUFFERS];
    int count;
    ImgsInternSlot table[IMGS_MAX_INTERN];
} ImgsImage;

/* status codes: every public call below returns one of these (or a handle) */
enum {
    IMGS_OK = 0,
    IMGS_EINVAL = 1,   /* null / bad dimensions / bad format / duplicate key */
    IMGS_ENOENT = 2,   /* key not interned */
    IMGS_EBUSY = 3     /* buffer not writable in place */
};

typedef struct {
    uint64_t creates;
    uint64_t releases;
    uint64_t shared_uses;      /* handles handed out by sharing, not copying */
    uint64_t copies;           /* actual pixel memcpys performed */
    uint64_t copy_bytes;
    uint64_t avoided_copies;   /* per-consumer copies elided by sharing */
    uint64_t avoided_copy_bytes;
    uint64_t writable_copies;  /* copy-on-write allocations */
    uint64_t illegal_calls;
    size_t   live_bytes;       /* currently allocated by this subsystem */
    size_t   peak_bytes;       /* high-water mark of live_bytes */
    int      live_buffers;
} ImgsStats;

void imgs_stats_reset(void);
const ImgsStats* imgs_stats(void);

/* lifecycle */
int  imgs_image_init(ImgsImage* img);
void imgs_image_destroy(ImgsImage* img);   /* frees everything, ignores refs */

/* create + intern (key==0 means private, not interned).
   Returns NULL on any invalid argument; increments illegal_calls. */
ImgsRef imgs_create(ImgsImage* img, int w, int h, ImgsFormat fmt,
                    uint64_t key, const char* name);

/* look up an interned key and take a shared reference. */
int  imgs_open(ImgsImage* img, uint64_t key, ImgsRef* out);

/* take another reference to a handle already owned by this image. */
ImgsRef imgs_acquire(ImgsImage* img, ImgsRef ref);

/* drop one reference; frees when the count reaches zero. */
int  imgs_release(ImgsImage* img, ImgsRef ref);

/* read-only sharing: returns ref itself while refs==1 and writable,
   otherwise a fresh private copy (copy-on-write). */
ImgsRef imgs_make_writable(ImgsImage* img, ImgsRef ref);

/* accessors / helpers */
int  imgs_image_has(const ImgsImage* img, ImgsRef ref);
int  imgs_refs(const ImgsRef ref);
size_t imgs_bytes(const ImgsRef ref);
int  imgs_pixel_fill(ImgsRef ref, int seed);
uint64_t imgs_pixel_hash(const ImgsRef ref);   /* 0 on invalid handle */

#endif /* IMGSHARE_H */
