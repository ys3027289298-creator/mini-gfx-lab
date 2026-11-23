/* decodecache: a self-contained RGBA8 image codec with a decode cache, so the
   same encoded image decoded by several consumers is only ever decoded once.

   The codec (DCI1) is a small, dependency-free image format: an RLE packet
   layer over PNG-style per-scanline byte filters (None/Sub/Up/Average/Paeth),
   with an FNV-1a checksum trailer.  Decoding expands the RLE stream, reverses
   the filters and verifies the checksum, i.e. real, measurable work.

   The cache is keyed by an immutable asset id (the same contract as an engine
   texture keyed by its asset path).  With the cache enabled a hit returns the
   stored pixels in O(1) and never re-decodes; with it disabled every call runs
   the full decoder, which is the legacy behaviour.  Both modes return
   byte-for-byte identical pixels for the same encoded blob. */
#ifndef DC_CODEC_H
#define DC_CODEC_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DC_MAGIC0 'D'
#define DC_MAGIC1 'C'
#define DC_MAGIC2 'I'
#define DC_MAGIC3 '1'
#define DC_CHANNELS 4              /* RGBA8 */
#define DC_MAX_DIM 512
#define DC_CACHE_CAP 8             /* fixed-capacity intern table */

/* A decoded image.  Borrowed from the cache: valid until the next dc_get() on
   the same cache or dc_cache_destroy().  Pixels are immutable to the caller. */
typedef struct {
    int w, h, channels;
    size_t bytes;                 /* w*h*channels */
    const unsigned char* pixels;
    uint64_t source_hash;         /* FNV-1a of the encoded blob, 0 on cache-off scratch */
    uint64_t pixel_hash;          /* FNV-1a over geometry + decoded pixels */
} DcImage;

typedef struct DcCache DcCache;

/* Create a cache. enabled != 0 caches decoded images; enabled == 0 takes the
   legacy path and re-decodes on every call.  The intern table holds at most
   DC_CACHE_CAP images and evicts FIFO once full.  NULL only on allocation
   failure. */
DcCache* dc_cache_create(int enabled);
void dc_cache_destroy(DcCache* c);

int dc_cache_enabled(const DcCache* c);

/* Decode the image identified by image_id.
   On a cache hit (enabled, same id and blob length) the stored pixels are
   returned and the decoder is not run.  On a miss the blob is fully decoded.
   Returns NULL and sets dc_last_error() on any malformed input. */
const DcImage* dc_get(DcCache* c, uint64_t image_id,
                      const unsigned char* blob, size_t len);

/* Deterministically build an encoded DCI1 blob for a synthetic image of the
   given size.  Identical arguments always yield identical bytes.  Returns the
   blob length, or 0 on error (bad dimensions / output buffer too small). */
size_t dc_build_blob(uint32_t seed, int w, int h,
                     unsigned char* out, size_t out_cap);

/* Encode caller-provided RGBA8 pixels (w*h*4 bytes) into a DCI1 blob.
   Returns the blob length, or 0 on error.  dc_get() on the result must hand
   back exactly these pixels, which gives the round-trip its independent
   reference (the pixels come from the caller, not from the encoder). */
size_t dc_encode_rgba(int w, int h, const unsigned char* pixels,
                      unsigned char* out, size_t out_cap);

/* FNV-1a 64 over a byte buffer. */
uint64_t dc_fnv64(const unsigned char* data, size_t len);

/* FNV-1a 64 over an image's geometry and pixels (the pixel-hash evidence). */
uint64_t dc_image_pixel_hash(int w, int h, int channels, const unsigned char* px);

/* Monotonic timestamp in microseconds, for the decode-timing table. */
unsigned long dc_now_us(void);

/* Introspection for the hit log and the zero-redecode assertion. */
unsigned long dc_decodes(const DcCache* c);   /* real decoder invocations */
unsigned long dc_hits(const DcCache* c);
unsigned long dc_misses(const DcCache* c);
const char* dc_last_log(const DcCache* c);    /* most recent hit/miss line */
const char* dc_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
