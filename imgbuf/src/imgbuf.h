/* imgbuf: shared image buffer — refcounted RGBA8 pixels with zero-copy
   sharing, plus copy/byte accounting so integrations can prove they
   did not make a second copy and stayed within a memory budget. */
#ifndef IMGBUF_H
#define IMGBUF_H

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct ImgBuf ImgBuf;

/* Create an owning buffer of w*h RGBA8 pixels, zeroed. NULL on bad args. */
ImgBuf* imgbuf_create(int w, int h);

/* Add a reference and return the same buffer. Shares pixels, never copies. */
ImgBuf* imgbuf_share(ImgBuf* b);

/* Drop a reference; the pixels are freed when the last reference goes away. */
void imgbuf_release(ImgBuf* b);

unsigned char* imgbuf_pixels(ImgBuf* b);
const unsigned char* imgbuf_pixels_ro(const ImgBuf* b);
int imgbuf_width(const ImgBuf* b);
int imgbuf_height(const ImgBuf* b);
size_t imgbuf_bytes(const ImgBuf* b);   /* w*h*4 */

/* The only pixel-copying entry point in the library; every call is counted.
   dst and src must have the same dimensions. Returns -1 on error. */
int imgbuf_copy_into(ImgBuf* dst, const ImgBuf* src);

/* Introspection for benchmarks and zero-copy verification. */
unsigned long imgbuf_copy_count(void);  /* pixel copies performed so far */
unsigned long imgbuf_live_bytes(void);  /* bytes held by owning buffers now */
unsigned long imgbuf_peak_bytes(void);  /* high-water mark of live bytes */
void imgbuf_reset_peak(void);           /* peak := current live bytes */

const char* imgbuf_last_error(void);

#ifdef __cplusplus
}
#endif

#endif
