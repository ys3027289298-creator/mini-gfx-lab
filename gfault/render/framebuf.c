
/* framebuf implementation: clipping primitives, PPM export, SHA-256 over
   raw RGBA for pixel-exact golden comparisons. No heap allocation. */
#include "framebuf.h"
#include <stdio.h>
#include <string.h>

static const FbRegion REGIONS[GFAULT_SEGMENTS] = {
    [GFAULT_SEG_PARTICLE] = {   0,   0, 160, 120 },
    [GFAULT_SEG_TILEMAP]  = { 160,   0, 160, 120 },
    [GFAULT_SEG_NAVGRID]  = {   0, 120, 160, 120 },
    [GFAULT_SEG_FONT]     = { 160, 120, 160,  60 },
    [GFAULT_SEG_PALETTE]  = { 160, 180, 160,  60 },
};

const FbRegion* fb_region(GfaultSeg segment) {
    if (segment < 0 || segment >= GFAULT_SEGMENTS) return 0;
    return &REGIONS[segment];
}

void fb_clear(FrameBuf* fb, Rgba color) {
    for (int y = 0; y < FB_H; y++)
        for (int x = 0; x < FB_W; x++)
            fb->px[y][x] = color;
}

void fb_pixel(FrameBuf* fb, int x, int y, Rgba color) {
    if (x < 0 || y < 0 || x >= FB_W || y >= FB_H) return;
    fb->px[y][x] = color;
}

void fb_rect(FrameBuf* fb, int x0, int y0, int x1, int y1, Rgba color) {
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= FB_W) x1 = FB_W - 1;
    if (y1 >= FB_H) y1 = FB_H - 1;
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            fb->px[y][x] = color;
}

void fb_hline(FrameBuf* fb, int x0, int x1, int y, Rgba color) {
    if (y < 0 || y >= FB_H) return;
    if (x0 < 0) x0 = 0;
    if (x1 >= FB_W) x1 = FB_W - 1;
    for (int x = x0; x <= x1; x++) fb->px[y][x] = color;
}

void fb_vline(FrameBuf* fb, int x, int y0, int y1, Rgba color) {
    if (x < 0 || x >= FB_W) return;
    if (y0 < 0) y0 = 0;
    if (y1 >= FB_H) y1 = FB_H - 1;
    for (int y = y0; y <= y1; y++) fb->px[y][x] = color;
}

/* ---- compact FIPS 180-4 SHA-256, stack/context only, no malloc ---- */
typedef struct {
    unsigned int h[8];
    unsigned long long len;
    unsigned char buf[64];
    int n;
} Sha256;

static const unsigned int K256[64] = {
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};

#define ROTR(x,n) (((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(Sha256* c, const unsigned char* p) {
    unsigned int w[64];
    for (int i = 0; i < 16; i++)
        w[i] = ((unsigned int)p[i*4] << 24) | ((unsigned int)p[i*4+1] << 16) |
               ((unsigned int)p[i*4+2] << 8) | (unsigned int)p[i*4+3];
    for (int i = 16; i < 64; i++) {
        unsigned int s0 = ROTR(w[i-15],7) ^ ROTR(w[i-15],18) ^ (w[i-15] >> 3);
        unsigned int s1 = ROTR(w[i-2],17) ^ ROTR(w[i-2],19) ^ (w[i-2] >> 10);
        w[i] = w[i-16] + s0 + w[i-7] + s1;
    }
    unsigned int a = c->h[0], b = c->h[1], cc = c->h[2], d = c->h[3];
    unsigned int e = c->h[4], f = c->h[5], g = c->h[6], hh = c->h[7];
    for (int i = 0; i < 64; i++) {
        unsigned int S1 = ROTR(e,6) ^ ROTR(e,11) ^ ROTR(e,25);
        unsigned int ch = (e & f) ^ (~e & g);
        unsigned int t1 = hh + S1 + ch + K256[i] + w[i];
        unsigned int S0 = ROTR(a,2) ^ ROTR(a,13) ^ ROTR(a,22);
        unsigned int maj = (a & b) ^ (a & cc) ^ (b & cc);
        unsigned int t2 = S0 + maj;
        hh = g; g = f; f = e; e = d + t1;
        d = cc; cc = b; b = a; a = t1 + t2;
    }
    c->h[0]+=a; c->h[1]+=b; c->h[2]+=cc; c->h[3]+=d;
    c->h[4]+=e; c->h[5]+=f; c->h[6]+=g; c->h[7]+=hh;
}

static void sha256_init(Sha256* c) {
    static const unsigned int iv[8] = {
        0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,0x510e527f,0x9b05688c,
        0x1f83d9ab,0x5be0cd19
    };
    memcpy(c->h, iv, sizeof(iv));
    c->len = 0; c->n = 0;
}

static void sha256_update(Sha256* c, const unsigned char* p, size_t n) {
    c->len += n;
    while (n > 0) {
        size_t take = 64 - (size_t)c->n;
        if (take > n) take = n;
        memcpy(c->buf + c->n, p, take);
        c->n += (int)take; p += take; n -= take;
        if (c->n == 64) { sha256_block(c, c->buf); c->n = 0; }
    }
}

static void sha256_final(Sha256* c, unsigned char out[32]) {
    unsigned long long bits = c->len * 8;
    c->buf[c->n++] = 0x80;
    if (c->n > 56) {
        while (c->n < 64) c->buf[c->n++] = 0;
        sha256_block(c, c->buf); c->n = 0;
    }
    while (c->n < 56) c->buf[c->n++] = 0;
    for (int i = 7; i >= 0; i--) c->buf[c->n++] = (unsigned char)(bits >> (i * 8));
    sha256_block(c, c->buf);
    for (int i = 0; i < 8; i++) {
        out[i*4]   = (unsigned char)(c->h[i] >> 24);
        out[i*4+1] = (unsigned char)(c->h[i] >> 16);
        out[i*4+2] = (unsigned char)(c->h[i] >> 8);
        out[i*4+3] = (unsigned char)c->h[i];
    }
}

int fb_hash_rgba(const FrameBuf* fb, unsigned char out32[32]) {
    Sha256 c;
    sha256_init(&c);
    sha256_update(&c, (const unsigned char*)fb->px, sizeof(fb->px));
    sha256_final(&c, out32);
    return 0;
}

int fb_write_ppm(const FrameBuf* fb, const char* path) {
    FILE* f = fopen(path, "wb");
    if (!f) return gfault_wrap(GFAULT_PAL_BAD_ARG);
    if (fprintf(f, "P6\n%d %d\n255\n", FB_W, FB_H) < 0) { fclose(f); return -1; }
    unsigned char row[FB_W * 3];
    for (int y = 0; y < FB_H; y++) {
        for (int x = 0; x < FB_W; x++) {
            row[x*3]   = fb->px[y][x].r;
            row[x*3+1] = fb->px[y][x].g;
            row[x*3+2] = fb->px[y][x].b;
        }
        if (fwrite(row, 1, sizeof(row), f) != sizeof(row)) { fclose(f); return -1; }
    }
    if (fclose(f) != 0) return -1;
    return 0;
}
