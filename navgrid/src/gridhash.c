#include "gridhash.h"

/* splitmix64 finalizer：固定的整数混合，任何平台上结果逐位相同。 */
static uint64_t mix64(uint64_t z) {
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
    z = z ^ (z >> 31);
    return z;
}

uint64_t gh_cell_token(uint32_t x, uint32_t y, uint32_t v) {
    uint64_t z = mix64(((uint64_t)x << 32) | (uint64_t)y);
    z ^= (uint64_t)v + 0x9E3779B97F4A7C15ULL;
    return mix64(z);
}

uint64_t gh_hash_full_strided(uint32_t w, uint32_t h,
                              const unsigned char* cells, size_t stride) {
    /* 维度混入种子，避免不同尺寸地图因格子序列前缀相同而碰撞。 */
    uint64_t acc = mix64(0xC0FFEE11ULL ^ ((uint64_t)w << 32) ^ (uint64_t)h);
    for (uint32_t y = 0; y < h; y++)
        for (uint32_t x = 0; x < w; x++)
            acc += gh_cell_token(x, y, cells[(size_t)y * stride + x]);
    return acc;
}

uint64_t gh_hash_full(uint32_t w, uint32_t h, const unsigned char* cells) {
    return gh_hash_full_strided(w, h, cells, w);
}

void gh_init(GridHash* gh, uint32_t w, uint32_t h, const unsigned char* cells) {
    gh->w = w; gh->h = h;
    gh->acc = gh_hash_full(w, h, cells);
}

void gh_on_cell_change(GridHash* gh, uint32_t x, uint32_t y,
                       unsigned char old_v, unsigned char new_v) {
    gh->acc += gh_cell_token(x, y, new_v) - gh_cell_token(x, y, old_v);
}

uint64_t gh_digest(const GridHash* gh) { return gh->acc; }

void gh_to_hex(uint64_t digest, char out[17]) {
    static const char HEX[] = "0123456789abcdef";
    for (int i = 0; i < 16; i++)
        out[i] = HEX[(digest >> (60 - 4 * i)) & 0xF];
    out[16] = '\0';
}

int gh_from_hex(const char hex[16], uint64_t* out) {
    uint64_t d = 0;
    for (int i = 0; i < 16; i++) {
        char c = hex[i];
        uint64_t nib;
        if (c >= '0' && c <= '9') nib = (uint64_t)(c - '0');
        else if (c >= 'a' && c <= 'f') nib = (uint64_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') nib = (uint64_t)(c - 'A' + 10);
        else return -1;
        d = (d << 4) | nib;
    }
    *out = d;
    return 0;
}
