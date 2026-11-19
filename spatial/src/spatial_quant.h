#ifndef SPATIAL_QUANT_H
#define SPATIAL_QUANT_H

/* Network codec for a mono source: 31 bits per source per frame instead of a
   3 x float position (96 bits).

   The layout is decision-aligned: the ITD code (6 bits) and IID level (6 bits)
   are sent explicitly, and 4 more azimuth bits give the position of the true
   angle inside the interval where both decisions stay unchanged. The receiver
   therefore re-derives exactly the sender's ITD/IID instead of a neighbour
   that happens to fall on the other side of a rounding boundary.

   Fields (LSB first):
     [0..13]  distance code, log-spaced 0.01 m .. 100 km
     [14]     back hemisphere flag (0 = front, 1 = behind)
     [15..20] ITD code, delay + 32, so -29..29 round-trips in 6 bits
     [21..26] IID level, 0..63
     [27..30] sub-cell azimuth index, 0..15

   Round-trip bounds from the sweep in main():
     |az' - az| <= (decision interval width) / 32. The bands are 0.02..8.3 deg
     wide over the sphere, so the bound is 0.26 deg and the measured maximum is
     the same; the cheap fallback is a uniform 11-bit azimuth grid at 0.18 deg,
     but that flips an ITD/IID decision on ~3 % of angles (see the control test).
     |d' - d| <= 0.21 % of d (log grid over 0.01 m .. 100 km).
     |gain' - gain| <= gain * 0.0021 (inverse-distance gain).
   All azimuth work is double so sender and receiver land on identical interval
   endpoints; only the final decoded angle is stored back as float. */

#include <stdint.h>
#include "binaural.h"

#define SQ_DIST_BITS 14
#define SQ_DIST_LEVELS (1u << SQ_DIST_BITS)          /* 16384 codes */
#define SQ_DIST_MIN 0.01
#define SQ_DIST_MAX 100000.0
#define SQ_DIST_LOG_RANGE 16.11809565095832          /* log(SQ_DIST_MAX / SQ_DIST_MIN) */

#define SQ_ITD_CODE_MAX 29                           /* lround(0.6 ms * 48 kHz) */
#define SQ_ITD_OFFSET 32                             /* fits -29..29 in 6 bits */
#define SQ_IID_BITS 6
#define SQ_IID_LEVELS BINAURAL_IID_LEVELS            /* 64 pan cells */
#define SQ_AZ_SUB_BITS 4
#define SQ_AZ_SUB (1 << SQ_AZ_SUB_BITS)              /* 16 sub-cells */
#define SQ_CODE_BITS 31
#define SQ_MAX_SOURCES 8
#define SQ_PACKET_BITS (1 + SQ_MAX_SOURCES * SQ_CODE_BITS)  /* sender count + codes */

_Static_assert(SQ_CODE_BITS == SQ_DIST_BITS + 1 + 6 + SQ_IID_BITS + SQ_AZ_SUB_BITS,
               "code layout must stay 31 bits");

typedef struct {
    float az;       /* canonical azimuth in (-pi, pi], same convention as binaural_azimuth */
    double dist;    /* metres */
    int itd;        /* reconstructed ITD decision, samples */
    int iid;        /* reconstructed IID decision, level 0..63 */
} SQuantized;

/* Decision maps. These call the same float helpers the renderer uses, and the
   cell edges come from the same helpers, so the code and the audio cannot
   drift apart. */
static inline int sq_itd_code(double az) {
    return binaural_itd_samples((float)az);
}
static inline int sq_iid_code(double az) {
    return binaural_iid_level((float)az);
}

/* u-interval of one ITD code / IID level, u = sin(az). The ITD end codes
   absorb the whole tail: at az = +-pi/2 the delay rounds to 29, but the
   decision bands are half-open and code 28 owns everything above 28.5/28.8. */
static inline void sq_itd_u_interval(int k, double* lo, double* hi) {
    *lo = binaural_itd_band_lo(k);
    *hi = binaural_itd_band_hi(k);
    if (k ==  SQ_ITD_CODE_MAX) *hi = 1.0;
    if (k == -SQ_ITD_CODE_MAX) *lo = -1.0;
    if (*lo < -1.0) *lo = -1.0;
    if (*hi >  1.0) *hi =  1.0;
}
static inline void sq_iid_u_interval(int level_in, double* lo, double* hi) {
    double glo = binaural_iid_gain_lo(level_in);
    double ghi = binaural_iid_gain_hi(level_in);
    *lo = 2.0 * glo * glo - 1.0;                      /* u = 2 t - 1, t = g^2 */
    *hi = 2.0 * ghi * ghi - 1.0;
    if (*lo < -1.0) *lo = -1.0;
    if (*hi >  1.0) *hi =  1.0;
}

/* u -> az. asin's domain is [0,1], but the interval endpoints come from cell
   arithmetic and can overshoot by an ulp; clamp rather than return NaN. */
static inline double sq_asin_clamped(double u) {
    if (u < -1.0) u = -1.0;
    if (u >  1.0) u =  1.0;
    return asin(u);
}
/* u interval -> azimuth interval on one hemisphere. Front covers
   [-pi/2, pi/2]; back is represented 2pi below (-3pi/2, -pi/2], which keeps a
   half-window from spanning both hemispheres. */
static inline void sq_az_bounds(double ulo, double uhi, int back,
                                double* alo, double* ahi) {
    if (!back) {
        *alo = sq_asin_clamped(ulo);
        *ahi = sq_asin_clamped(uhi);
    } else {
        *alo = -3.14159265358979323846 - sq_asin_clamped(uhi);
        *ahi = -3.14159265358979323846 - sq_asin_clamped(ulo);
    }
}

/* packed u-interval shared by the two decisions, in hemisphere `back` */
static inline int sq_interval(int itd, int iid, int back, double* alo, double* ahi) {
    double l1, h1, l2, h2;
    sq_itd_u_interval(itd, &l1, &h1);
    sq_iid_u_interval(iid, &l2, &h2);
    double ulo = l1 > l2 ? l1 : l2;
    double uhi = h1 < h2 ? h1 : h2;
    if (!(ulo < uhi)) return 0;
    sq_az_bounds(ulo, uhi, back, alo, ahi);
    return 1;
}

/* Position of `az` inside its hemisphere's interval, as a 0..1 fraction.
   The back branch shifts by -2pi; the caller only uses it against the same
   interval bounds, so the 2pi offset is consistent by construction. */
static inline double sq_pos_in_interval(double az, int back, double alo, double ahi) {
    double coord = back ? (az > 0.0 ? az - 6.28318530717958647692 : az) : az;
    return (coord - alo) / (ahi - alo);
}

/* Shortest-arc interpolation between two canonical azimuths, used to make the
   decoded trajectory continuous across the +-pi seam. */
static inline double sq_interp_az(double a0, double a1, double t) {
    double d = a1 - a0;
    if (d >  3.14159265358979323846) d -= 6.28318530717958647692;
    if (d < -3.14159265358979323846) d += 6.28318530717958647692;
    double a = a0 + d * t;
    if (a >  3.14159265358979323846) a -= 6.28318530717958647692;
    if (a < -3.14159265358979323846) a += 6.28318530717958647692;
    return a;
}

static inline uint32_t sq_encode_dist(double d) {
    if (d < SQ_DIST_MIN) d = SQ_DIST_MIN;
    if (d > SQ_DIST_MAX) d = SQ_DIST_MAX;
    double f = (log(d) - log(SQ_DIST_MIN)) / SQ_DIST_LOG_RANGE;
    long c = lround(f * (double)(SQ_DIST_LEVELS - 1));
    if (c < 0) c = 0;
    if (c > (long)(SQ_DIST_LEVELS - 1)) c = SQ_DIST_LEVELS - 1;
    return (uint32_t)c;
}
static inline double sq_decode_dist(uint32_t c) {
    return exp(log(SQ_DIST_MIN) + (double)c / (double)(SQ_DIST_LEVELS - 1) * SQ_DIST_LOG_RANGE);
}

static inline uint32_t sq_encode(double az, double d) {
    int itd = sq_itd_code(az);
    int iid = sq_iid_code(az);
    int back = cos(az) < 0.0;
    double alo = 0.0, ahi = 0.0;
    int sub = 0;
    if (sq_interval(itd, iid, back, &alo, &ahi)) {
        double f = sq_pos_in_interval(az, back, alo, ahi);
        int s = (int)floor(f * (double)SQ_AZ_SUB);
        if (s < 0) s = 0;
        if (s >= SQ_AZ_SUB) s = SQ_AZ_SUB - 1;
        sub = s;
    }
    uint32_t dc = sq_encode_dist(d);
    return dc | ((uint32_t)back << 14) | ((uint32_t)(itd + SQ_ITD_OFFSET) << 15)
            | ((uint32_t)iid << 21) | ((uint32_t)sub << 27);
}

static inline SQuantized sq_decode(uint32_t code) {
    uint32_t dc = code & (SQ_DIST_LEVELS - 1);
    int back = (int)((code >> 14) & 1u);
    int itd = (int)((code >> 15) & 63u) - SQ_ITD_OFFSET;
    int iid = (int)((code >> 21) & 63u);
    int sub = (int)((code >> 27) & 15u);
    double alo = 0.0, ahi = 0.0;
    double coord;
    if (sq_interval(itd, iid, back, &alo, &ahi)) {
        coord = alo + ((double)sub + 0.5) / (double)SQ_AZ_SUB * (ahi - alo);
    } else {
        coord = sq_asin_clamped(0.0);                 /* unreachable: grids overlap */
    }
    SQuantized q;
    q.az = (float)(back && coord < -3.14159265358979323846
                   ? coord + 6.28318530717958647692 : coord);
    q.dist = sq_decode_dist(dc);
    q.itd = itd;
    q.iid = iid;
    return q;
}

#endif
