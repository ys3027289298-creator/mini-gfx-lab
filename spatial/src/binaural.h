#ifndef BINAURAL_H
#define BINAURAL_H

/* Shared binaural decision code.
   The float renderer and the network quantizer both call these functions, so
   "quantization must not change the ITD/IID decision" is a statement about one
   implementation, not about two formulas that happen to look alike.

   The decision functions compute in double internally even though the input is
   a float angle: a float32 azimuth carries ~24 bits of mantissa, which is
   enough to land exactly on a rounding boundary, and double keeps the sender
   and the receiver on the same side of it. */
#include <math.h>

#define BINAURAL_SR 48000
#define BINAURAL_ITD_MAX (0.0006 * BINAURAL_SR)    /* 0.6 ms, Woodworth-lite */
#define BINAURAL_IID_LEVELS 64                     /* pan grid for the IID decision */

/* azimuth in radians, 0 = front, +pi/2 = right, +-pi = behind */
static inline float binaural_azimuth(float dx, float dz, float yaw) {
    float c = cosf(yaw), sn = sinf(yaw);
    float xr =  c * dx - sn * dz;
    float zr =  sn * dx + c * dz;
    return atan2f(xr, -zr);
}

/* integer delay in samples on each side of the head, and the azimuth whose
   rounding boundary sits halfway between two integer delays (the positive
   half-band, used by the quantizer to enumerate decision cells) */
static inline int binaural_itd_samples(float az) {
    long v = lround(sin((double)az) * BINAURAL_ITD_MAX);
    if (v > 29) v = 29;
    if (v < -29) v = -29;
    return (int)v;
}
static inline double binaural_itd_band_lo(int code) {
    return ((double)code - 0.5) / BINAURAL_ITD_MAX;
}
static inline double binaural_itd_band_hi(int code) {
    return ((double)code + 0.5) / BINAURAL_ITD_MAX;
}

/* continuous panning gains */
static inline void binaural_iid_gains(float az, float* gl, float* gr) {
    float t = (sinf(az) + 1.0f) * 0.5f;      /* 0 = hard left, 1 = hard right */
    *gl = sqrtf(1.0f - t);
    *gr = sqrtf(t);
}

/* right-ear gain quantized to the pan grid: this is the IID decision a packet
   has to reproduce, while the renderer keeps using the continuous gains.
   binaural_iid_gain_lo/hi give the gain-cell edges (half a level below and
   above); the quantizer squares them to get the u = sin(az) cell. */
static inline double binaural_iid_gain_of(float az) {
    return sqrt((sin((double)az) + 1.0) * 0.5);
}
static inline int binaural_iid_level(float az) {
    long v = lround(binaural_iid_gain_of(az) * (double)(BINAURAL_IID_LEVELS - 1));
    if (v < 0) return 0;
    if (v > BINAURAL_IID_LEVELS - 1) return BINAURAL_IID_LEVELS - 1;
    return (int)v;
}
static inline double binaural_iid_gain_lo(int level_in) {
    double g = ((double)level_in - 0.5) / (double)(BINAURAL_IID_LEVELS - 1);
    return g < 0.0 ? 0.0 : g;
}
static inline double binaural_iid_gain_hi(int level_in) {
    double g = ((double)level_in + 0.5) / (double)(BINAURAL_IID_LEVELS - 1);
    return g > 1.0 ? 1.0 : g;
}

static inline float binaural_distance_gain(float x, float y, float z) {
    float d = sqrtf(x * x + y * y + z * z);
    return 1.0f / (1.0f + d);
}

#endif
