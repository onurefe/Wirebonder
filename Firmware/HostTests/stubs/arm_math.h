#pragma once
#include <math.h>

typedef float float32_t;

#ifndef PI
#  define PI 3.14159265358979f
#endif

#ifdef __cplusplus
extern "C" {
#endif

static inline float32_t arm_sin_f32(float32_t x) { return sinf(x); }
static inline float32_t arm_cos_f32(float32_t x) { return cosf(x); }

#ifdef __cplusplus
}
#endif
