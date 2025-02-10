#ifndef COMPLEXF_H
#define COMPLEXF_H

#include <math.h>

/* Define a complex number type with 32-bit floats */
typedef struct {
    float re;
    float im;
} complexf;

/* Create a complex number from real and imaginary parts */
static inline complexf complexf_create(float re, float im) {
    return (complexf){ re, im };
}

/* Addition: (a.re + b.re, a.im + b.im) */
static inline complexf complexf_add(complexf a, complexf b) {
    return (complexf){ a.re + b.re, a.im + b.im };
}

/* Subtraction: (a.re - b.re, a.im - b.im) */
static inline complexf complexf_sub(complexf a, complexf b) {
    return (complexf){ a.re - b.re, a.im - b.im };
}

/* Multiplication: (a.re*b.re - a.im*b.im, a.re*b.im + a.im*b.re) */
static inline complexf complexf_mul(complexf a, complexf b) {
    return (complexf){ a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re };
}

/* Division: a / b = (a * conj(b)) / |b|^2 */
static inline complexf complexf_div(complexf a, complexf b) {
    float denom = b.re * b.re + b.im * b.im;
    return (complexf){
        (a.re * b.re + a.im * b.im) / denom,
        (a.im * b.re - a.re * b.im) / denom
    };
}

/* Magnitude (absolute value): sqrt(re^2 + im^2) */
static inline float complexf_abs(complexf z) {
    return sqrtf(z.re * z.re + z.im * z.im);
}

/* Squared magnitude: (re^2 + im^2) */
static inline float complexf_abs2(complexf z) {
    return z.re * z.re + z.im * z.im;
}

/* Complex conjugate: (re, -im) */
static inline complexf complexf_conj(complexf z) {
    return (complexf){ z.re, -z.im };
}

/* Scale a complex number by a real factor */
static inline complexf complexf_scale(complexf z, float s) {
    return (complexf){ z.re * s, z.im * s };
}

/* Compute exp(j*theta) = cos(theta) + j*sin(theta) */
static inline complexf complexf_exp(float theta) {
    return (complexf){ cosf(theta), sinf(theta) };
}

#endif // COMPLEXF_H
