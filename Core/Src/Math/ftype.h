#ifndef __FTYPE_H__
#define __FTYPE_H__

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string.h>

typedef float ftype;

#define acosF(x) acosf(x)
#define asinF(x) asinf(x)
#define sinF(x) sinf(x)
#define cosF(x) cosf(x)
#define tanF(x) tanf(x)
#define atanF(x) atanf(x)
#define atan2F(y,x) atan2f(y,x)
#define sqrtF(x) sqrtf(x)
#define fmaxF(x,y) fmaxf(x,y)
#define powF(x,y) powf(x,y)
#define logF(x) logf(x)
#define fabsF(x) fabsf(x)
#define ceilF(x) ceilf(x)
#define fminF(x,y) fminf(x,y)
#define fmodF(x,y) fmodf(x,y)
#define fabsF(x) fabsf(x)
#define toftype tofloat
#define FLT_EPSILON      1.192092896e-07F    
#define M_PI            3.14159265358979323846f  /* pi */
#define MIN(a, b)       ((a) < (b) ? (a) : (b))
#define ZERO_FARRAY(a) memset(a, 0, sizeof(a))

/*
 * @brief: Check whether a float is zero
 */
static inline bool is_zero(const float x) {
    return fabsf(x) < FLT_EPSILON;
}

static inline bool is_equal(const float x, const float y) {
    return fabsf(x - y) < FLT_EPSILON;
}

static inline bool is_positive(const float x) {
    return x > FLT_EPSILON;
}
static inline float radians(const float degrees) {
    return degrees * (M_PI / 180.0f);
}
/*
 * @brief: Check whether a double is zero
 */

#endif
