#ifndef NOTCHFILTER_H
#define NOTCHFILTER_H
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
typedef struct 
{
    /* data */
    bool initialised;
    bool need_reset;
    float b0, b1, b2, a1, a2;
    float center_freq_hz, sample_freq_hz, A;
    float ntchsig1, ntchsig2, signal2, signal1;
} NotchFilter_t;
void NotchFilter_init(NotchFilter_t* filter, float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
void NotchFilter_init_with_A_and_Q(NotchFilter_t* filter, float sample_freq_hz, float center_freq_hz, float A, float Q);
float NotchFilter_apply(NotchFilter_t* filter, float sample);
void NotchFilter_reset(NotchFilter_t* filter);
void NotchFilter_calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float* A, float* Q);
void NotchFilter_disable(NotchFilter_t* filter);

typedef struct
{
    uint8_t enable;
    float center_freq_hz;
    float bandwidth_hz;
    float attenuation_dB;

} NotchFilterParams;

// setters/getters
static inline float center_freq(const NotchFilterParams *p)
{
    return p->center_freq_hz;
}

static inline float bandwidth_hz(const NotchFilterParams *p)
{
    return p->bandwidth_hz;
}

static inline float attenuation_dB(const NotchFilterParams *p)
{
    return p->attenuation_dB;
}

static inline uint8_t enabled(const NotchFilterParams *p)
{
    return p->enable;
}

static inline void enable(NotchFilterParams *p)
{
    p->enable = 1;
}
#endif