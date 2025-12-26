#ifndef __LOW_PASS_FILTER_H
#define __LOW_PASS_FILTER_H
#include <stdint.h>
#include <string.h>
#include <math.h>
#define M_PI 3.14159265358979323846f
typedef struct 
{
    float cutoff_freq;
    float sample_freq;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;
} biquad_params_t;

typedef struct 
{
    biquad_params_t params;
    float delay_element_1;
    float delay_element_2;
} DigitalBiquadFilter_float;

typedef struct 
{
    DigitalBiquadFilter_float filter;
    biquad_params_t params;
} LowPassFilter2pFloat;

void DigitalBiquadFilter_init(DigitalBiquadFilter_float *filter);
float DigitalBiquadFilter_apply(DigitalBiquadFilter_float *filter, float sample, const biquad_params_t *params);
void DigitalBiquadFilter_reset(DigitalBiquadFilter_float *filter, float value, const biquad_params_t *params);
void DigitalBiquadFilter_compute_params(float sample_freq, float cutoff_freq, biquad_params_t *params);

void LowPassFilter2pFloat_init(LowPassFilter2pFloat *f, float sample_freq, float cutoff_freq);
void LowPassFilter2pFloat_set_cutoff_frequency(LowPassFilter2pFloat *f, float sample_freq, float cutoff_freq);
float LowPassFilter2pFloat_apply(LowPassFilter2pFloat *f, float sample);
void LowPassFilter2pFloat_reset(LowPassFilter2pFloat *f);
#endif
