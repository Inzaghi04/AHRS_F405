#include "LowPassFilter.h"
static inline float sq(float x) { return x * x; }
static inline int is_zero(float x) {return fabsf(x) < 1e-7f;}

void DigitalBiquadFilter_init(DigitalBiquadFilter_float *filter)
{
    filter->delay_element_1 = 0.0f;
    filter->delay_element_2 = 0.0f;
}
float DigitalBiquadFilter_apply(DigitalBiquadFilter_float *filter, float sample, const biquad_params_t *params)
{
    if (params->cutoff_freq <= 0.0f || params->sample_freq <= 0.0f) {
        return sample;
    }
    float d0 = sample - filter->delay_element_1 * params->a1 - filter->delay_element_2 * params->a2;
    float out = d0 * params->b0 + filter->delay_element_1 * params->b1 + filter->delay_element_2 * params->b2;

    filter->delay_element_2 = filter->delay_element_1;
    filter->delay_element_1 = d0;

    return out;
}
void DigitalBiquadFilter_reset(DigitalBiquadFilter_float *filter, float value, const biquad_params_t *params)
{
    float init = value * (1.0f / (1.0f + params->a1 + params->a2));
    filter->delay_element_1 = init;
    filter->delay_element_2 = init;
}
void DigitalBiquadFilter_compute_params(float sample_freq, float cutoff_freq, biquad_params_t *params)
{
    params->cutoff_freq = cutoff_freq;
    params->sample_freq = sample_freq;

    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        params->b0 = 1.0f; params->b1 = params->b2 = params->a1 = params->a2 = 0.0f;
        return;
    }

    if (cutoff_freq > sample_freq * 0.4f) {
        cutoff_freq = sample_freq * 0.4f;
        params->cutoff_freq = cutoff_freq;
    }

    float fr  = sample_freq / cutoff_freq;
    float ohm = tanf(M_PI / fr);
    float c   = 1.0f + 2.0f * cosf(M_PI/4.0f) * ohm + ohm*ohm;

    params->b0 = ohm*ohm / c;
    params->b1 = 2.0f * params->b0;
    params->b2 = params->b0;
    params->a1 = 2.0f * (ohm*ohm - 1.0f) / c;
    params->a2 = (1.0f - 2.0f * cosf(M_PI/4.0f)*ohm + ohm*ohm) / c;
}

void LowPassFilter2pFloat_init(LowPassFilter2pFloat *f, float sample_freq, float cutoff_freq)
{
    DigitalBiquadFilter_compute_params(sample_freq, cutoff_freq, &f->params);
    DigitalBiquadFilter_init(&f->filter);
}
void LowPassFilter2pFloat_set_cutoff_frequency(LowPassFilter2pFloat *f, float sample_freq, float cutoff_freq)
{
    DigitalBiquadFilter_compute_params(sample_freq, cutoff_freq, &f->params);
    f->filter.delay_element_1 = f->filter.delay_element_2 = 0.0f;
}
float LowPassFilter2pFloat_apply(LowPassFilter2pFloat *f, float sample)
{
    return DigitalBiquadFilter_apply(&f->filter, sample, &f->params);
}
void LowPassFilter2pFloat_reset(LowPassFilter2pFloat *f)
{
    f->filter.delay_element_1 = 0.0f;
    f->filter.delay_element_2 = 0.0f;
}