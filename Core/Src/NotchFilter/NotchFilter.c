#include "NotchFilter.h"
#define M_PI 3.14159265358979323846f
#define NOTCH_MAX_SLEW	0.05f;
#define NOTCH_MAX_SLEW_LOWER	1.0f - NOTCH_MAX_SLEW;
#define NOTCH_MAX_SLEW_UPPER	1.0f / NOTCH_MAX_SLEW_LOWER;
#define sq(x) ((x) * (x))

void NotchFilter_init(NotchFilter_t* filter, float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    filter->initialised = false;
    if ((center_freq_hz > 0.5f * bandwidth_hz) && (center_freq_hz < 0.5f * sample_freq_hz)) {
        float A, Q;
        NotchFilter_calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, &A, &Q);
        NotchFilter_init_with_A_and_Q(filter, sample_freq_hz, center_freq_hz, A, Q);
    }
}
void NotchFilter_init_with_A_and_Q(NotchFilter_t* filter, float sample_freq_hz, float center_freq_hz, float A, float Q)
{
    if (filter->initialised &&
        (center_freq_hz == filter->center_freq_hz) &&
        (sample_freq_hz == filter->sample_freq_hz) &&
        (A == filter->A)) {
        return;
    }

    float new_center_freq = center_freq_hz;

    if (filter->initialised && !filter->need_reset && (filter->center_freq_hz != 0.0f)) {
        float lower_limit = filter->center_freq_hz * NOTCH_MAX_SLEW_LOWER;
        float upper_limit = filter->center_freq_hz * NOTCH_MAX_SLEW_UPPER;
        if (new_center_freq < lower_limit) {
            new_center_freq = lower_limit;
        } else if (new_center_freq > upper_limit) {
            new_center_freq = upper_limit;
        }
    }

    if ((new_center_freq > 0.0f) && (new_center_freq < 0.5f * sample_freq_hz) && (Q > 0.0f)) {
        float omega = 2.0f * M_PI * new_center_freq / sample_freq_hz;
        float alpha = sinf(omega) / (2.0f * Q);

        filter->b0 =  1.0f + alpha*sq(A);
        filter->b1 = -2.0f * cosf(omega);
        filter->b2 =  1.0f - alpha*sq(A);
        filter->a1 = filter->b1;
        filter->a2 =  1.0f - alpha;

        const float a0_inv =  1.0f/(1.0f + alpha);

        filter->b0 *= a0_inv;
        filter->b1 *= a0_inv;
        filter->b2 *= a0_inv;
        filter->a1 *= a0_inv;
        filter->a2 *= a0_inv;

        filter->center_freq_hz = new_center_freq;
        filter->sample_freq_hz = sample_freq_hz;
        filter->A = A;
        filter->initialised = true;
    } else {
        filter->initialised = false;
    }
}
float NotchFilter_apply(NotchFilter_t* filter, float sample)
{
    if (!filter->initialised || filter->need_reset) {
        filter->ntchsig1 = sample;
        filter->ntchsig2 = sample;
        filter->signal1 = sample;
        filter->signal2 = sample;
        filter->need_reset = false;
        return sample;
    }
    float output = sample*filter->b0 + filter->ntchsig1*filter->b1 + filter->ntchsig2*filter->b2 - filter->signal1*filter->a1 - filter->signal2*filter->a2;
    filter->ntchsig2 = filter->ntchsig1;
    filter->ntchsig1 = sample;
    filter->signal2 = filter->signal1;
    filter->signal1 = output;
    return output;
}
void NotchFilter_reset(NotchFilter_t* filter)
{
    filter->need_reset = true;
}
void NotchFilter_calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float* A, float* Q)
{
    *A = powf(10.0f, -attenuation_dB / 40.0f);
    if (center_freq_hz > 0.5f * bandwidth_hz) {
        const float octaves = log2f(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0f));
        *Q = sqrtf(powf(2.0f, octaves)) / (powf(2.0f, octaves) - 1.0f);
    } else {
        *Q = 0.0f;
    }
}
void NotchFilter_disable(NotchFilter_t* filter)
{
    filter->initialised = false;
}
