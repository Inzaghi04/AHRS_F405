#ifndef __MADGWICK_AHRS_H
#define __MADGWICK_AHRS_H
#include <math.h>
#include <stdint.h>
#include <string.h>
typedef struct {
    float beta;				// algorithm gain
    float q0;
    float q1;
    float q2;
    float q3;	// quaternion of sensor frame relative to auxiliary frame
    float invSampleFreq;
    float roll;
    float pitch;
    float yaw;
    uint8_t anglesComputed;
} Madgwick;

void Madgwick_Init(Madgwick *imu);
void Madgwick_Begin(Madgwick *imu, float sampleFreq);

void Madgwick_Update(Madgwick *imu,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz);

void Madgwick_UpdateIMU(Madgwick *imu,
                        float gx, float gy, float gz,
                        float ax, float ay, float az);
void Madgwick_ComputeAngles(Madgwick *imu);
// getters
float Madgwick_GetRoll(Madgwick *imu);
float Madgwick_GetPitch(Madgwick *imu);
float Madgwick_GetYaw(Madgwick *imu);

float Madgwick_GetRollRad(Madgwick *imu);
float Madgwick_GetPitchRad(Madgwick *imu);
float Madgwick_GetYawRad(Madgwick *imu);

#endif