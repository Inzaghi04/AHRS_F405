#include "MadgwickAHRS.h"

#define sampleFreqDef   512.0f          // sample frequency in Hz
#define betaDef         0.1f            // 2 * proportional gain

static float invSqrt(float x)
{
  float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void Madgwick_Init(Madgwick *imu)
{
    imu->beta = betaDef;
    imu->q0 = 1.0f;
    imu->q1 = 0.0f;
    imu->q2 = 0.0f;
    imu->q3 = 0.0f;
    imu->invSampleFreq = 1.0f / sampleFreqDef;
    imu->anglesComputed = 0;
}

void Madgwick_Begin(Madgwick *imu, float sampleFreq)
{
    imu->invSampleFreq = 1.0f / sampleFreq;
}

void Madgwick_UpdateIMU(Madgwick *imu,
                        float gx, float gy, float gz,
                        float ax, float ay, float az)
{
  float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
  gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;
  qDot1 = 0.5f * (- imu->q1 * gx - imu->q2 * gy - imu->q3 * gz);
	qDot2 = 0.5f * (imu->q0 * gx + imu->q2 * gz - imu->q3 * gy);
	qDot3 = 0.5f * (imu->q0 * gy - imu->q1 * gz + imu->q3 * gx);
	qDot4 = 0.5f * (imu->q0 * gz + imu->q1 * gy - imu->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		float norm = ax*ax + ay*ay + az*az;
        if (norm < 1e-12f) return;  // hoặc continue;
        recipNorm = invSqrt(norm);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * imu->q0;
		_2q1 = 2.0f * imu->q1;
		_2q2 = 2.0f * imu->q2;
		_2q3 = 2.0f * imu->q3;
		_4q0 = 4.0f * imu->q0;
		_4q1 = 4.0f * imu->q1;
		_4q2 = 4.0f * imu->q2;
		_8q1 = 8.0f * imu->q1;
		_8q2 = 8.0f * imu->q2;
		q0q0 = imu->q0 * imu->q0;
		q1q1 = imu->q1 * imu->q1;
		q2q2 = imu->q2 * imu->q2;
		q3q3 = imu->q3 * imu->q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * imu->q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * imu->q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * imu->q3 - _2q1 * ax + 4.0f * q2q2 * imu->q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= imu->beta * s0;
		qDot2 -= imu->beta * s1;
		qDot3 -= imu->beta * s2;
		qDot4 -= imu->beta * s3;
	}
    imu->q0 += qDot1 * imu->invSampleFreq;
	imu->q1 += qDot2 * imu->invSampleFreq;
	imu->q2 += qDot3 * imu->invSampleFreq;
	imu->q3 += qDot4 * imu->invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(imu->q0 * imu->q0 + imu->q1 * imu->q1 + imu->q2 * imu->q2 + imu->q3 * imu->q3);
	imu->q0 *= recipNorm;
	imu->q1 *= recipNorm;
	imu->q2 *= recipNorm;
	imu->q3 *= recipNorm;
	imu->anglesComputed = 0;
}

void Madgwick_Update(Madgwick *imu,
                     float gx, float gy, float gz,
                     float ax, float ay, float az,
                     float mx, float my, float mz)
{
  float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		Madgwick_UpdateIMU(imu, gx, gy, gz, ax, ay, az);
		return;
	}
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (- imu->q1 * gx - imu->q2 * gy - imu->q3 * gz);
	qDot2 = 0.5f * (imu->q0 * gx + imu->q2 * gz - imu->q3 * gy);
	qDot3 = 0.5f * (imu->q0 * gy - imu->q1 * gz + imu->q3 * gx);
	qDot4 = 0.5f * (imu->q0 * gz + imu->q1 * gy - imu->q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		float norm = ax*ax + ay*ay + az*az;
        if (norm < 1e-12f) return;  // hoặc continue;
        recipNorm = invSqrt(norm);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * imu->q0 * mx;
		_2q0my = 2.0f * imu->q0 * my;
		_2q0mz = 2.0f * imu->q0 * mz;
		_2q1mx = 2.0f * imu->q1 * mx;
		_2q0 = 2.0f * imu->q0;
		_2q1 = 2.0f * imu->q1;
		_2q2 = 2.0f * imu->q2;
		_2q3 = 2.0f * imu->q3;
		_2q0q2 = 2.0f * imu->q0 * imu->q2;
		_2q2q3 = 2.0f * imu->q2 * imu->q3;
		q0q0 = imu->q0 * imu->q0;
		q0q1 = imu->q0 * imu->q1;
		q0q2 = imu->q0 * imu->q2;
		q0q3 = imu->q0 * imu->q3;
		q1q1 = imu->q1 * imu->q1;
		q1q2 = imu->q1 * imu->q2;
		q1q3 = imu->q1 * imu->q3;
		q2q2 = imu->q2 * imu->q2;
		q2q3 = imu->q2 * imu->q3;
		q3q3 = imu->q3 * imu->q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * imu->q3 + _2q0mz * imu->q2 + mx * q1q1 + _2q1 * my * imu->q2 + _2q1 * mz * imu->q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * imu->q3 + my * q0q0 - _2q0mz * imu->q1 + _2q1mx * imu->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * imu->q3 - my * q3q3;
		_2bx = sqrtf(hx * hx + hy * hy);
		_2bz = -_2q0mx * imu->q2 + _2q0my * imu->q1 + mz * q0q0 + _2q1mx * imu->q3 - mz * q1q1 + _2q2 * my * imu->q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * imu->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * imu->q3 + _2bz * imu->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * imu->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * imu->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * imu->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * imu->q2 + _2bz * imu->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * imu->q3 - _4bz * imu->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * imu->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * imu->q2 - _2bz * imu->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * imu->q1 + _2bz * imu->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * imu->q0 - _4bz * imu->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * imu->q3 + _2bz * imu->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * imu->q0 + _2bz * imu->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * imu->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= imu->beta * s0;
		qDot2 -= imu->beta * s1;
		qDot3 -= imu->beta * s2;
		qDot4 -= imu->beta * s3;
	}
	// Integrate rate of change of quaternion to yield quaternion
	imu->q0 += qDot1 * imu->invSampleFreq;
	imu->q1 += qDot2 * imu->invSampleFreq;
	imu->q2 += qDot3 * imu->invSampleFreq;
	imu->q3 += qDot4 * imu->invSampleFreq;

	// Normalise quaternion
	recipNorm = invSqrt(imu->q0 * imu->q0 + imu->q1 * imu->q1 + imu->q2 * imu->q2 + imu->q3 * imu->q3);
	imu->q0 *= recipNorm;
	imu->q1 *= recipNorm;
	imu->q2 *= recipNorm;
	imu->q3 *= recipNorm;
	imu->anglesComputed = 0;
}

void Madgwick_ComputeAngles(Madgwick *imu)
{
	imu->roll = atan2f(imu->q0*imu->q1 + imu->q2*imu->q3, 0.5f - imu->q1*imu->q1 - imu->q2*imu->q2);
	imu->pitch = asinf(-2.0f * (imu->q1*imu->q3 - imu->q0*imu->q2));
	imu->yaw = atan2f(imu->q1*imu->q2 + imu->q0*imu->q3, 0.5f - imu->q2*imu->q2 - imu->q3*imu->q3);
	imu->anglesComputed = 1;
}
float Madgwick_GetRoll(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->roll * 57.29578f;
}
float Madgwick_GetPitch(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->pitch * 57.29578f;
}
float Madgwick_GetYaw(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->yaw * 57.29578f;
}
float Madgwick_GetRollRad(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->roll;
}
float Madgwick_GetPitchRad(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->pitch;
}
float Madgwick_GetYawRad(Madgwick *imu)
{
    if (!imu->anglesComputed) Madgwick_ComputeAngles(imu);
    return imu->yaw;
}