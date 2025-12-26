#ifndef __EKF_H__
#define __EKF_H__

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "ftype.h"
#include "vector3.h"
#include "matrix3.h"

#define EKF_STATE_DIM 7 
#define EKF_MEAS_DIM  3
#define EKF_P_SIZE    (EKF_STATE_DIM * EKF_STATE_DIM)
#define DEG_TO_RAD 0.0174532925f
#define RAD_TO_DEG 57.2957795f
#define GRAVITY_MSS 9.80665f

/* Struct ánh x? State Vector */
typedef struct {
	float q0, q1, q2, q3; // Quaternion
	float bx, by, bz;     // Gyro Bias
	
} EKF_State_Map_t;

typedef union {
    float vec[EKF_STATE_DIM];
    EKF_State_Map_t s;
} EKF_State_t;

/* C?u hình Tuning & Gating */
typedef struct {
    // Noise co b?n
    float sigma_gyro;       // Rad/s
    float sigma_accel;      // m/s^2
    float sigma_mag;        // Normalized
    float sigma_bias;       // Bias instability

    // Dynamic Noise Scaling
    float q_scale_dynamic;  // H? s? tang Q khi quay nhanh
    float r_acc_scale;      // H? s? tang R Accel khi rung d?ng/gia t?c l?n

    // Innovation Gating (Mahalanobis distance thresholds)
    float gate_accel;       // Ví d?: 5.0 (sigma)
    float gate_mag;         // Ví d?: 5.0 (sigma)
	
		// ZUPT Parameters - M?I THÊM
    float zupt_acc_threshold;    // m/s² - Base threshold
    float zupt_gyro_threshold;   // rad/s - Base threshold
    float zupt_variance_scale;   // H? s? scale threshold theo variance
    float zupt_decay_tau;        // Time constant cho velocity decay (giây)
    float zupt_pos_decay_factor; // Position decay factor khi stationary
} EKF_Config_t;

typedef struct {
    uint16_t stationary_counter;
    float pos_at_stationary[3];  // Luu v? trí khi b?t d?u d?ng yên
    
    // Adaptive thresholds
    float acc_history[10];
    float gyro_history[10];
    uint8_t hist_idx;
    float acc_variance;
    float gyro_variance;
    
    bool was_stationary;  // Ð? detect transition
} ZUPT_State_t;

/* Struct chính ch?a toàn b? d? li?u EKF */
typedef struct {
    // --- State & Covariance ---
    EKF_State_t x;
    float P[EKF_P_SIZE]; 
    
    // --- Reference Vectors (NED Frame) ---
    Vector3f mag_ref;       // Vector t? tru?ng Trái d?t tham chi?u (Chu?n hóa)
    // --- Buffers (Tránh dùng Stack/Heap) ---
    float F[EKF_P_SIZE];          // 7x7
    float Q[EKF_P_SIZE];          // 7x7
    float P_temp[EKF_P_SIZE];     // 7x7
    float PP_temp[EKF_P_SIZE];    // 7x7
    
    float K[EKF_STATE_DIM * EKF_MEAS_DIM];  // 7x3
    float H[EKF_MEAS_DIM * EKF_STATE_DIM];  // 3x7
    float Ht[EKF_STATE_DIM * EKF_MEAS_DIM]; // 7x3
    float S[EKF_MEAS_DIM * EKF_MEAS_DIM];   // 3x3

    // --- Status Counters ---
    uint16_t accel_reject_count;
    uint16_t mag_reject_count;
		
		float vx_body, vy_body, vz_body;
		float px_body, py_body, pz_body;
		float vn, ve, vd;       // V?n t?c (m/s)
		float pn, pe, pd;       // V? trí (m) - M?I THÊM
		float abx, aby, abz;
      Vector3f accel_ned_prev; // Luu gia t?c cu d? tích phân hình thang
    bool is_stationary;     // C? báo d?ng yên (ZUPT)
		
		ZUPT_State_t zupt;
    // --- System ---
    float dt;
    bool initialized;
    EKF_Config_t cfg;

} EKF_Handle_t;

/* API Functions */
void EKF_Init(EKF_Handle_t *ekf, float dt);

// C?n g?i hàm này 1 l?n sau khi Calib Mag d? bi?t hu?ng t? tru?ng t?i noi bay
void EKF_SetMagReference(EKF_Handle_t *ekf, Vector3f mag_ned_normalized);
void EKF_PredictVelocity(EKF_Handle_t *ekf, Vector3f accel_mss, Vector3f gyro_rad);
void EKF_Predict(EKF_Handle_t *ekf, Vector3f gyro_rad, Vector3f accel_mss);
void EKF_FuseAccel(EKF_Handle_t *ekf, Vector3f accel_mss);
void EKF_FuseMag(EKF_Handle_t *ekf, Vector3f mag_norm); // 3D Fusion
void EKF_GetEuler(EKF_Handle_t *ekf, float *roll, float *pitch, float *yaw);
void EKF_GetQuaternion(EKF_Handle_t *ekf, float *q0, float *q1, float *q2, float *q3);
bool EKF_IsHealthy(EKF_Handle_t *ekf);
void EKF_GetPosition(EKF_Handle_t *ekf, float *pn, float *pe, float *pd);
void EKF_ApplyZUPT(EKF_Handle_t *ekf, float acc_mag, float gyro_mag);
#endif /* __EKF_H__ */