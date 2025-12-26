#include "EKF.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif



/* =========================================================================
 * MATRIX HELPERS (Internal)
 * ========================================================================= */
static void MatMul(float *C, const float *A, const float *B, int m, int n, int k) {
    memset(C, 0, m * k * sizeof(float));
    for (int i = 0; i < m; i++) {
        const float *pA = &A[i * n];
        for (int j = 0; j < k; j++) {
            float sum = 0.0f;
            for (int p = 0; p < n; p++) {
                sum += pA[p] * B[p * k + j];
            }
            C[i * k + j] = sum;
        }
    }
}

static void MatMulTransB(float *C, const float *A, const float *B, int m, int n, int k) {
    memset(C, 0, m * k * sizeof(float));
    for (int i = 0; i < m; i++) {
        const float *pA = &A[i * n];
        for (int j = 0; j < k; j++) {
            float sum = 0.0f;
            const float *pB = &B[j * n]; // Access row j of B (which acts as col j of B')
            for (int p = 0; p < n; p++) {
                sum += pA[p] * pB[p]; 
            }
            C[i * k + j] = sum;
        }
    }
}

static void MatAdd(float *C, const float *A, const float *B, int len) {
    for (int i = 0; i < len; i++) C[i] = A[i] + B[i];
}

static void MatSub(float *C, const float *A, const float *B, int len) {
    for (int i = 0; i < len; i++) C[i] = A[i] - B[i];
}

static void NormalizeQuaternion(float *x) {
    float norm = sqrtF(x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3]);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        x[0] *= inv; x[1] *= inv; x[2] *= inv; x[3] *= inv;
    } else {
        x[0] = 1.0f; x[1] = 0.0f; x[2] = 0.0f; x[3] = 0.0f;
    }
}

/* =========================================================================
 * CORE FUNCTIONS
 * ========================================================================= */

void EKF_Init(EKF_Handle_t *ekf, float dt) {
    memset(ekf, 0, sizeof(EKF_Handle_t));
    ekf->dt = dt;
    
    ekf->x.s.q0 = 1.0f;
    
    // Default Tuning
    ekf->cfg.sigma_gyro  = 0.002f;  
    ekf->cfg.sigma_accel = 0.1f;    
    ekf->cfg.sigma_mag   = 0.05f;   
    ekf->cfg.sigma_bias  = 1.0e-5f; // Gyro bias noise
    
    // --- ACCEL BIAS TUNING ---
    // Bias accel thay đổi rất chậm (Random Walk)

    ekf->cfg.q_scale_dynamic = 5.0f;
    ekf->cfg.r_acc_scale = 100.0f;
    ekf->cfg.gate_accel = 5.0f;
    ekf->cfg.gate_mag = 5.0f;
    
    // ZUPT Params (Giữ nguyên)
    ekf->cfg.zupt_acc_threshold = 0.3f;      
    ekf->cfg.zupt_gyro_threshold = 0.02f;    
    ekf->cfg.zupt_variance_scale = 10.0f;    
    ekf->cfg.zupt_decay_tau = 1.5f;          
    ekf->cfg.zupt_pos_decay_factor = 0.95f;  
    
    ekf->mag_ref.x = 1.0f; ekf->mag_ref.y = 0.0f; ekf->mag_ref.z = 0.0f;

    // Initial Covariance
    for (int i = 0; i < EKF_STATE_DIM; i++) 
        ekf->P[i * EKF_STATE_DIM + i] = 0.001f; 
    

    ekf->initialized = true;
}

/* Trong EKF.c */
void EKF_ApplyZUPT(EKF_Handle_t *ekf, float acc_mag, float gyro_mag) {
    ZUPT_State_t *zupt = &ekf->zupt;
    
    // 1. Update History
    zupt->acc_history[zupt->hist_idx] = acc_mag;
    zupt->gyro_history[zupt->hist_idx] = gyro_mag;
    zupt->hist_idx = (zupt->hist_idx + 1) % 10;
    
    // 2. Calc Variance
    float acc_mean = 0, gyro_mean = 0;
    for (int i = 0; i < 10; i++) {
        acc_mean += zupt->acc_history[i];
        gyro_mean += zupt->gyro_history[i];
    }
    acc_mean /= 10.0f;
    gyro_mean /= 10.0f;
    
    zupt->acc_variance = 0;
    zupt->gyro_variance = 0;
    for (int i = 0; i < 10; i++) {
        zupt->acc_variance += (zupt->acc_history[i] - acc_mean) * (zupt->acc_history[i] - acc_mean);
        zupt->gyro_variance += (zupt->gyro_history[i] - gyro_mean) * (zupt->gyro_history[i] - gyro_mean);
    }
    zupt->acc_variance /= 10.0f;
    zupt->gyro_variance /= 10.0f;
    
    // 3. Thresholds
    float acc_thresh = ekf->cfg.zupt_acc_threshold + zupt->acc_variance * ekf->cfg.zupt_variance_scale;
    float gyro_thresh = ekf->cfg.zupt_gyro_threshold + sqrtf(zupt->gyro_variance);
    
    // 4. Check Stationary
    bool is_stationary_now = (fabsf(acc_mag - GRAVITY_MSS) < acc_thresh) && (gyro_mag < gyro_thresh);
    
    if (is_stationary_now) {
        if (zupt->stationary_counter < 60000) zupt->stationary_counter++;
    } else {
        zupt->stationary_counter = 0;
        ekf->is_stationary = false;
        zupt->was_stationary = false;
    }
    
    // 5. Apply ZUPT & Position Lock
    // Counter > 20 tuc la dung yen 100ms (tai 200Hz)
    if (zupt->stationary_counter > 20) {
        
        // Capture position on first transition
        if (!zupt->was_stationary) {
            zupt->was_stationary = true;
            zupt->pos_at_stationary[0] = ekf->pn;
            zupt->pos_at_stationary[1] = ekf->pe;
            zupt->pos_at_stationary[2] = ekf->pd;
        }
        
        ekf->is_stationary = true;

        // --- HARD ZUPT: Force Velocity to 0 ---
        ekf->vn = 0.0f;
        ekf->ve = 0.0f;
        ekf->vd = 0.0f;
        
        ekf->vx_body = 0.0f;
        ekf->vy_body = 0.0f;
        ekf->vz_body = 0.0f;

        // --- POSITION LOCK: Prevent drift ---
        // Force current pos to stored reference
        ekf->pn = zupt->pos_at_stationary[0];
        ekf->pe = zupt->pos_at_stationary[1];
        ekf->pd = zupt->pos_at_stationary[2];
    }
}
void EKF_SetMagReference(EKF_Handle_t *ekf, Vector3f mag_ned_normalized) {
    ekf->mag_ref = mag_ned_normalized;
}

// Helper: Tính đạo hàm Quaternion: q_dot = 0.5 * q * omega
static void get_q_dot(float *q_dot, const float *q, float wx, float wy, float wz) {
    q_dot[0] = 0.5f * (-q[1]*wx - q[2]*wy - q[3]*wz);
    q_dot[1] = 0.5f * ( q[0]*wx - q[3]*wy + q[2]*wz);
    q_dot[2] = 0.5f * ( q[3]*wx + q[0]*wy - q[1]*wz);
    q_dot[3] = 0.5f * (-q[2]*wx + q[1]*wy + q[0]*wz);
}
void EKF_PredictVelocity(EKF_Handle_t *ekf, Vector3f accel_mss, Vector3f gyro_rad) {
    float dt = ekf->dt;
    
    // ========== BƯỚC 1: TRỪ BIAS ==========
    float accel_x = accel_mss.x;
    float accel_y = accel_mss.y;
    float accel_z = accel_mss.z;    
    float gyro_x = gyro_rad.x;
    float gyro_y = gyro_rad.y;
		float gyro_z = gyro_rad.z;    
    // ========== BƯỚC 2: TÍNH DELTA ANGLE (delAng) ==========
    // delAng = gyro * dt (góc quay trong khoảng dt)
    float delAng_x = gyro_x * dt;
    float delAng_y = gyro_y * dt;
    float delAng_z = gyro_z * dt;
    
    // ========== BƯỚC 3: CONING CORRECTION ==========
    static float delAng_prev_x = 0.0f;
    static float delAng_prev_y = 0.0f;
    static float delAng_prev_z = 0.0f;
    
    // Cross product: delAng_prev × delAng_current
    float coning_x = 0.5f * (delAng_prev_y * delAng_z - delAng_prev_z * delAng_y);
    float coning_y = 0.5f * (delAng_prev_z * delAng_x - delAng_prev_x * delAng_z);
    float coning_z = 0.5f * (delAng_prev_x * delAng_y - delAng_prev_y * delAng_x);
    
    // Áp dụng coning correction
    delAng_x += coning_x;
    delAng_y += coning_y;
    delAng_z += coning_z;
    
    // Lưu lại cho lần sau
    delAng_prev_x = delAng_x;
    delAng_prev_y = delAng_y;
    delAng_prev_z = delAng_z;
    
    // ========== BƯỚC 4: ROTATION VECTOR METHOD (Quaternion Integration) ==========
    // Lấy quaternion hiện tại
    float q0 = ekf->x.s.q0;
    float q1 = ekf->x.s.q1;
    float q2 = ekf->x.s.q2;
    float q3 = ekf->x.s.q3;
    
    // Tính độ lớn của rotation vector
    float delAng_mag = sqrtf(delAng_x*delAng_x + delAng_y*delAng_y + delAng_z*delAng_z);
    
    // Tạo delta quaternion từ rotation vector
    float dq0, dq1, dq2, dq3;
    
    if (delAng_mag > 1e-8f) {
        // Rotation vector to quaternion: q = [cos(θ/2), sin(θ/2)*axis]
        float half_angle = 0.5f * delAng_mag;
        float sin_half = sinf(half_angle);
        float cos_half = cosf(half_angle);
        
        float axis_scale = sin_half / delAng_mag;
        
        dq0 = cos_half;
        dq1 = delAng_x * axis_scale;
        dq2 = delAng_y * axis_scale;
        dq3 = delAng_z * axis_scale;
    } else {
        // Góc quay rất nhỏ, dùng approximation
        dq0 = 1.0f;
        dq1 = 0.5f * delAng_x;
        dq2 = 0.5f * delAng_y;
        dq3 = 0.5f * delAng_z;
    }
    
    // Quaternion multiplication: q_new = q_old ⊗ dq
    float q0_new = q0*dq0 - q1*dq1 - q2*dq2 - q3*dq3;
    float q1_new = q0*dq1 + q1*dq0 + q2*dq3 - q3*dq2;
    float q2_new = q0*dq2 - q1*dq3 + q2*dq0 + q3*dq1;
    float q3_new = q0*dq3 + q1*dq2 - q2*dq1 + q3*dq0;
    
    // Normalize quaternion
    float q_norm = sqrtf(q0_new*q0_new + q1_new*q1_new + q2_new*q2_new + q3_new*q3_new);
    if (q_norm > 1e-6f) {
        q0_new /= q_norm;
        q1_new /= q_norm;
        q2_new /= q_norm;
        q3_new /= q_norm;
    }
    
    // Lưu lại quaternion đã update
    ekf->x.s.q0 = q0_new;
    ekf->x.s.q1 = q1_new;
    ekf->x.s.q2 = q2_new;
    ekf->x.s.q3 = q3_new;
    
    // ========== BƯỚC 5: SCULLING CORRECTION (cho delta velocity) ==========
    float delVel_x = accel_x * dt;
    float delVel_y = accel_y * dt;
    float delVel_z = accel_z * dt;
    
    // Cross product: delAng × delVel
    float sculling_x = 0.3f * (delAng_y * delVel_z - delAng_z * delVel_y);
    float sculling_y = 0.3f * (delAng_z * delVel_x - delAng_x * delVel_z);
    float sculling_z = 0.3f * (delAng_x * delVel_y - delAng_y * delVel_x);
    
    delVel_x += sculling_x;
    delVel_y += sculling_y;
    delVel_z += sculling_z;
    
    // ========== BƯỚC 6: MA TRẬN QUAY R(q): Body -> NED ==========
    // Dùng quaternion MỚI để rotate
    float r11 = 1.0f - 2.0f*(q2_new*q2_new + q3_new*q3_new);
    float r12 = 2.0f*(q1_new*q2_new - q0_new*q3_new);
    float r13 = 2.0f*(q1_new*q3_new + q0_new*q2_new);
    
    float r21 = 2.0f*(q1_new*q2_new + q0_new*q3_new);
    float r22 = 1.0f - 2.0f*(q1_new*q1_new + q3_new*q3_new);
    float r23 = 2.0f*(q2_new*q3_new - q0_new*q1_new);
    
    float r31 = 2.0f*(q1_new*q3_new - q0_new*q2_new);
    float r32 = 2.0f*(q2_new*q3_new + q0_new*q1_new);
    float r33 = 1.0f - 2.0f*(q1_new*q1_new + q2_new*q2_new);
    
    // ========== BƯỚC 7: XOAY delVel SANG NED ==========
    float delVel_n = r11*delVel_x + r12*delVel_y + r13*delVel_z;
    float delVel_e = r21*delVel_x + r22*delVel_y + r23*delVel_z;
    float delVel_d = r31*delVel_x + r32*delVel_y + r33*delVel_z;
		
		// ========== FIX: ĐỔI DẤU PN ==========
		delVel_n = -delVel_n;
    
    // ========== BƯỚC 8: GRAVITY CANCELLATION ==========
    delVel_d -= GRAVITY_MSS * dt;
    
    // ========== BƯỚC 9: LƯU VẬN TỐC CŨ ==========
    float vn_old = ekf->vn;
    float ve_old = ekf->ve;
    float vd_old = ekf->vd;
    
    // ========== BƯỚC 10: TÍCH PHÂN VELOCITY ==========
		if (!ekf->is_stationary) {
        ekf->vn += delVel_n;
        ekf->ve += delVel_e;
        ekf->vd += delVel_d;
        
				// ========== BƯỚC 11: TÍCH PHÂN POSITION (Trapezoidal) ==========
				ekf->pn += (vn_old + ekf->vn) * 0.5f * dt;
				ekf->pe += (ve_old + ekf->ve) * 0.5f * dt;
				ekf->pd += (vd_old + ekf->vd) * 0.5f * dt;
    }
    
    // ========== BƯỚC 12: GIỚI HẠN VELOCITY ==========
    float max_horiz_vel = 30.0f;
    float max_vert_vel  = 15.0f;
    if (fabsf(ekf->vn) > max_horiz_vel) ekf->vn = max_horiz_vel * (ekf->vn > 0 ? 1.0f : -1.0f);
    if (fabsf(ekf->ve) > max_horiz_vel) ekf->ve = max_horiz_vel * (ekf->ve > 0 ? 1.0f : -1.0f);
    if (fabsf(ekf->vd) > max_vert_vel)  ekf->vd = max_vert_vel  * (ekf->vd  > 0 ? 1.0f : -1.0f);
    
    // ========== BƯỚC 13: ZUPT & DECAY ==========
    float acc_mag = sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z);
    float gyro_mag = sqrtf(gyro_x*gyro_x + gyro_y*gyro_y + gyro_z*gyro_z);
    
		EKF_ApplyZUPT(ekf, acc_mag, gyro_mag);
}


void EKF_Predict(EKF_Handle_t *ekf, Vector3f gyro_rad, Vector3f accel_mss)
{
	if (!ekf->initialized) return;
    float dt = ekf->dt;
		float gx_in = gyro_rad.x;
    float gy_in = gyro_rad.y;
    float gz_in = gyro_rad.z;
		if (ekf->is_stationary) {
       
        gx_in = ekf->x.s.bx;
        gy_in = ekf->x.s.by;
        gz_in = ekf->x.s.bz;
    } 
    else {
        // Simple Deadband (tùy chọn)
        if (fabsf(gx_in) < 0.002f) gx_in = 0;
        if (fabsf(gy_in) < 0.002f) gy_in = 0;
        if (fabsf(gz_in) < 0.002f) gz_in = 0;
    }
    // 1. Correct Gyro with Bias
    float wx = gyro_rad.x - ekf->x.s.bx;
    float wy = gyro_rad.y - ekf->x.s.by;
    float wz = gyro_rad.z - ekf->x.s.bz;

    // --- RK4 INTEGRATION ---
    float q[4] = {ekf->x.s.q0, ekf->x.s.q1, ekf->x.s.q2, ekf->x.s.q3};
    float k1[4], k2[4], k3[4], k4[4];
    float q_tmp[4];

    // k1 = f(q, w)
    get_q_dot(k1, q, wx, wy, wz);

    // k2 = f(q + 0.5*dt*k1, w)
    for(int i=0; i<4; i++) q_tmp[i] = q[i] + 0.5f * dt * k1[i];
    get_q_dot(k2, q_tmp, wx, wy, wz);

    // k3 = f(q + 0.5*dt*k2, w)
    for(int i=0; i<4; i++) q_tmp[i] = q[i] + 0.5f * dt * k2[i];
    get_q_dot(k3, q_tmp, wx, wy, wz);

    // k4 = f(q + dt*k3, w)
    for(int i=0; i<4; i++) q_tmp[i] = q[i] + dt * k3[i];
    get_q_dot(k4, q_tmp, wx, wy, wz);

    // Update Quaternion
    for(int i=0; i<4; i++) {
        ekf->x.vec[i] += (dt / 6.0f) * (k1[i] + 2.0f*k2[i] + 2.0f*k3[i] + k4[i]);
    }
    NormalizeQuaternion(ekf->x.vec);

    // --- DYNAMIC PROCESS NOISE (Q) ---
    // Nếu quay quá nhanh, ta tin vào tích phân quaternion ít hơn (tăng Q)
    float gyro_norm = sqrtF(wx*wx + wy*wy + wz*wz);
    float q_gain = 1.0f + gyro_norm * ekf->cfg.q_scale_dynamic; 

    // --- JACOBIAN F & Q MATRIX ---
    // Jacobian dùng mô hình tuyến tính (đủ tốt cho covariance)
    float dt2 = 0.5f * dt;
    memset(ekf->F, 0, sizeof(ekf->F));
    for(int i=0; i<7; i++) ekf->F[i*7 + i] = 1.0f;

    // F block (q vs q)
    ekf->F[0*7 + 1] = -wx * dt2; ekf->F[0*7 + 2] = -wy * dt2; ekf->F[0*7 + 3] = -wz * dt2;
    ekf->F[1*7 + 0] =  wx * dt2; ekf->F[1*7 + 2] =  wz * dt2; ekf->F[1*7 + 3] = -wy * dt2;
    ekf->F[2*7 + 0] =  wy * dt2; ekf->F[2*7 + 1] = -wz * dt2; ekf->F[2*7 + 3] =  wx * dt2;
    ekf->F[3*7 + 0] =  wz * dt2; ekf->F[3*7 + 1] =  wy * dt2; ekf->F[3*7 + 2] = -wx * dt2;

    // F block (q vs bias)
    float q0=ekf->x.s.q0, q1=ekf->x.s.q1, q2=ekf->x.s.q2, q3=ekf->x.s.q3;
    ekf->F[0*7 + 4] =  q1 * dt2; ekf->F[0*7 + 5] =  q2 * dt2; ekf->F[0*7 + 6] =  q3 * dt2;
    ekf->F[1*7 + 4] = -q0 * dt2; ekf->F[1*7 + 5] =  q3 * dt2; ekf->F[1*7 + 6] = -q2 * dt2;
    ekf->F[2*7 + 4] = -q3 * dt2; ekf->F[2*7 + 5] = -q0 * dt2; ekf->F[2*7 + 6] =  q1 * dt2;
    ekf->F[3*7 + 4] =  q2 * dt2; ekf->F[3*7 + 5] = -q1 * dt2; ekf->F[3*7 + 6] = -q0 * dt2;

    // Update Q with scaling
    memset(ekf->Q, 0, sizeof(ekf->Q));
    float q_gyro = ekf->cfg.sigma_gyro * ekf->cfg.sigma_gyro * dt * q_gain;
    float q_bias = ekf->cfg.sigma_bias * ekf->cfg.sigma_bias * dt;

    ekf->Q[0*7+0] = ekf->Q[1*7+1] = ekf->Q[2*7+2] = ekf->Q[3*7+3] = q_gyro;
    ekf->Q[4*7+4] = ekf->Q[5*7+5] = ekf->Q[6*7+6] = q_bias;

    // P = F * P * F' + Q
    MatMul(ekf->P_temp, ekf->F, ekf->P, 7, 7, 7);
    MatMulTransB(ekf->P, ekf->P_temp, ekf->F, 7, 7, 7); // Reuse P as dest
    MatAdd(ekf->P, ekf->P, ekf->Q, EKF_P_SIZE);
		EKF_PredictVelocity(ekf, accel_mss, gyro_rad);
		//EKF_PredictVelocityBodyFrame(ekf, accel_mss, gyro_rad);
}

// Helper: Innovation Check
// Trả về true nếu phép đo TỐT (trong ngưỡng)
static bool InnovationCheck(float *innov, float *S_inv, int dim, float gate_threshold) {
    float mahalanobis_sq = 0.0f;
    // Tính v^T * S^-1 * v
    // S_inv là dim x dim, innov là dim x 1
    for (int i=0; i<dim; i++) {
        float tmp = 0.0f;
        for (int j=0; j<dim; j++) {
            tmp += innov[j] * S_inv[j*dim + i]; // S_inv đối xứng
        }
        mahalanobis_sq += tmp * innov[i];
    }
    
    // Kiểm tra NaN hoặc âm (do lỗi số học)
    if (isnan(mahalanobis_sq) || mahalanobis_sq < 0) return false;

    return (mahalanobis_sq < (gate_threshold * gate_threshold));
}

void EKF_FuseAccel(EKF_Handle_t *ekf, Vector3f accel_mss) {
    float acc_norm = sqrtF(accel_mss.x*accel_mss.x + accel_mss.y*accel_mss.y + accel_mss.z*accel_mss.z);
    if (acc_norm < 0.1f) return;

    // --- DYNAMIC MEASUREMENT NOISE (R) ---
    // Nếu Accel khác 1g (9.81), tức là có ngoại lực hoặc rung, tăng R lên.
    float acc_err = fabsF(acc_norm - GRAVITY_MSS);
    float r_scale = 1.0f + (acc_err * acc_err) * ekf->cfg.r_acc_scale;
    
    // Normalize measurement
    float ax = accel_mss.x / acc_norm;
    float ay = accel_mss.y / acc_norm;
    float az = accel_mss.z / acc_norm;

    // Predicted Gravity in Body Frame: R(q)^T * [0,0,1]
    float q0=ekf->x.s.q0, q1=ekf->x.s.q1, q2=ekf->x.s.q2, q3=ekf->x.s.q3;
    float vx = 2.0f * (q1*q3 - q0*q2);
    float vy = 2.0f * (q0*q1 + q2*q3);
    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float innov[3] = {ax - vx, ay - vy, az - vz};

    // Jacobian H (3x7)
    memset(ekf->H, 0, sizeof(ekf->H));
    ekf->H[0*7+0] = -2*q2; ekf->H[0*7+1] =  2*q3; ekf->H[0*7+2] = -2*q0; ekf->H[0*7+3] =  2*q1;
    ekf->H[1*7+0] =  2*q1; ekf->H[1*7+1] =  2*q0; ekf->H[1*7+2] =  2*q3; ekf->H[1*7+3] =  2*q2;
    ekf->H[2*7+0] =  2*q0; ekf->H[2*7+1] = -2*q1; ekf->H[2*7+2] = -2*q2; ekf->H[2*7+3] =  2*q3;

    // S = HPH' + R
    MatMulTransB(ekf->Ht, ekf->P, ekf->H, 7, 7, 3); // Ht = P*H'
    MatMul(ekf->S, ekf->H, ekf->Ht, 3, 7, 3);       // S = H*(P*H')

    // Add R (Dynamic)
    float r_val = ekf->cfg.sigma_accel * ekf->cfg.sigma_accel * r_scale;
    ekf->S[0] += r_val; ekf->S[4] += r_val; ekf->S[8] += r_val;

    // Invert S
    Matrix3f matS = Matrix3f_create_from_elements(
        ekf->S[0], ekf->S[1], ekf->S[2],
        ekf->S[3], ekf->S[4], ekf->S[5],
        ekf->S[6], ekf->S[7], ekf->S[8]);
    Matrix3f matS_inv;
    if (!Matrix3f_inverse(&matS, &matS_inv)) return;
    
    // Copy S_inv back to array for Gating Check
    float S_inv_arr[9] = {matS_inv.a.x, matS_inv.a.y, matS_inv.a.z,
                          matS_inv.b.x, matS_inv.b.y, matS_inv.b.z,
                          matS_inv.c.x, matS_inv.c.y, matS_inv.c.z};

    // --- INNOVATION GATING ---
    if (!InnovationCheck(innov, S_inv_arr, 3, ekf->cfg.gate_accel)) {
        ekf->accel_reject_count++;
        return; // Bỏ qua phép đo này
    }

    // K = (P*H') * S^-1
    MatMul(ekf->K, ekf->Ht, S_inv_arr, 7, 3, 3);

    // Update State
    float dx[7];
    MatMul(dx, ekf->K, innov, 7, 3, 1);
    MatAdd(ekf->x.vec, ekf->x.vec, dx, 7);
    NormalizeQuaternion(ekf->x.vec);

    // Update P
    MatMul(ekf->Ht, ekf->H, ekf->P, 3, 7, 7);       // H*P
    MatMul(ekf->P_temp, ekf->K, ekf->Ht, 7, 3, 7);  // K*H*P
    MatSub(ekf->P, ekf->P, ekf->P_temp, EKF_P_SIZE);
}

void EKF_FuseMag(EKF_Handle_t *ekf, Vector3f mag_norm) {
    float q0=ekf->x.s.q0, q1=ekf->x.s.q1, q2=ekf->x.s.q2, q3=ekf->x.s.q3;
    float mx = ekf->mag_ref.x;
    float my = ekf->mag_ref.y;
    float mz = ekf->mag_ref.z;

    // Predicted mag in body frame: R(q)^T * mag_ref
    // R^T is the rotation from NED to Body
    
    // DCM elements (Body <- NED)
    float r11 = 1 - 2*(q2*q2 + q3*q3);
    float r12 = 2*(q1*q2 + q0*q3);
    float r13 = 2*(q1*q3 - q0*q2);
    
    float r21 = 2*(q1*q2 - q0*q3);
    float r22 = 1 - 2*(q1*q1 + q3*q3);
    float r23 = 2*(q2*q3 + q0*q1);
    
    float r31 = 2*(q1*q3 + q0*q2);
    float r32 = 2*(q2*q3 - q0*q1);
    float r33 = 1 - 2*(q1*q1 + q2*q2);

    float pred_x = r11*mx + r12*my + r13*mz;
    float pred_y = r21*mx + r22*my + r23*mz;
    float pred_z = r31*mx + r32*my + r33*mz;

    float innov[3] = {mag_norm.x - pred_x, mag_norm.y - pred_y, mag_norm.z - pred_z};

    // ========================================================================
    // JACOBIAN H (3x7): d(R^T * mag_ref) / dq
    // ========================================================================
    // Formula: d(R(q)^T * v) / dq_i
    // This is standard quaternion rotation derivative
    
    memset(ekf->H, 0, sizeof(ekf->H));
    
    // Row 0: d(pred_x)/dq = d(r11*mx + r12*my + r13*mz)/dq
    ekf->H[0*7+0] =  2*(q0*mx + q3*my - q2*mz);
    ekf->H[0*7+1] =  2*(q1*mx + q2*my + q3*mz);
    ekf->H[0*7+2] =  2*(-q2*mx + q1*my - q0*mz);
    ekf->H[0*7+3] =  2*(-q3*mx + q0*my + q1*mz);
    
    // Row 1: d(pred_y)/dq
    ekf->H[1*7+0] =  2*(-q3*mx + q0*my + q1*mz);
    ekf->H[1*7+1] =  2*(q2*mx - q1*my - q0*mz);
    ekf->H[1*7+2] =  2*(q1*mx + q2*my + q3*mz);
    ekf->H[1*7+3] =  2*(-q0*mx - q3*my + q2*mz);
    
    // Row 2: d(pred_z)/dq
    ekf->H[2*7+0] =  2*(q2*mx - q1*my + q0*mz);
    ekf->H[2*7+1] =  2*(q3*mx - q0*my - q1*mz);
    ekf->H[2*7+2] =  2*(q0*mx + q3*my - q2*mz);
    ekf->H[2*7+3] =  2*(q1*mx + q2*my + q3*mz);

    // Rest of fusion code (S, K, update) stays the same...
    MatMulTransB(ekf->Ht, ekf->P, ekf->H, 7, 7, 3);
    MatMul(ekf->S, ekf->H, ekf->Ht, 3, 7, 3);

    float r_val = ekf->cfg.sigma_mag * ekf->cfg.sigma_mag;
    ekf->S[0] += r_val; ekf->S[4] += r_val; ekf->S[8] += r_val;

    Matrix3f matS = Matrix3f_create_from_elements(
        ekf->S[0], ekf->S[1], ekf->S[2],
        ekf->S[3], ekf->S[4], ekf->S[5],
        ekf->S[6], ekf->S[7], ekf->S[8]);
    Matrix3f matS_inv;
    if (!Matrix3f_inverse(&matS, &matS_inv)) return;
    
    float S_inv_arr[9] = {
        matS_inv.a.x, matS_inv.a.y, matS_inv.a.z,
        matS_inv.b.x, matS_inv.b.y, matS_inv.b.z,
        matS_inv.c.x, matS_inv.c.y, matS_inv.c.z
    };

    if (!InnovationCheck(innov, S_inv_arr, 3, ekf->cfg.gate_mag)) {
        ekf->mag_reject_count++;
        return; 
    }

    MatMul(ekf->K, ekf->Ht, S_inv_arr, 7, 3, 3);
    
    float dx[7];
    MatMul(dx, ekf->K, innov, 7, 3, 1);
    MatAdd(ekf->x.vec, ekf->x.vec, dx, 7);
    NormalizeQuaternion(ekf->x.vec);

    MatMul(ekf->Ht, ekf->H, ekf->P, 3, 7, 7);
    MatMul(ekf->P_temp, ekf->K, ekf->Ht, 7, 3, 7);
    MatSub(ekf->P, ekf->P, ekf->P_temp, EKF_P_SIZE);
}
void EKF_GetPosition(EKF_Handle_t *ekf, float *pn, float *pe, float *pd) {
    *pn = ekf->pn;
    *pe = ekf->pe;
    *pd = ekf->pd;
}
void EKF_GetEuler(EKF_Handle_t *ekf, float *roll, float *pitch, float *yaw) {
    float q0 = ekf->x.s.q0;
    float q1 = ekf->x.s.q1;
    float q2 = ekf->x.s.q2;
    float q3 = ekf->x.s.q3;

    *roll  = atan2F(2.0f * (q0*q1 + q2*q3), 1.0f - 2.0f * (q1*q1 + q2*q2));
    *pitch = asinF(2.0f * (q0*q2 - q3*q1));
    *yaw   = atan2F(2.0f * (q0*q3 + q1*q2), 1.0f - 2.0f * (q2*q2 + q3*q3));
    
    *roll  *= RAD_TO_DEG;
    *pitch *= RAD_TO_DEG;
    *yaw   *= RAD_TO_DEG;
}
void EKF_GetQuaternion(EKF_Handle_t *ekf, float *q0, float *q1, float *q2, float *q3) {
    *q0 = ekf->x.s.q0;
    *q1 = ekf->x.s.q1;
    *q2 = ekf->x.s.q2;
    *q3 = ekf->x.s.q3;
}

bool EKF_IsHealthy(EKF_Handle_t *ekf) {
    if (!ekf->initialized) return false;
    
    // Kiểm tra quaternion hợp lệ
    float q_norm = sqrtF(ekf->x.s.q0*ekf->x.s.q0 + ekf->x.s.q1*ekf->x.s.q1 + 
                         ekf->x.s.q2*ekf->x.s.q2 + ekf->x.s.q3*ekf->x.s.q3);
    if (fabsF(q_norm - 1.0f) > 0.1f) return false;
    
    // Kiểm tra covariance không phát nổ
    for (int i = 0; i < EKF_STATE_DIM; i++) {
        if (ekf->P[i*EKF_STATE_DIM + i] > 100.0f || isnan(ekf->P[i*EKF_STATE_DIM + i])) 
            return false;
    }
    
    return true;
}