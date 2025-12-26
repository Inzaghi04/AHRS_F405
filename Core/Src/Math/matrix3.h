#ifndef __MATRIX3_H
#define __MATRIX3_H

#include "ftype.h"
#include "vector3.h"
#include "vector2.h"


/* ===== Constructors / Init ===== */
// Tạo ma trận từ 3 vector hàng
Matrix3f Matrix3f_create(const Vector3f *a, const Vector3f *b, const Vector3f *c);
// Tạo ma trận từ 9 phần tử
Matrix3f Matrix3f_create_from_elements(float ax, float ay, float az,
                                       float bx, float by, float bz,
                                       float cx, float cy, float cz);
// Gán ma trận về 0
void Matrix3f_zero(Matrix3f *m);
// Gán ma trận đơn vị
void Matrix3f_identity(Matrix3f *m);

/* ===== Basic Operations ===== */
bool Matrix3f_equals(const Matrix3f *m1, const Matrix3f *m2);
Matrix3f Matrix3f_add(const Matrix3f *m1, const Matrix3f *m2);
Matrix3f Matrix3f_sub(const Matrix3f *m1, const Matrix3f *m2);
Matrix3f Matrix3f_mult_scalar(const Matrix3f *m, float s);

/* ===== Vector Multiplications ===== */
// Nhân ma trận với vector cột: Result = M * v
Vector3f Matrix3f_mult_vec(const Matrix3f *m, const Vector3f *v);

// Nhân ma trận chuyển vị với vector: Result = M^T * v
Vector3f Matrix3f_mult_transpose_vec(const Matrix3f *m, const Vector3f *v);

// Nhân ma trận với vector, chỉ lấy phần XY: Result = (M * v).xy
Vector2f Matrix3f_mult_vec_xy(const Matrix3f *m, const Vector3f *v);

/* ===== Matrix Multiplications ===== */
// Nhân 2 ma trận: Result = m1 * m2
Matrix3f Matrix3f_mult_mat(const Matrix3f *m1, const Matrix3f *m2);

/* ===== Transpose / Determinant / Inverse ===== */
Matrix3f Matrix3f_transposed(const Matrix3f *m);
void Matrix3f_transpose_inplace(Matrix3f *m);
float Matrix3f_det(const Matrix3f *m);
bool Matrix3f_inverse(const Matrix3f *m, Matrix3f *out);
bool Matrix3f_invert_inplace(Matrix3f *m);

/* ===== Euler / Rotations (Standard 321) ===== */
// Tạo ma trận quay từ Euler angles (Roll -> Pitch -> Yaw)
void Matrix3f_from_euler(Matrix3f *m, float roll, float pitch, float yaw);

// Trích xuất Euler angles từ ma trận
void Matrix3f_to_euler(const Matrix3f *m, float *roll, float *pitch, float *yaw);

// Tạo ma trận từ Enum Rotation (MAVLink standard)
void Matrix3f_from_rotation(Matrix3f *m, RotationsEnum rotation);

// Tạo ma trận quay quanh 1 trục bất kỳ (Axis-Angle)
void Matrix3f_from_axis_angle(Matrix3f *m, const Vector3f *v, float theta);

/* ===== Advanced ===== */
// Normalize matrix (orthogonalize rows) - sửa lỗi trôi (drift)
void Matrix3f_normalize(Matrix3f *m);

// Apply rotation from gyro (integration step)
void Matrix3f_rotate_by_gyro(Matrix3f *m, const Vector3f *gyro);

/* ===== Columns Access ===== */
Vector3f Matrix3f_col_x(const Matrix3f *m);
Vector3f Matrix3f_col_y(const Matrix3f *m);
Vector3f Matrix3f_col_z(const Matrix3f *m);

#endif