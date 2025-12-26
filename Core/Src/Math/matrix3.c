#include "matrix3.h"
// Tạo ma trận từ 3 vector hàng
Matrix3f Matrix3f_create(const Vector3f *a, const Vector3f *b, const Vector3f *c)
{
    Matrix3f m;
    m.a = *a;
    m.b = *b;
    m.c = *c;
    return m;
}
// Tạo ma trận từ 9 phần tử
Matrix3f Matrix3f_create_from_elements(float ax, float ay, float az,
                                       float bx, float by, float bz,
                                       float cx, float cy, float cz)
{
Matrix3f m;
    m.a = Vector3f_create(ax, ay, az);
    m.b = Vector3f_create(bx, by, bz);
    m.c = Vector3f_create(cx, cy, cz);
    return m;
}                                    
// Gán ma trận về 0
void Matrix3f_zero(Matrix3f *m)
{
    m->a = Vector3f_create(0.0f, 0.0f, 0.0f);
    m->b = Vector3f_create(0.0f, 0.0f, 0.0f);
    m->c = Vector3f_create(0.0f, 0.0f, 0.0f);
}
// Gán ma trận đơn vị
void Matrix3f_identity(Matrix3f *m)
{
    Matrix3f_zero(m);
    m->a.x = 1.0f;
    m->b.y = 1.0f;
    m->c.z = 1.0f;
}

/* ===== Basic Operations ===== */
bool Matrix3f_equals(const Matrix3f *m1, const Matrix3f *m2)
{
    return Vector3f_equals(&m1->a, &m2->a) &&
           Vector3f_equals(&m1->b, &m2->b) &&
           Vector3f_equals(&m1->c, &m2->c);
}
Matrix3f Matrix3f_add(const Matrix3f *m1, const Matrix3f *m2)
{
    Matrix3f m;
    m.a = Vector3f_add(&m1->a, &m2->a);
    m.b = Vector3f_add(&m1->b, &m2->b);
    m.c = Vector3f_add(&m1->c, &m2->c);
    return m;
}
Matrix3f Matrix3f_sub(const Matrix3f *m1, const Matrix3f *m2)
{
    Matrix3f m;
    m.a = Vector3f_sub(&m1->a, &m2->a);
    m.b = Vector3f_sub(&m1->b, &m2->b);
    m.c = Vector3f_sub(&m1->c, &m2->c);
    return m;
}
Matrix3f Matrix3f_mult_scalar(const Matrix3f *m, float s)
{
    Matrix3f r;
    r.a = Vector3f_mult_scalar(&m->a, s);
    r.b = Vector3f_mult_scalar(&m->b, s);
    r.c = Vector3f_mult_scalar(&m->c, s);
    return r;
}

/* ===== Vector Multiplications ===== */
// Nhân ma trận với vector cột: Result = M * v
Vector3f Matrix3f_mult_vec(const Matrix3f *m, const Vector3f *v)
{
    Vector3f result;
    result.x = m->a.x * v->x + m->a.y * v->y + m->a.z * v->z;
    result.y = m->b.x * v->x + m->b.y * v->y + m->b.z * v->z;
    result.z = m->c.x * v->x + m->c.y * v->y + m->c.z * v->z;
    return result;
}

// Nhân ma trận chuyển vị với vector: Result = M^T * v
Vector3f Matrix3f_mult_transpose_vec(const Matrix3f *m, const Vector3f *v)
{
    Vector3f result;
    result.x = m->a.x * v->x + m->b.x * v->y + m->c.x * v->z;
    result.y = m->a.y * v->x + m->b.y * v->y + m->c.y * v->z;
    result.z = m->a.z * v->x + m->b.z * v->y + m->c.z * v->z;
    return result;
}

// Nhân ma trận với vector, chỉ lấy phần XY: Result = (M * v).xy
Vector2f Matrix3f_mult_vec_xy(const Matrix3f *m, const Vector3f *v)
{
    Vector2f result;
    result.x = m->a.x * v->x + m->a.y * v->y + m->a.z * v->z;
    result.y = m->b.x * v->x + m->b.y * v->y + m->b.z * v->z;
    return result;
}

/* ===== Matrix Multiplications ===== */
// Nhân 2 ma trận: Result = m1 * m2
Matrix3f Matrix3f_mult_mat(const Matrix3f *m1, const Matrix3f *m2)
{
    Matrix3f r;
    // Row A of m1 dot Cols of m2
    r.a.x = m1->a.x * m2->a.x + m1->a.y * m2->b.x + m1->a.z * m2->c.x;
    r.a.y = m1->a.x * m2->a.y + m1->a.y * m2->b.y + m1->a.z * m2->c.y;
    r.a.z = m1->a.x * m2->a.z + m1->a.y * m2->b.z + m1->a.z * m2->c.z;

    // Row B of m1 dot Cols of m2
    r.b.x = m1->b.x * m2->a.x + m1->b.y * m2->b.x + m1->b.z * m2->c.x;
    r.b.y = m1->b.x * m2->a.y + m1->b.y * m2->b.y + m1->b.z * m2->c.y;
    r.b.z = m1->b.x * m2->a.z + m1->b.y * m2->b.z + m1->b.z * m2->c.z;

    // Row C of m1 dot Cols of m2
    r.c.x = m1->c.x * m2->a.x + m1->c.y * m2->b.x + m1->c.z * m2->c.x;
    r.c.y = m1->c.x * m2->a.y + m1->c.y * m2->b.y + m1->c.z * m2->c.y;
    r.c.z = m1->c.x * m2->a.z + m1->c.y * m2->b.z + m1->c.z * m2->c.z;
    return r;
}

/* ===== Transpose / Determinant / Inverse ===== */
Matrix3f Matrix3f_transposed(const Matrix3f *m)
{
    Matrix3f r;
    r.a = Vector3f_create(m->a.x, m->b.x, m->c.x);
    r.b = Vector3f_create(m->a.y, m->b.y, m->c.y);
    r.c = Vector3f_create(m->a.z, m->b.z, m->c.z);
    return r;
}
void Matrix3f_transpose_inplace(Matrix3f *m)
{
    Matrix3f r = Matrix3f_transposed(m);
    *m = r;
}
float Matrix3f_det(const Matrix3f *m)
{
    return m->a.x * (m->b.y * m->c.z - m->b.z * m->c.y) +
           m->a.y * (m->b.z * m->c.x - m->b.x * m->c.z) +
           m->a.z * (m->b.x * m->c.y - m->b.y * m->c.x);
}
bool Matrix3f_inverse(const Matrix3f *m, Matrix3f *out)
{
    float d = Matrix3f_det(m);
    if (is_zero(d)) {
        return false;
    }
    float inv_d = 1.0f / d;

    out->a.x = (m->b.y * m->c.z - m->c.y * m->b.z) * inv_d;
    out->a.y = (m->a.z * m->c.y - m->a.y * m->c.z) * inv_d;
    out->a.z = (m->a.y * m->b.z - m->a.z * m->b.y) * inv_d;

    out->b.x = (m->b.z * m->c.x - m->b.x * m->c.z) * inv_d;
    out->b.y = (m->a.x * m->c.z - m->a.z * m->c.x) * inv_d;
    out->b.z = (m->b.x * m->a.z - m->a.x * m->b.z) * inv_d;

    out->c.x = (m->b.x * m->c.y - m->c.x * m->b.y) * inv_d;
    out->c.y = (m->c.x * m->a.y - m->a.x * m->c.y) * inv_d;
    out->c.z = (m->a.x * m->b.y - m->b.x * m->a.y) * inv_d;

    return true;
}
bool Matrix3f_invert_inplace(Matrix3f *m)
{
    Matrix3f inv;
    if (Matrix3f_inverse(m, &inv)) {
        *m = inv;
        return true;
    }
    return false;
}

/* ===== Euler / Rotations (Standard 321) ===== */
// Tạo ma trận quay từ Euler angles (Roll -> Pitch -> Yaw)
void Matrix3f_from_euler(Matrix3f *m, float roll, float pitch, float yaw)
{
    float c3 = cosF(pitch);
    float s3 = sinF(pitch);
    float s2 = sinF(roll);
    float c2 = cosF(roll);
    float s1 = sinF(yaw);
    float c1 = cosF(yaw);
    m->a.x = c1 * c3 - s1 * s2 * s3;
    m->b.y = c1 * c2;
    m->c.z = c3 * c2;
    m->a.y = -c2 * s1;
    m->a.z = s3 * c1 + c3 * s2 * s1;
    m->b.x = c3 * s1 + s3 * s2 * c1;
    m->b.z = s1 * s3 - s2 * c1 * c3;
    m->c.x = -s3 * c2;
    m->c.y = s2;
}

// Trích xuất Euler angles từ ma trận
void Matrix3f_to_euler(const Matrix3f *m, float *roll, float *pitch, float *yaw)
{
    *pitch = asinF(m->c.y);
    *roll  = atan2F(-m->c.x, m->c.z);
    *yaw   = atan2F(-m->a.y, m->b.y);
}

// Tạo ma trận từ Enum Rotation (MAVLink standard)
void Matrix3f_from_rotation(Matrix3f *m, RotationsEnum rotation)
{
    Matrix3f_identity(m);
    Vector3f_rotate(&m->a, rotation);
    Vector3f_rotate(&m->b, rotation);
    Vector3f_rotate(&m->c, rotation);
    // ArduPilot Matrix3 implementation rotates rows then transposes logic
    Matrix3f_transpose_inplace(m);
}

// Tạo ma trận quay quanh 1 trục bất kỳ (Axis-Angle)
void Matrix3f_from_axis_angle(Matrix3f *m, const Vector3f *v, float theta)
{
    float C = cosF(theta);
    float S = sinF(theta);
    float t = 1.0f - C;
    Vector3f normv = Vector3f_normalized(v);
    float x = normv.x;
    float y = normv.y;
    float z = normv.z;

    m->a.x = t * x * x + C;
    m->a.y = t * x * y - z * S;
    m->a.z = t * x * z + y * S;
    m->b.x = t * x * y + z * S;
    m->b.y = t * y * y + C;
    m->b.z = t * y * z - x * S;
    m->c.x = t * x * z - y * S;
    m->c.y = t * y * z + x * S;
    m->c.z = t * z * z + C;
}

/* ===== Advanced ===== */
// Normalize matrix (orthogonalize rows) - sửa lỗi trôi (drift)
void Matrix3f_normalize(Matrix3f *m)
{
    // Tính sai số trực giao (dot product của 2 hàng đầu lẽ ra phải bằng 0)
    float error = Vector3f_dot(&m->a, &m->b);
    
    Vector3f t0 = Vector3f_sub(&m->a, &(Vector3f){m->b.x * 0.5f * error, m->b.y * 0.5f * error, m->b.z * 0.5f * error});
    Vector3f t1 = Vector3f_sub(&m->b, &(Vector3f){m->a.x * 0.5f * error, m->a.y * 0.5f * error, m->a.z * 0.5f * error});
    
    // Hàng thứ 3 là cross product của 2 hàng đã chỉnh sửa
    Vector3f t2 = Vector3f_cross(&t0, &t1);

    // Normalize từng hàng
    m->a = Vector3f_mult_scalar(&t0, 1.0f / Vector3f_length(&t0));
    m->b = Vector3f_mult_scalar(&t1, 1.0f / Vector3f_length(&t1));
    m->c = Vector3f_mult_scalar(&t2, 1.0f / Vector3f_length(&t2));
}

void Matrix3f_rotate_by_gyro(Matrix3f *m, const Vector3f *gyro)
{
    // DCM update based on gyro vector (skew-symmetric matrix approximation)
    Matrix3f delta;
    
    // Row A update
    delta.a.x = m->a.y * gyro->z - m->a.z * gyro->y;
    delta.a.y = m->a.z * gyro->x - m->a.x * gyro->z;
    delta.a.z = m->a.x * gyro->y - m->a.y * gyro->x;

    // Row B update
    delta.b.x = m->b.y * gyro->z - m->b.z * gyro->y;
    delta.b.y = m->b.z * gyro->x - m->b.x * gyro->z;
    delta.b.z = m->b.x * gyro->y - m->b.y * gyro->x;

    // Row C update
    delta.c.x = m->c.y * gyro->z - m->c.z * gyro->y;
    delta.c.y = m->c.z * gyro->x - m->c.x * gyro->z;
    delta.c.z = m->c.x * gyro->y - m->c.y * gyro->x;

    // Add delta to current matrix
    m->a.x += delta.a.x; m->a.y += delta.a.y; m->a.z += delta.a.z;
    m->b.x += delta.b.x; m->b.y += delta.b.y; m->b.z += delta.b.z;
    m->c.x += delta.c.x; m->c.y += delta.c.y; m->c.z += delta.c.z;
}

/* ===== Columns Access ===== */
Vector3f Matrix3f_col_x(const Matrix3f *m)
{
    return Vector3f_create(m->a.x, m->b.x, m->c.x);
}

Vector3f Matrix3f_col_y(const Matrix3f *m)
{
    return Vector3f_create(m->a.y, m->b.y, m->c.y);
}

Vector3f Matrix3f_col_z(const Matrix3f *m)
{
    return Vector3f_create(m->a.z, m->b.z, m->c.z);
}