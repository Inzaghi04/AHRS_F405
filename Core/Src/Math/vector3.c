#include "vector3.h"
/* ===== Constructors / Init ===== */
Vector3f Vector3f_create(float x, float y, float z)
{
    Vector3f v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
}
Vector3f Vector3f_from_vector2(const Vector2f *v2, float z)
{
    Vector3f v;
    v.x = v2->x;
    v.y = v2->y;
    v.z = z;
    return v;
}
void Vector3f_zero(Vector3f *v)
{
    v->x = 0.0f;
    v->y = 0.0f;
    v->z = 0.0f;
}

/* ===== Basic Operations (Return new struct) ===== */
bool Vector3f_equals(const Vector3f *v1, const Vector3f *v2)
{
    return (is_equal(v1->x, v2->x) && is_equal(v1->y, v2->y) && is_equal(v1->z, v2->z));
}
bool Vector3f_not_equals(const Vector3f *v1, const Vector3f *v2)
{
    return (!is_equal(v1->x, v2->x) || !is_equal(v1->y, v2->y) || !is_equal(v1->z, v2->z));
}

Vector3f Vector3f_neg(const Vector3f *v)
{
    Vector3f result;
    result.x = -v->x;
    result.y = -v->y;
    result.z = -v->z;
    return result;
}
Vector3f Vector3f_add(const Vector3f *a, const Vector3f *b)
{
    Vector3f result;
    result.x = a->x + b->x;
    result.y = a->y + b->y;
    result.z = a->z + b->z;
    return result;
}
Vector3f Vector3f_sub(const Vector3f *a, const Vector3f *b)
{
    Vector3f result;
    result.x = a->x - b->x;
    result.y = a->y - b->y;
    result.z = a->z - b->z;
    return result;
}
Vector3f Vector3f_mult_scalar(const Vector3f *v, float s)
{
    Vector3f result;
    result.x = v->x * s;
    result.y = v->y * s;
    result.z = v->z * s;
    return result;
}
Vector3f Vector3f_div_scalar(const Vector3f *v, float s)
{
    Vector3f result;
    result.x = v->x / s;
    result.y = v->y / s;
    result.z = v->z / s;
    return result;
}
/* Element-wise multiplication
Vector3f Vector3f_mult_vec(const Vector3f *a, const Vector3f *b);

/* ===== In-place Operations (Modify input) ===== */
void Vector3f_add_inplace(Vector3f *v, const Vector3f *other)
{
    v->x += other->x;
    v->y += other->y;
    v->z += other->z;
}
void Vector3f_sub_inplace(Vector3f *v, const Vector3f *other)
{
    v->x -= other->x;
    v->y -= other->y;
    v->z -= other->z;
}
void Vector3f_mult_scalar_inplace(Vector3f *v, float s)
{
    v->x *= s;
    v->y *= s;
    v->z *= s;
}
void Vector3f_div_scalar_inplace(Vector3f *v, float s)
{
    v->x /= s;
    v->y /= s;
    v->z /= s;
}
void Vector3f_mult_vec_inplace(Vector3f *v, const Vector3f *other)
{
    v->x *= other->x;
    v->y *= other->y;
    v->z *= other->z;
}

/* ===== Vector Math ===== */
// Dot product
float Vector3f_dot(const Vector3f *a, const Vector3f *b)
{
    return a->x * b->x + a->y * b->y + a->z * b->z;
}

// Cross product
Vector3f Vector3f_cross(const Vector3f *a, const Vector3f *b)
{
    Vector3f result;
    result.x = a->y * b->z - a->z * b->y;
    result.y = a->z * b->x - a->x * b->z;
    result.z = a->x * b->y - a->y * b->x;
    return result;
}

// Length
float Vector3f_length(const Vector3f *v)
{
    return sqrtF(v->x * v->x + v->y * v->y + v->z * v->z);
}
float Vector3f_length_squared(const Vector3f *v)
{
    return v->x * v->x + v->y * v->y + v->z * v->z;
}

/* ===== Normalize & Limit ===== */
void Vector3f_normalize(Vector3f *v)
{
    float len = Vector3f_length(v);
    if (is_positive(len)) {
        v->x /= len;
        v->y /= len;
        v->z /= len;
    }  
}
Vector3f Vector3f_normalized(const Vector3f *v)
{
    Vector3f result = *v;
    float len = Vector3f_length(v);
    if (is_positive(len)) {
        result.x /= len;
        result.y /= len;
        result.z /= len;
    }
    return result;
}

bool Vector3f_limit_length_xy(Vector3f *v, float max_length)
{
    float len = sqrtF(v->x * v->x + v->y * v->y);
    if ((len > max_length) && is_positive(len)) {
        v->x *= (max_length / len);
        v->y *= (max_length / len);
        return true;
    }
    return false;
}

/* ===== Angles & Distances ===== */
float Vector3f_angle(const Vector3f *v1, const Vector3f *v2)
{
    float len = Vector3f_length(v1) * Vector3f_length(v2);
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = (Vector3f_dot(v1, v2)) / len;
    if (cosv >= 1) {
        return 0.0f;
    }
    if (cosv <= -1) {
        return M_PI;
    }
    return acosF(cosv);
}
float Vector3f_distance_squared(const Vector3f *v1, const Vector3f *v2)
{
    float dx = v1->x - v2->x;
    float dy = v1->y - v2->y;
    float dz = v1->z - v2->z;
    return dx * dx + dy * dy + dz * dz;
}

Vector2f Vector3f_xy(const Vector3f *v)
{
    Vector2f result;
    result.x = v->x;
    result.y = v->y;
    return result;
}

// Convert RFU (Right-Front-Up) to FRD (Front-Right-Down)
Vector3f Vector3f_rfu_to_frd(const Vector3f *v)
{
    Vector3f result;
    result.x = v->y;
    result.y = v->x;
    result.z = -v->z;
    return result;
}

/* ===== Rotations & Projections ===== */
void Vector3f_rotate(Vector3f *v, RotationsEnum  rotation)
{
    float tmp;
    switch (rotation) {
    case ROTATION_NONE:
        return;

    case ROTATION_YAW_45: {
        tmp = HALF_SQRT_2 * (v->x - v->y);
        v->y = HALF_SQRT_2 * (v->x + v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_YAW_90: {
        tmp = v->x; v->x = -v->y; v->y = tmp;
        return;
    }
    case ROTATION_YAW_135: {
        tmp = -HALF_SQRT_2 * (v->x + v->y);
        v->y =  HALF_SQRT_2 * (v->x - v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_YAW_180:
        v->x = -v->x; v->y = -v->y;
        return;

    case ROTATION_YAW_225: {
        tmp = HALF_SQRT_2 * (v->y - v->x);
        v->y = -HALF_SQRT_2 * (v->x + v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_YAW_270: {
        tmp = v->x; v->x = v->y; v->y = -tmp;
        return;
    }
    case ROTATION_YAW_315: {
        tmp = HALF_SQRT_2 * (v->x + v->y);
        v->y = HALF_SQRT_2 * (v->y - v->x);
        v->x = tmp;
        return;
    }
    case ROTATION_ROLL_180: {
        v->y = -v->y; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_45: {
        tmp = HALF_SQRT_2 * (v->x + v->y);
        v->y = HALF_SQRT_2 * (v->x - v->y);
        v->x = tmp; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_90:
    case ROTATION_PITCH_180_YAW_270: {
        tmp = v->x; v->x = v->y; v->y = tmp; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_135: {
        tmp = HALF_SQRT_2 * (v->y - v->x);
        v->y = HALF_SQRT_2 * (v->y + v->x);
        v->x = tmp; v->z = -v->z;
        return;
    }
    case ROTATION_PITCH_180: {
        v->x = -v->x; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_225: {
        tmp = -HALF_SQRT_2 * (v->x + v->y);
        v->y =  HALF_SQRT_2 * (v->y - v->x);
        v->x = tmp; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_270: 
    case ROTATION_PITCH_180_YAW_90: {
        tmp = v->x; v->x = -v->y; v->y = -tmp; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_180_YAW_315: {
        tmp =  HALF_SQRT_2 * (v->x - v->y);
        v->y = -HALF_SQRT_2 * (v->x + v->y);
        v->x = tmp; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_90: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_45: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = HALF_SQRT_2 * (v->x - v->y);
        v->y = HALF_SQRT_2 * (v->x + v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_90: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = v->x; v->x = -v->y; v->y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_135: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = -HALF_SQRT_2 * (v->x + v->y);
        v->y =  HALF_SQRT_2 * (v->x - v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_ROLL_270: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_45: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        tmp = HALF_SQRT_2 * (v->x - v->y);
        v->y = HALF_SQRT_2 * (v->x + v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_90: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        tmp = v->x; v->x = -v->y; v->y = tmp;
        return;
    }
    case ROTATION_ROLL_270_YAW_135: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        tmp = -HALF_SQRT_2 * (v->x + v->y);
        v->y =  HALF_SQRT_2 * (v->x - v->y);
        v->x = tmp;
        return;
    }
    case ROTATION_PITCH_90: {
        tmp = v->z; v->z = -v->x; v->x = tmp;
        return;
    }
    case ROTATION_PITCH_270: {
        tmp = v->z; v->z = v->x; v->x = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_90: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = v->z; v->z = -v->x; v->x = tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_90: {
        v->y = -v->y; v->z = -v->z;
        tmp = v->z; v->z = -v->x; v->x = tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_90: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        tmp = v->z; v->z = -v->x; v->x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        v->x = -v->x; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_270_PITCH_180: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        v->x = -v->x; v->z = -v->z;
        return;
    }
    case ROTATION_ROLL_90_PITCH_270: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = v->z; v->z = v->x; v->x = -tmp;
        return;
    }
    case ROTATION_ROLL_180_PITCH_270: {
        v->y = -v->y; v->z = -v->z;
        tmp = v->z; v->z = v->x; v->x = -tmp;
        return;
    }
    case ROTATION_ROLL_270_PITCH_270: {
        tmp = v->z; v->z = -v->y; v->y = tmp;
        tmp = v->z; v->z = v->x; v->x = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_180_YAW_90: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        v->x = -v->x; v->z = -v->z;
        tmp = v->x; v->x = -v->y; v->y = tmp;
        return;
    }
    case ROTATION_ROLL_90_YAW_270: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = v->x; v->x = v->y; v->y = -tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_68_YAW_293: {
        float tmpx = v->x;
        float tmpy = v->y;
        float tmpz = v->z;
        v->x =  0.14303897231223747232853327204793f * tmpx +  0.36877648650320382639478111741482f * tmpy + -0.91844638134308709265241077446262f * tmpz;
        v->y = -0.33213277779664740485543461545603f * tmpx + -0.85628942146641884303193137384369f * tmpy + -0.39554550256296522325882847326284f * tmpz;
        v->z = -0.93232380121551217122544130688766f * tmpx +  0.36162457008209242248497616856184f * tmpy +  0.00000000000000002214311861220361f * tmpz;
        return;
    }
    case ROTATION_PITCH_315: {
        tmp = HALF_SQRT_2 * (v->x - v->z);
        v->z = HALF_SQRT_2 * (v->x + v->z);
        v->x = tmp;
        return;
    }
    case ROTATION_ROLL_90_PITCH_315: {
        tmp = v->z; v->z = v->y; v->y = -tmp;
        tmp = HALF_SQRT_2 * (v->x - v->z);
        v->z = HALF_SQRT_2 * (v->x + v->z);
        v->x = tmp;
        return;
    }
    case ROTATION_PITCH_7: {
        const float sin_pitch = 0.1218693434051474899781908334262f; // sinF(pitch);
        const float cos_pitch = 0.99254615164132198312785249072476f; // cosF(pitch);
        float tmpx = v->x;
        float tmpz = v->z;
        v->x =  cos_pitch * tmpx + sin_pitch * tmpz;
        v->z = -sin_pitch * tmpx + cos_pitch * tmpz;
        return;
    }
    case ROTATION_ROLL_45: {
        tmp = HALF_SQRT_2 * (v->y - v->z);
        v->z = HALF_SQRT_2 * (v->y + v->z);
        v->y = tmp;
        return;
    }
    case ROTATION_ROLL_315: {
        tmp = HALF_SQRT_2 * (v->y + v->z);
        v->z = HALF_SQRT_2 * (v->z - v->y);
        v->y = tmp;
        return;
    }
    
    // Các case custom hoặc lỗi hệ thống, trong thư viện toán thuần ta bỏ qua hoặc break
    case ROTATION_CUSTOM_1:
    case ROTATION_CUSTOM_2:
    case ROTATION_MAX:
    case ROTATION_CUSTOM_OLD:
    case ROTATION_CUSTOM_END:
    default:
        break;
    }
}
void Vector3f_rotate_inverse(Vector3f *v, RotationsEnum rotation)
{
    Vector3f x_vec = {1.0f, 0.0f, 0.0f};
    Vector3f y_vec = {0.0f, 1.0f, 0.0f};
    Vector3f z_vec = {0.0f, 0.0f, 1.0f};
    Vector3f_rotate(&x_vec, rotation);
    Vector3f_rotate(&y_vec, rotation);
    Vector3f_rotate(&z_vec, rotation);

    Vector3f original = *v;

    v->x = original.x * x_vec.x + original.y * x_vec.y + original.z * x_vec.z;
    v->y = original.x * y_vec.x + original.y * y_vec.y + original.z * y_vec.z;
    v->z = original.x * z_vec.x + original.y * z_vec.y + original.z * z_vec.z;

}

void Vector3f_rotate_xy(Vector3f *v, float angle_rad)
{
    float cos_a = cosF(angle_rad);
    float sin_a = sinF(angle_rad);
    float x_new = cos_a * v->x - sin_a * v->y;
    float y_new = sin_a * v->x + cos_a * v->y;
    v->x = x_new;
    v->y = y_new;
}

void Vector3f_reflect(Vector3f *v, const Vector3f *n)
{
    float dot = Vector3f_dot(v, n);
    v->x = v->x - 2.0f * dot * n->x;
    v->y = v->y - 2.0f * dot * n->y;
    v->z = v->z - 2.0f * dot * n->z;
}

void Vector3f_project(Vector3f *v, const Vector3f *onto)
{
    float onto_len_sq = Vector3f_length_squared(onto);
    if (is_positive(onto_len_sq)) {
        float dot = Vector3f_dot(v, onto);
        float scale = dot / onto_len_sq;
        v->x = onto->x * scale;
        v->y = onto->y * scale;
        v->z = onto->z * scale;
    } else {
        v->x = 0.0f;
        v->y = 0.0f;
        v->z = 0.0f;
    }
}
Vector3f Vector3f_projected(const Vector3f *v, const Vector3f *onto)
{
    Vector3f result = *v;
    float onto_len_sq = Vector3f_length_squared(onto);
    if (is_positive(onto_len_sq)) {
        float dot = Vector3f_dot(v, onto);
        float scale = dot / onto_len_sq;
        result.x = onto->x * scale;
        result.y = onto->y * scale;
        result.z = onto->z * scale;
    } else {
        result.x = 0.0f;
        result.y = 0.0f;
        result.z = 0.0f;
    }
    return result;
}

/* ===== Advanced Geometry (Segments, Lines, Planes) ===== */
float Vector3f_distance_to_segment(const Vector3f *v, const Vector3f *seg_start, const Vector3f *seg_end)
{
    Vector3f seg_vec = Vector3f_sub(seg_end, seg_start);
    Vector3f pt_vec = Vector3f_sub(v, seg_start);
    float seg_len_sq = Vector3f_length_squared(&seg_vec);
    if (!is_positive(seg_len_sq)) {
        return Vector3f_length(&pt_vec);
    }
    float t = Vector3f_dot(&pt_vec, &seg_vec) / seg_len_sq;
    if (t < 0.0f) {
        return Vector3f_length(&pt_vec);
    } else if (t > 1.0f) {
        Vector3f end_to_pt = Vector3f_sub(v, seg_end);
        return Vector3f_length(&end_to_pt);
    } else {
        Vector3f seg_dir = Vector3f_mult_scalar(&seg_vec, t);
        Vector3f projection = Vector3f_add(seg_start, &seg_dir);
        Vector3f proj_to_pt = Vector3f_sub(v, &projection);
        return Vector3f_length(&proj_to_pt);
    }
}

void Vector3f_offset_bearing(Vector3f *v, float bearing_deg, float pitch_deg, float distance)
{
    float bearing_rad = radians(bearing_deg);
    float pitch_rad = radians(pitch_deg);

    float dx = distance * cosF(pitch_rad) * sinF(bearing_rad);
    float dy = distance * cosF(pitch_rad) * cosF(bearing_rad);
    float dz = distance * sinF(pitch_rad);

    v->x += dx;
    v->y += dy;
    v->z += dz;
}


Vector3f Vector3f_perpendicular(const Vector3f *p1, const Vector3f *v1)
{
    Vector3f v1_normalized = Vector3f_normalized(v1);
    Vector3f p1_proj = Vector3f_projected(p1, &v1_normalized);
    Vector3f perp = Vector3f_sub(p1, &p1_proj);
    return perp;
}

/* ===== Static/Helper Geometry Functions ===== */

float Vector3f_closest_distance_between_line_and_point(
    const Vector3f *w1, const Vector3f *w2, const Vector3f *p)
{
    Vector3f closest_point = Vector3f_point_on_line_closest_to_other_point(w1, w2, p);
    Vector3f diff = Vector3f_sub(p, &closest_point);
    return Vector3f_length(&diff);
}


Vector3f Vector3f_point_on_line_closest_to_other_point(
    const Vector3f *w1, const Vector3f *w2, const Vector3f *p)
{
    Vector3f line_vec = Vector3f_sub(w2, w1);
    Vector3f point_vec = Vector3f_sub(p, w1);
    float line_len_sq = Vector3f_length_squared(&line_vec);
    if (!is_positive(line_len_sq)) {
        return *w1;
    }
    float t = Vector3f_dot(&point_vec, &line_vec) / line_len_sq;
    Vector3f line_dir = Vector3f_mult_scalar(&line_vec, t);
    Vector3f closest_point = Vector3f_add(w1, &line_dir);
    return closest_point;
}

void Vector3f_segment_to_segment_closest_point(
    const Vector3f *seg1_start, const Vector3f *seg1_end,
    const Vector3f *seg2_start, const Vector3f *seg2_end,
    Vector3f *closest_point_out)
{
    // direction vectors
    Vector3f line1 = Vector3f_sub(seg1_end, seg1_start);
    Vector3f line2 = Vector3f_sub(seg2_end, seg2_start);
    Vector3f diff  = Vector3f_sub(seg1_start, seg2_start);

    float a = Vector3f_dot(&line1, &line1);
    float b = Vector3f_dot(&line1, &line2);
    float c = Vector3f_dot(&line2, &line2);
    float d = Vector3f_dot(&line1, &diff);
    float e = Vector3f_dot(&line2, &diff);

    float discriminant = (a * c) - (b * b);
    float sN, sD = discriminant;      // default sD = D >= 0
    float tc, tN, tD = discriminant;  // tc = tN / tD, default tD = D >= 0

    if (discriminant < FLT_EPSILON) {
        sN = 0.0f;        // force using point seg1_start on line 1
        sD = 1.0f;        // to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    } else {
        // get the closest points on the infinite lines
        sN = (b * e - c * d);
        tN = (a * e - b * d);
        
        if (sN < 0.0f) {
            // sc < 0 => the s=0 edge is visible
            sN = 0.0f;
            tN = e;
            tD = c;
        } else if (sN > sD) {
            // sc > 1  => the s=1 edge is visible
            sN = sD;
            tN = e + b;
            tD = c;
        }
    }

    if (tN < 0.0f) {
        // tc < 0 => the t=0 edge is visible
        tN = 0.0f;
        // recompute sc for this edge
        if (-d < 0.0f) {
            sN = 0.0f;
        } else if (-d > a) {
            sN = sD;
        } else {
            sN = -d;
            sD = a;
        }
    } else if (tN > tD) {
        // tc > 1  => the t=1 edge is visible
        tN = tD;
        // recompute sc for this edge
        if ((-d + b) < 0.0f) {
            sN = 0.0f;
        } else if ((-d + b) > a) {
            sN = sD;
        } else {
            sN = (-d + b);
            sD = a;
        }
    }

    // finally do the division to get tc
    // Sử dụng hàm is_zero từ ftype.h hoặc kiểm tra trực tiếp
    tc = (is_zero(tN) ? 0.0f : tN / tD);

    // closest point on seg2
    // closest_point = seg2_start + line2*tc;
    Vector3f line2_scaled = Vector3f_mult_scalar(&line2, tc);
    *closest_point_out = Vector3f_add(seg2_start, &line2_scaled);
}
bool Vector3f_segment_plane_intersect(
    const Vector3f *seg_start, 
    const Vector3f *seg_end, 
    const Vector3f *plane_normal, 
    const Vector3f *plane_point)
{
    // Vector3<T> u = seg_end - seg_start;
    Vector3f u = Vector3f_sub(seg_end, seg_start);

    // Vector3<T> w = seg_start - plane_point;
    Vector3f w = Vector3f_sub(seg_start, plane_point);

    // T D = plane_normal * u;
    float D = Vector3f_dot(plane_normal, &u);

    // T N = -(plane_normal * w);
    float N = -Vector3f_dot(plane_normal, &w);

    // Kiểm tra song song
    if (is_zero(D)) {
        if (is_zero(N)) {
            // segment lies in this plane (đoạn thẳng nằm trong mặt phẳng)
            return true;
        } else {
            // does not intersect (song song nhưng không chạm)
            return false;
        }
    }

    const float sI = N / D;

    // Kiểm tra xem điểm giao cắt có nằm trong đoạn thẳng hay không
    if (sI < 0.0f || sI > 1.0f) {
        // does not intersect
        return false;
    }

    // intersects at unique point (giao cắt tại 1 điểm duy nhất)
    return true;
}

/* ===== Checkers ===== */
bool Vector3f_is_nan(const Vector3f *v)
{
    return (isnan(v->x) || isnan(v->y) || isnan(v->z));
}
bool Vector3f_is_inf(const Vector3f *v)
{
    return (isinf(v->x) || isinf(v->y) || isinf(v->z));
}
bool Vector3f_is_zero(const Vector3f *v)
{
    return is_zero(v->x) && is_zero(v->y) && is_zero(v->z);
}

Vector3f Vector3f_row_times_mat(const Vector3f *v, const Matrix3f *m)
{
    Vector3f result;

    /* Phép nhân Vector hàng (1x3) với Ma trận (3x3) -> Vector (1x3)
       v * M = [x, y, z] * [ a.x a.y a.z ]
                           [ b.x b.y b.z ]
                           [ c.x c.y c.z ]
    */

    result.x = v->x * m->a.x + v->y * m->b.x + v->z * m->c.x;
    result.y = v->x * m->a.y + v->y * m->b.y + v->z * m->c.y;
    result.z = v->x * m->a.z + v->y * m->b.z + v->z * m->c.z;

    return result;
}