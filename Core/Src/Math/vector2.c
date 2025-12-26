#include "vector2.h"
Vector2f Vector2f_create(float x, float y)
{
    Vector2f v;
    v.x = x;
    v.y = y;
    return v;
}
//Basic vector operations
bool Vector2f_equals(const Vector2f* a, const Vector2f* b)
{
    return (is_equal(a->x, b->x) && is_equal(a->y, b->y));
}
bool Vector2f_not_equals(const Vector2f* a, const Vector2f* b)
{
    return (!is_equal(a->x, b->x) || !is_equal(a->y, b->y));
}

Vector2f Vector2f_neg(const Vector2f* v)
{
    Vector2f result;
    result.x = -v->x;
    result.y = -v->y;
    return result;
}
Vector2f Vector2f_add(const Vector2f* a, const Vector2f* b)
{
    Vector2f result;
    result.x = a->x + b->x;
    result.y = a->y + b->y;
    return result;
}
Vector2f Vector2f_sub(const Vector2f* a, const Vector2f* b)
{
    Vector2f result;
    result.x = a->x - b->x;
    result.y = a->y - b->y;
    return result;
}
Vector2f Vector2f_scale(const Vector2f* v, float s)
{
    Vector2f result;
    result.x = v->x * s;
    result.y = v->y * s;
    return result;
}
Vector2f Vector2f_div(const Vector2f* v, float s)
{
    Vector2f result;
    result.x = v->x / s;
    result.y = v->y / s;
    return result;
}

//implace óperations
void Vector2f_add_inplace(Vector2f* a, const Vector2f* b)
{
    a->x += b->x;
    a->y += b->y;
}
void Vector2f_sub_inplace(Vector2f* a, const Vector2f* b)
{
    a->x -= b->x;
    a->y -= b->y;
}
void Vector2f_scale_inplace(Vector2f* v, float s)
{
    v->x *= s;
    v->y *= s;
}
void Vector2f_div_inplace(Vector2f* v, float s)
{
    v->x /= s;
    v->y /= s;
}

/* ===== Dot / Cross ===== */
float Vector2f_dot(const Vector2f *a, const Vector2f *b)
{
    return a->x * b->x + a->y * b->y;
}
float Vector2f_cross(const Vector2f *a, const Vector2f *b)
{
    return a->x * b->y - a->y * b->x;
}

/* ===== Length ===== */
float Vector2f_length(const Vector2f *v)
{
    return sqrtF(v->x * v->x + v->y * v->y);
}
float Vector2f_length_squared(const Vector2f *v)
{
    return (v->x * v->x + v->y * v->y);
}

/* ===== Normalize ===== */
void Vector2f_normalize(Vector2f *v)
{
    float len = Vector2f_length(v);
    if (is_positive(len)) {
        v->x /= len;
        v->y /= len;
    }  
}
Vector2f Vector2f_normalized(const Vector2f *v)
{
    Vector2f result = *v;
    float len = Vector2f_length(v);
    if (is_positive(len)) {
        result.x /= len;
        result.y /= len;
    }
    return result;
}

/* ===== Limit length ===== */
bool Vector2f_limit_length(Vector2f *v, float max_length)
{
    float len = Vector2f_length(v);
    if ((len > max_length) && is_positive(len)) {
        v->x *= (max_length / len);
        v->y *= (max_length / len);
        return true;
    }
    return false;
}

/* ===== Angle ===== */
float Vector2f_angle_vec(const Vector2f *v)
{
    return atan2F(v->y, v->x);
}
float Vector2f_angle_between(const Vector2f *v1, const Vector2f *v2)
{
    float len = Vector2f_length(v1) * Vector2f_length(v2);
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = (Vector2f_dot(v1, v2)) / len;
    if (cosv >= 1) {
        return 0.0f;
    }
    if (cosv <= -1) {
        return M_PI;
    }
    return acosF(cosv);
}

/* ===== Rotate (radians) ===== */
Vector2f Vector2f_rotate(const Vector2f *v, float angle_rad)
{
    Vector2f result;
    float cs = cosF(angle_rad);
    float sn = sinF(angle_rad);
    result.x = v->x * cs - v->y * sn;
    result.y = v->x * sn + v->y * cs;
    return result;
}

/* ===== Reflect ===== */
void Vector2f_reflect(Vector2f *v, const Vector2f *n)
{
    Vector2f orig = *v;
    Vector2f_project(v, n);
    v->x = v->x * 2.0f - orig.x;
    v->y = v->y * 2.0f - orig.y;
}

/* ===== Projection ===== */
void Vector2f_project(Vector2f *v, const Vector2f *onto)
{
    float dot = Vector2f_dot(v, onto);
    float onto_len_sq = Vector2f_length_squared(onto);
    if (is_positive(onto_len_sq)) {
        float scale = dot / onto_len_sq;
        v->x = onto->x * scale;
        v->y = onto->y * scale;
    } else {
        v->x = 0.0f;
        v->y = 0.0f;
    }
}
Vector2f Vector2f_projected(const Vector2f *v, const Vector2f *onto)
{
    Vector2f result;
    float dot = Vector2f_dot(v, onto);
    float onto_len_sq = Vector2f_length_squared(onto);
    if (is_positive(onto_len_sq)) {
        float scale = dot / onto_len_sq;
        result.x = onto->x * scale;
        result.y = onto->y * scale;
    } else {
        result.x = 0.0f;
        result.y = 0.0f;
    }
    return result;
}

/* ===== Offset bearing (degrees) ===== */
void Vector2f_offset_bearing(Vector2f *p, float bearing_deg, float distance)
{
    p->x += cosF(radians(bearing_deg)) * distance;
    p->y += sinF(radians(bearing_deg)) * distance;
}

/* ===== Perpendicular ===== */
Vector2f Vector2f_perpendicular(const Vector2f *pos_delta, const Vector2f *v1)
{
    Vector2f perpendicular1 = Vector2f_create(-v1->y, v1->x);
    Vector2f perpendicular2 = Vector2f_create(v1->y, -v1->x);
    float d1 = Vector2f_dot(&perpendicular1, pos_delta);
    float d2 = Vector2f_dot(&perpendicular2, pos_delta);
    if (d1 > d2) {
        return perpendicular1;
    }
    return perpendicular2;
}

/* ===== Closest point helpers ===== */
Vector2f Vector2f_closest_point_on_segment(
    const Vector2f *p,
    const Vector2f *v,
    const Vector2f *w
)
{
    Vector2f vw = Vector2f_sub(w, v);
    Vector2f vp = Vector2f_sub(p, v);
    float len_sq = Vector2f_length_squared(&vw);
    if (is_zero(len_sq)) {
        return *v; // v == w case
    }
    float t = Vector2f_dot(&vp, &vw) / len_sq;
    if (t < 0.0f) {
        return *v;
    } else if (t > 1.0f) {
        return *w;
    }
    else 
    {
        Vector2f vw_scaled = Vector2f_scale(&vw, t);
        return Vector2f_add(v, &vw_scaled);
    }
}

Vector2f Vector2f_closest_point_on_radial(
    const Vector2f *p,
    const Vector2f *w
)
{
    
    float len_sq = Vector2f_length_squared(w);
    if (is_zero(len_sq)) {
        return *w; // v == w case
    }
    float t = Vector2f_dot(p, w) / len_sq; 
    if (t < 0.0f) {
        return Vector2f_create(0.0f, 0.0f);
    } else if (t > 1.0f) {
        return *w;
    }
    else 
    {
        Vector2f w_scaled = Vector2f_scale(w, t);
        return w_scaled;
    }
}
/* ===== Distances (point-line, etc.) ===== */
float Vector2f_closest_distance_line_point_squared(
    const Vector2f *w1,
    const Vector2f *w2,
    const Vector2f *p
)
{
    Vector2f w2_w1 = Vector2f_sub(w2, w1);
    Vector2f p_w1 = Vector2f_sub(p, w1);
    // Gọi hàm radial (đã có ở dưới)
    return Vector2f_closest_distance_radial_point_squared(&w2_w1, &p_w1); 
}
float Vector2f_closest_distance_line_point(
    const Vector2f *w1,
    const Vector2f *w2,
    const Vector2f *p
)
{
    return sqrtF(Vector2f_closest_distance_line_point_squared(w1, w2, p));
}

float Vector2f_closest_distance_lines_squared(
    const Vector2f *a1,
    const Vector2f *a2,
    const Vector2f *b1,
    const Vector2f *b2
)
{
    float dist1 = Vector2f_closest_distance_line_point_squared(b1,b2,a1);
    float dist2 = Vector2f_closest_distance_line_point_squared(b1,b2,a2);
    float dist3 = Vector2f_closest_distance_line_point_squared(a1,a2,b1);
    float dist4 = Vector2f_closest_distance_line_point_squared(a1,a2,b2);
    float min1 = MIN(dist1, dist2);
    float min2 = MIN(dist3, dist4);
    return MIN(min1, min2);
}

float Vector2f_closest_distance_radial_point_squared(
    const Vector2f *w,
    const Vector2f *p
)
{
    Vector2f closest = Vector2f_closest_point_on_radial(p, w);
    Vector2f delta = Vector2f_sub(&closest, p);
    return Vector2f_length_squared(&delta);
}

float Vector2f_closest_distance_radial_point(
    const Vector2f *w,
    const Vector2f *p
)
{
    return sqrtF(Vector2f_closest_distance_radial_point_squared(w, p));
}

/* ===== Segment Intersection ===== */
bool Vector2f_segment_intersection(
    const Vector2f *s1_start,
    const Vector2f *s1_end,
    const Vector2f *s2_start,
    const Vector2f *s2_end,
    Vector2f *intersection_out
)
{
    Vector2f r1 = Vector2f_sub(s1_end, s1_start);
    Vector2f r2 = Vector2f_sub(s2_end, s2_start);
    Vector2f ss2_ss1 = Vector2f_sub(s2_start, s1_start);
    float r1xr2 = Vector2f_cross(&r1, &r2);
    float q_pxr = Vector2f_cross(&ss2_ss1, &r1);
    if (is_zero(r1xr2)) {
        // either collinear or parallel and non-intersecting
        return false;
    } else {
        // t = (q - p) * s / (r * s)
        // u = (q - p) * r / (r * s)
        float t = Vector2f_cross(&ss2_ss1, &r2) / r1xr2;
        float u = q_pxr / r1xr2;
        if ((u >= 0) && (u <= 1) && (t >= 0) && (t <= 1)) {
            // lines intersect
            // t can be any non-negative value because (p, p + r) is a ray
            // u must be between 0 and 1 because (q, q + s) is a line segment
            Vector2f r1_scaled = Vector2f_scale(&r1, t);
            *intersection_out = Vector2f_add(s1_start, &r1_scaled);
            return true;
        } else {
            // non-parallel and non-intersecting
            return false;
        }
    }
}
/* ===== Circle-segment Intersection ===== */
bool Vector2f_circle_segment_intersection(
    const Vector2f *seg_start,
    const Vector2f *seg_end,
    const Vector2f *circle_center,
    float radius,
    Vector2f *intersection_out
)
{
    // segment start relative to circle center
    Vector2f seg_start_local = Vector2f_sub(seg_start, circle_center);
    // segment vector
    Vector2f d = Vector2f_sub(seg_end, seg_start);

    float a = Vector2f_length_squared(&d);
    float b = 2.0f * Vector2f_dot(&d, &seg_start_local);
    float c = Vector2f_length_squared(&seg_start_local) - radius*radius;

    // check invalid
    if (is_zero(a) || isnan(a) || isnan(b) || isnan(c)) return false;

    float delta = b*b - 4.0f*a*c;
    if (delta < 0.0f || isnan(delta)) return false;

    float sqrt_delta = sqrtF(delta);
    float t1 = (-b - sqrt_delta) / (2.0f * a);
    float t2 = (-b + sqrt_delta) / (2.0f * a);

    if (t1 >= 0.0f && t1 <= 1.0f) {
        Vector2f delta_scaled = Vector2f_scale(&d, t1);
        *intersection_out = Vector2f_add(seg_start, &delta_scaled);
        return true;
    } else if (t2 >= 0.0f && t2 <= 1.0f) {
        Vector2f delta_scaled = Vector2f_scale(&d, t2);
        *intersection_out = Vector2f_add(seg_start, &delta_scaled);
        return true;
    }

    return false;
}
/* ===== Point on segment ===== */
bool Vector2f_point_on_segment(
    const Vector2f *pt,
    const Vector2f *seg_start,
    const Vector2f *seg_end
)

{
    float expected_run = seg_end->x - seg_start->x;
    float intersection_run = pt->x - seg_start->x;
    // check slopes are identical:
    if (is_zero(expected_run)) {
        if (fabsF(intersection_run) > FLT_EPSILON) {
            return false;
        }
    } else {
        float expected_slope = (seg_end->y - seg_start->y) / expected_run;
        float intersection_slope = (pt->y - seg_start->y) / intersection_run;
        if (fabsF(expected_slope - intersection_slope) > FLT_EPSILON) {
            return false;
        }
    }
    // check for presence in bounding box
    if (seg_start->x < seg_end->x) {
        if (pt->x < seg_start->x || pt->x > seg_end->x) {
            return false;
        }
    } else {
        if (pt->x < seg_end->x || pt->x > seg_start->x) {
            return false;
        }
    }
    if (seg_start->y < seg_end->y) {
        if (pt->y < seg_start->y || pt->y > seg_end->y) {
            return false;
        }
    } else {
        if (pt->y < seg_end->y || pt->y > seg_start->y) {
            return false;
        }
    }
    return true;
}

/* ===== NAN/INF ===== */
bool Vector2f_is_nan(const Vector2f *v)
{
    return isnan(v->x) || isnan(v->y);
}
bool Vector2f_is_inf(const Vector2f *v)
{
    return isinf(v->x) || isinf(v->y);
}