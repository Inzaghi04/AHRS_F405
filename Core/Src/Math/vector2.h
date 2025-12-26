#ifndef __VECTOR2_H
#define __VECTOR2_H
#include "ftype.h"
typedef struct 
{
    float x;
    float y;
} Vector2f;
//Constructor
Vector2f Vector2f_create(float x, float y);

//Basic vector operations
bool Vector2f_equals(const Vector2f* a, const Vector2f* b);
bool Vector2f_not_equals(const Vector2f* a, const Vector2f* b);

Vector2f Vector2f_neg(const Vector2f* v);
Vector2f Vector2f_add(const Vector2f* a, const Vector2f* b);
Vector2f Vector2f_sub(const Vector2f* a, const Vector2f* b);
Vector2f Vector2f_scale(const Vector2f* v, float s);
Vector2f Vector2f_div(const Vector2f* v, float s);

//implace óperations
void Vector2f_add_inplace(Vector2f* a, const Vector2f* b);
void Vector2f_sub_inplace(Vector2f* a, const Vector2f* b);
void Vector2f_scale_inplace(Vector2f* v, float s);
void Vector2f_div_inplace(Vector2f* v, float s);

/* ===== Dot / Cross ===== */
float Vector2f_dot(const Vector2f *a, const Vector2f *b);
float Vector2f_cross(const Vector2f *a, const Vector2f *b);

/* ===== Length ===== */
float Vector2f_length(const Vector2f *v);
float Vector2f_length_squared(const Vector2f *v);

/* ===== Normalize ===== */
void Vector2f_normalize(Vector2f *v);
Vector2f Vector2f_normalized(const Vector2f *v);

/* ===== Limit length ===== */
bool Vector2f_limit_length(Vector2f *v, float max_length);

/* ===== Angle ===== */
float Vector2f_angle_vec(const Vector2f *v);
float Vector2f_angle_between(const Vector2f *v1, const Vector2f *v2);

/* ===== Rotate (radians) ===== */
Vector2f Vector2f_rotate(const Vector2f *v, float angle_rad);

/* ===== Reflect ===== */
void Vector2f_reflect(Vector2f *v, const Vector2f *n);

/* ===== Projection ===== */
void Vector2f_project(Vector2f *v, const Vector2f *onto);
Vector2f Vector2f_projected(const Vector2f *v, const Vector2f *onto);

/* ===== Offset bearing (degrees) ===== */
void Vector2f_offset_bearing(Vector2f *p, float bearing_deg, float distance);

/* ===== Perpendicular ===== */
Vector2f Vector2f_perpendicular(const Vector2f *pos_delta, const Vector2f *v1);

/* ===== Closest point helpers ===== */
Vector2f Vector2f_closest_point_on_segment(
    const Vector2f *p,
    const Vector2f *v,
    const Vector2f *w
);

Vector2f Vector2f_closest_point_on_radial(
    const Vector2f *p,
    const Vector2f *w
);

/* ===== Distances (point-line, etc.) ===== */
float Vector2f_closest_distance_line_point_squared(
    const Vector2f *w1,
    const Vector2f *w2,
    const Vector2f *p
);

float Vector2f_closest_distance_line_point(
    const Vector2f *w1,
    const Vector2f *w2,
    const Vector2f *p
);

float Vector2f_closest_distance_lines_squared(
    const Vector2f *a1,
    const Vector2f *a2,
    const Vector2f *b1,
    const Vector2f *b2
);

float Vector2f_closest_distance_radial_point_squared(
    const Vector2f *w,
    const Vector2f *p
);

float Vector2f_closest_distance_radial_point(
    const Vector2f *w,
    const Vector2f *p
);

/* ===== Segment Intersection ===== */
bool Vector2f_segment_intersection(
    const Vector2f *s1_start,
    const Vector2f *s1_end,
    const Vector2f *s2_start,
    const Vector2f *s2_end,
    Vector2f *intersection_out
);

/* ===== Circle-segment Intersection ===== */
bool Vector2f_circle_segment_intersection(
    const Vector2f *seg_start,
    const Vector2f *seg_end,
    const Vector2f *circle_center,
    float radius,
    Vector2f *intersection_out
);

/* ===== Point on segment ===== */
bool Vector2f_point_on_segment(
    const Vector2f *pt,
    const Vector2f *seg_start,
    const Vector2f *seg_end
);

/* ===== NAN/INF ===== */
bool Vector2f_is_nan(const Vector2f *v);
bool Vector2f_is_inf(const Vector2f *v);
#endif