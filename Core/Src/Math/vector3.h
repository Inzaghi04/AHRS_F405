#ifndef __VECTOR3_H
#define __VECTOR3_H
#include "ftype.h"
#include "rotations.h"
#include "vector2.h"

typedef struct
{
    float x, y, z;
} Vector3f;
typedef struct Matrix3f
{
    Vector3f a;
    Vector3f b;
    Vector3f c;
} Matrix3f;
/* ===== Constructors / Init ===== */
Vector3f Vector3f_create(float x, float y, float z);
Vector3f Vector3f_from_vector2(const Vector2f *v2, float z);
void Vector3f_zero(Vector3f *v);

/* ===== Basic Operations (Return new struct) ===== */
bool Vector3f_equals(const Vector3f *v1, const Vector3f *v2);
bool Vector3f_not_equals(const Vector3f *v1, const Vector3f *v2);

Vector3f Vector3f_neg(const Vector3f *v);
Vector3f Vector3f_add(const Vector3f *a, const Vector3f *b);
Vector3f Vector3f_sub(const Vector3f *a, const Vector3f *b);
Vector3f Vector3f_mult_scalar(const Vector3f *v, float s);
Vector3f Vector3f_div_scalar(const Vector3f *v, float s);

/* ===== In-place Operations (Modify input) ===== */
void Vector3f_add_inplace(Vector3f *v, const Vector3f *other);
void Vector3f_sub_inplace(Vector3f *v, const Vector3f *other);
void Vector3f_mult_scalar_inplace(Vector3f *v, float s);
void Vector3f_div_scalar_inplace(Vector3f *v, float s);
void Vector3f_mult_vec_inplace(Vector3f *v, const Vector3f *other);

/* ===== Vector Math ===== */
// Dot product
float Vector3f_dot(const Vector3f *a, const Vector3f *b);

// Cross product
Vector3f Vector3f_cross(const Vector3f *a, const Vector3f *b);

// Length
float Vector3f_length(const Vector3f *v);
float Vector3f_length_squared(const Vector3f *v);

/* ===== Normalize & Limit ===== */
void Vector3f_normalize(Vector3f *v);
Vector3f Vector3f_normalized(const Vector3f *v);

bool Vector3f_limit_length_xy(Vector3f *v, float max_length);

/* ===== Angles & Distances ===== */
float Vector3f_angle(const Vector3f *v1, const Vector3f *v2);
float Vector3f_distance_squared(const Vector3f *v1, const Vector3f *v2);

Vector2f Vector3f_xy(const Vector3f *v);

// Convert RFU (Right-Front-Up) to FRD (Front-Right-Down)
Vector3f Vector3f_rfu_to_frd(const Vector3f *v);

/* ===== Rotations & Projections ===== */
void Vector3f_rotate(Vector3f *v, RotationsEnum  rotation);
void Vector3f_rotate_inverse(Vector3f *v, RotationsEnum  rotation);

void Vector3f_rotate_xy(Vector3f *v, float angle_rad);

void Vector3f_reflect(Vector3f *v, const Vector3f *n);

void Vector3f_project(Vector3f *v, const Vector3f *onto);
Vector3f Vector3f_projected(const Vector3f *v, const Vector3f *onto);

/* ===== Advanced Geometry (Segments, Lines, Planes) ===== */
float Vector3f_distance_to_segment(const Vector3f *v, const Vector3f *seg_start, const Vector3f *seg_end);

void Vector3f_offset_bearing(Vector3f *v, float bearing_deg, float pitch_deg, float distance);


Vector3f Vector3f_perpendicular(const Vector3f *p1, const Vector3f *v1);

/* ===== Static/Helper Geometry Functions ===== */

float Vector3f_closest_distance_between_line_and_point(
    const Vector3f *w1, const Vector3f *w2, const Vector3f *p);


Vector3f Vector3f_point_on_line_closest_to_other_point(
    const Vector3f *w1, const Vector3f *w2, const Vector3f *p);

void Vector3f_segment_to_segment_closest_point(
    const Vector3f *seg1_start, const Vector3f *seg1_end,
    const Vector3f *seg2_start, const Vector3f *seg2_end,
    Vector3f *closest_point_out);

bool Vector3f_segment_plane_intersect(
    const Vector3f *seg_start, const Vector3f *seg_end,
    const Vector3f *plane_normal, const Vector3f *plane_point);

/* ===== Checkers ===== */
bool Vector3f_is_nan(const Vector3f *v);
bool Vector3f_is_inf(const Vector3f *v);
bool Vector3f_is_zero(const Vector3f *v);

Vector3f Vector3f_row_times_mat(const Vector3f *v1, const struct Matrix3f *v2);
#endif