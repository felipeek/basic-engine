#pragma once
#include "common.hpp"
#include "graphics_math.hpp"

typedef struct
{
    r32 x, y, z, w;
} Quaternion;

Quaternion quaternion_new(Vec3 axis, r32 angle);
Quaternion quaternion_product(const Quaternion* q1, const Quaternion* q2);
Quaternion quaternion_slerp(const Quaternion* q1, const Quaternion* q2, r32 t);
Quaternion quaternion_nlerp(const Quaternion* _q1, const Quaternion* _q2, r32 t);
Quaternion quaternion_inverse(const Quaternion* q);
Mat4       quaternion_get_matrix(const Quaternion* quat);
Vec3       quaternion_get_forward(const Quaternion* quat);
Vec3       quaternion_get_up(const Quaternion* quat);
Vec3       quaternion_get_right(const Quaternion* quat);
Quaternion quaternion_normalize(const Quaternion* q);