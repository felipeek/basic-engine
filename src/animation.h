#ifndef BASIC_ENGINE_ANIMATION_H
#define BASIC_ENGINE_ANIMATION_H

#include "common.h"
#include "gm.h"

vec3 animation_get_point_in_bezier_curve(u32 number_of_points, vec3* points, r32 t);

#endif