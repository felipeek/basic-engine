#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H

#include "common.h"
#include "graphics.h"

s32 collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3);
s32 collision_check_edge_collides_triangle(vec3 edge_p1, vec3 edge_p2, vec3 t1, vec3 t2, vec3 t3);

#endif