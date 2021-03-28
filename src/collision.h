#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H

#include "common.h"
#include "graphics.h"

s32 collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3);
s32 collision_check_ray_intersects_triangle(vec3 p, vec3 v, vec3 t1, vec3 t2, vec3 t3, vec3* intersection);
boolean collision_get_point_closest_intersection_with_mesh(vec3 point, Mesh m, vec3* closest_point, r32* closest_distance);

#endif