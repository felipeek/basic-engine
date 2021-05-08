#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

typedef struct {
	vec3 collision_point;
	vec3 r_lc;
	vec3 r_wc;
	vec3 normal;
    r32 penetration;
	r32 lambda_n;
	r32 lambda_t;
} Collision_Point;

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y);

#endif