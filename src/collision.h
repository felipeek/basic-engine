#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

typedef struct {
	vec3 collision_point;
	vec3 normal;
	r32 penetration;
} Collision_Point;

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y);

#endif