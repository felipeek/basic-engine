#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_wc;		// world coordinates
	vec3 r2_wc;		// world coordinates
	vec3 r1_lc;		// local coordinates
	vec3 r2_lc;		// local coordinates
	vec3 normal;
	r32 lambda_n;
	r32 lambda_t;
} Collision_Info;

Collision_Info* collision_get_plane_cube_points(Entity* cube, Entity* plane);
Collision_Info* collision_get_plane_sphere_points(Entity* sphere, Entity* plane);
Collision_Info* collision_get_sphere_sphere_points(Entity* sphere1, Entity* sphere2);
Collision_Info* collision_get_convex_convex_points(Entity* e1, Entity* e2);

#endif