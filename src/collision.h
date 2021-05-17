#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1_lc;
	vec3 r2_lc;
	vec3 r1_wc;
	vec3 r2_wc;
	vec3 normal;
	r32 lambda_n;
	r32 lambda_t;
} Collision_Info;

typedef struct {
	vec3 collision_point;
	vec3 r_lc;
	vec3 r_wc;
	vec3 normal;
    r32 penetration;
	r32 lambda_n;
	r32 lambda_t;
} Collision_Point;

Collision_Info* collision_collect_collisions(Entity* cube, Entity* plane);
Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y);
boolean collision_is_point_inside_with_mesh(vec3 point, Mesh m);
xvec3 collision_project_point_onto_plane(xvec3 p, xvec3 plane_normal, xvec3 plane_point);

#endif