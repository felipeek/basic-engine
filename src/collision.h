#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

#define BOUNDING_SHAPE_NAME_MAX_SIZE 64

typedef struct {
	vec3 collision_point;
	vec3 normal;
	r32 penetration;
	u32 vertex_index;
} Collision_Point;

typedef struct {
  vec3 v;   // minkowski difference
  vec3 sup; // original bshape vertex
} Support_Point;

typedef struct {
  Support_Point simplex[4];
  int current_index;
} GJK_Support_List;

typedef struct {
  u32 vertex_count;
  vec3* vertices;
} Bounding_Shape;

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y);

boolean collision_gjk_collides(GJK_Support_List* sup_list, Bounding_Shape* b1, Bounding_Shape* b2);
vec3 collision_epa(Support_Point* simplex, Bounding_Shape* b1, Bounding_Shape* b2, vec3* penetration);

#endif