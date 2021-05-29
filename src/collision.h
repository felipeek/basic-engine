#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H
#include "gm.h"
#include "graphics.h"

#define BOUNDING_SHAPE_NAME_MAX_SIZE 64

typedef struct {
	vec3 collision_point;
	vec3 normal;
	r32 penetration;

	vec3 other_collison_point;
} Collision_Point;

typedef struct {
  vec3 v;   // minkowski difference
  vec3 sup; // original bshape vertex
  vec3 sup2; // original bshape vertex
} Support_Point;

typedef struct {
  Support_Point simplex[4];
  int current_index;
} GJK_Support_List;

boolean collision_gjk_collides(GJK_Support_List* sup_list, Bounding_Shape* b1, Bounding_Shape* b2);
Collision_Point collision_epa(Support_Point* simplex, Bounding_Shape* b1, Bounding_Shape* b2);

#endif