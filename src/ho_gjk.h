#ifndef BASIC_ENGINE_HO_GJK_H
#define BASIC_ENGINE_HO_GJK_H
#include "gm.h"
#include "graphics.h"

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

boolean ho_gjk_collides(GJK_Support_List* sup_list, vec3* b1, vec3* b2);
vec3 ho_collision_epa(Support_Point* simplex, vec3* b1, vec3* b2);

#endif