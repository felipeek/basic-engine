#ifndef BASIC_ENGINE_GJK_H
#define BASIC_ENGINE_GJK_H

#include "graphics.h"

typedef struct {
	vec2 p;
	vec2 o_m1;
	vec2 o_m2;
} Minkowski_Point;

typedef struct {
	Minkowski_Point mp[3];
	u32 num;
} Simplex;

boolean gjk(Mesh m1, Mesh m2, Simplex* _simplex);
vec2 epa(Simplex gjk_simplex, Mesh m1, Mesh m2, vec2* collision_point);

#endif