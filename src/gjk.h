#ifndef BASIC_ENGINE_GJK_H
#define BASIC_ENGINE_GJK_H

#include "common.h"
#include "gm.h"

typedef struct {
	vec3 a, b, c, d;
	u32 num;
} GJK_Simplex;

boolean gjk_collides(vec3* shape1, vec3* shape2, GJK_Simplex* _simplex);
void epa(vec3* shape1, vec3* shape2, GJK_Simplex* simplex, vec3* _normal, r32* _penetration);

#endif