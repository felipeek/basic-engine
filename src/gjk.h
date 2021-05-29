#ifndef BASIC_ENGINE_GJK_H
#define BASIC_ENGINE_GJK_H

#include "graphics.h"

typedef struct {
	vec2 p[3];
	u32 num;
} Simplex;

boolean gjk(Mesh m1, Mesh m2, Simplex* _simplex);
vec2 epa(Simplex simplex, Mesh m1, Mesh m2);

#endif