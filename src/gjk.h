#ifndef BASIC_ENGINE_GJK_H
#define BASIC_ENGINE_GJK_H

#include "common.h"
#include "gm.h"

typedef struct {
	vec3 a, b, c, d;
	u32 num;
} GJK_Simplex;

typedef struct {
	vec3* polytope;
	dvec3* faces;
	boolean converged;
	vec3 min_normal;
	u32 min_normal_face;
} EPA_Debug;

boolean gjk_collides(vec3* shape1, vec3* shape2, GJK_Simplex* _simplex);
EPA_Debug epa(vec3* shape1, vec3* shape2, GJK_Simplex* simplex, vec3* _normal, r32* _penetration, int num_iterations);

#endif