#ifndef BASIC_ENGINE_PBD_H
#define BASIC_ENGINE_PBD_H
#include "graphics.h"

typedef enum {
	POSITIONAL_CONSTRAINT
} Constraint_Type;

typedef struct {
	Entity* e1;
	Entity* e2;
	vec3 r1;
	vec3 r2;
	r32 compliance;
	vec3 delta_x;
	
	r32* lambda;
} Positional_Constaint;

typedef struct {
	Constraint_Type type;

	union {
		Positional_Constaint positional_constraint;
	};
} Constraint;

void pbd_simulate(r32 dt, Entity* entities);

#endif