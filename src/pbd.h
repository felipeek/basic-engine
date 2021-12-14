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

	// Fields necessary to implement static friction.
	boolean is_static_friction_constraint;
	r32 static_friction_coefficient;
	r32* normal_lambda;		// The 'lambda' calculated by the normal constraint (NOTE: during the solver, this must be calculated first!)
} Positional_Constaint;

typedef struct {
	Constraint_Type type;

	union {
		Positional_Constaint positional_constraint;
	};
} Constraint;

void pbd_simulate(r32 dt, Entity* entities);
void collect_collisions(Entity* entities);

#endif