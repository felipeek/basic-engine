#ifndef BASIC_ENGINE_PHYSICS_H
#define BASIC_ENGINE_PHYSICS_H
#include "graphics.h"

typedef struct {
	vec3 position;
	vec3 force;
} Physics_Force;

void physics_simulate(Entity* entities, r32 dt, Physics_Force* forces);
#endif