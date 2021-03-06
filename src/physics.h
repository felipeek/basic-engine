#ifndef BASIC_ENGINE_PHYSICS_H
#define BASIC_ENGINE_PHYSICS_H

#include "common.h"
#include "graphics.h"

typedef struct {
	vec4 position;
	vec4 force;
} Physics_Force;

void physics_update(Entity* e, Physics_Force* forces, r32 dt);

#endif