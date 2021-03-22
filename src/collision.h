#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H

#include "common.h"
#include "graphics.h"

typedef struct {
	
} Collision_Point;

Collision_Point* collision_get(Entity* se, Entity* me);

#endif