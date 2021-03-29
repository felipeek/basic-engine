#ifndef BASIC_ENGINE_PHYSICS_H
#define BASIC_ENGINE_PHYSICS_H
#include "graphics.h"
#include "matrix.h"

void physics_simulate(Entity* entities, r32 dt);
r32* qp_solve(const Matrix* A, r32* b);
#endif