#ifndef BASIC_ENGINE_CONVEX_BARY_COORDS_H
#define BASIC_ENGINE_CONVEX_BARY_COORDS_H

#include "common.h"
#include "gm.h"

r32* convex_bary_coords_get(vec2 point, vec2* hull, vec2* normals);

#endif