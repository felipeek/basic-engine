#ifndef BASIC_ENGINE_CONVEX_BARY_COORDS_H
#define BASIC_ENGINE_CONVEX_BARY_COORDS_H

#include "common.h"
#include "gm.h"

boolean convex_bary_coords_get(vec2 point, vec2* hull, r32** bary_coords);

#endif