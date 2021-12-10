#ifndef BASIC_ENGINE_JARVIS_MARCH_H
#define BASIC_ENGINE_JARVIS_MARCH_H

#include "common.h"
#include "gm.h"
#include "collision.h"

typedef struct {
	vec3 normal;
	vec3 point;
} Plane;

Projected_Support_Point* jarvis_march(Projected_Support_Point* points);
vec2* sutherland_hodgman(vec2* subject, vec2* clip);
void SutherlandHodgmanClipping(
	vec3* input_polygon,
	int num_clip_planes,
	const Plane* clip_planes,
	vec3** out_polygon,
	boolean removeNotClipToPlane);

#endif