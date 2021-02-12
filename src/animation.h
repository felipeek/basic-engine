#ifndef BASIC_ENGINE_ANIMATION_H
#define BASIC_ENGINE_ANIMATION_H

#include "common.h"
#include "gm.h"
#include "quaternion.h"

typedef struct {
	u32 number_of_points;
	vec3* points;
	r32 total_arc_length;
	r32* arc_lengths;
} Bezier_Curve;

void animation_create_bezier_curve(Bezier_Curve* bezier_curve, vec3* points);
void animation_destroy_bezier_curve(Bezier_Curve* bezier_curve);
vec3 animation_get_point_in_bezier_curve(const Bezier_Curve* bezier_curve, r32 t);
r32 animation_get_curve_parameter_from_desired_distance(Bezier_Curve* bezier_curve, r32 distance);
Quaternion animation_get_orientation_for_path(const Bezier_Curve* bezier_curve, r32 t, const Bezier_Curve* first_derivate,
	const Bezier_Curve* second_derivate);
Bezier_Curve animation_derivate_bezier_curve(const Bezier_Curve* bezier_curve);

#endif