#include "animation.h"
#include <dynamic_array.h>

#define MAX_NUMBER_OF_POINTS 64

vec3 animation_get_point_in_bezier_curve(u32 number_of_points, vec3* points, r32 t)
{
	vec3 generated_points[MAX_NUMBER_OF_POINTS];

	for (u32 i = 0; i < number_of_points; ++i)
	{
		generated_points[i] = points[i];
	}

	u32 current_number_of_points = number_of_points;

	while (current_number_of_points > 1)
	{
		for (s32 i = 0; i < current_number_of_points; ++i) {
			vec3 new_point = gm_vec3_add(gm_vec3_scalar_product(1.0f - t, generated_points[i]), gm_vec3_scalar_product(t, generated_points[i + 1]));
			generated_points[i] = new_point;
		}

		--current_number_of_points;
	}

	return generated_points[0];
}