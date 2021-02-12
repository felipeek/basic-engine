#include "animation.h"
#include <dynamic_array.h>
#include <assert.h>
#include <float.h>

#define MAX_NUMBER_OF_POINTS 64

// Calculate the arc lengths of several fragments of the curve
static r32* calculate_arc_lengths(Bezier_Curve* bezier_curve, boolean manually_define_adaptive_subdivision_iterations,
	s32 adaptive_subdivision_iterations)
{
	const r32 ERROR_THRESHOLD = 0.0001f;
	r32* arc_lengths = array_create(r32, bezier_curve->number_of_points);
	r32* arc_lengths_buffer = array_create(r32, bezier_curve->number_of_points);

	// Starts with 2 samples
	u32 num_samples = 2;

	// Calculate the first set of samples
	for (u32 i = 0; i < num_samples; ++i)
	{
		r32 t1 = (r32)i / num_samples;
		r32 t2 = (r32)(i + 1) / num_samples;
		vec3 p1 = animation_get_point_in_bezier_curve(bezier_curve, t1);
		vec3 p2 = animation_get_point_in_bezier_curve(bezier_curve, t2);
		r32 arc_len = gm_vec3_length(gm_vec3_subtract(p2, p1));
		array_push(arc_lengths, &arc_len);
	}

	// Adaptive subdivision
	for (;;)
	{
		boolean need_to_subdivide_more = false;

		for (u32 i = 0; i < num_samples; ++i)
		{
			r32 current_arc_length_approximation = arc_lengths[i];

			// Generate a sampling point between t1 and t2
			r32 t1 = (r32)i / num_samples;
			r32 t2 = (r32)(i + 1) / num_samples;
			r32 t3 = (t2 - t1) / 2.0f + t1;
			vec3 p1 = animation_get_point_in_bezier_curve(bezier_curve, t1);
			vec3 p2 = animation_get_point_in_bezier_curve(bezier_curve, t2);
			vec3 intermediary = animation_get_point_in_bezier_curve(bezier_curve, t3);

			// Calculate the approx arc length of the two generated subdivisions
			r32 first_subdivision_arc_length = gm_vec3_length(gm_vec3_subtract(intermediary, p1));
			r32 second_subdivision_arc_length = gm_vec3_length(gm_vec3_subtract(p2, intermediary));

			array_push(arc_lengths_buffer, &first_subdivision_arc_length);
			array_push(arc_lengths_buffer, &second_subdivision_arc_length);

			r32 more_precise_arc_length = first_subdivision_arc_length + second_subdivision_arc_length;

			// Check whether the error is too big
			if (fabsf(more_precise_arc_length - current_arc_length_approximation) > ERROR_THRESHOLD ||
				fabsf(first_subdivision_arc_length - second_subdivision_arc_length) > ERROR_THRESHOLD)
				need_to_subdivide_more = true;
		}

		if (manually_define_adaptive_subdivision_iterations)
		{
			if (--adaptive_subdivision_iterations == 0)
				break;
		}
		else
		{
			if (!need_to_subdivide_more)
				break;
		}

		// Need to subdivide again
		num_samples *= 2;
		r32* tmp = arc_lengths;
		arc_lengths = arc_lengths_buffer;
		arc_lengths_buffer = tmp;
		array_clear(arc_lengths_buffer);
	}

	printf("%u curve samples were needed.\n", num_samples);

	array_release(arc_lengths_buffer);
	return arc_lengths;
}

// generates a new curve, which is the first order derivate of the received curve
Bezier_Curve animation_derivate_bezier_curve(const Bezier_Curve* bezier_curve)
{
	u32 number_of_points = bezier_curve->number_of_points - 1;
	vec3* points = array_create(vec3, number_of_points);
	for (u32 i = 0; i < number_of_points; ++i) {
		// found reference in https://pages.mtu.edu/~shene/COURSES/cs3621/NOTES/spline/Bezier/bezier-der.html
		vec3 p = gm_vec3_scalar_product(number_of_points,
			gm_vec3_subtract(bezier_curve->points[i + 1], bezier_curve->points[i]));
		array_push(points, &p);
	}

	Bezier_Curve derivated_curve;
	animation_create_bezier_curve(&derivated_curve, points, 1, 1);
	return derivated_curve;
}


void animation_create_bezier_curve(Bezier_Curve* bezier_curve, vec3* points,
	boolean manually_define_adaptive_subdivision_iterations, s32 adaptive_subdivision_iterations)
{
	bezier_curve->number_of_points = array_get_length(points);
	bezier_curve->points = points;
	bezier_curve->arc_lengths = calculate_arc_lengths(bezier_curve,
		manually_define_adaptive_subdivision_iterations, adaptive_subdivision_iterations);

	bezier_curve->total_arc_length = 0.0f;
	for (u32 i = 0; i < array_get_length(bezier_curve->arc_lengths); ++i)
	{
		bezier_curve->total_arc_length += bezier_curve->arc_lengths[i];
	}
}

void animation_destroy_bezier_curve(Bezier_Curve* bezier_curve)
{
	array_release(bezier_curve->arc_lengths);
}

vec3 animation_get_point_in_bezier_curve(const Bezier_Curve* bezier_curve, r32 t)
{
	vec3 generated_points[MAX_NUMBER_OF_POINTS];

	for (u32 i = 0; i < bezier_curve->number_of_points; ++i)
	{
		generated_points[i] = bezier_curve->points[i];
	}

	u32 current_number_of_points = bezier_curve->number_of_points;

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

r32 animation_get_curve_parameter_from_desired_distance(Bezier_Curve* bezier_curve, r32 relative_distance)
{
	if (relative_distance < 0.0f) {
		return 0.0f;
	}

	if (relative_distance > 1.0f) {
		return 1.0f;
	}

	// Calculate the arc length of several fragments of the curve by sampling it enough times
	u32 num_samples = array_get_length(bezier_curve->arc_lengths);

	r32 target_distance = relative_distance * bezier_curve->total_arc_length;
	r32 sum_distance = 0.0f;
	for (u32 i = 0; i < num_samples; ++i)
	{
		sum_distance += bezier_curve->arc_lengths[i];
		if (target_distance <= sum_distance)
		{
			r32 distance_within_fragment = target_distance - (sum_distance - bezier_curve->arc_lengths[i]);
			r32 relative_distance_within_fragment = distance_within_fragment / bezier_curve->arc_lengths[i];

			// Return the 't' value that can be used to roughly move the received distance on the curve
			return ((r32)i / num_samples) + (relative_distance_within_fragment / num_samples);
		}
	}

	assert(0);
}

// Use Frenet Frames to get the orientation to follow the path
Quaternion animation_get_orientation_for_path(const Bezier_Curve* bezier_curve, r32 t, const Bezier_Curve* first_derivate,
	const Bezier_Curve* second_derivate)
{
	vec3 first_derivate_point = animation_get_point_in_bezier_curve(first_derivate, t);
	vec3 second_derivate_point = animation_get_point_in_bezier_curve(second_derivate, t);
	vec3 tangent = gm_vec3_normalize(first_derivate_point);

	// suggested in the class slide, but it doesnt seem to work because it is not orthogonal to the tangent vector
	//vec3 normal = gm_vec3_normalize(second_derivate_point);	

	vec3 normal = gm_vec3_normalize(gm_vec3_cross(first_derivate_point, gm_vec3_cross(second_derivate_point, first_derivate_point)));
	//vec3 normal = gm_vec3_normalize(gm_vec3_cross(second_derivate_point, first_derivate_point));

	vec3 binormal = gm_vec3_normalize(gm_vec3_cross(tangent, normal));

	mat4 rotation_matrix = (mat4) {
		tangent.x, normal.x, binormal.x, 0.0f,
		tangent.y, normal.y, binormal.y, 0.0f,
		tangent.z, normal.z, binormal.z, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	return quaternion_from_matrix(&rotation_matrix);
}