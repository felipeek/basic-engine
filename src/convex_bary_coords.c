#include "convex_bary_coords.h"
#include <light_array.h>

// checks whether a point lies on a line (2D)
// l1_x, l1_y -> line's first point
// l2_x, l2_y -> line's second point
// p_x, p_y -> point of interest
// w -> output indicating the weight of the point
// returns 1 if point lies on the line, 0 if not
// https://gist.github.com/felipeek/93c873395506868e50ea6f2323eb3399
static boolean point_lies_on_line_2d(vec2 l1, vec2 l2, vec2 p, r32* w) {
	const r32 EPSILON = 0.000001f;
	boolean vertical_line = (l1.x - l2.x) > -EPSILON && (l1.x - l2.x) < EPSILON; // vertical line
	boolean horizontal_line = (l1.y - l2.y) > -EPSILON && (l1.y - l2.y) < EPSILON; // horizontal line

	if (vertical_line && horizontal_line) {	// degenerate case -> line is composed of a single point
		if (((p.x - l1.x) > -EPSILON && (p.x - l1.x) < EPSILON) &&
			((p.y - l1.y) > -EPSILON && (p.y - l1.y) < EPSILON)) {
			// in this case, 'w' has no real meaning
			*w = 0.5f;
			return true;
		}
	}

	if (vertical_line) { // vertical line
		if ((p.x - l1.x) > -EPSILON && (p.x - l1.x) < EPSILON) { // p also lies on same line?
			r32 alpha = (p.y - l1.y) / (l2.y - l1.y);
			if (alpha < 0.0f || alpha > 1.0f) {
				return false;
			}
			*w = alpha;
			return true;
		}
		
		return false;
	}

	if (horizontal_line) { // horizontal line
		if ((p.y - l1.y) > -EPSILON && (p.y - l1.y) < EPSILON) { // p also lies on same line?
			r32 alpha = (p.x - l1.x) / (l2.x - l1.x);
			if (alpha < 0.0f || alpha > 1.0f) {
				return false;
			}
			*w = alpha;
			return true;
		}
		
		return false;
	}

	// general case
	r32 alpha_x = (p.x - l1.x) / (l2.x - l1.x);
	if (alpha_x < 0.0f || alpha_x > 1.0f) {
		return false;
	}
	r32 alpha_y = (p.y - l1.y) / (l2.y - l1.y);
	if (alpha_y < 0.0f || alpha_y > 1.0f) {
		return false;
	}

	r32 alpha_diff = alpha_x - alpha_y;
	if (alpha_diff > -EPSILON && alpha_diff < EPSILON) {
		// edge found
		*w = alpha_x;
		return true;
	}

	return false;
}

boolean convex_bary_coords_get(vec2 point, vec2* hull, r32** bary_coords) {
	const r32 EPSILON = 0.000001f;
	r32 total_weight = 0.0f;
	r32* weights = array_new_len(vec2, array_length(hull));
	vec2* normals = array_new_len(vec2, array_length(hull));
	boolean degenerate = false;

	for (u32 i = 0; i < array_length(hull); ++i) {
		vec2 current = hull[i];
		vec2 next = hull[(i == array_length(hull) - 1) ? 0 : i + 1];

		vec2 edge = gm_vec2_subtract(next, current);
		vec2 n = gm_vec2_normalize((vec2){edge.y, -edge.x});
		array_push(normals, n);
	}

	for (u32 i = 0; i < array_length(hull); ++i) {
		vec2 n1 = normals[(i == 0) ? (array_length(hull) - 1) : i - 1];
		vec2 n2 = normals[i];
		vec2 vertex = hull[i];
		r32 numerator = fabsf(gm_vec2_cross(n1, n2));
		r32 denominator1 = gm_vec2_dot(n1, gm_vec2_subtract(vertex, point));
		r32 denominator2 = gm_vec2_dot(n2, gm_vec2_subtract(vertex, point));
		r32 denominator = denominator1 * denominator2;
		if (denominator > -EPSILON && denominator < EPSILON) {
			degenerate = true;
			break;
		}
		r32 weight = numerator / denominator;
		if (weight < 0.0f || weight > 1.0f) {
			// point is outside hull.
			array_free(weights);
			array_free(normals);
			return false;
		}
		array_push(weights, weight);
		total_weight += weight;
	}

	array_free(normals);

	if (degenerate) {
		// clear weights
		array_clear(weights);
		for (u32 i = 0; i < array_length(hull); ++i) {
			array_push(weights, 0.0f);
		}

		// find edge
		for (u32 i = 0; i < array_length(hull); ++i) {
			s32 current_idx = i;
			s32 next_idx = (i == array_length(hull) - 1) ? 0 : i + 1;
			vec2 current = hull[current_idx];
			vec2 next = hull[next_idx];
			r32 weight;

			if (point_lies_on_line_2d(current, next, point, &weight)) {
				weights[current_idx] = 1.0f - weight;
				weights[next_idx] = weight;
				*bary_coords = weights;
				return true;
			}
		}

		array_free(weights);
		return false;
	}

	for (u32 i = 0; i < array_length(weights); ++i) {
		weights[i] /= total_weight;
	}

	*bary_coords = weights;
	return true;
}