#include "convex_bary_coords.h"
#include <light_array.h>

// checks whether a point lies on a line (2D)
// l1_x, l1_y -> line's first point
// l2_x, l2_y -> line's second point
// p_x, p_y -> point of interest
// w -> output indicating the weight of the point
// returns 1 if point lies on the line, 0 if not
// https://gist.github.com/felipeek/93c873395506868e50ea6f2323eb3399
static boolean point_lies_on_line_2d(vec2 l1, vec2 l2, vec2 p, r32 *w)
{
	const r32 EPSILON = 0.000001f;
	boolean vertical_line = (l1.x - l2.x) > -EPSILON && (l1.x - l2.x) < EPSILON;   // vertical line
	boolean horizontal_line = (l1.y - l2.y) > -EPSILON && (l1.y - l2.y) < EPSILON; // horizontal line

	if (vertical_line && horizontal_line)
	{ // degenerate case -> line is composed of a single point
		if (((p.x - l1.x) > -EPSILON && (p.x - l1.x) < EPSILON) &&
			((p.y - l1.y) > -EPSILON && (p.y - l1.y) < EPSILON))
		{
			// in this case, 'w' has no real meaning
			*w = 0.5f;
			return true;
		}
	}

	if (vertical_line)
	{ // vertical line
		if ((p.x - l1.x) > -EPSILON && (p.x - l1.x) < EPSILON)
		{ // p also lies on same line?
			r32 alpha = (p.y - l1.y) / (l2.y - l1.y);
			if (alpha < 0.0f || alpha > 1.0f)
			{
				return false;
			}
			*w = alpha;
			return true;
		}

		return false;
	}

	if (horizontal_line)
	{ // horizontal line
		if ((p.y - l1.y) > -EPSILON && (p.y - l1.y) < EPSILON)
		{ // p also lies on same line?
			r32 alpha = (p.x - l1.x) / (l2.x - l1.x);
			if (alpha < 0.0f || alpha > 1.0f)
			{
				return false;
			}
			*w = alpha;
			return true;
		}

		return false;
	}

	// general case
	r32 alpha_x = (p.x - l1.x) / (l2.x - l1.x);
	if (alpha_x < 0.0f || alpha_x > 1.0f)
	{
		return false;
	}
	r32 alpha_y = (p.y - l1.y) / (l2.y - l1.y);
	if (alpha_y < 0.0f || alpha_y > 1.0f)
	{
		return false;
	}

	r32 alpha_diff = alpha_x - alpha_y;
	if (alpha_diff > -EPSILON && alpha_diff < EPSILON)
	{
		// edge found
		*w = alpha_x;
		return true;
	}

	return false;
}

static boolean get_barycentric_coords_relative_to_point(vec2 point, vec2 hull, r32 **bary_coords)
{
	const r32 EPSILON = 0.000001f;

	if ((point.x - hull.x) > -EPSILON && (point.x - hull.x) < EPSILON &&
	((point.y - hull.y) > -EPSILON && (point.y - hull.y) < EPSILON)
		)
	{
		*bary_coords = array_new(r32);
		array_push(*bary_coords, 1.0f);
		return true;
	}

	return false;
}

static boolean get_barycentric_coords_relative_to_line(vec2 point, vec2 l1, vec2 l2, r32 **bary_coords)
{
	r32 w;
	if (point_lies_on_line_2d(l1, l2, point, &w))
	{
		*bary_coords = array_new(r32);
		array_push(*bary_coords, 1.0f - w);
		array_push(*bary_coords, w);
		return true;
	}

	return false;
}

static boolean get_barycentric_coords_relative_to_triangle(vec2 point, vec2 a, vec2 b, vec2 c, r32 **bary_coords)
{
	// code from Crister Erickson's Real-Time Collision Detection
	vec2 v0 = gm_vec2_subtract(b, a);
	vec2 v1 = gm_vec2_subtract(c, a);
	vec2 v2 = gm_vec2_subtract(point, a);
	r32 d00 = gm_vec2_dot(v0, v0);
	r32 d01 = gm_vec2_dot(v0, v1);
	r32 d11 = gm_vec2_dot(v1, v1);
	r32 d20 = gm_vec2_dot(v2, v0);
	r32 d21 = gm_vec2_dot(v2, v1);
	r32 denom = d00 * d11 - d01 * d01;
	r32 w0 = (d11 * d20 - d01 * d21) / denom;
	if (w0 < 0.0f || w0 > 1.0f)
	{
		return false;
	}
	r32 w1 = (d00 * d21 - d01 * d20) / denom;
	if (w1 < 0.0f || w1 > 1.0f)
	{
		return false;
	}
	r32 w2 = 1.0f - w0 - w1;
	if (w2 < 0.0f || w2 > 1.0f)
	{
		return false;
	}

	*bary_coords = array_new(r32);
	array_push(*bary_coords, w0);
	array_push(*bary_coords, w1);
	array_push(*bary_coords, w2);

	return true;
}

boolean get_barycentric_coords_relative_to_arbitrary_convex_hull(vec2 point, vec2 *hull, r32 **bary_coords)
{
	const r32 EPSILON = 0.000001f;
	// Barycentric Coordinates for Convex Sets (Joe Warren et al)

	// Collect normals
	vec2 *normals = array_new_len(vec2, array_length(hull));
	for (u32 i = 0; i < array_length(hull); ++i)
	{
		vec2 current = hull[i];
		vec2 next = hull[(i == array_length(hull) - 1) ? 0 : i + 1];

		vec2 edge = gm_vec2_subtract(next, current);
		vec2 n = gm_vec2_normalize((vec2){edge.y, -edge.x});
		array_push(normals, n);
	}

	// Calculate barycentric coords for arbitrary convex hull
	boolean degenerate = false;
	r32 total_weight = 0.0f;
	*bary_coords = array_new(r32);
	for (u32 i = 0; i < array_length(hull); ++i)
	{
		vec2 n1 = normals[(i == 0) ? (array_length(hull) - 1) : i - 1];
		vec2 n2 = normals[i];
		vec2 vertex = hull[i];
		r32 numerator = fabsf(gm_vec2_cross(n1, n2));
		r32 denominator1 = gm_vec2_dot(n1, gm_vec2_subtract(vertex, point));
		r32 denominator2 = gm_vec2_dot(n2, gm_vec2_subtract(vertex, point));
		r32 denominator = denominator1 * denominator2;
		if (denominator > -EPSILON && denominator < EPSILON)
		{
			// if denominator is close to 0.0, we have a degenerate case
			// the point is in the border of the hull
			degenerate = true;
			break;
		}
		r32 weight = numerator / denominator;
		array_push(*bary_coords, weight);
		total_weight += weight;
	}

	array_free(normals);

	if (degenerate)
	{
		// solve degenerate case (point is in the border of convex hull)

		// clear weights
		array_clear(*bary_coords);
		for (u32 i = 0; i < array_length(hull); ++i)
		{
			array_push(*bary_coords, 0.0f);
		}

		// find edge that contains the point
		for (u32 i = 0; i < array_length(hull); ++i)
		{
			s32 current_idx = i;
			s32 next_idx = (i == array_length(hull) - 1) ? 0 : i + 1;
			vec2 current = hull[current_idx];
			vec2 next = hull[next_idx];
			r32 weight;

			if (point_lies_on_line_2d(current, next, point, &weight))
			{
				(*bary_coords)[current_idx] = 1.0f - weight;
				(*bary_coords)[next_idx] = weight;
				return true;
			}
		}

		// it is possible that no edge contains the point
		// this happens when the point lies on the infinite line generated by the edge
		// but it is not in the edge (it lies outside the edge, but within the line)
		array_free(*bary_coords);
		return false;
	}

	for (u32 i = 0; i < array_length(*bary_coords); ++i)
	{
		(*bary_coords)[i] /= total_weight;
		if ((*bary_coords)[i] < 0.0f || (*bary_coords)[i] > 1.0f)
		{
			// point is outside hull.
			array_free(*bary_coords);
			return false;
		}
	}

	return true;
}

boolean convex_bary_coords_get(vec2 point, vec2 *hull, r32 **bary_coords)
{
	const r32 EPSILON = 0.000001f;
	assert(array_length(hull) > 0);

	if (array_length(hull) == 1)
	{
		return get_barycentric_coords_relative_to_point(point, hull[0], bary_coords);
	}
	else if (array_length(hull) == 2)
	{
		return get_barycentric_coords_relative_to_line(point, hull[0], hull[1], bary_coords);
	}
	else if (array_length(hull) == 3)
	{
		return get_barycentric_coords_relative_to_triangle(point, hull[0], hull[1], hull[2], bary_coords);
	}

	return get_barycentric_coords_relative_to_arbitrary_convex_hull(point, hull, bary_coords);
}