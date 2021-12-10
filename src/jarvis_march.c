#include "jarvis_march.h"
#include <float.h>
#include <light_array.h>

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))

typedef struct {
	vec2 point;
	int selected_index;
	vec2 last_selected_point_to_me;
	r32 last_selected_point_to_me_distance;
	Projected_Support_Point psp;
} Jarvis_March_Point;

typedef struct {
	vec2 point;
	boolean processed;
	Projected_Support_Point psp;
} Point_To_Process;

static Point_To_Process point_to_process(Projected_Support_Point psp) {
	Point_To_Process ptp;
	ptp.point = psp.pv2;
	ptp.processed = false;
	ptp.psp = psp;
	return ptp;
}

static Jarvis_March_Point jarvis_march_point(vec2 point, int selected_index, vec2 last_selected_point_to_me, Projected_Support_Point psp) {
	Jarvis_March_Point jmp;
	jmp.point = point;
	jmp.selected_index = selected_index;
	jmp.last_selected_point_to_me = last_selected_point_to_me;
	jmp.last_selected_point_to_me_distance = gm_vec2_length(last_selected_point_to_me);
	jmp.psp = psp;
	return jmp;
}

static int get_leftest_point(Projected_Support_Point* points) {
	r32 smallest_x = FLT_MAX;
	int selected_index;
	for (u32 i = 0; i < array_length(points); ++i) {
		Projected_Support_Point psp = points[i];
		if (psp.pv2.x < smallest_x) {
			smallest_x = psp.pv2.x;
			selected_index = i;
		}
	}
	return selected_index;
}

// Returns points in CCW winding
Projected_Support_Point* jarvis_march(Projected_Support_Point* points) {
	// Corner-cases
	if (array_length(points) == 0 || array_length(points) == 1 || array_length(points) == 2) {
		return array_copy(points);
	}

	// convex_hull will hold the result
	Projected_Support_Point* convex_hull = array_new_len(Projected_Support_Point, array_length(points));
	// points_to_process will hold all points that shall be processed
	Point_To_Process* points_to_process = array_new_len(Point_To_Process, array_length(points));
	for (u32 i = 0; i < array_length(points); ++i) {
		Point_To_Process ptp = point_to_process(points[i]);
		array_push(points_to_process, ptp);
	}
	// colinear_points holds all colinear points selected for a given iteration
	Jarvis_March_Point* colinear_points = array_new_len(Jarvis_March_Point, 16);
	
	// Get the leftest point. We know that this point will always be in the convex hull. It will be our starting point.
	int start_point_index = get_leftest_point(points);
	vec2 start_point = points[start_point_index].pv2;
	array_push(convex_hull, points[start_point_index]);						// add it to the convex hull
	// Mark this point as processed so we ignore it during the iterations
	// NOTE(fek): after the first iteration, this will go back to unprocessed because the start point needs to be selected again to close the hull!
	points_to_process[start_point_index].processed = true;
	
	// Now we start the algorithm

	// last_selected_point holds the last point that was added to the convex hull
	vec2 last_selected_point = start_point;
	// candidate_point holds the point that is currently selected as the next selected point, for a given iteration
	Jarvis_March_Point candidate_point;
	boolean has_candidate_point = false;

	while (true) {
		for (u32 i = 0; i < array_length(points_to_process); ++i) {
			Point_To_Process* ptp = &points_to_process[i];

			// If the point was already processed, skip
			if (ptp->processed) {
				continue;
			}
			
			Jarvis_March_Point current_point = jarvis_march_point(ptp->point, i, gm_vec2_subtract(ptp->point, last_selected_point), ptp->psp);

			// If we don't have a candidate point yet, pick this one and go to the next iteration
			if (!has_candidate_point) {
				candidate_point = current_point;
				has_candidate_point = true;
				continue;
			}

			// Here we compare if the current point should be the new candidate point by comparing the vectors going from
			// the last selected point to the current point with the vector going from the last selected point to the candidate point
			r32 cross = gm_vec2_cross(candidate_point.last_selected_point_to_me, current_point.last_selected_point_to_me);
			if (cross < 0.0f) {
				// In this case, the current point is the new candidate point
				candidate_point = current_point;
				// Clear the colinear points since they were related to the last candidate point - they don't apply anymore.
				array_clear(colinear_points);
			} else if (cross == 0.0f) { // This test determines whether the output will be CW or CCW
				// In this case, the current point and the candidate point are colinear points. We need to handle them specially.
				// We check the which one is closer to the last selected point. The closer one goes to the colinear_points array.
				if (current_point.last_selected_point_to_me_distance > candidate_point.last_selected_point_to_me_distance) {
					// The current point comes first! Therefore it is our new candidate point. The old candidate point goes to the array
					array_push(colinear_points, candidate_point);
					candidate_point = current_point;
				} else {
					// The candidate point comes first. In this case, the current point goes to the array. We need to be careful and add
					// it to the array keeping the order from closer to farthest.
					boolean inserted = false;
					for (u32 j = 0; j < array_length(colinear_points); ++j) {
						Jarvis_March_Point current_colinear_point = colinear_points[j];
						if (current_point.last_selected_point_to_me_distance < current_colinear_point.last_selected_point_to_me_distance) {
							array_insert(colinear_points, current_point, j);
							inserted = true;
							break;
						}
					}
					if (!inserted) {
						array_push(colinear_points, current_point);
					}
				}
			}
		}

		// At this point, the iteration is done and we have our selected point in the 'candidate_point' and all its colinear points.

		// Start by adding all colinear points to the hull, keeping their order.
		for (u32 i = 0; i < array_length(colinear_points); ++i) {
			array_push(convex_hull, colinear_points[i].psp);
			points_to_process[colinear_points[i].selected_index].processed = true;
		}
		array_clear(colinear_points);

		// If the candidate point is the start point, we closed the convex hull and, therefore, we are done.
		if (candidate_point.selected_index == start_point_index) {
			break;
		}

		// Add the candidate point to the hull.
		array_push(convex_hull, candidate_point.psp);
		points_to_process[candidate_point.selected_index].processed = true;

		// Update the last selected point
		last_selected_point = candidate_point.point;

		// Prepare the next iteration by setting has_candidate_point = false
		has_candidate_point = false;
		
		// Force the start point to have 'processed' equals 'false'!
		// This is important because we always need to process it since we need to close the hull (except in the first iteration!)
		points_to_process[start_point_index].processed = false;
	}

	array_free(points_to_process);
	array_free(colinear_points);
	return convex_hull;
}

static vec2 intersection_line_line(vec2 p1, vec2 p2, vec2 p3, vec2 p4) {
	vec2 result;

	r32 D = (p1.x - p2.x) * (p3.y - p4.y) - (p1.y - p2.y) * (p3.x - p4.x);

	result.x = ((((p1.x * p2.y) - (p1.y * p2.x)) * (p3.x - p4.x)) - ((p1.x - p2.x) * ((p3.x * p4.y) - (p3.y * p4.x)))) / D;
	result.y = ((((p1.x * p2.y) - (p1.y * p2.x)) * (p3.y - p4.y)) - ((p1.y - p2.y) * ((p3.x * p4.y) - (p3.y * p4.x)))) / D;

	return result;
}

// Accepts points in CCW winding and returns points in CCW winding
vec2* sutherland_hodgman(vec2* subject, vec2* clip) {
	vec2* output_list = array_copy(subject);

	for (u32 i = 0; i < array_length(clip); ++i) {
		vec2 e1 = clip[i];
		vec2 e2 = clip[(i == array_length(clip) - 1) ? 0 : i + 1];
		vec2 edge = gm_vec2_subtract(e2, e1);

		vec2* input_list = array_copy(output_list);
		array_clear(output_list);

		for (u32 j = 0; j < array_length(input_list); ++j) {
			vec2 current_point = input_list[j];
			vec2 prev_point = input_list[(j == 0) ? (array_length(input_list) - 1): j - 1];

			vec2 e1_to_current_point = gm_vec2_subtract(current_point, e1);
			vec2 e1_to_prev_point = gm_vec2_subtract(prev_point, e1);
			if (gm_vec2_cross(edge, e1_to_current_point) >= 0.0f) {
				if (gm_vec2_cross(edge, e1_to_prev_point) < 0.0f) {
					vec2 intersecting_point = intersection_line_line(prev_point, current_point, e1, e2);
					array_push(output_list, intersecting_point);
				}
				array_push(output_list, current_point);
			} else if (gm_vec2_cross(edge, e1_to_prev_point) >= 0.0f) {
				vec2 intersecting_point = intersection_line_line(prev_point, current_point, e1, e2);
				array_push(output_list, intersecting_point);
			}
		}
	}

	return output_list;
}

boolean PointInPlane(const Plane* plane, vec3 position) {
	float distance = -gm_vec3_dot(plane->normal, plane->point);
	if (gm_vec3_dot(position, plane->normal) + distance < 0.0f) {
		return false;
	}

	return true;
}

boolean PlaneEdgeIntersection(
	const Plane* plane,
	const vec3 start,
	const vec3 end,
	vec3* out_point)
{
	vec3 ab = gm_vec3_subtract(end, start);

	//Check that the edge and plane are not parallel and thus never intersect
	// We do this by projecting the line (start - A, End - B) ab along the plane
	float ab_p = gm_vec3_dot(plane->normal, ab);
	if (fabs(ab_p) > 1e-6f)
	{
		//Generate a random point on the plane (any point on the plane will suffice)
		float distance = -gm_vec3_dot(plane->normal, plane->point);
		vec3 p_co = gm_vec3_scalar_product(-distance, plane->normal);

		//Work out the edge factor to scale edge by
		// e.g. how far along the edge to traverse before it meets the plane.
		//      This is computed by: -proj<plane_nrml>(edge_start - any_planar_point) / proj<plane_nrml>(edge_start - edge_end)
		float fac = -gm_vec3_dot(plane->normal, gm_vec3_subtract(start, p_co)) / ab_p;

		//Stop any large floating point divide issues with almost parallel planes
		fac = min(max(fac, 0.0f), 1.0f); 

		//Return point on edge
		*out_point = gm_vec3_add(start, gm_vec3_scalar_product(fac, ab));
		return true;
	}

	return false;
}

//Performs sutherland hodgman clipping algorithm to clip the provided mesh
//    or polygon in regards to each of the provided clipping planes.
void SutherlandHodgmanClipping(
	vec3* input_polygon,
	int num_clip_planes,
	const Plane* clip_planes,
	vec3** out_polygon,
	boolean removeNotClipToPlane)
{
	assert(out_polygon != NULL);
	assert(num_clip_planes > 0);

	//Create temporary list of vertices
	// - We will keep ping-pong'ing between the two lists updating them as we go.
	vec3* input = array_copy(input_polygon);
	vec3* output = array_new(vec3);

	//Iterate over each clip_plane provided
	for (int i = 0; i < num_clip_planes; ++i)
	{
		//If every single point on our shape has already been removed previously, just exit
		if (array_length(input) == 0)
			break;

		const Plane* plane = &clip_planes[i];

		//Loop through each edge of the polygon (see line_loop from gfx) and clip
		// that edge against the current plane.
		vec3 tempPoint, startPoint = input[array_length(input) - 1];
		for (u32 j = 0; j < array_length(input); ++j)
		{
			vec3 endPoint = input[j];
			boolean startInPlane = PointInPlane(plane, startPoint);
			boolean endInPlane = PointInPlane(plane, endPoint);

			//If it's the final pass, just remove all points outside the reference plane
			// - This won't return a true polygon if set, but is needed for the last step
			//   we do in the manifold generation
			if (removeNotClipToPlane)
			{
				if (endInPlane)
					array_push(output, endPoint);
			}
			else
			{
				//If the edge is entirely within the clipping plane, keep it as it is
				if (startInPlane && endInPlane)
				{
					array_push(output, endPoint);
				}
				//If the edge interesects the clipping plane, cut the edge along clip plane
				else if (startInPlane && !endInPlane)
				{
					if (PlaneEdgeIntersection(plane, startPoint, endPoint, &tempPoint))
						array_push(output, tempPoint);
				}
				else if (!startInPlane && endInPlane)
				{
					if (PlaneEdgeIntersection(plane, startPoint, endPoint, &tempPoint))
						array_push(output, tempPoint);

					array_push(output, endPoint);
				}
			}
			//..otherwise the edge is entirely outside the clipping plane and should be removed/ignored

			startPoint = endPoint;
		}

		//Swap input/output polygons, and clear output list for us to generate afresh
		vec3* tmp = input;
		input = output;
		output = tmp;
		array_clear(output);
	}

	*out_polygon = input;
}