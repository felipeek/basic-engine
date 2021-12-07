#include "jarvis_march.h"
#include <float.h>
#include <light_array.h>

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
			} else if (cross == 0.0f) {
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