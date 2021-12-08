#include <GLFW/glfw3.h>
#include <time.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "menu.h"
#include <float.h>

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Entity* entities;
static Entity* edges;
vec2* points;
vec2* hull;
Mesh circle_mesh;
vec2 center;

typedef struct {
	vec2 point;
	int selected_index;
	vec2 last_selected_point_to_me;
	r32 last_selected_point_to_me_distance;
} Jarvis_March_Point;

typedef struct {
	vec2 point;
	boolean processed;
} Point_To_Process;

static Point_To_Process point_to_process(vec2 point) {
	Point_To_Process ptp;
	ptp.point = point;
	ptp.processed = false;
	return ptp;
}

static Jarvis_March_Point jarvis_march_point(vec2 point, int selected_index, vec2 last_selected_point_to_me) {
	Jarvis_March_Point jmp;
	jmp.point = point;
	jmp.selected_index = selected_index;
	jmp.last_selected_point_to_me = last_selected_point_to_me;
	jmp.last_selected_point_to_me_distance = gm_vec2_length(last_selected_point_to_me);
	return jmp;
}

static int get_leftest_point(vec2* points) {
	r32 smallest_x = FLT_MAX;
	int selected_index;
	for (u32 i = 0; i < array_length(points); ++i) {
		vec2 p = points[i];
		if (p.x < smallest_x) {
			smallest_x = p.x;
			selected_index = i;
		}
	}
	return selected_index;
}

vec2* jarvis_march(vec2* points) {
	// Corner-cases
	if (array_length(points) == 0 || array_length(points) == 1 || array_length(points) == 2) {
		return array_copy(points);
	}

	// convex_hull will hold the result
	vec2* convex_hull = array_new_len(vec2, array_length(points));
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
	vec2 start_point = points[start_point_index];
	array_push(convex_hull, start_point);						// add it to the convex hull
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
			
			Jarvis_March_Point current_point = jarvis_march_point(ptp->point, i, gm_vec2_subtract(ptp->point, last_selected_point));

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
			array_push(convex_hull, colinear_points[i].point);
			points_to_process[colinear_points[i].selected_index].processed = true;
		}
		array_clear(colinear_points);

		// If the candidate point is the start point, we closed the convex hull and, therefore, we are done.
		if (candidate_point.selected_index == start_point_index) {
			break;
		}

		// Add the candidate point to the hull.
		array_push(convex_hull, candidate_point.point);
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

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

static u32* generate_indices_for(Vertex* vertices) {
	u32* indices = array_new(u32);
	for (u32 i = 1; i < array_length(vertices); ++i) {
		array_push(indices, i - 1);
		array_push(indices, i);
	}

	array_push(indices, array_length(vertices) - 1);
	array_push(indices, 0);
	return indices;
}

static void add_vertex(Vertex** vertices, r32 x, r32 y) {
	Vertex v;
	v.position = (vec2){x, y};
	array_push(*vertices, v);
}

r32 delta = 0.0f;

static void create_circle_vertices(Vertex* vertices, vec2 center) {
	for (u32 i = 0; i < array_length(vertices); ++i) {
		r32 theta = ((r32)i / array_length(vertices)) * (2.0f * PI_F);
		theta += delta;
		vertices[i].position = gm_vec2_scalar_product(0.01f, (vec2){cosf(theta), sinf(theta)});
		vertices[i].position = gm_vec2_add(vertices[i].position, center);
	}
}

int core_init()
{
	Entity e;

	entities = array_new(Entity);
	edges = array_new(Entity);
	Vertex* circle_vertices = array_new(Vertex);
	points = array_new(vec2);
	hull = array_new(vec2);

	for (r32 i = 0; i < 2000; ++i) {
		add_vertex(&circle_vertices, 0.0f, 0.0f);
	}
	create_circle_vertices(circle_vertices, (vec2){0.0f, 0.0f});
	circle_mesh = graphics_mesh_create(circle_vertices, generate_indices_for(circle_vertices));

#if 1
	vec2 p1 = (vec2){-2.0f, 1.0f};
	vec2 p2 = (vec2){1.0f, 1.0f};
	vec2 p3 = (vec2){4.0f, 2.0f};
	vec2 p4 = (vec2){2.0f, 2.0f};
	vec2 p5 = (vec2){4.0f, 4.0f};
	vec2 p6 = (vec2){2.0f, -1.0f};
	vec2 p7 = (vec2){4.0f, -3.0f};
	vec2 p8 = (vec2){-1.0f, -3.0f};
	array_push(points, p1);
	array_push(points, p2);
	array_push(points, p3);
	array_push(points, p4);
	array_push(points, p5);
	array_push(points, p6);
	array_push(points, p7);
	array_push(points, p8);
#else
	srand(time(0));
	for (u32 i = 0; i < 100; ++i) {
		vec2 p = (vec2){20.0f * ((r32)rand() / RAND_MAX) - 10.0f, 20.0f * ((r32)rand() / RAND_MAX) - 10.0f};
		array_push(points, p);
	}
#endif

	menu_register_dummy_callback(menu_dummy_callback);
	return 0;
}

void core_destroy()
{
}

void core_update(r32 delta_time)
{
	Entity e;
	array_clear(entities);

	for (u32 i = 0; i < array_length(points); ++i) {
		vec2 point = points[i];
		vec2 norm_point = (vec2){point.x / 10.0f, point.y / 10.0f};
		graphics_entity_create_with_color(&e, circle_mesh, norm_point, (vec3){1.0f, 1.0f, 1.0f});	
		array_push(entities, e);
	}

	vec2 norm_point = (vec2){center.x / 10.0f, center.y / 10.0f};
	graphics_entity_create_with_color(&e, circle_mesh, norm_point, (vec3){0.0f, 1.0f, 0.0f});
	array_push(entities, e);
}

void core_render()
{
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_basic_shader(&entities[i]);
	}

	for (u32 i = 0; i < array_length(edges); ++i) {
		graphics_entity_render_basic_shader(&edges[i]);
	}
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	if (key_state[GLFW_KEY_SPACE]) {
		hull = jarvis_march(points);
		array_clear(edges);

		for (u32 i = 0; i < array_length(hull); ++i) {
			vec2 current = hull[i];
			vec2 next = hull[(i == array_length(hull) - 1) ? 0 : i + 1];

			Vertex* vertices = array_new(Vertex);
			Vertex v;
			v.position = gm_vec2_scalar_product(0.1f, current);
			array_push(vertices, v);
			v.position = gm_vec2_scalar_product(0.1f, next);
			array_push(vertices, v);
			u32* indices = array_new(u32);
			array_push(indices, 0);
			array_push(indices, 1);

			Mesh m = graphics_mesh_create(vertices, indices);
			Entity e;
			graphics_entity_create_with_color(&e, m, (vec2){0.0f, 0.0f}, (vec3){1.0f, 1.0f, 1.0f});
			array_push(edges, e);

			printf("<%.3f, %.3f>\n", current.x, current.y);
		}

		center = (vec2){0.0f, 0.0f};
		r32 area = 0.0f;
		for (u32 i = 0; i < array_length(hull); ++i) {
			vec2 current = hull[i];
			vec2 next = hull[(i == array_length(hull) - 1) ? 0 : i + 1];

			r32 tmp = current.x * next.y - next.x * current.y;
			area += 0.5f * tmp;

			r32 cx = (current.x + next.x) * tmp;
			r32 cy = (current.y + next.y) * tmp;

			center = gm_vec2_add(center, (vec2){cx, cy});
		}
		
		center = gm_vec2_scalar_product(1.0f / (6.0f * area), center);

		key_state[GLFW_KEY_SPACE] = false;
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos)
{

}

void core_scroll_change_process(r64 x_offset, r64 y_offset)
{

}

void core_window_resize_process(s32 width, s32 height)
{
}