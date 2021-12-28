#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "gjk.h"
#include "ho_gjk.h"
#include <time.h>

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Perspective_Camera camera;
static Light* lights;
static Entity e1, e2;
static boolean is_mouse_bound_to_joint_target_position;
static Entity* bound_entity;

static boolean epa_render;
static vec3 epa_normal;
static r32 epa_penetration;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec3 camera_position = (vec3) { 0.0f, 0.0f, 5.0f };
	r32 camera_near_plane = -0.01f;
	r32 camera_far_plane = -1000.0f;
	r32 camera_fov = 45.0f;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

static Light* create_lights()
{
	Light light;
	Light* lights = array_new(Light);

	vec3 light_position = (vec3) {0.0f, 0.0f, 15.0f};
	vec4 ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	vec4 specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	return lights;
}

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

vec3* collect_vertices_of_entity(const Entity* e) {
	vec3* vertices = array_new(vec3);

	for (u32 i = 0; i < array_length(e->mesh.vertices); ++i) {
		const Vertex* v = &e->mesh.vertices[i];
		vec3 local_coords_position = v->position;
		mat4 model_matrix = e->model_matrix;
		vec3 world_coords_position = gm_mat4_multiply_vec3(&model_matrix, local_coords_position);

		boolean already_exists = false;

		for (u32 j = 0; j < array_length(vertices); ++j) {
			vec3 existing_element = vertices[j];
			if (gm_vec3_equal(world_coords_position, existing_element)) {
				already_exists = true;
				break;
			}
		}

		if (!already_exists) {
			array_push(vertices, world_coords_position);
		}
	}

	return vertices;
}

static vec3* e1_vertices;
static vec3* e2_vertices;
static GJK_Simplex gjk_simplex;
static int num_iterations = 1;
static EPA_Debug epa_debug;
static vec4* colors;

vec4 rand_color() {
	r32 f1 = (r32)rand() / RAND_MAX;
	r32 f2 = (r32)rand() / RAND_MAX;
	r32 f3 = (r32)rand() / RAND_MAX;
	vec4 c = (vec4){f1, f2, 0.5f, 1.0f};
	return c;
}

int core_init()
{
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e1, m, (vec3){0.0f, 0.0f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	Mesh m2 = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&e2, m2, (vec3){0.0f, 2.1f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	graphics_entity_create_with_color(&e2, m2, (vec3){-0.39f, 1.989f, 0.292f}, (Quaternion){-0.562f, -0.333f, -0.572f, 0.497f},
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	menu_register_dummy_callback(menu_dummy_callback);

	e1_vertices = collect_vertices_of_entity(&e1);
	e2_vertices = collect_vertices_of_entity(&e2);

	assert(gjk_collides(e1_vertices, e2_vertices, &gjk_simplex) == true);
	e1.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
	e2.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};

	epa_debug = epa(e1_vertices, e2_vertices, &gjk_simplex, &epa_normal, &epa_penetration, num_iterations++);

	colors = array_new(vec4);
	vec4 c = rand_color();
	array_push(colors, c);
	c = rand_color();
	array_push(colors, c);
	c = rand_color();
	array_push(colors, c);
	c = rand_color();
	array_push(colors, c);

	return 0;
}

void core_destroy()
{
	array_free(lights);
}

void core_update(r32 delta_time)
{
	//printf("Position: <%.3f, %.3f, %.3f>\n", e2.world_position.x, e2.world_position.y, e2.world_position.z);
	//printf("Rotation: <%.3f, %.3f, %.3f, %.3f>\n", e2.world_rotation.x, e2.world_rotation.y, e2.world_rotation.z, e2.world_rotation.w);
}

void core_render()
{
	//graphics_entity_render_phong_shader(&camera, &e1, lights);
	//graphics_entity_render_phong_shader(&camera, &e2, lights);
    //graphics_renderer_debug_vector((vec3){0.0f, 0.0f, 0.0f}, (vec3){1.0f, 0.0f, 0.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	for (u32 i = 0; i < array_length(epa_debug.faces); ++i) {
		dvec3 face = epa_debug.faces[i];
		vec3 p1 = epa_debug.polytope[face.x];
		vec3 p2 = epa_debug.polytope[face.y];
		vec3 p3 = epa_debug.polytope[face.z];

		graphics_renderer_debug_triangle(p1, p2, p3, colors[i]);
	}


	for (u32 i = 0; i < array_length(e1_vertices); ++i) {
		for (u32 j = 0; j < array_length(e2_vertices); ++j) {
			vec3 v1 = e1_vertices[i];
			vec3 v2 = e2_vertices[j];

			vec3 md = gm_vec3_subtract(v1, v2);
			graphics_renderer_debug_points(&md, 1, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
		}
	}

	dvec3 normal_face = epa_debug.faces[epa_debug.min_normal_face];
	vec3 p = (vec3){0.0f, 0.0f, 0.0f};
	p = gm_vec3_add(epa_debug.polytope[normal_face.x], epa_debug.polytope[normal_face.y]);
	p = gm_vec3_add(p, epa_debug.polytope[normal_face.z]);
	p = gm_vec3_scalar_product(1.0f / 3.0f, p);
	graphics_renderer_debug_vector(p, gm_vec3_add(epa_debug.min_normal, p), (vec4){1.0f, 1.0f, 1.0f, 1.0f});

	// render origin
	vec3 origin = {0};
	graphics_renderer_debug_points(&origin, 1, (vec4){1.0f, 1.0f, 1.0f, 1.0f});

	//graphics_renderer_debug_vector((vec3){0.0f, 0.0f, 0.0f}, gm_vec3_scalar_product(epa_penetration, epa_normal), (vec4){1.0f, 0.0f, 0.0f, 1.0f});
    graphics_renderer_primitives_flush(&camera);
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 60.0f;

	if (key_state[GLFW_KEY_LEFT_SHIFT]) {
		movement_speed = 0.5f;
		rotation_speed = 30.0f;
	}
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.1f;

	is_mouse_bound_to_joint_target_position = false;
	if (key_state[GLFW_KEY_1])
	{
		bound_entity = &e2;
		is_mouse_bound_to_joint_target_position = true;
	}

	if (key_state[GLFW_KEY_W])
		camera_move_forward(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_S])
		camera_move_forward(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_A])
		camera_move_right(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_D])
		camera_move_right(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e2, quaternion_product(&rotation, &e2.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_L])
	{
		static boolean wireframe = false;

		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		wireframe = !wireframe;
		key_state[GLFW_KEY_L] = false;
	}

	if (key_state[GLFW_KEY_SPACE]) {
		vec3 delta = gm_vec3_scalar_product(epa_penetration, epa_normal);
		graphics_entity_set_position(&e2, gm_vec3_add(e2.world_position, delta));
		key_state[GLFW_KEY_SPACE] = false;
	}

	if (key_state[GLFW_KEY_M]) {
		epa_debug = epa(e1_vertices, e2_vertices, &gjk_simplex, &epa_normal, &epa_penetration, num_iterations++);
		if (epa_debug.converged) {
			printf("Finished.\n");
		}
		vec4 c = rand_color();
		array_push(colors, c);
		key_state[GLFW_KEY_M] = false;
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
	static r64 x_pos_old, y_pos_old;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	x_pos_old = x_pos;
	y_pos_old = y_pos;

	if (reset) return;

	if (is_mouse_bound_to_joint_target_position && bound_entity != NULL)
	{
		// MOVE TARGET POSITIONS!
		vec3 camera_y = camera_get_y_axis(&camera);
		vec3 camera_x = camera_get_x_axis(&camera);

		static const r32 target_point_move_speed = 0.001f;
		vec3 y_diff = gm_vec3_scalar_product(-target_point_move_speed * (r32)y_difference, camera_y);
		vec3 x_diff = gm_vec3_scalar_product(target_point_move_speed * (r32)x_difference, camera_x);

		vec3 e_pos = bound_entity->world_position;
		e_pos = gm_vec3_add(e_pos, y_diff);
		e_pos = gm_vec3_add(e_pos, x_diff);
		graphics_entity_set_position(bound_entity, e_pos);
	}
	else
	{
		// NORMAL CAMERA MOVEMENT!
		static const r32 camera_mouse_speed = 0.1f;
		camera_rotate_x(&camera, camera_mouse_speed * (r32)x_difference);
		camera_rotate_y(&camera, camera_mouse_speed * (r32)y_difference);
	}
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos)
{

}

void core_scroll_change_process(r64 x_offset, r64 y_offset)
{

}

void core_window_resize_process(s32 width, s32 height)
{
	camera_force_matrix_recalculation(&camera);
}