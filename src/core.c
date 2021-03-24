#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "physics.h"
#include "collision.h"

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}
#define PLANE_Y (0.0f)

static Perspective_Camera camera;
static Light* lights;
static Entity* entities;

static boolean is_mouse_bound_to_joint_target_position;
static Entity* bound_entity;
static Render_Primitives_Context r_ctx;

static boolean stop = false;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 2.0f, 10.0f, 1.0f};
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

	vec4 light_position = (vec4) {0.0f, 0.0f, 15.0f, 1.0f};
	vec4 ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	vec4 specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	light_position = (vec4) {0.0f, 0.0f, -15.0f, 1.0f};
	ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, light);

	return lights;
}

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

int core_init()
{
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	// Create entities
	entities = array_new(Entity);
	Entity e;
	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec4){0.0f, PLANE_Y - 1.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1000.0f, 1.0f, 1000.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	array_push(entities, e);
	m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec4){0.0f, 0.0f, -10.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1000.0f, 1000.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.4f, 1.0f}, 1000000000.0f);
	array_push(entities, e);
	//m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&e, m, (vec4){0.0f, 0.3f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){0.3f, 0.3f, 2.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);
	//array_push(entities, e);
	//m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&e, m, (vec4){1.0f, 1.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){5.0f, 0.05f, 0.7f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);
	//array_push(entities, e);
	//m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&e, m, (vec4){-3.0f, 2.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){0.5f, 0.5f, 0.5f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 100.0f);
	//array_push(entities, e);
	m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec4){0.0f, 10.0f, 0.0f, 1.0f}, quaternion_new((vec3){3.0f, 1.0f, 0.5f}, 45.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);
	array_push(entities, e);

	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, PLANE_Y, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1000.0f, 1000.0f, 1000.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/real_plane.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){0.0f, 5.0f, 0.0f, 1.0f}, quaternion_new((vec3){3.0f, 1.0f, 0.5f}, 45.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);

	menu_register_dummy_callback(menu_dummy_callback);

	graphics_renderer_primitives_init(&r_ctx);
	bound_entity = &entities[0];

	return 0;
}

void core_destroy()
{
	array_free(lights);
}

vec3 col_point;
boolean collision;
vec3 penetration;

void core_update(r32 delta_time)
{
	if (stop) return;

	Physics_Force gravity_force;
	gravity_force.position = (vec3) {0.0f, 0.0f, 0.0f};
	for (u32 i = 0; i < array_length(entities); ++i) {
		if (entities[i].mass < MAX_MASS_TO_CONSIDER_STATIC_BODY) {
			gravity_force.force = (vec3){0.0f, -10.0f * entities[i].mass, 0.0f};
			array_push(entities[i].forces, gravity_force);
		}
	}

	physics_simulate(entities, delta_time);

	for (u32 i = 0; i < array_length(entities); ++i) {
		array_clear(entities[i].forces);
	}
}

void core_render()
{
	for (u32 i = 0; i < array_length(entities); ++i) {
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);
	}

	if (collision) {
		graphics_renderer_debug_points(&r_ctx, &col_point, 1, (vec4){1.0f, 1.0f, 1.0f});
		graphics_renderer_debug_vector(&r_ctx, col_point, gm_vec3_add(col_point, penetration), (vec4){1.0f, 1.0f, 1.0f, 1.0f});
		graphics_renderer_primitives_flush(&r_ctx, &camera);
	}
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 300.0f;

	if (key_state[GLFW_KEY_LEFT_SHIFT])
		movement_speed = 0.5f;
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.1f;

	if (key_state[GLFW_KEY_W])
		camera_move_forward(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_S])
		camera_move_forward(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_A])
		camera_move_right(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_D])
		camera_move_right(&camera, movement_speed * delta_time);
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

	is_mouse_bound_to_joint_target_position = false;
	if (key_state[GLFW_KEY_1])
	{
		bound_entity = &entities[0];
		is_mouse_bound_to_joint_target_position = true;
	}
	if (key_state[GLFW_KEY_2])
	{
		bound_entity = &entities[1];
		is_mouse_bound_to_joint_target_position = true;
	}

/*
	if (key_state[GLFW_KEY_U])
	{
		stop = true;
		key_state[GLFW_KEY_U] = false;
	}
	*/

	if (key_state[GLFW_KEY_Q] || key_state[GLFW_KEY_9]) {
		// MOVE TARGET POSITIONS!
		vec3 camera_z = camera_get_z_axis(&camera);
		vec4 camera_pos = camera.position;
		r32 distance = 5.0f;
		vec3 diff = gm_vec3_scalar_product(-distance, camera_z);
		vec4 cube_position = gm_vec4_add(camera_pos, (vec4){diff.x, diff.y, diff.z, 0.0f});

		Entity e;
		char* mesh_name;
		int r = rand();
		if (r % 3 == 0) {
			mesh_name = "./res/cube.obj";
		} else if (r % 3 == 1) {
			mesh_name = "./res/ico.obj";
		} else {
			mesh_name = "./res/cone.obj";
		}
		Mesh m = graphics_mesh_create_from_obj(mesh_name, 0);
		graphics_entity_create_with_color(&e, m, cube_position, quaternion_new((vec3){0.35f, 0.44f, 0.12f}, 33.0f),
			(vec3){1.0f, 1.0f, 1.0f}, (vec4){rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, 1.0f}, 10.0f);

		Physics_Force force;
		force.force = gm_vec3_scalar_product(10000.0f, gm_vec3_scalar_product(-1.0f, camera_z));
		force.position = (vec3) {0.0f, 0.0f, 0.0f};
		array_push(e.forces, force);

		array_push(entities, e);
		key_state[GLFW_KEY_Q] = false;
		key_state[GLFW_KEY_9] = false;
	}

	if (key_state[GLFW_KEY_U]) {
		Physics_Force force;
		force.force = (vec3){1000.0f, 0.0f, 0.0f};
		force.position = (vec3){0.0f, 0.0f, 0.0f};
		array_push(entities[1].forces, force);
		key_state[GLFW_KEY_U] = false;
	}

	//if (key_state[GLFW_KEY_9]) {
	//	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//	Entity e;
	//	graphics_entity_create_with_color(&e, m, (vec4){5.0f, 10.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//		(vec3){0.5f, 0.5f, 0.5f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 1000.0f);
	//	array_push(entities, e);
	//	key_state[GLFW_KEY_9] = false;
	//}

	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound_entity, quaternion_product(&rotation, &bound_entity->world_rotation));
		}
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

		vec3 e_pos = gm_vec4_to_vec3(bound_entity->world_position);
		e_pos = gm_vec3_add(e_pos, y_diff);
		e_pos = gm_vec3_add(e_pos, x_diff);
		graphics_entity_set_position(bound_entity, (vec4){e_pos.x, e_pos.y, e_pos.z, 1.0f});
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