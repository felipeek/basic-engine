#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "collision.h"

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Perspective_Camera camera;
static Light* lights;
static Entity e1, e2;

// Mouse binding to target positions
static boolean is_mouse_bound_to_entity;
static Entity* bound_entity;

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

int core_init()
{
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e1, m, (vec3){-1.0f, 0.0f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e2, m, (vec3){0.0f, -2.0f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){10.0f, 1.0f, 10.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy()
{
	array_free(lights);
}

static Collision_Point cp;
static boolean has_collision;

void core_update(r32 delta_time)
{
	GJK_Support_List gjk_support_list = {0};
	if (collision_gjk_collides(&gjk_support_list, &e1.bs, &e2.bs)) {
		e1.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		e2.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};

		cp = collision_epa(gjk_support_list.simplex, &e1.bs, &e2.bs);
		has_collision = true;
	} else {
		e1.diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		e2.diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		has_collision = false;
	}
}

void core_render()
{
	graphics_entity_render_phong_shader(&camera, &e1, lights);
	graphics_entity_render_phong_shader(&camera, &e2, lights);

	if (has_collision) {
		graphics_renderer_debug_points(&cp.collision_point, 1, (vec4){1.0f, 1.0f, 1.0f, 1.0f});
		graphics_renderer_debug_vector(cp.collision_point, gm_vec3_add(cp.collision_point, cp.normal), (vec4){1.0f, 1.0f, 1.0f, 1.0f});
		graphics_renderer_primitives_flush(&camera);
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
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e1, quaternion_product(&rotation, &e1.world_rotation));
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

	is_mouse_bound_to_entity = false;
	if (key_state[GLFW_KEY_1])
	{
		bound_entity = &e1;
		is_mouse_bound_to_entity = true;
	}
	if (key_state[GLFW_KEY_2])
	{
		bound_entity = &e2;
		is_mouse_bound_to_entity = true;
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

	if (is_mouse_bound_to_entity && bound_entity != NULL)
	{
		// MOVE TARGET POSITIONS!
		vec3 camera_y = camera_get_y_axis(&camera);
		vec3 camera_x = camera_get_x_axis(&camera);

		static const r32 target_point_move_speed = 0.001f;
		vec3 y_diff = gm_vec3_scalar_product(-target_point_move_speed * (r32)y_difference, camera_y);
		vec3 x_diff = gm_vec3_scalar_product(target_point_move_speed * (r32)x_difference, camera_x);

		vec3 new_position = bound_entity->world_position;
		new_position = gm_vec3_add(new_position, y_diff);
		new_position = gm_vec3_add(new_position, x_diff);
		graphics_entity_set_position(bound_entity, new_position);
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