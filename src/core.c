#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "util.h"

extern s32 window_width;
extern s32 window_height;
extern boolean use_lookat_camera;

static Camera camera;
static Light* lights;
static Entity e;

// Relevant for lookat camera
static boolean is_rotating_camera;
static boolean is_panning_camera;
static boolean alternative_panning_method;

static Camera create_lookat_camera()
{
	Camera camera;
	vec3 lookat_position = (vec3){0.0f, 0.0f, 0.0f};
	r32 lookat_distance = 4.0f;
	r32 near_plane = -0.01f;
	r32 far_plane = -1000.0f;
	r32 fov = 45.0f;
	camera_init_lookat(&camera, lookat_position, lookat_distance, near_plane, far_plane, fov, true);
	return camera;
}

static Camera create_free_camera()
{
	Camera camera;
	vec3 position = (vec3) { 0.0f, 0.0f, 5.0f };
	r32 near_plane = -0.01f;
	r32 far_plane = -1000.0f;
	r32 fov = 45.0f;
	camera_init_free(&camera, position, near_plane, far_plane, fov, true);
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
	camera = use_lookat_camera ? create_lookat_camera() : create_free_camera();
	// Create light
	lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec3){0.0f, 0.0f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy()
{
	array_free(lights);
}

void core_update(r32 delta_time)
{

}

void core_render()
{
	graphics_entity_render_phong_shader(&camera, &e, lights);
    graphics_renderer_debug_vector((vec3){0.0f, 0.0f, 0.0f}, (vec3){1.0f, 0.0f, 0.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
    graphics_renderer_primitives_flush(&camera);
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 300.0f;

	if (key_state[GLFW_KEY_LEFT_SHIFT])
		movement_speed = 0.5f;
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.1f;

	if (camera.type == CAMERA_FREE)
	{
		if (key_state[GLFW_KEY_W])
			camera_move_forward(&camera, movement_speed * delta_time);
		if (key_state[GLFW_KEY_S])
			camera_move_forward(&camera, -movement_speed * delta_time);
		if (key_state[GLFW_KEY_A])
			camera_move_right(&camera, -movement_speed * delta_time);
		if (key_state[GLFW_KEY_D])
			camera_move_right(&camera, movement_speed * delta_time);
	}
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
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
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
	static r64 x_pos_old, y_pos_old;
	// This constant is basically the mouse sensibility.
	// @TODO: Allow mouse sensibility to be configurable.
	static const r32 camera_mouse_speed = 0.1f;
	static const r32 panning_speed = 0.001f;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	if (!reset)
	{
		r32 mouse_x, mouse_y;
		util_normalize_window_coords_to_ndc(x_pos_old, y_pos_old, window_width, window_height, &mouse_x, &mouse_y);

		if ((camera.type == CAMERA_LOOKAT && is_rotating_camera) || camera.type == CAMERA_FREE)
			camera_rotate(&camera, camera_mouse_speed * (r32)x_difference, camera_mouse_speed * (r32)y_difference, mouse_x, mouse_y);

		if (is_panning_camera)
		{
			vec3 y_axis = camera_get_y_axis(&camera);
			vec3 inc = gm_vec3_scalar_product(-panning_speed * lookat_camera_get_lookat_distance(&camera.lookat_camera) * y_difference, y_axis);
			lookat_camera_set_position(&camera.lookat_camera, gm_vec3_add(lookat_camera_get_lookat_position(&camera.lookat_camera), inc));

			vec3 x_axis = camera_get_x_axis(&camera);
			inc = gm_vec3_scalar_product(-panning_speed * lookat_camera_get_lookat_distance(&camera.lookat_camera) * x_difference, x_axis);
			lookat_camera_set_position(&camera.lookat_camera, gm_vec3_add(lookat_camera_get_lookat_position(&camera.lookat_camera), inc));
		}
	}

	x_pos_old = x_pos;
	y_pos_old = y_pos;
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos)
{
	y_pos = window_height - y_pos;

	if (camera.type == CAMERA_LOOKAT)
	{
		if (button == GLFW_MOUSE_BUTTON_2) // right click
		{
			if (action == GLFW_PRESS)
				is_rotating_camera = true;

			if (action == GLFW_RELEASE)
				is_rotating_camera = false;
		}
		else if (button == GLFW_MOUSE_BUTTON_3) // middle button
		{
			if (!alternative_panning_method)
			{
				if (action == GLFW_PRESS)
					is_panning_camera = true;

				if (action == GLFW_RELEASE)
					is_panning_camera = false;
			}
		}
	}
}

void core_scroll_change_process(r64 x_offset, r64 y_offset)
{
	const r32 zoom_speed = 0.2;

	if (camera.type == CAMERA_LOOKAT)
	{
		r32 current_lookat_distance = lookat_camera_get_lookat_distance(&camera.lookat_camera);
		lookat_camera_set_lookat_distance(&camera.lookat_camera, current_lookat_distance - y_offset * zoom_speed);
	}
}

void core_window_resize_process(s32 width, s32 height)
{
	camera_force_matrix_recalculation(&camera);
}