#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "physics.h"

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}
#define PLANE_Y (0.0f)

static Perspective_Camera camera;
static Light* lights;
static Entity plane;
static Entity cube;
static Physics_Force* forces;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 2.0f, 5.0f, 1.0f};
	r32 camera_near_plane = -0.01f;
	r32 camera_far_plane = -1000.0f;
	r32 camera_fov = 45.0f;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

static Light* create_lights()
{
	Light light;
	Light* lights = array_create(Light, 1);

	vec4 light_position = (vec4) {0.0f, 0.0f, 15.0f, 1.0f};
	vec4 ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	vec4 specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, &light);

	light_position = (vec4) {0.0f, 0.0f, -15.0f, 1.0f};
	ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, &light);

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

	Mesh m = graphics_mesh_create_from_obj("./res/real_plane.obj", 0);
	graphics_entity_create_with_color(&plane, m, (vec4){0.0f, PLANE_Y, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1000.0f, 1000.0f, 1000.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 1000000000.0f);
	m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&cube, m, (vec4){0.0f, 2.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.5f}, 45.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);

	menu_register_dummy_callback(menu_dummy_callback);

	forces = array_create(Physics_Force, 1);
	Physics_Force gravity_force;
	gravity_force.force = (vec3){0.0f, -10.0f, 0.0f};
	gravity_force.position = (vec3) {0.0f, 0.0f, 0.0f};
	array_push(forces, &gravity_force);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	physics_simulate(&cube, &plane, PLANE_Y, delta_time, forces);
}

void core_render()
{
	graphics_entity_render_phong_shader(&camera, &plane, lights);
	graphics_entity_render_phong_shader(&camera, &cube, lights);
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
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
	static r64 x_pos_old, y_pos_old;
	// This constant is basically the mouse sensibility.
	// @TODO: Allow mouse sensibility to be configurable.
	static const r32 camera_mouse_speed = 0.1f;

	if (!reset)
	{
		r64 x_difference = x_pos - x_pos_old;
		r64 y_difference = y_pos - y_pos_old;

		camera_rotate_x(&camera, camera_mouse_speed * (r32)x_difference);
		camera_rotate_y(&camera, camera_mouse_speed * (r32)y_difference);
	}

	x_pos_old = x_pos;
	y_pos_old = y_pos;
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