#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include "core.h"
#include "graphics.h"
#include <math.h>
#include "obj.h"
#include <stdio.h>

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader phong_shader;
static Perspective_Camera camera;
static Light* lights;
static Entity e;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 0.0f, 1.0f, 1.0f};
	vec4 camera_up = (vec4) {0.0f, 1.0f, 0.0f, 1.0f};
	vec4 camera_view = (vec4) {0.0f, 0.0f, -1.0f, 0.0f};
	r32 camera_near_plane = -0.01f;
	r32 camera_far_plane = -1000.0f;
	r32 camera_fov = 45.0f;
	camera_init(&camera, camera_position, camera_up, camera_view, camera_near_plane, camera_far_plane, camera_fov);
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

	return lights;
}

int core_init()
{
	// Create shader
	phong_shader = graphics_shader_create(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj_with_color("./res/sphere.obj", 0, (vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, (vec3){0.0f, 0.0f, 0.0f}, (vec3){1.0f, 1.0f, 1.0f});

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{

}

void core_render()
{
	graphics_entity_render_phong_shader(phong_shader, &camera, &e, lights);
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 3.0f;

	if (key_state[GLFW_KEY_LEFT_SHIFT])
		movement_speed = 0.5f;
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.1f;

	if (key_state[GLFW_KEY_W])
		camera_set_position(&camera, gm_vec4_add(camera.position, gm_vec4_scalar_product(movement_speed * delta_time, gm_vec4_normalize(camera.view))));
	if (key_state[GLFW_KEY_S])
		camera_set_position(&camera, gm_vec4_add(camera.position, gm_vec4_scalar_product(-movement_speed * delta_time, gm_vec4_normalize(camera.view))));
	if (key_state[GLFW_KEY_A])
		camera_set_position(&camera, gm_vec4_add(camera.position, gm_vec4_scalar_product(-movement_speed * delta_time, gm_vec4_normalize(camera.x_axis))));
	if (key_state[GLFW_KEY_D])
		camera_set_position(&camera, gm_vec4_add(camera.position, gm_vec4_scalar_product(movement_speed * delta_time, gm_vec4_normalize(camera.x_axis))));
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec3 rotation = e.world_rotation;
			rotation.x -= rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
		}
		else
		{
			vec3 rotation = e.world_rotation;
			rotation.x += rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec3 rotation = e.world_rotation;
			rotation.y += rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
		}
		else
		{
			vec3 rotation = e.world_rotation;
			rotation.y -= rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec3 rotation = e.world_rotation;
			rotation.z += rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
		}
		else
		{
			vec3 rotation = e.world_rotation;
			rotation.z -= rotation_speed * delta_time;
			graphics_entity_set_rotation(&e, rotation);
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
	static const r32 camera_mouse_speed = 0.001f;

	if (!reset)
	{
		r64 x_difference = x_pos - x_pos_old;
		r64 y_difference = y_pos - y_pos_old;

		r32 pitch_angle = -camera_mouse_speed * (float)x_difference;
		r32 yaw_angle = camera_mouse_speed * (float)y_difference;

		camera_inc_pitch(&camera, pitch_angle);
		camera_inc_yaw(&camera, yaw_angle);
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

}