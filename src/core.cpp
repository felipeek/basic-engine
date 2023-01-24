#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "ui.h"
#include "util.h"
#include "camera/lookat.h"
#include "camera/free.h"

#define USE_LOOKAT_CAMERA true

extern dvec2 window_size;
extern dvec2 framebuffer_size;

static Camera create_lookat_camera()
{
	Camera camera;
	const vec3 lookat_position = (vec3){0.0f, 0.0f, 0.0f};
	const r32 lookat_distance = 4.0f;
	const r32 near_plane = -0.01f;
	const r32 far_plane = -1000.0f;
	const r32 fov = 45.0f;
	const r32 movement_speed = 0.2f;
	const r32 rotation_speed = 0.1f;
	const r32 panning_speed = 0.001f;

	lookat_camera_init(&camera, lookat_position, lookat_distance, near_plane, far_plane, fov, true, movement_speed, rotation_speed, 
		panning_speed);
	return camera;
}

static Camera create_free_camera()
{
	Camera camera;
	const vec3 position = (vec3) { 0.0f, 0.0f, 5.0f };
	const r32 near_plane = -0.01f;
	const r32 far_plane = -1000.0f;
	const r32 fov = 45.0f;
	const r32 movement_speed = 3.0f;
	const r32 rotation_speed = 0.1f;

	free_camera_init(&camera, position, near_plane, far_plane, fov, true, movement_speed, rotation_speed);
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

Core_Ctx core_init(GLFWwindow* window)
{
	Core_Ctx ctx;
	memset(&ctx, 0, sizeof(Core_Ctx));

	util_viewport_for_complete_window(); // make sure viewport is correct, proved to be necessary on OS X
	ctx.ui_ctx = ui_init();

	// Create camera
	ctx.camera = USE_LOOKAT_CAMERA ? create_lookat_camera() : create_free_camera();
	// Create light
	ctx.lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&ctx.e, m, (vec3){0.0f, 0.0f, 0.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});

	ctx.window = window;
	ctx.alternative_panning_method = false;
	return ctx;
}

void core_destroy(Core_Ctx* ctx)
{
	array_free(ctx->lights);
	ui_destroy(&ctx->ui_ctx);
}

void core_update(Core_Ctx* ctx, r32 delta_time)
{
}

void core_render(Core_Ctx* ctx)
{
	graphics_entity_render_phong_shader(&ctx->camera, &ctx->e, ctx->lights);
	graphics_renderer_debug_vector((vec3){0.0f, 0.0f, 0.0f}, (vec3){1.0f, 0.0f, 0.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	graphics_renderer_primitives_flush(&ctx->camera);
	ui_render(&ctx->ui_ctx, ctx->is_ui_active);
}

void core_input_process(Core_Ctx* ctx, r32 delta_time)
{
	r32 camera_movement_multiplier = 1.0f;
	if (ctx->key_state[GLFW_KEY_LEFT_SHIFT])
		camera_movement_multiplier = 0.5f;
	if (ctx->key_state[GLFW_KEY_RIGHT_SHIFT])
		camera_movement_multiplier = 0.1f;

	if (ctx->camera.type == CAMERA_FREE)
	{
		if (ctx->key_state[GLFW_KEY_W])
			camera_move_forward(&ctx->camera, camera_movement_multiplier * delta_time);
		if (ctx->key_state[GLFW_KEY_S])
			camera_move_forward(&ctx->camera, -camera_movement_multiplier* delta_time);
		if (ctx->key_state[GLFW_KEY_A])
			camera_move_right(&ctx->camera, -camera_movement_multiplier * delta_time);
		if (ctx->key_state[GLFW_KEY_D])
			camera_move_right(&ctx->camera, camera_movement_multiplier * delta_time);
	}

	const r32 rotation_speed = 300.0f;
	if (ctx->key_state[GLFW_KEY_X])
	{
		if (ctx->key_state[GLFW_KEY_LEFT_SHIFT] || ctx->key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
	}
	if (ctx->key_state[GLFW_KEY_Y])
	{
		if (ctx->key_state[GLFW_KEY_LEFT_SHIFT] || ctx->key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
	}
	if (ctx->key_state[GLFW_KEY_Z])
	{
		if (ctx->key_state[GLFW_KEY_LEFT_SHIFT] || ctx->key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&ctx->e, quaternion_product(&rotation, &ctx->e.world_rotation));
		}
	}
	if (ctx->key_state[GLFW_KEY_L])
	{
		static bool wireframe = false;

		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		wireframe = !wireframe;
		ctx->key_state[GLFW_KEY_L] = false;
	}
	if (ctx->key_state[GLFW_KEY_KP_DECIMAL])
	{
		if (ctx->camera.type == CAMERA_LOOKAT)
		{
			vec3 lookat_position = (vec3){0.0f, 0.0f, 0.0f};
			lookat_camera_set_lookat_position(&ctx->camera, lookat_position);
			ctx->key_state[GLFW_KEY_KP_DECIMAL] = false;
		}
	}

	if (ctx->alternative_panning_method)
	{
		if (!ctx->key_state[GLFW_KEY_LEFT_SHIFT] && !ctx->key_state[GLFW_KEY_RIGHT_SHIFT])
			ctx->is_panning_camera = false;
		else
			ctx->is_panning_camera = true;
	}
	
	if (ctx->key_state[GLFW_KEY_ESCAPE])
	{
		ctx->is_ui_active = !ctx->is_ui_active;
		ctx->key_state[GLFW_KEY_ESCAPE] = false;
	}
}

void core_key_press_process(Core_Ctx* ctx, s32 key, s32 scanCode, s32 action, s32 mods)
{
	if (action == GLFW_PRESS)
		ctx->key_state[key] = true;

	if (action == GLFW_RELEASE)
		ctx->key_state[key] = false;
}

void core_mouse_change_process(Core_Ctx* ctx, r64 x_pos, r64 y_pos)
{
	static r64 x_pos_old, y_pos_old;

	r64 x_difference = x_pos - x_pos_old;
	r64 y_difference = y_pos - y_pos_old;

	r32 mouse_x, mouse_y;
	util_normalize_window_coords_to_ndc(x_pos_old, y_pos_old, window_size.x, window_size.y, &mouse_x, &mouse_y);

	if (ctx->is_rotating_camera)
		camera_rotate(&ctx->camera, (r32)x_difference, (r32)y_difference, mouse_x, mouse_y);

	if (ctx->is_panning_camera)
		lookat_camera_pan(&ctx->camera, y_difference, x_difference);

	x_pos_old = x_pos;
	y_pos_old = y_pos;
}

void core_mouse_click_process(Core_Ctx* ctx, s32 button, s32 action, r64 x_pos, r64 y_pos)
{
	y_pos = window_size.y - y_pos;

	if (button == GLFW_MOUSE_BUTTON_2) // right click
	{
		if (action == GLFW_PRESS)
			ctx->is_rotating_camera = true;

		if (action == GLFW_RELEASE)
			ctx->is_rotating_camera = false;
	}

	if (ctx->camera.type == CAMERA_LOOKAT)
	{
		if (button == GLFW_MOUSE_BUTTON_3) // middle button
		{
			if (!ctx->alternative_panning_method)
			{
				if (action == GLFW_PRESS)
					ctx->is_panning_camera = true;

				if (action == GLFW_RELEASE)
					ctx->is_panning_camera = false;
			}
		}
	}
}

void core_scroll_change_process(Core_Ctx* ctx, r64 x_offset, r64 y_offset)
{
	if (ctx->camera.type == CAMERA_LOOKAT)
		lookat_camera_approximate(&ctx->camera, y_offset);
}

void core_window_resize_process(Core_Ctx* ctx, s32 width, s32 height)
{
	util_viewport_for_complete_window();
	camera_force_matrix_recalculation(&ctx->camera);
}