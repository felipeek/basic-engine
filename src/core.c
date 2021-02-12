#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "animation.h"

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader phong_shader;
static Perspective_Camera camera;
static Light* lights;
static Entity e;
static Render_Primitives_Context rpc;

// Animation
static vec3* bezier_points;
static Bezier_Curve bezier_curve, first_derivate, second_derivate;
static boolean is_animating = false, ensure_constant_speed = false, use_frenet_frames = false;
static r32 t_animation = 0.0f;
static r32 animation_speed;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 0.0f, 10.0f, 1.0f};
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

	return lights;
}

static void menu_bezier_points_callback(u32 number_of_points, vec3* points)
{
	array_clear(bezier_points);
	for (u32 i = 0; i < number_of_points; ++i)
	{
		array_push(bezier_points, &points[i]);
	}
}

static void menu_animate_callback(r32 speed, boolean _ensure_constant_speed, boolean _use_frenet_frames,
	boolean manually_define_adaptive_subdivision_iterations, s32 adaptive_subdivision_iterations)
{
	char buffer[64];

	if (array_get_length(bezier_points) > 2)
	{
		if (is_animating)
		{
			animation_destroy_bezier_curve(&bezier_curve);
			animation_destroy_bezier_curve(&first_derivate);
			animation_destroy_bezier_curve(&second_derivate);
		}

		is_animating = true;
		t_animation = 0;
		animation_speed = speed;
		ensure_constant_speed = _ensure_constant_speed;
		use_frenet_frames = _use_frenet_frames;
		animation_create_bezier_curve(&bezier_curve, bezier_points,
			manually_define_adaptive_subdivision_iterations, adaptive_subdivision_iterations);
		first_derivate = animation_derivate_bezier_curve(&bezier_curve);
		second_derivate = animation_derivate_bezier_curve(&first_derivate);
	}
}

int core_init()
{
	// Create shader
	phong_shader = graphics_shader_create(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	Mesh m = graphics_mesh_create_from_obj_with_color("./res/cow.obj", 0, (vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){0.1f, 0.1f, 0.1f});

	menu_register_bezier_points_callback(menu_bezier_points_callback);
	menu_register_animate_callback(menu_animate_callback);

	bezier_points = array_create(vec3, 1);
	graphics_renderer_primitives_init(&rpc);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	if (is_animating)
	{
		t_animation += animation_speed * delta_time * (1.0f / bezier_curve.total_arc_length), 1.0f;
		if (t_animation > 1.0f) t_animation = 1.0f;

		r32 t = ensure_constant_speed ? animation_get_curve_parameter_from_desired_distance(&bezier_curve, t_animation) : t_animation;
		vec3 point_in_curve = animation_get_point_in_bezier_curve(&bezier_curve, t);
		graphics_entity_set_position(&e, (vec4){ point_in_curve.x, point_in_curve.y, point_in_curve.z, 1.0f });
		if (use_frenet_frames)
			graphics_entity_set_rotation(&e, animation_get_orientation_for_path(&bezier_curve, t, &first_derivate, &second_derivate));

		if (t_animation >= 1.0f)
		{
			is_animating = false;
			animation_destroy_bezier_curve(&bezier_curve);
		}
	}
}

void core_render()
{
	graphics_entity_render_phong_shader(phong_shader, &camera, &e, lights);

	for (s32 i = 0; i < array_get_length(bezier_points); ++i)
	{
		graphics_renderer_debug_points(&rpc, &bezier_points[i], 1, (vec4){0.0f, 1.0f, 0.0f, 1.0f});

		if (i > 0)
		{
			graphics_renderer_debug_vector(&rpc, bezier_points[i - 1], bezier_points[i], (vec4){0.0f, 0.8f, 0.8f, 1.0f});
		}
	}

	graphics_renderer_primitives_flush(&rpc, &camera);
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

}