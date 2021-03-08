#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "physics.h"
#include "collision.h"
#include <float.h>

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Vertex triangle_vertices[3];
static u32 triangle_indices[3];

static Perspective_Camera camera;
static Light* lights;
static Physics_Force* forces;
static Render_Primitives_Context pctx;
// Mouse binding to target positions
static boolean is_mouse_bound_to_joint_target_position;
static Entity* bound;
static Entity face, fake_face;
static Entity e_point, fake_e_point;

// frame f-1
static vec4 old_frame_point_position;
static vec4 old_frame_entity_position;
static Quaternion old_frame_entity_rotation;

// frame f
static vec4 new_frame_point_position;
static vec4 new_frame_entity_position;
static Quaternion new_frame_entity_rotation;

typedef struct {
	vec4 point_position;
	vec4 face_position;
	Quaternion face_rotation;
	vec4 result_point_position;
	s32 hit;
} Sample;
static Sample* sampled_points;
vec4 colors[256];

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

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

static void init_triangle()
{
	triangle_vertices[0].position = (vec4){0.0f, 0.0f, 0.0f, 1.0f};
	triangle_vertices[1].position = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
	triangle_vertices[2].position = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
	triangle_vertices[0].normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	triangle_vertices[1].normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	triangle_vertices[2].normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	triangle_vertices[0].texture_coordinates = (vec2){0.0f, 0.0f};
	triangle_vertices[1].texture_coordinates = (vec2){1.0f, 0.0f};
	triangle_vertices[2].texture_coordinates = (vec2){0.0f, 1.0f};
	triangle_indices[0] = 0;
	triangle_indices[1] = 1;
	triangle_indices[2] = 2;
}

int core_init()
{
	init_triangle();
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	forces = array_create(Physics_Force, 1);
	sampled_points = array_create(Sample, 1);

	Entity e;
	Mesh m = graphics_mesh_create(
		triangle_vertices,
		sizeof(triangle_vertices) / sizeof(Vertex),
		triangle_indices,
		sizeof(triangle_indices) / sizeof(u32),
		0);
	graphics_entity_create_with_color(&face, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	graphics_entity_create_with_color(&fake_face, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){0.0f, 1.0f, 0.0f, 1.0f});
	m = graphics_mesh_create_from_obj("res/sphere.obj", 0);
	graphics_entity_create_with_color(&e_point, m, (vec4){1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){0.01f, 0.01f, 0.01f}, (vec4){0.0f, 0.0f, 1.0f, 1.0f});
	graphics_entity_create_with_color(&fake_e_point, m, (vec4){1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){0.01f, 0.01f, 0.01f}, (vec4){0.0f, 1.0f, 0.0f, 1.0f});

	menu_register_dummy_callback(menu_dummy_callback);
	graphics_renderer_primitives_init(&pctx);

	old_frame_point_position = face.world_position;
	old_frame_entity_position = face.world_position;
	old_frame_entity_rotation = face.world_rotation;

	// frame f
	new_frame_point_position = e_point.world_position;
	new_frame_entity_position = face.world_position;
	new_frame_entity_rotation = face.world_rotation;

	for (u32 i = 0; i < 256; ++i) {
		colors[i] = (vec4){rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, rand() / (r32)RAND_MAX, 1.0f};
	}
	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	//physics_update(&e, forces, delta_time);
	array_clear(sampled_points);

	const u32 NUM_SAMPLES = 5;

	vec4 last_iteration_interpolated_point_position = old_frame_point_position;
	vec4 last_iteration_interpolated_entity_position = old_frame_entity_position;
	Quaternion last_iteration_interpolated_entity_rotation = old_frame_entity_rotation;

	for (u32 i = 0; i < NUM_SAMPLES; ++i) {
		vec4 this_iteration_interpolated_point_position = gm_vec4_add(
			gm_vec4_scalar_product((r32)(i + 1) / NUM_SAMPLES, new_frame_point_position),
			gm_vec4_scalar_product((r32)(NUM_SAMPLES - 1 - i) / NUM_SAMPLES, old_frame_point_position)
		);

		vec4 this_iteration_interpolated_entity_position = gm_vec4_add(
			gm_vec4_scalar_product((r32)(i + 1) / NUM_SAMPLES, new_frame_entity_position),
			gm_vec4_scalar_product((r32)(NUM_SAMPLES - 1 - i) / NUM_SAMPLES, old_frame_entity_position)
		);

		Quaternion this_iteration_interpolated_entity_rotation = quaternion_slerp(
			&old_frame_entity_rotation,
			&new_frame_entity_rotation,
			(r32)(i + 1) / NUM_SAMPLES
		);

		mat4 frame_model_matrix = graphics_model_matrix(last_iteration_interpolated_entity_position,
			last_iteration_interpolated_entity_rotation, face.world_scale);
		mat4 inverse;
		assert(gm_mat4_inverse(&frame_model_matrix, &inverse));

		// we use the point position in this iteration, which means that we are effectively moving the point before rotating
		// (one way of doing it)
		vec4 point_in_local_coords = gm_mat4_multiply_vec4(&inverse, this_iteration_interpolated_point_position);
		Quaternion inverse_new_frame_rot = quaternion_inverse(&this_iteration_interpolated_entity_rotation);
		Quaternion point_rotation = quaternion_product(&last_iteration_interpolated_entity_rotation, &inverse_new_frame_rot);
		point_rotation = quaternion_product(&point_rotation, &last_iteration_interpolated_entity_rotation);
		mat4 rot_matrix = quaternion_get_matrix(&point_rotation);

		vec4 result_point_position = gm_mat4_multiply_vec4(&rot_matrix, point_in_local_coords);
		result_point_position = gm_vec4_add(result_point_position, last_iteration_interpolated_entity_position);
		result_point_position = gm_vec4_add(result_point_position, gm_vec4_scalar_product(-1.0f, this_iteration_interpolated_entity_position));
		result_point_position = gm_vec4_add(result_point_position, last_iteration_interpolated_entity_position);
		result_point_position.w = 1.0f;

		vec4 p1 = face.mesh.vertices[0].position;
		vec4 p2 = face.mesh.vertices[1].position;
		vec4 p3 = face.mesh.vertices[2].position;
		vec4 t_p1 = gm_mat4_multiply_vec4(&frame_model_matrix, p1);
		vec4 t_p2 = gm_mat4_multiply_vec4(&frame_model_matrix, p2);
		vec4 t_p3 = gm_mat4_multiply_vec4(&frame_model_matrix, p3);

		Sample s;
		s.point_position = last_iteration_interpolated_point_position;// the ray begins at the last_iteration point
		s.face_position = last_iteration_interpolated_entity_position;  // the ray needs to pass through the last_iteration face
		s.face_rotation = last_iteration_interpolated_entity_rotation;  // the ray needs to pass through the last_iteration face
		s.result_point_position = result_point_position;
		s.hit = 0;
		vec3 intersection;
		r32 d;
		s.hit = collision_check_edge_collides_triangle(
			gm_vec4_to_vec3(last_iteration_interpolated_point_position),
			gm_vec4_to_vec3(result_point_position),
			gm_vec4_to_vec3(t_p1),
			gm_vec4_to_vec3(t_p2),
			gm_vec4_to_vec3(t_p3),
			&d,
			&intersection
		);
		array_push(sampled_points, &s);

		last_iteration_interpolated_entity_position = this_iteration_interpolated_entity_position;
		last_iteration_interpolated_entity_rotation = this_iteration_interpolated_entity_rotation;
		last_iteration_interpolated_point_position = this_iteration_interpolated_point_position;
	}
}

void core_render()
{
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_CCW);
	graphics_entity_render_phong_shader(&camera, &face, lights);
	graphics_entity_render_phong_shader(&camera, &e_point, lights);
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
	for (u32 i = 0; i < array_get_length(sampled_points); ++i) {
		Sample* s = &sampled_points[i];
		graphics_entity_set_position(&fake_face, s->face_position);
		graphics_entity_set_rotation(&fake_face, s->face_rotation);
		graphics_entity_set_position(&fake_e_point, s->point_position);
		fake_face.diffuse_info.diffuse_color = colors[i];
		if (s->hit)
			fake_e_point.diffuse_info.diffuse_color = (vec4){1.0f, 1.0f, 1.0f, 1.0f};
		else
			fake_e_point.diffuse_info.diffuse_color = colors[i];
		graphics_entity_render_phong_shader(&camera, &fake_face, lights);
		graphics_entity_render_phong_shader(&camera, &fake_e_point, lights);
		vec3 r = gm_vec4_to_vec3(s->result_point_position);
		graphics_renderer_debug_points(&pctx, &r, 1, colors[i]);
		if (s->hit)
			graphics_renderer_debug_vector(&pctx, gm_vec4_to_vec3(s->point_position), r, (vec4){1.0f, 1.0f, 1.0f, 1.0f});
		else
			graphics_renderer_debug_vector(&pctx, gm_vec4_to_vec3(s->point_position), r, colors[i]);
	}
	graphics_renderer_primitives_flush(&pctx, &camera);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
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
	if (key_state[GLFW_KEY_X] && bound)
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y] && bound)
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z] && bound)
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(bound, quaternion_product(&rotation, &bound->world_rotation));
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
	if (key_state[GLFW_KEY_M])
	{
		new_frame_point_position = e_point.world_position;
		new_frame_entity_position = face.world_position;
		new_frame_entity_rotation = face.world_rotation;
		key_state[GLFW_KEY_M] = false;
	}
	if (key_state[GLFW_KEY_N])
	{
		old_frame_point_position = e_point.world_position;
		old_frame_entity_position = face.world_position;
		old_frame_entity_rotation = face.world_rotation;
		key_state[GLFW_KEY_N] = false;
	}

	is_mouse_bound_to_joint_target_position = false;
	if (key_state[GLFW_KEY_1])
	{
		bound = &face;
		is_mouse_bound_to_joint_target_position = true;
	}
	if (key_state[GLFW_KEY_2])
	{
		bound = &e_point;
		is_mouse_bound_to_joint_target_position = true;
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

	if (is_mouse_bound_to_joint_target_position && bound != NULL)
	{
		// MOVE TARGET POSITIONS!
		vec3 camera_y = camera_get_y_axis(&camera);
		vec3 camera_x = camera_get_x_axis(&camera);

		static const r32 target_point_move_speed = 0.001f;
		vec3 y_diff = gm_vec3_scalar_product(-target_point_move_speed * (r32)y_difference, camera_y);
		vec3 x_diff = gm_vec3_scalar_product(target_point_move_speed * (r32)x_difference, camera_x);

		vec4 position = bound->world_position;
		position = gm_vec4_add(position, (vec4){y_diff.x, y_diff.y, y_diff.z, 0.0f});
		position = gm_vec4_add(position, (vec4){x_diff.x, x_diff.y, x_diff.z, 0.0f});
		graphics_entity_set_position(bound, position);
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