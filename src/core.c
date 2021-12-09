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
static Entity plane;
static Entity cube;
static Physics_Force* forces;
static Entity debug_plane, debug_plane_with_penetration;

static Bounding_Shape cube_bs, plane_bs;

static boolean is_mouse_bound_to_joint_target_position;
static Entity* bound_entity;
static Render_Primitives_Context r_ctx;

static vec3 find_ortho(vec3 v) {
	// tricky problem: https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector

	r32 v1 = v.z * v.z + v.y * v.y;
	r32 v2 = v.z * v.z + v.x * v.x;
	r32 v3 = v.y * v.y + v.x * v.x;

	if (v1 > v2 && v1 > v3) {
		return gm_vec3_normalize((vec3){0.0f, v.z, -v.y});
	} else if (v2 > v3) {
		return gm_vec3_normalize((vec3){-v.z, 0.0f, v.x});
	} else {
		return gm_vec3_normalize((vec3){-v.y, v.x, 0.0f});
	}
}

static Entity createPlane(vec3 point, vec3 normal, vec4 color) {
	Entity e;
	Mesh m;

	vec3 ortho = find_ortho(normal);
	vec3 c = gm_vec3_cross(normal, ortho);

	ortho = gm_vec3_scalar_product(1000.0f, ortho);
	c = gm_vec3_scalar_product(1000.0f, c);

	vec3 p1 = gm_vec3_add(c, ortho);
	vec3 p2 = gm_vec3_add(c, gm_vec3_scalar_product(-1.0f, ortho));
	vec3 p3 = gm_vec3_add(gm_vec3_scalar_product(-1.0f, c), ortho);
	vec3 p4 = gm_vec3_add(gm_vec3_scalar_product(-1.0f, c), gm_vec3_scalar_product(-1.0f, ortho));
	
	vec3 q1 = gm_vec3_add(gm_vec3_subtract(p1,
			gm_vec3_scalar_product(gm_vec3_dot(normal, p1), normal)), point);
	vec3 q2 = gm_vec3_add(gm_vec3_subtract(p2,
			gm_vec3_scalar_product(gm_vec3_dot(normal, p2), normal)), point);
	vec3 q3 = gm_vec3_add(gm_vec3_subtract(p3,
			gm_vec3_scalar_product(gm_vec3_dot(normal, p3), normal)), point);
	vec3 q4 = gm_vec3_add(gm_vec3_subtract(p4,
			gm_vec3_scalar_product(gm_vec3_dot(normal, p4), normal)), point);
	Vertex* vertices = array_new(Vertex);
	Vertex v1, v2, v3, v4;
	v1.normal = (vec4){normal.x, normal.y, normal.z, 0.0f};
	v1.position = (vec4){q1.x, q1.y, q1.z, 1.0f};
	v1.texture_coordinates = (vec2){0.0f, 0.0f};
	v2.normal = (vec4){normal.x, normal.y, normal.z, 0.0f};
	v2.position = (vec4){q2.x, q2.y, q2.z, 1.0f};
	v2.texture_coordinates = (vec2){0.0f, 0.0f};
	v3.normal = (vec4){normal.x, normal.y, normal.z, 0.0f};
	v3.position = (vec4){q3.x, q3.y, q3.z, 1.0f};
	v3.texture_coordinates = (vec2){0.0f, 0.0f};
	v4.normal = (vec4){normal.x, normal.y, normal.z, 0.0f};
	v4.position = (vec4){q4.x, q4.y, q4.z, 1.0f};
	v4.texture_coordinates = (vec2){0.0f, 0.0f};
	array_push(vertices, v1);
	array_push(vertices, v2);
	array_push(vertices, v3);
	array_push(vertices, v4);

	u32* indices = array_new(u32);
	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	array_push(indices, 1);
	array_push(indices, 3);
	array_push(indices, 2);

	m = graphics_mesh_create(vertices, array_length(vertices), indices, array_length(indices), 0);
	m.vertices = vertices;
	m.indices = indices;
	m.indexes_size = array_length(indices);
	graphics_entity_create_with_color(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, color, 1.0f);

	return e;
}

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

	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){-1.272, -0.059, -0.221}, (Quaternion){-0.381, -0.008, -0.434, 0.816},
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);

	// bug 1
	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){0.0f, 0.7f, 0.5f, 1.0f}, quaternion_new((vec3){3.0f, 1.0f, 0.5f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);

	// bug2
	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/sphere.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){-0.201f, 0.904f, 1.225f, 1.0f}, (Quaternion){0.095f, -0.386f, 0.080f, 0.914f},
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);

	// bug (sphere) -> incorrect collision
	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/sphere.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){-0.846f, 0.998, 0.677f, 1.0f}, (Quaternion){0.095f, -0.386f, 0.080f, 0.914f},
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){0.095f, -0.386f, 0.080f, 0.914f}, 10.0f);

	// bug (ico) -> expected collision not being reported
	//Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	//graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	//m = graphics_mesh_create_from_obj("./res/ico_low.obj", 0);
	//graphics_entity_create_with_color(&cube, m, (vec4){0.461, 0.895, 1.100, 1.0f}, (Quaternion){0.095, -0.386, 0.080, 0.914},
	//	(vec3){1.0f, 1.0f, 1.0f}, (vec4){0.095f, -0.386f, 0.080f, 0.914f}, 10.0f);

	// bug (ico) -> incorrect collision
	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&plane, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.5f, 0.0f, 1.0f}, 1000000000.0f);
	m = graphics_mesh_create_from_obj("./res/ico_low.obj", 0);
	graphics_entity_create_with_color(&cube, m, (vec4){-0.167, 1.248, -0.273, 1.0f}, (Quaternion){-0.326, -0.314, 0.239, 0.859},
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){0.095f, -0.386f, 0.080f, 0.914f}, 10.0f);

	//menu_register_dummy_callback(menu_dummy_callback);

	forces = array_new(Physics_Force);
	Physics_Force gravity_force;
	gravity_force.force = (vec3){0.0f, -100.0f, 0.0f};
	gravity_force.position = (vec3) {0.0f, 0.0f, 0.0f};
	array_push(forces, gravity_force);

	graphics_renderer_primitives_init(&r_ctx);

	cube_bs.vertex_count = array_length(cube.mesh.vertices);
	cube_bs.vertices = malloc(sizeof(vec3) * cube_bs.vertex_count);
	plane_bs.vertex_count = array_length(plane.mesh.vertices);
	plane_bs.vertices = malloc(sizeof(vec3) * plane_bs.vertex_count);
	
	bound_entity = &cube;

	return 0;
}

void core_destroy()
{
	array_free(lights);
}

static boolean collision;
static Persistent_Manifold pm;
extern vec3 nnn;
extern vec3 ppp;
extern r32 pppenetration;

void core_update(r32 delta_time)
{
	for (u32 i = 0; i < array_length(cube.mesh.vertices); ++i) {
		cube_bs.vertices[i] = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&cube.model_matrix, cube.mesh.vertices[i].position));
	}
	for (u32 i = 0; i < array_length(plane.mesh.vertices); ++i) {
		plane_bs.vertices[i] = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&plane.model_matrix, plane.mesh.vertices[i].position));
	}

	GJK_Support_List simplex = { 0 };
	if (collision_gjk_collides(&simplex, &cube_bs, &plane_bs)) {
		cube.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		plane.diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		pm = collision_epa(simplex.simplex, &cube_bs, &plane_bs, &cube.mesh, &plane.mesh);
		collision = true;
	} else {
		cube.diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		plane.diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		collision = false;
	}

	printf("wc: %.3f, %.3f, %.3f\n", cube.world_position.x, cube.world_position.y, cube.world_position.z);
	printf("q: %.3f, %.3f, %.3f, %.3f\n", cube.world_rotation.x, cube.world_rotation.y, cube.world_rotation.z, cube.world_rotation.w);

	//physics_simulate(&cube, &plane, PLANE_Y, delta_time, forces);
	//array_clear(forces);

	vec3 point_is = gm_vec3_subtract(ppp, gm_vec3_scalar_product(pppenetration, nnn));
	debug_plane = createPlane(ppp, nnn, (vec4){1.0f, 0.0f, 1.0f, 0.0f});
	debug_plane_with_penetration = createPlane(point_is, nnn, (vec4){1.0f, 1.0f, 1.0f, 1.0f});
}

boolean render_debug_plane;
boolean render_debug_plane_with_penetration;
void core_render()
{
	if (render_debug_plane)
		graphics_entity_render_phong_shader(&camera, &debug_plane, lights);
	if (render_debug_plane_with_penetration)
		graphics_entity_render_phong_shader(&camera, &debug_plane_with_penetration, lights);
	graphics_entity_render_phong_shader(&camera, &plane, lights);
	graphics_entity_render_phong_shader(&camera, &cube, lights);

	if (collision) {
		for (int i = 0; i < array_length(pm.collision_points1); ++i) {
			vec3 cp = pm.collision_points1[i];
			graphics_renderer_debug_points(&r_ctx, &cp, 1, (vec4){1.0f, 0.0f, 0.5f, 1.0f});
			graphics_renderer_debug_vector(&r_ctx, cp, gm_vec3_add(cp, gm_vec3_scalar_product(-1.0f, pm.normal)),
				(vec4){1.0f, 0.5f, 0.5f, 1.0f});
			graphics_renderer_primitives_flush(&r_ctx, &camera);
		}

		for (int i = 0; i < array_length(pm.collision_points2); ++i) {
			vec3 cp = pm.collision_points2[i];
			graphics_renderer_debug_points(&r_ctx, &cp, 1, (vec4){0.5f, 0.0f, 0.5f, 1.0f});
			graphics_renderer_debug_vector(&r_ctx, cp, gm_vec3_add(cp, gm_vec3_scalar_product(-1.0f, pm.normal)),
				(vec4){0.5f, 1.0f, 0.5f, 1.0f});
			graphics_renderer_primitives_flush(&r_ctx, &camera);
		}

		vec3 aaa = gm_vec3_scalar_product(0.582058f, (vec3){-0.540517f, 0.818457f, 0.194857f});
		graphics_renderer_debug_points(&r_ctx, &aaa, 1, (vec4){0.0f, 1.0f, 1.0f, 1.0f});
	}
}

extern int tmp;
void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 30.0f;

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
	if (key_state[GLFW_KEY_M])
	{
		Physics_Force gravity_force;
		gravity_force.force = (vec3){0.0f, -100.0f, 0.0f};
		gravity_force.position = (vec3) {0.0f, 0.0f, 0.0f};
		array_push(forces, gravity_force);
		key_state[GLFW_KEY_M] = false;
	}

	is_mouse_bound_to_joint_target_position = false;
	if (key_state[GLFW_KEY_1])
	{
		bound_entity = &cube;
		is_mouse_bound_to_joint_target_position = true;
	}
	if (key_state[GLFW_KEY_2])
	{
		bound_entity = &plane;
		is_mouse_bound_to_joint_target_position = true;
	}

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

	if (key_state[GLFW_KEY_SPACE]) {
		tmp = !tmp;
		key_state[GLFW_KEY_SPACE] = false;
	}

	if (key_state[GLFW_KEY_O]) {
		render_debug_plane = !render_debug_plane;
		key_state[GLFW_KEY_O] = false;
	}
	if (key_state[GLFW_KEY_P]) {
		render_debug_plane_with_penetration = !render_debug_plane_with_penetration;
		key_state[GLFW_KEY_P] = false;
	}
	if (key_state[GLFW_KEY_U]) {
		vec3 delta = gm_vec3_scalar_product(-pppenetration, nnn);
		vec3 pos = gm_vec3_add(gm_vec4_to_vec3(cube.world_position), delta);
		graphics_entity_set_position(&cube, (vec4){pos.x, pos.y, pos.z, 1.0f});
		key_state[GLFW_KEY_U] = false;
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