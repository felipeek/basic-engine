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

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Perspective_Camera camera;
static Light* lights;
static Entity* entities;
static vec3* collision_points;
static Physics_Force* forces;
static Render_Primitives_Context pctx;
// Mouse binding to target positions
static boolean is_mouse_bound_to_joint_target_position;
static Entity* bound;

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

int core_init()
{
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	forces = array_create(Physics_Force, 1);
	entities = array_create(Entity, 1);
	collision_points = array_create(vec3, 1);

	Entity e;
	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec4){-1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	array_push(entities, &e);
	graphics_entity_create_with_color(&e, m, (vec4){1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		//(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
		(vec3){0.3f, 0.3f, 0.3f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	array_push(entities, &e);

	menu_register_dummy_callback(menu_dummy_callback);
	graphics_renderer_primitives_init(&pctx);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

static void check_entity_entity_collision(Entity* e1, Entity* e2)
{
	s32 found_vertex_collision = 0;

	// TEST e1 against e2
	for (u32 v = 0; v < array_get_length(e1->mesh.vertices); ++v) { // for each vertex of e1
		Vertex* point = &e1->mesh.vertices[v];
		vec4 transformed_point = gm_mat4_multiply_vec4(&e1->model_matrix, point->position);
		//vec4 transformed_point = gm_mat4_multiply_vec4(&e1->model_matrix, e1->world_position);

		s32 is_vertex_outside_mesh = 0;
		// for each vertex we need to check if it is inside the other mesh.
		for (u32 t = 0; t < array_get_length(e2->mesh.indices); t += 3) {
			Vertex* v1 = &e2->mesh.vertices[e2->mesh.indices[t + 0]];
			Vertex* v2 = &e2->mesh.vertices[e2->mesh.indices[t + 1]];
			Vertex* v3 = &e2->mesh.vertices[e2->mesh.indices[t + 2]];
			vec4 transformed_v1 = gm_mat4_multiply_vec4(&e2->model_matrix, v1->position);
			vec4 transformed_v2 = gm_mat4_multiply_vec4(&e2->model_matrix, v2->position);
			vec4 transformed_v3 = gm_mat4_multiply_vec4(&e2->model_matrix, v3->position);

			// 1 -> out
			// 0 -> in
			is_vertex_outside_mesh = collision_check_point_side_of_triangle(
				gm_vec4_to_vec3(transformed_point),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3)
			);

			// if the vertex is outside the triangle, then we know that this vertex is NOT inside the mesh
			if (is_vertex_outside_mesh)
				break;
		}

		// if !is_vertex_outside_mesh, then the vertex WAS inside the mesh, so there IS collision!
		if (!is_vertex_outside_mesh) {
			vec3 v = gm_vec4_to_vec3(transformed_point);
			array_push(collision_points, &v);
			found_vertex_collision = 1;
			// keep going to render all collision points
			//break;
		}
	}

	// TEST e2 against e1
	for (u32 v = 0; v < array_get_length(e2->mesh.vertices); ++v) { // for each vertex of e2
		Vertex* point = &e2->mesh.vertices[v];
		vec4 transformed_point = gm_mat4_multiply_vec4(&e2->model_matrix, point->position);
		//vec4 transformed_point = gm_mat4_multiply_vec4(&e2->model_matrix, e2->world_position);

		s32 is_vertex_outside_mesh = 0;
		// for each vertex we need to check if it is inside the other mesh.
		for (u32 t = 0; t < array_get_length(e1->mesh.indices); t += 3) {
			Vertex* v1 = &e1->mesh.vertices[e1->mesh.indices[t + 0]];
			Vertex* v2 = &e1->mesh.vertices[e1->mesh.indices[t + 1]];
			Vertex* v3 = &e1->mesh.vertices[e1->mesh.indices[t + 2]];
			vec4 transformed_v1 = gm_mat4_multiply_vec4(&e1->model_matrix, v1->position);
			vec4 transformed_v2 = gm_mat4_multiply_vec4(&e1->model_matrix, v2->position);
			vec4 transformed_v3 = gm_mat4_multiply_vec4(&e1->model_matrix, v3->position);

			// 1 -> out
			// 0 -> in
			is_vertex_outside_mesh = collision_check_point_side_of_triangle(
				gm_vec4_to_vec3(transformed_point),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3)
			);

			// if the vertex is outside the triangle, then we know that this vertex is NOT inside the mesh
			if (is_vertex_outside_mesh)
				break;
		}

		// if !is_vertex_outside_mesh, then the vertex WAS inside the mesh, so there IS collision!
		if (!is_vertex_outside_mesh) {
			vec3 v = gm_vec4_to_vec3(transformed_point);
			array_push(collision_points, &v);
			found_vertex_collision = 1;
			// keep going to render all collision points
			//break;
		}
	}

	// At this point, we analysed all vertices.
	// We now analyze the edges
	// However if we already found some vertices, we skip this part

	if (found_vertex_collision) {
		e1->diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		e2->diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		return;
	}

	/*
		@TODO: improve edge-edge collision point detection
		we are generating two distinct points in the most common scenario (when there are no vertices colliding)
		i think we should generate a single one, just like with vertices?
	*/

	vec3* edge1_collisions = array_create(vec3, 1);
	vec3* edge2_collisions = array_create(vec3, 1);
	vec3* edge3_collisions = array_create(vec3, 1);

	// TEST e1 against e2
	s32 found_edge_collision = 0;
	for (u32 v = 0; v < array_get_length(e1->mesh.indices); v += 3) { // for each edge of e1
		Vertex* f1 = &e1->mesh.vertices[e1->mesh.indices[v]];
		Vertex* f2 = &e1->mesh.vertices[e1->mesh.indices[v + 1]];
		Vertex* f3 = &e1->mesh.vertices[e1->mesh.indices[v + 2]];
		vec4 transformed_f1 = gm_mat4_multiply_vec4(&e1->model_matrix, f1->position);
		vec4 transformed_f2 = gm_mat4_multiply_vec4(&e1->model_matrix, f2->position);
		vec4 transformed_f3 = gm_mat4_multiply_vec4(&e1->model_matrix, f3->position);
		array_clear(edge1_collisions);
		array_clear(edge2_collisions);
		array_clear(edge3_collisions);

		// for each vertex we need to check if it is inside the other mesh.
		for (u32 t = 0; t < array_get_length(e2->mesh.indices); t += 3) {
			Vertex* v1 = &e2->mesh.vertices[e2->mesh.indices[t + 0]];
			Vertex* v2 = &e2->mesh.vertices[e2->mesh.indices[t + 1]];
			Vertex* v3 = &e2->mesh.vertices[e2->mesh.indices[t + 2]];
			vec4 transformed_v1 = gm_mat4_multiply_vec4(&e2->model_matrix, v1->position);
			vec4 transformed_v2 = gm_mat4_multiply_vec4(&e2->model_matrix, v2->position);
			vec4 transformed_v3 = gm_mat4_multiply_vec4(&e2->model_matrix, v3->position);
			vec3 intersection;

			s32 edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f1),
				gm_vec4_to_vec3(transformed_f2),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge1_collisions, &intersection);

			edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f2),
				gm_vec4_to_vec3(transformed_f3),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge2_collisions, &intersection);

			edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f3),
				gm_vec4_to_vec3(transformed_f1),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge3_collisions, &intersection);
		}

		if (array_get_length(edge1_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge1_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge1_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge1_collisions), average_point);
			array_push(collision_points, &average_point);
		}
		if (array_get_length(edge2_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge2_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge2_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge2_collisions), average_point);
			array_push(collision_points, &average_point);
		}
		if (array_get_length(edge3_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge3_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge3_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge3_collisions), average_point);
			array_push(collision_points, &average_point);
		}

		if (array_get_length(edge1_collisions) > 0 ||
			array_get_length(edge2_collisions) > 0 ||
			array_get_length(edge3_collisions) > 0) {
			// collision!
			found_edge_collision = 1;
			break;
		}
	}

	// TEST e2 against e1
	for (u32 v = 0; v < array_get_length(e2->mesh.indices); v += 3) { // for each edge of e2
		Vertex* f1 = &e2->mesh.vertices[e2->mesh.indices[v]];
		Vertex* f2 = &e2->mesh.vertices[e2->mesh.indices[v + 1]];
		Vertex* f3 = &e2->mesh.vertices[e2->mesh.indices[v + 2]];
		vec4 transformed_f1 = gm_mat4_multiply_vec4(&e2->model_matrix, f1->position);
		vec4 transformed_f2 = gm_mat4_multiply_vec4(&e2->model_matrix, f2->position);
		vec4 transformed_f3 = gm_mat4_multiply_vec4(&e2->model_matrix, f3->position);
		array_clear(edge1_collisions);
		array_clear(edge2_collisions);
		array_clear(edge3_collisions);

		// for each vertex we need to check if it is inside the other mesh.
		for (u32 t = 0; t < array_get_length(e1->mesh.indices); t += 3) {
			Vertex* v1 = &e1->mesh.vertices[e1->mesh.indices[t + 0]];
			Vertex* v2 = &e1->mesh.vertices[e1->mesh.indices[t + 1]];
			Vertex* v3 = &e1->mesh.vertices[e1->mesh.indices[t + 2]];
			vec4 transformed_v1 = gm_mat4_multiply_vec4(&e1->model_matrix, v1->position);
			vec4 transformed_v2 = gm_mat4_multiply_vec4(&e1->model_matrix, v2->position);
			vec4 transformed_v3 = gm_mat4_multiply_vec4(&e1->model_matrix, v3->position);
			vec3 intersection;

			s32 edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f1),
				gm_vec4_to_vec3(transformed_f2),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge1_collisions, &intersection);

			edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f2),
				gm_vec4_to_vec3(transformed_f3),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge2_collisions, &intersection);

			edge_collides = collision_check_edge_collides_triangle(
				gm_vec4_to_vec3(transformed_f3),
				gm_vec4_to_vec3(transformed_f1),
				gm_vec4_to_vec3(transformed_v1),
				gm_vec4_to_vec3(transformed_v2),
				gm_vec4_to_vec3(transformed_v3),
				&intersection
			);

			// if the edge collides with the face, then there is a collision
			if (edge_collides) array_push(edge3_collisions, &intersection);
		}

		if (array_get_length(edge1_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge1_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge1_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge1_collisions), average_point);
			array_push(collision_points, &average_point);
		}
		if (array_get_length(edge2_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge2_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge2_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge2_collisions), average_point);
			array_push(collision_points, &average_point);
		}
		if (array_get_length(edge3_collisions) > 0) {
			vec3 average_point = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 i = 0; i < array_get_length(edge3_collisions); ++i) {
				average_point = gm_vec3_add(average_point, edge1_collisions[i]);
			}
			average_point = gm_vec3_scalar_product(1.0f / array_get_length(edge3_collisions), average_point);
			array_push(collision_points, &average_point);
		}

		if (array_get_length(edge1_collisions) > 0 ||
			array_get_length(edge2_collisions) > 0 ||
			array_get_length(edge3_collisions) > 0) {
			// collision!
			found_edge_collision = 1;
			break;
		}
	}

	if (found_edge_collision) {
		e1->diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
	}

	array_release(edge1_collisions);
	array_release(edge2_collisions);
	array_release(edge3_collisions);
}

void core_update(r32 delta_time)
{
	//physics_update(&e, forces, delta_time);
	array_clear(forces);
	array_clear(collision_points);
	for (u32 i = 0; i < array_get_length(entities); ++i) {
		entities[i].diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
	}

	for (u32 i = 0; i < array_get_length(entities); ++i) {	// for each entity
		Entity* first_entity = &entities[i];
		for (u32 j = i + 1; j < array_get_length(entities); ++j) { // for each other entity
			Entity* other_entity = &entities[j];
			check_entity_entity_collision(first_entity, other_entity);
		}
	}
}

void core_render()
{
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_CCW);
	for (u32 i = 0; i < array_get_length(entities); ++i)
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);
	graphics_renderer_debug_points(&pctx, collision_points, array_get_length(collision_points), (vec4){0.0f, 0.0f, 1.0f, 1.0f});
	graphics_renderer_primitives_flush(&pctx, &camera);

	//vec3 p = (vec3){1.0f, -1.0f, 1.0f};
	//vec3 f = gm_vec3_add(p, (vec3){0.0f, 0.0f, -1.0f});
	//graphics_renderer_debug_points(&pctx, &p, 1, (vec4){0.0f, 1.0f, 0.0f, 1.0f});
	//graphics_renderer_debug_points(&pctx, &f, 1, (vec4){0.0f, 0.0f, 1.0f, 1.0f});
	//graphics_renderer_primitives_flush(&pctx, &camera);
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
		Physics_Force pf;
		//pf.force = (vec4){0.0f, 0.0f, -0.1f, 0.0f};
		//pf.position = (vec4){-1.0f, -1.0f, 1.0f, 1.0f};
		//array_push(forces, &pf);
		pf.force = (vec4){0.0f, 0.0f, -1000.0f, 0.0f};
		pf.position = (vec4){1.0f, -1.0f, 1.0f, 1.0f};
		array_push(forces, &pf);
		key_state[GLFW_KEY_M] = false;
	}

	is_mouse_bound_to_joint_target_position = false;
	if (key_state[GLFW_KEY_1])
	{
		bound = &entities[0];
		is_mouse_bound_to_joint_target_position = true;
	}
	if (key_state[GLFW_KEY_2])
	{
		bound = &entities[1];
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