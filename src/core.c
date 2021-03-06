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

	Entity e;
	Mesh m = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&e, m, (vec4){-1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	array_push(entities, &e);
	graphics_entity_create_with_color(&e, m, (vec4){1.2f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f),
		(vec3){1.0f, 1.0f, 1.0f}, (vec4){1.0f, 0.0f, 0.0f, 1.0f});
	array_push(entities, &e);

	menu_register_dummy_callback(menu_dummy_callback);
	graphics_renderer_primitives_init(&pctx);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	//physics_update(&e, forces, delta_time);
	array_clear(forces);

	for (u32 i = 0; i < array_get_length(entities); ++i) {	// for each entity
		Entity* first_entity = &entities[i];
		s32 outside_mesh = 0;

		for (u32 j = 0; j < array_get_length(entities); ++j) { // for each other entity
			if (i == j) continue;

			Entity* other_entity = &entities[j];

			for (u32 v = 0; v < array_get_length(first_entity->mesh.vertices); ++v) { // for each vertex of the (first) entity
				Vertex* point = &first_entity->mesh.vertices[v];
				vec4 transformed_point = gm_mat4_multiply_vec4(&first_entity->model_matrix, point->position);
				//vec4 transformed_point = gm_mat4_multiply_vec4(&first_entity->model_matrix, first_entity->world_position);

				// for each vertex we need to check if it is inside the other mesh.
				for (u32 t = 0; t < array_get_length(other_entity->mesh.indices); t += 3) {
					Vertex* v1 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 0]];
					Vertex* v2 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 1]];
					Vertex* v3 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 2]];
					vec4 transformed_v1 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v1->position);
					vec4 transformed_v2 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v2->position);
					vec4 transformed_v3 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v3->position);

					// 1 -> out
					// 0 -> in
					outside_mesh = collision_check_point_side_of_triangle(
						gm_vec4_to_vec3(transformed_point),
						gm_vec4_to_vec3(transformed_v1),
						gm_vec4_to_vec3(transformed_v2),
						gm_vec4_to_vec3(transformed_v3)
					);

					// if the vertex is outside the triangle, then we know that this vertex is NOT inside the mesh
					if (outside_mesh) break;
				}

				// if !hit, then the vertex WAS inside the mesh, so there IS collision!
				if (!outside_mesh) break;
			}

			// since we dont care with which entity the collision happend, we can just skip additional entities
			if (!outside_mesh) break;
		}

		/* 
		1 - implement edge checking
		2 - render collision points
		*/

		// no need to proceed.
		if (!outside_mesh) {
			first_entity->diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
			continue;
		}

		s32 edge_collides = 0;
		for (u32 j = 0; j < array_get_length(entities); ++j) { // for each other entity
			if (i == j) continue;

			Entity* other_entity = &entities[j];

			for (u32 v = 0; v < array_get_length(first_entity->mesh.indices); v += 3) { // for each edge of the (first) entity
				Vertex* f1 = &first_entity->mesh.vertices[first_entity->mesh.indices[v]];
				Vertex* f2 = &first_entity->mesh.vertices[first_entity->mesh.indices[v + 1]];
				Vertex* f3 = &first_entity->mesh.vertices[first_entity->mesh.indices[v + 2]];
				vec4 transformed_f1 = gm_mat4_multiply_vec4(&first_entity->model_matrix, f1->position);
				vec4 transformed_f2 = gm_mat4_multiply_vec4(&first_entity->model_matrix, f2->position);
				vec4 transformed_f3 = gm_mat4_multiply_vec4(&first_entity->model_matrix, f3->position);

				// for each vertex we need to check if it is inside the other mesh.
				for (u32 t = 0; t < array_get_length(other_entity->mesh.indices); t += 3) {
					Vertex* v1 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 0]];
					Vertex* v2 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 1]];
					Vertex* v3 = &other_entity->mesh.vertices[other_entity->mesh.indices[t + 2]];
					vec4 transformed_v1 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v1->position);
					vec4 transformed_v2 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v2->position);
					vec4 transformed_v3 = gm_mat4_multiply_vec4(&other_entity->model_matrix, v3->position);

					edge_collides = collision_check_edge_collides_triangle(
						gm_vec4_to_vec3(transformed_f1),
						gm_vec4_to_vec3(transformed_f2),
						gm_vec4_to_vec3(transformed_v1),
						gm_vec4_to_vec3(transformed_v2),
						gm_vec4_to_vec3(transformed_v3)
					);

					// if the edge collides with the face, then there is a collision
					if (edge_collides) break;

					edge_collides = collision_check_edge_collides_triangle(
						gm_vec4_to_vec3(transformed_f2),
						gm_vec4_to_vec3(transformed_f3),
						gm_vec4_to_vec3(transformed_v1),
						gm_vec4_to_vec3(transformed_v2),
						gm_vec4_to_vec3(transformed_v3)
					);

					// if the edge collides with the face, then there is a collision
					if (edge_collides) break;

					edge_collides = collision_check_edge_collides_triangle(
						gm_vec4_to_vec3(transformed_f3),
						gm_vec4_to_vec3(transformed_f1),
						gm_vec4_to_vec3(transformed_v1),
						gm_vec4_to_vec3(transformed_v2),
						gm_vec4_to_vec3(transformed_v3)
					);

					// if the edge collides with the face, then there is a collision
					if (edge_collides) break;
				}

				// if edge_collides, then the edge collided with the face, so there IS collision!
				if (edge_collides) break;
			}

			// since we dont care with which entity the collision happend, we can just skip additional entities
			if (edge_collides) break;
		}

		if (edge_collides) {
			first_entity->diffuse_info.diffuse_color = (vec4){0.0f, 1.0f, 0.0f, 1.0f};
		} else {
			first_entity->diffuse_info.diffuse_color = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
		}
	}
}

void core_render()
{
	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_CCW);
	for (u32 i = 0; i < array_get_length(entities); ++i)
		graphics_entity_render_phong_shader(&camera, &entities[i], lights);

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