#include "collision.h"
#include <light_array.h>
#include <float.h>

Collision_Info* collision_collect_collisions(Entity* cube, Entity* plane) {
	Collision_Info* collision_infos = array_new(Collision_Info);

	// Collect Collisions
	Collision_Point* pts = collision_get_plane_cube_points(cube, plane->world_position.y);
	for (u32 j = 0; j < array_length(pts); ++j) {
		Collision_Point* cp = &pts[j];
		Collision_Info ci = (Collision_Info){0};
		ci.e1 = plane;
		ci.e2 = cube;
		ci.r1_lc = (vec3){0.0f, 0.0f, 0.0f};
		ci.r2_lc = cp->r_lc;
		ci.r1_wc = (vec3){0.0f, 0.0f, 0.0f};
		ci.r2_wc = cp->r_wc;
		ci.normal = cp->normal;
		array_push(collision_infos, ci);
	}

	array_free(pts);
	return collision_infos;
}

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y) {
	Collision_Point* collision_points = array_new(Collision_Point);
	for (u32 i = 0; i < array_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		mat4 model_matrix = graphics_entity_get_model_matrix(cube);
		vec4 pos_wc = gm_mat4_multiply_vec4(&model_matrix, (vec4){v->position.x, v->position.y, v->position.z, 1.0f});
        pos_wc = gm_vec4_scalar_product(1.0f / pos_wc.w, pos_wc);
		if (pos_wc.y <= plane_y) {
			Collision_Point cp;
			cp.collision_point = gm_vec4_to_vec3(pos_wc);
			cp.normal = (vec3){0.0f, 1.0f, 0.0f};
			cp.penetration = plane_y - pos_wc.y;
			cp.r_lc = (vec3){v->position.x, v->position.y, v->position.z};
			cp.r_wc = gm_vec3_subtract((vec3){pos_wc.x, pos_wc.y, pos_wc.z}, cube->world_position);
			cp.lambda_n = 0.0f;
			cp.lambda_t = 0.0f;
			array_push(collision_points, cp);
		}
	}

	// collision_points array might have duplicated collision points if the meshes have duplicated vertices
	// we need to get rid of the duplicated stuff
	for (u32 i = 0; i < array_length(collision_points); ++i) {
		for (u32 j = i + 1; j < array_length(collision_points); ++j) {
			if (collision_points[i].collision_point.x == collision_points[j].collision_point.x &&
				collision_points[i].collision_point.y == collision_points[j].collision_point.y &&
				collision_points[i].collision_point.z == collision_points[j].collision_point.z) {

				// array length will decrease 1, everything should work
				array_remove(collision_points, j);
				--j;
			}
		}
	}
	return collision_points;
}