#include "collision.h"
#include <light_array.h>
#include <float.h>

Collision_Info* collision_get_plane_cube_points(Entity* cube, Entity* plane) {
	Collision_Info* collision_infos = array_new(Collision_Info);
	for (u32 i = 0; i < array_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		mat4 model_matrix = graphics_entity_get_model_matrix(cube);
		vec4 pos_wc = gm_mat4_multiply_vec4(&model_matrix, (vec4){v->position.x, v->position.y, v->position.z, 1.0f});
        pos_wc = gm_vec4_scalar_product(1.0f / pos_wc.w, pos_wc);
		if (pos_wc.y <= plane->world_position.y) {
			Collision_Info ci;
			ci.e1 = plane;
			ci.e2 = cube;
			ci.normal = (vec3){0.0f, 1.0f, 0.0f};
			ci.r1_wc = (vec3){0.0f, 0.0f, 0.0f};
			ci.r1_lc = (vec3){0.0f, 0.0f, 0.0f};
			ci.r2_wc = gm_vec3_subtract((vec3){pos_wc.x, pos_wc.y, pos_wc.z}, cube->world_position);
			ci.r2_lc = (vec3){v->position.x, v->position.y, v->position.z};
			ci.lambda_n = 0.0f;
			ci.lambda_t = 0.0f;
			array_push(collision_infos, ci);
		}
	}

	// collision_points array might have duplicated collision points if the meshes have duplicated vertices
	// we need to get rid of the duplicated stuff
	for (u32 i = 0; i < array_length(collision_infos); ++i) {
		for (u32 j = i + 1; j < array_length(collision_infos); ++j) {
			if (collision_infos[i].r2_lc.x == collision_infos[j].r2_lc.x &&
				collision_infos[i].r2_lc.y == collision_infos[j].r2_lc.y &&
				collision_infos[i].r2_lc.z == collision_infos[j].r2_lc.z) {

				// array length will decrease 1, everything should work
				array_remove(collision_infos, j);
				--j;
			}
		}
	}
	return collision_infos;
}