#include "collision.h"
#include <light_array.h>
#include <float.h>

static r32 get_sphere_radius(Entity* sphere) {
	// sphere must be 0,0,0 centered
	return gm_vec3_length(sphere->mesh.vertices[0].position);
}

Collision_Info* collision_get_sphere_sphere_points(Entity* sphere1, Entity* sphere2) {
	Collision_Info* collision_infos = array_new(Collision_Info);
	r32 sphere1_radius = get_sphere_radius(sphere1);
	r32 sphere2_radius = get_sphere_radius(sphere2);
	vec3 distance_vec = gm_vec3_subtract(sphere2->world_position, sphere1->world_position);
	if (gm_vec3_length(distance_vec) < sphere1_radius + sphere2_radius) {
		Collision_Info ci;
		ci.e1 = sphere1;
		ci.e2 = sphere2;
		ci.normal = gm_vec3_normalize(distance_vec);
		ci.r1_wc = gm_vec3_scalar_product(sphere1_radius, ci.normal);
		ci.r2_wc = gm_vec3_scalar_product(-sphere2_radius, ci.normal);
		Quaternion inv = quaternion_inverse(&sphere1->world_rotation);
		mat3 inv_m = quaternion_get_matrix3(&inv);
		ci.r1_lc = gm_mat3_multiply_vec3(&inv_m, ci.r1_wc);
		inv = quaternion_inverse(&sphere2->world_rotation);
		inv_m = quaternion_get_matrix3(&inv);
		ci.r2_lc = gm_mat3_multiply_vec3(&inv_m, ci.r2_wc);
		ci.lambda_n = 0.0f;
		ci.lambda_t = 0.0f;
		array_push(collision_infos, ci);

		// INVESTIGATE DOUBLE-Q !
	}
	return collision_infos;
}

Collision_Info* collision_get_plane_sphere_points(Entity* sphere, Entity* plane) {
	Collision_Info* collision_infos = array_new(Collision_Info);
	r32 sphere_radius = get_sphere_radius(sphere);
	if (sphere->world_position.y - sphere_radius <= plane->world_position.y) {
		Collision_Info ci;
		ci.e1 = plane;
		ci.e2 = sphere;
		ci.normal = (vec3){0.0f, 1.0f, 0.0f};
		ci.r1_wc = (vec3){0.0f, 0.0f, 0.0f};
		ci.r1_lc = (vec3){0.0f, 0.0f, 0.0f};
		ci.r2_wc = (vec3){0.0f, -sphere_radius, 0.0f};
		Quaternion inv = quaternion_inverse(&sphere->world_rotation);
		mat3 inv_m = quaternion_get_matrix3(&inv);
		ci.r2_lc = gm_mat3_multiply_vec3(&inv_m, ci.r2_wc);
		ci.lambda_n = 0.0f;
		ci.lambda_t = 0.0f;
		array_push(collision_infos, ci);
	}

	return collision_infos;
}

PMC_Contact* collision_get_plane_cube_points(Entity* cube, Entity* plane) {
	PMC_Contact* collision_infos = array_new(PMC_Contact);
	for (u32 i = 0; i < array_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		mat4 model_matrix = graphics_entity_get_model_matrix(cube);
		vec4 pos_wc = gm_mat4_multiply_vec4(&model_matrix, (vec4){v->position.x, v->position.y, v->position.z, 1.0f});
        pos_wc = gm_vec4_scalar_product(1.0f / pos_wc.w, pos_wc);
		if (pos_wc.y <= PLANE_Y + 1.0f) {
			PMC_Contact ci;
			ci.e1 = plane;
			ci.e2 = cube;
			ci.normal = (vec3){0.0f, 1.0f, 0.0f};
			ci.r1_wc = gm_vec3_subtract((vec3){pos_wc.x, PLANE_Y + 1.0f, pos_wc.z}, plane->world_position);
			ci.r1_lc = ci.r1_wc; // assuming no SCALE and no ROTATION (only translation)
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