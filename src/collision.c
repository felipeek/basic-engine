#include "collision.h"
#include <dynamic_array.h>

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y) {
	Collision_Point* collision_points = array_create(Collision_Point, 1);
	for (u32 i = 0; i < array_get_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		vec4 pos_wc = gm_mat4_multiply_vec4(&cube->model_matrix, v->position);
		if (pos_wc.y <= plane_y) {
			Collision_Point cp;
			cp.collision_point = gm_vec4_to_vec3(pos_wc);
			cp.normal = (vec3){0.0f, 1.0f, 0.0f};
			cp.penetration = plane_y - pos_wc.y;
			array_push(collision_points, &cp);
		}
	}

	// collision_points array might have duplicated collision points if the meshes have duplicated vertices
	// we need to get rid of the duplicated stuff
	for (u32 i = 0; i < array_get_length(collision_points); ++i) {
		for (u32 j = i + 1; j < array_get_length(collision_points); ++j) {
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