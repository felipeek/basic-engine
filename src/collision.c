#include "collision.h"
#include <light_array.h>
#include <float.h>

/*
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
*/

s32 collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3) {
	vec3 v1 = gm_vec3_subtract(t2, t1);
	vec3 v2 = gm_vec3_subtract(t3, t1);
	vec3 plane_normal = gm_vec3_cross(v1, v2);
	r32 k = -(plane_normal.x * t1.x + plane_normal.y * t1.y + plane_normal.z * t1.z);
	// plane equation: plane_normal.x * x + plane_normal.y * y + plane_normal.z * z + k = 0

	r32 point_applied_on_eq = plane_normal.x * point.x + plane_normal.y * point.y + plane_normal.z * point.z + k;
	return point_applied_on_eq >= 0; // true -> "outside" | false -> "inside"
}

boolean collision_is_point_inside_with_mesh(vec3 point, Mesh m) {
	boolean inside_mesh = true;

	for (u32 i = 0; i < array_length(m.indices); i += 3) {
		vec3 t1 = m.vertices[i + 0].position;
		vec3 t2 = m.vertices[i + 1].position;
		vec3 t3 = m.vertices[i + 2].position;

		if (collision_check_point_side_of_triangle(point, t1, t2, t3)) {
			inside_mesh = false;
		}
	}

	return inside_mesh;
}

xvec3 collision_project_point_onto_plane(xvec3 p, xvec3 plane_normal, xvec3 plane_point) {
	plane_normal = gm_xvec3_normalize(plane_normal);
	r64 d = -gm_xvec3_dot(plane_normal, plane_point);
	return gm_xvec3_subtract(p, gm_xvec3_scalar_product(gm_xvec3_dot(plane_normal, p) + d, plane_normal));
}