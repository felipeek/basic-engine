#include "collision.h"

boolean collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3) {
	vec3 v1 = gm_vec3_subtract(t2, t1);
	vec3 v2 = gm_vec3_subtract(t3, t1);
	vec3 plane_normal = gm_vec3_cross(v1, v2);
	r32 k = -(plane_normal.x * t1.x + plane_normal.y * t1.y + plane_normal.z * t1.z);
	// plane equation: plane_normal.x * x + plane_normal.y * y + plane_normal.z * z + k = 0

	r32 point_applied_on_eq = plane_normal.x * point.x + plane_normal.y * point.y + plane_normal.z * point.z + k;
	return point_applied_on_eq >= 0; // true -> "outside" | false -> "inside"
}

static boolean collision_intersection_ray_plane(vec3 ray_position, vec3 ray_direction, vec3 plane_point, vec3 plane_normal,
	vec3* _intersection)
{
  ray_direction = gm_vec3_normalize(ray_direction);
  plane_normal = gm_vec3_normalize(plane_normal);
  r32 ray_plane_dot = gm_vec3_dot(ray_direction, plane_normal);

  // Ray parallel to plane
  if (ray_plane_dot == 0.0f)
    return false;

  r32 d = gm_vec3_dot(gm_vec3_subtract(plane_point, ray_position), plane_normal) / ray_plane_dot;
  vec3 intersection = gm_vec3_add(gm_vec3_scalar_product(d, ray_direction), ray_position);
  if (_intersection) *_intersection = intersection;

  return true;
}

boolean collision_check_edge_collides_triangle(vec3 edge_p1, vec3 edge_p2, vec3 t1, vec3 t2, vec3 t3, vec3* intersection) {
	vec3 v1 = gm_vec3_subtract(t2, t1);
	vec3 v2 = gm_vec3_subtract(t3, t1);
	vec3 plane_normal = gm_vec3_cross(v1, v2);
	r32 k = -(plane_normal.x * t1.x + plane_normal.y * t1.y + plane_normal.z * t1.z);
	// plane equation: cross.x * x + cross.y * y + cross.z * z + k = 0
	
	r32 edge_p1_applied_on_eq = plane_normal.x * edge_p1.x + plane_normal.y * edge_p1.y + plane_normal.z * edge_p1.z + k;
	r32 edge_p2_applied_on_eq = plane_normal.x * edge_p2.x + plane_normal.y * edge_p2.y + plane_normal.z * edge_p2.z + k;
	int e1_check = edge_p1_applied_on_eq >= 0;
	int e2_check = edge_p2_applied_on_eq >= 0;

	if (e1_check && !e2_check || !e1_check && e2_check) {
		// edge intersects triangle plane
		// find intersection point

		assert(collision_intersection_ray_plane(edge_p1, gm_vec3_subtract(edge_p2, edge_p1), t1, plane_normal, intersection));

		// now we need to check its 2D containment inside the face (using barycentric coords)
		vec3 v3 = gm_vec3_subtract(*intersection, t1);
		r32 d00 = gm_vec3_dot(v1, v1);
		r32 d01 = gm_vec3_dot(v1, v2);
		r32 d11 = gm_vec3_dot(v2, v2);
		r32 d20 = gm_vec3_dot(v3, v1);
		r32 d21 = gm_vec3_dot(v3, v2);
		r32 denom = d00 * d11 - d01 * d01;
		r32 alpha = (d11 * d20 - d01 * d21) / denom;
		r32 beta = (d00 * d21 - d01 * d20) / denom;
		r32 gamma = 1.0f - alpha - beta;

		s32 inside =
			(0.0f <= alpha && alpha <= 1.0f) &&
			(0.0f <= beta && beta <= 1.0f) &&
			(0.0f <= gamma && gamma <= 1.0f);
		
		return inside;
	}

	return false;
}

boolean collision_check_dynamic_collision_between_point_and_entity_face(
	vec3 initial_point_position,
	vec3 final_point_position,
	vec3 initial_entity_position,
	vec3 final_entity_position,
	Quaternion initial_entity_rotation,
	Quaternion final_entity_rotation,
	vec3 entity_scale,
	vec3 face_point_local_coords_1,
	vec3 face_point_local_coords_2,
	vec3 face_point_local_coords_3,
	r32* time,
	vec3* normal)
{
	const u32 NUM_SAMPLES = 5;

	vec3 last_iteration_interpolated_point_position = initial_point_position;
	vec3 last_iteration_interpolated_entity_position = initial_entity_position;
	Quaternion last_iteration_interpolated_entity_rotation = initial_entity_rotation;

	for (u32 i = 0; i < NUM_SAMPLES; ++i) {
		vec3 this_iteration_interpolated_point_position = gm_vec3_add(
			gm_vec3_scalar_product((r32)(i + 1) / NUM_SAMPLES, final_point_position),
			gm_vec3_scalar_product((r32)(NUM_SAMPLES - 1 - i) / NUM_SAMPLES, initial_point_position)
		);

		vec3 this_iteration_interpolated_entity_position = gm_vec3_add(
			gm_vec3_scalar_product((r32)(i + 1) / NUM_SAMPLES, final_entity_position),
			gm_vec3_scalar_product((r32)(NUM_SAMPLES - 1 - i) / NUM_SAMPLES, initial_entity_position)
		);

		Quaternion this_iteration_interpolated_entity_rotation = quaternion_slerp(
			&initial_entity_rotation,
			&final_entity_rotation,
			(r32)(i + 1) / NUM_SAMPLES
		);

		mat4 frame_model_matrix = graphics_model_matrix(
			(vec4){last_iteration_interpolated_entity_position.x, last_iteration_interpolated_entity_position.y, last_iteration_interpolated_entity_position.z, 1.0f},
			last_iteration_interpolated_entity_rotation, entity_scale
		);
		mat4 inverse;
		assert(gm_mat4_inverse(&frame_model_matrix, &inverse));

		// we use the point position in this iteration, which means that we are effectively moving the point before rotating
		// (one way of doing it)
		vec4 point_in_local_coords = gm_mat4_multiply_vec4(
			&inverse,
			(vec4){this_iteration_interpolated_point_position.x, this_iteration_interpolated_point_position.y, this_iteration_interpolated_point_position.z, 1.0f}
		);

		Quaternion inverse_new_frame_rot = quaternion_inverse(&this_iteration_interpolated_entity_rotation);
		Quaternion point_rotation = quaternion_product(&last_iteration_interpolated_entity_rotation, &inverse_new_frame_rot);
		point_rotation = quaternion_product(&point_rotation, &last_iteration_interpolated_entity_rotation);
		mat4 rot_matrix = quaternion_get_matrix(&point_rotation);

		vec3 result_point_position = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&rot_matrix, point_in_local_coords));
		result_point_position = gm_vec3_add(result_point_position, last_iteration_interpolated_entity_position);
		result_point_position = gm_vec3_add(result_point_position, gm_vec3_scalar_product(-1.0f, this_iteration_interpolated_entity_position));
		result_point_position = gm_vec3_add(result_point_position, last_iteration_interpolated_entity_position);

		vec3 t_p1 = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&frame_model_matrix, (vec4){face_point_local_coords_1.x, face_point_local_coords_1.y, face_point_local_coords_1.z, 1.0f}));
		vec3 t_p2 = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&frame_model_matrix, (vec4){face_point_local_coords_2.x, face_point_local_coords_2.y, face_point_local_coords_2.z, 1.0f}));
		vec3 t_p3 = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&frame_model_matrix, (vec4){face_point_local_coords_3.x, face_point_local_coords_3.y, face_point_local_coords_3.z, 1.0f}));

		vec3 intersection;
		s32 collided = collision_check_edge_collides_triangle(
			last_iteration_interpolated_point_position,
			result_point_position,
			t_p1,
			t_p2,
			t_p3,
			&intersection
		);

		// to calculate the time of intersection we leverage the fact that the intersection is between
		// 'result_point_position' and 'last_iteration_interpolated_point_position', thus
		// d * vec(result_point_position - last_iteration_interpolated_point_position) = (intersection - last_iteration_interpolated_point_position)
		// therefore if we call the above equation d * A = B, then we know that d = B.x/A.x , d = B.y/A.y , d = B.z/A.z ... we chose 'x' in the impl
		vec3 point_to_point_vec = gm_vec3_subtract(result_point_position, last_iteration_interpolated_point_position);
		vec3 intersection_in_vec = gm_vec3_subtract(intersection, last_iteration_interpolated_point_position);
		r32 relative_edge_position_in_intersection = intersection_in_vec.x / point_to_point_vec.x;

		if (collided) {
			vec3 p2p1 = gm_vec3_subtract(t_p2, t_p1);
			vec3 p3p1 = gm_vec3_subtract(t_p3, t_p1);
			*normal = gm_vec3_normalize(gm_vec3_cross(p2p1, p3p1));
			*time = (r32)i / NUM_SAMPLES + relative_edge_position_in_intersection / NUM_SAMPLES;
			return true;
		}

		last_iteration_interpolated_entity_position = this_iteration_interpolated_entity_position;
		last_iteration_interpolated_entity_rotation = this_iteration_interpolated_entity_rotation;
		last_iteration_interpolated_point_position = this_iteration_interpolated_point_position;
	}

	return false;
}