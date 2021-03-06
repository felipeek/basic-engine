#include "collision.h"

s32 collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3) {
	vec3 v1 = gm_vec3_subtract(t2, t1);
	vec3 v2 = gm_vec3_subtract(t3, t1);
	vec3 plane_normal = gm_vec3_cross(v1, v2);
	r32 k = -(plane_normal.x * t1.x + plane_normal.y * t1.y + plane_normal.z * t1.z);
	// plane equation: plane_normal.x * x + plane_normal.y * y + plane_normal.z * z + k = 0

	r32 point_applied_on_eq = plane_normal.x * point.x + plane_normal.y * point.y + plane_normal.z * point.z + k;
	return point_applied_on_eq >= 0; // true -> "outside" | false -> "inside"
}

static s32 collision_intersection_ray_plane(vec3 ray_position, vec3 ray_direction, vec3 plane_point, vec3 plane_normal,
	r32* _d, vec3* _intersection)
{
  ray_direction = gm_vec3_normalize(ray_direction);
  plane_normal = gm_vec3_normalize(plane_normal);
  r32 ray_plane_dot = gm_vec3_dot(ray_direction, plane_normal);

  // Ray parallel to plane
  if (ray_plane_dot == 0.0f)
    return false;

  r32 d = gm_vec3_dot(gm_vec3_subtract(plane_point, ray_position), plane_normal) / ray_plane_dot;
  vec3 intersection = gm_vec3_add(gm_vec3_scalar_product(d, ray_direction), ray_position);
  if (_d) *_d = d;
  if (_intersection) *_intersection = intersection;

  return true;
}

s32 collision_check_edge_collides_triangle(vec3 edge_p1, vec3 edge_p2, vec3 t1, vec3 t2, vec3 t3, r32* d, vec3* intersection) {
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

		assert(collision_intersection_ray_plane(edge_p1, gm_vec3_subtract(edge_p2, edge_p1), t1, plane_normal, d, intersection));

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