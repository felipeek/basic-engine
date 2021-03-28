#include "collision.h"
#include <float.h>
#include <light_array.h>

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

s32 collision_check_ray_intersects_triangle(vec3 p, vec3 v, vec3 t1, vec3 t2, vec3 t3, vec3* intersection) {
	vec3 v1 = gm_vec3_subtract(t2, t1);
	vec3 v2 = gm_vec3_subtract(t3, t1);
	vec3 plane_normal = gm_vec3_cross(v1, v2);
	assert(collision_intersection_ray_plane(p, v, t1, plane_normal, intersection));

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

static vec3 collision_project_point_onto_plane(vec3 p, vec3 plane_normal, vec3 plane_point) {
	plane_normal = gm_vec3_normalize(plane_normal);
	r32 d = -gm_vec3_dot(plane_normal, plane_point);
	return gm_vec3_subtract(p, gm_vec3_scalar_product(gm_vec3_dot(plane_normal, p) + d, plane_normal));
}

static vec3 collision_closest_point_to_triangle(vec3 p, vec3 a, vec3 b, vec3 c) {
	// Check if P in vertex region outside A
	vec3 ab = gm_vec3_subtract(b, a);
	vec3 ac = gm_vec3_subtract(c, a);
	vec3 ap = gm_vec3_subtract(p, a);
	r32 d1 = gm_vec3_dot(ab, ap);
	r32 d2 = gm_vec3_dot(ac, ap);
	if (d1 <= 0.0f && d2 <= 0.0f) return a; // barycentric coordinates (1,0,0)

	// Check if P in vertex region outside B
	vec3 bp = gm_vec3_subtract(p, b);
	r32 d3 = gm_vec3_dot(ab, bp);
	r32 d4 = gm_vec3_dot(ac, bp);
	if (d3 >= 0.0f && d4 <= d3) return b; // barycentric coordinates (0,1,0)

	// Check if P in edge region of AB, if so return projection of P onto AB
	r32 vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f) {
		r32 v = d1 / (d1 - d3);
		return gm_vec3_add(a, gm_vec3_scalar_product(v, ab)); // barycentric coordinates (1-v,v,0)
	}

	// Check if P in vertex region outside C
	vec3 cp = gm_vec3_subtract(p, c);
	r32 d5 = gm_vec3_dot(ab, cp);
	r32 d6 = gm_vec3_dot(ac, cp);
	if (d6 >= 0.0f && d5 <= d6) return c; // barycentric coordinates (0,0,1)

	// Check if P in edge region of AC, if so return projection of P onto AC
	r32 vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f) {
		r32 w = d2 / (d2 - d6);
		return gm_vec3_add(a, gm_vec3_scalar_product(w, ac));
	}

	// Check if P in edge region of BC, if so return projection of P onto BC
	r32 va = d3 * d6 - d5 * d4;
	if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f) {
		r32 w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		return gm_vec3_add(b, gm_vec3_scalar_product(w, gm_vec3_subtract(c, b))); // // barycentric coordinates (0,1-w,w)
	}

	// P inside face region. Compute Q through its barycentric coordinates (u,v,w)
	r32 denom = 1.0f / (va + vb + vc);
	r32 v = vb * denom;
	r32 w = vc * denom;
	return gm_vec3_add(a, gm_vec3_add(gm_vec3_scalar_product(v, ab), gm_vec3_scalar_product(w, ac)));
}

boolean collision_get_point_closest_intersection_with_mesh(vec3 point, Mesh m, vec3* closest_point, r32* closest_distance) {
	*closest_distance = FLT_MAX;
	boolean inside_mesh = true;

	for (u32 i = 0; i < array_length(m.indices); i += 3) {
		vec3 t1 = gm_vec4_to_vec3(m.vertices[i + 0].position);
		vec3 t2 = gm_vec4_to_vec3(m.vertices[i + 1].position);
		vec3 t3 = gm_vec4_to_vec3(m.vertices[i + 2].position);

		vec3 current_closest_point = collision_closest_point_to_triangle(point, t1, t2, t3);
		r32 current_closest_distance = gm_vec3_length(gm_vec3_subtract(point, current_closest_point));

		if (current_closest_distance < *closest_distance) {
			*closest_distance = current_closest_distance;
			*closest_point = current_closest_point;
		}

		// @TODO: i think we could run it only for the closest triangle...
		if (collision_check_point_side_of_triangle(point, t1, t2, t3)) {
			inside_mesh = false;
		}
	}

	return inside_mesh;
}