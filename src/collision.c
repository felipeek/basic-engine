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

s32
collision_gjk_individual_support(Bounding_Shape* b, vec3 direction)
{
  float max = -FLT_MAX;
  int index = 0;
  for (int i = 0; i < b->vertex_count; ++i)
  {
	float dot = gm_vec3_dot(b->vertices[i], direction);
	if (dot > max)
	{
	  max = dot;
	  index = i;
	}
  }

//  Support_Point sup_point = {
//	.v = b->vertices[index],
//	.sup = b->vertices[index]
//  };
  return index;
}

/* ************** */

#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))
int tmp = 0;

boolean PointInPlane(const Plane* plane, vec3 position) {
	float distance = -gm_vec3_dot(plane->normal, plane->point);
	if (gm_vec3_dot(position, plane->normal) + distance < 0.0f) {
		return false;
	}

	return true;
}

boolean PlaneEdgeIntersection(
	const Plane* plane,
	const vec3 start,
	const vec3 end,
	vec3* out_point)
{
	vec3 ab = gm_vec3_subtract(end, start);

	//Check that the edge and plane are not parallel and thus never intersect
	// We do this by projecting the line (start - A, End - B) ab along the plane
	float ab_p = gm_vec3_dot(plane->normal, ab);
	if (fabs(ab_p) > 1e-6f)
	{
		//Generate a random point on the plane (any point on the plane will suffice)
		float distance = -gm_vec3_dot(plane->normal, plane->point);
		vec3 p_co = gm_vec3_scalar_product(-distance, plane->normal);

		//Work out the edge factor to scale edge by
		// e.g. how far along the edge to traverse before it meets the plane.
		//      This is computed by: -proj<plane_nrml>(edge_start - any_planar_point) / proj<plane_nrml>(edge_start - edge_end)
		float fac = -gm_vec3_dot(plane->normal, gm_vec3_subtract(start, p_co)) / ab_p;

		//Stop any large floating point divide issues with almost parallel planes
		fac = min(max(fac, 0.0f), 1.0f); 

		//Return point on edge
		*out_point = gm_vec3_add(start, gm_vec3_scalar_product(fac, ab));
		return true;
	}

	return false;
}

//Performs sutherland hodgman clipping algorithm to clip the provided mesh
//    or polygon in regards to each of the provided clipping planes.
void SutherlandHodgmanClipping(
	vec3* input_polygon,
	int num_clip_planes,
	const Plane* clip_planes,
	vec3** out_polygon,
	boolean removeNotClipToPlane)
{
	assert(out_polygon != NULL);
	assert(num_clip_planes > 0);

	//Create temporary list of vertices
	// - We will keep ping-pong'ing between the two lists updating them as we go.
	vec3* input = array_copy(input_polygon);
	vec3* output = array_new(vec3);

	//Iterate over each clip_plane provided
	for (int i = 0; i < num_clip_planes; ++i)
	{
		//If every single point on our shape has already been removed previously, just exit
		if (array_length(input) == 0)
			break;

		const Plane* plane = &clip_planes[i];

		//Loop through each edge of the polygon (see line_loop from gfx) and clip
		// that edge against the current plane.
		vec3 tempPoint, startPoint = input[array_length(input) - 1];
		for (u32 j = 0; j < array_length(input); ++j)
		{
			vec3 endPoint = input[j];
			boolean startInPlane = PointInPlane(plane, startPoint);
			boolean endInPlane = PointInPlane(plane, endPoint);

			//If it's the final pass, just remove all points outside the reference plane
			// - This won't return a true polygon if set, but is needed for the last step
			//   we do in the manifold generation
			if (removeNotClipToPlane)
			{
				if (endInPlane)
					array_push(output, endPoint);
			}
			else
			{
				//If the edge is entirely within the clipping plane, keep it as it is
				if (startInPlane && endInPlane)
				{
					array_push(output, endPoint);
				}
				//If the edge interesects the clipping plane, cut the edge along clip plane
				else if (startInPlane && !endInPlane)
				{
					if (PlaneEdgeIntersection(plane, startPoint, endPoint, &tempPoint))
						array_push(output, tempPoint);
				}
				else if (!startInPlane && endInPlane)
				{
					if (PlaneEdgeIntersection(plane, startPoint, endPoint, &tempPoint))
						array_push(output, tempPoint);

					array_push(output, endPoint);
				}
			}
			//..otherwise the edge is entirely outside the clipping plane and should be removed/ignored

			startPoint = endPoint;
		}

		//Swap input/output polygons, and clear output list for us to generate afresh
		vec3* tmp = input;
		input = output;
		output = tmp;
		array_clear(output);
	}

	*out_polygon = input;
}

// Gets the closest point x on the line (edge) to point (pos)
vec3 GetClosestPoint(vec3 pos, vec3 e1, vec3 e2)
{
	//As we have a line not two points, the final value could be anywhere between the edge points A/B
	//	We solve this by projecting the point (pos) onto the line described by the edge, and then
	//  clamping it so can it has to be between the line's start and end points.
	//  - Notation: A - Line Start, B - Line End, P - Point not on the line we want to project
	vec3 diff_AP = gm_vec3_subtract(pos, e1);
	vec3 diff_AB = gm_vec3_subtract(e2, e1);

	//Distance along the line of point 'pos' in world distance 
	float ABAPproduct = gm_vec3_dot(diff_AP, diff_AB);
	float magnitudeAB = gm_vec3_dot(diff_AB, diff_AB);

	//Distance along the line of point 'pos' between 0-1 where 0 is line start and 1 is line end
	float distance = ABAPproduct / magnitudeAB;

	//Clamp distance so it cant go beyond the line's start/end in either direction
	distance = max(min(distance, 1.0f), 0.0f);

	//Use distance from line start (0-1) to compute the actual position
	return gm_vec3_add(e1, gm_vec3_scalar_product(distance, diff_AB));
}

vec3 GetClosestPointPolygon(vec3 position, vec3* polygon)
{
	vec3 final_closest_point = (vec3){0.0f, 0.0f, 0.0f};
	float final_closest_distsq = FLT_MAX;

	vec3 last = polygon[array_length(polygon) - 1];
	for (u32 i = 0; i < array_length(polygon); ++i)
	{
		vec3 next = polygon[i];
		vec3 edge_closest_point = GetClosestPoint(position, last, next);

		//Compute the distance of the closest point on the line to the actual point
		vec3 diff = gm_vec3_subtract(position, edge_closest_point);
		float temp_distsq = gm_vec3_dot(diff, diff);

		//Only store the closest point if it's closer than all the other line's closest points
		if (temp_distsq < final_closest_distsq)
		{
			final_closest_distsq = temp_distsq;
			final_closest_point = edge_closest_point;
		}

		last = next;
	}

	return final_closest_point;
}

static vec3 get_normal_of_triangle(vec3 t1, vec3 t2, vec3 t3) {
	vec3 t12 = gm_vec3_subtract(t2, t1);
	vec3 t13 = gm_vec3_subtract(t3, t1);
	return gm_vec3_normalize(gm_vec3_cross(t12, t13));
}

static boolean same_vector(vec3 a, vec3 b) {
	return a.x == b.x && a.y == b.y && a.z == b.z;
}

static Plane* find_boundary_planes(Bounding_Shape* b, Mesh* m, vec3* points) {
	Plane* result = array_new(Plane);

	for (u32 i = 0; i < array_length(m->indices); i += 3) {
		u32 i1 = m->indices[i + 0];
		u32 i2 = m->indices[i + 1];
		u32 i3 = m->indices[i + 2];

		vec3 v1 = b->vertices[i1];
		vec3 v2 = b->vertices[i2];
		vec3 v3 = b->vertices[i3];

		boolean found_v1 = false;
		boolean found_v2 = false;
		boolean found_v3 = false;

		for (u32 j = 0; j < array_length(points); ++j) {
			vec3 point = points[j];
			if (same_vector(v1, points[j])) {
				found_v1 = true;
			}
			if (same_vector(v2, points[j])) {
				found_v2 = true;
			}
			if (same_vector(v3, points[j])) {
				found_v3 = true;
			}
		}

#if 1
		if (found_v1 || found_v2 || found_v3) {
			if (!(found_v1 && found_v2 && found_v3)) {
				Plane p;
				p.point = v3;
				p.normal = gm_vec3_scalar_product(-1.0f, get_normal_of_triangle(v1, v2, v3));
				array_push(result, p);
			}
		}
#else
		if (found_v1 && found_v2 && !found_v3) {
			Plane p;
			p.point = v3;
			p.normal = gm_vec3_scalar_product(-1.0f, get_normal_of_triangle(v1, v2, v3));
			array_push(result, p);
		} else if (found_v1 && !found_v2 && found_v3) {
			Plane p;
			p.point = v2;
			p.normal = gm_vec3_scalar_product(-1.0f, get_normal_of_triangle(v1, v2, v3));
			array_push(result, p);
		} else if (!found_v1 && found_v2 && found_v3) {
			Plane p;
			p.point = v1;
			p.normal = gm_vec3_scalar_product(-1.0f, get_normal_of_triangle(v1, v2, v3));
			array_push(result, p);
		}
#endif
	}

	// filter planes
	for (u32 i = 0; i < array_length(result); ++i) {
		for (u32 j = i + 1; j < array_length(result); ++j) {
			Plane* p1 = &result[i];
			Plane* p2 = &result[j];
			const r32 EPSILON = 0.0000001f;
			r32 proj1 = gm_vec3_dot(p1->point, p1->normal);
			r32 proj2 = gm_vec3_dot(p2->point, p2->normal);
			if (fabsf(proj1 - proj2) < EPSILON) {
				if (gm_vec3_length(gm_vec3_subtract(p1->normal, p2->normal)) < EPSILON) {
					array_remove(result, j);
					--j;
				}
			}
		}
	}
	return result;
}

static vec3* get_triangle_of_most_fitting_neighbor(Bounding_Shape* b, Mesh* m, vec3* most_fitting_triangle, vec3 normal, vec3* neighbor_normal) {
	vec3* result = array_new(vec3);
	boolean found = false;
	r32 max_dot;
	vec3 chosen_normal;

	vec3 target1 = most_fitting_triangle[0];
	vec3 target2 = most_fitting_triangle[1];
	vec3 target3 = most_fitting_triangle[2];

	for (u32 i = 0; i < array_length(m->indices); i += 3) {
		u32 i1 = m->indices[i + 0];
		u32 i2 = m->indices[i + 1];
		u32 i3 = m->indices[i + 2];

		vec3 v1 = b->vertices[i1];
		vec3 v2 = b->vertices[i2];
		vec3 v3 = b->vertices[i3];

		if (same_vector(v1, target1) || same_vector(v1, target2) || same_vector(v1, target3)) {
			if (same_vector(v2, target1) || same_vector(v2, target2) || same_vector(v2, target3)) {
				if (!same_vector(v3, target1) && !same_vector(v3, target2) && !same_vector(v3, target3)) {
					// v3
					vec3 n = get_normal_of_triangle(v1, v2, v3);
					r32 dot = gm_vec3_dot(n, normal);
					if (!found || dot > max_dot) {
						found = true;
						max_dot = dot;
						array_clear(result);
						array_push(result, v1);
						array_push(result, v2);
						array_push(result, v3);
						chosen_normal = n;
					}
				}
			} else {
				if (same_vector(v3, target1) || same_vector(v3, target2) || same_vector(v3, target3)) {
					// v2
					vec3 n = get_normal_of_triangle(v1, v2, v3);
					r32 dot = gm_vec3_dot(n, normal);
					if (!found || dot > max_dot) {
						found = true;
						max_dot = dot;
						array_clear(result);
						array_push(result, v1);
						array_push(result, v2);
						array_push(result, v3);
						chosen_normal = n;
					}
				}
			}
		} else {
			if (same_vector(v2, target1) || same_vector(v2, target2) || same_vector(v2, target3)) {
				if (same_vector(v3, target1) || same_vector(v3, target2) || same_vector(v3, target3)) {
					// v1
					vec3 n = get_normal_of_triangle(v1, v2, v3);
					r32 dot = gm_vec3_dot(n, normal);
					if (!found || dot > max_dot) {
						found = true;
						max_dot = dot;
						array_clear(result);
						array_push(result, v1);
						array_push(result, v2);
						array_push(result, v3);
						chosen_normal = n;
					}
				}
			}
		}
	}


	assert(found);
	*neighbor_normal = chosen_normal;
	return result;
}

static vec3* get_triangle_with_most_fitting_normal(Bounding_Shape* b, vec3 normal, Mesh* m, s32 vertex_idx, vec3* triangle_normal) {
	vec3* result = array_new(vec3);

	boolean found = false;
	r32 max_dot;
	vec3 chosen_normal;

	vec3 vertex_of_interest = b->vertices[vertex_idx];

	for (u32 i = 0; i < array_length(m->indices); i += 3) {
		u32 i1 = m->indices[i + 0];
		u32 i2 = m->indices[i + 1];
		u32 i3 = m->indices[i + 2];

		vec3 v1 = b->vertices[i1];
		vec3 v2 = b->vertices[i2];
		vec3 v3 = b->vertices[i3];

		if (v1.x == vertex_of_interest.x && v1.y == vertex_of_interest.y && v1.z == vertex_of_interest.z) {
			vec3 n = get_normal_of_triangle(v1, v2, v3);
			r32 dot = gm_vec3_dot(n, normal);
			if (!found || dot > max_dot) {
				found = true;
				max_dot = dot;
				array_clear(result);
				array_push(result, v1);
				array_push(result, v2);
				array_push(result, v3);
				chosen_normal = n;
			}
		} else if (v2.x == vertex_of_interest.x && v2.y == vertex_of_interest.y && v2.z == vertex_of_interest.z) {
			vec3 n = get_normal_of_triangle(v1, v2, v3);
			r32 dot = gm_vec3_dot(n, normal);
			if (!found || dot > max_dot) {
				found = true;
				max_dot = dot;
				array_clear(result);
				array_push(result, v1);
				array_push(result, v2);
				array_push(result, v3);
				chosen_normal = n;
			}
		} else if (v3.x == vertex_of_interest.x && v3.y == vertex_of_interest.y && v3.z == vertex_of_interest.z) {
			vec3 n = get_normal_of_triangle(v1, v2, v3);
			r32 dot = gm_vec3_dot(n, normal);
			if (!found || dot > max_dot) {
				found = true;
				max_dot = dot;
				array_clear(result);
				array_push(result, v1);
				array_push(result, v2);
				array_push(result, v3);
				chosen_normal = n;
			}
		}
	}

	assert(found);
	*triangle_normal = chosen_normal;
	return result;
}

static vec3* merge_triangles(vec3* triangle1, vec3* triangle2) {
	vec3* result = array_new(vec3);

	boolean found = false;
	vec3 different_vertex_triangle_2;

	for (u32 i = 0; i < 3; ++i) {
		boolean same = false;
		for (u32 j = 0; j < 3; ++j) {
			if (same_vector(triangle2[i], triangle1[j])) {
				same = true;
				break;
			}
		}

		if (!same) {
			different_vertex_triangle_2 = triangle2[i];
			found = true;
			break;
		}
	}

	assert(found);
	found = false;

	for (u32 i = 0; i < 3; ++i) {
		boolean same = false;
		for (u32 j = 0; j < 3; ++j) {
			if (same_vector(triangle1[i], triangle2[j])) {
				same = true;
				break;
			}
		}

		if (!same) {
			array_push(result, triangle1[i]);
			array_push(result, triangle1[((int)i + 1) % 3]);
			array_push(result, different_vertex_triangle_2);
			array_push(result, triangle1[((int)i + 2) % 3]);
			found = true;
			break;
		}
	}

	assert(found);
	return result;
}

static vec3* get_face_with_most_fitting_normal_in_winding_order(Bounding_Shape* b, vec3 normal, Mesh* m, vec3* face_normal) {
	const r32 EPSILON = 0.000001f;
	s32 support_idx = collision_gjk_individual_support(b, normal);
	vec3 most_fitting_normal;
	vec3* most_fitting_triangle = get_triangle_with_most_fitting_normal(b, normal, m, support_idx, &most_fitting_normal);
	vec3 neighbor_normal;
	vec3* most_fitting_neighbor = get_triangle_of_most_fitting_neighbor(b, m, most_fitting_triangle, normal, &neighbor_normal);

	// need to test most_fitting_normal against neighbor_normal since we wanna assert whether we are dealing with the same plane
	r32 proj = gm_vec3_dot(most_fitting_normal, neighbor_normal);

	if ((proj - 1.0f) > -EPSILON && (proj - 1.0f) < EPSILON) {
		most_fitting_triangle = merge_triangles(most_fitting_triangle, most_fitting_neighbor);
	}

	*face_normal = most_fitting_normal;

	return most_fitting_triangle;
}

typedef struct {
	vec2 pv2;
	vec3 wc1;
	vec3 wc2;
} Persistent_Manifold_Point;

static vec3* collect_all_edges_of_support(Bounding_Shape* b, Mesh* m, vec3 support) {
	vec3* edges = array_new(vec3);

	for (u32 i = 0; i < array_length(m->indices); i += 3) {
		vec3 v1 = b->vertices[m->indices[i + 0]];
		vec3 v2 = b->vertices[m->indices[i + 1]];
		vec3 v3 = b->vertices[m->indices[i + 2]];

		if (!same_vector(support, v1) && !same_vector(support, v2) && !same_vector(support, v3)) {
			continue;
		}

		vec3 m_last, m_next;

		if (same_vector(support, v1)) {
			m_last = v3;
			m_next = v2;
		} else if (same_vector(support, v2)) {
			m_last = v1;
			m_next = v3;
		} else {
			m_last = v2;
			m_next = v1;
		}

		// each triangle defines 2 edges!
		array_push(edges, m_last);
		array_push(edges, support);
		array_push(edges, support);
		array_push(edges, m_next);

		// we are actually duplicating a lot of edges here because each edge appear in 2 different triangles...
		// but with different winding orders...
		// when we have a proper implementation, we need to take a careful look at the winding of the edges
		// for now, we know that at for each duplicated edge duo, one of them will have the correct winding
	}

	return edges;
}

vec3* get_edge_with_most_fitting_normal(Bounding_Shape* b1, Bounding_Shape* b2, Mesh* m1, Mesh* m2, vec3 normal, vec3* edge_normal) {
	vec3 inverted_normal = gm_vec3_scalar_product(-1.0f, normal);
	s32 support1_idx = collision_gjk_individual_support(b1, normal);
	s32 support2_idx = collision_gjk_individual_support(b2, inverted_normal);

	vec3 support1 = b1->vertices[support1_idx];
	vec3 support2 = b2->vertices[support2_idx];

	vec3* edges1 = collect_all_edges_of_support(b1, m1, support1);
	vec3* edges2 = collect_all_edges_of_support(b2, m2, support2);

	vec3* result = array_new(vec3);
	r32 max_dot = -FLT_MAX;

	for (u32 i = 0; i < array_length(edges1); i += 2) {
		vec3 e1_last = edges1[i];
		vec3 e1_current = edges1[i + 1];
		vec3 edge1 = gm_vec3_subtract(e1_current, e1_last);
		for (u32 j = 0; j < array_length(edges2); j += 2) {
			vec3 e2_last = edges2[j];
			vec3 e2_current = edges2[j + 1];
			vec3 edge2 = gm_vec3_subtract(e2_current, e2_last);

			vec3 current_normal = gm_vec3_normalize(gm_vec3_cross(edge1, edge2));

			r32 dot = gm_vec3_dot(current_normal, normal);
			if (dot > max_dot) {
				max_dot = dot;
				array_clear(result);
				array_push(result, e1_last);
				array_push(result, e1_current);
				array_push(result, e2_last);
				array_push(result, e2_current);
				*edge_normal = current_normal;
			}
		}
	}

	return result;
}

// Solve 2x2 linear system
// a1*x + b1*y = c1
// a2*x + b2*y = c2
// Outputs: x and y
static boolean
solve_2x2_linear_system(r32 a1, r32 b1, r32 c1, r32 a2, r32 b2, r32 c2, r32* x, r32* y)
{
  if ((a1 * b2) - (a2 * b1) == 0)
    return false;

  if (x)
    *x = ((c1 * b2) - (c2 * b1)) / ((a1 * b2) - (a2 * b1));

  if (y)
    *y = ((a1 * c2) - (a2 * c1)) / ((a1 * b2) - (a2 * b1));

  return true;
}

// This function calculates the distance between two indepedent skew lines in the 3D world
// The first line is given by a known point P1 and a direction vector D1
// The second line is given by a known point P2 and a direction vector D2
// Outputs:
// L1 is the closest POINT to the second line that belongs to the first line
// L2 is the closest POINT to the first line that belongs to the second line
// _N is the number that satisfies L1 = P1 + _N * D1
// _M is the number that satisfies L2 = P2 + _M * D2
boolean
collision_distance_between_skew_lines(vec3 p1, vec3 d1, vec3 p2, vec3 d2, vec3 * l1, vec3 * l2, r32 * _n, r32 * _m)
{
  r32 n1 = d1.x * d2.x + d1.y * d2.y + d1.z * d2.z;
  r32 n2 = d2.x * d2.x + d2.y * d2.y + d2.z * d2.z;
  r32 m1 = -d1.x * d1.x - d1.y * d1.y - d1.z * d1.z;
  r32 m2 = -d2.x * d1.x - d2.y * d1.y - d2.z * d1.z;
  r32 r1 = -d1.x * p2.x + d1.x * p1.x - d1.y * p2.y + d1.y * p1.y - d1.z * p2.z + d1.z * p1.z;
  r32 r2 = -d2.x * p2.x + d2.x * p1.x - d2.y * p2.y + d2.y * p1.y - d2.z * p2.z + d2.z * p1.z;

  r32 m, n;
  if (!solve_2x2_linear_system(n1, m1, r1, n2, m2, r2, &n, &m))
    return false;

  if (l1)
    *l1 = gm_vec3_add(p1, gm_vec3_scalar_product(m, d1));
  if (l2)
    *l2 = gm_vec3_add(p2, gm_vec3_scalar_product(n, d2));
  if (_n)
    *_n = n;
  if (_m)
    *_m = m;

  return true;
}
extern int paused;
Persistent_Manifold create_persistent_manifold(Bounding_Shape* b1, Bounding_Shape* b2, vec3 normal, Mesh* m1, Mesh* m2) {
	Persistent_Manifold pm;
	pm.normal = normal;
	pm.collision_points1 = array_new(vec3);
	pm.collision_points2 = array_new(vec3);

	Persistent_Manifold_Point* persistent_manifold_points = array_new(Persistent_Manifold_Point);

	vec3 inverted_normal = gm_vec3_scalar_product(-1.0f, normal);

	vec3 chosen_normal1, chosen_normal2, edge_normal;
	vec3* support_points1 = get_face_with_most_fitting_normal_in_winding_order(b1, normal, m1, &chosen_normal1);
	vec3* support_points2 = get_face_with_most_fitting_normal_in_winding_order(b2, inverted_normal, m2, &chosen_normal2);
	vec3* selected_edges = get_edge_with_most_fitting_normal(b1, b2, m1, m2, normal, &edge_normal);

	r32 chosen_normal1_dot = gm_vec3_dot(chosen_normal1, normal);
	r32 chosen_normal2_dot = gm_vec3_dot(chosen_normal2, inverted_normal);
	r32 edge_normal_dot = gm_vec3_dot(edge_normal, normal);

	r32 EPSILON = 0.0001f;
	//if (edge_normal_dot > chosen_normal1_dot && edge_normal_dot > chosen_normal2_dot) {
	if (edge_normal_dot > chosen_normal1_dot + EPSILON && edge_normal_dot > chosen_normal2_dot + EPSILON) {
		printf("edge contact\n");
		Persistent_Manifold_Point pmp;
		vec3 l1, l2;
		vec3 p1 = selected_edges[0];
		vec3 d1 = gm_vec3_subtract(selected_edges[1], selected_edges[0]);
		vec3 p2 = selected_edges[2];
		vec3 d2 = gm_vec3_subtract(selected_edges[3], selected_edges[2]);
		assert(collision_distance_between_skew_lines(p1, d1, p2, d2, &l1, &l2, 0, 0));
		//pmp.wc1 = p1;
		//pmp.wc2 = gm_vec3_add(d1, p1);
		//pmp.wc1 = p2;
		//pmp.wc2 = gm_vec3_add(d2, p2);
		pmp.wc1 = l1;
		pmp.wc2 = l2;
		array_push(persistent_manifold_points, pmp);
	} else {
		printf("face contact\n");
		boolean is_normal1_more_parallel = chosen_normal1_dot > chosen_normal2_dot;
		vec3* reference_face_support_points = is_normal1_more_parallel ? support_points1 : support_points2;
		vec3* incident_face_support_points = is_normal1_more_parallel ? support_points2 : support_points1;

		Plane* boundary_planes = is_normal1_more_parallel ? find_boundary_planes(b1, m1, support_points1) :
			find_boundary_planes(b2, m2, support_points2);

		vec3* out_polygon;
		SutherlandHodgmanClipping(incident_face_support_points, array_length(boundary_planes), boundary_planes, &out_polygon, false);

		Plane reference_plane;
		reference_plane.normal = is_normal1_more_parallel ? gm_vec3_scalar_product(-1.0f, chosen_normal1) :
			gm_vec3_scalar_product(-1.0f, chosen_normal2);
		reference_plane.point = reference_face_support_points[0];

		SutherlandHodgmanClipping(out_polygon, 1, &reference_plane, &out_polygon, true);

		for (u32 i = 0; i < array_length(out_polygon); ++i) {
			vec3 point = out_polygon[i];
			vec3 closest_point = GetClosestPointPolygon(point, reference_face_support_points);
			vec3 point_diff = gm_vec3_subtract(point, closest_point);
			r32 contact_penetration;

			// we are projecting the points that are in the incident face on the reference planes
			// so the points that we have are part of the incident object.
			Persistent_Manifold_Point pmp;
			if (is_normal1_more_parallel) {
				contact_penetration = gm_vec3_dot(point_diff, normal);
				pmp.wc1 = gm_vec3_subtract(point, gm_vec3_scalar_product(contact_penetration, normal));
				pmp.wc2 = point;
			} else {
				contact_penetration = - gm_vec3_dot(point_diff, normal);
				pmp.wc1 = point;
				pmp.wc2 = gm_vec3_add(point, gm_vec3_scalar_product(contact_penetration, normal));
			}

			if (contact_penetration < 0.0f) {
				array_push(persistent_manifold_points, pmp);
			} else {
				printf("not adding\n");
			}
		}
	}
	if (tmp) {
		persistent_manifold_points = array_new(Persistent_Manifold_Point);
		//for (u32 i = 0; i < array_length(support_points1); ++i) {
		//	Persistent_Manifold_Point pmp;
		//	pmp.pv2 = (vec2){0.0f, 0.0f};
		//	pmp.wc1 = support_points1[i];
		//	pmp.wc2 = (vec3){-9999.0f, -9999.0f, -9999.0f};
		//	array_push(persistent_manifold_points, pmp);
		//}

		//for (u32 i = 0; i < array_length(support_points2); ++i) {
		//	Persistent_Manifold_Point pmp;
		//	pmp.pv2 = (vec2){0.0f, 0.0f};
		//	pmp.wc1 = (vec3){-9999.0f, -9999.0f, -9999.0f};
		//	pmp.wc2 = support_points2[i];
		//	array_push(persistent_manifold_points, pmp);
		//}

#if 0
		for (u32 i = 0; i < array_length(reference_face_support_points); ++i) {
			Persistent_Manifold_Point pmp;
			pmp.pv2 = (vec2){0.0f, 0.0f};
			pmp.wc1 = (vec3){-9999.0f, -9999.0f, -9999.0f};
			pmp.wc2 = reference_face_support_points[i];
			array_push(persistent_manifold_points, pmp);
		}
#else
		//for (u32 i = 0; i < array_length(out_polygon); ++i) {
		//	Persistent_Manifold_Point pmp;
		//	pmp.pv2 = (vec2){0.0f, 0.0f};
		//	pmp.wc1 = (vec3){-9999.0f, -9999.0f, -9999.0f};
		//	pmp.wc2 = out_polygon[i];
		//	array_push(persistent_manifold_points, pmp);
		//}
#endif
	} else {
		// Clear duplicated points
		//const r32 EPSILON = 0.000001f;
		//for (u32 i = 0; i < array_length(persistent_manifold_points); ++i) {
		//	Persistent_Manifold_Point* pmp1 = &persistent_manifold_points[i];
		//	for (u32 j = i + 1; j < array_length(persistent_manifold_points); ++j) {
		//		Persistent_Manifold_Point* pmp2 = &persistent_manifold_points[j];

		//		if (((pmp1->pv2.x - pmp2->pv2.x) > -EPSILON && (pmp1->pv2.x - pmp2->pv2.x) < EPSILON) &&
		//			((pmp1->pv2.y - pmp2->pv2.y) > -EPSILON && (pmp1->pv2.y - pmp2->pv2.y) < EPSILON)) {
		//			array_remove(persistent_manifold_points, j);
		//			--j;
		//		}
		//	}
		//}
	}

	if (array_length(persistent_manifold_points) == 0) {
		// We didn't find any intersection :(
		// in this case, we fall back to simply getting the center of the convex hull
		printf("Warning: no intersection was found - falling back to center of convex hull (inaccurate)\n");
		//vec2 center1 = get_center_of_polygon(polygon1);
		//vec2 center2 = get_center_of_polygon(polygon2);
		//Persistent_Manifold_Point pmp;
		//assert(find_world_coords_for_vertex_within_polygon(center1, polygon1, &pmp.wc1) == true);
		//assert(find_world_coords_for_vertex_within_polygon(center2, polygon2, &pmp.wc2) == true);
		//pmp.pv2 = (vec2){0.0f, 0.0f};
		//array_push(persistent_manifold_points, pmp);
	}

	if (array_length(persistent_manifold_points) > 4) {
		printf("maior q 4\n");
	}

	for (u32 i = 0; i < array_length(persistent_manifold_points); ++i) {
		array_push(pm.collision_points1, persistent_manifold_points[i].wc1);
	}

	for (u32 i = 0; i < array_length(persistent_manifold_points); ++i) {
		array_push(pm.collision_points2, persistent_manifold_points[i].wc2);
	}

	return pm;
}

PMC_Contact* collision_get_convex_convex_points(Entity* e1, Entity* e2, vec3 normal) {
	PMC_Contact* result = array_new(PMC_Contact);
	Persistent_Manifold pm = create_persistent_manifold(&e1->bs, &e2->bs, normal, &e1->mesh, &e2->mesh);

	assert(array_length(pm.collision_points1) == array_length(pm.collision_points2));

	for (u32 i = 0; i < array_length(pm.collision_points1); ++i) {
		vec3 cp1 = pm.collision_points1[i];
		vec3 cp2 = pm.collision_points2[i];

		PMC_Contact pmc_contact;
		pmc_contact.e1 = e1;
		pmc_contact.e2 = e2;
		pmc_contact.normal = normal;
		pmc_contact.lambda_n = 0.0f;
		pmc_contact.lambda_t = 0.0f;
		pmc_contact.r1_wc = gm_vec3_subtract(cp1, e1->world_position);
		pmc_contact.r2_wc = gm_vec3_subtract(cp2, e2->world_position);
		Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
		mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
		pmc_contact.r1_lc = gm_mat3_multiply_vec3(&q1_mat, pmc_contact.r1_wc);

		Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
		mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
		pmc_contact.r2_lc = gm_mat3_multiply_vec3(&q2_mat, pmc_contact.r2_wc);

		array_push(result, pmc_contact);
	}

	return result;
}