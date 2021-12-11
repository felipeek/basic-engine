#include "collision.h"
#include <light_array.h>
#include <float.h>
#include "jarvis_march.h"
#include "convex_bary_coords.h"
#include <sys/time.h>

Collision_Point* collision_get_plane_cube_points(Entity* cube, r32 plane_y) {
	Collision_Point* collision_points = array_new(Collision_Point);
	for (u32 i = 0; i < array_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		vec4 pos_wc = gm_mat4_multiply_vec4(&cube->model_matrix, v->position);
		if (pos_wc.y <= plane_y) {
			Collision_Point cp;
			cp.collision_point = gm_vec4_to_vec3(pos_wc);
			cp.normal = (vec3){0.0f, 1.0f, 0.0f};
			cp.penetration = plane_y - pos_wc.y;
			cp.vertex_index = i;
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

// GJK

static void
support_list_add(GJK_Support_List* list, Support_Point point)
{
  list->simplex[list->current_index] = point;
  list->current_index++;
}

Support_Point
collision_gjk_support(Bounding_Shape* b1, Bounding_Shape* b2, vec3 direction)
{
  float max = -FLT_MAX;
  int index = 0;
  for (int i = 0; i < b1->vertex_count; ++i)
  {
	float dot = gm_vec3_dot(b1->vertices[i], direction);
	if (dot > max)
	{
	  max = dot;
	  index = i;
	}
  }
  int b1_index = index;

  max = -FLT_MAX;
  index = 0;
  for (int i = 0; i < b2->vertex_count; ++i)
  {
	float dot = gm_vec3_dot(b2->vertices[i], gm_vec3_subtract((vec3) { 0.0f, 0.0f, 0.0f }, direction));
	if (dot > max)
	{
	  max = dot;
	  index = i;
	}
  }
  int b2_index = index;

  Support_Point sup_point = {
	.v = gm_vec3_subtract(b1->vertices[b1_index], b2->vertices[b2_index]),
	.sup = b1->vertices[b1_index]
  };
  return sup_point;
}

#define GJK_3D
boolean
gjk_simplex(GJK_Support_List* support_list, vec3* direction)
{
  int num_entries = support_list->current_index;

  if (num_entries == 2)
  {
	Support_Point A = support_list->simplex[1];
	Support_Point B = support_list->simplex[0];
	vec3 AO = gm_vec3_negative(A.v);      //(0,0,0) - A
	vec3 AB = gm_vec3_subtract(B.v, A.v);
	if (gm_vec3_dot(AB, AO) > 0)
	{
	  *direction = gm_vec3_cross(gm_vec3_cross(AB, AO), AB);
	}
	else
	{
	  support_list->simplex[0] = support_list->simplex[1];
	  support_list->current_index--;
	  *direction = AO;
	}
  }
  else if (num_entries == 3)
  {
	Support_Point B = support_list->simplex[0];
	Support_Point C = support_list->simplex[1];
	Support_Point A = support_list->simplex[2];

	vec3 AB = gm_vec3_subtract(B.v, A.v);
	vec3 AC = gm_vec3_subtract(C.v, A.v);
	vec3 ABC = gm_vec3_cross(AB, AC);   // normal to the triangle
	vec3 AO = gm_vec3_negative(A.v);

	vec3 edge4 = gm_vec3_cross(AB, ABC);
	vec3 edge1 = gm_vec3_cross(ABC, AC);

	if (gm_vec3_dot(edge1, AO) > 0)
	{
	  if (gm_vec3_dot(AC, AO) > 0)
	  {
		// simplex is A, C

		support_list->simplex[0] = A;
		support_list->current_index--;
		*direction = gm_vec3_cross(gm_vec3_cross(AC, AO), AC);
	  }
	  else
	  {
		if (gm_vec3_dot(AB, AO) > 0)
		{
		  // simplex is A, B

		  support_list->simplex[0] = A;
		  support_list->simplex[1] = B;
		  support_list->current_index--;
		  *direction = gm_vec3_cross(gm_vec3_cross(AB, AO), AB);
		}
		else
		{
		  // simplex is A

		  support_list->simplex[0] = A;
		  support_list->current_index -= 2;
		  *direction = AO;
		}
	  }
	}
	else
	{
	  if (gm_vec3_dot(edge4, AO) > 0)
	  {
		if (gm_vec3_dot(AB, AO) > 0)
		{
		  // simplex is A, B

		  support_list->simplex[0] = A;
		  support_list->simplex[1] = B;
		  support_list->current_index--;
		  *direction = gm_vec3_cross(gm_vec3_cross(AB, AO), AB);
		}
		else
		{
		  // simplex is A

		  support_list->simplex[0] = A;
		  support_list->current_index -= 2;
		  *direction = AO;
		}
	  }
	  else
	  {
		// for 2D this is enough
#ifndef GJK_3D
		return true;
#else
		if (gm_vec3_dot(ABC, AO) > 0)
		{
		  // simplex is A, B, C

		  support_list->simplex[0] = A;
		  support_list->simplex[1] = B;
		  support_list->simplex[2] = C;
		  *direction = ABC;
		}
		else
		{
		  // simplex is A, C, B

		  support_list->simplex[0] = A;
		  support_list->simplex[1] = C;
		  support_list->simplex[2] = B;
		  *direction = gm_vec3_negative(ABC);
		}
#endif
	  }
	}
  }
#ifdef GJK_3D
  else if (num_entries == 4)
  {
	Support_Point A = support_list->simplex[3];
	Support_Point B = support_list->simplex[2];
	Support_Point C = support_list->simplex[1];
	Support_Point D = support_list->simplex[0];

	vec3 AO = gm_vec3_negative(A.v);
	vec3 AB = gm_vec3_subtract(B.v, A.v);
	vec3 AC = gm_vec3_subtract(C.v, A.v);
	vec3 AD = gm_vec3_subtract(D.v, A.v);

	vec3 ABC = gm_vec3_cross(AB, AC);

	if (gm_vec3_dot(ABC, AO) > 0)
	{
	  // in front of ABC

	  support_list->simplex[0] = C;
	  support_list->simplex[1] = B;
	  support_list->simplex[2] = A;
	  support_list->current_index--;
	  *direction = ABC;
	  return false;
	}

	vec3 ADB = gm_vec3_cross(AD, AB);
	if (gm_vec3_dot(ADB, AO) > 0)
	{
	  // in front of ADB

	  support_list->simplex[0] = B;
	  support_list->simplex[1] = D;
	  support_list->simplex[2] = A;
	  support_list->current_index--;
	  *direction = ADB;
	  return false;
	}

	vec3 ACD = gm_vec3_cross(AC, AD);
	if (gm_vec3_dot(ACD, AO) > 0)
	{
	  // in front of ACD

	  support_list->simplex[0] = D;
	  support_list->simplex[1] = C;
	  support_list->simplex[2] = A;
	  support_list->current_index--;
	  *direction = ACD;
	  return false;
	}

	return true;                // inside the tetrahedron
  }
#endif
  return false;
}

boolean
collision_gjk_collides(GJK_Support_List* sup_list, Bounding_Shape* b1, Bounding_Shape* b2)
{
  vec3 search_direction = (vec3) { 1.0f, 0.0f, 0.0f };
  Support_Point support_point = collision_gjk_support(b1, b2, search_direction);
  vec3 support = support_point.v;

  support_list_add(sup_list, support_point);

  vec3 opposite_direction = gm_vec3_negative(search_direction);

  int max = 200;
  while (true)
  {
	if (max <= 0)
	  return true;
	max--;
	Support_Point sup_pt_a = collision_gjk_support(b1, b2, opposite_direction);
	vec3 a = sup_pt_a.v;

	float dotval = gm_vec3_dot(sup_pt_a.v, opposite_direction);
	if (dotval < 0)
	{
	  return false;             // there is no intersection
	}

	support_list_add(sup_list, sup_pt_a);
	if (gjk_simplex(sup_list, &opposite_direction))
	{
	  return true;
	}
  }
}

Bounding_Shape
collision_bounding_shape_new(vec3 size)
{
  Bounding_Shape bs = { 0 };

  bs.vertex_count = 8;
  bs.vertices = calloc(1, sizeof(vec3) * bs.vertex_count);

  vec3 half_size = gm_vec3_scalar_product(0.5f, size);

  bs.vertices[0] = (vec3) {-half_size.x, -half_size.y, -half_size.z};
  bs.vertices[1] = (vec3) {half_size.x, -half_size.y, -half_size.z};
  bs.vertices[2] = (vec3) {-half_size.x, half_size.y, -half_size.z};
  bs.vertices[3] = (vec3) {half_size.x, -half_size.y, -half_size.z};

  bs.vertices[4] = (vec3) {-half_size.x, -half_size.y, half_size.z};
  bs.vertices[5] = (vec3) {half_size.x, -half_size.y, half_size.z};
  bs.vertices[6] = (vec3) {-half_size.x, half_size.y, half_size.z};
  bs.vertices[7] = (vec3) {half_size.x, -half_size.y, half_size.z};

  return bs;
}


// EPA algorithm

typedef struct {
  Support_Point a;
  Support_Point b;
} Edge;

typedef struct {
  Support_Point a;
  Support_Point b;
  Support_Point c;
  vec3 normal;
  r32 distance;
} Face;

vec3
triangle_centroid(Face f)
{
  r32 ox = (f.a.v.x + f.b.v.x + f.c.v.x) / 3.0f;
  r32 oy = (f.a.v.y + f.b.v.y + f.c.v.y) / 3.0f;
  r32 oz = (f.a.v.z + f.b.v.z + f.c.v.z) / 3.0f;
  return (vec3) {ox, oy, oz};
}

static Face
face_new(Support_Point v1, Support_Point v2, Support_Point v3)
{
  Face f = { 0 };
  f.a = v1;
  f.b = v2;
  f.c = v3;
  return f;
}

static int
closest_face(Face* faces, Bounding_Shape* b1, Bounding_Shape* b2, boolean invert_normals)
{
  r32 distance = FLT_MAX;
  int index = -1;
  for (int i = 0; i < array_length(faces); ++i)
  {
	Face f = faces[i];
	vec3 ba = gm_vec3_subtract(f.b.v, f.a.v);
	vec3 ca = gm_vec3_subtract(f.c.v, f.a.v);
	vec3 normal = gm_vec3_cross(ba, ca);
	normal = gm_vec3_normalize(normal);

	if (invert_normals && gm_vec3_dot(normal, f.a.v) < 0.0f)
	{
	  normal = gm_vec3_negative(normal);
	}

	r32 d = fabsf(gm_vec3_dot(f.a.v, normal));

	if (invert_normals && d == 0.0f)
	{
	  d = FLT_MAX;
	}

	faces[i].distance = d;
	faces[i].normal = normal;
	//faces[i].support = collision_gjk_support(b1, b2, normal);

	if (d < distance)
	{
	  distance = d;
	  index = i;
	}
  }

  return index;
}


static int
edge_in(Edge* edges, Edge e)
{
  for (int i = 0; i < array_length(edges); ++i)
  {
	Edge t = edges[i];

	if (gm_vec3_equal(t.a.v, e.a.v) && gm_vec3_equal(t.b.v, e.b.v))
	  return i;
	if (gm_vec3_equal(t.b.v, e.a.v) && gm_vec3_equal(t.a.v, e.b.v))
	  return i;
  }
  return -1;
}

void
barycentric(vec3 p, vec3 a, vec3 b, vec3 c, r32 *u, r32 *v, r32 *w)
{
  // code from Crister Erickson's Real-Time Collision Detection
  vec3 v0 = gm_vec3_subtract(b, a);
  vec3 v1 = gm_vec3_subtract(c, a);
  vec3 v2 = gm_vec3_subtract(p, a);
  r32 d00 = gm_vec3_dot(v0, v0);
  r32 d01 = gm_vec3_dot(v0, v1);
  r32 d11 = gm_vec3_dot(v1, v1);
  r32 d20 = gm_vec3_dot(v2, v0);
  r32 d21 = gm_vec3_dot(v2, v1);
  r32 denom = d00 * d11 - d01 * d01;
  *v = (d11 * d20 - d01 * d21) / denom;
  *w = (d00 * d21 - d01 * d20) / denom;
  *u = 1.0f - *v - *w;
}

static vec3 find_ortho(vec3 v) {
	// tricky problem: https://math.stackexchange.com/questions/137362/how-to-find-perpendicular-vector-to-another-vector

	r32 v1 = v.z * v.z + v.y * v.y;
	r32 v2 = v.z * v.z + v.x * v.x;
	r32 v3 = v.y * v.y + v.x * v.x;

	if (v1 > v2 && v1 > v3) {
		return gm_vec3_normalize((vec3){0.0f, v.z, -v.y});
	} else if (v2 > v3) {
		return gm_vec3_normalize((vec3){-v.z, 0.0f, v.x});
	} else {
		return gm_vec3_normalize((vec3){-v.y, v.x, 0.0f});
	}
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


#define max(a,b)    (((a) > (b)) ? (a) : (b))
#define min(a,b)    (((a) < (b)) ? (a) : (b))
int tmp = 0;

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

	for (u32 i = 0; i < array_length(persistent_manifold_points); ++i) {
		array_push(pm.collision_points1, persistent_manifold_points[i].wc1);
	}

	for (u32 i = 0; i < array_length(persistent_manifold_points); ++i) {
		array_push(pm.collision_points2, persistent_manifold_points[i].wc2);
	}

	return pm;
}

Persistent_Manifold collision_epa(Support_Point* simplex, Bounding_Shape* b1, Bounding_Shape* b2, Mesh* m1, Mesh* m2)
{
  int index = -1;
  Face *faces = array_new_len(Face, 4);
  array_length(faces) = 4;
  faces[0] = face_new(simplex[0], simplex[1], simplex[2]);
  faces[1] = face_new(simplex[0], simplex[3], simplex[1]);
  faces[2] = face_new(simplex[1], simplex[3], simplex[2]);
  faces[3] = face_new(simplex[0], simplex[2], simplex[3]);

  int kk = 0;
  for (; kk < 128; kk++)
  {
	// closest face is the face at index
	index = closest_face(faces, b1, b2, kk > 0);

	// Whenever the distance is 0, 
	if (faces[index].distance == 0.0f)
	{
	  array_free(faces);
	  Persistent_Manifold pm;
	  pm.normal = (vec3) {0.0f, 0.0f, 0.0f};
	  pm.collision_points1 = array_new(Persistent_Manifold);
	  pm.collision_points2 = array_new(Persistent_Manifold);
	  return pm;
	}
	// Find the new support in the normal direction of the closest face
	Support_Point sup_p = collision_gjk_support(b1, b2, faces[index].normal);
	vec3 p = sup_p.v;

	if (gm_vec3_dot(p, faces[index].normal) - faces[index].distance < 0.01f)
	{
	  if (gm_vec3_equal(faces[index].normal, (vec3) {0.0f, 0.0f, 0.0f}))
	  {
		assert(0);
	  }

	  array_free(faces);

	  Persistent_Manifold pm = create_persistent_manifold(b1, b2, gm_vec3_normalize(faces[index].normal), m1, m2);
	  return pm;
	}
	// Expand polytope
	Edge *edges = array_new_len(Edge, 16);
	for (int i = 0; i < array_length(faces); ++i)
	{
	  // If the face is facing in the direction of the support point
	  // it can be removed
	  vec3 centroid = triangle_centroid(faces[i]);
	  r32 r = gm_vec3_dot(faces[i].normal, gm_vec3_subtract(p, centroid));

	  if (r > 0.0f)
	  {
		Face f = faces[i];
		array_remove(faces, i);
		i--;

		Edge ea = { f.a, f.b };
		int ea_in = edge_in(edges, ea);
		if (ea_in != -1)
		{
		  array_remove(edges, ea_in);
		}
		else
		{
		  array_push(edges, ea);
		}

		Edge eb = { f.b, f.c };
		int eb_in = edge_in(edges, eb);
		if (eb_in != -1)
		{
		  array_remove(edges, eb_in);
		}
		else
		{
		  array_push(edges, eb);
		}

		Edge ec = { f.c, f.a };
		int ec_in = edge_in(edges, ec);
		if (ec_in != -1)
		{
		  array_remove(edges, ec_in);
		}
		else
		{
		  array_push(edges, ec);
		}
	  }
	}
	for (int i = 0; i < array_length(edges); ++i)
	{
	  Face f = face_new(edges[i].a, edges[i].b, sup_p);
	  array_push(faces, f);
	}
	array_free(edges);
  }

  array_free(faces);
  printf("The EPA routine took %d iterations and didn't complete", kk);
  assert(0); // this should be unreachable
  return (Persistent_Manifold){0};
}