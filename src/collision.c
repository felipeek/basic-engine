#include "collision.h"
#include <light_array.h>
#include <float.h>
#include "jarvis_march.h"

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

Support_Point
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

  Support_Point sup_point = {
	.v = b->vertices[index],
	.sup = b->vertices[index]
  };
  return sup_point;
}

vec3* get_support_points_with_perturbation(Bounding_Shape* b, vec3 direction) {
	vec3* result = array_new(vec3);

	Support_Point support = collision_gjk_individual_support(b, direction);
	array_push(result, support.v);

	const r32 PERTURBATION_STRENGTH = 0.05f;
	const int NUM_ITERATIONS = 16;
	r32 angle = 360.0f / NUM_ITERATIONS;
	vec3 ortho = gm_vec3_scalar_product(PERTURBATION_STRENGTH, find_ortho(direction));
	for (int i = 0; i < NUM_ITERATIONS; ++i) {
		Quaternion rotation_around_support_direction = quaternion_new(direction, angle * i);
		mat3 m = quaternion_get_matrix3(&rotation_around_support_direction);
		vec3 rotated_ortho = gm_mat3_multiply_vec3(&m, ortho);
		vec3 perturbed_support_direction1 = gm_vec3_add(direction, rotated_ortho);

		Support_Point support = collision_gjk_individual_support(b, perturbed_support_direction1);

		boolean duplicated = false;
		for (int j = 0; j < array_length(result); ++j) {
			vec3 current = result[j];
			if (support.v.x == current.x && support.v.y == current.y && support.v.z == current.z) {
				duplicated = true;
				break;
			}
		}

		if (!duplicated) {
			array_push(result, support.v);
		}
	}

	return result;
}

// @TODO This can be merged with the get_support_points_with_perturbation function. Keeping it separated to ease debugging
Projected_Support_Point* project_support_points_onto_normal_plane(vec3* support_points, vec3 normal) {
	Projected_Support_Point* projected_support_points = array_new(Projected_Support_Point);

	for (u32 i = 0; i < array_length(support_points); ++i) {
		Projected_Support_Point psp;
		psp.p = gm_vec3_subtract(support_points[i],
			gm_vec3_scalar_product(gm_vec3_dot(normal, support_points[i]), normal));
		psp.world_coords = support_points[i];
		vec3 e1 = find_ortho(normal);
		vec3 e2 = gm_vec3_cross(e1, normal);
		psp.pv2.x = gm_vec3_dot(e1, support_points[i]);
		psp.pv2.y = gm_vec3_dot(e2, support_points[i]);

		array_push(projected_support_points, psp);
	}

	return projected_support_points;
}

static boolean is_point_inside_polygon(Projected_Support_Point point_to_test, Projected_Support_Point* polygon, vec3 polygon_center, vec3 polygon_normal) {
	boolean inside_polygon = true;

	for (u32 j = 0; j < array_length(polygon); ++j) {
		Projected_Support_Point* current = &polygon[j];
		Projected_Support_Point* next = &polygon[(j == array_length(polygon) - 1) ? 0 : j + 1];
		vec3 e_i = gm_vec3_subtract(next->p, current->p);
		vec3 u_i = gm_vec3_cross(polygon_normal, e_i);
		vec3 v_i = gm_vec3_subtract(current->p, point_to_test.p);
		vec3 w_i = gm_vec3_subtract(current->p, polygon_center);
		r32 x_i = gm_vec3_dot(v_i, u_i) * gm_vec3_dot(w_i, u_i);
		if (x_i < 0) {
			return false;
		}
	}

	return true;
}

void clip_support_points(Projected_Support_Point* proj_support_points1, Projected_Support_Point* proj_support_points2,
	vec3 normal, Projected_Support_Point** _clipped_points1, Projected_Support_Point** _clipped_points2) {

	Projected_Support_Point* clipped_points1 = array_new(Projected_Support_Point);
	Projected_Support_Point* clipped_points2 = array_new(Projected_Support_Point);

	vec3 center1 = (vec3){0.0f, 0.0f, 0.0f};
	vec3 center2 = (vec3){0.0f, 0.0f, 0.0f};
	for (u32 i = 0; i < array_length(proj_support_points1); ++i) {
		center1 = gm_vec3_add(center1, proj_support_points1[i].p);
	}
	for (u32 i = 0; i < array_length(proj_support_points2); ++i) {
		center2 = gm_vec3_add(center2, proj_support_points2[i].p);
	}
	center1 = gm_vec3_scalar_product(1.0f / array_length(proj_support_points1), center1);
	center2 = gm_vec3_scalar_product(1.0f / array_length(proj_support_points2), center2);

	// check if points in proj_support_points1 are inside proj_support_points2
	for (u32 i = 0; i < array_length(proj_support_points1); ++i) {
		Projected_Support_Point point_to_test = proj_support_points1[i];
		if (is_point_inside_polygon(point_to_test, proj_support_points2, center2, normal)) {
			array_push(clipped_points1, point_to_test);
		}
	}

	// check if points in proj_support_points2 are inside proj_support_points1
	for (u32 i = 0; i < array_length(proj_support_points2); ++i) {
		Projected_Support_Point point_to_test = proj_support_points2[i];
		if (is_point_inside_polygon(point_to_test, proj_support_points1, center1, normal)) {
			array_push(clipped_points2, point_to_test);
		}
	}

	*_clipped_points1 = clipped_points1;
	*_clipped_points2 = clipped_points2;
}

Persistent_Manifold create_persistent_manifold(Bounding_Shape* b1, Bounding_Shape* b2, vec3 normal) {
	Persistent_Manifold pm;
	pm.normal = normal;
	pm.collision_points1 = array_new(vec3);
	pm.collision_points2 = array_new(vec3);

	vec3* support_points1 = get_support_points_with_perturbation(b1, normal);
	vec3* support_points2 = get_support_points_with_perturbation(b2, gm_vec3_scalar_product(-1.0f, normal));

	Projected_Support_Point* proj_support_points1 = project_support_points_onto_normal_plane(support_points1, normal);
	Projected_Support_Point* proj_support_points2 = project_support_points_onto_normal_plane(support_points2, normal);

	Projected_Support_Point* polygon1 = jarvis_march(proj_support_points1);
	Projected_Support_Point* polygon2 = jarvis_march(proj_support_points2);

	Projected_Support_Point* clipped_support_points1;
	Projected_Support_Point* clipped_support_points2;
	clip_support_points(polygon1, polygon2, normal, &clipped_support_points1, &clipped_support_points2);

	for (u32 i = 0; i < array_length(clipped_support_points1); ++i) {
		array_push(pm.collision_points1, clipped_support_points1[i].world_coords);
	}

	for (u32 i = 0; i < array_length(clipped_support_points2); ++i) {
		array_push(pm.collision_points2, clipped_support_points2[i].world_coords);
	}

	return pm;
}

Persistent_Manifold collision_epa(Support_Point* simplex, Bounding_Shape* b1, Bounding_Shape* b2)
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
	  return create_persistent_manifold(b1, b2, gm_vec3_normalize(faces[index].normal));
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