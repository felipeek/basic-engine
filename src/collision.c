#include "collision.h"
#include <light_array.h>
#include <float.h>

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

Collision_Point
collision_epa(Support_Point* simplex, Bounding_Shape* b1, Bounding_Shape* b2)
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
	  //assert(0);
	  //return (vec3) {0.0f, 0.0f, 0.0f};

	  // @TODO: fix this
	  Collision_Point cp;
	  cp.collision_point = (vec3){0.0f, 0.0f, 0.0f};
	  cp.normal = (vec3){0.0f, 1.0f, 0.0};
	  cp.penetration = 0.0f;
	  return cp;
	}
	// Find the new support in the normal direction of the closest face
	Support_Point sup_p = collision_gjk_support(b1, b2, faces[index].normal);
	vec3 p = sup_p.v;

	if (gm_vec3_dot(p, faces[index].normal) - faces[index].distance < 0.1f)
	{
	  if (gm_vec3_equal(faces[index].normal, (vec3) {0.0f, 0.0f, 0.0f}))
	  {
		assert(0);
	  }
	  float bary_u,bary_v,bary_w;
	  barycentric(gm_vec3_scalar_product(faces[index].distance, faces[index].normal),
		faces[index].a.v, faces[index].b.v, faces[index].c.v, &bary_u, &bary_v, &bary_w);

	  // collision point on object a in world space

	  vec3 wcolpoint = 
		gm_vec3_add(
		  gm_vec3_add(gm_vec3_scalar_product(bary_u, faces[index].a.sup), gm_vec3_scalar_product(bary_v, faces[index].b.sup)),
		  gm_vec3_scalar_product(bary_w, faces[index].c.sup)
		);

	  vec3 penetration = gm_vec3_scalar_product(faces[index].distance, gm_vec3_normalize(faces[index].normal));
	  array_free(faces);

	  Collision_Point cp;
	  cp.collision_point = wcolpoint;
	  cp.normal = gm_vec3_scalar_product(-1.0f, penetration);
	  cp.penetration = gm_vec3_length(penetration);
	  return cp;
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
  return (Collision_Point){0};
}