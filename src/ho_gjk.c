#include "ho_gjk.h"
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
collision_gjk_support(vec3* b1, vec3* b2, vec3 direction)
{
  float max = -FLT_MAX;
  int index = 0;
  for (int i = 0; i < array_length(b1); ++i)
  {
	float dot = gm_vec3_dot(b1[i], direction);
	if (dot > max)
	{
	  max = dot;
	  index = i;
	}
  }
  int b1_index = index;

  max = -FLT_MAX;
  index = 0;
  for (int i = 0; i < array_length(b2); ++i)
  {
	float dot = gm_vec3_dot(b2[i], gm_vec3_subtract((vec3) { 0.0f, 0.0f, 0.0f }, direction));
	if (dot > max)
	{
	  max = dot;
	  index = i;
	}
  }
  int b2_index = index;

  Support_Point sup_point = {
	.v = gm_vec3_subtract(b1[b1_index], b2[b2_index]),
	.sup = b1[b1_index],
	.sup2 = b2[b2_index]
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
ho_gjk_collides(GJK_Support_List* sup_list, vec3* b1, vec3* b2)
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