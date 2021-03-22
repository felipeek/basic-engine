#include "collision.h"
#include <light_array.h>
#include <math.h>
#include <assert.h>

Quaternion transform_quaternion(Quaternion q) {
	vec3 non_unit_quat_vec_part = gm_vec3_scalar_product(tanf(q.w), (vec3){q.x, q.y, q.z});
	return (Quaternion) {non_unit_quat_vec_part.x, non_unit_quat_vec_part.y, non_unit_quat_vec_part.z, 1.0f};
}

Collision_Point* collision_get(Entity* se, Entity* me) {
	Quaternion inverse_last_rotation = quaternion_inverse(&me->last_rotation);
	Quaternion me_quaternion_diff = quaternion_product(&me->world_rotation, &inverse_last_rotation);
	vec3 me_position_diff = gm_vec3_subtract(gm_vec4_to_vec3(me->world_position), gm_vec4_to_vec3(me->last_position));

	// transform quaternion
	me_quaternion_diff = transform_quaternion(me_quaternion_diff);

	Quaternion qf = me_quaternion_diff;
	vec3 xf = me_position_diff;

	for (u32 i = 0; i < array_length(me->mesh.indices); i += 3) {
		vec3 object_v1 = gm_vec4_to_vec3(me->mesh.vertices[me->mesh.indices[i]].position);
		vec3 object_v2 = gm_vec4_to_vec3(me->mesh.vertices[me->mesh.indices[i + 1]].position);
		vec3 object_v3 = gm_vec4_to_vec3(me->mesh.vertices[me->mesh.indices[i + 2]].position);

		vec3 normal = gm_vec3_normalize(gm_vec3_cross(gm_vec3_subtract(object_v2, object_v1), gm_vec3_subtract(object_v3, object_v1)));
		r32 dA = gm_vec3_dot(normal, object_v1);
		//r32 dA_1 = gm_vec3_dot(normal, v2);
		//r32 dA_2 = gm_vec3_dot(normal, v3);

		Quaternion FA = (Quaternion){normal.x, normal.y, normal.z, dA};

		for (u32 j = 0; j < array_length(se->mesh.vertices); ++j) {
			// obstacle vertex
			vec3 pB = gm_vec4_to_vec3(se->mesh.vertices[j].position);


		}
	}

}

#if 0
typedef struct {
	u32 c, k;
	r32* q;
} Working_List_Element;

typedef struct {
	u32 c, k, h;
} Interval;

static r32 evaluate_polynomial_at(r32* polynomial_coefficients, r32 x) {
	r32 result = 0.0;

	for (u32 i = 0; i < array_length(polynomial_coefficients); ++i) {
		result += polynomial_coefficients[i] * powf(x, i);
	}

	return result;
}

static r32* poly_transform_1(r32* q, u32 degree) {
	r32* result = array_copy(q);

	switch (degree) {
		case 3: {
			result[0] = q[3] + q[2] + q[1] + q[0];
			result[1] = q[2] + 2.0f * q[1] + 3.0f * q[0];
			result[2] = q[1] + 3.0f * q[0];
			result[3] = q[0];
		} break;
		case 2: {
			result[0] = q[2] + q[1] + q[0];
			result[1] = q[1] + 2.0f * q[0];
			result[2] = q[0];
		} break;
		case 1: {
			result[0] = q[1] + q[0];
			result[1] = q[0];
		} break;
		default: assert(0);
	}

	return result;
}

static r32* poly_transform_2(r32* q, u32 degree) {
	r32* result = array_copy(q);

	switch (degree) {
		case 3: {
			result[0] = 8.0f * q[0];
			result[1] = 4.0f * q[1];
			result[2] = 2.0f * q[2];
			result[3] = q[3];
		} break;
		case 2: {
			result[0] = 4.0f * q[0];
			result[1] = 2.0f * q[1];
			result[2] = q[2];
		} break;
		case 1: {
			result[0] = 2.0f * q[0];
			result[1] = q[1];
		} break;
		default: assert(0);
	}

	return result;
}

static r32* poly_transform_3(r32* q, u32 degree) {
	r32* result = array_copy(q);

	switch (degree) {
		case 3: {
			result[0] =	q[3] + 2.0f * q[2] + 4.0f * q[1] + 8.0f * q[0];
			result[1] = 3.0f * q[3] + 4.0f * q[2] + 4.0f * q[1];
			result[2] = 3.0f * q[3] + 2.0f * q[2];
			result[3] = q[3];
		} break;
		case 2: {
			result[0] = q[3] + 2.0f * q[2] + 4.0f * q[0];
			result[1] = 2.0f * q[3] + 2.0f * q[2];
			result[2] = q[3];
		} break;
		case 1: {
			result[0] = q[1] + 2.0f * q[0];
			result[1] = q[1];
		} break;
		default: assert(0);
	}

	return result;
}

static u32 var(r32* q, u32 degree) {
	u32 sign_changes = 0;
	boolean last_sign = q[degree] > 0;
	for (s32 i = (s32)degree - 1; i >= 0; --i) {
		if (q[i] != 0.0f) {
			boolean current_sign = q[i] > 0;
			if (current_sign != last_sign) {
				++sign_changes;
				last_sign = current_sign;
			}
		}
	}
	return sign_changes;
}

void bisection(r32* polynomial_coefficients) {
	Working_List_Element* L = array_new(Working_List_Element);
	Working_List_Element wle = (Working_List_Element){0u, 0u, polynomial_coefficients};
	array_push(L, wle);

	Interval* isol = array_new(Interval);
	r32 n = array_length(polynomial_coefficients) - 1;

	for (u32 i = 0; i < array_length(L); ++i) {
		Working_List_Element* wle = &L[i];
		if (wle->q[0] == 0.0f) {
			array_remove_ordered(wle->q, 0);
			--n;
			Interval i = (Interval){wle->c, wle->k, 0u};
			array_push(isol, i);
		}	

		r32* q1 = poly_transform_1(wle->q, n);
		u32 v = var(q1, n);

		if (v == 1) {
			Interval i = (Interval){wle->c, wle->k, 1u};
			array_push(isol, i);
		} else if (v > 1) {
			Working_List_Element new_wle = (Working_List_Element){2u * wle->c, wle->k + 1u, poly_transform_2(wle->q, n)};
			array_push(L, new_wle);
			new_wle = (Working_List_Element){2u * wle->c + 1u, wle->k + 1u, poly_transform_3(wle->q, n)};
			array_push(L, new_wle);
		}
	}
}
#endif