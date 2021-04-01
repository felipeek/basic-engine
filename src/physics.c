#include "physics.h"
#include "collision.h"
#include <light_array.h>
#include "matrix.h"
#include <float.h>

static mat3 physics_get_dynamic_inertia_tensor_inverse(Entity* e) {
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inverse_inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

static vec3 physics_get_linear_velocity(Entity* e) {
    return gm_vec3_scalar_product(1.0f / e->mass, e->linear_momentum);
}

static vec3 physics_get_angular_velocity(Entity* e, mat3* dynamic_inertia_tensor_inverse) {
    return gm_mat3_multiply_vec3(dynamic_inertia_tensor_inverse, e->angular_momentum);
}

static void physics_calculate_total_force_and_torque(Entity* e, vec3* total_force, vec3* total_torque) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    *total_force = (vec3){0.0f, 0.0f, 0.0f};
    *total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; i < array_length(e->forces); ++i) {
        vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
        *total_force = gm_vec3_add(*total_force, e->forces[i].force);
        *total_torque = gm_vec3_add(*total_torque, gm_vec3_cross(distance, e->forces[i].force));
    }
}

static void physics_update_momenta_based_on_forces(Entity* e, r32 dt, Physics_Force* forces) {
	vec3 total_force, total_torque;
	physics_calculate_total_force_and_torque(e, &total_force, &total_torque);
    vec3 angular_momentum_change = gm_vec3_scalar_product(dt, total_torque);
    vec3 linear_momentum_change = gm_vec3_scalar_product(dt, total_force);
    e->linear_momentum = gm_vec3_add(e->linear_momentum, linear_momentum_change);
    e->angular_momentum = gm_vec3_add(e->angular_momentum, angular_momentum_change);
}

extern vec3 col_point;
extern boolean collision;
extern vec3 penetration;

extern r32 static_friction_coefficient, dynamic_friction_coefficient;
static void apply_impulse(Entity* e1, Entity* e2, Collision_Point* cp, Physics_Force* forces, r32 restitution) {
    cp->normal = gm_vec3_normalize(cp->normal);

	mat3 e1_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(e1);
	vec3 e1_angular_velocity = physics_get_angular_velocity(e1, &e1_dynamic_inertia_tensor_inverse);
	vec3 e1_linear_velocity = physics_get_linear_velocity(e1);

	mat3 e2_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(e2);
	vec3 e2_angular_velocity = physics_get_angular_velocity(e2, &e2_dynamic_inertia_tensor_inverse);
	vec3 e2_linear_velocity = physics_get_linear_velocity(e2);

    /* inputs */
    vec3 vA = e1_linear_velocity;
    vec3 vB = e2_linear_velocity;
    vec3 wA = e1_angular_velocity;
    vec3 wB = e2_angular_velocity;
    vec3 rA = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(e1->world_position));
    vec3 rB = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(e2->world_position));
    vec3 N = gm_vec3_normalize(cp->normal);
    r32 mA = e1->mass;
    r32 mB = e2->mass;

    /* impulse algorithm */
    vec3 vP1 = gm_vec3_add(vA, gm_vec3_cross(wA, rA));
    vec3 vP2 = gm_vec3_add(vB, gm_vec3_cross(wB, rB));
    r32 fact = gm_vec3_dot(cp->normal, gm_vec3_subtract(vP2, vP1));
    if (fact < -0.01f) {
        // resting contact
        return;
    }
    vec3 vR = gm_vec3_subtract(vP2, vP1);
    r32 numerator = gm_vec3_dot(gm_vec3_scalar_product(-(1.0f + restitution), vR), N);
    vec3 rA_cross_N = gm_vec3_cross(rA, N);
    vec3 rB_cross_N = gm_vec3_cross(rB, N);
    vec3 IA_res = gm_mat3_multiply_vec3(&e1_dynamic_inertia_tensor_inverse, rA_cross_N);
    vec3 IB_res = gm_mat3_multiply_vec3(&e2_dynamic_inertia_tensor_inverse, rB_cross_N);
    r32 denominator_last_factor = gm_vec3_dot(gm_vec3_add(gm_vec3_cross(IA_res, rA), gm_vec3_cross(IB_res, rB)), N);
    r32 denominator = 1.0f / mA + 1.0f / mB + denominator_last_factor;
    r32 j_r = numerator / denominator;

#if 1
    // check
    vec3 vA_plus = gm_vec3_subtract(vA, gm_vec3_scalar_product(j_r / mA, N));
    vec3 vB_plus = gm_vec3_add(vB, gm_vec3_scalar_product(j_r / mB, N));
    vec3 wA_plus = gm_vec3_subtract(wA, gm_vec3_scalar_product(j_r, gm_mat3_multiply_vec3(&e1_dynamic_inertia_tensor_inverse, gm_vec3_cross(rA, N))));
    vec3 wB_plus = gm_vec3_add(wB, gm_vec3_scalar_product(j_r, gm_mat3_multiply_vec3(&e2_dynamic_inertia_tensor_inverse, gm_vec3_cross(rB, N))));
    vec3 vP1_plus = gm_vec3_add(vA_plus, gm_vec3_cross(wA_plus, rA));
    vec3 vP2_plus = gm_vec3_add(vB_plus, gm_vec3_cross(wB_plus, rB));
    vec3 vR_plus = gm_vec3_subtract(vP2_plus, vP1_plus);
    r32 vR_plus_dot_N = gm_vec3_dot(vR_plus, N);
    vec3 wA_plus_cross_rA = gm_vec3_cross(wA_plus, rA);
    vec3 wB_plus_cross_rB = gm_vec3_cross(wB_plus, rB);
    r32 vRel_plus = gm_vec3_dot(N, gm_vec3_subtract(gm_vec3_add(vA_plus, wA_plus_cross_rA), gm_vec3_add(vB_plus, wB_plus_cross_rB)));
#endif

    vec3 J_r = gm_vec3_scalar_product(j_r, cp->normal);
    
    e1->linear_momentum = gm_vec3_add(e1->linear_momentum, gm_vec3_scalar_product(-1.0f, J_r));
    //e2->linear_momentum = gm_vec3_add(e2->linear_momentum, J_r);
    e1->angular_momentum = gm_vec3_add(e1->angular_momentum, gm_vec3_scalar_product(-1.0f, gm_vec3_cross(rA, J_r)));
	//e2->angular_momentum = gm_vec3_add(e2->angular_momentum, gm_vec3_cross(rB, J_r));

#if 0
    // Tangential component
    vec3 fe = {0};
    for (u32 i = 0; i < array_length(forces); ++i) {
        fe = gm_vec3_add(fe, forces[i].force);
    }
    r32 vR_dot_N = gm_vec3_dot(vR_plus, cp->normal);
    r32 fe_dot_N = gm_vec3_dot(fe, cp->normal);
    vec3 t;
	const r32 threshold = 0.1f;
    if (fabs(vR_dot_N) < threshold) {
        t = gm_vec3_normalize(gm_vec3_subtract(vR_plus, gm_vec3_scalar_product(vR_dot_N, cp->normal)));
    } else if (fabs(fe_dot_N) < threshold) {
        t = gm_vec3_normalize(gm_vec3_subtract(fe, gm_vec3_scalar_product(fe_dot_N, cp->normal)));
    } else {
        t = (vec3){0.0f, 0.0f, 0.0f};
    }

	// project tangent onto normal's plane so we dont accumulate an error
	// @TODO: somehow make this generic...
	t.y = 0.0f;

    const r32 static_coefficient = static_friction_coefficient;
    const r32 dynamic_coefficient = dynamic_friction_coefficient;
    r32 j_s = static_coefficient * j_r;
    r32 j_d = dynamic_coefficient * j_r;

    r32 m_vR_dot_t = gm_vec3_dot(gm_vec3_scalar_product(e1->mass, vR_plus), t);
    r32 j_f;
    if (gm_vec3_length(vR_plus) < threshold) {
        if (m_vR_dot_t <= j_s) {
            j_f = -m_vR_dot_t;
        } else {
            j_f = -j_s;
        }
    } else {
        j_f = -j_d;
    }

    vec3 J_f = gm_vec3_scalar_product(j_f, t);
    //printf("J_f: <%.3f, %.3f, %.3f>\n", J_f.x, J_f.y, J_f.z);
    e1->linear_momentum = gm_vec3_add(e1->linear_momentum, gm_vec3_scalar_product(1.0f, J_f));
    //e2->linear_momentum = gm_vec3_add(e2->linear_momentum, J_f);
    e1->angular_momentum = gm_vec3_add(e1->angular_momentum, gm_vec3_scalar_product(1.0f, gm_vec3_cross(rA, J_f)));
    //e2->angular_momentum = gm_vec3_add(e2->angular_momentum, gm_vec3_cross(rB, J_f));
#endif
}

// --------- REST CONTACT START -----------
static void max_step(r32* f, r32* a, r32* delta_f, r32* delta_a, s32 d, u32* C, u32* NC, r32* s, s32* j) {
	*s = FLT_MAX;
	*j = -1;
	if (delta_a[d] > 0) {
		*j = d;
		*s = -a[d] / delta_a[d];
	}

	for (u32 i = 0; i < array_length(C); ++i) {
		if (delta_f[i] < 0.0f) {
			r32 _s = -f[i] / delta_f[i];
			if (_s < *s) {
				*s = _s;
				*j = i;
			}
		}
	}

	for (u32 i = 0; i < array_length(NC); ++i) {
		if (delta_a[i] < 0.0f) {
			r32 _s = -a[i] / delta_a[i];
			if (_s < *s) {
				*s = _s;
				*j = i;
			}
		}
	}
}

// C must be sorted (low to high)
static r32* fdirection(s32 d, const Matrix* A, u32* C) {
	s32 delta_f_size = A->columns;
	r32* delta_f = array_new_len(r32, delta_f_size);
	for (u32 i = 0; i < delta_f_size; ++i) {
		array_push(delta_f, 0.0f);
	}
	delta_f[d] = 1.0f;

	u32 C_len = array_length(C);
	if (C_len == 0) {
		// nothing to solve here
		return delta_f;
	} else if (C_len == 1) {
		// 1x1 system - solve directly
		r32 A11 = A->data[C[0]][C[0]];
		r32 v1 = A->data[C[0]][d];
		r32 x = -v1 / A11;
		delta_f[C[0]] = x;
	} else {
		// 2x2 or greater system - solve via LU decomposition

		// build the submatrix
		Matrix Acc = matrix_create(array_length(C), array_length(C));
		r32* neg_v1 = array_new_len(r32, array_length(C));
		for (u32 i = 0; i < array_length(C); ++i) {
			u32 row = C[i];
			for (u32 j = 0; j < array_length(C); ++j) {
				u32 column = C[j];
				Acc.data[i][j] = A->data[row][column];
			}

			array_push(neg_v1, -A->data[row][d]);
		}

		r32* x = malloc(sizeof(r32) * array_length(C));
		matrix_solve_system(&Acc, neg_v1, x);

		for (u32 i = 0; i < array_length(C); ++i) {
			delta_f[C[i]] = x[i];
		}

		matrix_destroy(&Acc);
		free(x);
	}

	return delta_f;
}

static s32 v_find(u32* v, u32 x) {
	// can be optimized
	for (s32 i = 0; i < array_length(v); ++i) {
		if (v[i] == x) {
			return i;
		}
	}

	return -1;
}

int v_compare(const void* e1, const void* e2) {
	return *(u32*)e1 - *(u32*)e2;
}

static boolean v_put(u32* v, u32 x) {
	array_push(v, x);
	qsort(v, array_length(v), sizeof(u32), v_compare);
}

static boolean v_remove(u32* v, s32 index) {
	array_remove_ordered(v, index);
}

static void drive_to_zero(s32 d, const Matrix* A, u32* C, u32* NC, r32* f, r32* a) {
	while (true) {
		r32* delta_f = fdirection(d, A, C);
		Matrix delta_f_matrix = matrix_from_vec(delta_f);
		Matrix delta_a_matrix = matrix_multiply(A, &delta_f_matrix);
		r32* delta_a = array_new_len(r32, delta_a_matrix.rows);
		for (u32 i = 0; i < delta_a_matrix.rows; ++i) {
			array_push(delta_a, delta_a_matrix.data[i][0]);
		}

		r32 s;
		s32 j;
		max_step(f, a, delta_f, delta_a, d, C, NC, &s, &j);

		for (u32 i = 0; i < array_length(f); ++i) {
			f[i] = f[i] + s * delta_f[i];
		}

		for (u32 i = 0; i < array_length(a); ++i) {
			a[i] = a[i] + s * delta_a[i];
		}

		array_free(delta_f);
		array_free(delta_a);
		matrix_destroy(&delta_f_matrix);
		matrix_destroy(&delta_a_matrix);

		s32 j_C = v_find(C, j);
		s32 j_NC = v_find(NC, j);
		if (j_C != -1) {
			v_remove(C, j_C);
			v_put(NC, j);
		} else if (j_NC != -1) {
			v_remove(NC, j_C);
			v_put(C, j);
		} else {
			v_put(C, j);
			return;
		}
	}
}

r32* qp_solve(const Matrix* A, r32* b) {
	assert(A->columns == A->rows);
	assert(array_length(b) == A->columns);
	r32* a = array_copy(b);
	r32* f = array_new_len(r32, array_length(b));
	for (u32 i = 0; i < array_length(b); ++i) {
		array_push(f, 0.0f);
	}

	u32* C = array_new_len(u32, array_length(f));
	u32* NC = array_new_len(u32, array_length(f));

	for (u32 d = 0; d < array_length(a); ++d) {
		if (a[d] < 0.0f) {
			drive_to_zero(d, A, C, NC, f, a);
		}
	}

	array_free(a);
	return f;
}

typedef struct {
	Entity* a;
	Entity* b;
	vec3 contact_normal;		// must be attached to 'b'
	vec3 contact_position;
} Contact;

static vec3 compute_ndot(Contact* c) {
	// @TODO: we could store the angular velocity
	mat3 dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(c->b);
	vec3 angular_velocity = physics_get_angular_velocity(c->b, &dynamic_inertia_tensor_inverse);
	return gm_vec3_cross(angular_velocity, c->contact_normal);
}

static vec3 pt_velocity(Entity* e, vec3 p) {
	// @TODO: we could store the linear and angular velocity
	vec3 linear_velocity = physics_get_linear_velocity(e);
	mat3 dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(e);
	vec3 angular_velocity = physics_get_angular_velocity(e, &dynamic_inertia_tensor_inverse);
	return gm_vec3_add(linear_velocity, gm_vec3_cross(angular_velocity, gm_vec3_subtract(p, gm_vec4_to_vec3(e->world_position))));
}

static r32* compute_b_vector(Contact* contacts) {
	r32* b = array_new_len(r32, array_length(contacts));

	for (u32 i = 0; i < array_length(contacts); ++i) {
		Contact* c = &contacts[i];
		Entity* A = contacts[i].a;
		Entity* B = contacts[i].b;
		vec3 n = c->contact_normal;
		vec3 ra = gm_vec3_subtract(c->contact_position, gm_vec4_to_vec3(A->world_position));
		vec3 rb = gm_vec3_subtract(c->contact_position, gm_vec4_to_vec3(B->world_position));
		vec3 f_ext_a, f_ext_b, t_ext_a, t_ext_b;
		physics_calculate_total_force_and_torque(A, &f_ext_a, &t_ext_a);
		physics_calculate_total_force_and_torque(B, &f_ext_b, &t_ext_b);

		mat3 a_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(A);
		vec3 a_angular_velocity = physics_get_angular_velocity(A, &a_dynamic_inertia_tensor_inverse);
		mat3 b_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(B);
		vec3 b_angular_velocity = physics_get_angular_velocity(B, &b_dynamic_inertia_tensor_inverse);

		vec3 a_ext_part = gm_vec3_add(gm_vec3_scalar_product(1.0f / A->mass, f_ext_a),
			gm_vec3_cross(gm_mat3_multiply_vec3(&a_dynamic_inertia_tensor_inverse, t_ext_a), ra));
		vec3 b_ext_part = gm_vec3_add(gm_vec3_scalar_product(1.0f / B->mass, f_ext_b),
			gm_vec3_cross(gm_mat3_multiply_vec3(&b_dynamic_inertia_tensor_inverse, t_ext_b), rb));

		vec3 a_vel_part = gm_vec3_add(
			gm_vec3_cross(a_angular_velocity, gm_vec3_cross(a_angular_velocity, ra)),
			gm_vec3_cross(gm_mat3_multiply_vec3(&a_dynamic_inertia_tensor_inverse, gm_vec3_cross(A->angular_momentum, a_angular_velocity)), ra));
		vec3 b_vel_part = gm_vec3_add(
			gm_vec3_cross(b_angular_velocity, gm_vec3_cross(b_angular_velocity, rb)),
			gm_vec3_cross(gm_mat3_multiply_vec3(&b_dynamic_inertia_tensor_inverse, gm_vec3_cross(B->angular_momentum, b_angular_velocity)), rb));
		
		r32 k1 = gm_vec3_dot(n, gm_vec3_subtract(gm_vec3_add(a_ext_part, a_vel_part), gm_vec3_add(b_ext_part, b_vel_part)));
		vec3 ndot = compute_ndot(c);
		r32 k2 = 2.0f * gm_vec3_dot(ndot, gm_vec3_subtract(pt_velocity(A, c->contact_position), pt_velocity(B, c->contact_position)));

		array_push(b, k1 + k2);
	}

	return b;
}

static r32 compute_aij(Contact* ci, Contact* cj) {
	if ((ci->a != cj->a) && (ci->b != cj->b) && (ci->a != cj->b) && (ci->b != cj->a)) {
		return 0.0f;
	}

	Entity* A = ci->a;
	Entity* B = ci->b;
	vec3 ni = ci->contact_normal;
	vec3 nj = cj->contact_normal;
	vec3 pi = ci->contact_position;
	vec3 pj = cj->contact_position;
	vec3 ra = gm_vec3_subtract(pi, gm_vec4_to_vec3(A->world_position));
	vec3 rb = gm_vec3_subtract(pi, gm_vec4_to_vec3(B->world_position));

	vec3 force_on_a = (vec3){0.0f, 0.0f, 0.0f};
	vec3 torque_on_a = (vec3){0.0f, 0.0f, 0.0f};

	if (cj->a == ci->a) {
		force_on_a = nj;
		torque_on_a = gm_vec3_cross(gm_vec3_subtract(pj, gm_vec4_to_vec3(A->world_position)), nj);
	} else if (cj->b == ci->a) {
		force_on_a = gm_vec3_scalar_product(-1.0f, nj);
		torque_on_a = gm_vec3_cross(gm_vec3_subtract(pj, gm_vec4_to_vec3(A->world_position)), nj);
	}

	vec3 force_on_b = (vec3){0.0f, 0.0f, 0.0f};
	vec3 torque_on_b = (vec3){0.0f, 0.0f, 0.0f};

	if (cj->a == ci->b) {
		force_on_b = nj;
		torque_on_b = gm_vec3_cross(gm_vec3_subtract(pj, gm_vec4_to_vec3(B->world_position)), nj);
	} else if (cj->b == ci->b) {
		force_on_b = gm_vec3_scalar_product(-1.0f, nj);
		torque_on_b = gm_vec3_cross(gm_vec3_subtract(pj, gm_vec4_to_vec3(B->world_position)), nj);
	}

	mat3 a_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(A);
	mat3 b_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(B);

	vec3 a_linear = gm_vec3_scalar_product(1.0f / A->mass, force_on_a);
	vec3 a_angular = gm_vec3_cross(gm_mat3_multiply_vec3(&a_dynamic_inertia_tensor_inverse, torque_on_a), ra);
	vec3 b_linear = gm_vec3_scalar_product(1.0f / B->mass, force_on_b);
	vec3 b_angular = gm_vec3_cross(gm_mat3_multiply_vec3(&b_dynamic_inertia_tensor_inverse, torque_on_b), rb);

	return gm_vec3_dot(ni, gm_vec3_subtract(gm_vec3_add(a_linear, a_angular), gm_vec3_add(b_linear, b_angular)));
}

static Matrix compute_a_matrix(Contact* contacts) {
	Matrix A = matrix_create(array_length(contacts), array_length(contacts));
	for (u32 i = 0; i < array_length(contacts); ++i) {
		for (u32 j = 0; j < array_length(contacts); ++j) {
			A.data[i][j] = compute_aij(&contacts[i], &contacts[j]);
		}
	}
	return A;
}

static vec3 calculate_pt_acceleration(Entity* B, vec3 p) {
	mat3 dyn_inertia = physics_get_dynamic_inertia_tensor_inverse(B);
	vec3 angular_velocity = physics_get_angular_velocity(B, &dyn_inertia);
	// calculate acceleration of point
	vec3 total_force, total_torque;
	physics_calculate_total_force_and_torque(B, &total_force, &total_torque);
	vec3 first_term = gm_vec3_scalar_product(1.0f / B->mass, total_force);
	vec3 rb = gm_vec3_subtract(p, gm_vec4_to_vec3(B->world_position));
	vec3 second_term = gm_vec3_cross(gm_mat3_multiply_vec3(&dyn_inertia, total_torque), rb);
	vec3 third_term = gm_vec3_cross(angular_velocity, gm_vec3_cross(angular_velocity, rb));
	vec3 fourth_term = gm_vec3_cross(gm_mat3_multiply_vec3(&dyn_inertia, gm_vec3_cross(B->angular_momentum, angular_velocity)), rb);
	vec3 acc = gm_vec3_add(gm_vec3_add(first_term, second_term), gm_vec3_add(third_term, fourth_term));
	return acc;
}

void compute_contact_forces(Contact* contacts, r32 dt) {
	#if 0
	Entity EA, EB;
	Mesh cube = graphics_mesh_create_from_obj("./res/cube.obj", 0);
	graphics_entity_create_with_color(&EA, cube, (vec4){0.0f, 0.0f, 0.0f}, (Quaternion){0.0f, 0.0f, 0.0f, 0.0f}, (vec3){1.0f, 1.0f, 1.0f},
		(vec4){1.0f, 0.0f, 0.0f, 1.0f}, 10.0f);
	graphics_entity_create_with_color(&EB, cube, (vec4){0.0f, -1.0f, 0.0f}, (Quaternion){0.0f, 0.0f, 0.0f, 0.0f}, (vec3){1.0f, 1.0f, 1.0f},
		(vec4){1.0f, 0.0f, 0.0f, 1.0f}, 1000000000.0f);
	EA.linear_momentum = (vec3){0.0f, -10.0f, 0.0f};
	EA.angular_momentum = (vec3){0.0f, -10.0f, 0.0f};
	Physics_Force pf;
	pf.force = (vec3){0.0f, -10.0f, 0.0f};
	pf.position = (vec3){0.0f, 0.0f, 0.0f};
	array_push(EA.forces, pf);
	Contact fake;
	fake.a = &EA;
	fake.b = &EB;
	fake.contact_normal = (vec3){0.0f, 1.0f, 0.0f};
	fake.contact_position = (vec3){30.0f, -1.0f, 0.0f};
	array_clear(contacts);
	array_push(contacts, fake);

	Matrix A_matrix = compute_a_matrix(contacts);
	r32* b = compute_b_vector(contacts);
	r32* f = qp_solve(&A_matrix, b);
	r32 f_i = f[0];

	vec3 n = contacts[0].contact_normal;
	Entity* A = contacts[0].a;
	Entity* B = contacts[0].b;

	vec3 original_acc_B = calculate_pt_acceleration(B, contacts[0].contact_position);
	vec3 original_acc_A = calculate_pt_acceleration(A, contacts[0].contact_position);
	r32 original_final = gm_vec3_dot(contacts[0].contact_normal, gm_vec3_subtract(original_acc_A, original_acc_B)) +
		gm_vec3_dot(compute_ndot(&contacts[0]), gm_vec3_subtract(pt_velocity(A, contacts[0].contact_position), pt_velocity(B, contacts[0].contact_position)));
	r32 original_a00 = compute_aij(&contacts[0], &contacts[0]);

	Physics_Force A_force, B_force;
	A_force.force = gm_vec3_scalar_product(f_i, n);
	A_force.position = gm_vec3_subtract(contacts[0].contact_position, gm_vec4_to_vec3(A->world_position));
	B_force.force = gm_vec3_scalar_product(-f_i, n);
	B_force.position = gm_vec3_subtract(contacts[0].contact_position, gm_vec4_to_vec3(B->world_position));
	array_push(A->forces, A_force);
	array_push(B->forces, B_force);

	vec3 new_acc_B = calculate_pt_acceleration(B, contacts[0].contact_position);
	vec3 new_acc_A = calculate_pt_acceleration(A, contacts[0].contact_position);
	r32 new_final = gm_vec3_dot(contacts[0].contact_normal, gm_vec3_subtract(new_acc_A, new_acc_B)) +
		gm_vec3_dot(compute_ndot(&contacts[0]), gm_vec3_subtract(pt_velocity(A, contacts[0].contact_position), pt_velocity(B, contacts[0].contact_position)));
	r32 new_a00 = compute_aij(&contacts[0], &contacts[0]);

	// trash code
	physics_update_momenta_based_on_forces(B, 0.1f, B->forces);
	mat3 dyn_inertia = physics_get_dynamic_inertia_tensor_inverse(B);
	vec3 angular_velocity = physics_get_angular_velocity(B, &dyn_inertia);
	vec3 linear_velocity = physics_get_linear_velocity(B);
	vec3 instantaneous_velocity = gm_vec3_add(linear_velocity,
		gm_vec3_cross(angular_velocity, gm_vec3_subtract(contacts[0].contact_position, gm_vec4_to_vec3(B->world_position))));
	printf("<%f, %f, %f>\n", instantaneous_velocity.x, instantaneous_velocity.y, instantaneous_velocity.z);

	vec3 acc_after_time_B = calculate_pt_acceleration(B, contacts[0].contact_position);
	vec3 acc_after_time_A = calculate_pt_acceleration(A, contacts[0].contact_position);
	r32 after_time_final = gm_vec3_dot(contacts[0].contact_normal, gm_vec3_subtract(acc_after_time_A, acc_after_time_B)) +
		gm_vec3_dot(compute_ndot(&contacts[0]), gm_vec3_subtract(pt_velocity(A, contacts[0].contact_position), pt_velocity(B, contacts[0].contact_position)));

#else
	Matrix A_matrix = compute_a_matrix(contacts);
	r32* b = compute_b_vector(contacts);
	r32* f = qp_solve(&A_matrix, b);

	for (u32 i = 0; i < array_length(contacts); ++i) {
		r32 f_i = f[i];
		vec3 n = contacts[i].contact_normal;
		Entity* A = contacts[i].a;
		Entity* B = contacts[i].b;

#if 0
		vec3 A_force = gm_vec3_scalar_product(f_i, n);
		vec3 A_torque = gm_vec3_cross(gm_vec3_subtract(contacts[i].contact_position, gm_vec4_to_vec3(A->world_position)), gm_vec3_scalar_product(f_i, n));
		vec3 B_force = gm_vec3_scalar_product(-f_i, n);
		vec3 B_torque = gm_vec3_scalar_product(-1.0f, gm_vec3_cross(gm_vec3_subtract(contacts[i].contact_position, gm_vec4_to_vec3(B->world_position)), gm_vec3_scalar_product(f_i, n)));

		vec3 A_angular_momentum_change = gm_vec3_scalar_product(dt, A_torque);
		vec3 A_linear_momentum_change = gm_vec3_scalar_product(dt, A_force);
		A->angular_momentum = gm_vec3_add(A->angular_momentum, A_angular_momentum_change);
		A->linear_momentum = gm_vec3_add(A->linear_momentum, A_linear_momentum_change);

		vec3 B_angular_momentum_change = gm_vec3_scalar_product(dt, B_torque);
		vec3 B_linear_momentum_change = gm_vec3_scalar_product(dt, B_force);
		B->angular_momentum = gm_vec3_add(B->angular_momentum, B_angular_momentum_change);
		B->linear_momentum = gm_vec3_add(B->linear_momentum, B_linear_momentum_change);
#else
		vec3 original_acc_B = calculate_pt_acceleration(B, contacts[i].contact_position);
		vec3 original_acc_A = calculate_pt_acceleration(A, contacts[i].contact_position);
		r32 original_final = gm_vec3_dot(contacts[i].contact_normal, gm_vec3_subtract(original_acc_A, original_acc_B)) +
			gm_vec3_dot(compute_ndot(&contacts[i]), gm_vec3_subtract(pt_velocity(A, contacts[i].contact_position), pt_velocity(B, contacts[i].contact_position)));

		Physics_Force A_force, B_force;
		A_force.force = gm_vec3_scalar_product(f_i, n);
		A_force.position = gm_vec3_subtract(contacts[i].contact_position, gm_vec4_to_vec3(A->world_position));
		B_force.force = gm_vec3_scalar_product(-f_i, n);
		B_force.position = gm_vec3_subtract(contacts[i].contact_position, gm_vec4_to_vec3(B->world_position));
		array_push(A->forces, A_force);
		array_push(B->forces, B_force);
		vec3 relative_velocity = pt_velocity(B, contacts[i].contact_position);
		vec3 new_acc = calculate_pt_acceleration(B, contacts[i].contact_position);
		vec3 new_acc_A = calculate_pt_acceleration(A, contacts[i].contact_position);
		r32 new_final = gm_vec3_dot(contacts[i].contact_normal, gm_vec3_subtract(new_acc_A, new_acc)) +
			gm_vec3_dot(compute_ndot(&contacts[i]), gm_vec3_subtract(pt_velocity(A, contacts[i].contact_position), pt_velocity(B, contacts[i].contact_position)));
/*

		// trash code
		physics_update_momenta_based_on_forces(B, 0.1f, B->forces);
		mat3 dyn_inertia = physics_get_dynamic_inertia_tensor_inverse(B);
		vec3 angular_velocity = physics_get_angular_velocity(B, &dyn_inertia);
		vec3 linear_velocity = physics_get_linear_velocity(B);
		vec3 instantaneous_velocity = gm_vec3_add(linear_velocity,
			gm_vec3_cross(angular_velocity, gm_vec3_subtract(contacts[i].contact_position, gm_vec4_to_vec3(B->world_position))));
		printf("<%f, %f, %f>\n", instantaneous_velocity.x, instantaneous_velocity.y, instantaneous_velocity.z);

		vec3 acc_after_time_B = calculate_pt_acceleration(B, contacts[0].contact_position);
		vec3 acc_after_time_A = calculate_pt_acceleration(A, contacts[0].contact_position);
		r32 after_time_final = gm_vec3_dot(contacts[0].contact_normal, gm_vec3_subtract(acc_after_time_B, acc_after_time_B)) +
			gm_vec3_dot(compute_ndot(&contacts[0]), gm_vec3_subtract(pt_velocity(A, contacts[0].contact_position), pt_velocity(B, contacts[0].contact_position)));
*/
#endif
		printf("Hello\n");
	}
#endif
}

// --------- REST CONTACT FINISH -----------

static void physics_update(Entity* e, r32 dt) {
	if (e->mass > MAX_MASS_TO_CONSIDER_STATIC_BODY) {
		return;
	}
	mat3 dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(e);
	vec3 angular_velocity = physics_get_angular_velocity(e, &dynamic_inertia_tensor_inverse);
	vec3 linear_velocity = physics_get_linear_velocity(e);

    // Calculate orientation change
    r32 rotation_angle = gm_vec3_length(angular_velocity) * dt;
    vec3 rotation_axis = gm_vec3_normalize(angular_velocity);
    Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
    graphics_entity_set_rotation(e, quaternion_product(&orientation_change, &e->world_rotation));

    // Calculate position change
    vec3 position_change = gm_vec3_scalar_product(dt, linear_velocity);
    graphics_entity_set_position(e, gm_vec4_add(e->world_position, (vec4){position_change.x, position_change.y, position_change.z, 0.0f}));
}

static void force_collision_fix(Entity* moving_body, Entity* static_body) {
	GJK_Support_List gjk_sl = {0};
	boolean has_collision = collision_gjk_collides(&gjk_sl, &static_body->bs, &moving_body->bs);

	if (has_collision) {
		Collision_Point cp = collision_epa(gjk_sl.simplex, &static_body->bs, &moving_body->bs);
		cp.normal = gm_vec3_scalar_product(-1.0f, cp.normal);
		gjk_sl = (GJK_Support_List){0};
		graphics_entity_set_position(moving_body, gm_vec4_add(moving_body->world_position, (vec4){cp.normal.x, cp.normal.y, cp.normal.z, 0.0f}));
		//moving_body->linear_momentum = (vec3){0};
		//moving_body->angular_momentum = (vec3){0};
	}
}

extern r32 restitution;

#if 0
void physics_simulate(Entity* entities, r32 dt) {

	//const r32 restitution = 0.1f;

	// First pass
	boolean any_collision_found = true;
	for (u32 i = 0; i < 5 && any_collision_found; ++i) {
		any_collision_found = false;

		// Update all entities based on their current momenta
		vec4* entities_original_positions = array_new(vec4);
		Quaternion* entities_original_rotations = array_new(Quaternion);
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			array_push(entities_original_positions, e->world_position);
			array_push(entities_original_rotations, e->world_rotation);
			physics_update(e, dt);
		}

		// Resolve all collisions
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e1 = &entities[j];
			vec4 e1_old_position = e1->world_position;
			Quaternion e1_old_rotation = e1->world_rotation;

			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e2 = &entities[k];
				vec4 e2_old_position = e2->world_position;
				Quaternion e2_old_rotation = e2->world_rotation;

				GJK_Support_List gjk_sl = {0};
				boolean has_collision = collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs);

				if (has_collision) {
					Collision_Point cp = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
					if (e1->mass < 1000.0f && e2->mass < 1000.0f) {
						collision = true;
						col_point = cp.collision_point;
						penetration = cp.normal;
					}
					
					apply_impulse(e1, e2, &cp, e1->forces, restitution);

					// Unfortunately we need to run GJK/EPA again to get the collision point wrt the other entity.
					// The problem is that the impulse algorithm expects that the impulse is applied at the time of collision,
					// i.e. the collision point is exact the same for both entities.
					// Unfortunately, this will not be true, since there will be an overlap between the entities.
					// For this reason, we run GJK/EPA again so we can correctly estimate the collision point of each one
					// If we don't do this, an error will be introduced and the error might grow as the time passes
					// @TODO 1: Investigate if we can tweak GJK/EPA to get both points at the same time
					// @TODO 2: Modify main loop to use dt_frac scheme, and see if by rewinding the time we can disregard the error
					// @TODO 3: When one of the rigid bodies is static, we can avoid the second run.
					gjk_sl = (GJK_Support_List){0};
					// @TODO: this should always be true, but sometimes it isnt
					if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
						cp = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
						apply_impulse(e2, e1, &cp, e1->forces, restitution);
					}

					any_collision_found = true;
				}
			}
		}

		// Reset position/orientation of all entities
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			graphics_entity_set_position(e, entities_original_positions[j]);
			graphics_entity_set_rotation(e, entities_original_rotations[j]);
		}

		array_free(entities_original_positions);
		array_free(entities_original_rotations);
	}

	// Update entities momenta based on forces
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		physics_update_momenta_based_on_forces(e, dt, e->forces);
	}

	// Second pass
	any_collision_found = true;
	for (u32 i = 0; i < 10 && any_collision_found; ++i) {
		any_collision_found = false;

		// Update all entities based on their current momenta
		vec4* entities_original_positions = array_new(vec4);
		Quaternion* entities_original_rotations = array_new(Quaternion);
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			array_push(entities_original_positions, e->world_position);
			array_push(entities_original_rotations, e->world_rotation);
			physics_update(e, dt);
		}

		// Resolve all collisions
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e1 = &entities[j];
			vec4 e1_old_position = e1->world_position;
			Quaternion e1_old_rotation = e1->world_rotation;

			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e2 = &entities[k];
				vec4 e2_old_position = e2->world_position;
				Quaternion e2_old_rotation = e2->world_rotation;

				GJK_Support_List gjk_sl = {0};
				boolean has_collision = collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs);

				if (has_collision) {
					Collision_Point cp = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
					apply_impulse(e1, e2, &cp, e1->forces, 0.0f);

					// Unfortunately we need to run GJK/EPA again to get the collision point wrt the other entity.
					// The problem is that the impulse algorithm expects that the impulse is applied at the time of collision,
					// i.e. the collision point is exact the same for both entities.
					// Unfortunately, this will not be true, since there will be an overlap between the entities.
					// For this reason, we run GJK/EPA again so we can correctly estimate the collision point of each one
					// If we don't do this, an error will be introduced and the error might grow as the time passes
					// @TODO 1: Investigate if we can tweak GJK/EPA to get both points at the same time
					// @TODO 2: Modify main loop to use dt_frac scheme, and see if by rewinding the time we can disregard the error
					// @TODO 3: When one of the rigid bodies is static, we can avoid the second run.
					gjk_sl = (GJK_Support_List){0};
					// @TODO: this should always be true, but sometimes it isnt
					if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
						cp = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
						apply_impulse(e2, e1, &cp, e1->forces, 0.0f);
					}

					any_collision_found = true;
				}
			}
		}

		// Reset position/orientation of all entities
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			graphics_entity_set_position(e, entities_original_positions[j]);
			graphics_entity_set_rotation(e, entities_original_rotations[j]);
		}

		array_free(entities_original_positions);
		array_free(entities_original_rotations);
	}

	for (u32 j = 0; j < array_length(entities); ++j) {
		Entity* e = &entities[j];
		physics_update(e, dt);
	}

	//force_collision_fix(&entities[2], &entities[0]);
	//force_collision_fix(&entities[3], &entities[0]);
	//force_collision_fix(&entities[4], &entities[0]);
	//force_collision_fix(&entities[3], &entities[2]);
	//force_collision_fix(&entities[4], &entities[3]);
}
#endif

void physics_simulate(Entity* entities, r32 dt) {
	Contact* contacts = array_new(Contact);

	boolean any_collision_found = true;
	for (u32 i = 0; i < 5 && any_collision_found; ++i) {
		any_collision_found = false;

		// Update all entities based on their current momenta
		vec4* entities_original_positions = array_new(vec4);
		Quaternion* entities_original_rotations = array_new(Quaternion);
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			array_push(entities_original_positions, e->world_position);
			array_push(entities_original_rotations, e->world_rotation);
			physics_update(e, dt);
		}

		// Resolve all collisions
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e1 = &entities[j];
			vec4 e1_old_position = e1->world_position;
			Quaternion e1_old_rotation = e1->world_rotation;

			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e2 = &entities[k];
				if (e1->mass >= 10000.0f && e2->mass >= 10000.0f) {
					continue;
				}
				vec4 e2_old_position = e2->world_position;
				Quaternion e2_old_rotation = e2->world_rotation;

				GJK_Support_List gjk_sl = {0};
				boolean has_collision = collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs);

				if (has_collision) {
					Collision_Point cp = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
					cp.normal = gm_vec3_normalize(cp.normal);

					// bem tosco
					vec3 padot = pt_velocity(e1, cp.collision_point);
					vec3 pbdot = pt_velocity(e2, cp.collision_point);
					r32 vrel = gm_vec3_dot(cp.normal, gm_vec3_subtract(padot, pbdot));
					const r32 THRESHOLD = 0.0001f;
					if (vrel > THRESHOLD) {
						// moving away
					} else if (vrel > -THRESHOLD) {
						printf("Resting :)\n");
						// resting contact
						Contact c;
						c.a = e1;
						c.b = e2;
						c.contact_normal = gm_vec3_normalize(cp.normal);
						c.contact_position = cp.collision_point;
						array_push(contacts, c);
					} else {
						apply_impulse(e1, e2, &cp, e1->forces, 0.0f);

						// Unfortunately we need to run GJK/EPA again to get the collision point wrt the other entity.
						// The problem is that the impulse algorithm expects that the impulse is applied at the time of collision,
						// i.e. the collision point is exact the same for both entities.
						// Unfortunately, this will not be true, since there will be an overlap between the entities.
						// For this reason, we run GJK/EPA again so we can correctly estimate the collision point of each one
						// If we don't do this, an error will be introduced and the error might grow as the time passes
						// @TODO 1: Investigate if we can tweak GJK/EPA to get both points at the same time
						// @TODO 2: Modify main loop to use dt_frac scheme, and see if by rewinding the time we can disregard the error
						// @TODO 3: When one of the rigid bodies is static, we can avoid the second run.
						gjk_sl = (GJK_Support_List){0};
						// @TODO: this should always be true, but sometimes it isnt
						if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
							cp = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
							apply_impulse(e2, e1, &cp, e1->forces, 0.0f);
						}

						any_collision_found = true;
					}
				}
			}
		}

		if (array_length(contacts) > 0) {
			Contact c;
			c.a = contacts[0].a;
			c.b = contacts[0].b;
			array_clear(contacts);

			c.contact_position = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&entities[0].model_matrix, entities[0].mesh.vertices[4].position));
			c.contact_normal = (vec3){0.0f, 1.0f, 0.0f};
			if (c.contact_position.y <= 0.0f) array_push(contacts, c);
			c.contact_position = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&entities[0].model_matrix, entities[0].mesh.vertices[5].position));
			c.contact_normal = (vec3){0.0f, 1.0f, 0.0f};
			if (c.contact_position.y <= 0.0f) array_push(contacts, c);
			c.contact_position = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&entities[0].model_matrix, entities[0].mesh.vertices[7].position));
			c.contact_normal = (vec3){0.0f, 1.0f, 0.0f};
			if (c.contact_position.y <= 0.0f) array_push(contacts, c);
			c.contact_position = gm_vec4_to_vec3(gm_mat4_multiply_vec4(&entities[0].model_matrix, entities[0].mesh.vertices[9].position));
			c.contact_normal = (vec3){0.0f, 1.0f, 0.0f};
			if (c.contact_position.y <= 0.0f) array_push(contacts, c);
			compute_contact_forces(contacts, dt);
		}

		array_clear(contacts);

		// Reset position/orientation of all entities
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			graphics_entity_set_position(e, entities_original_positions[j]);
			graphics_entity_set_rotation(e, entities_original_rotations[j]);
		}

		array_free(entities_original_positions);
		array_free(entities_original_rotations);
	}

	// Update entities momenta based on forces
	for (u32 i = 0; i < array_length(entities); ++i) {
		Entity* e = &entities[i];
		physics_update_momenta_based_on_forces(e, dt, e->forces);
	}

	for (u32 j = 0; j < array_length(entities); ++j) {
		Entity* e = &entities[j];
		physics_update(e, dt);
	}

	array_free(contacts);
}