#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include <float.h>
#include "collision.h"
#include "gjk.h"

#define NUM_SUBSTEPS 1
#define NUM_POS_ITERS 1

// Calculate the sum of all external forces acting on an entity
static vec3 calculate_external_force(Entity* e) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        total_force = gm_vec3_add(total_force, e->forces[i].force);
    }
	return total_force;
}

// Calculate the sum of all external torques acting on an entity
static vec3 calculate_external_torque(Entity* e) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
    }
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inertia_tensor(Entity* e) {
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inverse_inertia_tensor(Entity* e) {
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inverse_inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

// Solves the positional constraint, updating the position and orientation of the entities accordingly
static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = constraint->positional_constraint.e1;
	Entity* e2 = constraint->positional_constraint.e2;
	mat3 e1_rot = quaternion_get_matrix3(&e1->world_rotation);
	mat3 e2_rot = quaternion_get_matrix3(&e2->world_rotation);
	vec3 r1 = constraint->positional_constraint.r1;	// in world coordinates
	vec3 r2 = constraint->positional_constraint.r2; // in world coordinates
	r32 lambda = *constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	vec3 delta_x = constraint->positional_constraint.delta_x;

	mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
	mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);

	// split delta_x into n and c.
	vec3 n = gm_vec3_normalize(delta_x);
	r32 c = gm_vec3_length(delta_x);

	// calculate the inverse masses of both entities
	r32 w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
	r32 w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));

	// calculate the delta_lambda (XPBD) and updates the constraint
	r32 til_compliance = compliance / (h * h);
	r32 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);
	*constraint->positional_constraint.lambda += delta_lambda;

	// calculates the positional impulse
	vec3 positional_impulse = gm_vec3_scalar_product(delta_lambda, n);

	// updates the position of the entities based on eq (6) and (7)
	if (!e1->fixed) {
		e1->world_position = gm_vec3_add(e1->world_position, gm_vec3_scalar_product(e1->inverse_mass, positional_impulse));
	}
	if (!e2->fixed) {
		e2->world_position = gm_vec3_add(e2->world_position, gm_vec3_scalar_product(-e2->inverse_mass, positional_impulse));
	}

	// updates the rotation of the entities based on eq (8) and (9)
	vec3 aux1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 aux2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	Quaternion aux_q1 = (Quaternion){aux1.x, aux1.y, aux1.z, 0.0f};
	Quaternion aux_q2 = (Quaternion){aux2.x, aux2.y, aux2.z, 0.0f};
	Quaternion q1 = quaternion_product(&aux_q1, &e1->world_rotation);
	Quaternion q2 = quaternion_product(&aux_q2, &e2->world_rotation);
	if (!e1->fixed) {
		e1->world_rotation.x = e1->world_rotation.x + 0.5f * q1.x;
		e1->world_rotation.y = e1->world_rotation.y + 0.5f * q1.y;
		e1->world_rotation.z = e1->world_rotation.z + 0.5f * q1.z;
		e1->world_rotation.w = e1->world_rotation.w + 0.5f * q1.w;
		// should we normalize?
		e1->world_rotation = quaternion_normalize(&e1->world_rotation);
	}
	if (!e2->fixed) {
		e2->world_rotation.x = e2->world_rotation.x - 0.5f * q2.x;
		e2->world_rotation.y = e2->world_rotation.y - 0.5f * q2.y;
		e2->world_rotation.z = e2->world_rotation.z - 0.5f * q2.z;
		e2->world_rotation.w = e2->world_rotation.w - 0.5f * q2.w;
		// should we normalize?
		e2->world_rotation = quaternion_normalize(&e2->world_rotation);
	}
}

static void solve_constraint(Constraint* constraint, r32 h) {
	switch (constraint->type) {
		case POSITIONAL_CONSTRAINT: {
			solve_positional_constraint(constraint, h);
			return;
		}
	}

	assert(0);
}

static vec3 calculate_p_til(Entity* e, vec3 r_lc) {
	switch (e->type) {
		case PLANE: {
			return (vec3){0.0f, e->world_position.y, 0.0f}; // we consider an infinite plane, so we just project the cube vertex onto the plane
		} break;
		case SPHERE: {
			mat3 previous_rot_matrix = quaternion_get_matrix3(&e->previous_world_rotation);
			return gm_vec3_add(e->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, r_lc));
		} break;
		default: assert(0);
	}
}
static vec3 calculate_p(Entity* e, vec3 r_lc) {
	switch (e->type) {
		case PLANE: {
			return (vec3){0.0f, e->world_position.y, 0.0f}; // we consider an infinite plane, so we just project the cube vertex onto the plane
		} break;
		case SPHERE: {
			mat3 current_rot_matrix = quaternion_get_matrix3(&e->world_rotation);
			return gm_vec3_add(e->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, r_lc));
		} break;
		default: assert(0);
	}
}

// Create 'artificial' position constraints for a given collision that was detected in the simulation
static void create_constraints_for_collision(Collision_Info* ci, Constraint** constraints) {
	Constraint constraint = {0};
	constraint.type = POSITIONAL_CONSTRAINT;
	constraint.positional_constraint.compliance = 0.0f; // as stated in sec 3.5, compliance is 0 for collision constraints
	constraint.positional_constraint.e1 = ci->e1;
	constraint.positional_constraint.e2 = ci->e2;
	// we make lambda a reference to the value stored in the collision_info, so we can accumulate per iteration
	constraint.positional_constraint.lambda = &ci->lambda_n;
	constraint.positional_constraint.r1 = ci->r1_wc;
	constraint.positional_constraint.r2 = ci->r2_wc;

	// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
	vec3 p1 = calculate_p(ci->e1, ci->r1_lc);
	vec3 p2 = calculate_p(ci->e2, ci->r2_lc);
	r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), ci->normal);
	constraint.positional_constraint.delta_x = gm_vec3_scalar_product(d, ci->normal);
	if (d > 0.0f) {
		// if 'd' is greater than 0.0, we add the constraint.
		array_push(*constraints, constraint);

		// if 'd' is greater than 0.0, we should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
		// the problem is that if NUM_POS_ITERS is 1, lambda_t and lambda_n will always be 0.0, so this will never be used.
		const r32 static_friction_coefficient = 0.5f;
		if (ci->lambda_t < static_friction_coefficient * ci->lambda_n) {
			vec3 p1_til = calculate_p_til(ci->e1, ci->r1_lc);
			vec3 p2_til = calculate_p_til(ci->e2, ci->r2_lc);
			vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
			vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(gm_vec3_dot(delta_p, ci->normal), ci->normal));
			Constraint constraint = {0};
			constraint.type = POSITIONAL_CONSTRAINT;
			constraint.positional_constraint.compliance = 0.0f;
			constraint.positional_constraint.e1 = ci->e1;
			constraint.positional_constraint.e2 = ci->e2;
			constraint.positional_constraint.lambda = &ci->lambda_t;
			constraint.positional_constraint.r1 = ci->r1_wc;
			constraint.positional_constraint.r2 = ci->r2_wc;
			constraint.positional_constraint.delta_x = delta_p_t;
			array_push(*constraints, constraint);
		}
	}
}

static vec3 find_ortho(vec3 v) {
	vec3 aux = v;
	aux.x += 100.0;
	aux = gm_vec3_normalize(aux);
	aux = gm_vec3_cross(v, aux);
	return gm_vec3_normalize(aux);
}

Collision_Info* collision_infos = NULL;
int has_unlucky_one;
Collision_Info unlucky_one;
int has_got_from_gjk;
Collision_Info got_from_gjk;

static void update_collision_infos() {
	for (int i = 0; i < array_length(collision_infos); ++i) {
		Collision_Info* current = &collision_infos[i];
		mat3 q1_mat = quaternion_get_matrix3(&current->e1->world_rotation);
		current->r1_wc = gm_mat3_multiply_vec3(&q1_mat, current->r1_lc);

		mat3 q2_mat = quaternion_get_matrix3(&current->e2->world_rotation);
		current->r2_wc = gm_mat3_multiply_vec3(&q2_mat, current->r2_lc);

		vec3 collision_point1 = gm_vec3_add(current->e1->world_position, current->r1_wc);
		vec3 collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);
		r32 distance = gm_vec3_dot(gm_vec3_subtract(collision_point1, collision_point2), current->normal);
		//r32 distance = -gm_vec3_length(gm_vec3_subtract(collision_point1, collision_point2));
		//printf("distance: %.3f\n", distance);

		//if (distance > 0.1f) {
		//	printf("removing\n");
		//	array_remove(collision_infos, i);
		//	--i;
		//}
	}
}

static r32 calc_area_4_points(vec3 p0, vec3 p1, vec3 p2, vec3 p3) {
	vec3 a0 = gm_vec3_subtract(p0, p1);
	vec3 a1 = gm_vec3_subtract(p0, p2);
	vec3 a2 = gm_vec3_subtract(p0, p3);
	vec3 b0 = gm_vec3_subtract(p2, p3);
	vec3 b1 = gm_vec3_subtract(p1, p3);
	vec3 b2 = gm_vec3_subtract(p1, p2);

	vec3 tmp0 = gm_vec3_cross(a0, b0);
	vec3 tmp1 = gm_vec3_cross(a1, b1);
	vec3 tmp2 = gm_vec3_cross(a2, b2);

	r32 l0 = gm_vec3_length(tmp0);
	r32 l1 = gm_vec3_length(tmp1);
	r32 l2 = gm_vec3_length(tmp2);

	if (l0 > l1 && l0 > l2) return l0;
	if (l1 > l2) return l1;
	return l2;
}

#if 0
static void add_collision_info(Collision_Info ci) {
	has_got_from_gjk = 1; got_from_gjk = ci;

	vec3 received_collision_point1 = gm_vec3_add(ci.e1->world_position, ci.r1_wc);
	vec3 received_collision_point2 = gm_vec3_add(ci.e2->world_position, ci.r2_wc);

	for (int i = 0; i < array_length(collision_infos); ++i) {
		Collision_Info* current = &collision_infos[i];
		vec3 collision_point1 = gm_vec3_add(current->e1->world_position, current->r1_wc);
		vec3 collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);

		r32 diff1 = gm_vec3_length(gm_vec3_subtract(collision_point1, received_collision_point1));
		r32 diff2 = gm_vec3_length(gm_vec3_subtract(collision_point2, received_collision_point2));
		r32 diff = diff1 < diff2 ? diff1 : diff2;
		if (diff2 < 0.02f) {
			*current = ci;
			// already there
			return;
		}
	}

	array_push(collision_infos, ci);
	if (array_length(collision_infos) <= 4) {
		return;
	}

	// Need to remove one collision point!
	vec3* collision_points1 = array_new(vec3);
	vec3* collision_points2 = array_new(vec3);
	r32 deepest_penetration = -FLT_MAX;
	Collision_Info chosen;
	int chosen_idx = -1;

	for (int i = 0; i < array_length(collision_infos); ++i) {
		Collision_Info* current = &(collision_infos)[i];

		vec3 current_collision_point1 = gm_vec3_add(current->e1->world_position, current->r1_wc);
		vec3 current_collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);
		array_push(collision_points1, current_collision_point1);
		array_push(collision_points2, current_collision_point2);

		r32 current_penetration = gm_vec3_dot(gm_vec3_subtract(current_collision_point1, current_collision_point2), current->normal);

		if (current_penetration > deepest_penetration) {
			deepest_penetration = current_penetration;
			chosen_idx = i;
			chosen = *current;
		}
	}
	chosen = collision_infos[3];
	chosen_idx = 3;

	// We are left with 4 vertices. We keep the 3 that form the biggest triangle.

	// remove the chosen one just to ease calculations
	array_remove(collision_infos, chosen_idx); // tmp, 
	array_remove(collision_points1, chosen_idx);
	array_remove(collision_points2, chosen_idx);

	r32 a1_1 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points1[1], collision_points1[0]),
		gm_vec3_subtract(collision_points1[2], collision_points1[0]))); // 0, 1, 2
	r32 a2_1 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points1[2], collision_points1[1]),
		gm_vec3_subtract(collision_points1[3], collision_points1[1]))); // 1, 2, 3
	r32 a3_1 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points1[2], collision_points1[0]),
		gm_vec3_subtract(collision_points1[3], collision_points1[0]))); // 0, 2, 3
	r32 a4_1 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points1[1], collision_points1[0]),
		gm_vec3_subtract(collision_points1[3], collision_points1[0]))); // 0, 1, 3

	r32 a1_2 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points2[1], collision_points2[0]),
		gm_vec3_subtract(collision_points2[2], collision_points2[0]))); // 0, 1, 2
	r32 a2_2 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points2[2], collision_points2[1]),
		gm_vec3_subtract(collision_points2[3], collision_points2[1]))); // 1, 2, 3
	r32 a3_2 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points2[2], collision_points2[0]),
		gm_vec3_subtract(collision_points2[3], collision_points2[0]))); // 0, 2, 3
	r32 a4_2 = gm_vec3_length(gm_vec3_cross(gm_vec3_subtract(collision_points2[1], collision_points2[0]),
		gm_vec3_subtract(collision_points2[3], collision_points2[0]))); // 0, 1, 3

	r32 a1 = a1_1 + a1_2;
	r32 a2 = a2_1 + a2_2;
	r32 a3 = a3_1 + a3_2;
	r32 a4 = a4_1 + a4_2;

	array_free(collision_points1);
	array_free(collision_points2);

	if (a1 > a2 && a1 > a3 && a1 > a4) {
		// a1 wins
		unlucky_one = (collision_infos)[3]; has_unlucky_one = 1;
		array_remove(collision_infos, 3);
	} else if (a2 > a3 && a2 > a4) {
		// a2 wins
		unlucky_one = (collision_infos)[0]; has_unlucky_one = 1;
		array_remove(collision_infos, 0);
	} else if (a3 > a4) {
		// a3 wins
		unlucky_one = (collision_infos)[1]; has_unlucky_one = 1;
		array_remove(collision_infos, 1);
	} else {
		// a4 wins
		unlucky_one = (collision_infos)[2]; has_unlucky_one = 1;
		array_remove(collision_infos, 2);
	}

	unlucky_one = chosen;
	array_push(collision_infos, chosen);
}
#else
int get_cache_entry(vec3 cp) {
	int nearest_point = -1;
	r32 shortest_dist = 0.3f;

	for (int i = 0; i < array_length(collision_infos); ++i) {
		Collision_Info* current = &collision_infos[i];
		vec3 collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);

		r32 diff2 = gm_vec3_length(gm_vec3_subtract(collision_point2, cp));
		if (diff2 < shortest_dist) {
			nearest_point = i;
			shortest_dist = diff2;
		}
	}

	return nearest_point;
}

int sort_cached_points(Collision_Info ci) {
	vec3 received_collision_point1 = gm_vec3_add(ci.e1->world_position, ci.r1_wc);
	vec3 received_collision_point2 = gm_vec3_add(ci.e2->world_position, ci.r2_wc);

	int max_penetration_index = -1;

	// calculate max penetration
	vec3* collision_points1 = array_new(vec3);
	vec3* collision_points2 = array_new(vec3);
	r32 deepest_penetration = gm_vec3_dot(gm_vec3_subtract(received_collision_point1, received_collision_point2), ci.normal);
	Collision_Info chosen;
	int chosen_idx = -1;

	for (int i = 0; i < array_length(collision_infos); ++i) {
		Collision_Info* current = &(collision_infos)[i];

		vec3 current_collision_point1 = gm_vec3_add(current->e1->world_position, current->r1_wc);
		vec3 current_collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);
		array_push(collision_points1, current_collision_point1);
		array_push(collision_points2, current_collision_point2);

		r32 current_penetration = gm_vec3_dot(gm_vec3_subtract(current_collision_point1, current_collision_point2), current->normal);

		if (current_penetration > deepest_penetration) {
			deepest_penetration = current_penetration;
			chosen_idx = i;
			chosen = *current;
		}
	}

	r32 res0 = 0.0f, res1 = 0.0f, res2 = 0.0f, res3 = 0.0f;
	if (chosen_idx != 0) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res0 = calc_area_4_points(received_collision_point2, collision_points2[1], collision_points2[2], collision_points2[3]);
	}

	if (chosen_idx != 1) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res1 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[2], collision_points2[3]);
	}

	if (chosen_idx != 2) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res2 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[1], collision_points2[3]);
	}

	if (chosen_idx != 3) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res3 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[1], collision_points2[2]);
	}

	if (res0 > res1 && res0 > res2 && res0 > res3) return 0;
	if (res1 > res2 && res1 > res3) return 1;
	if (res2 > res3) return 2;
	return 3;
}

static void add_collision_info(Collision_Info ci) {
	vec3 received_collision_point2 = gm_vec3_add(ci.e2->world_position, ci.r2_wc);
	int existing_entry = get_cache_entry(received_collision_point2);

	if (existing_entry >= 0) {
		collision_infos[existing_entry] = ci;
		return;
	}

	if (array_length(collision_infos) < 4) {
		array_push(collision_infos, ci);
		return;
	}

	int insert_index = sort_cached_points(ci);
	collision_infos[insert_index] = ci;
}
#endif

static void collect_collision_infos_via_perturbation(Entity* e1, Entity* e2, Collision_Info* ci) {
	Quaternion e1_rotation = e1->world_rotation;
	Quaternion e2_rotation = e2->world_rotation;

	vec3 normal = ci->normal;
	vec3 ortho = find_ortho(normal);

	const r32 PERTURBATION_ANGLE = 5.0f;
	const int NUM_ITERATIONS = 16;
	Quaternion Rp = quaternion_new(ortho, PERTURBATION_ANGLE);
	r32 angle = 360.0f / NUM_ITERATIONS;

	for (int i = 0; i < NUM_ITERATIONS; ++i) {
		Quaternion Rn = quaternion_new(normal, angle * i);
		Quaternion Rn_inv = quaternion_inverse(&Rn);
		Quaternion new_rotation = quaternion_product(&Rn, &e2_rotation);
		new_rotation = quaternion_product(&Rp, &new_rotation);
		new_rotation = quaternion_product(&Rn_inv, &new_rotation);
		graphics_entity_set_rotation(e2, new_rotation);

		graphics_entity_update_bounding_shapes(e1);
		graphics_entity_update_bounding_shapes(e2);
		GJK_Support_List gjk_sl = {0};
		if (collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs)) {
			Collision_Point cp1 = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
			gjk_sl = (GJK_Support_List){0};
			if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
				Collision_Point cp2 = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
				Collision_Info ci = (Collision_Info){0};
				ci.e1 = e1;
				ci.e2 = e2;
				ci.normal = gm_vec3_normalize(normal); // REVIEW, DO WE USE THIS NORMAL OR THE ORIGINAL ONE?

				// we calculate only r1_lc and r2_lc because the cube is artifically rotated
				vec3 r1_wc = gm_vec3_subtract(cp1.collision_point, e1->world_position);
				vec3 r2_wc = gm_vec3_subtract(cp2.collision_point, e2->world_position);

				Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
				mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
				ci.r1_lc = gm_mat3_multiply_vec3(&q1_mat, r1_wc);

				Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
				mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
				ci.r2_lc = gm_mat3_multiply_vec3(&q2_mat, r2_wc);

				// now we calculate r1_wc and r2_wc based on the real rotation
				q1_mat = quaternion_get_matrix3(&e1_rotation);
				ci.r1_wc = gm_mat3_multiply_vec3(&q1_mat, ci.r1_lc);

				q2_mat = quaternion_get_matrix3(&e2_rotation);
				ci.r2_wc = gm_mat3_multiply_vec3(&q2_mat, ci.r2_lc);

				graphics_entity_set_rotation(e1, e1_rotation); // reset for safety
				graphics_entity_set_rotation(e2, e2_rotation); // reset for safety
				add_collision_info(ci);
				//printf("perturbation find collision!\n");
			}
		}
	}

	graphics_entity_set_rotation(e2, e2_rotation);
}

void collect_collisions(Entity* entities) {
	if (collision_infos == NULL) {
		collision_infos = array_new(Collision_Info);
	}

	update_collision_infos();

	// As explained in sec 3.5, in each substep we need to check for collisions
	// (I am not pre-collecting potential collision pairs.)
	// Here we just check the plane-cube collision and collect the intersections.
	for (u32 j = 0; j < array_length(entities); ++j) {
		for (u32 k = j + 1; k < array_length(entities); ++k) {
			Entity* e1 = &entities[j];
			Entity* e2 = &entities[k];

			graphics_entity_update_bounding_shapes(e1);
			graphics_entity_update_bounding_shapes(e2);

			GJK_Support_List gjk_sl = {0};
			if (collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs)) {
				Collision_Point cp1 = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
				gjk_sl = (GJK_Support_List){0};
				if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
					Collision_Point cp2 = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
					Collision_Info ci = (Collision_Info){0};
					ci.e1 = e1;
					ci.e2 = e2;
					ci.normal = gm_vec3_normalize(cp2.normal); // why cp1 though?
					ci.r1_wc = gm_vec3_subtract(cp1.collision_point, e1->world_position);
					ci.r2_wc = gm_vec3_subtract(cp2.collision_point, e2->world_position);

					Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
					mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
					ci.r1_lc = gm_mat3_multiply_vec3(&q1_mat, ci.r1_wc);

					Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
					mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
					ci.r2_lc = gm_mat3_multiply_vec3(&q2_mat, ci.r2_wc);
					add_collision_info(ci);
					unlucky_one = ci; has_unlucky_one = 1;

					//collect_collision_infos_via_perturbation(e1, e2, &ci);
					update_collision_infos();

					//q1_mat = quaternion_get_matrix3(&ci.e1->world_rotation);
					//vec3 ci_r1_wc = gm_mat3_multiply_vec3(&q1_mat, ci.r1_lc);

					//q2_mat = quaternion_get_matrix3(&ci.e2->world_rotation);
					//vec3 ci_r2_wc = gm_mat3_multiply_vec3(&q2_mat, ci.r2_lc);

					//vec3 collision_point1 = gm_vec3_add(ci.e1->world_position, ci_r1_wc);
					//vec3 collision_point2 = gm_vec3_add(ci.e2->world_position, ci_r2_wc);
					////r32 distance = gm_vec3_dot(gm_vec3_subtract(collision_point1, collision_point2), ci.normal);
					//r32 distance = gm_vec3_length(gm_vec3_subtract(collision_point1, collision_point2));
				}
			}
		}
	}
}

void pbd_simulate(r32 dt, Entity* entities) {
	if (dt <= 0.0f) return;
	r32 h = dt / NUM_SUBSTEPS;
	//r32 h = 0.01f;

	// The main loop of the PBD simulation
	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			if (e->fixed) continue;	

			// Calculate the external force and torque of the entity
			vec3 external_force = calculate_external_force(e);
			vec3 external_torque = calculate_external_torque(e);

			// Stores the previous position and orientation of the entity
			e->previous_world_position = e->world_position;
			e->previous_world_rotation = e->world_rotation;

			// Update the entity position and linear velocity based on the current velocity and applied forces
			e->linear_velocity = gm_vec3_add(e->linear_velocity, gm_vec3_scalar_product(h * e->inverse_mass, external_force));
			e->world_position = gm_vec3_add(e->world_position, gm_vec3_scalar_product(h, e->linear_velocity));

			// Update the entity orientation and angular velocity based on the current velocity and applied forces
			mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e);
			mat3 e_inertia_tensor = get_dynamic_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, e->angular_velocity))))));
			Quaternion aux = (Quaternion){e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z, 0.0f};
			Quaternion q = quaternion_product(&aux, &e->world_rotation);
			e->world_rotation.x = e->world_rotation.x + h * 0.5f * q.x;
			e->world_rotation.y = e->world_rotation.y + h * 0.5f * q.y;
			e->world_rotation.z = e->world_rotation.z + h * 0.5f * q.z;
			e->world_rotation.w = e->world_rotation.w + h * 0.5f * q.w;
			// should we normalize?
			e->world_rotation = quaternion_normalize(&e->world_rotation);
		}

		if (collision_infos == NULL) {
			collision_infos = array_new(Collision_Info);
		}

#if 1
		update_collision_infos();

		// As explained in sec 3.5, in each substep we need to check for collisions
		// (I am not pre-collecting potential collision pairs.)
		// Here we just check the plane-cube collision and collect the intersections.
		for (u32 j = 0; j < array_length(entities); ++j) {
			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e1 = &entities[j];
				Entity* e2 = &entities[k];

				graphics_entity_update_bounding_shapes(e1);
				graphics_entity_update_bounding_shapes(e2);

				GJK_Support_List gjk_sl = {0};
				if (collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs)) {
					Collision_Point cp1 = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs);
					gjk_sl = (GJK_Support_List){0};
					if (collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
						Collision_Point cp2 = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs);
						Collision_Info ci = (Collision_Info){0};
						ci.e1 = e1;
						ci.e2 = e2;
						ci.normal = gm_vec3_normalize(cp2.normal); // why cp1 though?
						ci.r1_wc = gm_vec3_subtract(cp1.collision_point, e1->world_position);
						ci.r2_wc = gm_vec3_subtract(cp2.collision_point, e2->world_position);

						Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
						mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
						ci.r1_lc = gm_mat3_multiply_vec3(&q1_mat, ci.r1_wc);

						Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
						mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
						ci.r2_lc = gm_mat3_multiply_vec3(&q2_mat, ci.r2_wc);
						add_collision_info(ci);
						unlucky_one = ci; has_unlucky_one = 1;

						//collect_collision_infos_via_perturbation(e1, e2, &ci);
						update_collision_infos();

						//q1_mat = quaternion_get_matrix3(&ci.e1->world_rotation);
						//vec3 ci_r1_wc = gm_mat3_multiply_vec3(&q1_mat, ci.r1_lc);

						//q2_mat = quaternion_get_matrix3(&ci.e2->world_rotation);
						//vec3 ci_r2_wc = gm_mat3_multiply_vec3(&q2_mat, ci.r2_lc);

						//vec3 collision_point1 = gm_vec3_add(ci.e1->world_position, ci_r1_wc);
						//vec3 collision_point2 = gm_vec3_add(ci.e2->world_position, ci_r2_wc);
						////r32 distance = gm_vec3_dot(gm_vec3_subtract(collision_point1, collision_point2), ci.normal);
						//r32 distance = gm_vec3_length(gm_vec3_subtract(collision_point1, collision_point2));
					}
				}
			}
		}
	#endif

		//collision_infos = collision_get_plane_cube_points(&entities[1], &entities[0]);

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		Constraint* constraints = array_new(Constraint);
		printf("We have %ld collisions\n", array_length(collision_infos));
		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {

			// Before solving the constraints, we need to generate additional constraints if we detected collisions
			for (u32 k = 0; k < array_length(collision_infos); ++k) {
				Collision_Info* ci = &collision_infos[k];
				create_constraints_for_collision(ci, &constraints);
			}

			// Now, solve the constraints
			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
			}	

			// Here we should only clear the constraints generated because of collisions, but for now all of them fall into this category
			array_clear(constraints);
		}

		// The PBD velocity update
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			if (e->fixed) continue;
			
			// We start by storing the current velocities (this is needed for the velocity solver that comes at the end of the loop)
			e->previous_linear_velocity = e->linear_velocity;
			e->previous_angular_velocity = e->angular_velocity;

			// Update the linear velocity based on the position difference
			e->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(e->world_position, e->previous_world_position));

			// Update the angular velocity based on the orientation difference
			Quaternion inv = quaternion_inverse(&e->previous_world_rotation);
			Quaternion delta_q = quaternion_product(&e->world_rotation, &inv);
			if (delta_q.w >= 0.0f) {
				e->angular_velocity = gm_vec3_scalar_product(2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			} else {
				e->angular_velocity = gm_vec3_scalar_product(-2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			}
		}

		// The velocity solver - we run this additional solver for every collision that we found
		for (u32 j = 0; j < array_length(collision_infos); ++j) {
			Collision_Info* ci = &collision_infos[j];
			vec3 v1 = ci->e1->linear_velocity;
			vec3 w1 = ci->e1->angular_velocity;
			vec3 r1_wc = ci->r1_wc;
			vec3 v2 = ci->e2->linear_velocity;
			vec3 w2 = ci->e2->angular_velocity;
			vec3 r2_wc = ci->r2_wc;
			vec3 n = ci->normal;
			r32 lambda_n = ci->lambda_n;
			r32 lambda_t = ci->lambda_t;
			Entity* e1 = ci->e1;
			Entity* e2 = ci->e2;

			// We start by calculating the relative normal and tangential velocities at the contact point, as described in (3.6)
			// @NOTE: equation (29) was modified here
			vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1_wc)), gm_vec3_add(v2, gm_vec3_cross(w2, r2_wc)));
			r32 vn = gm_vec3_dot(n, v);
			vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));

			// delta_v stores the velocity change that we need to perform at the end of the solver
			vec3 delta_v = (vec3){0.0f, 0.0f, 0.0f};
			
			// we start by applying Coloumb's dynamic friction force
			const r32 dynamic_friction_coefficient = 0.5f;
			r32 fn = lambda_n / (h * h);
			// @NOTE: equation (30) was modified here
			r32 fact = MIN(-h * dynamic_friction_coefficient * fn, gm_vec3_length(vt));
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt)));

			// Now we handle restitution
			vec3 old_v1 = e1->previous_linear_velocity;
			vec3 old_w1 = e1->previous_angular_velocity;
			vec3 old_v2 = e2->previous_linear_velocity;
			vec3 old_w2 = e2->previous_angular_velocity;
			vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, r1_wc)), gm_vec3_add(old_v2, gm_vec3_cross(old_w2, r2_wc)));
			r32 vn_til = gm_vec3_dot(n, v_til);
			//r32 e = (fabsf(vn) > 2.0f * GRAVITY * h) ? 0.8f : 0.0f;
			r32 e = 0.0f;
			// @NOTE: equation (34) was modified here
			fact = -vn + MIN(-e * vn_til, 0.0f);
			// update delta_v
			delta_v = gm_vec3_add(delta_v, gm_vec3_scalar_product(fact, n));

			// Finally, we end the solver by applying delta_v, considering the inverse masses of both entities
			mat3 e1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e1);
			mat3 e2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e2);
			r32 _w1 = e1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1_wc, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, n)));
			r32 _w2 = e2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2_wc, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, n)));
			vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
			if (!e1->fixed) {
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(e1->inverse_mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1_wc, p)));
			}
			if (!e2->fixed) {
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(e2->inverse_mass, p));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2_wc, p)));
			}
		}

		array_free(constraints);
		//array_free(collision_infos);
	}
}