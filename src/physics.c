#include "physics.h"
#include "collision.h"
#include <light_array.h>

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

static void physics_update_momenta_based_on_forces(Entity* e, r32 dt, Physics_Force* forces) {
    // Calculate total force and torque
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; forces && i < array_length(forces); ++i) {
        vec3 distance = gm_vec3_subtract(forces[i].position, center_of_mass);
        total_force = gm_vec3_add(total_force, forces[i].force);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, forces[i].force));
    }
    vec3 angular_momentum_change = gm_vec3_scalar_product(dt, total_torque);
    vec3 linear_momentum_change = gm_vec3_scalar_product(dt, total_force);
    e->linear_momentum = gm_vec3_add(e->linear_momentum, linear_momentum_change);
    e->angular_momentum = gm_vec3_add(e->angular_momentum, angular_momentum_change);
}

extern vec3 col_point;
extern boolean collision;
extern vec3 penetration;

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

#if 1
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
	//t = gm_vec3_normalize(gm_vec3_subtract(t, gm_vec3_scalar_product(gm_vec3_dot(t, cp->normal), cp->normal)));
	t.y = 0.0f;

    const r32 static_coefficient = 1.0f;
    const r32 dynamic_coefficient = 1.0f;
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
		moving_body->linear_momentum = (vec3){0};
		moving_body->angular_momentum = (vec3){0};
	}
}

void physics_simulate(Entity* entities, r32 dt) {

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
					apply_impulse(e1, e2, &cp, e1->forces, 0.1f);


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
						apply_impulse(e2, e1, &cp, e1->forces, 0.1f);
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
}