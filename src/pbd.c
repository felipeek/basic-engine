#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include <float.h>
#include "collision.h"
#include "gjk.h"
#include "pmc.h"
#include <hash_map.h>

#define NUM_SUBSTEPS 10
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
	mat3 previous_rot_matrix = quaternion_get_matrix3(&e->previous_world_rotation);
	return gm_vec3_add(e->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, r_lc));
}
static vec3 calculate_p(Entity* e, vec3 r_lc) {
	mat3 current_rot_matrix = quaternion_get_matrix3(&e->world_rotation);
	return gm_vec3_add(e->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, r_lc));
}

// Create 'artificial' position constraints for a given collision that was detected in the simulation
static void create_constraints_for_contacts(PMC* pmc, Constraint** constraints) {
	for (u32 i = 0; i < array_length(pmc->contacts); ++i) {
		PMC_Contact* contact = &pmc->contacts[i];

		Constraint constraint = {0};
		constraint.type = POSITIONAL_CONSTRAINT;
		constraint.positional_constraint.compliance = 0.0f; // as stated in sec 3.5, compliance is 0 for collision constraints
		constraint.positional_constraint.e1 = contact->e1;
		constraint.positional_constraint.e2 = contact->e2;
		// we make lambda a reference to the value stored in the collision_info, so we can accumulate per iteration
		constraint.positional_constraint.lambda = &contact->lambda_n;
		constraint.positional_constraint.r1 = contact->r1_wc;
		constraint.positional_constraint.r2 = contact->r2_wc;

		// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
		vec3 p1 = calculate_p(contact->e1, contact->r1_lc);
		vec3 p2 = calculate_p(contact->e2, contact->r2_lc);
		r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), contact->normal);
		constraint.positional_constraint.delta_x = gm_vec3_scalar_product(d, contact->normal);
		if (d > 0.0f) {
			// if 'd' is greater than 0.0, we add the constraint.
			array_push(*constraints, constraint);

			// if 'd' is greater than 0.0, we should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
			// the problem is that if NUM_POS_ITERS is 1, lambda_t and lambda_n will always be 0.0, so this will never be used.
			const r32 static_friction_coefficient = 0.5f;
			if (contact->lambda_t < static_friction_coefficient * contact->lambda_n) {
				vec3 p1_til = calculate_p_til(contact->e1, contact->r1_lc);
				vec3 p2_til = calculate_p_til(contact->e2, contact->r2_lc);
				vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
				vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(gm_vec3_dot(delta_p, contact->normal), contact->normal));
				Constraint constraint = {0};
				constraint.type = POSITIONAL_CONSTRAINT;
				constraint.positional_constraint.compliance = 0.0f;
				constraint.positional_constraint.e1 = contact->e1;
				constraint.positional_constraint.e2 = contact->e2;
				constraint.positional_constraint.lambda = &contact->lambda_t;
				constraint.positional_constraint.r1 = contact->r1_wc;
				constraint.positional_constraint.r2 = contact->r2_wc;
				constraint.positional_constraint.delta_x = delta_p_t;
				array_push(*constraints, constraint);
			}
		}
	}
}

extern int paused;

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

		pmc_clear_all();
		pmc_update();
#if 1
		// As explained in sec 3.5, in each substep we need to check for collisions
		// (I am not pre-collecting potential collision pairs.)
		// Here we just check the plane-cube collision and collect the intersections.
		for (u32 j = 0; j < array_length(entities); ++j) {
			for (u32 k = j + 1; k < array_length(entities); ++k) {
				Entity* e1 = &entities[j];
				Entity* e2 = &entities[k];

				graphics_entity_update_bounding_shapes(e1);
				graphics_entity_update_bounding_shapes(e2);

				Collision_Point cp;
				GJK_Support_List gjk_sl = {0};
				if (collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs)) {
					boolean found_cp = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs, &cp);
					if (found_cp) {
						PMC_Contact* contacts = collision_get_convex_convex_points(e1, e2, cp.normal);
						for (u32 l = 0; l < array_length(contacts); ++l) {
							pmc_add(contacts[l]);
							//paused = true;
						}
						//pmc_perturb(e1, e2, contact.normal);
						//pmc_update(); // check if necessary
					}
				}
			}
		}
#else
		for (u32 k = 1; k < array_length(entities); ++k) {
			PMC_Contact* contacts = collision_get_plane_cube_points(&entities[k], &entities[0]);
			for (u32 j = 0; j < array_length(contacts); ++j) {
				pmc_add(contacts[j]);
			}
		}
#endif

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		Constraint* constraints = array_new(Constraint);

		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {
			PMC_Map_Key key;
			PMC* pmc;

			// Before solving the constraints, we need to generate additional constraints if we detected collisions
			Hash_Map_Iterator iterator = hash_map_get_iterator(&pmc_map);
			while ((iterator = hash_map_iterator_next(&pmc_map, iterator, &key, &pmc)) != HASH_MAP_ITERATOR_END) {
				create_constraints_for_contacts(pmc, &constraints);
			}

			// Now, solve the constraints

#if 0
			typedef struct {
				Constraint c;
				r32 height;
			} Constraint_And_Height;

			Constraint_And_Height* chs = array_new(Constraint_And_Height);

			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				vec3 w1 = constraint->positional_constraint.e1->world_position;
				vec3 w2 = constraint->positional_constraint.e2->world_position;
				r32 height = gm_vec3_add(w1, w2).y;

				Constraint_And_Height ch;
				ch.c = *constraint;
				ch.height = height;
				array_push(chs, ch);
			}

			for (s32 k = 0; k < (s32)array_length(chs) - 1; ++k) {
				for (s32 l = 0; l < (s32)array_length(chs) - k - 1; ++l) {
					if (chs[l].height < chs[l + 1].height) {
						Constraint_And_Height aux = chs[l];
						chs[l] = chs[l + 1];
						chs[l + 1] = aux;
					}
				}
			}

			array_clear(constraints);

			for (u32 k = 0; k < array_length(chs); ++k) {
				Constraint_And_Height* ch = &chs[k];
				array_push(constraints, ch->c);
			}	
#endif

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
		PMC_Map_Key key;
		PMC* pmc;
		Hash_Map_Iterator iterator = hash_map_get_iterator(&pmc_map);
		while ((iterator = hash_map_iterator_next(&pmc_map, iterator, &key, &pmc)) != HASH_MAP_ITERATOR_END) {
			for (u32 j = 0; j < array_length(pmc->contacts); ++j) {
				PMC_Contact* contact = &pmc->contacts[j];
				vec3 v1 = contact->e1->linear_velocity;
				vec3 w1 = contact->e1->angular_velocity;
				vec3 r1_wc = contact->r1_wc;
				vec3 v2 = contact->e2->linear_velocity;
				vec3 w2 = contact->e2->angular_velocity;
				vec3 r2_wc = contact->r2_wc;
				vec3 n = contact->normal;
				r32 lambda_n = contact->lambda_n;
				r32 lambda_t = contact->lambda_t;
				Entity* e1 = contact->e1;
				Entity* e2 = contact->e2;

				// We start by calculating the relative normal and tangential velocities at the contact point, as described in (3.6)
				// @NOTE: equation (29) was modified here
				vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1_wc)), gm_vec3_add(v2, gm_vec3_cross(w2, r2_wc)));
				r32 vn = gm_vec3_dot(n, v);
				vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));

				// delta_v stores the velocity change that we need to perform at the end of the solver
				vec3 delta_v = (vec3){0.0f, 0.0f, 0.0f};
				
				// we start by applying Coloumb's dynamic friction force
				const r32 dynamic_friction_coefficient = 1.0f;
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
				r32 e = 0.2f;
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
		}

		array_free(constraints);
		//array_free(collision_infos);
	}
}