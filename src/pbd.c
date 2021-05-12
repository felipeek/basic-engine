#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include "collision.h"

#define NUM_SUBSTEPS 1
#define NUM_POS_ITERS 1

// Calculate the sum of all external forces acting on an entity
static vec3 calculate_external_force(Particle* p) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; p->forces && i < array_length(p->forces); ++i) {
        total_force = gm_vec3_add(total_force, p->forces[i].force);
    }
	return total_force;
}

// Calculate the sum of all external torques acting on an entity
static vec3 calculate_external_torque(Particle* p) {
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; p->forces && i < array_length(p->forces); ++i) {
        vec3 distance = gm_vec3_subtract(p->forces[i].position, center_of_mass);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, p->forces[i].force));
    }
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inertia_tensor(Particle* p) {
	return gm_mat3_identity();
    //mat4 rotation_matrix = quaternion_get_matrix(&p->world_rotation);
    //mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    //mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    //mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &p->inertia_tensor);
    //return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inverse_inertia_tensor(Particle* p) {
	return gm_mat3_identity();
    //mat4 rotation_matrix = quaternion_get_matrix(&p->world_rotation);
    //mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    //mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    //mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &p->inverse_inertia_tensor);
    //return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
}

// Solves the positional constraint, updating the position and orientation of the entities accordingly
static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Particle* p1 = constraint->positional_constraint.p1;
	Particle* p2 = constraint->positional_constraint.p2;
	mat3 p1_rot = quaternion_get_matrix3(&p1->world_rotation);
	mat3 p2_rot = quaternion_get_matrix3(&p2->world_rotation);
	vec3 r1 = constraint->positional_constraint.r1;	// in world coordinates
	vec3 r2 = constraint->positional_constraint.r2; // in world coordinates
	r32 lambda = constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	//vec3 delta_x = constraint->positional_constraint.delta_x;
	r32 delta_x = constraint->positional_constraint.delta_x;

	mat3 p1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(p1);
	mat3 p2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(p2);

	// split delta_x into n and c.
	//vec3 n = gm_vec3_normalize(delta_x);
	//r32 c = gm_vec3_length(delta_x);
	
	vec3 particle_distance = gm_vec3_subtract(p2->world_position, p1->world_position);
	vec3 n = gm_vec3_normalize(particle_distance);
	r32 c = delta_x - gm_vec3_length(particle_distance);

	// calculate the inverse masses of both entities
	r32 w1 = p1->inverse_mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&p1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
	r32 w2 = p2->inverse_mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&p2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));

	// calculate the delta_lambda (XPBD) and updates the constraint
	r32 til_compliance = compliance / (h * h);
	r32 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);
	constraint->positional_constraint.lambda += delta_lambda;

	// calculates the positional impulse
	vec3 positional_impulse = gm_vec3_scalar_product(delta_lambda, n);

	// updates the position of the entities based on eq (6) and (7)
	if (!p1->fixed) {
		p1->world_position = gm_vec3_add(p1->world_position, gm_vec3_scalar_product(p1->inverse_mass, positional_impulse));
	}
	if (!p2->fixed) {
		p2->world_position = gm_vec3_add(p2->world_position, gm_vec3_scalar_product(-p2->inverse_mass, positional_impulse));
	}

	// updates the rotation of the entities based on eq (8) and (9)
	vec3 aux1 = gm_mat3_multiply_vec3(&p1_inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 aux2 = gm_mat3_multiply_vec3(&p2_inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	Quaternion aux_q1 = (Quaternion){aux1.x, aux1.y, aux1.z, 0.0f};
	Quaternion aux_q2 = (Quaternion){aux2.x, aux2.y, aux2.z, 0.0f};
	Quaternion q1 = quaternion_product(&aux_q1, &p1->world_rotation);
	Quaternion q2 = quaternion_product(&aux_q2, &p2->world_rotation);
	if (!p1->fixed) {
		p1->world_rotation.x = p1->world_rotation.x + 0.5f * q1.x;
		p1->world_rotation.y = p1->world_rotation.y + 0.5f * q1.y;
		p1->world_rotation.z = p1->world_rotation.z + 0.5f * q1.z;
		p1->world_rotation.w = p1->world_rotation.w + 0.5f * q1.w;
		// should we normalize?
		p1->world_rotation = quaternion_normalize(&p1->world_rotation);
	}
	if (!p2->fixed) {
		p2->world_rotation.x = p2->world_rotation.x - 0.5f * q2.x;
		p2->world_rotation.y = p2->world_rotation.y - 0.5f * q2.y;
		p2->world_rotation.z = p2->world_rotation.z - 0.5f * q2.z;
		p2->world_rotation.w = p2->world_rotation.w - 0.5f * q2.w;
		// should we normalize?
		p2->world_rotation = quaternion_normalize(&p2->world_rotation);
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

static void create_constraints_for_particles(Particle_Connection* connections, Constraint** constraints, vec3 center_of_mass) {
	for (u32 i = 0; i < array_length(connections); ++i) {
		Particle_Connection* conn = &connections[i];
		Particle* p1 = conn->p1;
		Particle* p2 = conn->p2;
		vec3 r1 = gm_vec3_subtract(p1->world_position, center_of_mass);
		vec3 r2 = gm_vec3_subtract(p2->world_position, center_of_mass);
		r32 delta_x = conn->distance;

		Constraint c;
		c.type = POSITIONAL_CONSTRAINT;
		c.positional_constraint.p1 = p1;
		c.positional_constraint.p2 = p2;
		c.positional_constraint.r1 = r1;
		c.positional_constraint.r2 = r2;
		c.positional_constraint.lambda = 0.0f;
		c.positional_constraint.compliance = 0.0f;
		c.positional_constraint.delta_x = delta_x;

		array_push(*constraints, c);
	}
}

/*
// Create 'artificial' position constraints for a given collision that was detected in the simulation
static void create_constraints_for_collision(Collision_Info* ci, Constraint** constraints) {
	Constraint constraint = {0};
	constraint.type = POSITIONAL_CONSTRAINT;
	constraint.positional_constraint.compliance = 0.0f; // as stated in sec 3.5, compliance is 0 for collision constraints
	constraint.positional_constraint.p1 = ci->e1;
	constraint.positional_constraint.p2 = ci->e2;
	// we make lambda a reference to the value stored in the collision_info, so we can accumulate per iteration
	constraint.positional_constraint.lambda = &ci->lambda_n;
	constraint.positional_constraint.r1 = ci->r1_wc;
	constraint.positional_constraint.r2 = ci->r2_wc;

	// here we calculate 'p1' and 'p2' in order to calculate 'd', as stated in sec (3.5)
	mat3 current_rot_matrix = quaternion_get_matrix3(&ci->e2->world_rotation);
	mat3 previous_rot_matrix = quaternion_get_matrix3(&ci->e2->previous_world_rotation);
	vec3 p2 = gm_vec3_add(ci->e2->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, ci->r2_lc));
	vec3 p1 = (vec3){p2.x, PLANE_Y, p2.z}; // we consider an infinite plane, so we just project the cube vertex onto the plane
	r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), ci->normal);
	constraint.positional_constraint.delta_x = gm_vec3_scalar_product(d, ci->normal);
	if (d > 0.0f) {
		// if 'd' is greater than 0.0, we add the constraint.
		array_push(*constraints, constraint);

		// if 'd' is greater than 0.0, we should also add a constraint for static friction, but only if lambda_t < u_s * lambda_n
		// the problem is that if NUM_POS_ITERS is 1, lambda_t and lambda_n will always be 0.0, so this will never be used.
		const r32 static_friction_coefficient = 0.5f;
		if (ci->lambda_t < static_friction_coefficient * ci->lambda_n) {
			vec3 p2_til = gm_vec3_add(ci->e2->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, ci->r2_lc));
			vec3 p1_til = (vec3){p2.x, PLANE_Y, p2.z};
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
*/

void pbd_simulate(r32 dt, Entity* entities) {
	if (dt <= 0.0f) return;
	r32 h = dt / NUM_SUBSTEPS;
	//r32 h = 0.001f;

	// The main loop of the PBD simulation
	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			e->center_of_mass = graphics_entity_get_center_of_mass(e);

			for (u32 k = 0; k < array_length(e->particles); ++k) {
				Particle* p = e->particles[k];
				if (p->fixed) continue;	

				// Calculate the external force and torque of the entity
				vec3 external_force = calculate_external_force(p);
				vec3 external_torque = calculate_external_torque(p);

				// Stores the previous position and orientation of the entity
				p->previous_world_position = p->world_position;
				p->previous_world_rotation = p->world_rotation;

				// Update the entity position and linear velocity based on the current velocity and applied forces
				p->linear_velocity = gm_vec3_add(p->linear_velocity, gm_vec3_scalar_product(h * p->inverse_mass, external_force));
				p->world_position = gm_vec3_add(p->world_position, gm_vec3_scalar_product(h, p->linear_velocity));

				// Update the entity orientation and angular velocity based on the current velocity and applied forces
				mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(p);
				mat3 e_inertia_tensor = get_dynamic_inertia_tensor(p);
				p->angular_velocity = gm_vec3_add(p->angular_velocity, gm_vec3_scalar_product(h, 
					gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(external_torque,
					gm_vec3_cross(p->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, p->angular_velocity))))));
				Quaternion aux = (Quaternion){p->angular_velocity.x, p->angular_velocity.y, p->angular_velocity.z, 0.0f};
				Quaternion q = quaternion_product(&aux, &p->world_rotation);
				p->world_rotation.x = p->world_rotation.x + h * 0.5f * q.x;
				p->world_rotation.y = p->world_rotation.y + h * 0.5f * q.y;
				p->world_rotation.z = p->world_rotation.z + h * 0.5f * q.z;
				p->world_rotation.w = p->world_rotation.w + h * 0.5f * q.w;
				// should we normalize?
				p->world_rotation = quaternion_normalize(&p->world_rotation);
			}
		}

		// As explained in sec 3.5, in each substep we need to check for collisions
		// (I am not pre-collecting potential collision pairs.)
		// Here we just check the plane-cube collision and collect the intersections.
		//Collision_Info* collision_infos = collision_collect_collisions(&entities[0], &entities[1]);

		// Now we run the PBD solver with NUM_POS_ITERS iterations
		Constraint* constraints = array_new(Constraint);
		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {

			//// Before solving the constraints, we need to generate additional constraints if we detected collisions
			//for (u32 k = 0; k < array_length(collision_infos); ++k) {
			//	Collision_Info* ci = &collision_infos[k];
			//	create_constraints_for_collision(ci, &constraints);
			//}

			for (u32 k = 0; k < array_length(entities); ++k) {
				Entity* e = &entities[k];
				create_constraints_for_particles(e->connections, &constraints, e->center_of_mass);
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
			for (u32 k = 0; k < array_length(entities->particles); ++k) {
				Particle* p = entities->particles[k];
				if (p->fixed) continue;
				
				// We start by storing the current velocities (this is needed for the velocity solver that comes at the end of the loop)
				p->previous_linear_velocity = p->linear_velocity;
				p->previous_angular_velocity = p->angular_velocity;

				// Update the linear velocity based on the position difference
				p->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(p->world_position, p->previous_world_position));

				// Update the angular velocity based on the orientation difference
				Quaternion inv = quaternion_inverse(&p->previous_world_rotation);
				Quaternion delta_q = quaternion_product(&p->world_rotation, &inv);
				if (delta_q.w >= 0.0f) {
					p->angular_velocity = gm_vec3_scalar_product(2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
				} else {
					p->angular_velocity = gm_vec3_scalar_product(-2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
				}
			}
		}

/*
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
			r32 e = 0.1f;
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
*/

		array_free(constraints);
		//array_free(collision_infos);
	}
}