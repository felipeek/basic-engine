#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include "collision.h"

#define NUM_SUBSTEPS 20
#define NUM_POS_ITERS 1

extern Entity* entities;

static r64 get_angle_between_two_vectors(xvec3 a, xvec3 b) {
	r64 cos_angle = gm_xvec3_dot(a, b) / (gm_xvec3_length(a) * gm_xvec3_length(b));
	return acos(MIN(1.0, MAX(-1.0, cos_angle)));
}

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
static vec3 calculate_external_torque(Particle* p, vec3 center_of_mass, vec3 position) {
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; p->forces && i < array_length(p->forces); ++i) {
        vec3 distance = gm_vec3_subtract(position, center_of_mass);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, p->forces[i].force));
    }
	return total_torque;
}

// Calculate the dynamic inertia tensor of an entity, i.e., the inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inertia_tensor(Entity* e) {
	mat3 inertia_tensor = graphics_get_symmetric_dynamic_inertia_tensor_for_particles(e->particles);
	return inertia_tensor;
}

// Calculate the dynamic inverse inertia tensor of an entity, i.e., the inverse inertia tensor transformed considering entity's rotation
static mat3 get_dynamic_inverse_inertia_tensor(Entity* e) {
	mat3 inertia_tensor = graphics_get_symmetric_dynamic_inertia_tensor_for_particles(e->particles);
	mat3 inverse;
	assert(gm_mat3_inverse(&inertia_tensor, &inverse));
	return inverse;
}

// Solves the positional constraint, updating the position and orientation of the entities accordingly
static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Particle* p1 = constraint->positional_constraint.p1;
	Particle* p2 = constraint->positional_constraint.p2;
	//vec3 r1 = (vec3){0.0f, 0.0f, 0.0f};
	//vec3 r2 = (vec3){0.0f, 0.0f, 0.0f};
	vec3 r1 = constraint->positional_constraint.r1;	// in world coordinates
	vec3 r2 = constraint->positional_constraint.r2; // in world coordinates
	r32 lambda = constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	//vec3 delta_x = constraint->positional_constraint.delta_x;
	r32 delta_x = constraint->positional_constraint.delta_x;

	//mat3 p1_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(p1);
	//mat3 p2_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(p2);

	// split delta_x into n and c.
	//vec3 n = gm_vec3_normalize(delta_x);
	//r32 c = gm_vec3_length(delta_x);
	
	vec3 particle_distance = gm_vec3_subtract(p2->world_position, p1->world_position);
	//vec3 n = gm_vec3_normalize(particle_distance);
	//r32 c = delta_x - gm_vec3_length(particle_distance);
	vec3 diff = gm_vec3_subtract(gm_vec3_scalar_product(delta_x, gm_vec3_normalize(particle_distance)), particle_distance);
	vec3 n = gm_vec3_normalize(diff);
	r32 c = gm_vec3_length(diff);

	if (c < 0.00000001f) {
		return;
	}

	// calculate the inverse masses of both entities
	r32 w1 = p1->inverse_mass;
	r32 w2 = p2->inverse_mass;

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

void pbd_simulate(r32 dt, Entity* entities) {
	if (dt <= 0.0f) return;
	r32 h = dt / NUM_SUBSTEPS;

	// The main loop of the PBD simulation
	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			//e->center_of_mass = graphics_entity_get_center_of_mass(e);

			// Calculate the total torque applied to the entity by inspecting the force applied to all particles
			vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
			for (u32 k = 0; k < array_length(e->particles); ++k) {
				Particle* p = e->particles[k];
				if (p->fixed) continue;	

				vec3 external_torque = calculate_external_torque(p, gm_xvec3_to_vec3(e->center_of_mass), p->world_position);
					
				total_torque = gm_vec3_add(total_torque, external_torque);
			}

			// Update the angular velocity of the entity based on the torque
			mat3 e_inverse_inertia_tensor = get_dynamic_inverse_inertia_tensor(e);
			mat3 e_inertia_tensor = get_dynamic_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e_inverse_inertia_tensor, gm_vec3_subtract(total_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e_inertia_tensor, e->angular_velocity))))));

			// Calculate the rotation to be applied to the particles
			r32 angular_velocity_length = gm_vec3_length(e->angular_velocity);
			Quaternion rotation = quaternion_new_radians(gm_vec3_normalize(e->angular_velocity), angular_velocity_length * h);
			mat3 rot_matrix = quaternion_get_matrix3(&rotation);

			// Apply the rotation to each particle
			for (u32 k = 0; k < array_length(e->particles); ++k) {
				Particle* p = e->particles[k];

				if (p->fixed) continue;	

				xvec3 r = gm_xvec3_subtract(gm_vec3_to_xvec3(p->world_position), e->center_of_mass);
				if (angular_velocity_length > 0.000000001f) {
					xvec3 r_rotated = gm_mat3_multiply_xvec3(&rot_matrix, r);
					p->world_position = gm_vec3_add(gm_xvec3_to_vec3(e->center_of_mass), gm_xvec3_to_vec3(r_rotated));

					p->r_before_rotation = r;
					p->r_after_rotation = r_rotated;
				} else {
					p->r_before_rotation = r;
					p->r_after_rotation = r;
				}
			}

			// Linear velocity and position update
			for (u32 k = 0; k < array_length(e->particles); ++k) {
				Particle* p = e->particles[k];

				p->previous_world_position = p->world_position;

				if (p->fixed) continue;	

				// Calculate the external force and torque of the entity
				vec3 external_force = calculate_external_force(p);

				// Update the entity position and linear velocity based on the current velocity and applied forces
				p->linear_velocity = gm_vec3_add(p->linear_velocity, gm_vec3_scalar_product(h * p->inverse_mass, external_force));
				p->world_position = gm_vec3_add(p->world_position, gm_vec3_scalar_product(h, p->linear_velocity));
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
				create_constraints_for_particles(e->connections, &constraints, gm_xvec3_to_vec3(e->center_of_mass));
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

			xvec3 initial_center_of_mass = e->center_of_mass;
			xvec3 new_center_of_mass = graphics_entity_get_center_of_mass(e);
			e->center_of_mass = new_center_of_mass;
			r64 angular_velocity_diff = 0.0f;
			u32 num = 0;

			for (u32 k = 0; k < array_length(e->particles); ++k) {
				Particle* p = e->particles[k];
				if (p->fixed) continue;

				// Update the linear velocity based on the position difference
				p->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(p->world_position, p->previous_world_position));

				xvec3 current_r = gm_xvec3_subtract(gm_vec3_to_xvec3(p->world_position), new_center_of_mass);

				const r32 EPSILON = 0.00001f;
				if (gm_xvec3_length(p->r_before_rotation) < EPSILON) {
					// particle in the middle of the cube matches the center of mass so the radius is defined by floating pointing errors
					continue;
				}

				xvec3 p1 = (xvec3){0.0, 0.0, 0.0};
				xvec3 p2 = p->r_before_rotation;
				xvec3 p3 = p->r_after_rotation;

				xvec3 plane_normal = gm_xvec3_normalize(gm_xvec3_cross(gm_xvec3_subtract(p2, p1), gm_xvec3_subtract(p3, p1)));
				xvec3 projected_current_r = collision_project_point_onto_plane(current_r, plane_normal, p1);

				++num;

				//r32 diff = gm_vec3_length(gm_vec3_subtract(projected_current_r, p->r_after_rotation));
				////printf("diff: %f\n", diff);
				//if (diff < EPSILON) {
				//	continue;
				//}
				
				r64 orig_angle = get_angle_between_two_vectors(p->r_after_rotation, p->r_before_rotation);
				r64 new_angle = get_angle_between_two_vectors(projected_current_r, p->r_before_rotation);

				if (orig_angle != new_angle) {
					//printf("AJSdoasjdas");
				}
					
				r64 angle_diff = orig_angle - new_angle;
				angular_velocity_diff += MAX(0.0, angle_diff);


				//if (MIN(1.0, (new_angle / orig_angle)) < 0.99) {
				//	//printf("ksadjas");
				//}
				//if (orig_angle != 0.0f) {
				//	angular_velocity_multiplier += MIN(1.0, (new_angle / orig_angle));
				//} else {
				//	angular_velocity_multiplier += 1.0f;
				//}

				//printf("Orig angle: %f\nNew Angle: %f\n", orig_angle, new_angle);
			}

			//angular_velocity_multiplier /= num;
			angular_velocity_diff /= num;
			//e->angular_velocity = gm_vec3_scalar_product(angular_velocity_multiplier, e->angular_velocity);
			//printf("multipler: %f\n", angular_velocity_multiplier);
			e->angular_velocity = gm_vec3_scalar_product(1.0f - angular_velocity_diff / h, e->angular_velocity);
			printf("angular velocity: <%.5f, %.5f, %.5f>\n", e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z);
			//printf("angular_velocity_diff: %lf\n", angular_velocity_diff);
		}

		array_free(constraints);
		//array_free(collision_infos);
	}
}