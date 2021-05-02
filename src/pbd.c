#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include "collision.h"

#define NUM_SUBSTEPS 1
#define NUM_POS_ITERS 1

static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = constraint->positional_constraint.e1;
	Entity* e2 = constraint->positional_constraint.e2;
	vec3 r1 = constraint->positional_constraint.r1;
	vec3 r2 = constraint->positional_constraint.r2;
	r32 lambda = *constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	vec3 delta_x = constraint->positional_constraint.delta_x_wc;

	vec3 n = gm_vec3_normalize(delta_x);
	r32 c = gm_vec3_length(delta_x);

	r32 w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, n)));
	r32 w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, n)));
	r32 til_compliance = compliance / (h * h);
	r32 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);

	*constraint->positional_constraint.lambda += delta_lambda;

	vec3 positional_impulse = gm_vec3_scalar_product(delta_lambda, n);
	e1->world_position = gm_vec3_add(e1->world_position, gm_vec3_scalar_product(1.0f / e1->mass, positional_impulse));
	e2->world_position = gm_vec3_add(e2->world_position, gm_vec3_scalar_product(-1.0f / e2->mass, positional_impulse));
	vec3 rot1 = gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 rot2 = gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	r32 q1_rotation_angle = gm_vec3_length(rot1);
	vec3 q1_rotation_axis = gm_vec3_normalize(rot1);
	Quaternion q1 = quaternion_new_radians(q1_rotation_axis, q1_rotation_angle);
	r32 q2_rotation_angle = gm_vec3_length(rot2);
	vec3 q2_rotation_axis = gm_vec3_normalize(rot2);
	Quaternion q2 = quaternion_new_radians(q2_rotation_axis, q2_rotation_angle);
	Quaternion inv_q2 = quaternion_inverse(&q2);
	e1->world_rotation = quaternion_product(&e1->world_rotation, &q1);
	e2->world_rotation = quaternion_product(&e2->world_rotation, &inv_q2);
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

static vec3 calculate_external_force(Entity* e) {
    // Calculate total force and torque
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        total_force = gm_vec3_add(total_force, e->forces[i].force);
    }
	return total_force;
}

static vec3 calculate_external_torque(Entity* e) {
    // Calculate total force and torque
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
    }
	return total_torque;
}

void pbd_simulate(r32 dt, Entity* entities) {
	if (dt <= 0.0f) return;
	r32 h = dt / NUM_SUBSTEPS;
	//r32 h = 0.01f;

	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			vec3 external_force = calculate_external_force(e);
			vec3 external_torque = calculate_external_torque(e);
			e->previous_world_position = e->world_position;
			e->linear_velocity = gm_vec3_add(e->linear_velocity, gm_vec3_scalar_product(h / e->mass, external_force));
			e->world_position = gm_vec3_add(e->world_position, gm_vec3_scalar_product(h, e->linear_velocity));

			e->previous_world_rotation = e->world_rotation;
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&e->inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e->inertia_tensor, e->angular_velocity))))));

			// deviated from paper
			r32 rotation_angle = gm_vec3_length(e->angular_velocity) * h;
			vec3 rotation_axis = gm_vec3_normalize(e->angular_velocity);
			Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
			e->world_rotation = quaternion_product(&orientation_change, &e->world_rotation);
		}

		typedef struct {
			Entity* e1;
			Entity* e2;
			vec3 r1;
			vec3 r2;
			vec3 normal;
			r32 lambda_n;
			r32 lambda_t;
		} Collision_Info;

		Collision_Info* collision_infos = array_new(Collision_Info);

		// Collect Collisions
		Collision_Point* pts = collision_get_plane_cube_points(&entities[0], -2.0f);
		for (u32 j = 0; j < array_length(pts); ++j) {
			Collision_Point* cp = &pts[j];
			Collision_Info ci = (Collision_Info){0};
			ci.e1 = &entities[1];
			ci.e2 = &entities[0];
			ci.r1 = (vec3){0.0f, 0.0f, 0.0f};
			ci.r2 = cp->r_lc;
			ci.normal = cp->normal;
			array_push(collision_infos, ci);
		}

		// Solve Constraints
		Constraint* constraints = array_new(Constraint);
		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {

			// Generate constraints per collision
			for (u32 k = 0; k < array_length(collision_infos); ++k) {
				Collision_Info* ci = &collision_infos[k];
				Constraint constraint = {0};
				constraint.type = POSITIONAL_CONSTRAINT;
				constraint.positional_constraint.compliance = 0.0f;
				constraint.positional_constraint.e1 = ci->e1;
				constraint.positional_constraint.e2 = ci->e2;
				constraint.positional_constraint.lambda = &ci->lambda_n;
				constraint.positional_constraint.r1 = ci->r1;
				constraint.positional_constraint.r2 = ci->r2;

				mat3 current_rot_matrix = quaternion_get_matrix3(&ci->e2->world_rotation);
				mat3 previous_rot_matrix = quaternion_get_matrix3(&ci->e2->previous_world_rotation);
				vec3 p2 = gm_vec3_add(ci->e2->world_position, gm_mat3_multiply_vec3(&current_rot_matrix, ci->r2));
				vec3 p2_til = gm_vec3_add(ci->e2->previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, ci->r2));
				vec3 p1 = (vec3){p1.x, -2.0f, p1.z};
				vec3 p1_til = (vec3){p1.x, -2.0f, p1.z};
				r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), ci->normal);
				constraint.positional_constraint.delta_x_wc = gm_vec3_scalar_product(d, ci->normal);
				if (d > 0.0f) {
					array_push(constraints, constraint);

					// static friction
					#if 0
					const r32 static_friction_coefficient = 1.0f;
					vec3 delta_p = gm_vec3_subtract(gm_vec3_subtract(p1, p1_til), gm_vec3_subtract(p2, p2_til));
					vec3 delta_p_t = gm_vec3_subtract(delta_p, gm_vec3_scalar_product(gm_vec3_dot(delta_p, cp->normal), cp->normal));
					Constraint constraint = {0};
					constraint.type = POSITIONAL_CONSTRAINT;
					constraint.positional_constraint.compliance = 0.0f;
					constraint.positional_constraint.e1 = &entities[1];
					constraint.positional_constraint.e2 = &entities[0];
					constraint.positional_constraint.lambda = &cp->lambda_t;
					// r1 and r2 world coords?
					constraint.positional_constraint.r1 = (vec3){0.0f, 0.0f, 0.0f};
					vec3 r2 = cp->r_lc;
					constraint.positional_constraint.r2 = r2;
					constraint.positional_constraint.delta_x_wc = delta_p_t;
					array_push(constraints, constraint);
					#endif
				}
			}

			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
			}

			array_clear(constraints);
		}

		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			e->previous_linear_velocity = e->linear_velocity;
			e->previous_angular_velocity = e->angular_velocity;
			e->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(e->world_position, e->previous_world_position));
			Quaternion inv = quaternion_inverse(&e->previous_world_rotation);
			Quaternion delta_q = quaternion_product(&e->world_rotation, &inv);
			vec3 prev = e->angular_velocity;
			if (delta_q.w >= 0.0f) {
				e->angular_velocity = gm_vec3_scalar_product(2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			} else {
				e->angular_velocity = gm_vec3_scalar_product(-2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			}
			printf("previous angular_velocity = <%.3f, %.3f, %.3f>\n", prev.x, prev.y, prev.z);
			printf("new angular_velocity = <%.3f, %.3f, %.3f>\n", e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z);
			graphics_entity_recalculate_model_matrix(&entities[j]);
		}

		// solve velocities
		#if 1
		for (u32 j = 0; j < array_length(collision_infos); ++j) {
			Collision_Info* ci = &collision_infos[j];
			vec3 v1 = ci->e1->linear_velocity;
			vec3 w1 = ci->e1->angular_velocity;
			vec3 r1 = ci->r1;
			vec3 v2 = ci->e2->linear_velocity;
			vec3 w2 = ci->e2->angular_velocity;
			vec3 r2 = ci->r2;
			vec3 n = ci->normal;
			r32 lambda_n = ci->lambda_n;
			r32 lambda_t = ci->lambda_t;
			Entity* e1 = ci->e1;
			Entity* e2 = ci->e2;

			vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1)), gm_vec3_subtract(v2, gm_vec3_cross(w2, r2)));
			r32 vn = gm_vec3_dot(n, v);
			vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));
			
			#if 0
			{
				const r32 dynamic_friction_coefficient = 0.2f;
				r32 fn = lambda_n / (h * h);
				r32 fact = MIN(h * dynamic_friction_coefficient * fn, gm_vec3_length(vt));
				vec3 delta_v = gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt));

				r32 _w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, n)));
				r32 _w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, n)));
				vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(1.0f / e1->mass, p));
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(1.0f / e2->mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, p)));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, p)));
			}
			#endif

			#if 0
			{
				vec3 old_v1 = e1->previous_linear_velocity;
				vec3 old_w1 = e1->previous_angular_velocity;
				vec3 old_v2 = e2->previous_linear_velocity;
				vec3 old_w2 = e2->previous_angular_velocity;
				vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, r1)), gm_vec3_subtract(old_v2, gm_vec3_cross(old_w2, r2)));
				r32 vn_til = gm_vec3_dot(n, v_til);
				r32 e = 1.0f;
				r32 fact = -vn + MAX(-e * vn_til, 0.0f);
				vec3 delta_v = gm_vec3_scalar_product(fact, n);

				r32 _w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, n)));
				r32 _w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, n)));
				vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(1.0f / e1->mass, p));
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(1.0f / e2->mass, p));
				//e2->linear_velocity = gm_vec3_scalar_product(vn_til, n);
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1->inverse_inertia_tensor, gm_vec3_cross(r1, p)));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2->inverse_inertia_tensor, gm_vec3_cross(r2, p)));
			}
			#endif

			printf("e2->linear_velocity = <%.3f, %.3f, %.3f>\n", e2->linear_velocity.x, e2->linear_velocity.y, e2->linear_velocity.z);
			printf("e2->angular_velocity = <%.3f, %.3f, %.3f>\n", e2->angular_velocity.x, e2->angular_velocity.y, e2->angular_velocity.z);
		}
		#endif

		array_free(pts);
		array_free(constraints);
		array_free(collision_infos);
	}
}