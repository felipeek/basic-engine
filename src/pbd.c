#include "pbd.h"
#include <light_array.h>
#include <assert.h>
#include "collision.h"

#define NUM_SUBSTEPS 10
#define NUM_POS_ITERS 1

static mat3 get_inertia_tensor(Entity* e) {
#if 0
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	return e->inertia_tensor;
#endif
}

static mat3 get_inverse_inertia_tensor(Entity* e) {
#if 0
    mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &e->inverse_inertia_tensor);
    return gm_mat3_multiply(&aux, &transposed_rotation_matrix);
#else
	return e->inverse_inertia_tensor;
#endif
}

static void solve_positional_constraint(Constraint* constraint, r32 h) {
	assert(constraint->type == POSITIONAL_CONSTRAINT);

	Entity* e1 = constraint->positional_constraint.e1;
	Entity* e2 = constraint->positional_constraint.e2;
	vec3 r1 = constraint->positional_constraint.r1_wc;
	vec3 r2 = constraint->positional_constraint.r2_wc;
	r32 lambda = constraint->positional_constraint.lambda;
	r32 compliance = constraint->positional_constraint.compliance;
	vec3 delta_x = constraint->positional_constraint.delta_x_wc;

	mat3 e1_inverse_inertia_tensor = get_inverse_inertia_tensor(e1);
	mat3 e2_inverse_inertia_tensor = get_inverse_inertia_tensor(e2);

	vec3 n = gm_vec3_normalize(delta_x);
	r32 c = gm_vec3_length(delta_x);

	r32 w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
	r32 w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));
	r32 til_compliance = compliance / (h * h);
	r32 delta_lambda = (- c - til_compliance * lambda) / (w1 + w2 + til_compliance);

	constraint->positional_constraint.lambda += delta_lambda;

	vec3 positional_impulse = gm_vec3_scalar_product(delta_lambda, n);
	#if 0
	e1->world_position = gm_vec3_add(e1->world_position, gm_vec3_scalar_product(1.0f / e1->mass, positional_impulse));
	e2->world_position = gm_vec3_add(e2->world_position, gm_vec3_scalar_product(-1.0f / e2->mass, positional_impulse));
	vec3 rot1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 rot2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	e1->world_rotation.x += 0.5f * rot1.x * e1->world_rotation.x;
	e1->world_rotation.y += 0.5f * rot1.y * e1->world_rotation.y;
	e1->world_rotation.z += 0.5f * rot1.z * e1->world_rotation.z;
	e2->world_rotation.x -= 0.5f * rot2.x * e2->world_rotation.x;
	e2->world_rotation.y -= 0.5f * rot2.y * e2->world_rotation.y;
	e2->world_rotation.z -= 0.5f * rot2.z * e2->world_rotation.z;
	#else
	e1->world_position = gm_vec3_add(e1->world_position, gm_vec3_scalar_product(1.0f / e1->mass, positional_impulse));
	e2->world_position = gm_vec3_add(e2->world_position, gm_vec3_scalar_product(-1.0f / e2->mass, positional_impulse));
	vec3 rot1 = gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, positional_impulse));
	vec3 rot2 = gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, positional_impulse));
	r32 q1_rotation_angle = gm_vec3_length(rot1);
	vec3 q1_rotation_axis = gm_vec3_normalize(rot1);
	Quaternion q1 = quaternion_new_radians(q1_rotation_axis, q1_rotation_angle);
	r32 q2_rotation_angle = gm_vec3_length(rot1);
	vec3 q2_rotation_axis = gm_vec3_normalize(rot1);
	Quaternion q2 = quaternion_new_radians(q2_rotation_axis, q2_rotation_angle);
	Quaternion inv_q2 = quaternion_inverse(&q2);
	e1->world_rotation = quaternion_product(&e1->world_rotation, &q1);
	e2->world_rotation = quaternion_product(&e2->world_rotation, &inv_q2);
	#endif
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

	for (u32 i = 0; i < NUM_SUBSTEPS; ++i) {
		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			vec3 external_force = calculate_external_force(e);
			vec3 external_torque = calculate_external_torque(e);
			e->previous_world_position = e->world_position;
			e->linear_velocity = gm_vec3_add(e->linear_velocity, gm_vec3_scalar_product(h / e->mass, external_force));
			e->world_position = gm_vec3_add(e->world_position, gm_vec3_scalar_product(h, e->linear_velocity));

			e->previous_world_rotation = e->world_rotation;
			mat3 inverse_inertia_tensor = get_inverse_inertia_tensor(e);
			mat3 inertia_tensor = get_inertia_tensor(e);
			e->angular_velocity = gm_vec3_add(e->angular_velocity, gm_vec3_scalar_product(h, 
				gm_mat3_multiply_vec3(&inverse_inertia_tensor, gm_vec3_subtract(external_torque,
				gm_vec3_cross(e->angular_velocity, gm_mat3_multiply_vec3(&e->inertia_tensor, e->angular_velocity))))));

			// deviated from paper
			r32 rotation_angle = gm_vec3_length(e->angular_velocity) * h;
			vec3 rotation_axis = gm_vec3_normalize(e->angular_velocity);
			Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
			e->world_rotation = quaternion_product(&orientation_change, &e->world_rotation);
			//Quaternion qq = (Quaternion){e->angular_velocity.x, e->angular_velocity.y, e->angular_velocity.z, 0.0f};
			//Quaternion rr = quaternion_product(&qq, &e->world_rotation);
			//e->world_rotation.x += h * 0.5f * rr.x;
			//e->world_rotation.y += h * 0.5f * rr.y;
			//e->world_rotation.z += h * 0.5f * rr.z;
			//e->world_rotation = quaternion_normalize(&e->world_rotation);
		}

		Constraint* constraints = array_new(Constraint);
		// collisions
		Collision_Point* pts = collision_get_plane_cube_points(&entities[0], -2.0f);
		for (u32 j = 0; j < array_length(pts); ++j) {
			Collision_Point* cp = &pts[j];
			Constraint constraint = {0};
			constraint.type = POSITIONAL_CONSTRAINT;
			constraint.positional_constraint.compliance = 0.0f;
			constraint.positional_constraint.e1 = &entities[1];
			constraint.positional_constraint.e2 = &entities[0];
			constraint.positional_constraint.lambda = 0.0f;
			// r1 and r2 world coords?
			constraint.positional_constraint.r1_wc = (vec3){0.0f, 0.0f, 0.0f};
			vec3 r2 = cp->r_lc;
			constraint.positional_constraint.r2_wc = r2;
			mat3 current_rot_matrix = quaternion_get_matrix3(&entities[0].world_rotation);
			mat3 previous_rot_matrix = quaternion_get_matrix3(&entities[0].previous_world_rotation);
			vec3 p2 = gm_vec3_add(entities[0].world_position, gm_mat3_multiply_vec3(&current_rot_matrix, r2));
			vec3 p2_til = gm_vec3_add(entities[0].previous_world_position, gm_mat3_multiply_vec3(&previous_rot_matrix, r2));
			vec3 p1 = (vec3){p1.x, -2.0f, p1.z};
			vec3 p1_til = (vec3){p1.x, -2.0f, p1.z};
			r32 d = gm_vec3_dot(gm_vec3_subtract(p1, p2), cp->normal);
			constraint.positional_constraint.delta_x_wc = gm_vec3_scalar_product(d, cp->normal);
			if (d > 0.0f) {
				array_push(constraints, constraint);
			}
		}

		// solve positions
		for (u32 j = 0; j < NUM_POS_ITERS; ++j) {
			for (u32 k = 0; k < array_length(constraints); ++k) {
				Constraint* constraint = &constraints[k];
				solve_constraint(constraint, h);
			}
		}

		for (u32 j = 0; j < array_length(entities); ++j) {
			Entity* e = &entities[j];
			e->previous_linear_velocity = e->linear_velocity;
			e->previous_angular_velocity = e->angular_velocity;
			//if (j == 0) printf("previous: <%.3f, %.3f, %.3f>\n", e->previous_linear_velocity.x, e->previous_linear_velocity.y, e->previous_linear_velocity.z);
			e->linear_velocity = gm_vec3_scalar_product(1.0f / h, gm_vec3_subtract(e->world_position, e->previous_world_position));
			//if (j == 0) printf("new: <%.3f, %.3f, %.3f>\n", e->linear_velocity.x, e->linear_velocity.y, e->linear_velocity.z);
			Quaternion inv = quaternion_inverse(&e->previous_world_rotation);
			Quaternion delta_q = quaternion_product(&e->world_rotation, &inv);
			if (delta_q.w >= 0.0f) {
				e->angular_velocity = gm_vec3_scalar_product(2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			} else {
				e->angular_velocity = gm_vec3_scalar_product(-2.0f / h, (vec3){delta_q.x, delta_q.y, delta_q.z});
			}
			graphics_entity_recalculate_model_matrix(&entities[j]);
		}

		// solve velocities
		#if 0
		for (u32 j = 0; j < array_length(constraints); ++j) {
			Constraint* constraint = &constraints[j];
			vec3 v1 = entities[1].linear_velocity;
			vec3 w1 = entities[1].angular_velocity;
			vec3 r1 = constraint->positional_constraint.r1_wc;
			vec3 v2 = entities[0].linear_velocity;
			vec3 w2 = entities[0].angular_velocity;
			vec3 r2 = constraint->positional_constraint.r2_wc;
			vec3 n = (vec3){0.0f, 1.0f, 0.0f};
			r32 lambda = constraint->positional_constraint.lambda;
			Entity* e1 = constraint->positional_constraint.e1;
			Entity* e2 = constraint->positional_constraint.e2;

			vec3 v = gm_vec3_subtract(gm_vec3_add(v1, gm_vec3_cross(w1, r1)), gm_vec3_subtract(v2, gm_vec3_cross(w2, r2)));
			r32 vn = gm_vec3_dot(n, v);
			vec3 vt = gm_vec3_subtract(v, gm_vec3_scalar_product(vn, n));
			
			/*
			{
				const r32 dynamic_friction_coefficient = 0.5f;
				r32 fn = lambda / (h * h);
				r32 fact = MIN(h * dynamic_friction_coefficient * fn, gm_vec3_length(vt));
				vec3 delta_v = gm_vec3_scalar_product(-fact, gm_vec3_normalize(vt));
				mat3 e1_inverse_inertia_tensor = get_inverse_inertia_tensor(e1);
				mat3 e2_inverse_inertia_tensor = get_inverse_inertia_tensor(e2);

				r32 _w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
				r32 _w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));
				vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(1.0f / e1->mass, p));
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(1.0f / e2->mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, p)));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, p)));
			}
			*/

			{
				vec3 old_v1 = entities[1].previous_linear_velocity;
				vec3 old_w1 = entities[1].previous_angular_velocity;
				vec3 old_v2 = entities[0].previous_linear_velocity;
				vec3 old_w2 = entities[0].previous_angular_velocity;
				vec3 v_til = gm_vec3_subtract(gm_vec3_add(old_v1, gm_vec3_cross(old_w1, r1)), gm_vec3_subtract(old_v2, gm_vec3_cross(old_w2, r2)));
				r32 vn_til = gm_vec3_dot(n, v_til);
				r32 e = 0.0f;
				r32 fact = -vn + MAX(-e * vn_til, 0.0f);
				vec3 delta_v = gm_vec3_scalar_product(fact, n);
				mat3 e1_inverse_inertia_tensor = get_inverse_inertia_tensor(e1);
				mat3 e2_inverse_inertia_tensor = get_inverse_inertia_tensor(e2);

				r32 _w1 = 1.0f / e1->mass + gm_vec3_dot(gm_vec3_cross(r1, n), gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, n)));
				r32 _w2 = 1.0f / e2->mass + gm_vec3_dot(gm_vec3_cross(r2, n), gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, n)));
				vec3 p = gm_vec3_scalar_product(1.0f / (_w1 + _w2), delta_v);
				e1->linear_velocity = gm_vec3_add(e1->linear_velocity, gm_vec3_scalar_product(1.0f / e1->mass, p));
				e2->linear_velocity = gm_vec3_subtract(e2->linear_velocity, gm_vec3_scalar_product(1.0f / e2->mass, p));
				e1->angular_velocity = gm_vec3_add(e1->angular_velocity, gm_mat3_multiply_vec3(&e1_inverse_inertia_tensor, gm_vec3_cross(r1, p)));
				e2->angular_velocity = gm_vec3_subtract(e2->angular_velocity, gm_mat3_multiply_vec3(&e2_inverse_inertia_tensor, gm_vec3_cross(r2, p)));
			}
		}
		#endif

		array_free(pts);
		array_free(constraints);
	}
}