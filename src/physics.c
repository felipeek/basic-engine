#include "physics.h"
#include <dynamic_array.h>

void physics_update(Entity* e, Physics_Force* forces, r32 dt) {
	if (e->static_body) {
		e->linear_momentum = (vec4){0.0f, 0.0f, 0.0f, 0.0f};
		e->angular_momentum = (vec4){0.0f, 0.0f, 0.0f, 0.0f};
		return;
	}

	if (array_get_length(forces) != 0) {
		printf("there are forces\n");
	}

	mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
	mat3 object_inertia_tensor = e->inertia_tensor;
	mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
	mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &object_inertia_tensor);
	mat3 dynamic_inertia_tensor = gm_mat3_multiply(&aux, &transposed_rotation_matrix);
	mat3 dynamic_inertia_tensor_inverse;
	assert(gm_mat3_inverse(&dynamic_inertia_tensor, &dynamic_inertia_tensor_inverse));

	vec3 angular_velocity = gm_mat3_multiply_vec3(&dynamic_inertia_tensor_inverse, gm_vec4_to_vec3(e->angular_momentum));
	vec4 linear_velocity = gm_vec4_scalar_product(1.0f / e->mass, e->linear_momentum);
	assert(linear_velocity.w == 0.0f);

	// Calculate total force and torque
	const vec4 center_of_mass = (vec4){0.0f, 0.0f, 0.0f, 1.0f};
	vec4 total_force = (vec4){0.0f, 0.0f, 0.0f, 0.0f};
	vec4 total_torque = (vec4){0.0f, 0.0f, 0.0f, 0.0f};
	for (u32 i = 0; i < array_get_length(forces); ++i) {
		vec4 distance = gm_vec4_subtract(forces[i].position, center_of_mass);
		total_force = gm_vec4_add(total_force, forces[i].force);
		total_torque = gm_vec4_add(total_torque, gm_vec4_cross(distance, forces[i].force));
	}
	assert(total_force.w == 0.0f);
	assert(total_torque.w == 0.0f);

	// Calculate orientation change
	r32 angular_velocity_length = dt * gm_vec3_length(angular_velocity) * 71.1f;
	Quaternion orientation_change = quaternion_new(gm_vec3_normalize(angular_velocity), angular_velocity_length);
	vec4 position_change = gm_vec4_scalar_product(dt, linear_velocity);
	vec4 angular_momentum_change = gm_vec4_scalar_product(dt, total_torque);
	vec4 linear_momentum_change = gm_vec4_scalar_product(dt, total_force);
	assert(angular_momentum_change.w == 0.0f);
	assert(linear_momentum_change.w == 0.0f);
	assert(position_change.w == 0.0f);

	graphics_entity_set_position(e, gm_vec4_add(e->world_position, position_change));
	graphics_entity_set_rotation(e, quaternion_product(&e->world_rotation, &orientation_change));
	e->linear_momentum = gm_vec4_add(e->linear_momentum, linear_momentum_change);
	e->angular_momentum = gm_vec4_add(e->angular_momentum, angular_momentum_change);

	vec3 rA = gm_vec4_to_vec3(gm_vec4_subtract(gm_mat4_multiply_vec4(&e->model_matrix, e->mesh.vertices[4].position), e->world_position));
    vec3 velocity_at_vertex_4 = gm_vec3_add(gm_vec4_to_vec3(linear_velocity), gm_vec3_cross(angular_velocity, rA));
	printf("velocity at vertex4: %f\n", velocity_at_vertex_4.y);
}