#include "physics.h"
#include <dynamic_array.h>

void physics_update(Entity* e, Physics_Force* forces, r32 dt) {
	if (array_get_length(forces) != 0) {
		printf("lol");
	}
	static r32 object_mass = 100.0f;

	mat4 rotation_matrix = quaternion_get_matrix(&e->world_rotation);
	mat4 transposed_rotation_matrix = gm_mat4_transpose(&rotation_matrix);

	// todo
	mat4 object_inertia_tensor = gm_mat4_identity();
	mat4 object_inertia_tensor_inverse = gm_mat4_identity();

	mat4 aux = gm_mat4_multiply(&object_inertia_tensor, &transposed_rotation_matrix);
	mat4 dynamic_inertia_tensor = gm_mat4_multiply(&rotation_matrix, &aux);

	vec4 angular_velocity = gm_mat4_multiply_vec4(&dynamic_inertia_tensor, e->angular_momentum);
	vec4 linear_velocity = gm_vec4_scalar_product(1.0f / object_mass, e->linear_momentum);
	assert(linear_velocity.w == 0.0f);
	assert(angular_velocity.w == 0.0f);

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
	r32 angular_velocity_length = dt * gm_vec4_length(angular_velocity);
	Quaternion orientation_change = quaternion_new(gm_vec3_normalize(gm_vec4_to_vec3(angular_velocity)), angular_velocity_length);
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
}