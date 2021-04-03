#include "physics.h"
#include <light_array.h>

#define MAX_MASS_TO_CONSIDER_STATIC_BODY 10000.0f

static mat3 physics_get_inverse_inertia_tensor_dynamic(Entity* e) {
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

static void physics_update_momenta_based_on_forces(Entity* e, r32 dt) {
    // Calculate total force and torque
    const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
    vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
    for (u32 i = 0; e->forces && i < array_length(e->forces); ++i) {
        vec3 distance = gm_vec3_subtract(e->forces[i].position, center_of_mass);
        total_force = gm_vec3_add(total_force, e->forces[i].force);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, e->forces[i].force));
    }
    vec3 angular_momentum_change = gm_vec3_scalar_product(dt, total_torque);
    vec3 linear_momentum_change = gm_vec3_scalar_product(dt, total_force);
    e->linear_momentum = gm_vec3_add(e->linear_momentum, linear_momentum_change);
    e->angular_momentum = gm_vec3_add(e->angular_momentum, angular_momentum_change);
}

static void physics_update(Entity* e, r32 dt) {
	if (e->mass > MAX_MASS_TO_CONSIDER_STATIC_BODY) {
		return;
	}
	mat3 inverse_inertia_tensor_dynamic = physics_get_inverse_inertia_tensor_dynamic(e);
	vec3 angular_velocity = physics_get_angular_velocity(e, &inverse_inertia_tensor_dynamic);
	vec3 linear_velocity = physics_get_linear_velocity(e);

    // Calculate orientation change
    r32 rotation_angle = gm_vec3_length(angular_velocity) * dt;
    vec3 rotation_axis = gm_vec3_normalize(angular_velocity);
    Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
    graphics_entity_set_rotation(e, quaternion_product(&orientation_change, &e->world_rotation));

    // Calculate position change
    vec3 position_change = gm_vec3_scalar_product(dt, linear_velocity);
    graphics_entity_set_position(e, gm_vec3_add(e->world_position, (vec3){position_change.x, position_change.y, position_change.z}));
}

void physics_simulate(Entity* entities, r32 dt) {
	for (u32 j = 0; j < array_length(entities); ++j) {
		Entity* e = &entities[j];
        physics_update_momenta_based_on_forces(e, dt);
		physics_update(e, dt);
	}
}