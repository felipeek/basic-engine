#include "physics.h"
#include "collision.h"
#include <dynamic_array.h>
#define TIME_TOLERANCE (0.0000001f)
#define PENETRATION_TOLERANCE (0.01f)

static void apply_impulse(Entity* cube, Entity* plane, Collision_Point* cp) {
	cp->normal = gm_vec3_normalize(cp->normal);

	// @TODO: copied from physics
	mat4 cube_rotation_matrix = quaternion_get_matrix(&cube->world_rotation);
	mat3 cube_object_inertia_tensor = cube->inertia_tensor;
	mat3 cube_rotation_matrix_m3 = gm_mat4_to_mat3(&cube_rotation_matrix);
	mat3 cube_transposed_rotation_matrix = gm_mat3_transpose(&cube_rotation_matrix_m3);
	mat3 cube_aux = gm_mat3_multiply(&cube_rotation_matrix_m3, &cube_object_inertia_tensor);
	mat3 cube_dynamic_inertia_tensor = gm_mat3_multiply(&cube_aux, &cube_transposed_rotation_matrix);
	mat3 cube_dynamic_inertia_tensor_inverse;
	assert(gm_mat3_inverse(&cube_dynamic_inertia_tensor, &cube_dynamic_inertia_tensor_inverse));
	vec3 cube_angular_velocity = gm_mat3_multiply_vec3(&cube_dynamic_inertia_tensor_inverse, cube->angular_momentum);
	vec3 cube_linear_velocity = gm_vec3_scalar_product(1.0f / cube->mass, cube->linear_momentum);

	mat4 plane_rotation_matrix = quaternion_get_matrix(&plane->world_rotation);
	mat3 plane_object_inertia_tensor = plane->inertia_tensor;
	mat3 plane_rotation_matrix_m3 = gm_mat4_to_mat3(&plane_rotation_matrix);
	mat3 plane_transposed_rotation_matrix = gm_mat3_transpose(&plane_rotation_matrix_m3);
	mat3 plane_aux = gm_mat3_multiply(&plane_rotation_matrix_m3, &plane_object_inertia_tensor);
	mat3 plane_dynamic_inertia_tensor = gm_mat3_multiply(&plane_aux, &plane_transposed_rotation_matrix);
	mat3 plane_dynamic_inertia_tensor_inverse;
	assert(gm_mat3_inverse(&plane_dynamic_inertia_tensor, &plane_dynamic_inertia_tensor_inverse));
	vec3 plane_angular_velocity = gm_mat3_multiply_vec3(&plane_dynamic_inertia_tensor_inverse, plane->angular_momentum);
	vec3 plane_linear_velocity = gm_vec3_scalar_product(1.0f / plane->mass, plane->linear_momentum);

	/* inputs */
	vec3 vA = cube_linear_velocity;
	vec3 vB = plane_linear_velocity;
	vec3 wA = cube_angular_velocity;
	vec3 wB = plane_angular_velocity;
	vec3 rA = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(cube->world_position));
	vec3 rB = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(plane->world_position));
	const r32 epsilon = 0.0f;
	vec3 N = gm_vec3_normalize(cp->normal);
	r32 mA = cube->mass;
	r32 mB = plane->mass;

    /* impulse algorithm */
    vec3 vP1 = gm_vec3_add(vA, gm_vec3_cross(wA, rA));
    vec3 vP2 = gm_vec3_add(vB, gm_vec3_cross(wB, rB));
    vec3 vR = gm_vec3_subtract(vP2, vP1);
    r32 numerator = gm_vec3_dot(gm_vec3_scalar_product(-(1.0f + epsilon), vR), N);
    vec3 rA_cross_N = gm_vec3_cross(rA, N);
    vec3 rB_cross_N = gm_vec3_cross(rB, N);
    vec3 IA_res = gm_mat3_multiply_vec3(&cube_dynamic_inertia_tensor_inverse, rA_cross_N);
    vec3 IB_res = gm_mat3_multiply_vec3(&plane_dynamic_inertia_tensor_inverse, rB_cross_N);
    r32 denominator_last_factor = gm_vec3_dot(gm_vec3_add(gm_vec3_cross(IA_res, rA), gm_vec3_cross(IB_res, rB)), N);
    r32 denominator = 1.0f / mA + 1.0f / mB + denominator_last_factor;
    r32 j = numerator / denominator;

#if 0
    // check
    vec3 vA_plus = gm_vec3_subtract(vA, gm_vec3_scalar_product(j / mA, N));
    vec3 vB_plus = gm_vec3_add(vB, gm_vec3_scalar_product(j / mB, N));
    vec3 wA_plus = gm_vec3_subtract(wA, gm_vec3_scalar_product(j, gm_mat3_multiply_vec3(&cube_dynamic_inertia_tensor_inverse, gm_vec3_cross(rA, N))));
    vec3 wB_plus = gm_vec3_add(wB, gm_vec3_scalar_product(j, gm_mat3_multiply_vec3(&plane_dynamic_inertia_tensor_inverse, gm_vec3_cross(rB, N))));
    vec3 vP1_plus = gm_vec3_add(vA_plus, gm_vec3_cross(wA_plus, rA));
    vec3 vP2_plus = gm_vec3_add(vB_plus, gm_vec3_cross(wB_plus, rB));
    vec3 vR_plus = gm_vec3_subtract(vP2_plus, vP1_plus);
    r32 vR_plus_dot_N = gm_vec3_dot(vR_plus, N);
    vec3 wA_plus_cross_rA = gm_vec3_cross(wA_plus, rA);
    vec3 wB_plus_cross_rB = gm_vec3_cross(wB_plus, rB);
    r32 vRel_plus = gm_vec3_dot(N, gm_vec3_subtract(gm_vec3_add(vA_plus, wA_plus_cross_rA), gm_vec3_add(vB_plus, wB_plus_cross_rB)));
#endif

	vec3 J = gm_vec3_scalar_product(j, cp->normal);
	
	cube->linear_momentum = gm_vec3_add(cube->linear_momentum, gm_vec3_scalar_product(-1.0f, J));
	plane->linear_momentum = gm_vec3_add(plane->linear_momentum, J);
	cube->angular_momentum = gm_vec3_add(cube->angular_momentum, gm_vec3_scalar_product(-1.0f, gm_vec3_cross(rA, J)));
	plane->angular_momentum = gm_vec3_add(plane->angular_momentum, gm_vec3_cross(rB, J));
}

static void physics_update(Entity* cube, r32 dt, Physics_Force* forces) {
	// Calculate total force and torque
	const vec3 center_of_mass = (vec3){0.0f, 0.0f, 0.0f};
	vec3 total_force = (vec3){0.0f, 0.0f, 0.0f};
	vec3 total_torque = (vec3){0.0f, 0.0f, 0.0f};
	for (u32 i = 0; i < array_get_length(forces); ++i) {
		vec3 distance = gm_vec3_subtract(forces[i].position, center_of_mass);
		total_force = gm_vec3_add(total_force, forces[i].force);
		total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, forces[i].force));
	}
	vec3 angular_momentum_change = gm_vec3_scalar_product(dt, total_torque);
	vec3 linear_momentum_change = gm_vec3_scalar_product(dt, total_force);
	cube->linear_momentum = gm_vec3_add(cube->linear_momentum, linear_momentum_change);
	cube->angular_momentum = gm_vec3_add(cube->angular_momentum, angular_momentum_change);

	mat4 rotation_matrix = quaternion_get_matrix(&cube->world_rotation);
	mat3 object_inertia_tensor = cube->inertia_tensor;
	mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
	mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
	mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &object_inertia_tensor);
	mat3 dynamic_inertia_tensor = gm_mat3_multiply(&aux, &transposed_rotation_matrix);
	mat3 dynamic_inertia_tensor_inverse;
	assert(gm_mat3_inverse(&dynamic_inertia_tensor, &dynamic_inertia_tensor_inverse));

	vec3 angular_velocity = gm_mat3_multiply_vec3(&dynamic_inertia_tensor_inverse, cube->angular_momentum);
	vec3 linear_velocity = gm_vec3_scalar_product(1.0f / cube->mass, cube->linear_momentum);

	#if 1
	Quaternion av = (Quaternion){angular_velocity.x, angular_velocity.y, angular_velocity.z, 0.0f};
	Quaternion delta = quaternion_product(&cube->world_rotation, &av);
	delta.x *= 0.5f * dt; delta.y *= 0.5f * dt; delta.z *= 0.5f * dt; delta.w *= 0.5f * dt;
	Quaternion new_orientation = cube->world_rotation;
	new_orientation.x += delta.x;
	new_orientation.y += delta.y;
	new_orientation.z += delta.z;
	new_orientation.w += delta.w;
	new_orientation = quaternion_normalize(&new_orientation);
	graphics_entity_set_rotation(cube, new_orientation);
	#endif

	// Calculate orientation change
	//r32 angular_velocity_length = dt * gm_vec3_length(angular_velocity) * 360.0f;
	//Quaternion orientation_change = quaternion_new(gm_vec3_normalize(angular_velocity), angular_velocity_length);
	vec3 position_change = gm_vec3_scalar_product(dt, linear_velocity);

	graphics_entity_set_position(cube, gm_vec4_add(cube->world_position, (vec4){position_change.x, position_change.y, position_change.z, 0.0f}));
	//graphics_entity_set_rotation(e, quaternion_product(&e->world_rotation, &orientation_change));
}

static r32 force_cube_y(const Entity* cube, r32 plane_y)
{
	r32 y = 0.0f;
	for (u32 i = 0; i < array_get_length(cube->mesh.vertices); ++i) {
		Vertex* v = &cube->mesh.vertices[i];
		vec4 wc_pos = gm_mat4_multiply_vec4(&cube->model_matrix, v->position);
		r32 y_diff = -wc_pos.y + plane_y;
		if (y_diff > 0.0f && y_diff > y) {
			y = y_diff;
		}
	}

	return y > 0.0f ? y + 0.00001f : 0.0f;
}

void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
	r32 dt_frac = dt / 10000;
	for (u32 i = 0; i < 10000; ++i) {
		r32 y_diff = force_cube_y(cube, plane_y);
		vec4 new_pos = cube->world_position;
		new_pos.y += y_diff;
		graphics_entity_set_position(cube, new_pos);

		vec4 old_position = cube->world_position;
		Quaternion old_rotation = cube->world_rotation;

		physics_update(cube, dt_frac, forces);
		Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
		if (array_get_length(collision_points) > 0) {
			//for (u32 i = 0; i < array_get_length(collision_points); ++i) {
				Collision_Point* cp = &collision_points[0];
				apply_impulse(cube, plane, cp);
			//}
		}
	}
}

#if 0
void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
	boolean try_again = true;
	r32 dt_until_collision = dt;

	while (try_again && dt_until_collision > TIME_TOLERANCE) {
		try_again = false;
		vec4 old_position = cube->world_position;
		Quaternion old_rotation = cube->world_rotation;

		physics_update(cube, dt_until_collision, forces);

		Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);

#if 1
		if (array_get_length(collision_points) > 0) {
			Collision_Point* cp = &collision_points[0];

			// deduce first collision point
			for (u32 i = 1; i < array_get_length(collision_points); ++i) {
				Collision_Point* collision_point = &collision_points[i];
				if (collision_point->collision_point.y < cp->collision_point.y) {
					cp = collision_point;
				}
			}

			if (cp->penetration > PENETRATION_TOLERANCE) {
				dt_until_collision = dt_until_collision / 2.0f;
				if (dt_until_collision <= TIME_TOLERANCE) {
					apply_impulse(cube, plane, cp);
					physics_update(cube, dt - dt_until_collision, forces);
					try_again = false;
				} else {
					graphics_entity_set_position(cube, old_position);
					graphics_entity_set_rotation(cube, old_rotation);
					try_again = true;
				}
			} else {
				apply_impulse(cube, plane, cp);
				physics_update(cube, dt - dt_until_collision, forces);
			}
		}
#else
		if (array_get_length(collision_points) > 0) {
			// @temporary
			Collision_Point* cp = &collision_points[0];

			if (cp->penetration > PENETRATION_TOLERANCE) {
				dt_until_collision = dt_until_collision / 2.0f;
				if (dt_until_collision <= TIME_TOLERANCE) {
					apply_impulse(cube, plane, cp);
					physics_update(cube, dt - dt_until_collision, forces);
					try_again = false;
				} else {
					graphics_entity_set_position(cube, old_position);
					graphics_entity_set_rotation(cube, old_rotation);
					try_again = true;
				}
			} else {
				apply_impulse(cube, plane, cp);
				physics_update(cube, dt - dt_until_collision, forces);
			}
		}
#endif
	}
}
#endif