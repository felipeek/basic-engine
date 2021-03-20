#include "physics.h"
#include "collision.h"
#include <dynamic_array.h>

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
    for (u32 i = 0; forces && i < array_get_length(forces); ++i) {
        vec3 distance = gm_vec3_subtract(forces[i].position, center_of_mass);
        total_force = gm_vec3_add(total_force, forces[i].force);
        total_torque = gm_vec3_add(total_torque, gm_vec3_cross(distance, forces[i].force));
    }
    vec3 angular_momentum_change = gm_vec3_scalar_product(dt, total_torque);
    vec3 linear_momentum_change = gm_vec3_scalar_product(dt, total_force);
    e->linear_momentum = gm_vec3_add(e->linear_momentum, linear_momentum_change);
    e->angular_momentum = gm_vec3_add(e->angular_momentum, angular_momentum_change);
}

static void apply_impulse(Entity* cube, Entity* plane, Collision_Point* cp, Physics_Force* forces, r32 restitution) {
    cp->normal = gm_vec3_normalize(cp->normal);

	mat3 cube_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(cube);
	vec3 cube_angular_velocity = physics_get_angular_velocity(cube, &cube_dynamic_inertia_tensor_inverse);
	vec3 cube_linear_velocity = physics_get_linear_velocity(cube);

	mat3 plane_dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(plane);
	vec3 plane_angular_velocity = physics_get_angular_velocity(plane, &plane_dynamic_inertia_tensor_inverse);
	vec3 plane_linear_velocity = physics_get_linear_velocity(plane);

    /* inputs */
    vec3 vA = cube_linear_velocity;
    vec3 vB = plane_linear_velocity;
    vec3 wA = cube_angular_velocity;
    vec3 wB = plane_angular_velocity;
    vec3 rA = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(cube->world_position));
    vec3 rB = gm_vec3_subtract(cp->collision_point, gm_vec4_to_vec3(plane->world_position));
    vec3 N = gm_vec3_normalize(cp->normal);
    r32 mA = cube->mass;
    r32 mB = plane->mass;

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
    vec3 IA_res = gm_mat3_multiply_vec3(&cube_dynamic_inertia_tensor_inverse, rA_cross_N);
    vec3 IB_res = gm_mat3_multiply_vec3(&plane_dynamic_inertia_tensor_inverse, rB_cross_N);
    r32 denominator_last_factor = gm_vec3_dot(gm_vec3_add(gm_vec3_cross(IA_res, rA), gm_vec3_cross(IB_res, rB)), N);
    r32 denominator = 1.0f / mA + 1.0f / mB + denominator_last_factor;
    r32 j_r = numerator / denominator;

#if 1
    // check
    vec3 vA_plus = gm_vec3_subtract(vA, gm_vec3_scalar_product(j_r / mA, N));
    vec3 vB_plus = gm_vec3_add(vB, gm_vec3_scalar_product(j_r / mB, N));
    vec3 wA_plus = gm_vec3_subtract(wA, gm_vec3_scalar_product(j_r, gm_mat3_multiply_vec3(&cube_dynamic_inertia_tensor_inverse, gm_vec3_cross(rA, N))));
    vec3 wB_plus = gm_vec3_add(wB, gm_vec3_scalar_product(j_r, gm_mat3_multiply_vec3(&plane_dynamic_inertia_tensor_inverse, gm_vec3_cross(rB, N))));
    vec3 vP1_plus = gm_vec3_add(vA_plus, gm_vec3_cross(wA_plus, rA));
    vec3 vP2_plus = gm_vec3_add(vB_plus, gm_vec3_cross(wB_plus, rB));
    vec3 vR_plus = gm_vec3_subtract(vP2_plus, vP1_plus);
    r32 vR_plus_dot_N = gm_vec3_dot(vR_plus, N);
    vec3 wA_plus_cross_rA = gm_vec3_cross(wA_plus, rA);
    vec3 wB_plus_cross_rB = gm_vec3_cross(wB_plus, rB);
    r32 vRel_plus = gm_vec3_dot(N, gm_vec3_subtract(gm_vec3_add(vA_plus, wA_plus_cross_rA), gm_vec3_add(vB_plus, wB_plus_cross_rB)));
#endif

    vec3 J_r = gm_vec3_scalar_product(j_r, cp->normal);
    
    cube->linear_momentum = gm_vec3_add(cube->linear_momentum, gm_vec3_scalar_product(-1.0f, J_r));
    plane->linear_momentum = gm_vec3_add(plane->linear_momentum, J_r);
    cube->angular_momentum = gm_vec3_add(cube->angular_momentum, gm_vec3_scalar_product(-1.0f, gm_vec3_cross(rA, J_r)));
    plane->angular_momentum = gm_vec3_add(plane->angular_momentum, gm_vec3_cross(rB, J_r));

#if 0
    // Tangential component
    vec3 fe = {0};
    for (u32 i = 0; i < array_get_length(forces); ++i) {
        fe = gm_vec3_add(fe, forces[i].force);
    }
    r32 vR_dot_N = gm_vec3_dot(vR, cp->normal);
    r32 fe_dot_N = gm_vec3_dot(fe, cp->normal);
    vec3 t;
    if (vR_dot_N != 0.0f) {
        t = gm_vec3_normalize(gm_vec3_subtract(vR, gm_vec3_scalar_product(vR_dot_N, cp->normal)));
    } else if (fe_dot_N != 0.0f) {
        t = gm_vec3_normalize(gm_vec3_subtract(fe, gm_vec3_scalar_product(fe_dot_N, cp->normal)));
    } else {
        t = (vec3){0.0f, 0.0f, 0.0f};
    }

    const r32 static_coefficient = 0.5f;
    const r32 dynamic_coefficient = 0.25f;
    r32 j_s = static_coefficient * j_r;
    r32 j_d = dynamic_coefficient * j_r;

    r32 m_vR_dot_t = gm_vec3_dot(gm_vec3_scalar_product(cube->mass, vR), t);
    r32 j_f;
    const r32 STATIC_THRESHOLD = 0.00001f;
    if (gm_vec3_length(vR) < STATIC_THRESHOLD) {
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
    cube->linear_momentum = gm_vec3_add(cube->linear_momentum, gm_vec3_scalar_product(-1.0f, J_f));
    plane->linear_momentum = gm_vec3_add(plane->linear_momentum, J_f);
    cube->angular_momentum = gm_vec3_add(cube->angular_momentum, gm_vec3_scalar_product(-1.0f, gm_vec3_cross(rA, J_f)));
    plane->angular_momentum = gm_vec3_add(plane->angular_momentum, gm_vec3_cross(rB, J_f));
#endif
}

static void physics_update(Entity* cube, r32 dt) {
	mat3 dynamic_inertia_tensor_inverse = physics_get_dynamic_inertia_tensor_inverse(cube);
	vec3 angular_velocity = physics_get_angular_velocity(cube, &dynamic_inertia_tensor_inverse);
	vec3 linear_velocity = physics_get_linear_velocity(cube);

    // Calculate orientation change
    r32 rotation_angle = gm_vec3_length(angular_velocity) * dt;
    vec3 rotation_axis = gm_vec3_normalize(angular_velocity);
    Quaternion orientation_change = quaternion_new_radians(rotation_axis, rotation_angle);
    graphics_entity_set_rotation(cube, quaternion_product(&orientation_change, &cube->world_rotation));

    // Calculate position change
    vec3 position_change = gm_vec3_scalar_product(dt, linear_velocity);
    graphics_entity_set_position(cube, gm_vec4_add(cube->world_position, (vec4){position_change.x, position_change.y, position_change.z, 0.0f}));
}

#if 1
// nonconvex rigid bodies with stacking by Guendelman et al.
void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
	vec4 old_position = cube->world_position;
	Quaternion old_rotation = cube->world_rotation;

	boolean has_collision = true;
	for (u32 i = 0; i < 5 && has_collision; ++i) {
		has_collision = false;
        physics_update(cube, dt);

        Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
        if (array_get_length(collision_points) > 0) {
			for (u32 j = 0; j < array_get_length(collision_points); ++j) {
				vec4 new_pos = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[collision_points[j].vertex_index].position);
				collision_points[j].collision_point = gm_vec4_to_vec3(new_pos);
				apply_impulse(cube, plane, &collision_points[j], forces, 0.8f);
			}

			has_collision = true;
		}

		graphics_entity_set_position(cube, old_position);
		graphics_entity_set_rotation(cube, old_rotation);
	}

    physics_update_momenta_based_on_forces(cube, dt, forces);

	has_collision = true;
	for (u32 i = 0; i < 10 && has_collision; ++i) {
		has_collision = false;
		r32 restitution = -0.9f;
        physics_update(cube, dt);

        Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
        if (array_get_length(collision_points) > 0) {
			for (u32 j = 0; j < array_get_length(collision_points); ++j) {
				vec4 new_pos = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[collision_points[j].vertex_index].position);
				collision_points[j].collision_point = gm_vec4_to_vec3(new_pos);
				apply_impulse(cube, plane, &collision_points[j], forces, 0.0f);
			}

			has_collision = true;
			restitution += 0.1f;
		}

		graphics_entity_set_position(cube, old_position);
		graphics_entity_set_rotation(cube, old_rotation);
	}
	
	physics_update(cube, dt);
}
#endif

#if 0
void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
    r32 dt_f = dt;

	vec4 old_position = cube->world_position;
	Quaternion old_rotation = cube->world_rotation;

    physics_update_momenta_based_on_forces(cube, dt, forces);
	boolean has_collision = true;
	for (u32 i = 0; i < 10 && has_collision; ++i) {
		has_collision = false;
        physics_update(cube, dt_f);

        Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
        if (array_get_length(collision_points) > 0) {
			r32 max_penetration = 0.0f;
			for (u32 i = 0; i < array_get_length(collision_points); ++i) {
				if (collision_points[i].penetration > max_penetration) {
					max_penetration = collision_points[i].penetration;
				}
			}

			if (max_penetration > 0.001f && i < 9) {
				dt_f = dt_f / 2.0f;
				graphics_entity_set_position(cube, old_position);
				graphics_entity_set_rotation(cube, old_rotation);
				has_collision = true;
			} else {
				for (u32 j = 0; j < array_get_length(collision_points); ++j) {
					vec4 new_pos = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[collision_points[j].vertex_index].position);
					collision_points[j].collision_point = gm_vec4_to_vec3(new_pos);
					apply_impulse(cube, plane, &collision_points[j], forces, 0.8f);
				}

				physics_update(cube, dt - dt_f);
			}
        }
	}
}
#endif

#if 0
void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
    physics_update_momenta_based_on_forces(cube, dt, forces);

    r32 dt_f = dt / 10.0f;
    for (u32 i = 0; i < 10; ++i) {
        vec4 old_position = cube->world_position;
        Quaternion old_rotation = cube->world_rotation;
        physics_update(cube, dt_f);

        Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
        if (array_get_length(collision_points) > 0) {
            graphics_entity_set_position(cube, old_position);
            graphics_entity_set_rotation(cube, old_rotation);
            for (u32 j = 0; j < array_get_length(collision_points); ++j) {
                vec4 new_pos = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[collision_points[j].vertex_index].position);
                collision_points[j].collision_point = gm_vec4_to_vec3(new_pos);
                apply_impulse(cube, plane, &collision_points[j], forces, 0.8f);
            }

            physics_update(cube, dt_f);
        }
    }
}
#endif

#if 0
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

    return y > 0.0f ? y + 0.01f : 0.0f;
}
#endif