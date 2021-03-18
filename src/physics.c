#include "physics.h"
#include "collision.h"
#include <dynamic_array.h>
#define TIME_TOLERANCE (0.0000001f)
#define PENETRATION_TOLERANCE (0.01f)

mat3 rotation_matrix_from_axis_and_angle(vec3 axis, r32 angle) {
    mat3 m;
    m.data[0][0] = cosf(angle) + axis.x * axis.x * (1.0f - cosf(angle));
    m.data[0][1] = axis.x * axis.y * (1.0f - cosf(angle)) - axis.z * sinf(angle);
    m.data[0][2] = axis.x * axis.z * (1.0f - cosf(angle)) + axis.y * sinf(angle);
    m.data[1][0] = axis.y * axis.x * (1.0f - cosf(angle)) + axis.z * sinf(angle);
    m.data[1][1] = cosf(angle) + axis.y * axis.y * (1.0f - cosf(angle));
    m.data[1][2] = axis.y * axis.z * (1.0f - cosf(angle)) - axis.x * sinf(angle);
    m.data[2][0] = axis.z * axis.x * (1.0f - cosf(angle)) - axis.y * sinf(angle);
    m.data[2][1] = axis.z * axis.y * (1.0f - cosf(angle)) + axis.x * sinf(angle);
    m.data[2][2] = cosf(angle) + axis.z * axis.z * (1.0f - cosf(angle));
    return m;
}

static void apply_impulse(Entity* cube, Entity* plane, Collision_Point* cp, Physics_Force* forces, r32 restitution) {
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
    vec3 N = gm_vec3_normalize(cp->normal);
    r32 mA = cube->mass;
    r32 mB = plane->mass;

    /* impulse algorithm */
    vec3 vP1 = gm_vec3_add(vA, gm_vec3_cross(wA, rA));
    vec3 vP2 = gm_vec3_add(vB, gm_vec3_cross(wB, rB));
    r32 fact = gm_vec3_dot(cp->normal, gm_vec3_subtract(vP2, vP1));
    if (fact < 0) {
        // cube already moving away from plane
        //return;
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

static void physics_update_momenta_based_on_forces(Entity* cube, r32 dt, Physics_Force* forces) {
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
    cube->linear_momentum = gm_vec3_add(cube->linear_momentum, linear_momentum_change);
    cube->angular_momentum = gm_vec3_add(cube->angular_momentum, angular_momentum_change);
}

static void physics_update(Entity* cube, r32 dt) {

    vec4 old_vx_wc = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[2].position);

    mat4 rotation_matrix = quaternion_get_matrix(&cube->world_rotation);
    mat3 object_inertia_tensor = cube->inertia_tensor;
    mat3 object_inertia_tensor_inverse;
    assert(gm_mat3_inverse(&object_inertia_tensor, &object_inertia_tensor_inverse));
    mat3 rotation_matrix_m3 = gm_mat4_to_mat3(&rotation_matrix);
    mat3 transposed_rotation_matrix = gm_mat3_transpose(&rotation_matrix_m3);
    mat3 aux = gm_mat3_multiply(&rotation_matrix_m3, &object_inertia_tensor_inverse);
    mat3 dynamic_inertia_tensor = gm_mat3_multiply(&aux, &transposed_rotation_matrix);

    vec3 angular_velocity = gm_mat3_multiply_vec3(&dynamic_inertia_tensor, cube->angular_momentum);
    vec3 linear_velocity = gm_vec3_scalar_product(1.0f / cube->mass, cube->linear_momentum);
    //printf("Angular velocity: <%f, %f, %f>\n", angular_velocity.x, angular_velocity.y, angular_velocity.z);
    //printf("Linear velocity: <%f, %f, %f>\n", linear_velocity.x, linear_velocity.y, linear_velocity.z);

    vec3 rA = gm_vec4_to_vec3(gm_vec4_subtract(old_vx_wc, cube->world_position));
    vec3 vP1 = gm_vec3_add(linear_velocity, gm_vec3_cross(angular_velocity, rA));
    //printf("Vertex 2 velocity: <%f, %f, %f>\n", vP1_plus.x, vP1_plus.y, vP1_plus.z);
    printf("Point velocity before update: <%f, %f, %f>\n", vP1.x, vP1.y, vP1.z);

    // Calculate orientation change
    r32 _angle = gm_vec3_length(angular_velocity) * dt;
    vec3 _axis = gm_vec3_normalize(angular_velocity);
    Quaternion orientation_change = quaternion_new_radians(_axis, _angle);
    graphics_entity_set_rotation(cube, quaternion_product(&orientation_change, &cube->world_rotation));

    vec3 position_change = gm_vec3_scalar_product(dt, linear_velocity);
    graphics_entity_set_position(cube, gm_vec4_add(cube->world_position, (vec4){position_change.x, position_change.y, position_change.z, 0.0f}));

    vec4 vx_wc = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[2].position);
    vec4 vx_diff = gm_vec4_subtract(vx_wc, old_vx_wc);
    printf("Vertex 2 position difference: <%f, %f, %f>\n", vx_diff.x, vx_diff.y, vx_diff.z);

    vec3 rA_after_update = gm_vec4_to_vec3(gm_vec4_subtract(vx_wc, cube->world_position));
    vec3 vP1_after_update = gm_vec3_add(linear_velocity, gm_vec3_cross(angular_velocity, rA_after_update));
    printf("Point velocity after update: <%f, %f, %f>\n", vP1_after_update.x, vP1_after_update.y, vP1_after_update.z);
    printf("----------------------------------\n");
}

void physics_simulate(Entity* cube, Entity* plane, r32 plane_y, r32 dt, Physics_Force* forces) {
    physics_update_momenta_based_on_forces(cube, dt, forces);

    r32 dt_f = dt / 10.0f;
    for (u32 i = 0; i < 10; ++i) {
        vec4 old_position = cube->world_position;
        Quaternion old_rotation = cube->world_rotation;
        physics_update(cube, dt_f);

        Collision_Point* collision_points = collision_get_plane_cube_points(cube, plane_y);
        if (array_get_length(collision_points) > 0) {
            if (array_get_length(collision_points) > 1) {
                printf("double stuff\n");
            }
            graphics_entity_set_position(cube, old_position);
            graphics_entity_set_rotation(cube, old_rotation);
            for (u32 j = 0; j < array_get_length(collision_points); ++j) {
                vec4 new_pos = gm_mat4_multiply_vec4(&cube->model_matrix, cube->mesh.vertices[collision_points[j].vertex_index].position);
                collision_points[j].collision_point = gm_vec4_to_vec3(new_pos);
                apply_impulse(cube, plane, &collision_points[j], 0, 0.5f);
            }

            printf("Update because of collision...\n");
            physics_update(cube, dt_f);
        }
    }
}

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