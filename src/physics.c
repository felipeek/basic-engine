#include "physics.h"
#include <light_array.h>

#define MAX_MASS_TO_CONSIDER_STATIC_BODY 10000.0f

typedef struct {
    vec3 position_factor;
    vec3 rotation_factor;
} Jacobian_Entry;

typedef struct {
    vec3 linear_velocity;
    vec3 angular_velocity;
} Body_Velocity;

typedef struct {
    vec3 force;
    vec3 torque;
} Constraint_Reaction;

typedef struct {
    vec3 linear_velocity;
    vec3 angular_velocity;
} Constraint_Velocity;

static Jacobian_Entry retrieve_Jsp_entry(Jacobian_Entry* Jsp, u32 i, u32 j) {
    assert(j == 0 || j == 1);
    return Jsp[i * 2 + j];
}

static Jacobian_Entry retrieve_Bsp_entry(Jacobian_Entry* Bsp, u32 i, u32 j) {
    assert(i == 0 || i == 1);
    return Bsp[j * 2 + i];
}

static u32 retrieve_Jmap_entry(u32* Jmap, u32 i, u32 j) {
    assert(j == 0 || j == 1);
    return Jmap[i * 2 + j];
}

static Constraint_Velocity* compute_constraint_derivates(Jacobian_Entry* Jsp, u32* Jmap, Body_Velocity* V) {
    u32 s = array_length(V);
    Constraint_Velocity* velocities = array_new_len(Constraint_Velocity, s);
    for (u32 i = 0; i < s; ++i) {
        u32 b1 = retrieve_Jmap_entry(Jmap, i, 0);
        u32 b2 = retrieve_Jmap_entry(Jmap, i, 1);
        Constraint_Velocity sum;
        sum.angular_velocity = (vec3){0.0f, 0.0f, 0.0f};
        sum.linear_velocity = (vec3){0.0f, 0.0f, 0.0f};
        if (b1 > 0) {
            Jacobian_Entry jsp_entry = retrieve_Jsp_entry(Jsp, i, 0);
            Body_Velocity body_velocity = V[b1];
            sum.linear_velocity.x += jsp_entry.position_factor.x * body_velocity.linear_velocity.x;
            sum.linear_velocity.y += jsp_entry.position_factor.y * body_velocity.linear_velocity.y;
            sum.linear_velocity.z += jsp_entry.position_factor.z * body_velocity.linear_velocity.z;
            sum.angular_velocity.x += jsp_entry.rotation_factor.x * body_velocity.angular_velocity.x;
            sum.angular_velocity.y += jsp_entry.rotation_factor.y * body_velocity.angular_velocity.y;
            sum.angular_velocity.z += jsp_entry.rotation_factor.z * body_velocity.angular_velocity.z;
        }

        Jacobian_Entry jsp_entry = retrieve_Jsp_entry(Jsp, i, 1);
        Body_Velocity body_velocity = V[b2];
        sum.linear_velocity.x += jsp_entry.position_factor.x * body_velocity.linear_velocity.x;
        sum.linear_velocity.y += jsp_entry.position_factor.y * body_velocity.linear_velocity.y;
        sum.linear_velocity.z += jsp_entry.position_factor.z * body_velocity.linear_velocity.z;
        sum.angular_velocity.x += jsp_entry.rotation_factor.x * body_velocity.angular_velocity.x;
        sum.angular_velocity.y += jsp_entry.rotation_factor.y * body_velocity.angular_velocity.y;
        sum.angular_velocity.z += jsp_entry.rotation_factor.z * body_velocity.angular_velocity.z;

        velocities[i] = sum;
    }
}

// Fc = J^T * lambda
static Constraint_Reaction* compute_constraint_reactions(Jacobian_Entry* Jsp, u32* Jmap, r32* lambda) {
    u32 n = array_length(Jsp) / 2;
    u32 s = array_length(lambda);
    Constraint_Reaction* Fc = array_new_len(Constraint_Reaction, n);

    for (u32 i = 0; i < n; ++i) {
        Constraint_Reaction cr;
        cr.force = (vec3){0.0f, 0.0f, 0.0f};
        cr.torque = (vec3){0.0f, 0.0f, 0.0f};
        array_push(Fc, cr);
    }

    for (u32 i = 0; i < s; ++i) {
        u32 b1 = retrieve_Jmap_entry(Jmap, i, 0);
        u32 b2 = retrieve_Jmap_entry(Jmap, i, 1);
        if (b1 > 0) {
            Jacobian_Entry jsp_entry = retrieve_Jsp_entry(Jsp, i, 0);
            Fc[b1].force = gm_vec3_add(Fc[b1].force,
                gm_vec3_scalar_product(lambda[i], jsp_entry.position_factor));
            Fc[b1].torque = gm_vec3_add(Fc[b1].torque,
                gm_vec3_scalar_product(lambda[i], jsp_entry.rotation_factor));
        }

        Jacobian_Entry jsp_entry = retrieve_Jsp_entry(Jsp, i, 1);
        Fc[b2].force = gm_vec3_add(Fc[b2].force,
            gm_vec3_scalar_product(lambda[i], jsp_entry.position_factor));
        Fc[b2].torque = gm_vec3_add(Fc[b2].torque,
            gm_vec3_scalar_product(lambda[i], jsp_entry.rotation_factor));
    }

    return Fc;
}

// a = B * lambda
static Constraint_Reaction* compute_a(Jacobian_Entry* Bsp, u32* Jmap, r32* lambda) {
    u32 n = array_length(Bsp) / 2;
    u32 s = array_length(lambda);
    Constraint_Reaction* a = array_new_len(Constraint_Reaction, n);

    for (u32 i = 0; i < n; ++i) {
        Constraint_Reaction cr;
        cr.force = (vec3){0.0f, 0.0f, 0.0f};
        cr.torque = (vec3){0.0f, 0.0f, 0.0f};
        array_push(a, cr);
    }

    for (u32 i = 0; i < s; ++i) {
        u32 b1 = retrieve_Jmap_entry(Jmap, i, 0);
        u32 b2 = retrieve_Jmap_entry(Jmap, i, 1);
        if (b1 > 0) {
            Jacobian_Entry bsp_entry = retrieve_Bsp_entry(Bsp, 0, i);
            a[b1].force = gm_vec3_add(a[b1].force,
                gm_vec3_scalar_product(lambda[i], bsp_entry.position_factor));
            a[b1].torque = gm_vec3_add(a[b1].torque,
                gm_vec3_scalar_product(lambda[i], bsp_entry.rotation_factor));
        }

        Jacobian_Entry bsp_entry = retrieve_Bsp_entry(Bsp, 1, i);
        a[b2].force = gm_vec3_add(a[b2].force,
            gm_vec3_scalar_product(lambda[i], bsp_entry.position_factor));
        a[b2].torque = gm_vec3_add(a[b2].torque,
            gm_vec3_scalar_product(lambda[i], bsp_entry.rotation_factor));
    }

    return a;
}

// Jsp = array of <Ji1, Ji2> pairs, where Jij contains the position and rotation of rigid body j,
// related to constraint i
// Jmap = array of <bi1, bi2> pairs, where bij is the index of a rigid body
// Bsp is the representation of B = M^-1 * J^T
static r32* solve_system_via_projected_gauss_seidel(Jacobian_Entry* Jsp, u32* Jmap, Jacobian_Entry* Bsp,
    r32* previous_lambda, Jacobian_Entry* eta) {
    r32* lambda = array_copy(previous_lambda);
    Constraint_Reaction* a = compute_a(Bsp, Jmap, lambda);
    u32 s = array_length(lambda);
    Jacobian_Entry* d = array_new_len(Jacobian_Entry, s);
    for (u32 i = 0; i < s; ++i) {
        d[i].position_factor.x =
            retrieve_Jsp_entry(Jsp, i, 0).position_factor.x * retrieve_Bsp_entry(Bsp, 0, i).position_factor.x +
            retrieve_Jsp_entry(Jsp, i, 1).position_factor.x * retrieve_Bsp_entry(Bsp, 1, i).position_factor.x;
        d[i].position_factor.y =
            retrieve_Jsp_entry(Jsp, i, 0).position_factor.y * retrieve_Bsp_entry(Bsp, 0, i).position_factor.y +
            retrieve_Jsp_entry(Jsp, i, 1).position_factor.y * retrieve_Bsp_entry(Bsp, 1, i).position_factor.y;
        d[i].position_factor.z =
            retrieve_Jsp_entry(Jsp, i, 0).position_factor.z * retrieve_Bsp_entry(Bsp, 0, i).position_factor.z +
            retrieve_Jsp_entry(Jsp, i, 1).position_factor.z * retrieve_Bsp_entry(Bsp, 1, i).position_factor.z;
        d[i].rotation_factor.x =
            retrieve_Jsp_entry(Jsp, i, 0).rotation_factor.x * retrieve_Bsp_entry(Bsp, 0, i).rotation_factor.x +
            retrieve_Jsp_entry(Jsp, i, 1).rotation_factor.x * retrieve_Bsp_entry(Bsp, 1, i).rotation_factor.x;
        d[i].rotation_factor.y =
            retrieve_Jsp_entry(Jsp, i, 0).rotation_factor.y * retrieve_Bsp_entry(Bsp, 0, i).rotation_factor.y +
            retrieve_Jsp_entry(Jsp, i, 1).rotation_factor.y * retrieve_Bsp_entry(Bsp, 1, i).rotation_factor.y;
        d[i].rotation_factor.z =
            retrieve_Jsp_entry(Jsp, i, 0).rotation_factor.z * retrieve_Bsp_entry(Bsp, 0, i).rotation_factor.z +
            retrieve_Jsp_entry(Jsp, i, 1).rotation_factor.z * retrieve_Bsp_entry(Bsp, 1, i).rotation_factor.z;
    }

    for (u32 iter = 0; iter < 10; ++iter) {
        for (u32 i = 0; i < s; ++i) {
            u32 b1 = retrieve_Jmap_entry(Jmap, i, 0);
            u32 b2 = retrieve_Jmap_entry(Jmap, i, 1);

            Jacobian_Entry tmp;
            tmp.position_factor.x =
                retrieve_Jsp_entry(Jsp, i, 0).position_factor.x * a[b1].force.x +
                retrieve_Jsp_entry(Jsp, i, 1).position_factor.x * a[b2].force.x;
            r32 delta_lambda = 
        }
    }
}

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