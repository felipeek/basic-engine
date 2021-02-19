#include "inverse_kinematics.h"
#include "matrix.h"
#include <dynamic_array.h>

static vec3 rotation_axis = (vec3){0.0f, 0.0f, 1.0f};

static vec3 get_position_from_matrix(mat4 m)
{
	vec3 position;
	position.x = m.data[0][3];
	position.y = m.data[1][3];
	position.z = m.data[2][3];
	return position;
}

static Matrix generate_jacobian(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, vec3* v)
{
	// We start by collecting all joint positions
	vec3* joint_positions = array_create(vec3, 1);
	Hierarchical_Model_Joint* current_joint = leaf_joint;
	while (current_joint)
	{
		vec3 current_joint_position = get_position_from_matrix(current_joint->e.model_matrix);
		array_push(joint_positions, &current_joint_position);
		current_joint = current_joint->parent;
	}

	// Now, we calculate the end-effector position
	// Here, we are leveraging the fact that we know the exact shape of the joint...
	// We are dealing with a 2x2 cube shifted one unit in the X axis...
	// If the model changes, this will not work... the end-effector needs to be re-evaluated
	mat4 last_joint_transform = leaf_joint->e.model_matrix;
	vec4 end_effector_position_in_local_coordinates = (vec4){2.0f, 0.0f, 0.0f, 1.0f};
	vec4 end_effector_position = gm_mat4_multiply_vec4(&last_joint_transform, end_effector_position_in_local_coordinates);

	vec3 E = gm_vec4_to_vec3(end_effector_position);
	vec3 G = target_point;

	// Calculate the vector 'v' (left side of the equation)
	*v = gm_vec3_subtract(G, E);

	// Finally, calculate the jacobian
	Matrix jacobian = matrix_create(3, array_get_length(joint_positions));
	for (u32 i = 0; i < jacobian.columns; ++i)
	{
		// Since the joint positions array start with the leaf joint and end with the root joint,
		// we need to iterate through it from the end to the beggining
		u32 joint_index = array_get_length(joint_positions) - 1 - i;
		vec3 V = gm_vec3_subtract(E, joint_positions[joint_index]);
		vec3 direction = gm_vec3_cross(rotation_axis, V);
		jacobian.data[0][i] = direction.x;
		jacobian.data[1][i] = direction.y;
		jacobian.data[2][i] = direction.z;
	}

	return jacobian;
}

static vec3 calculate_beta(Matrix jacobian, vec3 v)
{
	Matrix jacobian_transposed = matrix_transpose(&jacobian);
	Matrix m = matrix_multiply(&jacobian, &jacobian_transposed);
	
	Matrix v_m = matrix_from_vec3(v);
	Matrix beta_m = matrix_multiply(&m, &v_m);
	vec3 beta = (vec3){beta_m.data[0][0], beta_m.data[1][0], beta_m.data[2][0]};

	matrix_destroy(&jacobian_transposed);
	matrix_destroy(&m);
	matrix_destroy(&v_m);
	matrix_destroy(&beta_m);

	return beta;
}

static Matrix calculate_angles(Matrix jacobian, vec3 beta)
{
	Matrix jacobian_transposed = matrix_transpose(&jacobian);
	Matrix beta_m = matrix_from_vec3(beta);
	Matrix angles = matrix_multiply(&jacobian_transposed, &beta_m);
	matrix_destroy(&jacobian_transposed);
	matrix_destroy(&beta_m);
	return angles;
}

void rotate_joints_towards_target_point(Hierarchical_Model_Joint* leaf_joint, vec3 target_point)
{
	vec3 v;
	Matrix jacobian = generate_jacobian(leaf_joint, target_point, &v);
	vec3 beta = calculate_beta(jacobian, v);
	Matrix angles = calculate_angles(jacobian, beta);

	const r32 delta = 0.01f;
	Hierarchical_Model_Joint* current_joint = leaf_joint;
	for (s32 i = angles.rows - 1; i >= 0; --i)
	{
		r32 angle = angles.data[i][0];
		Quaternion rotation = quaternion_new(rotation_axis, delta * angle);
		current_joint->rotation = quaternion_product(&rotation, &current_joint->rotation);
		current_joint = current_joint->parent;
	}

	matrix_destroy(&jacobian);
}