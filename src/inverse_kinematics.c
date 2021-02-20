#include "inverse_kinematics.h"
#include "matrix.h"
#include <dynamic_array.h>

static vec3 get_position_from_matrix(mat4 m)
{
	vec3 position;
	position.x = m.data[0][3];
	position.y = m.data[1][3];
	position.z = m.data[2][3];
	return position;
}

static u32 get_joint_height(Hierarchical_Model_Joint* leaf_joint)
{
	Hierarchical_Model_Joint* current = leaf_joint;
	u32 h = 0;
	while (current)
	{
		++h;
		current = current->parent;
	}
	return h;
}

static vec3 get_end_effector(Hierarchical_Model_Joint* leaf_joint)
{
	// Here, we are leveraging the fact that we know the exact shape of the joint...
	// We are dealing with a 2x2 cube shifted one unit in the X axis...
	// If the model changes, this will not work... the end-effector needs to be re-evaluated
	mat4 last_joint_transform = leaf_joint->e.model_matrix;
	vec4 end_effector_position_in_local_coordinates = (vec4){2.0f, 0.0f, 0.0f, 1.0f};
	vec4 end_effector_position = gm_mat4_multiply_vec4(&last_joint_transform, end_effector_position_in_local_coordinates);
	return gm_vec4_to_vec3(end_effector_position);
}

static vec3 clamp_mag(vec3 w, r32 d)
{
	r32 l = gm_vec3_length(w);
	if (l <= d)
		return w;
	return gm_vec3_scalar_product(d / l, w);
}

static Matrix generate_jacobian(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, vec3* v, vec3 end_effector)
{
	// We start by collecting all joint positions
	vec3* joint_positions = array_create(vec3, 1);
	vec3* joint_rotation_axis = array_create(vec3, 1);
	Hierarchical_Model_Joint* current_joint = leaf_joint;
	while (current_joint)
	{
		vec3 current_joint_position = get_position_from_matrix(current_joint->e.model_matrix);
		vec3 current_joint_rotation_axis = current_joint->rotation_axis;
		array_push(joint_positions, &current_joint_position);
		array_push(joint_rotation_axis, &current_joint_rotation_axis);
		current_joint = current_joint->parent;
	}

	vec3 E = end_effector;
	vec3 G = target_point;

	// Calculate the vector 'v' (left side of the equation)
	*v = gm_vec3_subtract(G, E);
	//const r32 D_max = 0.1f;
	//*v = clamp_mag(*v, D_max);

	// Finally, calculate the jacobian
	Matrix jacobian = matrix_create(3, array_get_length(joint_positions));
	for (u32 i = 0; i < jacobian.columns; ++i)
	{
		// Since the joint positions array start with the leaf joint and end with the root joint,
		// we need to iterate through it from the end to the beggining
		u32 joint_index = array_get_length(joint_positions) - 1 - i;
		vec3 V = gm_vec3_subtract(E, joint_positions[joint_index]);
		vec3 direction = gm_vec3_cross(joint_rotation_axis[joint_index], V);
		jacobian.data[0][i] = direction.x;
		jacobian.data[1][i] = direction.y;
		jacobian.data[2][i] = direction.z;
	}

	array_release(joint_positions);
	array_release(joint_rotation_axis);
	return jacobian;
}

static Matrix get_rotation_angles_using_transpose(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, vec3 end_effector)
{
	vec3 v;
	Matrix jacobian = generate_jacobian(leaf_joint, target_point, &v, end_effector);
	Matrix jacobian_transpose = matrix_transpose(&jacobian);
	Matrix v_m = matrix_from_vec3(v);
	Matrix angles = matrix_multiply(&jacobian_transpose, &v_m);
	matrix_destroy(&v_m);
	matrix_destroy(&jacobian_transpose);
	matrix_destroy(&jacobian);
	return angles;
}

static Matrix get_rotation_angles_using_damped_least_squares(Hierarchical_Model_Joint* leaf_joint, vec3 target_point,
	vec3 end_effector, r32 lambda)
{
	vec3 v;
	Matrix jacobian = generate_jacobian(leaf_joint, target_point, &v, end_effector);
	Matrix jacobian_transposed = matrix_transpose(&jacobian);
	Matrix jj = matrix_multiply(&jacobian, &jacobian_transposed);
	Matrix I = matrix_identity(jj.rows);
	Matrix lambda_I = matrix_scalar_multiply(&I, lambda * lambda);
	Matrix sum = matrix_sum(&jj, &lambda_I);
	Matrix sum_inv;
	if (matrix_invert(&sum, &sum_inv))
	{
		fprintf(stderr, "Failed to inverse the matrix... falling back to transpose\n");
		matrix_destroy(&jacobian);
		matrix_destroy(&jacobian_transposed);
		matrix_destroy(&jj);
		matrix_destroy(&I);
		matrix_destroy(&lambda_I);
		matrix_destroy(&sum);
		return get_rotation_angles_using_transpose(leaf_joint, target_point, end_effector);
	}
	Matrix result_matrix = matrix_multiply(&jacobian_transposed, &sum_inv);
	Matrix v_m = matrix_from_vec3(v);
	Matrix angles = matrix_multiply(&result_matrix, &v_m);
	matrix_destroy(&v_m);
	matrix_destroy(&jacobian);
	matrix_destroy(&jacobian_transposed);
	matrix_destroy(&jj);
	matrix_destroy(&I);
	matrix_destroy(&lambda_I);
	matrix_destroy(&sum);
	matrix_destroy(&result_matrix);
	return angles;
}

static Matrix get_rotation_angles_using_pseudo_inverse(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, vec3 end_effector)
{
	vec3 v;
	Matrix jacobian = generate_jacobian(leaf_joint, target_point, &v, end_effector);
	Matrix jacobian_transposed = matrix_transpose(&jacobian);

	// calculate beta
	Matrix m = matrix_multiply(&jacobian, &jacobian_transposed);
	Matrix m_inv;
	if (matrix_invert(&m, &m_inv))
	{
		fprintf(stderr, "Failed to inverse the matrix... falling back to transpose\n");
		matrix_destroy(&jacobian);
		matrix_destroy(&jacobian_transposed);
		matrix_destroy(&m);
		return get_rotation_angles_using_transpose(leaf_joint, target_point, end_effector);
	}
	
	Matrix v_m = matrix_from_vec3(v);
	Matrix beta_m = matrix_multiply(&m_inv, &v_m);
	Matrix angles = matrix_multiply(&jacobian_transposed, &beta_m);
	matrix_destroy(&m);
	matrix_destroy(&m_inv);
	matrix_destroy(&v_m);
	matrix_destroy(&beta_m);
	matrix_destroy(&jacobian_transposed);
	matrix_destroy(&jacobian);
	return angles;
}

static Matrix get_rotation_angles_using_inverse(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, vec3 end_effector)
{
	vec3 v;
	Matrix jacobian = generate_jacobian(leaf_joint, target_point, &v, end_effector);
	r32 det = matrix_determinant(&jacobian); //printf("determinant: %.3f\n", det);
	//if (fabsf(det) < 0.1f)
	//{
	//	//fprintf(stderr, "Jacobian determinant too low... falling back to transpose\n");
	//	matrix_destroy(&jacobian);
	//	return get_rotation_angles_using_transpose(leaf_joint, target_point, end_effector);
	//}
	Matrix jacobian_inverse;
	if (matrix_invert(&jacobian, &jacobian_inverse))
	{
		fprintf(stderr, "Failed to inverse the jacobian... falling back to transpose\n");
		matrix_destroy(&jacobian);
		return get_rotation_angles_using_transpose(leaf_joint, target_point, end_effector);
	}
	Matrix v_m = matrix_from_vec3(v);
	Matrix angles = matrix_multiply(&jacobian_inverse, &v_m);
	matrix_destroy(&v_m);
	matrix_destroy(&jacobian_inverse);
	matrix_destroy(&jacobian);
	return angles;
}

s32 rotate_joints_towards_target_point(Hierarchical_Model_Joint* leaf_joint, vec3 target_point, IK_Method ik_method, r32 lambda)
{
	const r32 MINIMUM_DISTANCE_TO_CONSIDER_CORRECT = 0.1f;
	const r32 DELTA = 0.0001f;

	vec3 end_effector = get_end_effector(leaf_joint);

	// If the end effector is close enough to the target point, we stop animating
	if (gm_vec3_length(gm_vec3_subtract(target_point, end_effector)) < MINIMUM_DISTANCE_TO_CONSIDER_CORRECT)
		return 0;
	
	Matrix angles;
	switch (ik_method)
	{
		default:
		case IK_METHOD_INVERSE: {
			u32 height = get_joint_height(leaf_joint);
			if (height != 3)
			{
				fprintf(stderr, "Cannot use INVERSE ik method when number of joints is not 3!\n");
				return 1;
			}
			angles = get_rotation_angles_using_inverse(leaf_joint, target_point, end_effector);
		} break;
		case IK_METHOD_PSEUDO_INVERSE: {
			angles = get_rotation_angles_using_pseudo_inverse(leaf_joint, target_point, end_effector);
		} break;
		case IK_METHOD_TRANSPOSE: {
			angles = get_rotation_angles_using_transpose(leaf_joint, target_point, end_effector);
		} break;
		case IK_METHOD_DAMPED_LEAST_SQUARES: {
			angles = get_rotation_angles_using_damped_least_squares(leaf_joint, target_point, end_effector, lambda);
		} break;
	}

	Hierarchical_Model_Joint* current_joint = leaf_joint;
	//printf("Angles: <%.3f, %.3f, %.3f>\n", angles.data[0][0], angles.data[1][0], angles.data[2][0]);
	for (s32 i = angles.rows - 1; i >= 0; --i)
	{
		const float MAX_ANGLE = 5.0f;
		//r32 angle = MAX(MIN(angles.data[i][0], MAX_ANGLE), -MAX_ANGLE);
		r32 angle = angles.data[i][0];
		Quaternion rotation = quaternion_new(current_joint->rotation_axis, DELTA * angle);
		current_joint->rotation = quaternion_product(&rotation, &current_joint->rotation);
		current_joint = current_joint->parent;
	}

	matrix_destroy(&angles);
	return 1;
}