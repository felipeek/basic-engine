#include "free.h"
#include "common.h"

static void free_camera_recalculate_view_matrix(Free_Camera* camera)
{
	const Quaternion camera_rotation = free_camera_get_rotation(camera);
	camera->view_matrix = camera_common_calculate_view_matrix(camera->position, &camera_rotation);
}

static void free_camera_recalculate_projection_matrix(Free_Camera* camera)
{
	camera->projection_matrix = camera_common_calculate_projection_matrix(camera->near_plane, camera->far_plane, camera->fov);
}

vec3 free_camera_get_position(const Free_Camera* camera)
{
	return camera->position;
}

Quaternion free_camera_get_rotation(const Free_Camera* camera)
{
	if (camera->lock_rotation)
		return quaternion_product(&camera->rotation, &camera->y_rotation);
	
	return camera->rotation;
}

mat4 free_camera_get_view_matrix(const Free_Camera* camera)
{
	return camera->view_matrix;
}

mat4 free_camera_get_projection_matrix(const Free_Camera* camera)
{
	return camera->projection_matrix;
}

void free_camera_set_position(Free_Camera* camera, vec3 position)
{
	camera->position = position;
	free_camera_recalculate_view_matrix(camera);
}

void free_camera_set_near_plane(Free_Camera* camera, r32 near_plane)
{
	camera->near_plane = near_plane;
	free_camera_recalculate_projection_matrix(camera);
}

void free_camera_set_far_plane(Free_Camera* camera, r32 far_plane)
{
	camera->far_plane = far_plane;
	free_camera_recalculate_projection_matrix(camera);
}

void free_camera_set_fov(Free_Camera* camera, r32 fov)
{
	camera->fov = fov;
	free_camera_recalculate_projection_matrix(camera);
}

vec3 free_camera_rotate(Free_Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	if (camera->lock_rotation)
	{
		x_diff = -x_diff;
		// If the location is locked, we need to rotate using two different quaternions
		camera->y_rotation = camera_common_rotate_x(&camera->y_rotation, x_diff);

		// X rotation is constrained
		vec3 right = quaternion_get_right_inverted(&camera->rotation);
		right = gm_vec3_normalize(right);
		Quaternion q = quaternion_new(right, y_diff);
		camera->rotation = quaternion_product(&camera->rotation, &q);
		camera->rotation = quaternion_normalize(&camera->rotation);
		free_camera_recalculate_view_matrix(camera);
	}
	else
		camera->rotation = camera_common_rotate(&camera->rotation, x_diff, y_diff);

	free_camera_recalculate_view_matrix(camera);
}

void free_camera_force_matrix_recalculation(Free_Camera* camera)
{
	free_camera_recalculate_view_matrix(camera);
	free_camera_recalculate_projection_matrix(camera);
}

void free_camera_init(Free_Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation)
{
	*camera = (Free_Camera){0};
	camera->position = position;
	camera->rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->y_rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;
	camera->lock_rotation = lock_rotation;
	
	free_camera_recalculate_view_matrix(camera);
	free_camera_recalculate_projection_matrix(camera);
}