#include "free.h"
#include "util.h"

void free_camera_init(Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation)
{
	*camera = (Camera){0};

	camera->type = CAMERA_FREE;
	camera->position = position;
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;

	camera->free_camera.rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->free_camera.y_rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->free_camera.lock_rotation = lock_rotation;
	
	camera_force_matrix_recalculation(camera);
}

Quaternion free_camera_get_rotation(const Camera* camera)
{
	assert(camera->type == CAMERA_FREE);
	const Free_Camera* free_camera = &camera->free_camera;

	if (free_camera->lock_rotation)
		return quaternion_product(&free_camera->rotation, &free_camera->y_rotation);
	
	return free_camera->rotation;
}

void free_camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	assert(camera->type == CAMERA_FREE);
	Free_Camera* free_camera = &camera->free_camera;

	if (free_camera->lock_rotation)
	{
		// If the location is locked, we need to rotate using two different quaternions
		free_camera->y_rotation = camera_util_rotate_x(&free_camera->y_rotation, x_diff);

		// X rotation is constrained
		vec3 right = quaternion_get_right_inverted(&free_camera->rotation);
		right = gm_vec3_normalize(right);
		Quaternion q = quaternion_new(right, y_diff);
		free_camera->rotation = quaternion_product(&free_camera->rotation, &q);
		free_camera->rotation = quaternion_normalize(&free_camera->rotation);
	}
	else
		free_camera->rotation = camera_util_rotate(&free_camera->rotation, x_diff, y_diff);

	camera_recalculate_view_matrix(camera);
}
