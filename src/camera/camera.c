#include "camera.h"
#include "common.h"

void camera_init_free(Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation)
{
	*camera = (Camera){0};
	camera->type = CAMERA_FREE;
	free_camera_init(&camera->free_camera, position, near_plane, far_plane, fov, lock_rotation);
}

void camera_init_lookat(Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov,
	int consider_mouse_coords_when_rotating)
{
	*camera = (Camera){0};
	camera->type = CAMERA_LOOKAT;
	lookat_camera_init(&camera->lookat_camera, lookat_position, lookat_distance, near_plane, far_plane, fov, consider_mouse_coords_when_rotating);
}

vec3 camera_get_view(const Camera* camera)
{
	mat4 view_matrix = camera_get_view_matrix(camera);
	return camera_common_get_view(&view_matrix);
}

vec3 camera_get_x_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	return camera_common_get_x_axis(&camera_rotation);
}

vec3 camera_get_y_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	return camera_common_get_y_axis(&camera_rotation);
}

vec3 camera_get_z_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	return camera_common_get_z_axis(&camera_rotation);
}

void camera_move_forward(Camera* camera, r32 amount)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 forward = quaternion_get_forward_inverted(&camera_rotation);
	forward = gm_vec3_scalar_product(amount, gm_vec3_normalize(forward));
	vec3 new_position = gm_vec3_add(gm_vec3_negative(forward), camera_get_position(camera));
	camera_set_position(camera, new_position);
}

void camera_move_right(Camera* camera, r32 amount)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 right = quaternion_get_right_inverted(&camera_rotation);
	right = gm_vec3_scalar_product(amount, gm_vec3_normalize(right));
	vec3 new_position = gm_vec3_add(right, camera_get_position(camera));
	camera_set_position(camera, new_position);
}

vec3 camera_get_position(const Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: return free_camera_get_position(&camera->free_camera);
		case CAMERA_LOOKAT: return lookat_camera_get_position(&camera->lookat_camera);
		default: assert(0);
	}
}

Quaternion camera_get_rotation(const Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: return free_camera_get_rotation(&camera->free_camera);
		case CAMERA_LOOKAT: return lookat_camera_get_rotation(&camera->lookat_camera);
		default: assert(0);
	}
}

mat4 camera_get_view_matrix(const Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: return free_camera_get_view_matrix(&camera->free_camera);
		case CAMERA_LOOKAT: return lookat_camera_get_view_matrix(&camera->lookat_camera);
		default: assert(0);
	}
}

mat4 camera_get_projection_matrix(const Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: return free_camera_get_projection_matrix(&camera->free_camera);
		case CAMERA_LOOKAT: return lookat_camera_get_projection_matrix(&camera->lookat_camera);
		default: assert(0);
	}
}

void camera_set_position(Camera* camera, vec3 position)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_set_position(&camera->free_camera, position); break;
		case CAMERA_LOOKAT: lookat_camera_set_position(&camera->lookat_camera, position); break;
		default: assert(0);
	}
}

void camera_set_near_plane(Camera* camera, r32 near_plane)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_set_near_plane(&camera->free_camera, near_plane); break;
		case CAMERA_LOOKAT: lookat_camera_set_near_plane(&camera->lookat_camera, near_plane); break;
		default: assert(0);
	}
}

void camera_set_far_plane(Camera* camera, r32 far_plane)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_set_far_plane(&camera->free_camera, far_plane); break;
		case CAMERA_LOOKAT: lookat_camera_set_far_plane(&camera->lookat_camera, far_plane); break;
		default: assert(0);
	}
}

void camera_set_fov(Camera* camera, r32 fov)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_set_fov(&camera->free_camera, fov); break;
		case CAMERA_LOOKAT: lookat_camera_set_fov(&camera->lookat_camera, fov); break;
		default: assert(0);
	}
}

void camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_rotate(&camera->free_camera, x_diff, y_diff, mouse_x, mouse_y); break;
		case CAMERA_LOOKAT: lookat_camera_rotate(&camera->lookat_camera, x_diff, y_diff, mouse_x, mouse_y); break;
		default: assert(0);
	}
}

void camera_force_matrix_recalculation(Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_force_matrix_recalculation(&camera->free_camera); break;
		case CAMERA_LOOKAT: lookat_camera_force_matrix_recalculation(&camera->lookat_camera); break;
		default: assert(0);
	}
}