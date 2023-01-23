#include "lookat.h"
#include "util.h"

void lookat_camera_init(Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov,
	int consider_roll)
{
	vec3 camera_view = (vec3) { 0.0f, 0.0f, -1.0f };
	vec3 position = gm_vec3_subtract(lookat_position, gm_vec3_scalar_product(lookat_distance, gm_vec3_normalize(camera_view)));

	*camera = (Camera){0};

	camera->type = CAMERA_LOOKAT;
	camera->position = position;
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;

	camera->lookat_camera.rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->lookat_camera.lookat_position = lookat_position;
	camera->lookat_camera.lookat_distance = lookat_distance;
	camera->lookat_camera.consider_roll = consider_roll;
	
	camera_force_matrix_recalculation(camera);
}

Quaternion lookat_camera_get_rotation(const Camera* camera)
{
	assert(camera->type == CAMERA_LOOKAT);
	const Lookat_Camera* lookat_camera = &camera->lookat_camera;

	return lookat_camera->rotation;
}

vec3 lookat_camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	assert(camera->type == CAMERA_LOOKAT);
	Lookat_Camera* lookat_camera = &camera->lookat_camera;

	if (lookat_camera->consider_roll)
		lookat_camera->rotation = camera_util_rotate_considering_click_coords(&lookat_camera->rotation, x_diff, y_diff, mouse_x, mouse_y);
	else
		lookat_camera->rotation = camera_util_rotate(&lookat_camera->rotation, x_diff, y_diff);

	// get the view from the quaternion (we could also recalculate the view matrix and use camera_util_get_view)
	vec3 camera_view = gm_vec3_negative(quaternion_get_forward_inverted(&lookat_camera->rotation));
	camera->position = gm_vec3_subtract(lookat_camera->lookat_position, gm_vec3_scalar_product(lookat_camera->lookat_distance, camera_view));
	camera_recalculate_view_matrix(camera);
}

r32 lookat_camera_get_lookat_distance(const Camera* camera)
{
	assert(camera->type == CAMERA_LOOKAT);
	const Lookat_Camera* lookat_camera = &camera->lookat_camera;

	return lookat_camera->lookat_distance;
}

vec3 lookat_camera_get_lookat_position(const Camera* camera)
{
	assert(camera->type == CAMERA_LOOKAT);
	const Lookat_Camera* lookat_camera = &camera->lookat_camera;

	return lookat_camera->lookat_position;
}

void lookat_camera_set_lookat_position(Camera* camera, vec3 lookat_position)
{
	assert(camera->type == CAMERA_LOOKAT);
	Lookat_Camera* lookat_camera = &camera->lookat_camera;

	lookat_camera->lookat_position = lookat_position;
	vec3 camera_view = camera_get_view(camera);
	camera->position = gm_vec3_subtract(lookat_camera->lookat_position, gm_vec3_scalar_product(lookat_camera->lookat_distance, camera_view));
	camera_recalculate_view_matrix(camera);
}

void lookat_camera_set_lookat_distance(Camera* camera, r32 lookat_distance)
{
	assert(camera->type == CAMERA_LOOKAT);
	Lookat_Camera* lookat_camera = &camera->lookat_camera;
	
	lookat_camera->lookat_distance = lookat_distance;
	vec3 camera_view = camera_get_view(camera);
	camera->position = gm_vec3_subtract(lookat_camera->lookat_position, gm_vec3_scalar_product(lookat_camera->lookat_distance, camera_view));
	camera_recalculate_view_matrix(camera);
}