#include "lookat.h"
#include "common.h"

static void lookat_camera_recalculate_view_matrix(Lookat_Camera* camera)
{
	camera->view_matrix = camera_common_calculate_view_matrix(camera->position, &camera->rotation);
}

static void lookat_camera_recalculate_projection_matrix(Lookat_Camera* camera)
{
	camera->projection_matrix = camera_common_calculate_projection_matrix(camera->near_plane, camera->far_plane, camera->fov);
}

vec3 lookat_camera_get_position(const Lookat_Camera* camera)
{
	return camera->position;
}

Quaternion lookat_camera_get_rotation(const Lookat_Camera* camera)
{
	return camera->rotation;
}

mat4 lookat_camera_get_view_matrix(const Lookat_Camera* camera)
{
	return camera->view_matrix;
}

mat4 lookat_camera_get_projection_matrix(const Lookat_Camera* camera)
{
	return camera->projection_matrix;
}

void lookat_camera_set_position(Lookat_Camera* camera, vec3 position)
{
	camera->position = position;
	lookat_camera_recalculate_view_matrix(camera);
}

void lookat_camera_set_near_plane(Lookat_Camera* camera, r32 near_plane)
{
	camera->near_plane = near_plane;
	lookat_camera_recalculate_projection_matrix(camera);
}

void lookat_camera_set_far_plane(Lookat_Camera* camera, r32 far_plane)
{
	camera->far_plane = far_plane;
	lookat_camera_recalculate_projection_matrix(camera);
}

void lookat_camera_set_fov(Lookat_Camera* camera, r32 fov)
{
	camera->fov = fov;
	lookat_camera_recalculate_projection_matrix(camera);
}

vec3 lookat_camera_rotate(Lookat_Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	if (camera->consider_roll)
		camera->rotation = camera_common_rotate_considering_click_coords(&camera->rotation, x_diff, y_diff, mouse_x, mouse_y);
	else
		camera->rotation = camera_common_rotate(&camera->rotation, x_diff, y_diff);

	// get the view from the quaternion (we could also recalculate the view matrix and use camera_common_get_view)
	vec3 camera_view = gm_vec3_negative(quaternion_get_forward_inverted(&camera->rotation));
	camera->position = gm_vec3_subtract(camera->lookat_position, gm_vec3_scalar_product(camera->lookat_distance, camera_view));
	lookat_camera_recalculate_view_matrix(camera);
}

void lookat_camera_force_matrix_recalculation(Lookat_Camera* camera)
{
	lookat_camera_recalculate_view_matrix(camera);
	lookat_camera_recalculate_projection_matrix(camera);
}

// Look At Camera specific functions

void lookat_camera_init(Lookat_Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov,
	int consider_roll)
{
	vec3 camera_view = (vec3) { 0.0f, 0.0f, -1.0f };
	vec3 position = gm_vec3_subtract(lookat_position, gm_vec3_scalar_product(lookat_distance, gm_vec3_normalize(camera_view)));
	*camera = (Lookat_Camera){0};
	camera->position = position;
	camera->rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;
	camera->lookat_position = lookat_position;
	camera->lookat_distance = lookat_distance;
	camera->consider_roll = consider_roll;
	
	lookat_camera_recalculate_view_matrix(camera);
	lookat_camera_recalculate_projection_matrix(camera);
}

r32 lookat_camera_get_lookat_distance(const Lookat_Camera* camera)
{
	return camera->lookat_distance;
}

vec3 lookat_camera_get_lookat_position(const Lookat_Camera* camera)
{
	return camera->lookat_position;
}

void lookat_camera_set_lookat_position(Lookat_Camera* camera, vec3 lookat_position)
{
	camera->lookat_position = lookat_position;
	vec3 camera_view = camera_common_get_view(&camera->view_matrix);
	camera->position = gm_vec3_subtract(camera->lookat_position, gm_vec3_scalar_product(camera->lookat_distance, camera_view));
	lookat_camera_recalculate_view_matrix(camera);
}

void lookat_camera_set_lookat_distance(Lookat_Camera* camera, r32 lookat_distance)
{
	camera->lookat_distance = lookat_distance;
	vec3 camera_view = camera_common_get_view(&camera->view_matrix);
	camera->position = gm_vec3_subtract(camera->lookat_position, gm_vec3_scalar_product(camera->lookat_distance, camera_view));
	lookat_camera_recalculate_view_matrix(camera);
}