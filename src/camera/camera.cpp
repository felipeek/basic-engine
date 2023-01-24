#include "camera.h"
#include "free.h"
#include "lookat.h"

extern dvec2 window_size;

vec3 camera_get_view(const Camera* camera)
{
	mat4 view_matrix = camera->view_matrix;
	vec3 camera_view = (vec3) { -view_matrix.data[2][0], -view_matrix.data[2][1], -view_matrix.data[2][2] };
	return gm_vec3_normalize(camera_view);
}

vec3 camera_get_x_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 right = quaternion_get_right_inverted(&camera_rotation);
	right = gm_vec3_normalize(right);
	return (vec3) { right.x, right.y, right.z };
}

vec3 camera_get_y_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 up = quaternion_get_up_inverted(&camera_rotation);
	up = gm_vec3_normalize(up);
	return (vec3) { up.x, up.y, up.z };
}

vec3 camera_get_z_axis(const Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 forward = quaternion_get_forward_inverted(&camera_rotation);
	forward = gm_vec3_normalize(forward);
	return (vec3) { forward.x, forward.y, forward.z };
}

void camera_move_forward(Camera* camera, r32 amount)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 forward = quaternion_get_forward_inverted(&camera_rotation);
	forward = gm_vec3_scalar_product(camera->movement_speed * amount, gm_vec3_normalize(forward));
	vec3 new_position = gm_vec3_add(gm_vec3_negative(forward), camera_get_position(camera));
	camera_set_position(camera, new_position);
}

void camera_move_right(Camera* camera, r32 amount)
{
	Quaternion camera_rotation = camera_get_rotation(camera);
	vec3 right = quaternion_get_right_inverted(&camera_rotation);
	right = gm_vec3_scalar_product(camera->movement_speed * amount, gm_vec3_normalize(right));
	vec3 new_position = gm_vec3_add(right, camera_get_position(camera));
	camera_set_position(camera, new_position);
}

vec3 camera_get_position(const Camera* camera)
{
	return camera->position;
}

Quaternion camera_get_rotation(const Camera* camera)
{
	switch (camera->type)
	{
		case CAMERA_FREE: return free_camera_get_rotation(camera);
		case CAMERA_LOOKAT: return lookat_camera_get_rotation(camera);
		default: assert(0);
	}
}

r32 camera_get_movement_speed(const Camera* camera)
{
	return camera->movement_speed;
}

r32 camera_get_rotation_speed(const Camera* camera)
{
	return camera->rotation_speed;
}

mat4 camera_get_view_matrix(const Camera* camera)
{
	return camera->view_matrix;
}

mat4 camera_get_projection_matrix(const Camera* camera)
{
	return camera->projection_matrix;
}

void camera_set_position(Camera* camera, vec3 position)
{
	camera->position = position;
	camera_recalculate_view_matrix(camera);
}

void camera_set_near_plane(Camera* camera, r32 near_plane)
{
	camera->near_plane = near_plane;
	camera_recalculate_projection_matrix(camera);
}

void camera_set_far_plane(Camera* camera, r32 far_plane)
{
	camera->far_plane = far_plane;
	camera_recalculate_projection_matrix(camera);
}

void camera_set_fov(Camera* camera, r32 fov)
{
	camera->fov = fov;
	camera_recalculate_projection_matrix(camera);
}

void camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	switch (camera->type)
	{
		case CAMERA_FREE: free_camera_rotate(camera, x_diff * camera->rotation_speed, y_diff * camera->rotation_speed,
			mouse_x, mouse_y); break;
		case CAMERA_LOOKAT: lookat_camera_rotate(camera, x_diff * camera->rotation_speed, y_diff * camera->rotation_speed,
			mouse_x, mouse_y); break;
		default: assert(0);
	}
}

void camera_set_movement_speed(Camera* camera, r32 movement_speed)
{
	camera->movement_speed = movement_speed;
}

void camera_set_rotation_speed(Camera* camera, r32 rotation_speed)
{
	camera->rotation_speed = rotation_speed;
}

void camera_force_matrix_recalculation(Camera* camera)
{
	camera_recalculate_view_matrix(camera);
	camera_recalculate_projection_matrix(camera);
}

void camera_recalculate_view_matrix(Camera* camera)
{
	Quaternion camera_rotation = camera_get_rotation(camera);

	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, -camera->position.x,
		0.0f, 1.0f, 0.0f, -camera->position.y,
		0.0f, 0.0f, 1.0f, -camera->position.z,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(&camera_rotation);
	camera->view_matrix = gm_mat4_multiply(&rotation_matrix, &translation_matrix);
}

void camera_recalculate_projection_matrix(Camera* camera)
{
	r32 near = camera->near_plane;
	r32 far = camera->far_plane;
	r32 fov = camera->fov;
	r32 top = (r32)fabs(near) * atanf(gm_radians(fov) / 2.0f);
	r32 bottom = -top;
	r32 right = top * ((r32)window_size.x / (r32)window_size.y);
	r32 left = -right;

	mat4 p = (mat4) {
		near, 0.0f, 0.0f, 0.0f,
		0.0f, near, 0.0f, 0.0f,
		0.0f, 0.0f, near + far, -near * far,
		0.0f, 0.0f, 1.0f, 0.0f
	};

	mat4 m = (mat4) {
		2.0f / (right - left), 0.0f, 0.0f, -(right + left) / (right - left),
		0.0f, 2.0f / (top - bottom), 0.0f, -(top + bottom) / (top - bottom),
		0.0f, 0.0f, 2.0f / (far - near), -(far + near) / (far - near),
		0.0f, 0.0f, 0.0f, 1.0f
	};

	// Need to transpose when sending to shader
	mat4 mp = gm_mat4_multiply(&m, &p);
	camera->projection_matrix = gm_mat4_scalar_product(-1.0f, &mp);
}