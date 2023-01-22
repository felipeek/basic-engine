#include "common.h"

extern s32 window_width;
extern s32 window_height;

vec3 camera_common_get_view(const mat4* view_matrix)
{
	vec3 camera_view = (vec3) { -view_matrix->data[2][0], -view_matrix->data[2][1], -view_matrix->data[2][2] };
	return gm_vec3_normalize(camera_view);
}

vec3 camera_common_get_x_axis(const Quaternion* camera_rotation)
{
	vec3 right = quaternion_get_right_inverted(camera_rotation);
	right = gm_vec3_normalize(right);
	return (vec3) { right.x, right.y, right.z };
}

vec3 camera_common_get_y_axis(const Quaternion* camera_rotation)
{
	vec3 up = quaternion_get_up_inverted(camera_rotation);
	up = gm_vec3_normalize(up);
	return (vec3) { up.x, up.y, up.z };
}

vec3 camera_common_get_z_axis(const Quaternion* camera_rotation)
{
	vec3 forward = quaternion_get_forward_inverted(camera_rotation);
	forward = gm_vec3_normalize(forward);
	return (vec3) { forward.x, forward.y, forward.z };
}

Quaternion camera_common_rotate(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff)
{
	Quaternion intermediary = camera_common_rotate_x(camera_rotation, x_diff);
	return camera_common_rotate_y(&intermediary, y_diff);
}

Quaternion camera_common_rotate_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	Quaternion intermediary = camera_common_rotate_x_considering_click_coords(camera_rotation, x_diff, mouse_y);
	return camera_common_rotate_y_considering_click_coords(&intermediary, y_diff, -mouse_x);
}

Quaternion camera_common_rotate_x(const Quaternion* camera_rotation, r32 x_diff)
{
	return camera_common_rotate_axis(camera_rotation, x_diff, (vec3){0.0, 1.0, 0.0});
}

Quaternion camera_common_rotate_y(const Quaternion* camera_rotation, r32 y_diff)
{
	return camera_common_rotate_axis(camera_rotation, y_diff, (vec3){1.0, 0.0, 0.0});
}

Quaternion camera_common_rotate_x_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 mouse_y)
{
	// If the user clicked on the top (or bottom) of the screen, we also consider the Z axis.
	vec3 axis_z_component = gm_vec3_scalar_product(mouse_y, (vec3){0.0, 0.0, 1.0});
	vec3 axis_y_component = gm_vec3_scalar_product(1 - fabsf(mouse_y), (vec3){0.0, 1.0, 0.0});
	vec3 axis = gm_vec3_add(axis_z_component, axis_y_component);
	return camera_common_rotate_axis(camera_rotation, x_diff, axis);
}

Quaternion camera_common_rotate_y_considering_click_coords(const Quaternion* camera_rotation, r32 y_diff, r32 mouse_x)
{
	// If the user clicked on the right (or left) of the screen, we also consider the Z axis.
	vec3 axis_z_component = gm_vec3_scalar_product(mouse_x, (vec3){0.0, 0.0, 1.0});
	vec3 axis_x_component = gm_vec3_scalar_product(1 - fabsf(mouse_x), (vec3){1.0, 0.0, 0.0});
	vec3 axis = gm_vec3_add(axis_z_component, axis_x_component);
	return camera_common_rotate_axis(camera_rotation, y_diff, axis);
}

Quaternion camera_common_rotate_axis(const Quaternion* camera_rotation, r32 angle, vec3 axis)
{
	Quaternion q = quaternion_new(axis, angle);
	Quaternion new_rotation = quaternion_product(&q, camera_rotation);
	new_rotation = quaternion_normalize(&new_rotation);
	return new_rotation;
}

mat4 camera_common_calculate_view_matrix(vec3 camera_position, const Quaternion* camera_rotation)
{
	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, -camera_position.x,
		0.0f, 1.0f, 0.0f, -camera_position.y,
		0.0f, 0.0f, 1.0f, -camera_position.z,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(camera_rotation);
	return gm_mat4_multiply(&rotation_matrix, &translation_matrix);
}

mat4 camera_common_calculate_projection_matrix(r32 near, r32 far, r32 fov)
{
	r32 top = (r32)fabs(near) * atanf(gm_radians(fov) / 2.0f);
	r32 bottom = -top;
	r32 right = top * ((r32)window_width / (r32)window_height);
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
	return gm_mat4_scalar_product(-1.0f, &mp);
}