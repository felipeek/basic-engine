#include "util.h"

extern s32 window_width;
extern s32 window_height;

Quaternion camera_util_rotate(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff)
{
	Quaternion intermediary = camera_util_rotate_x(camera_rotation, x_diff);
	return camera_util_rotate_y(&intermediary, y_diff);
}

Quaternion camera_util_rotate_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y)
{
	Quaternion intermediary = camera_util_rotate_x_considering_click_coords(camera_rotation, x_diff, -mouse_y);
	return camera_util_rotate_y_considering_click_coords(&intermediary, y_diff, -mouse_x);
}

Quaternion camera_util_rotate_x(const Quaternion* camera_rotation, r32 x_diff)
{
	return camera_util_rotate_axis(camera_rotation, x_diff, (vec3){0.0, 1.0, 0.0});
}

Quaternion camera_util_rotate_y(const Quaternion* camera_rotation, r32 y_diff)
{
	return camera_util_rotate_axis(camera_rotation, y_diff, (vec3){1.0, 0.0, 0.0});
}

Quaternion camera_util_rotate_x_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 mouse_y)
{
	// If the user clicked on the top (or bottom) of the screen, we also consider the Z axis.
	vec3 axis_z_component = gm_vec3_scalar_product(mouse_y, (vec3){0.0, 0.0, 1.0});
	vec3 axis_y_component = gm_vec3_scalar_product(1 - fabsf(mouse_y), (vec3){0.0, 1.0, 0.0});
	vec3 axis = gm_vec3_add(axis_z_component, axis_y_component);
	return camera_util_rotate_axis(camera_rotation, x_diff, axis);
}

Quaternion camera_util_rotate_y_considering_click_coords(const Quaternion* camera_rotation, r32 y_diff, r32 mouse_x)
{
	// If the user clicked on the right (or left) of the screen, we also consider the Z axis.
	vec3 axis_z_component = gm_vec3_scalar_product(mouse_x, (vec3){0.0, 0.0, 1.0});
	vec3 axis_x_component = gm_vec3_scalar_product(1 - fabsf(mouse_x), (vec3){1.0, 0.0, 0.0});
	vec3 axis = gm_vec3_add(axis_z_component, axis_x_component);
	return camera_util_rotate_axis(camera_rotation, y_diff, axis);
}

Quaternion camera_util_rotate_axis(const Quaternion* camera_rotation, r32 angle, vec3 axis)
{
	Quaternion q = quaternion_new(axis, angle);
	Quaternion new_rotation = quaternion_product(&q, camera_rotation);
	new_rotation = quaternion_normalize(&new_rotation);
	return new_rotation;
}