#ifndef BASIC_ENGINE_CAMERA_COMMON_H
#define BASIC_ENGINE_CAMERA_COMMON_H

#include "../gm.h"
#include "../quaternion.h"

vec3 camera_common_get_view(const mat4* view_matrix);
vec3 camera_common_get_x_axis(const Quaternion* camera_rotation);
vec3 camera_common_get_y_axis(const Quaternion* camera_rotation);
vec3 camera_common_get_z_axis(const Quaternion* camera_rotation);
Quaternion camera_common_rotate(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff);
Quaternion camera_common_rotate_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
Quaternion camera_common_rotate_x(const Quaternion* camera_rotation, r32 x_diff);
Quaternion camera_common_rotate_y(const Quaternion* camera_rotation, r32 y_diff);
Quaternion camera_common_rotate_x_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 mouse_y);
Quaternion camera_common_rotate_y_considering_click_coords(const Quaternion* camera_rotation, r32 y_diff, r32 mouse_x);
Quaternion camera_common_rotate_axis(const Quaternion* camera_rotation, r32 angle, vec3 axis);
mat4 camera_common_calculate_view_matrix(vec3 camera_position, const Quaternion* camera_rotation);
mat4 camera_common_calculate_projection_matrix(r32 near, r32 far, r32 fov);

#endif