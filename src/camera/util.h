#ifndef BASIC_ENGINE_CAMERA_UTIL_H
#define BASIC_ENGINE_CAMERA_UTIL_H

#include "../gm.h"
#include "../quaternion.h"

Quaternion camera_util_rotate(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff);
Quaternion camera_util_rotate_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
Quaternion camera_util_rotate_x(const Quaternion* camera_rotation, r32 x_diff);
Quaternion camera_util_rotate_y(const Quaternion* camera_rotation, r32 y_diff);
Quaternion camera_util_rotate_x_considering_click_coords(const Quaternion* camera_rotation, r32 x_diff, r32 mouse_y);
Quaternion camera_util_rotate_y_considering_click_coords(const Quaternion* camera_rotation, r32 y_diff, r32 mouse_x);
Quaternion camera_util_rotate_axis(const Quaternion* camera_rotation, r32 angle, vec3 axis);

#endif