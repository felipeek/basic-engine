#ifndef BASIC_ENGINE_CAMERA_FREE_H
#define BASIC_ENGINE_CAMERA_FREE_H
#include "camera.h"

void free_camera_init(Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation);
Quaternion free_camera_get_rotation(const Camera* camera);
vec3 free_camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);

#endif