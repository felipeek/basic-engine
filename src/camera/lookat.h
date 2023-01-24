#ifndef BASIC_ENGINE_CAMERA_LOOKAT_H
#define BASIC_ENGINE_CAMERA_LOOKAT_H

#include "camera.h"

void lookat_camera_init(Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov,
	int consider_roll, r32 movement_speed, r32 rotation_speed, r32 panning_speed);
Quaternion lookat_camera_get_rotation(const Camera* camera);
void lookat_camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
r32 lookat_camera_get_lookat_distance(const Camera* camera);
vec3 lookat_camera_get_lookat_position(const Camera* camera);
r32 lookat_camera_get_panning_speed(const Camera* camera);
void lookat_camera_set_lookat_position(Camera* camera, vec3 lookat_position);
void lookat_camera_set_lookat_distance(Camera* camera, r32 lookat_distance);
void lookat_camera_set_panning_speed(Camera* camera, r32 panning_speed);
void lookat_camera_pan(Camera* camera, r32 y_difference, r32 x_difference);
void lookat_camera_approximate(Camera* camera, r32 y_offset);

#endif