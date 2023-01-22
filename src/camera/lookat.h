#ifndef BASIC_ENGINE_CAMERA_LOOKAT_H
#define BASIC_ENGINE_CAMERA_LOOKAT_H

#include "../gm.h"
#include "../quaternion.h"

typedef struct {
	vec3 position;
	r32 near_plane;
	r32 far_plane;
	r32 fov;
	mat4 view_matrix;
	mat4 projection_matrix;
	Quaternion rotation;

	vec3 lookat_position;
	r32 lookat_distance;
	boolean consider_roll;
} Lookat_Camera;

vec3 lookat_camera_get_position(const Lookat_Camera* camera);
Quaternion lookat_camera_get_rotation(const Lookat_Camera* camera);
mat4 lookat_camera_get_view_matrix(const Lookat_Camera* camera);
mat4 lookat_camera_get_projection_matrix(const Lookat_Camera* camera);
void lookat_camera_set_position(Lookat_Camera* camera, vec3 position);
void lookat_camera_set_near_plane(Lookat_Camera* camera, r32 near_plane);
void lookat_camera_set_far_plane(Lookat_Camera* camera, r32 far_plane);
void lookat_camera_set_fov(Lookat_Camera* camera, r32 fov);
void lookat_camera_rotate(Lookat_Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
void lookat_camera_force_matrix_recalculation(Lookat_Camera* camera);

// Look At Camera specific functions
void lookat_camera_init(Lookat_Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov, int consider_roll);
r32 lookat_camera_get_lookat_distance(const Lookat_Camera* camera);
vec3 lookat_camera_get_lookat_position(const Lookat_Camera* camera);
void lookat_camera_set_lookat_position(Lookat_Camera* camera, vec3 lookat_position);
void lookat_camera_set_lookat_distance(Lookat_Camera* camera, r32 lookat_distance);

#endif