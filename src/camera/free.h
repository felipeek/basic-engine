#ifndef BASIC_ENGINE_CAMERA_FREE_H
#define BASIC_ENGINE_CAMERA_FREE_H
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
	Quaternion y_rotation;

	boolean lock_rotation;
} Free_Camera;

vec3 free_camera_get_position(const Free_Camera* camera);
Quaternion free_camera_get_rotation(const Free_Camera* camera);
mat4 free_camera_get_view_matrix(const Free_Camera* camera);
mat4 free_camera_get_projection_matrix(const Free_Camera* camera);
void free_camera_set_position(Free_Camera* camera, vec3 position);
void free_camera_set_near_plane(Free_Camera* camera, r32 near_plane);
void free_camera_set_far_plane(Free_Camera* camera, r32 far_plane);
void free_camera_set_fov(Free_Camera* camera, r32 fov);
vec3 free_camera_rotate(Free_Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
void free_camera_force_matrix_recalculation(Free_Camera* camera);

// Free Camera specific functions
void free_camera_init(Free_Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation);

#endif