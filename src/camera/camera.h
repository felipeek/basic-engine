#ifndef BASIC_ENGINE_CAMERA_CAMERA_H
#define BASIC_ENGINE_CAMERA_CAMERA_H

#include "../gm.h"
#include "../quaternion.h"
#include "free.h"
#include "lookat.h"

typedef enum {
	CAMERA_FREE,
	CAMERA_LOOKAT
} Camera_Type;

typedef struct {
	Camera_Type type;
	union {
		Free_Camera free_camera;
		Lookat_Camera lookat_camera;
	};
} Camera;

void camera_init_free(Camera* camera, vec3 position, r32 near_plane, r32 far_plane, r32 fov, int lock_rotation);
void camera_init_lookat(Camera* camera, vec3 lookat_position, r32 lookat_distance, r32 near_plane, r32 far_plane, r32 fov,
	int consider_mouse_coords_when_rotating);
vec3 camera_get_view(const Camera* camera);
vec3 camera_get_x_axis(const Camera* camera);
vec3 camera_get_y_axis(const Camera* camera);
vec3 camera_get_z_axis(const Camera* camera);
void camera_move_forward(Camera* camera, r32 amount);
void camera_move_right(Camera* camera, r32 amount);
vec3 camera_get_position(const Camera* camera);
Quaternion camera_get_rotation(const Camera* camera);
mat4 camera_get_view_matrix(const Camera* camera);
mat4 camera_get_projection_matrix(const Camera* camera);
void camera_set_position(Camera* camera, vec3 position);
void camera_set_near_plane(Camera* camera, r32 near_plane);
void camera_set_far_plane(Camera* camera, r32 far_plane);
void camera_set_fov(Camera* camera, r32 fov);
void camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
void camera_force_matrix_recalculation(Camera* camera);

#endif