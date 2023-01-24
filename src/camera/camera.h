#ifndef BASIC_ENGINE_CAMERA_CAMERA_H
#define BASIC_ENGINE_CAMERA_CAMERA_H

#include "../gm.h"
#include "../quaternion.h"

typedef enum {
	CAMERA_FREE,
	CAMERA_LOOKAT
} Camera_Type;

typedef struct {
	Quaternion rotation;
	Quaternion y_rotation;
	bool lock_rotation;
} Free_Camera;

typedef struct {
	Quaternion rotation;
	vec3 lookat_position;
	r32 lookat_distance;
	r32 panning_speed;
	bool consider_roll;
} Lookat_Camera;

typedef struct {
	Camera_Type type;
	vec3 position;
	r32 near_plane;
	r32 far_plane;
	r32 fov;
	r32 rotation_speed;
	r32 movement_speed;
	mat4 view_matrix;
	mat4 projection_matrix;
	union {
		Free_Camera free_camera;
		Lookat_Camera lookat_camera;
	};
} Camera;

vec3 camera_get_view(const Camera* camera);
vec3 camera_get_x_axis(const Camera* camera);
vec3 camera_get_y_axis(const Camera* camera);
vec3 camera_get_z_axis(const Camera* camera);
void camera_move_forward(Camera* camera, r32 amount);
void camera_move_right(Camera* camera, r32 amount);
vec3 camera_get_position(const Camera* camera);
Quaternion camera_get_rotation(const Camera* camera);
r32 camera_get_movement_speed(const Camera* camera);
r32 camera_get_rotation_speed(const Camera* camera);
mat4 camera_get_view_matrix(const Camera* camera);
mat4 camera_get_projection_matrix(const Camera* camera);
void camera_set_position(Camera* camera, vec3 position);
void camera_set_near_plane(Camera* camera, r32 near_plane);
void camera_set_far_plane(Camera* camera, r32 far_plane);
void camera_set_fov(Camera* camera, r32 fov);
void camera_rotate(Camera* camera, r32 x_diff, r32 y_diff, r32 mouse_x, r32 mouse_y);
void camera_set_movement_speed(Camera* camera, r32 movement_speed);
void camera_set_rotation_speed(Camera* camera, r32 rotation_speed);
void camera_force_matrix_recalculation(Camera* camera);
void camera_recalculate_view_matrix(Camera* camera);
void camera_recalculate_projection_matrix(Camera* camera);

#endif