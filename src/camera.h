#ifndef BASIC_ENGINE_CAMERA_H
#define BASIC_ENGINE_CAMERA_H
#include "gm.h"
#include "quaternion.h"

typedef struct
{
	vec4 position;
	r32 near_plane;
	r32 far_plane;
	r32 fov;
	mat4 view_matrix;
	mat4 projection_matrix;
	Quaternion rotation;
	Quaternion yrotation;
} Perspective_Camera;

void camera_init(Perspective_Camera* camera, vec4 position, r32 near_plane, r32 far_plane, r32 fov);
void camera_set_position(Perspective_Camera* camera, vec4 position);
void camera_set_near_plane(Perspective_Camera* camera, r32 near_plane);
void camera_set_far_plane(Perspective_Camera* camera, r32 far_plane);
void camera_set_fov(Perspective_Camera* camera, r32 fov);
void camera_rotate_x(Perspective_Camera* camera, r32 x_difference);
void camera_rotate_y(Perspective_Camera* camera, r32 y_difference);
vec3 camera_get_x_axis(const Perspective_Camera* camera);
vec3 camera_get_y_axis(const Perspective_Camera* camera);
vec3 camera_get_z_axis(const Perspective_Camera* camera);
void camera_move_forward(Perspective_Camera* camera, r32 amount);
void camera_move_right(Perspective_Camera* camera, r32 amount);
void camera_force_matrix_recalculation(Perspective_Camera* camera);

#endif