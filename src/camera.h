#ifndef BASIC_ENGINE_CAMERA_H
#define BASIC_ENGINE_CAMERA_H
#include "gm.h"

typedef struct
{
	vec4 position;
	vec4 up;
	vec4 view;
	r32 near_plane;
	r32 far_plane;
	r32 pitch;
	r32 yaw;
	r32 fov;
	mat4 view_matrix;
	mat4 projection_matrix;
	vec4 x_axis;
	vec4 y_axis;
	vec4 z_axis;
} Perspective_Camera;

extern void camera_init(Perspective_Camera* camera, vec4 position, vec4 up, vec4 view, r32 near_plane, r32 far_plane, r32 fov);
extern void camera_set_position(Perspective_Camera* camera, vec4 position);
extern void camera_set_up(Perspective_Camera* camera, vec4 up);
extern void camera_set_view(Perspective_Camera* camera, vec4 view);
extern void camera_set_near_plane(Perspective_Camera* camera, r32 near_plane);
extern void camera_set_far_plane(Perspective_Camera* camera, r32 far_plane);
extern void camera_inc_pitch(Perspective_Camera* camera, r32 angle);
extern void camera_inc_yaw(Perspective_Camera* camera, r32 angle);
extern void camera_set_fov(Perspective_Camera* camera, r32 fov);
extern void camera_force_matrix_recalculation(Perspective_Camera* camera);

#endif