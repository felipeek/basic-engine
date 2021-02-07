#include "camera.h"
#include <math.h>

extern s32 window_width;
extern s32 window_height;

static void truncate_angles(Perspective_Camera* camera)
{
	const float yaw_truncate_threshold = 0.1f;
	const float yaw_bottom_limit = PI_F - yaw_truncate_threshold;
	const float yaw_top_limit = yaw_truncate_threshold;

	if (camera->yaw > yaw_bottom_limit)
		camera->yaw = yaw_bottom_limit;
	else if (camera->yaw < yaw_top_limit)
		camera->yaw = yaw_top_limit;
}

static void recalculate_angles(Perspective_Camera* camera)
{
	camera->pitch = atan2f(camera->view.x, camera->view.z);
	camera->yaw = atan2f(sqrtf(camera->view.x * camera->view.x + camera->view.z * camera->view.z), camera->view.y);
	truncate_angles(camera);
}

static void recalculate_view_matrix(Perspective_Camera* camera)
{
	vec3 up_vec3 = (vec3) { camera->up.x, camera->up.y, camera->up.z };
	vec4 w = gm_vec4_scalar_product(-1, gm_vec4_normalize(camera->view));
	vec3 w_vec3 = (vec3) { w.x, w.y, w.z };
	vec3 up_w_cross = gm_vec3_cross(up_vec3, w_vec3);
	vec4 u = gm_vec4_normalize((vec4) { up_w_cross.x, up_w_cross.y, up_w_cross.z, 0.0f });
	vec3 u_vec3 = (vec3) { u.x, u.y, u.z };
	vec3 v_vec3 = gm_vec3_cross(w_vec3, u_vec3);
	vec4 v = (vec4) { v_vec3.x, v_vec3.y, v_vec3.z, 0.0f };
	// Useless, but conceptually correct.
	vec4 world_to_camera_vec = gm_vec4_subtract(camera->position, (vec4) { 0.0f, 0.0f, 0.0f, 1.0f });

	camera->x_axis = u;
	camera->y_axis = v;
	camera->z_axis = w;

	// Need to transpose when sending to shader
	camera->view_matrix = (mat4) {
		u.x, u.y, u.z, -gm_vec4_dot(u, world_to_camera_vec),
			v.x, v.y, v.z, -gm_vec4_dot(v, world_to_camera_vec),
			w.x, w.y, w.z, -gm_vec4_dot(w, world_to_camera_vec),
			0.0f, 0.0f, 0.0f, 1.0f
	};
}


static void recalculate_projection_matrix(Perspective_Camera* camera)
{
	r32 near = camera->near_plane;
	r32 far = camera->far_plane;
	r32 top = (r32)fabs(near) * atanf(gm_radians(camera->fov) / 2.0f);
	r32 bottom = -top;
	r32 right = top * ((r32)window_width / (r32)window_height);
	r32 left = -right;

	mat4 p = (mat4) {
		near, 0, 0, 0,
			0, near, 0, 0,
			0, 0, near + far, -near * far,
			0, 0, 1, 0
	};

	mat4 m = (mat4) {
		2.0f / (right - left), 0, 0, -(right + left) / (right - left),
			0, 2.0f / (top - bottom), 0, -(top + bottom) / (top - bottom),
			0, 0, 2.0f / (far - near), -(far + near) / (far - near),
			0, 0, 0, 1
	};

	// Need to transpose when sending to shader
	mat4 mp = gm_mat4_multiply(&m, &p);
	camera->projection_matrix = gm_mat4_scalar_product(-1, &mp);
}

static void recalculate_view(Perspective_Camera* camera)
{
	camera->view.x = sinf(camera->pitch) * sinf(camera->yaw);
	camera->view.y = cosf(camera->yaw);
	camera->view.z = cosf(camera->pitch) * sinf(camera->yaw);
	camera->view.w = 0.0f;
}

void camera_init(Perspective_Camera* camera, vec4 position, vec4 up, vec4 view, r32 near_plane, r32 far_plane, r32 fov)
{
	camera->position = position;
	camera->up = up;
	camera->view = view;
	camera->near_plane = near_plane;
	camera->far_plane = far_plane;
	camera->fov = fov;
	recalculate_angles(camera);
	recalculate_view_matrix(camera);
	recalculate_projection_matrix(camera);
}

void camera_set_position(Perspective_Camera* camera, vec4 position)
{
	camera->position = position;
	recalculate_view_matrix(camera);
}

void camera_set_up(Perspective_Camera* camera, vec4 up)
{
	camera->up = up;
	recalculate_view_matrix(camera);
}

void camera_set_view(Perspective_Camera* camera, vec4 view)
{
	camera->view = view;
	recalculate_angles(camera);
	recalculate_view_matrix(camera);
}

void camera_set_near_plane(Perspective_Camera* camera, r32 near_plane)
{
	camera->near_plane = near_plane;
	recalculate_projection_matrix(camera);
}

void camera_set_far_plane(Perspective_Camera* camera, r32 far_plane)
{
	camera->far_plane = far_plane;
	recalculate_projection_matrix(camera);
}

void camera_inc_pitch(Perspective_Camera* camera, r32 angle)
{
	camera->pitch += angle;
	truncate_angles(camera);
	recalculate_view(camera);
	recalculate_view_matrix(camera);
}

void camera_inc_yaw(Perspective_Camera* camera, r32 angle)
{
	camera->yaw += angle;
	truncate_angles(camera);
	recalculate_view(camera);
	recalculate_view_matrix(camera);
}

void camera_set_fov(Perspective_Camera* camera, r32 fov)
{
	camera->fov = fov;
	recalculate_projection_matrix(camera);
}

void camera_force_matrix_recalculation(Perspective_Camera* camera)
{
	recalculate_view_matrix(camera);
	recalculate_projection_matrix(camera);
}