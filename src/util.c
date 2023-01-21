#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include "camera.h"

r32 util_random_float(r32 min, r32 max)
{
	r32 scale = rand() / (r32)RAND_MAX;
	return min + scale * (max - min);
}

s8* util_read_file(const s8* path, s32* _file_length)
{
	FILE* file;
	s8* buffer;
	s32 file_length;

	file = fopen(path, "rb");
	fseek(file, 0, SEEK_END);
	file_length = ftell(file);
	rewind(file);

	buffer = (s8*)malloc((file_length + 1) * sizeof(s8));
	fread(buffer, file_length, 1, file);
	fclose(file);

	buffer[file_length] = '\0';

	if (_file_length)
		*_file_length = file_length;

	return buffer;
}

void util_free_file(s8* file)
{
	free(file);
}

// Convert window coords to NDC (from [0, WinWidth],[0, WinHeight] to [-1, 1])
void util_normalize_window_coords_to_ndc(r32 x, r32 y, s32 window_width, s32 window_height, r32* x_ndc, r32* y_ndc)
{
	y = window_height - y;
	x = (2.0f * x) / window_width - 1.0f;
	y = (2.0f * y) / window_height - 1.0f;

	*x_ndc = x;
	*y_ndc = y;
}

// Get mouse's click position and direction vector in World Coordinates
// Inputs:
// camera - camera to consider
// mouseX, mouseY - mouse coordinates in range [0, WindowWidth] and [0, WindowHeight]
// Outputs:
// Position: Click position in world coords
// Direction: Direction vector in world coords
void util_mouse_get_ray_world_coords(const Perspective_Camera* camera, r32 mouse_x, r32 mouse_y, s32 window_width, s32 window_height,
		vec3* _position, vec3* _direction)
{
	r32 x, y;
	util_normalize_window_coords_to_ndc(mouse_x, mouse_y, window_width, window_height, &x, &y);

	mat4 proj_matrix_inv, view_matrix_inv;
	assert(gm_mat4_inverse(&camera->projection_matrix, &proj_matrix_inv));
	assert(gm_mat4_inverse(&camera->view_matrix, &view_matrix_inv));

	// Get the exact point that the user clicked on. This point is in NDC coordinates (i.e. "projection coordinates").
	// We are picking the point that is in the closest plane to the screen (i.e., the plane z = -1.0)
	// Note that this is a point, not a vector.
	vec4 ray_clip_ndc_coords = (vec4) { x, y, -1, 1 };

	// Transform the point back to view coordinates.
	vec4 ray_clip_view_coords = gm_mat4_multiply_vec4(&proj_matrix_inv, ray_clip_ndc_coords);
	ray_clip_view_coords = gm_vec4_scalar_product(1.0f / ray_clip_view_coords.w, ray_clip_view_coords);

	// Get vector from camera origin to point, in view coordinates.
	// Note that we are in view coordinates, so the origin is always <0,0,0,1>.
	// Therefore, performing the subtraction "ray - origin" is the same as making the w coord 0.
	vec4 ray_eye_view_coords = (vec4) { ray_clip_view_coords.x, ray_clip_view_coords.y, ray_clip_view_coords.z, 0.0f };

	// Transform ray vector from view coords to world coords.
	vec4 ray_eye_world_coords = gm_vec4_normalize(gm_mat4_multiply_vec4(&view_matrix_inv, ray_eye_view_coords));

	if (_position)
		*_position = camera->position;
	if (_direction)
		*_direction = (vec3) {ray_eye_world_coords.x, ray_eye_world_coords.y, ray_eye_world_coords.z};
}

void util_viewport_based_on_window_size(s32 x, s32 y, s32 width, s32 height)
{
#if defined(__APPLE__)
	// When apple retina displays are used, the window size reported by GLFW is half the size of the framebuffer
	glViewport(x, y, 2 * width, 2 * height);
#else
	glViewport(x, y, width, height);
#endif
}

void util_viewport_based_on_framebuffer_size(s32 x, s32 y, s32 width, s32 height)
{
	glViewport(x, y, width, height);
}