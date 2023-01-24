#ifndef BASIC_ENGINE_UTIL_H
#define BASIC_ENGINE_UTIL_H
#include "common.h"
#include "camera/camera.h"

s8* util_read_file(const s8* path, s32* file_length);
void util_free_file(s8* file);
r32 util_random_float(r32 min, r32 max);
void util_normalize_window_coords_to_ndc(r32 x, r32 y, s32 window_width, s32 window_height, r32* x_ndc, r32* y_ndc);
void util_mouse_get_ray_world_coords(const Camera* camera, r32 mouse_x, r32 mouse_y, s32 window_width, s32 window_height,
		vec3* _position, vec3* _direction);
void util_viewport_for_complete_window();

#endif