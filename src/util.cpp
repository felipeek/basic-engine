#include "util.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

extern dvec2 framebuffer_size;

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

void util_viewport_for_complete_window()
{
	glViewport(0, 0, framebuffer_size.x, framebuffer_size.y);
}