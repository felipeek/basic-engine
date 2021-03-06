#ifndef BASIC_ENGINE_UTIL_H
#define BASIC_ENGINE_UTIL_H
#include "common.h"

s8* util_read_file(const s8* path, s32* file_length);
void util_free_file(s8* file);
r32 util_random_float(r32 min, r32 max);

#endif