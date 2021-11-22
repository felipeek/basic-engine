#ifndef BASIC_ENGINE_OBJ_H
#define BASIC_ENGINE_OBJ_H

#include "common.h"
#include "graphics.h"

int obj_parse(const char* obj_path, Vertex** vertices, u32** indexes);
int obj_randomizer(const char* obj_path, Vertex** vertices, u32** indexes);

#endif