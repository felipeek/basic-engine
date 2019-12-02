#ifndef BASIC_ENGINE_COLLADA_H
#define BASIC_ENGINE_COLLADA_H

#include "common.hpp"
#include "graphics.hpp"
#include "animation.hpp"

int colladaLoad(const s8* path, AnimatedVertex*** verticesMatrix, unsigned int*** indicesMatrix, Animation* animation, Joint* rootJoint);

#endif