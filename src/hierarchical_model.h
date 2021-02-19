#ifndef BASIC_ENGINE_HIERARCHICAL_MODEL_H
#define BASIC_ENGINE_HIERARCHICAL_MODEL_H
#include "common.h"
#include "gm.h"
#include "graphics.h"
#include "quaternion.h"

typedef struct Hierarchical_Model_Joint {
	Entity e;
	vec4 translation;
	Quaternion rotation;
	vec3 scale;

	struct Hierarchical_Model_Joint* children;
} Hierarchical_Model_Joint;

typedef struct {
	Hierarchical_Model_Joint root;
} Hierarchical_Model;

void hierarchical_model_create(Hierarchical_Model* hierarchical_model, Hierarchical_Model_Joint root);
void hierarchical_model_joint_create(Hierarchical_Model_Joint* joint, vec4 translation, Quaternion rotation, vec3 scale, 
	Hierarchical_Model_Joint* children);
void hierarchical_model_create(Hierarchical_Model* hierarchical_model, Hierarchical_Model_Joint root);
void hierarchical_model_render(Hierarchical_Model* hierarchical_model, const Perspective_Camera* camera, const Light* lights);

#endif