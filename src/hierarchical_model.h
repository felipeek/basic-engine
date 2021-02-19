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
	vec3 rotation_axis;

	struct Hierarchical_Model_Joint* parent;
	struct Hierarchical_Model_Joint* children;
} Hierarchical_Model_Joint;

void hierarchical_model_joint_create(Hierarchical_Model_Joint* joint, vec4 translation, Quaternion rotation, vec3 scale,
	vec4 color, vec3 rotation_axis, Mesh mesh, Hierarchical_Model_Joint* children, Hierarchical_Model_Joint* parent);
void hierarchical_model_joint_destroy_recursively(Hierarchical_Model_Joint* joint);
void hierarchical_model_render(Hierarchical_Model_Joint* root, const Perspective_Camera* camera, const Light* lights);
void hierarchical_model_update(Hierarchical_Model_Joint* root);

#endif