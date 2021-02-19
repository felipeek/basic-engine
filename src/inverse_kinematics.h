#ifndef BASIC_ENGINE_INVERSE_KINEMATICS_H
#define BASIC_ENGINE_INVERSE_KINEMATICS_H
#include "hierarchical_model.h"
void rotate_joints_towards_target_point(Hierarchical_Model_Joint* leaf_joint, vec3 target_point);
#endif