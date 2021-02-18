#include "hierarchical_model.h"
#include <dynamic_array.h>

void hierarchical_model_create(Hierarchical_Model* hierarchical_model)
{
	Entity e;
	Mesh m = graphics_mesh_create_from_obj_with_color("./res/cube.obj", 0, (vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f},
		quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){1.0f, 1.0f, 1.0f});
	hierarchical_model->root.children = array_create(Hierarchical_Model, 1);
	hierarchical_model->root.translation = (vec4){0.0f, 0.0f, 0.0f, 1.0f};
	hierarchical_model->root.rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	hierarchical_model->root.scale = (vec3){1.0f, 1.0f, 1.0f};
	hierarchical_model->root.e = e;

	Hierarchical_Model_Joint joint;
	joint.children = array_create(Hierarchical_Model, 1);
	joint.translation = (vec4){1.0f, 0.0f, 0.0f, 1.0f};
	joint.rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f);
	joint.scale = (vec3){1.0f, 1.0f, 1.0f};
	joint.e = e;
}

static mat4 generate_model_matrix_for_joint(Hierarchical_Model_Joint* joint)
{
	r32 s, c;

	mat4 scale_matrix = (mat4) {
		joint->scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, joint->scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, joint->scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(&joint->rotation);

	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, joint->translation.x,
			0.0f, 1.0f, 0.0f, joint->translation.y,
			0.0f, 0.0f, 1.0f, joint->translation.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	model_matrix = gm_mat4_multiply(&translation_matrix, &model_matrix);
	return model_matrix;
}

static void render_joint(const Hierarchical_Model_Joint* joint, const Hierarchical_Model_Joint* parent)
{
	mat4 parent_transform = parent->e.model_matrix;
	vec4 joint_position = gm_mat4_multiply_vec4(&parent_transform, joint->translation);
	joint_position = gm_vec4_scalar_product(1.0f / joint_position.w, joint_position);
	joint_position.w = 1.0f; // ensure
	graphics_entity_set_position(&joint->e, joint_position);
	graphics_entity_set_rotation(&joint->e, joint->rotation);
	graphics_entity_set_scale(&joint->e, joint->scale);
	graphics_entity_render_phong_shader(&)
}

void hierarchical_model_render(Hierarchical_Model* hierarchical_model)
{
}