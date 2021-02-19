#include "hierarchical_model.h"
#include <dynamic_array.h>

void hierarchical_model_joint_create(Hierarchical_Model_Joint* joint, vec4 translation, Quaternion rotation, vec3 scale,
	vec4 color, Mesh mesh, Hierarchical_Model_Joint* children)
{
	Entity e;
	joint->children = children;
	joint->translation = translation;
	joint->rotation = rotation;
	joint->scale = scale;
	graphics_entity_create_with_color(&e, mesh, (vec4){0.0f, 0.0f, 0.0f, 1.0f},
		quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){1.0f, 1.0f, 1.0f}, color);
	joint->e = e;
}

void hierarchical_model_joint_destroy_recursively(Hierarchical_Model_Joint* joint)
{
	for (u32 i = 0; i < array_get_length(joint->children); ++i)
	{
		hierarchical_model_joint_destroy_recursively(&joint->children[i]);
	}
	graphics_entity_destroy(&joint->e);
	array_release(joint->children);
}

void hierarchical_model_create(Hierarchical_Model* hierarchical_model, Hierarchical_Model_Joint root)
{
	hierarchical_model->root = root;
}

static mat4 get_model_matrix(vec4 translation, Quaternion rotation, vec3 scale)
{
	r32 s, c;

	mat4 scale_matrix = (mat4) {
		scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(&rotation);

	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, translation.x,
			0.0f, 1.0f, 0.0f, translation.y,
			0.0f, 0.0f, 1.0f, translation.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	return gm_mat4_multiply(&translation_matrix, &model_matrix);
}

static void render_joint(Hierarchical_Model_Joint* joint, mat4 parent_transform, const Perspective_Camera* camera, const Light* lights)
{
	mat4 joint_transform = get_model_matrix(joint->translation, joint->rotation, joint->scale);
	joint_transform = gm_mat4_multiply(&parent_transform, &joint_transform);
	graphics_entity_set_model_matrix(&joint->e, joint_transform);
	graphics_entity_render_phong_shader(camera, &joint->e, lights);

	// Render children
	for (u32 i = 0; i < array_get_length(joint->children); ++i)
		render_joint(&joint->children[i], joint_transform, camera, lights);
}

void hierarchical_model_render(Hierarchical_Model* hierarchical_model, const Perspective_Camera* camera, const Light* lights)
{
	Hierarchical_Model_Joint* root = &hierarchical_model->root;
	mat4 root_transform = get_model_matrix(root->translation, root->rotation, root->scale);
	graphics_entity_set_model_matrix(&root->e, root_transform);
	graphics_entity_render_phong_shader(camera, &root->e, lights);

	// Render children
	for (u32 i = 0; i < array_get_length(root->children); ++i)
		render_joint(&root->children[i], root_transform, camera, lights);
}