#include "hierarchical_model.h"
#include <dynamic_array.h>

void hierarchical_model_joint_create(Hierarchical_Model_Joint* joint, vec4 translation, Quaternion rotation, vec3 scale,
	vec4 color, vec3 rotation_axis, Mesh mesh, Hierarchical_Model_Joint* children, Hierarchical_Model_Joint* parent)
{
	Entity e;
	joint->children = children;
	joint->parent = parent;
	joint->translation = translation;
	joint->rotation = rotation;
	joint->scale = scale;
	joint->rotation_axis = rotation_axis;
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

void hierarchical_model_render(Hierarchical_Model_Joint* root, const Perspective_Camera* camera, const Light* lights) 
 {
	graphics_entity_render_phong_shader(camera, &root->e, lights);

	// Render children
	for (u32 i = 0; i < array_get_length(root->children); ++i)
		hierarchical_model_render(&root->children[i], camera, lights);
}

static void update_joint(Hierarchical_Model_Joint* joint, mat4 parent_transform_without_scale)
{
	mat4 joint_transform_without_scale = graphics_generate_model_matrix(joint->translation, joint->rotation, (vec3){1.0f, 1.0f, 1.0f});
	joint_transform_without_scale = gm_mat4_multiply(&parent_transform_without_scale, &joint_transform_without_scale);
	mat4 joint_transform = graphics_generate_model_matrix(joint->translation, joint->rotation, joint->scale);
	joint_transform = gm_mat4_multiply(&parent_transform_without_scale, &joint_transform);
	graphics_entity_set_model_matrix(&joint->e, joint_transform);

	// Update children
	for (u32 i = 0; i < array_get_length(joint->children); ++i)
		update_joint(&joint->children[i], joint_transform_without_scale);
}

void hierarchical_model_update(Hierarchical_Model_Joint* root) 
 {
	mat4 root_transform_without_scale = graphics_generate_model_matrix(root->translation, root->rotation, (vec3){1.0f, 1.0f, 1.0f});
	mat4 root_transform = graphics_generate_model_matrix(root->translation, root->rotation, root->scale);
	graphics_entity_set_model_matrix(&root->e, root_transform);

	// Render children
	for (u32 i = 0; i < array_get_length(root->children); ++i)
		update_joint(&root->children[i], root_transform_without_scale);
}