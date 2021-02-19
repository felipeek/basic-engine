#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"
#include "hierarchical_model.h"
#include "inverse_kinematics.h"

#define NUMBER_OF_IK_ITERATIONS_PER_FRAME 5000

static Perspective_Camera camera;
static Light* lights;
static Mesh cube_mesh;
static Render_Primitives_Context render_primitives_ctx;
static Entity sphere;

// Hierarchical model
static boolean is_hierarchical_model_created, is_animating;
static Hierarchical_Model_Joint hierarchical_model;
static IK_Method ik_method;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position = (vec4) {0.0f, 0.0f, 5.0f, 1.0f};
	r32 camera_near_plane = -0.01f;
	r32 camera_far_plane = -1000.0f;
	r32 camera_fov = 45.0f;
	camera_init(&camera, camera_position, camera_near_plane, camera_far_plane, camera_fov);
	return camera;
}

static Light* create_lights()
{
	Light light;
	Light* lights = array_create(Light, 1);

	vec4 light_position = (vec4) {0.0f, 0.0f, 15.0f, 1.0f};
	vec4 ambient_color = (vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	vec4 diffuse_color = (vec4) {0.8, 0.8, 0.8, 1.0f};
	vec4 specular_color = (vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphics_light_create(&light, light_position, ambient_color, diffuse_color, specular_color);
	array_push(lights, &light);

	return lights;
}

static vec3 get_rotation_axis(Rotation_Axis rotation_axis)
{
	switch (rotation_axis)
	{
		default:
		case ROTATION_AXIS_X: return (vec3){1.0f, 0.0f, 0.0f};
		case ROTATION_AXIS_Y: return (vec3){0.0f, 1.0f, 0.0f};
		case ROTATION_AXIS_Z: return (vec3){0.0f, 0.0f, 1.0f};
	}
}

static Hierarchical_Model_Joint create_joint_from_definition(const Joint_Definition* joint_definition)
{
	Hierarchical_Model_Joint joint;
	vec4 translation = (vec4){joint_definition->translation.x, joint_definition->translation.y, joint_definition->translation.z, 1.0f};
	Quaternion rotation_x = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, joint_definition->rotation.x);
	Quaternion rotation_y = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, joint_definition->rotation.y);
	Quaternion rotation_z = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, joint_definition->rotation.z);
	Quaternion rotation = quaternion_product(&rotation_x, &rotation_y);
	rotation = quaternion_product(&rotation, &rotation_z);
	vec3 scale = joint_definition->scale;
	vec4 color = (vec4){joint_definition->color.x, joint_definition->color.y, joint_definition->color.z, 1.0f};
	vec3 rotation_axis = get_rotation_axis(joint_definition->rotation_axis);
	Hierarchical_Model_Joint* children = array_create(Hierarchical_Model_Joint, 1);
	for (u32 i = 0; i < array_get_length(joint_definition->children); ++i)
	{
		Hierarchical_Model_Joint child = create_joint_from_definition(&joint_definition->children[i]);
		array_push(children, &child);
	}
	hierarchical_model_joint_create(&joint, translation, rotation, scale, color, rotation_axis, cube_mesh, children, NULL);
	return joint;
}

static void fill_joint_parents(Hierarchical_Model_Joint* joint, const Hierarchical_Model_Joint* parent)
{
	for (u32 i = 0; i < array_get_length(joint->children); ++i)
		fill_joint_parents(&joint->children[i], joint);
	joint->parent = parent;
}

static void hierarchical_model_set_callback(const Joint_Definition* joint_definition)
{
	if (is_hierarchical_model_created)
		hierarchical_model_joint_destroy_recursively(&hierarchical_model);
	hierarchical_model = create_joint_from_definition(joint_definition);
	fill_joint_parents(&hierarchical_model, NULL);	// second-pass just to fill the parents... can't fill at creation time because of chicken-egg problem
	is_hierarchical_model_created = true;
}

static void animate_callback(IK_Method ik_method)
{
	if (!is_hierarchical_model_created)
		return;
	is_animating = true;
	ik_method = ik_method;
}

static void stop_callback()
{
	is_animating = false;
}

int core_init()
{
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	graphics_renderer_primitives_init(&render_primitives_ctx);

	Mesh sphere_mesh = graphics_mesh_create_from_obj("./res/sphere.obj", 0);
	graphics_entity_create_with_color(&sphere, sphere_mesh, (vec4){1.0f, 1.0f, 0.0f, 1.0f}, quaternion_new((vec3){1.0f, 0.0f, 0.0f}, 0.0f),
		(vec3){0.1f, 0.1f, 0.1f}, (vec4){1.0f, 0.0f, 1.0f, 1.0f});

	cube_mesh = graphics_mesh_create_from_obj("./res/moved_cube.obj", 0);
	menu_register_hierarchical_model_set_callback(hierarchical_model_set_callback);
	menu_register_animate_callback(animate_callback);
	menu_register_stop_callback(stop_callback);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	if (is_hierarchical_model_created)
	{
		hierarchical_model_update(&hierarchical_model);
		if (is_animating)
		{
			Hierarchical_Model_Joint* leaf_joint = &hierarchical_model.children[0].children[0];
			for (u32 i = 0; i < NUMBER_OF_IK_ITERATIONS_PER_FRAME; ++i)
				rotate_joints_towards_target_point(leaf_joint, gm_vec4_to_vec3(sphere.world_position), ik_method);
		}
	}
}

void core_render()
{
	if (is_hierarchical_model_created)
		hierarchical_model_render(&hierarchical_model, &camera, lights);
	graphics_entity_render_phong_shader(&camera, &sphere, lights);
	//graphics_renderer_debug_points(&render_primitives_ctx, &target_position, 1, (vec4){1.0f, 0.0f, 1.0f, 1.0f});
	//graphics_renderer_primitives_flush(&render_primitives_ctx, &camera);


	// render end-effector
	if (is_hierarchical_model_created)
	{
		mat4 last_joint_transform = hierarchical_model.children[0].children[0].e.model_matrix;
		vec4 end_effector_position_in_local_coordinates = (vec4){2.0f, 0.0f, 0.0f, 1.0f};
		vec4 E = gm_mat4_multiply_vec4(&last_joint_transform, end_effector_position_in_local_coordinates);
		vec3 E3 = gm_vec4_to_vec3(E);
		graphics_renderer_debug_points(&render_primitives_ctx, &E3, 1, (vec4){1.0f, 0.0f, 1.0f, 1.0f});
		graphics_renderer_primitives_flush(&render_primitives_ctx, &camera);
	}
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 translation_speed = 0.01f;

	if (key_state[GLFW_KEY_LEFT_SHIFT])
		movement_speed = 0.5f;
	if (key_state[GLFW_KEY_RIGHT_SHIFT])
		movement_speed = 0.1f;

	if (key_state[GLFW_KEY_W])
		camera_move_forward(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_S])
		camera_move_forward(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_A])
		camera_move_right(&camera, -movement_speed * delta_time);
	if (key_state[GLFW_KEY_D])
		camera_move_right(&camera, movement_speed * delta_time);
	if (key_state[GLFW_KEY_X])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec4 delta = (vec4){-translation_speed, 0.0f, 0.0f, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
		else
		{
			vec4 delta = (vec4){translation_speed, 0.0f, 0.0f, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec4 delta = (vec4){0.0f, -translation_speed, 0.0f, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
		else
		{
			vec4 delta = (vec4){0.0f, translation_speed, 0.0f, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			vec4 delta = (vec4){0.0f, 0.0f, -translation_speed, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
		else
		{
			vec4 delta = (vec4){0.0f, 0.0f, translation_speed, 0.0f};
			graphics_entity_set_position(&sphere, gm_vec4_add(sphere.world_position, delta));
		}
	}
	if (key_state[GLFW_KEY_L])
	{
		static boolean wireframe = false;

		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		wireframe = !wireframe;
		key_state[GLFW_KEY_L] = false;
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
	static r64 x_pos_old, y_pos_old;
	// This constant is basically the mouse sensibility.
	// @TODO: Allow mouse sensibility to be configurable.
	static const r32 camera_mouse_speed = 0.1f;

	if (!reset)
	{
		r64 x_difference = x_pos - x_pos_old;
		r64 y_difference = y_pos - y_pos_old;

		camera_rotate_x(&camera, camera_mouse_speed * (r32)x_difference);
		camera_rotate_y(&camera, camera_mouse_speed * (r32)y_difference);
	}

	x_pos_old = x_pos;
	y_pos_old = y_pos;
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos)
{

}

void core_scroll_change_process(r64 x_offset, r64 y_offset)
{

}

void core_window_resize_process(s32 width, s32 height)
{
	camera_force_matrix_recalculation(&camera);
}