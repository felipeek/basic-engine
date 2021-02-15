#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"

#define PBR_VERTEX_SHADER_PATH "./shaders/pbr_shader.vs"
#define PBR_FRAGMENT_SHADER_PATH "./shaders/pbr_shader.fs"
#define BASIC_VERTEX_SHADER_PATH "./shaders/basic_shader.vs"
#define BASIC_FRAGMENT_SHADER_PATH "./shaders/basic_shader.fs"
#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader pbr_shader, basic_shader, equirectangular_to_cube_shader;
static Perspective_Camera camera;
static Light* lights;
static Entity e, light_entity;
static u32 cube_map_tex, irradiance_map, bound_tex;

static Entity* test_entities;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 0.0f, 1.0f, 1.0f};
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

	vec3 light_position = (vec3) {1.0f, 0.5f, 0.5f};
	const r32 strongness = 1.0f;
	vec3 color = (vec3) {strongness, strongness, strongness};
	graphics_light_create(&light, light_position, color);
	array_push(lights, &light);

	Mesh m = graphics_mesh_create_from_obj_with_texture("./res/sphere.obj", -1, -1, -1, -1);
	graphics_entity_create(&light_entity, m,
		(vec4){lights[0].position.x, lights[0].position.y, lights[0].position.z, 1.0f},
		quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){0.1f, 0.1f, 0.1f});

	return lights;
}

static void create_test_entities()
{
	test_entities = array_create(Entity, 1);
	int x_n = 5, y_n = 5;

	const r32 distance = 0.3f;
	for (int y = 0; y < y_n; ++y)
	{
		for (int x = 0; x < x_n; ++x)
		{
			Mesh m = graphics_mesh_create_from_obj_with_texture("./res/sphere.obj", -1, -1, -1, -1);
			m.roughness_info.roughness = ((r32)x / (x_n - 1));
			m.metallic_info.metallic = ((r32)y / (y_n - 1));
			m.albedo_info.albedo = (vec3){1.0f, 0.0f, 0.0f};
			Entity entity;
			graphics_entity_create(&entity, m, (vec4){distance * x, distance * y, 0.0f, 1.0f},
				quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){0.1f, 0.1f, 0.1f});
			array_push(test_entities, &entity);
		}
	}
}

int core_init()
{
	// Create shader
	pbr_shader = graphics_shader_create(PBR_VERTEX_SHADER_PATH, PBR_FRAGMENT_SHADER_PATH);
	basic_shader = graphics_shader_create(BASIC_VERTEX_SHADER_PATH, BASIC_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = create_camera();
	// Create light
	lights = create_lights();

	create_test_entities();

	u32 albedo = graphics_texture_create("./res/rustediron2_basecolor.png");
	u32 metallic = graphics_texture_create("./res/rustediron2_metallic.png");
	u32 roughness = graphics_texture_create("./res/rustediron2_roughness.png");
	u32 normal = graphics_texture_create("./res/rustediron2_normal.png");
	Mesh m = graphics_mesh_create_from_obj_with_texture("./res/sphere.obj", normal, albedo, metallic, roughness);
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){0.1f, 0.1f, 0.1f});

	Float_Image_Data fid = graphics_float_image_load("./res/newport_loft.hdr");
	u32 equirectangular_map = graphics_texture_create_from_float_data(&fid);
	cube_map_tex = graphics_generate_cube_map_from_equirectangular_map(equirectangular_map);
	irradiance_map = graphics_generate_irradiance_map_from_cube_map(cube_map_tex);
	bound_tex = irradiance_map;

#if 0
	/* NORMALS TEST */
	// create plane
	Vertex* vertices = array_create(Vertex, 4);
	u32* indexes = array_create(u32, 6);
	Vertex v1, v2, v3, v4;
	v1.position = (vec4){-1.0f, -1.0f, 0.0f, 1.0f};
	v2.position = (vec4){1.0f, -1.0f, 0.0f, 1.0f};
	v3.position = (vec4){-1.0f, 1.0f, 0.0f, 1.0f};
	v4.position = (vec4){1.0f, 1.0f, 0.0f, 1.0f};
	v1.normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	v2.normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	v3.normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	v4.normal = (vec4){0.0f, 0.0f, 1.0f, 0.0f};
	v1.texture_coordinates = (vec2){0.0f, 0.0f};
	v2.texture_coordinates = (vec2){1.0f, 0.0f};
	v3.texture_coordinates = (vec2){0.0f, 1.0f};
	v4.texture_coordinates = (vec2){1.0f, 1.0f};
	v1.tangent = (vec4){1.0f, 0.0f, 0.0f, 0.0f};
	v2.tangent = (vec4){1.0f, 0.0f, 0.0f, 0.0f};
	v3.tangent = (vec4){1.0f, 0.0f, 0.0f, 0.0f};
	v4.tangent = (vec4){1.0f, 0.0f, 0.0f, 0.0f};
	array_push(vertices, &v1);
	array_push(vertices, &v2);
	array_push(vertices, &v3);
	array_push(vertices, &v4);
	u32 i1 = 0;
	u32 i2 = 1;
	u32 i3 = 2;
	u32 i4 = 1;
	u32 i5 = 3;
	u32 i6 = 2;
	array_push(indexes, &i1);
	array_push(indexes, &i2);
	array_push(indexes, &i3);
	array_push(indexes, &i4);
	array_push(indexes, &i5);
	array_push(indexes, &i6);
	u32 normal = graphics_texture_create("./res/brick_normals.png");
	u32 albedo = graphics_texture_create("./res/brickwall.jpg");
	Mesh m = graphics_mesh_create_with_texture(vertices, 4, indexes, 6, normal, albedo, -1, -1);
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){1.0f, 1.0f, 1.0f});
#endif

	//menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy()
{
	array_release(lights);
}

void core_update(r32 delta_time)
{
	graphics_entity_set_position(&light_entity,
		(vec4){lights[0].position.x, lights[0].position.y, lights[0].position.z, 1.0f});
	lights[0].position.x = light_entity.world_position.x;
	lights[0].position.y = light_entity.world_position.y;
	lights[0].position.z = light_entity.world_position.z;
}

void core_render()
{
	//graphics_entity_render_pbr_shader(pbr_shader, &camera, &e, lights, irradiance_map);
	graphics_entity_render_basic_shader(basic_shader, &camera, &light_entity);
	graphics_render_skybox(bound_tex, &camera);

	for (u32 i = 0; i < array_get_length(test_entities); ++i)
	{
		Entity* current_entity = &test_entities[i];
		graphics_entity_render_pbr_shader(pbr_shader, &camera, current_entity, lights, irradiance_map);
	}
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	r32 movement_speed = 3.0f;
	r32 rotation_speed = 300.0f;

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
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){1.0f, 0.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Y])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 1.0f, 0.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
	}
	if (key_state[GLFW_KEY_Z])
	{
		if (key_state[GLFW_KEY_LEFT_SHIFT] || key_state[GLFW_KEY_RIGHT_SHIFT])
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
		}
		else
		{
			Quaternion rotation = quaternion_new((vec3){0.0f, 0.0f, 1.0f}, -rotation_speed * delta_time);
			graphics_entity_set_rotation(&e, quaternion_product(&rotation, &e.world_rotation));
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
	if (key_state[GLFW_KEY_N])
	{
		static boolean normal = false;

		if (normal)
			e.mesh.normal_info.use = 0;
		else
			e.mesh.normal_info.use = 1;

		normal = !normal;
		key_state[GLFW_KEY_N] = false;
	}
	if (key_state[GLFW_KEY_B])
	{
		static boolean tex = false;

		if (tex)
			bound_tex = irradiance_map;
		else
			bound_tex = cube_map_tex;

		tex = !tex;
		key_state[GLFW_KEY_B] = false;
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