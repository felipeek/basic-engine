#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include <stdio.h>
#include <math.h>
#include <stb_image_write.h>
#include "core.h"
#include "graphics.h"
#include "obj.h"
#include "menu.h"

#define CLAMP(V, X, Y) (((V) < (X)) ? (X) : ((V) > (Y)) ? (Y) : (V))

#define PBR_VERTEX_SHADER_PATH "./shaders/pbr_shader.vs"
#define PBR_FRAGMENT_SHADER_PATH "./shaders/pbr_shader.fs"
#define BASIC_VERTEX_SHADER_PATH "./shaders/basic_shader.vs"
#define BASIC_FRAGMENT_SHADER_PATH "./shaders/basic_shader.fs"
#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader pbr_shader, basic_shader, equirectangular_to_cube_shader;
static Perspective_Camera camera;
static Light* lights;
static Entity e, light_entity;

typedef struct
{
	u32 environment_map, irradiance_map, prefiltered_map, brdf_lut;
} IBL_Data;
static IBL_Data* ibl_datas;
static u32 bound_ibl_data = 0;

static Entity* test_entities;

static Perspective_Camera create_camera()
{
	Perspective_Camera camera;
	vec4 camera_position =(vec4) {0.0f, 2.0f, 6.0f, 1.0f};
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

static IBL_Data ibl_data_create(const char* hdr_tex_file)
{
	IBL_Data ibl_data;
	Float_Image_Data fid = graphics_float_image_load(hdr_tex_file);
	u32 equirectangular_map = graphics_texture_create_from_float_data(&fid);
	ibl_data.environment_map = graphics_generate_cube_map_from_equirectangular_map(equirectangular_map);
	ibl_data.irradiance_map = graphics_generate_irradiance_map_from_cube_map(ibl_data.environment_map);
	ibl_data.prefiltered_map = graphics_generate_prefiltered_environment_map_from_cube_map(ibl_data.environment_map);
	ibl_data.brdf_lut = graphics_generate_brdf_lut_tex();
	return ibl_data;
}

static Float_Image_Data extract_3d_texture(u32 texture, GLenum texture_specific_target, GLint level)
{
	glBindTexture(GL_TEXTURE_CUBE_MAP, texture);

	u32 width, height;
	glGetTexLevelParameteriv(texture_specific_target, level, GL_TEXTURE_WIDTH, &width);
	glGetTexLevelParameteriv(texture_specific_target, level, GL_TEXTURE_HEIGHT, &height);

	r32* pixels = malloc(width * height * 3 * sizeof(r32));
	glGetTexImage(texture_specific_target, level, GL_RGB, GL_FLOAT, pixels);

	Float_Image_Data fid;
	fid.channels = 3;
	fid.data = pixels;
	fid.height = height;
	fid.width = width;

	return fid;
}

static Float_Image_Data extract_brdf_lut_texture(u32 texture)
{
	glBindTexture(GL_TEXTURE_2D, texture);

	u32 width, height;
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
	glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);

	r32* pixels = malloc(width * height * 2 * sizeof(r32));
	glGetTexImage(GL_TEXTURE_2D, 0, GL_RG, GL_FLOAT, pixels);

	Float_Image_Data fid;
	fid.channels = 2;
	fid.data = pixels;
	fid.height = height;
	fid.width = width;

	return fid;
}

static IBL_Data duplicate_ibl_data(const IBL_Data* ibl_data)
{
	IBL_Data dup;
	Float_Image_Data fid;

	// ENVIRONMENT MAP
	unsigned int env_cube_map;
	glGenTextures(1, &env_cube_map);
	for (unsigned int i = 0; i < 6; ++i)
	{
		fid = extract_3d_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0);
		// note that we store each face with 16 bit floating point values
		glBindTexture(GL_TEXTURE_CUBE_MAP, env_cube_map);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 
					fid.width, fid.height, 0, GL_RGB, GL_FLOAT, fid.data);
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	void glGenerateMipmap(GLenum target);
	// used to pre-compute prefiltered map.. i dont think we need it for rendering
	glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
	dup.environment_map = env_cube_map;

	// IRRADIANCE MAP
	unsigned int irradiance_map;
	glGenTextures(1, &irradiance_map);
	for (unsigned int i = 0; i < 6; ++i)
	{
		fid = extract_3d_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0);
		// note that we store each face with 16 bit floating point values
		glBindTexture(GL_TEXTURE_CUBE_MAP, irradiance_map);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 
					fid.width, fid.height, 0, GL_RGB, GL_FLOAT, fid.data);
	}
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	dup.irradiance_map = irradiance_map;

	// PREFILTERED MAP
	unsigned int prefiltered_map;
	glGenTextures(1, &prefiltered_map);
	glBindTexture(GL_TEXTURE_CUBE_MAP, prefiltered_map);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// Pre-allocate memory for texture (mipmap 0), so we can allocate the correct memory for the mipmaps
	// (this is needed because we need to set the size and format of the texture via glTexImage2D before allocating the mipmaps)
	for (unsigned int i = 0; i < 6; ++i)
	{
		fid = extract_3d_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0);
		// note that we store each face with 16 bit floating point values
		glBindTexture(GL_TEXTURE_CUBE_MAP, prefiltered_map);
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_RGB16F, 
					fid.width, fid.height, 0, GL_RGB, GL_FLOAT, 0);
	}
	// allocate memory for mipmaps
	glGenerateMipmap(GL_TEXTURE_CUBE_MAP);
	for (u32 mip = 0; mip < 5; ++mip)
	{
		for (unsigned int i = 0; i < 6; ++i)
		{
			fid = extract_3d_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, mip);
			// note that we store each face with 16 bit floating point values
			glBindTexture(GL_TEXTURE_CUBE_MAP, prefiltered_map);
			glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, mip, GL_RGB16F, 
						fid.width, fid.height, 0, GL_RGB, GL_FLOAT, fid.data);
		}
	}
	dup.prefiltered_map = prefiltered_map;

	// BRDF LUT TEXTURE
	u32 brdf_lut_tex;

	glGenTextures(1, &brdf_lut_tex);
	fid = extract_brdf_lut_texture(ibl_data->brdf_lut);
	glBindTexture(GL_TEXTURE_2D, brdf_lut_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RG16F, fid.width, fid.height, 0, GL_RG, GL_FLOAT, fid.data);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glBindTexture(GL_TEXTURE_2D, 0);
	dup.brdf_lut = brdf_lut_tex;

	return dup;
}

static void store_texture(u32 texture, GLenum texture_target, GLenum texture_specific_target, GLint level, const char* path)
{
	glBindTexture(texture_target, texture);

	u32 width, height;
	glGetTexLevelParameteriv(texture_specific_target, level, GL_TEXTURE_WIDTH, &width);
	glGetTexLevelParameteriv(texture_specific_target, level, GL_TEXTURE_HEIGHT, &height);

	r32* pixels = malloc(width * height * 3 * sizeof(r32));
	glGetTexImage(texture_specific_target, level, GL_RGB, GL_FLOAT, pixels);

	u8* u8_pixels = malloc(width * height * 3 * sizeof(u8));
	for (u32 y = 0; y < height; ++y)
	{
		for (u32 x = 0; x < width; ++x)
		{
			r32 r = CLAMP(pixels[y * width * 3 + x * 3 + 0], 0.0f, 1.0f);
			r32 g = CLAMP(pixels[y * width * 3 + x * 3 + 1], 0.0f, 1.0f);
			r32 b = CLAMP(pixels[y * width * 3 + x * 3 + 2], 0.0f, 1.0f);
			u8_pixels[y * width * 3 + x * 3 + 0] = (u8)floorf(255.0f * r);
			u8_pixels[y * width * 3 + x * 3 + 1] = (u8)floorf(255.0f * g);
			u8_pixels[y * width * 3 + x * 3 + 2] = (u8)floorf(255.0f * b);
		}
	}
	stbi_write_png(path, width, height, 3, u8_pixels, width * 3 * sizeof(u8));
	free(pixels);
	free(u8_pixels);
}

static void read_ibl_textures(const IBL_Data* ibl_data)
{
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, "output/env_map_pos_x.png");
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, "output/env_map_pos_y.png");
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, "output/env_map_pos_z.png");
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, "output/env_map_neg_x.png");
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, "output/env_map_neg_y.png");
	store_texture(ibl_data->environment_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, "output/env_map_neg_z.png");

	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, "output/irr_map_pos_x.png");
	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, "output/irr_map_pos_y.png");
	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, "output/irr_map_pos_z.png");
	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, "output/irr_map_neg_x.png");
	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, "output/irr_map_neg_y.png");
	store_texture(ibl_data->irradiance_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, "output/irr_map_neg_z.png");

	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 0, "output/prefiltered_map_pos_x_0.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 0, "output/prefiltered_map_pos_y_0.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 0, "output/prefiltered_map_pos_z_0.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 0, "output/prefiltered_map_neg_x_0.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 0, "output/prefiltered_map_neg_y_0.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 0, "output/prefiltered_map_neg_z_0.png");

	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 1, "output/prefiltered_map_pos_x_1.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 1, "output/prefiltered_map_pos_y_1.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 1, "output/prefiltered_map_pos_z_1.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 1, "output/prefiltered_map_neg_x_1.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 1, "output/prefiltered_map_neg_y_1.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 1, "output/prefiltered_map_neg_z_1.png");

	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 2, "output/prefiltered_map_pos_x_2.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 2, "output/prefiltered_map_pos_y_2.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 2, "output/prefiltered_map_pos_z_2.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 2, "output/prefiltered_map_neg_x_2.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 2, "output/prefiltered_map_neg_y_2.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 2, "output/prefiltered_map_neg_z_2.png");

	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 3, "output/prefiltered_map_pos_x_3.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 3, "output/prefiltered_map_pos_y_3.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 3, "output/prefiltered_map_pos_z_3.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 3, "output/prefiltered_map_neg_x_3.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 3, "output/prefiltered_map_neg_y_3.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 3, "output/prefiltered_map_neg_z_3.png");

	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_X, 4, "output/prefiltered_map_pos_x_4.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Y, 4, "output/prefiltered_map_pos_y_4.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_POSITIVE_Z, 4, "output/prefiltered_map_pos_z_4.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_X, 4, "output/prefiltered_map_neg_x_4.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Y, 4, "output/prefiltered_map_neg_y_4.png");
	store_texture(ibl_data->prefiltered_map, GL_TEXTURE_CUBE_MAP, GL_TEXTURE_CUBE_MAP_NEGATIVE_Z, 4, "output/prefiltered_map_neg_z_4.png");

	store_texture(ibl_data->brdf_lut, GL_TEXTURE_2D, GL_TEXTURE_2D, 0, "output/brdf_lut.png");
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

	ibl_datas = array_create(IBL_Data, 1);
	IBL_Data ibl_data = ibl_data_create("./res/newport_loft.hdr");
	IBL_Data dup = duplicate_ibl_data(&ibl_data);
	read_ibl_textures(&dup);
	array_push(ibl_datas, &dup);
	ibl_data = ibl_data_create("./res/immenstadter_horn_8k.hdr");
	//array_push(ibl_datas, &ibl_data);

	test_entities = array_create(Entity, 1);
	//create_test_entities();

	//u32 albedo = graphics_texture_create("./res/rustediron2_basecolor.png");
	//u32 metallic = graphics_texture_create("./res/rustediron2_metallic.png");
	//u32 roughness = graphics_texture_create("./res/rustediron2_roughness.png");
	//u32 normal = graphics_texture_create("./res/rustediron2_normal.png");
	//Mesh m = graphics_mesh_create_from_obj_with_texture("./res/sphere.obj", normal, albedo, metallic, roughness);
	//graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){0.1f, 0.1f, 0.1f});

	u32 albedo = graphics_texture_create("./res/hydrant/DefaultMaterial_Base_Color.png");
	u32 metallic = graphics_texture_create("./res/hydrant/DefaultMaterial_Metallic.png");
	u32 roughness = graphics_texture_create("./res/hydrant/DefaultMaterial_Roughness.png");
	u32 normal = graphics_texture_create("./res/hydrant/DefaultMaterial_Normal_OpenGL.png");
	//u32 albedo = graphics_texture_create("./res/hydrant/tex2/hydrant_DefaultMaterial_BaseColor.png");
	//u32 metallic = graphics_texture_create("./res/hydrant/tex2/hydrant_DefaultMaterial_Metallic.png");
	//u32 roughness = graphics_texture_create("./res/hydrant/tex2/hydrant_DefaultMaterial_Roughness.png");
	//u32 normal = graphics_texture_create("./res/hydrant/tex2/hydrant_DefaultMaterial_Normal.png");
	Mesh m = graphics_mesh_create_from_obj_with_texture("./res/hydrant/hydrant.obj", normal, albedo, metallic, roughness);
	graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 360.0f - 45.0f), (vec3){1.0f, 1.0f, 1.0f});

	//u32 albedo = graphics_texture_create("./res/coupe/coupe_d.png");
	//u32 metallic = graphics_texture_create("./res/coupe/coupe_m.png");
	//u32 roughness = graphics_texture_create("./res/coupe/coupe_r.png");
	//u32 normal = graphics_texture_create("./res/coupe/coupe_n.png");
	//Mesh m = graphics_mesh_create_from_obj_with_texture("./res/coupe/coupe.obj", normal, albedo, metallic, roughness);
	//graphics_entity_create(&e, m, (vec4){0.0f, 0.0f, 0.0f, 1.0f}, quaternion_new((vec3){0.0f, 1.0f, 0.0f}, 0.0f), (vec3){1.0f, 1.0f, 1.0f});


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
	graphics_entity_render_pbr_shader(pbr_shader, &camera, &e, lights, ibl_datas[bound_ibl_data].irradiance_map,
	ibl_datas[bound_ibl_data].prefiltered_map, ibl_datas[bound_ibl_data].brdf_lut);
	graphics_entity_render_basic_shader(basic_shader, &camera, &light_entity);
	graphics_render_skybox(ibl_datas[bound_ibl_data].environment_map, &camera);

	for (u32 i = 0; i < array_get_length(test_entities); ++i)
	{
		Entity* current_entity = &test_entities[i];
		graphics_entity_render_pbr_shader(pbr_shader, &camera, current_entity, lights, ibl_datas[bound_ibl_data].irradiance_map,
			ibl_datas[bound_ibl_data].prefiltered_map, ibl_datas[bound_ibl_data].brdf_lut);
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
		++bound_ibl_data;
		if (bound_ibl_data == array_get_length(ibl_datas))
			bound_ibl_data = 0;
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