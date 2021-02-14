#ifndef BASIC_ENGINE_GRAPHICS_H
#define BASIC_ENGINE_GRAPHICS_H
#include "gm.h"
#include "camera.h"

typedef u32 Shader;

#pragma pack(push, 1)
typedef struct
{
	vec4 position;
	vec4 normal;
	vec2 texture_coordinates;
	vec4 tangent;
} Vertex;
#pragma pack(pop)

typedef struct
{
	boolean use;
	u32 map;
} Normal_Info;

typedef struct
{
	boolean use;
	u32 map;
	vec3 albedo;
} Albedo_Info;

typedef struct
{
	boolean use;
	u32 map;
	r32 metallic;
} Metallic_Info;

typedef struct
{
	boolean use;
	u32 map;
	r32 roughness;
} Roughness_Info;

typedef struct
{
	u32 VAO, VBO, EBO;
	s32 indexes_size;
	Normal_Info normal_info;
	Albedo_Info albedo_info;
	Metallic_Info metallic_info;
	Roughness_Info roughness_info;
} Mesh;

typedef struct
{
	Mesh mesh;
	vec4 world_position;
	Quaternion world_rotation;
	vec3 world_scale;
	mat4 model_matrix;
} Entity;

typedef struct
{
	vec3 position;
	vec3 color;
} Light;

typedef struct
{
	u8* data;
	s32 width, height, channels;
} Image_Data;

typedef struct
{
	r32* data;
	s32 width, height, channels;
} Float_Image_Data;

Image_Data graphics_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data);
void graphics_image_free(Image_Data* image_data);
void graphics_float_image_Free(Float_Image_Data* image_data);
void graphics_image_save(const s8* image_path, const Image_Data* image_data);
void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data);
Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
Mesh graphics_mesh_create_with_texture(Vertex* vertices, s32 vertices_size, u32* indices, s32 indices_size, u32 normal_map,
	u32 albedo_map, u32 metallic_map, u32 roughness_map);
Mesh graphics_mesh_create_from_obj_with_texture(const s8* obj_path, u32 normal_map, u32 albedo_map, u32 metallic_map, u32 roughness_map);
void graphics_mesh_render(Shader shader, Mesh mesh);
// If mesh already has an albedo/metallic/roughness map, the older diffuse map will be deleted if delete_*_map is true.
void graphics_mesh_change_normal_map(Mesh* mesh, u32 normal_map, boolean delete_normal_map);
void graphics_mesh_change_albedo_map(Mesh* mesh, u32 albedo_map, boolean delete_albedo_map);
void graphics_mesh_change_metallic_map(Mesh* mesh, u32 metallic_map, boolean delete_metallic_map);
void graphics_mesh_change_roughness_map(Mesh* mesh, u32 roughness_map, boolean delete_roughness_map);
void graphics_entity_create(Entity* entity, Mesh mesh, vec4 world_position, Quaternion world_rotation, vec3 world_scale);
void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map, boolean delete_albedo_map,
	boolean delete_metallic_map, boolean delete_roughness_map);
void graphics_entity_set_position(Entity* entity, vec4 world_position);
void graphics_entity_set_rotation(Entity* entity, Quaternion world_rotation);
void graphics_entity_set_scale(Entity* entity, vec3 world_scale);
void graphics_entity_render_basic_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity);
void graphics_entity_render_pbr_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity, const Light* lights);
void graphics_light_create(Light* light, vec3 position, vec3 color);
u32 graphics_texture_create(const s8* texture_path);
u32 graphics_texture_create_from_data(const Image_Data* image_data);
u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data);
void graphics_texture_delete(u32 texture_id);
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory);
Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_Data, u8* memory);
u32 graphics_generate_cube_map_from_equirectangular_map(u32 equirectangular_map);
void graphics_render_skybox(u32 skybox_texture, const Perspective_Camera* camera);

#endif