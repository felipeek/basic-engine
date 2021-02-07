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
} Vertex;
#pragma pack(pop)

typedef struct
{
	boolean use_normal_map;
	boolean tangent_space;		// @TODO: Not implemented yet.
	u32 normal_map_texture;
} Normal_Mapping_Info;

typedef struct
{
	boolean use_diffuse_map;
	u32 diffuse_map;
	vec4 diffuse_color;
} Diffuse_Info;

typedef struct
{
	u32 VAO, VBO, EBO;
	s32 indexes_size;
	Normal_Mapping_Info normal_info;
	Diffuse_Info diffuse_info;
} Mesh;

typedef struct
{
	Mesh mesh;
	vec4 world_position;
	vec3 world_rotation;
	vec3 world_scale;
	mat4 model_matrix;
} Entity;

typedef struct
{
	vec4 position;
	vec4 ambient_color;
	vec4 diffuse_color;
	vec4 specular_color;
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

extern Image_Data graphics_image_load(const s8* image_path);
extern Float_Image_Data graphics_float_image_load(const s8* image_path);
extern Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data);
extern void graphics_image_free(Image_Data* image_data);
extern void graphics_float_image_Free(Float_Image_Data* image_data);
extern void graphics_image_save(const s8* image_path, const Image_Data* image_data);
extern void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data);
extern Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
extern Mesh graphics_quad_create_with_texture(u32 texture);
extern Mesh graphics_quad_create_with_color(vec4 color);
extern Mesh graphics_mesh_create_with_texture(Vertex* vertices, s32 vertices_size, u32* indices, s32 indices_size, Normal_Mapping_Info* normal_info, u32 diffuse_map);
extern Mesh graphics_mesh_create_with_color(Vertex* vertices, s32 vertices_size, u32* indices, s32 indices_size, Normal_Mapping_Info* normal_info, vec4 diffuse_color);
extern Mesh graphics_mesh_create_from_obj_with_color(const s8* obj_path, Normal_Mapping_Info* normal_info, vec4 diffuse_color);
extern Mesh graphics_mesh_create_from_obj_with_texture(const s8* obj_path, Normal_Mapping_Info* normal_info, u32 diffuse_map);
extern void graphics_mesh_render(Shader shader, Mesh mesh);
// If mesh already has a diffuse map, the older diffuse map will be deleted if delete_diffuse_map is true.
// If mesh has a color instead of a diffuse map, the mesh will lose the color and be set to use the diffuse map.
extern void graphics_mesh_change_diffuse_map(Mesh* mesh, u32 diffuse_map, boolean delete_diffuse_map);
// If the mesh already has a color, the older color will be deleted.
// If mesh has a diffuse map instead of a color, the diffuse map will be deleted if delete_diffuse_map is true
// The mesh will be set to use the color.
extern void graphics_mesh_change_color(Mesh* mesh, vec4 color, boolean delete_diffuse_map);
extern void graphics_entity_create(Entity* entity, Mesh mesh, vec4 world_position, vec3 world_rotation, vec3 world_scale);
extern void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map, boolean delete_diffuse_map);
extern void graphics_entity_set_position(Entity* entity, vec4 world_position);
extern void graphics_entity_set_rotation(Entity* entity, vec3 world_rotation);
extern void graphics_entity_set_scale(Entity* entity, vec3 world_scale);
extern void graphics_entity_render_basic_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity);
extern void graphics_entity_render_phong_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity, const Light* lights);
extern void graphics_light_create(Light* light, vec4 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color);
extern u32 graphics_texture_create(const s8* texture_path);
extern u32 graphics_texture_create_from_data(const Image_Data* image_data);
extern u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data);
extern void graphics_texture_delete(u32 texture_id);
extern Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory);
extern Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_Data, u8* memory);

#endif