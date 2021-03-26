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
	Normal_Mapping_Info normal_info;
	Vertex* vertices;
	u32* indices;
} Mesh;

typedef struct
{
	Mesh mesh;
	vec4 world_position;
	Quaternion world_rotation;
	vec3 world_scale;
	mat4 model_matrix;
	Diffuse_Info diffuse_info;
} Entity;

typedef struct {
	vec3 position;
	vec3 velocity;
	r32 inverse_mass;

	vec3 candidate_position; // auxiliar
} Particle;

typedef enum {
	CONSTRAINT_DISTANCE
} Constraint_Type;

typedef struct {
	u32 p1;
	u32 p2;
	r32 distance;
} Distance_Constraint;

typedef struct {
	Constraint_Type type;
	r32 stiffness;
	union {
		Distance_Constraint distance_constraint;
	};
} Constraint;

typedef struct {
	Particle* particles;
	Constraint* constraints;
} Particle_Object;

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

Image_Data graphics_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_load(const s8* image_path);
Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data);
void graphics_image_free(Image_Data* image_data);
void graphics_float_image_Free(Float_Image_Data* image_data);
void graphics_image_save(const s8* image_path, const Image_Data* image_data);
void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data);
Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
Mesh graphics_quad_create();
Mesh graphics_mesh_create(Vertex* vertices, u32* indices, Normal_Mapping_Info* normal_info);
Mesh graphics_mesh_create_from_obj(const s8* obj_path, Normal_Mapping_Info* normal_info);
void graphics_mesh_render(Shader shader, Mesh mesh);
void graphics_entity_create_with_color(Entity* entity, Mesh mesh, vec4 world_position, Quaternion world_rotation, vec3 world_scale, vec4 color);
void graphics_entity_create_with_texture(Entity* entity, Mesh mesh, vec4 world_position, Quaternion world_rotation, vec3 world_scale, u32 texture);
void graphics_entity_destroy(Entity* entity);
// If entity already has a diffuse map, the older diffuse map will be deleted if delete_diffuse_map is true.
// If entity has a color instead of a diffuse map, the mesh will lose the color and be set to use the diffuse map.
void graphics_entity_change_diffuse_map(Entity* entity, u32 diffuse_map, boolean delete_diffuse_map);
// If the entity already has a color, the older color will be deleted.
// If entity has a diffuse map instead of a color, the diffuse map will be deleted if delete_diffuse_map is true
// The entity will be set to use the color.
void graphics_entity_change_color(Entity* entity, vec4 color, boolean delete_diffuse_map);
void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map);
void graphics_entity_set_position(Entity* entity, vec4 world_position);
void graphics_entity_set_rotation(Entity* entity, Quaternion world_rotation);
void graphics_entity_set_scale(Entity* entity, vec3 world_scale);
void graphics_entity_render_basic_shader(const Perspective_Camera* camera, const Entity* entity);
void graphics_entity_render_phong_shader(const Perspective_Camera* camera, const Entity* entity, const Light* lights);
void graphics_particle_object_create(Particle_Object* po);
void graphics_particle_object_render_phong_shader(const Perspective_Camera* camera, const Particle_Object* po, const Light* lights);
void graphics_light_create(Light* light, vec4 position, vec4 ambient_color, vec4 diffuse_color, vec4 specular_color);
u32 graphics_texture_create(const s8* texture_path);
u32 graphics_texture_create_from_data(const Image_Data* image_data);
u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data);
void graphics_texture_delete(u32 texture_id);
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory);
Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_Data, u8* memory);

#endif