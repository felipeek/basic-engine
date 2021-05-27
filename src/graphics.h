#ifndef BASIC_ENGINE_GRAPHICS_H
#define BASIC_ENGINE_GRAPHICS_H
#include "gm.h"

typedef u32 Shader;

#pragma pack(push, 1)
typedef struct
{
	vec2 position;
} Vertex;
#pragma pack(pop)

typedef struct
{
	u32 VAO, VBO, EBO;
	Vertex* vertices;
	u32* indices;
} Mesh;

typedef struct
{
	Mesh mesh;
	vec2 world_position;
	vec3 color;
} Entity;

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path);
Mesh graphics_mesh_create(Vertex* vertices, u32* indices);
void graphics_mesh_render(Shader shader, Mesh mesh);
void graphics_entity_create_with_color(Entity* entity, Mesh mesh, vec2 world_position, vec3 color);
void graphics_entity_render_basic_shader(const Entity* entity);

#endif