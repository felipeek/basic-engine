#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <iostream>
#include "graphics.h"
#include <dynamic_array.h>

extern "C" int obj_parse(const char* obj_path, Vertex** vertices, u32** indexes)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::string warn;
	std::string err;

	bool ret = tinyobj::LoadObj(&attrib, &shapes, 0, &warn, &err, obj_path, 0, true);

	if (!warn.empty()) {
		std::cout << warn << std::endl;
	}

		if (!err.empty()) {
		std::cerr << err << std::endl;
	}

	if (!ret) {
		exit(1);
	}

	*vertices = array_create(Vertex, 1);
	*indexes = array_create(u32, 1);

	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		for (size_t f = 0; f < shapes[s].mesh.indices.size(); f++) {
			array_push(*indexes, &shapes[s].mesh.indices[f].vertex_index);
		}
	}

	// Loop over vertices
	for (size_t f = 0; f < attrib.vertices.size(); f += 3) {
		Vertex v;
		v.position.x = attrib.vertices[f];
		v.position.y = attrib.vertices[f + 1];
		v.position.z = attrib.vertices[f + 2];
		v.position.w = 1.0f;
		v.normal = (vec4){0.0f, 0.0f, 0.0f, 0.0f};
		v.texture_coordinates = (vec2) {0.0f, 0.0f};
		array_push(*vertices, &v);
	}

	// If normals were provided... fill them
	for (size_t f = 0; f < attrib.normals.size(); f += 3) {
		Vertex* v = &(*vertices)[f / 3];
		v->normal.x = attrib.normals[f];
		v->normal.y = attrib.normals[f + 1];
		v->normal.z = attrib.normals[f + 2];
		v->normal.w = 0.0f;
	}

	if (attrib.normals.empty()) {
		// Calculate normals
		size_t indexes_length = array_get_length(*indexes);

		for (size_t i = 0; i < indexes_length; i += 3)
		{
			Vertex* vertex_a, *vertex_b, *vertex_c;
			unsigned int i1 = (*indexes)[i + 0];
			unsigned int i2 = (*indexes)[i + 1];
			unsigned int i3 = (*indexes)[i + 2];

			// Find vertices
			vertex_a = *vertices + i1;
			vertex_b = *vertices + i2;
			vertex_c = *vertices + i3;

			// Manually calculate triangle's normal
			vec3 A = (vec3) {vertex_a->position.x, vertex_a->position.y, vertex_a->position.z};
			vec3 B = (vec3) {vertex_b->position.x, vertex_b->position.y, vertex_b->position.z};
			vec3 C = (vec3) {vertex_c->position.x, vertex_c->position.y, vertex_c->position.z};
			vec3 first_edge = (vec3){B.x - A.x, B.y - A.y, B.z - A.z};
			vec3 second_edge = (vec3){C.x - A.x, C.y - A.y, C.z - A.z};
			vec4 normal;
			normal.x = first_edge.y * second_edge.z - first_edge.z * second_edge.y;
			normal.y = first_edge.z * second_edge.x - first_edge.x * second_edge.z;
			normal.z = first_edge.x * second_edge.y - first_edge.y * second_edge.x;

			// Assign normals
			vertex_a->normal = (vec4){vertex_a->normal.x + normal.x, vertex_a->normal.y + normal.y, vertex_a->normal.z + normal.z};
			vertex_b->normal = (vec4){vertex_b->normal.x + normal.x, vertex_b->normal.y + normal.y, vertex_b->normal.z + normal.z};
			vertex_c->normal = (vec4){vertex_c->normal.x + normal.x, vertex_c->normal.y + normal.y, vertex_c->normal.z + normal.z};
		}
	}

	// If tex_coords were provided... fill them
	for (size_t f = 0; f < attrib.texcoords.size(); f += 2) {
		Vertex* v = &(*vertices)[f / 2];
		v->texture_coordinates.x = attrib.texcoords[f];
		v->texture_coordinates.y = attrib.texcoords[f + 1];
	}

	return 0;
}