#define TINYOBJLOADER_IMPLEMENTATION
#include <tiny_obj_loader.h>
#include <iostream>
#include "graphics.hpp"
#include <dynamic_array.h>

int objParse(const char* objPath, Vertex** vertices, u32** indexes)
{
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::string warn;
	std::string err;

	bool ret = tinyobj::LoadObj(&attrib, &shapes, 0, &warn, &err, objPath, 0, true);

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
		v.normal = (Vec4){0.0f, 0.0f, 0.0f, 0.0f};
		v.textureCoordinates = (Vec2) {0.0f, 0.0f};
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
		size_t indexesLength = array_get_length(*indexes);

		for (size_t i = 0; i < indexesLength; i += 3)
		{
			Vertex* vertexA, *vertexB, *vertexC;
			unsigned int i1 = (*indexes)[i + 0];
			unsigned int i2 = (*indexes)[i + 1];
			unsigned int i3 = (*indexes)[i + 2];

			// Find vertices
			vertexA = *vertices + i1;
			vertexB = *vertices + i2;
			vertexC = *vertices + i3;

			// Manually calculate triangle's normal
			Vec3 A = (Vec3) {vertexA->position.x, vertexA->position.y, vertexA->position.z};
			Vec3 B = (Vec3) {vertexB->position.x, vertexB->position.y, vertexB->position.z};
			Vec3 C = (Vec3) {vertexC->position.x, vertexC->position.y, vertexC->position.z};
			Vec3 firstEdge = (Vec3){B.x - A.x, B.y - A.y, B.z - A.z};
			Vec3 secondEdge = (Vec3){C.x - A.x, C.y - A.y, C.z - A.z};
			Vec4 normal;
			normal.x = firstEdge.y * secondEdge.z - firstEdge.z * secondEdge.y;
			normal.y = firstEdge.z * secondEdge.x - firstEdge.x * secondEdge.z;
			normal.z = firstEdge.x * secondEdge.y - firstEdge.y * secondEdge.x;

			// Assign normals
			vertexA->normal = (Vec4){vertexA->normal.x + normal.x, vertexA->normal.y + normal.y, vertexA->normal.z + normal.z};
			vertexB->normal = (Vec4){vertexB->normal.x + normal.x, vertexB->normal.y + normal.y, vertexB->normal.z + normal.z};
			vertexC->normal = (Vec4){vertexC->normal.x + normal.x, vertexC->normal.y + normal.y, vertexC->normal.z + normal.z};
		}
	}

	// If texCoords were provided... fill them
	for (size_t f = 0; f < attrib.texcoords.size(); f += 2) {
		Vertex* v = &(*vertices)[f / 2];
		v->textureCoordinates.x = attrib.texcoords[f];
		v->textureCoordinates.y = attrib.texcoords[f + 1];
	}

	return 0;
}