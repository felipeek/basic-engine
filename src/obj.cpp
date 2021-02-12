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

	// For now, we are not re-using vertices, i.e., our indices are 0 1 2 3 4 5 6 7...
	// This is because tinyobj loads the object in such a way that the position, tex coords and normals are completely independent
	// vectors (just like in the obj file)
	// for this reason, if we wanna re-use vertices, we need to be sure that all three pos,tex_coords,normals are the same...
	// for example, we may have two indices poiting to the same vertex position, but using different uv_coords... so we would not
	// be able to re-use this specific vertex in this case
	// we can make such algorithm if needed, but for now, we are just duplicating everything
	s32 index_counter = 0;
	for (size_t s = 0; s < shapes.size(); s++) {
		for (size_t f = 0; f < shapes[s].mesh.indices.size() / 3; f++) {
			array_push(*indexes, &index_counter);
			++index_counter;
			array_push(*indexes, &index_counter);
			++index_counter;
			array_push(*indexes, &index_counter);
			++index_counter;

			tinyobj::index_t idx0 = shapes[s].mesh.indices[3 * f + 0];
			tinyobj::index_t idx1 = shapes[s].mesh.indices[3 * f + 1];
			tinyobj::index_t idx2 = shapes[s].mesh.indices[3 * f + 2];
			Vertex v[3];

			if (attrib.texcoords.size() > 0) {
				v[0].texture_coordinates.x = attrib.texcoords[2 * idx0.texcoord_index];
				v[0].texture_coordinates.y = attrib.texcoords[2 * idx0.texcoord_index + 1];
				v[1].texture_coordinates.x = attrib.texcoords[2 * idx1.texcoord_index];
				v[1].texture_coordinates.y = attrib.texcoords[2 * idx1.texcoord_index + 1];
				v[2].texture_coordinates.x = attrib.texcoords[2 * idx2.texcoord_index];
				v[2].texture_coordinates.y = attrib.texcoords[2 * idx2.texcoord_index + 1];
			}

			int f0 = idx0.vertex_index;
			int f1 = idx1.vertex_index;
			int f2 = idx2.vertex_index;

			v[0].position.x = attrib.vertices[3 * f0 + 0];
			v[0].position.y = attrib.vertices[3 * f0 + 1];
			v[0].position.z = attrib.vertices[3 * f0 + 2];
			v[0].position.w = 1.0f;
			v[1].position.x = attrib.vertices[3 * f1 + 0];
			v[1].position.y = attrib.vertices[3 * f1 + 1];
			v[1].position.z = attrib.vertices[3 * f1 + 2];
			v[1].position.w = 1.0f;
			v[2].position.x = attrib.vertices[3 * f2 + 0];
			v[2].position.y = attrib.vertices[3 * f2 + 1];
			v[2].position.z = attrib.vertices[3 * f2 + 2];
			v[2].position.w = 1.0f;

			if (attrib.normals.size() > 0) {
				int nf0 = idx0.normal_index;
				int nf1 = idx1.normal_index;
				int nf2 = idx2.normal_index;

				v[0].normal.x = attrib.normals[3 * nf0 + 0];
				v[0].normal.y = attrib.normals[3 * nf0 + 1];
				v[0].normal.z = attrib.normals[3 * nf0 + 2];
				v[0].normal.w = 0.0f;
				v[1].normal.x = attrib.normals[3 * nf1 + 0];
				v[1].normal.y = attrib.normals[3 * nf1 + 1];
				v[1].normal.z = attrib.normals[3 * nf1 + 2];
				v[1].normal.w = 0.0f;
				v[2].normal.x = attrib.normals[3 * nf2 + 0];
				v[2].normal.y = attrib.normals[3 * nf2 + 1];
				v[2].normal.z = attrib.normals[3 * nf2 + 2];
				v[2].normal.w = 0.0f;
			}
			array_push(*vertices, &v[0]);
			array_push(*vertices, &v[1]);
			array_push(*vertices, &v[2]);
		}
	}

	return 0;
}