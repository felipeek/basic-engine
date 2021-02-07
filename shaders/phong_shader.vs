#version 330 core

layout (location = 0) in vec4 vertex_position;
layout (location = 1) in vec4 vertex_normal;
layout (location = 2) in vec2 vertex_texture_coords;

out vec4 fragment_position;
out vec4 fragment_normal;
out vec2 fragment_texture_coords;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	vec3 normal_3D = mat3(inverse(transpose(model_matrix))) * vertex_normal.xyz;
	fragment_normal = vec4(normal_3D, 0.0);
	fragment_texture_coords = vertex_texture_coords;
	fragment_position = model_matrix * vertex_position;
	gl_Position = projection_matrix * view_matrix * model_matrix * vertex_position;
}