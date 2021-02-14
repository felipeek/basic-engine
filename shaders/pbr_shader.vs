#version 330 core

layout (location = 0) in vec4 vertex_position;
layout (location = 1) in vec4 vertex_normal;
layout (location = 2) in vec2 vertex_texture_coords;
layout (location = 3) in vec4 vertex_tangent;

out vec3 fragment_position;
out vec3 fragment_normal;
out vec2 fragment_texture_coords;
out mat4 tangent_matrix;

struct Normal_Info
{
  bool use;
  sampler2D map;
};

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 projection_matrix;
uniform Normal_Info normal_info;

void main()
{
	vec3 normal_3D = mat3(inverse(transpose(model_matrix))) * vertex_normal.xyz;
	fragment_normal = normal_3D;
	fragment_texture_coords = vertex_texture_coords;
	fragment_position = (model_matrix * vertex_position).xyz;

	// Build TBN matrix (normal mapping)
	if (normal_info.use)
	{
		vec4 T = normalize(model_matrix * vec4(vertex_tangent.xyz, 0.0));
		vec4 N = normalize(model_matrix * vec4(vertex_normal.xyz, 0.0));
		vec4 B = vec4(cross(T.xyz, N.xyz), 0.0);
		tangent_matrix = mat4(T, B, N, vec4(0.0, 0.0, 0.0, 0.0));
	}

	gl_Position = projection_matrix * view_matrix * model_matrix * vertex_position;
}