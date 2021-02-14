#version 330 core

layout (location = 0) in vec3 vertex_position;

out vec3 tex_coords;

uniform mat4 view_matrix;
uniform mat4 projection_matrix;

void main()
{
	vec4 p = projection_matrix * view_matrix * vec4(vertex_position, 1.0);
	gl_Position = p.xyww;
	tex_coords = vertex_position;
} 