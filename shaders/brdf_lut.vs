#version 330 core
layout (location = 0) in vec2 vertex_position;

out vec2 tex_coords;

void main()
{
    tex_coords = vertex_position.xy;
	if (tex_coords.x < 0.0) {
		tex_coords.x = 0.0;
	}
	if (tex_coords.y < 0.0) {
		tex_coords.y = 0.0;
	}
    gl_Position = vec4(vertex_position.xy, 0.0, 1.0);
}