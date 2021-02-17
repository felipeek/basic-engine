#version 330 core
layout (location = 0) in vec3 vertex_position;

out vec3 fragment_position;

uniform mat4 transform_matrix;

void main()
{
    fragment_position = vertex_position;  
    gl_Position = transform_matrix * vec4(vertex_position, 1.0);
}