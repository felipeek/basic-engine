#version 330 core
layout (location = 0) in vec4 vertex_position;

out vec4 fragment_position;

uniform mat4 transform_matrix;

void main()
{
    fragment_position = vertex_position;  
    gl_Position = transform_matrix * vertex_position;
}