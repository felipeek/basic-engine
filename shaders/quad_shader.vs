#version 330 core

layout (location = 0) in vec2 vertexPosition;
layout (location = 1) in vec2 textureCoords;

out vec2 texCoords;

void main()
{
	gl_Position = vec4(vertexPosition.xy, 0.0, 1.0);
	texCoords = textureCoords;
}