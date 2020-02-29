#version 330 core

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec4 vertexNormal;
layout (location = 2) in vec2 textureCoords;

uniform mat4 modelMatrix;

void main()
{
	gl_Position = modelMatrix * vertexPosition;
}