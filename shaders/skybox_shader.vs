#version 330 core

layout (location = 0) in vec3 vertexPosition;

out vec3 texCoords;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
	vec4 p = projectionMatrix * viewMatrix * vec4(vertexPosition, 1.0);
	gl_Position = p.xyww;
	texCoords = vertexPosition;
} 