#version 330 core

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec4 vertexNormal;
layout (location = 2) in vec2 vertexTextureCoords;

out vec4 gsFragmentPosition;
out vec4 gsFragmentNormal;
out vec2 gsFragmentTextureCoords;
out vec4 gsFragmentNormalClipCoords;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

void main()
{
	vec3 normal3D = mat3(inverse(transpose(modelMatrix))) * vertexNormal.xyz;
	gsFragmentNormal = vec4(normal3D, 0.0);
	gsFragmentTextureCoords = vertexTextureCoords;
	gsFragmentPosition = modelMatrix * vertexPosition;
	gl_Position = projectionMatrix * viewMatrix * modelMatrix * vertexPosition;
	gsFragmentNormalClipCoords = projectionMatrix * viewMatrix * gsFragmentNormal;
}