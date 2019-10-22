#version 330 core

layout (location = 0) in vec2 vertexPosition;

out vec2 fragmentTexCoords;

void main()
{
	gl_Position = vec4(vertexPosition.x, vertexPosition.y, 0.0, 1.0);
	fragmentTexCoords = vec2(vertexPosition.x < 0.0 ? 0.0 : 1.0, vertexPosition.y < 0.0 ? 0.0 : 1.0);
}