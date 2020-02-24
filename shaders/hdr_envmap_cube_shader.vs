#version 330 core

layout (location = 0) in vec4 vertexPosition;

uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

out vec4 vPos;

void main()
{
	mat4 rotView = mat4(mat3(viewMatrix)); // remove translation from the view matrix
	gl_Position = (projectionMatrix * rotView * vertexPosition).xyww;
	vPos = vertexPosition;
}