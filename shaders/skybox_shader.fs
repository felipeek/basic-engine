#version 330 core

in vec3 texCoords;
out vec4 finalColor;

uniform samplerCube cubeTexture;

void main()
{
	finalColor = texture(cubeTexture, texCoords);
	//finalColor = vec4(1.0,0.0,0.0,1.0);
}