#version 330 core

in vec3 texCoords;
out vec4 finalColor;

uniform samplerCube cubeTexture;

void main()
{
	vec4 lol = texture(cubeTexture, texCoords);
	finalColor = vec4(lol.rrr, 1.0);
} 