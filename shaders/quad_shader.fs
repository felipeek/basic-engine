#version 330 core

in vec2 texCoords;
out vec4 finalColor;

uniform sampler2D screenTexture;

void main()
{
	finalColor = texture(screenTexture, texCoords);
	//finalColor = vec4(1.0,0.0,0.0,1.0);
}