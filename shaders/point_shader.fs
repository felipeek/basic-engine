#version 330 core

in vec4 gColor;
out vec4 finalColor;

void main()
{
	//finalColor = vec4(1.0, 1.0, 1.0, 1.0);
	finalColor = gColor;
}