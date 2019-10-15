#version 330 core

in vec4 fragmentPosition;
in vec4 fragmentNormal;
in vec2 fragmentTextureCoords;

out vec4 finalColor;

uniform vec4 cameraPosition;
uniform samplerCube skybox;

void main()
{
	// MIRROR EFFECT
	//vec3 I = normalize(fragmentPosition - cameraPosition).xyz;
	//vec3 R = reflect(I, normalize(fragmentNormal).xyz);
	//finalColor = vec4(texture(skybox, R).rgb, 1.0);

	// GLASS EFFECT
	float ratio = 1.00 / 1.20;
	vec3 I = normalize(fragmentPosition - cameraPosition).xyz;
	vec3 R = refract(I, normalize(fragmentNormal).xyz, ratio);
	finalColor = vec4(texture(skybox, R).rgb, 1.0);
}