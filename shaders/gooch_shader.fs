#version 330 core

in vec4 fragmentPosition;
in vec4 fragmentNormal;
in vec2 fragmentTextureCoords;

out vec4 finalColor;

uniform vec4 lightColor;

// World Coords
uniform vec4 lightPosition;
uniform vec4 cameraPosition;

void main()
{
	vec4 normal = normalize(fragmentNormal);

	vec3 surfaceColor = vec3(0.0, 1.0, 0.0);
	vec3 coolColor = vec3(0.0, 0.0, 0.55) + 0.25 * surfaceColor;
	vec3 warmColor = vec3(0.3, 0.3, 0.0) + 0.25 * surfaceColor;
	vec3 highLightColor = vec3(2.0, 2.0, 2.0);

	vec4 lightDirection = normalize(lightPosition - fragmentPosition);
	//vec4 r = 2.0 * dot(normal, lightDirection) * (normal - lightDirection);
	vec4 r = reflect(-lightDirection, normal);
	vec4 v = normalize(cameraPosition - fragmentPosition);
	float s = clamp((100.0 * dot(r, v) - 97.0), 0.0, 1.0);

	vec3 unlitColor = 0.5 * coolColor;
	vec3 litColor = s * highLightColor + (1 - s) * warmColor;

	float nL = clamp(dot(normal, lightDirection), 0.0, 1.0);
	vec3 finalColorV3 = unlitColor + nL * lightColor.xyz * litColor;
	
	finalColor = vec4(finalColorV3.x, finalColorV3.y, finalColorV3.z, 1.0);
	//finalColor = vec4(litColor.xyz, 1.0);
}