#version 330 core

in vec4 fragmentPosition;
in vec4 fragmentNormal;
in vec2 fragmentTextureCoords;

#define POINT_LIGHT 0
#define DIRECTIONAL_LIGHT 1

// Light
struct Light
{
	int type;
	vec4 positionOrDirection;
	vec4 ambientColor;
	vec4 diffuseColor;
	vec4 specularColor;
};

// Normal Mapping
struct NormalMappingInfo
{
	bool useNormalMap;
	bool tangentSpace;	// @not implemented
	sampler2D normalMapTexture;
};

// Diffuse Info
struct DiffuseInfo
{
	bool useDiffuseMap;
	vec4 diffuseColor;
	sampler2D diffuseMap;
};

uniform mat4 modelMatrix;
uniform Light lights[16];
uniform int lightQuantity;
uniform NormalMappingInfo normalMappingInfo;
uniform vec4 cameraPosition;
uniform float objectShineness;
uniform DiffuseInfo diffuseInfo;
// Specular map will not be used (<1,1,1,1> assumed)

out vec4 finalColor;

vec4 getCorrectNormal()
{
	vec4 normal;

	if (normalMappingInfo.useNormalMap)
	{
		// Sample normal map (range [0, 1])
		normal = texture(normalMappingInfo.normalMapTexture, fragmentTextureCoords);
		// Transform normal vector to range [-1, 1]
		// normal = normal * 2.0 - 1.0;
		// W coordinate must be 0
		normal.w = 0;
		// Normalize normal
		normal = normalize(normal);

		vec3 normalv3 = mat3(inverse(transpose(modelMatrix))) * normal.xyz;
		normal = vec4(normalv3, 0);
		normal = normalize(normal);
	}
	else
		normal = normalize(fragmentNormal);

	return normal;
}

vec3 getPointColorOfLight(Light light)
{
	vec4 normal = getCorrectNormal();
	vec4 realDiffuseColor = diffuseInfo.useDiffuseMap ? texture(diffuseInfo.diffuseMap, fragmentTextureCoords) :
		diffuseInfo.diffuseColor;

	vec4 fragmentToPointLightVec = normalize(light.positionOrDirection - fragmentPosition);

	// Ambient Color
	vec4 pointAmbientColor = light.ambientColor * realDiffuseColor;

	// Diffuse Color
	float pointDiffuseContribution = max(0, dot(fragmentToPointLightVec, normal));
	vec4 pointDiffuseColor = pointDiffuseContribution * light.diffuseColor * realDiffuseColor;
	
	// Specular Color
	vec4 fragmentToCameraVec = normalize(cameraPosition - fragmentPosition);
	float pointSpecularContribution = pow(max(dot(fragmentToCameraVec, reflect(-fragmentToPointLightVec, normal)), 0.0), objectShineness);
	vec4 pointSpecularColor = pointSpecularContribution * light.specularColor * vec4(1.0, 1.0, 1.0, 1.0);

	// Attenuation
	float pointLightDistance = length(light.positionOrDirection - fragmentPosition);
	float pointAttenuation = 1.0 / (1.0 + 0.0014 * pointLightDistance +
		0.000007 * pointLightDistance * pointLightDistance);

	pointAmbientColor *= pointAttenuation;
	pointDiffuseColor *= pointAttenuation;
	pointSpecularColor *= pointAttenuation;

	vec4 pointColor = pointAmbientColor + pointDiffuseColor;// + pointSpecularColor;
	return pointColor.xyz;
}

vec3 getDirectionalColorOfLight(Light light)
{
	vec4 normal = getCorrectNormal();
	vec4 realDiffuseColor = diffuseInfo.useDiffuseMap ? texture(diffuseInfo.diffuseMap, fragmentTextureCoords) :
		diffuseInfo.diffuseColor;

	vec4 fragmentToPointLightVec = - normalize(light.positionOrDirection);

	// Ambient Color
	vec4 pointAmbientColor = light.ambientColor * realDiffuseColor;

	// Diffuse Color
	float pointDiffuseContribution = max(0, dot(fragmentToPointLightVec, normal));
	vec4 pointDiffuseColor = pointDiffuseContribution * light.diffuseColor * realDiffuseColor;
	
	// Specular Color
	vec4 fragmentToCameraVec = normalize(cameraPosition - fragmentPosition);
	float pointSpecularContribution = pow(max(dot(fragmentToCameraVec, reflect(-fragmentToPointLightVec, normal)), 0.0), objectShineness);
	vec4 pointSpecularColor = pointSpecularContribution * light.specularColor * vec4(1.0, 1.0, 1.0, 1.0);

	vec4 pointColor = pointAmbientColor + pointDiffuseColor;// + pointSpecularColor;
	return pointColor.xyz;
}

void main()
{
	finalColor = vec4(0.0, 0.0, 0.0, 0.0);

	for (int i = 0; i < lightQuantity; ++i)
	{
		Light light = lights[i];
		vec3 color = vec3(0.0, 0.0, 0.0);

		if (light.type == POINT_LIGHT) {
			finalColor.xyz += getPointColorOfLight(light);
		} else if (light.type == DIRECTIONAL_LIGHT) {
			finalColor.xyz += getDirectionalColorOfLight(light);
		}
	}
}