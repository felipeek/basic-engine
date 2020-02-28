#version 330 core

in vec4 fragmentPosition;
in vec4 fragmentNormal;
in vec2 fragmentTextureCoords;
in vec4 fragmentPosLightSpace;

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
uniform sampler2D shadowMap;
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

float shadowCalculation(vec4 fragmentPosLightSpace, vec4 normal, vec4 lightDir) {
	// perform perspective divide
    vec3 projCoords = fragmentPosLightSpace.xyz / fragmentPosLightSpace.w;
    // transform to [0,1] range
    projCoords = projCoords * 0.5 + 0.5;
	float currentDepth = projCoords.z;
	if (currentDepth > 1.0) return 0.0;

	float shadow = 0.0;
	vec2 texelSize = 1.0 / textureSize(shadowMap, 0);
	for (int x = -2; x <= 2; ++x)
	{
		for (int y = -2; y <= 2; ++y)
		{
			// get closest depth value from light's perspective (using [0,1] range fragPosLight as coords)
			float closestDepth = texture(shadowMap, projCoords.xy + vec2(x,y) * texelSize).r; 
			// get depth of current fragment from light's perspective
			// check whether current frag pos is in shadow
			float bias = max(0.01 * (1.0 - dot(normal, lightDir)), 0.005);  
			shadow += currentDepth - bias > closestDepth ? 1.0 : 0.0;
		}
	}

    return shadow / (5 * 5);
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

	//float shadow = shadowCalculation(fragmentPosLightSpace, normal, fragmentToPointLightVec);
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

	float shadow = shadowCalculation(fragmentPosLightSpace, normal, fragmentToPointLightVec);
	vec4 pointColor = pointAmbientColor + (1.0 - shadow) * pointDiffuseColor;// + pointSpecularColor;
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