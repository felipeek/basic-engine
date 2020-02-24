#version 330 core

in vec4 fragmentPosition;
in vec4 fragmentNormal;
in vec2 fragmentTextureCoords;

out vec4 finalColor;

uniform vec4 cameraPosition;
uniform int tweak;

const float PI_F = 3.14159265359;

// Albedo Info
struct AlbedoInfo
{
	bool useAlbedoMap;
	vec3 albedoColor;
	sampler2D albedoMap;
};

// Metallic Info
struct MetallicInfo
{
	bool useMetallicMap;
	float metallic;
	sampler2D metallicMap;
};

// Roughness Map
struct RoughnessInfo
{
	bool useRoughnessMap;
	float roughness;
	sampler2D roughnessMap;
};

// PBRLight
struct PBRLight
{
	vec3 position;
	vec3 color;
};

uniform AlbedoInfo albedoInfo;
uniform MetallicInfo metallicInfo;
uniform RoughnessInfo roughnessInfo;
uniform PBRLight lights[16];
uniform int lightQuantity;

uniform sampler2D irradianceMap;

const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 sampleSphericalMap(vec4 v)
{
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= invAtan;
    uv += 0.5;
    return uv;
}

// GGX implementation [NDF]
float ndfGGX(vec3 N, vec3 H, float roughness)
{
	float a = roughness * roughness;
	//float a = roughness;
	float a2 = a * a;
	float nDotH = max(dot(N, H), 0.0);
	float nDotH2 = nDotH * nDotH;
	float num = a2;
	float denom = (nDotH2 * (a2 - 1) + 1);
	denom = PI_F * denom * denom;
	return num / denom;
}

float GeometrySchlickGGX(vec3 N, vec3 V, float roughness)
{
	float NdotV = max(dot(N, V), 0.0);
	float r = (roughness + 1.0);
    float k = (r*r) / 8.0;

    float num   = NdotV;
    float denom = NdotV * (1.0 - k) + k;
	
    return num / denom;
}

float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness) {
	float gV = GeometrySchlickGGX(N, V, roughness);
	float gL = GeometrySchlickGGX(N, L, roughness);
	return gV*gL;
}

vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
    return F0 + (1.0 - F0) * pow(1.0 - min(cosTheta, 1.0), 5.0);
}

vec3 shirleyDiffuseTerm(vec3 albedo, vec3 N, vec3 L, vec3 V) {
	float NdotL = max(dot(N, L), 0.0);
	float NdotV = max(dot(N, V), 0.0);

	return (21.0 / 20.0) * albedo * (1.0 - pow((1.0 - NdotL), 5)) * (1.0 - pow((1.0 - NdotV), 5));
}

vec3 fresnelSchlickRoughness(float cosTheta, vec3 F0, float roughness)
{
    return F0 + (max(vec3(1.0 - roughness), F0) - F0) * pow(1.0 - cosTheta, 5.0);
}

void main()
{
	vec3 albedo = albedoInfo.albedoColor;
	float metallic = metallicInfo.metallic;
	float roughness = roughnessInfo.roughness;

	if (albedoInfo.useAlbedoMap) {
		albedo = pow(texture(albedoInfo.albedoMap, fragmentTextureCoords).rgb, vec3(2.2));
	}

	if (metallicInfo.useMetallicMap) {
		metallic = texture(metallicInfo.metallicMap, fragmentTextureCoords).r;
	}

	if (roughnessInfo.useRoughnessMap) {
		roughness = texture(roughnessInfo.roughnessMap, fragmentTextureCoords).r;
	}

	metallic = 0.0;
	albedo = vec3(1.0,1.0,1.0);
	roughness = 0.8;

	float ao = 1.0;

	vec3 N = normalize(fragmentNormal.xyz);
	vec3 V = normalize(cameraPosition.xyz - fragmentPosition.xyz);

	vec3 F0 = vec3(0.04); 
    F0 = mix(F0, albedo, metallic);

	vec3 Lo = vec3(0.0);

	for (int i = 0; i < lightQuantity; ++i) {
		
		PBRLight l = lights[i];
		// Li Radiance
		vec3 L = normalize(l.position - fragmentPosition.xyz);
		vec3 H = normalize(V + L);

		float distance = length(l.position - fragmentPosition.xyz);
		float attenuation = 1.0 / (distance * distance);

		vec3 radiance = l.color * attenuation;

		float G = GeometrySmith(N, V, L, roughness);
		float NDF = ndfGGX(N, H, roughness);
		vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);

		vec3 kS = F;
		vec3 kD = vec3(1.0) - F;
		kD *= 1.0 - metallic;

		vec3 numerator = NDF * G * F;
		float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
		vec3 specular = numerator / max(denominator, 0.001);

		//vec3 dTerm = tweak != 0 ? shirleyDiffuseTerm(albedo, N, L, V) : albedo;
		vec3 dTerm = albedo;//shirleyDiffuseTerm(albedo, N, L, V);

		float NdotL = max(dot(N, L), 0.0);
		Lo += (kD * dTerm / PI_F + specular) * radiance * NdotL;
	}


	vec3 kS = fresnelSchlickRoughness(max(dot(N, V), 0.0), F0, roughness);
	vec3 kD = 1.0 - kS;
	vec2 sampleUV = sampleSphericalMap(vec4(N.x, N.y, N.z, 0.0));
	vec3 irradiance = texture(irradianceMap, sampleUV).rgb;
	vec3 diffuse    = irradiance * albedo;
	vec3 ambient    = tweak != 0 ? (kD * diffuse) * ao : vec3(0,0,0);
    vec3 color = ambient + Lo;
	
    color = color / (color + vec3(1.0));
    color = pow(color, vec3(1.0/2.2));
	
	finalColor = vec4(color, 1.0);
}