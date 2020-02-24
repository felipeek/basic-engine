#include "hdr.h"
#include "graphics.h"
#include <math.h>
#include <stdlib.h>

#define CLAMP(V, X, Y) (((V) < (X)) ? (X) : ((V) > (Y)) ? (Y) : (V))

const Vec2 invAtan = (Vec2){0.1591f, 0.3183f};

Vec3 uvToDirection(Vec2 uv)
{
	uv.x -= 0.5f;
	uv.y -= 0.5f;
	uv.x /= invAtan.x;
	uv.y /= invAtan.y;

	return (Vec3) {
		cosf(uv.x) * cosf(uv.y),
		sinf(uv.y),
		sinf(uv.x) * cosf(uv.y)
	};
}

Vec2 sampleSphericalMap(Vec3 v)
{
    Vec2 uv = (Vec2){ atan2f(v.z, v.x), asinf(v.y) };
    uv.x *= invAtan.x;
    uv.y *= invAtan.y;
    uv.x += 0.5f;
    uv.y += 0.5f;
    return uv;
}

static Vec3 radianceFromTex(const FloatImageData* hdrEnvironmentMap, Vec3 direction)
{
	Vec2 uv = sampleSphericalMap(direction);
	float x = uv.x * hdrEnvironmentMap->width;
	float y = uv.y * hdrEnvironmentMap->height;

	int lowX = (int)floorf(x);
	int highX = (int)ceilf(x);
	int lowY = (int)floorf(y);
	int highY = (int)ceilf(y);
	float lowXWeight = CLAMP(x - (float)lowX, 0.0f, 1.0f);
	float lowYWeight = CLAMP(y - (float)lowY, 0.0f, 1.0f);

	if (highY == hdrEnvironmentMap->height) highY = lowY;
	if (highX == hdrEnvironmentMap->width) highX = lowX;

	Vec3 radiance = (Vec3){0.0f, 0.0f, 0.0f};
	Vec3 lowXlowYRadiance = *(Vec3*)&hdrEnvironmentMap->data[lowY * hdrEnvironmentMap->width * hdrEnvironmentMap->channels +
		lowX * hdrEnvironmentMap->channels];
	Vec3 lowXhighYRadiance = *(Vec3*)&hdrEnvironmentMap->data[highY * hdrEnvironmentMap->width * hdrEnvironmentMap->channels +
		lowX * hdrEnvironmentMap->channels];
	Vec3 highXlowYRadiance = *(Vec3*)&hdrEnvironmentMap->data[lowY * hdrEnvironmentMap->width * hdrEnvironmentMap->channels +
		highX * hdrEnvironmentMap->channels];
	Vec3 highXhighYRadiance = *(Vec3*)&hdrEnvironmentMap->data[highY * hdrEnvironmentMap->width * hdrEnvironmentMap->channels +
		highX * hdrEnvironmentMap->channels];

	radiance = gmAddVec3(radiance, gmScalarProductVec3(lowXWeight * lowYWeight, lowXlowYRadiance));
	radiance = gmAddVec3(radiance, gmScalarProductVec3(lowXWeight * (1.0f - lowYWeight), lowXhighYRadiance));
	radiance = gmAddVec3(radiance, gmScalarProductVec3((1.0f - lowXWeight) * lowYWeight, highXlowYRadiance));
	radiance = gmAddVec3(radiance, gmScalarProductVec3((1.0f - lowXWeight) * (1.0f - lowYWeight), highXhighYRadiance));

	return radiance;
}

static Vec3 calculateIrradiance(Vec3 normal, const FloatImageData* hdrEnvironmentMap)
{
	Vec3 irradiance = (Vec3){0.0f, 0.0f, 0.0f};  
	Vec3 up    = (Vec3){0.0f, 1.0f, 0.0f};
	Vec3 right = gmCrossProduct(up, normal);
	up         = gmCrossProduct(normal, right);

	float sampleDelta = 0.025f;
	float nrSamples = 0.0f; 
	for (float phi = 0.0f; phi < 2.0f * PI_F; phi += sampleDelta)
	{
		for (float theta = 0.0f; theta < 0.5f * PI_F; theta += sampleDelta)
		{
			// spherical to cartesian (in tangent space)
			Vec3 tangentSample = (Vec3){sinf(theta) * cosf(phi),  sinf(theta) * sinf(phi), cosf(theta)};
			// tangent space to world
			Vec3 sampleVec = gmAddVec3(gmScalarProductVec3(tangentSample.x, right), gmScalarProductVec3(tangentSample.y, up));
			sampleVec = gmAddVec3(sampleVec, gmScalarProductVec3(tangentSample.z, normal));

			Vec3 radiance = radianceFromTex(hdrEnvironmentMap, sampleVec);
			radiance = gmScalarProductVec3(cosf(theta), radiance);
			radiance = gmScalarProductVec3(sinf(theta), radiance);

			irradiance = gmAddVec3(irradiance, radiance);
			nrSamples++;
		}
	}

	irradiance = gmScalarProductVec3(PI_F * (1.0f / nrSamples), irradiance);
	return irradiance;
}

extern FloatImageData hdrFromEnvironmentMapToIrradianceMap(const FloatImageData* hdrEnvironmentMap)
{
	FloatImageData fid;

	fid.data = malloc(hdrEnvironmentMap->width * hdrEnvironmentMap->height * hdrEnvironmentMap->channels * sizeof(r32));
	fid.height = hdrEnvironmentMap->height;
	fid.width = hdrEnvironmentMap->width;
	fid.channels = hdrEnvironmentMap->channels;

	for (u32 i = 0; i < fid.height; ++i) {
		printf("Running %d of %d\n", i, fid.height);
		for (u32 j = 0; j < fid.width; ++j) {
			Vec2 correspondingUV;
			correspondingUV.x = (r32)j / (r32)fid.width;
			correspondingUV.y = (r32)i / (r32)fid.height;
			Vec3 n = uvToDirection(correspondingUV);
			n = gmNormalizeVec3(n);
			Vec2 uvSampled = sampleSphericalMap(n);

			Vec3 irradiance = calculateIrradiance(n, hdrEnvironmentMap);
			fid.data[i * hdrEnvironmentMap->width * hdrEnvironmentMap->channels + j * hdrEnvironmentMap->channels] = irradiance.r;
			fid.data[i * hdrEnvironmentMap->width * hdrEnvironmentMap->channels + j * hdrEnvironmentMap->channels + 1] = irradiance.g;
			fid.data[i * hdrEnvironmentMap->width * hdrEnvironmentMap->channels + j * hdrEnvironmentMap->channels + 2] = irradiance.b;
		}
	}

	return fid;
}