#version 330 core

in vec4 vPos;

out vec4 finalColor;

uniform sampler2D equirectangularMap;

const vec2 invAtan = vec2(0.1591, 0.3183);
vec2 sampleSphericalMap(vec4 v)
{
    vec2 uv = vec2(atan(v.z, v.x), asin(v.y));
    uv *= invAtan;
    uv += 0.5;
    return uv;
}

void main()
{
	vec2 uv = sampleSphericalMap(normalize(vPos));
	vec3 color = texture(equirectangularMap, uv).rgb;
	finalColor = vec4(color, 1.0);
}