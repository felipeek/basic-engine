#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in vec4 gsFragmentPosition[];
in vec4 gsFragmentNormal[];
in vec2 gsFragmentTextureCoords[];

in vec4 gsFragmentNormalClipCoords[];

out vec4 fragmentPosition;
out vec4 fragmentNormal;
out vec2 fragmentTextureCoords;

uniform float time;

vec4 getTriangleNormal()
{
   vec4 a = gsFragmentNormalClipCoords[0];
   vec4 b = gsFragmentNormalClipCoords[1];
   vec4 c = gsFragmentNormalClipCoords[2];
   return normalize((a + b + c) / 3.0);
}  

vec4 explosionContribution(vec4 normal) {
	float EXPLOSION_MAG = 1.3;
	return pow(((sin(time) + 1.0) / 2.0), EXPLOSION_MAG) * normal;
}

void main() {
	vec4 normal = getTriangleNormal();

	gl_Position = gl_in[0].gl_Position + explosionContribution(normal);
	fragmentPosition = gsFragmentPosition[0];
	fragmentNormal = gsFragmentNormal[0];
	fragmentTextureCoords = gsFragmentTextureCoords[0];
	EmitVertex();

	gl_Position = gl_in[1].gl_Position + explosionContribution(normal);
	fragmentPosition = gsFragmentPosition[1];
	fragmentNormal = gsFragmentNormal[1];
	fragmentTextureCoords = gsFragmentTextureCoords[1];
	EmitVertex();

	gl_Position = gl_in[2].gl_Position + explosionContribution(normal);
	fragmentPosition = gsFragmentPosition[2];
	fragmentNormal = gsFragmentNormal[2];
	fragmentTextureCoords = gsFragmentTextureCoords[2];
	EmitVertex();

	EndPrimitive();
}