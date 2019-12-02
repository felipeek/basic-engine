#version 330 core

#define MAX_JOINTS 20

layout (location = 0) in vec4 vertexPosition;
layout (location = 1) in vec4 vertexNormal;
layout (location = 2) in vec2 vertexTextureCoords;
layout (location = 3) in vec3 boneWeights;
layout (location = 4) in ivec3 boneIDs;

out vec4 fragmentPosition;
out vec4 fragmentNormal;
out vec2 fragmentTextureCoords;

uniform mat4 modelMatrix;
uniform mat4 viewMatrix;
uniform mat4 projectionMatrix;

uniform mat4 joints[MAX_JOINTS];
uniform int jointsNum;

void main()
{
	vec3 normal3D = mat3(inverse(transpose(modelMatrix))) * vertexNormal.xyz;
	fragmentNormal = vec4(normal3D, 0.0);
	fragmentTextureCoords = vertexTextureCoords;
	fragmentPosition = modelMatrix * vertexPosition;

	vec4 totalLocalPos = vec4(0.0);

	for (int i = 0; i < 3; ++i) {
		if (boneIDs[i] != 0) {
			totalLocalPos += vertexPosition * boneWeights[i];
		} else {
			mat4 jointTransform = joints[boneIDs[i]];
			vec4 posePosition = jointTransform * vertexPosition;
			totalLocalPos += posePosition * boneWeights[i];
		}
	}

	gl_Position = projectionMatrix * viewMatrix * modelMatrix * totalLocalPos;
}