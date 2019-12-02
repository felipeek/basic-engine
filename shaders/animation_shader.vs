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
	fragmentTextureCoords = vertexTextureCoords;

	vec4 totalLocalPos = vec4(0.0);
	vec4 totalNormal = vec4(0.0);

	for (int i = 0; i < 3; ++i) {
		//if (boneIDs[i] == 6) {
			mat4 jointTransform = joints[boneIDs[i]];
			vec4 posePosition = jointTransform * vec4(vertexPosition.xyz, 1.0);
			totalLocalPos += posePosition * boneWeights[i];

			vec4 worldNormal = jointTransform * vec4(vertexNormal.xyz, 0.0);
			totalNormal += worldNormal * boneWeights[i];
		//} else {
		//	totalLocalPos += vertexPosition * boneWeights[i];
		//	vec4 worldNormal = vertexPosition * vec4(vertexNormal.xyz, 0.0);
		//	totalNormal += worldNormal * boneWeights[i];
		//}
	}

	vec3 normal3D = mat3(inverse(transpose(modelMatrix))) * totalNormal.xyz;
	fragmentNormal = vec4(normal3D, 0.0);
	gl_Position = projectionMatrix * viewMatrix * modelMatrix * totalLocalPos;
	fragmentPosition = modelMatrix * totalLocalPos;
}