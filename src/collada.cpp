#define HASH_TABLE_IMPLEMENT
#define LIGHT_ARENA_IMPLEMENT
#include <hash_table.h>
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <iostream>
#include <dynamic_array.h>
#include "graphics.hpp"
#include "animation.hpp"

static void fetchJoints(aiNode* rootBoneNode, Joint* parentJoint, Hash_Table htBoneNameToIndex) {
	s32 parentJointID = *(s32*)hash_table_get(&htBoneNameToIndex, rootBoneNode->mName.C_Str());
	for (unsigned int i = 0; i < rootBoneNode->mNumChildren; ++i) {
		aiNode* child = rootBoneNode->mChildren[i];
		s32 childJointID = *(s32*)hash_table_get(&htBoneNameToIndex, child->mName.C_Str());
		Joint childJoint;
		childJoint.children = array_create(Joint, 1);
		childJoint.boneID = childJointID;

		Mat4 childMatrix = *(Mat4*)&child->mTransformation;
		//childMatrix = gmTransposeMat4(&childMatrix);
		childJoint.bindModelSpaceToBoneSpaceTransform = gmMultiplyMat4(&parentJoint->bindModelSpaceToBoneSpaceTransform, &childMatrix);

		fetchJoints(child, &childJoint, htBoneNameToIndex);

		Mat4 invertedMatrix;
		boolean inverted = gmInverseMat4(&childJoint.bindModelSpaceToBoneSpaceTransform, &invertedMatrix);
		assert(inverted == true);
		childJoint.bindModelSpaceToBoneSpaceTransform = invertedMatrix;

		array_push(parentJoint->children, &childJoint);
	}
}

static aiNode* findRootBone(aiNode* rootNode, aiString* boneNames) {
	for (unsigned int i = 0; i < rootNode->mNumChildren; ++i) {
		aiNode* child = rootNode->mChildren[i];
		for (unsigned int j = 0; j < array_get_length(boneNames); ++j) {
			aiString currentBoneName = boneNames[j];
			if (currentBoneName == child->mName) {
				printf("found root bone! it is: %s\n", currentBoneName.C_Str());
				return child;
			}
		}

		aiNode* childrenResult = findRootBone(child, boneNames);
		if (findRootBone(child, boneNames) != NULL) {
			return childrenResult;
		}
	}

	return NULL;
}

static void processMesh(aiMesh* mesh, const aiScene* scene, AnimatedVertex** _vertices, unsigned int** _indices, Animation* _animation,
	Joint* _rootJoint)
{
	AnimatedVertex* vertices = array_create(AnimatedVertex, 100);
	unsigned int* indices = array_create(unsigned int, 100);

	// Fill Vertices
	for (unsigned int i = 0; i < mesh->mNumVertices; ++i)
	{
		AnimatedVertex vertex;

		vertex.position.x = mesh->mVertices[i].x;
		vertex.position.y = mesh->mVertices[i].y;
		vertex.position.z = mesh->mVertices[i].z;
		vertex.position.w = 1.0f;

		vertex.normal.x = mesh->mNormals[i].x;
		vertex.normal.y = mesh->mNormals[i].y;
		vertex.normal.z = mesh->mNormals[i].z;
		vertex.normal.w = 0.0f;

		/*
		if (mesh->mTangents)
		{
			vertex.tangent.x = mesh->mTangents[i].x;
			vertex.tangent.y = mesh->mTangents[i].y;
			vertex.tangent.z = mesh->mTangents[i].z;
			vertex.tangent.w = 0.0f;
		}
		*/
		
		if (mesh->mTextureCoords[0])
		{
			vertex.textureCoordinates.x = mesh->mTextureCoords[0][i].x;
			vertex.textureCoordinates.y = mesh->mTextureCoords[0][i].y;
		}
		else
			vertex.textureCoordinates = (Vec2){0.0f, 0.0f};

		vertex.boneIDs = (DiscreteVec3){0,0,0};
		vertex.boneWeights = (Vec3){0.0f, 0.0f, 0.0f};

		array_push(vertices, &vertex);
	}

	Hash_Table htBoneNameToIndex = hash_table_create(1000, sizeof(s32));

	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		aiBone* currentBone = mesh->mBones[i];
		hash_table_insert(&htBoneNameToIndex, mesh->mBones[i]->mName.C_Str(), &i);
		printf("%d. %s\n", i, mesh->mBones[i]->mName.C_Str());
	}

	aiString* boneNames = array_create(aiString, 10);

	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		aiBone* currentBone = mesh->mBones[i];
		s32 boneID = *(s32*)hash_table_get(&htBoneNameToIndex, currentBone->mName.C_Str());
		array_push(boneNames, &currentBone->mName);
		aiMatrix4x4 boneMatrix = currentBone->mOffsetMatrix;
		assert(boneID < mesh->mNumBones && boneID >= 0);
		for (unsigned int j = 0; j < currentBone->mNumWeights; ++j) {
			aiVertexWeight vertexWeight = currentBone->mWeights[j];
			s32 vertexIndex = vertexWeight.mVertexId;
			AnimatedVertex* animatedVertex = &vertices[vertexIndex];

			// @TODO: don't discard excess...
			if (animatedVertex->boneWeights.x == 0.0f) {
				animatedVertex->boneWeights.x = vertexWeight.mWeight;
				animatedVertex->boneIDs.x = boneID;
			} else if (animatedVertex->boneWeights.y == 0.0f) {
				animatedVertex->boneWeights.y = vertexWeight.mWeight;
				animatedVertex->boneIDs.y = boneID;
			} else if (animatedVertex->boneWeights.z == 0.0f) {
				animatedVertex->boneWeights.z = vertexWeight.mWeight;
				animatedVertex->boneIDs.z = boneID;
			}
		}
	}

	// normalize weights
	for (unsigned int i = 0; i < array_get_length(vertices); ++i) {
		AnimatedVertex* animatedVertex = &vertices[i];
		float sum = animatedVertex->boneWeights.x + animatedVertex->boneWeights.y + animatedVertex->boneWeights.z;
		float k = 1.0f / sum;
		animatedVertex->boneWeights = gmScalarProductVec3(k, animatedVertex->boneWeights);
		//printf("GOT: <%.3f, %.3f, %.3f>\n", animatedVertex->boneWeights.x, animatedVertex->boneWeights.y, animatedVertex->boneWeights.z);
		//printf("with BoneID: <%d, %d, %d>\n", animatedVertex->boneIDs.x, animatedVertex->boneIDs.y, animatedVertex->boneIDs.z);
	}

	Joint rootJoint;
	aiNode* rootBoneNode = findRootBone(scene->mRootNode, boneNames);
	rootJoint.bindModelSpaceToBoneSpaceTransform = *(Mat4*)&scene->mRootNode->mChildren[1]->mChildren[0]->mTransformation;

	Vec4 test = (Vec4){0.0f, 0.0f, 0.0f, 1.0f};
	Vec4 result = gmMultiplyMat4AndVec4(&rootJoint.bindModelSpaceToBoneSpaceTransform, test);

	//rootJoint.bindModelSpaceToBoneSpaceTransform = gmTransposeMat4(&(rootJoint.jointSpaceToModelSpaceTransform));
	rootJoint.boneID = 0;
	rootJoint.children = array_create(Joint, 1);
	
	Mat4 CORRECTION = (Mat4){
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, -1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	//rootJoint.bindModelSpaceToBoneSpaceTransform = gmMultiplyMat4(&CORRECTION, &(rootJoint.bindModelSpaceToBoneSpaceTransform));

	test = (Vec4){0.0f, 0.0f, 0.0f, 1.0f};
	result = gmMultiplyMat4AndVec4(&rootJoint.bindModelSpaceToBoneSpaceTransform, test);

	// ADJUST JOINT MATRICES
	fetchJoints(rootBoneNode, &rootJoint, htBoneNameToIndex);

	Mat4 invertedMatrix;
	boolean inverted = gmInverseMat4(&(rootJoint.bindModelSpaceToBoneSpaceTransform), &invertedMatrix);
	assert(inverted == true);
	rootJoint.bindModelSpaceToBoneSpaceTransform = invertedMatrix;

	test = (Vec4){0.0f, 0.0f, 0.0f, 1.0f};
	result = gmMultiplyMat4AndVec4(&rootJoint.bindModelSpaceToBoneSpaceTransform, test);

	*_rootJoint = rootJoint;

	Animation animation;
	animationCreate(&animation);

	// only consider first animation for now!
	if (scene->mNumAnimations > 0)
	{
		aiAnimation* aiAnimation = scene->mAnimations[0];
		double aiAnimationDuration = scene->mAnimations[0]->mDuration;
		
		for (unsigned int j = 0; j < aiAnimation->mNumChannels; ++j) {
			aiNodeAnim* meshAnim = aiAnimation->mChannels[j];

			// FOR NOW, THIS IS A VERY SIMPLE IMPLEMENTATION THAT CONSIDERS THAT POSITIONS AND ROTATIONS
			// WILL ALWAYS HAVE THE SAME VEC_SIZE AND SHARE THE SAME mTime.
			for (unsigned k = 0; k < meshAnim->mNumPositionKeys; ++k) {
				aiVectorKey positionVK = meshAnim->mPositionKeys[k];
				aiQuatKey rotationVK = meshAnim->mRotationKeys[k];

				KeyFrame* currentKeyFrame;
				if (!(currentKeyFrame = animationGetKeyFrame(animation, positionVK.mTime))) {
					KeyFrame keyFrame;
					keyFrameCreate(&keyFrame, positionVK.mTime);
					animationAddKeyFrame(&animation, keyFrame);
					assert((currentKeyFrame = animationGetKeyFrame(animation, positionVK.mTime)) != NULL);
				}
				BonePosition bp;
   				bp.boneID = *(s32*)hash_table_get(&htBoneNameToIndex, meshAnim->mNodeName.C_Str());
				bp.position = (Vec3){positionVK.mValue.x, positionVK.mValue.y, positionVK.mValue.z};
				bp.rotation = (Quaternion){rotationVK.mValue.x, rotationVK.mValue.y, rotationVK.mValue.z, rotationVK.mValue.w};
				keyFrameAddBonePosition(currentKeyFrame, bp);
			}
		}
	}

	*_animation = animation;

	// Fill Indices
	for (unsigned int i = 0; i < mesh->mNumFaces; ++i)
		for (unsigned int j = 0; j < mesh->mFaces[i].mNumIndices; ++j)
			array_push(indices, &mesh->mFaces[i].mIndices[j]);

	// Fill Material
	/* if (mesh->mMaterialIndex >= 0)
	{
		aiMaterial* material = scene->mMaterials[mesh->mMaterialIndex];
		diffuseMap = this->loadMaterialTexture(material, aiTextureType_DIFFUSE, directory);
		specularMap = this->loadMaterialTexture(material, aiTextureType_SPECULAR, directory);
		normalMap = this->loadMaterialTexture(material, aiTextureType_HEIGHT, directory);
	} */

	*_vertices = vertices;
	*_indices = indices;
}

void processNode(aiNode* node, const aiScene* scene, AnimatedVertex** verticesMatrix, unsigned int** indicesMatrix, Animation* animation,
	Joint* rootJoint)
{
	for (unsigned int i = 0; i < node->mNumMeshes; ++i)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		AnimatedVertex* vertices;
		unsigned int* indices;
		processMesh(mesh, scene, &vertices, &indices, animation, rootJoint);
		array_push(verticesMatrix, &vertices);
		array_push(indicesMatrix, &indices);
	}

	for (unsigned int i = 0; i < node->mNumChildren; ++i)
	{
		processNode(node->mChildren[i], scene, verticesMatrix, indicesMatrix, animation, rootJoint);
	}
}

int colladaLoad(const s8* path, AnimatedVertex*** verticesMatrix, unsigned int*** indicesMatrix, Animation* animation, Joint* rootJoint)
{
	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate | /*aiProcess_FlipUVs |*/
		aiProcess_GenSmoothNormals | aiProcess_CalcTangentSpace);

	if (!scene || scene->mFlags & AI_SCENE_FLAGS_INCOMPLETE || !scene->mRootNode)
	{
		std::cout << "Error loading model with path " << path << ":";
		std::cout << "Error: " << importer.GetErrorString() << std::endl;
		return -1;
	}

	aiNode* node = scene->mRootNode;

	*verticesMatrix = array_create(AnimatedVertex*, 1);
	*indicesMatrix = array_create(unsigned int*, 1);

	processNode(node, scene, *verticesMatrix, *indicesMatrix, animation, rootJoint);

	return 0;
}
