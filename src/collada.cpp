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

void processMesh(aiMesh* mesh, const aiScene* scene, AnimatedVertex** _vertices, unsigned int** _indices, Animation* _animation)
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
	}

	for (unsigned int i = 0; i < mesh->mNumBones; ++i)
	{
		aiBone* currentBone = mesh->mBones[i];
		s32 boneID = *(s32*)hash_table_get(&htBoneNameToIndex, currentBone->mName.C_Str());
		for (unsigned int j = 0; j < currentBone->mNumWeights; ++j) {
			aiVertexWeight vertexWeight = currentBone->mWeights[j];
			s32 vertexIndex = vertexWeight.mVertexId;
			AnimatedVertex* animatedVertex = &vertices[vertexIndex];

			if (animatedVertex->boneWeights.x < vertexWeight.mWeight) {
				animatedVertex->boneWeights.x = vertexWeight.mWeight;
				animatedVertex->boneIDs.x = boneID;
			} else if (animatedVertex->boneWeights.y < vertexWeight.mWeight) {
				animatedVertex->boneWeights.y = vertexWeight.mWeight;
				animatedVertex->boneIDs.y = boneID;
			} else if (animatedVertex->boneWeights.z < vertexWeight.mWeight) {
				animatedVertex->boneWeights.z = vertexWeight.mWeight;
				animatedVertex->boneIDs.z = boneID;
			}
		}
	}

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
				bp.rotation = quaternion_new((Vec3){rotationVK.mValue.x, rotationVK.mValue.y,
					rotationVK.mValue.z}, rotationVK.mValue.w);
				keyFrameAddBonePosition(currentKeyFrame, bp);
			}
		}
	}

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

void processNode(aiNode* node, const aiScene* scene, AnimatedVertex** verticesMatrix, unsigned int** indicesMatrix)
{
	for (unsigned int i = 0; i < node->mNumMeshes; ++i)
	{
		aiMesh* mesh = scene->mMeshes[node->mMeshes[i]];
		AnimatedVertex* vertices;
		unsigned int* indices;
		Animation animation;
		processMesh(mesh, scene, &vertices, &indices, &animation);
		array_push(verticesMatrix, &vertices);
		array_push(indicesMatrix, &indices);
	}

	for (unsigned int i = 0; i < node->mNumChildren; ++i)
	{
		processNode(node->mChildren[i], scene, verticesMatrix, indicesMatrix);
	}
}

int colladaLoad(const s8* path, AnimatedVertex*** verticesMatrix, unsigned int*** indicesMatrix)
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

	processNode(node, scene, *verticesMatrix, *indicesMatrix);

	return 0;
}
