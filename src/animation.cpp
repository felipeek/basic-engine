#include "animation.hpp"
#include <assert.h>
#include <dynamic_array.h>
#include <GLFW/glfw3.h>

void animationCreate(Animation* animation)
{
	*animation = array_create(KeyFrame, 10);
}

void animationDestroy(Animation animation)
{
	array_release(animation);
}

void animationAddKeyFrame(Animation* animation, KeyFrame keyFrame)
{
	array_push(*animation, &keyFrame);
}

KeyFrame* animationGetKeyFrame(const Animation animation, double time)
{
	for (s32 i = 0; i < array_get_length(animation); ++i)
	{
		double aTime = animation[i].time;
		if (aTime == time)
			return &animation[i];
	}

	return NULL;
}

void keyFrameCreate(KeyFrame* keyFrame, double time)
{
	keyFrame->bonePositions = array_create(BonePosition, 10);
	keyFrame->time = time;
}

void keyFrameDestroy(KeyFrame keyFrame)
{
	array_release(keyFrame.bonePositions);
}

void keyFrameAddBonePosition(KeyFrame* keyFrame, BonePosition bonePosition)
{
	array_push(keyFrame->bonePositions, &bonePosition);
}

BonePosition* keyFrameGetBonePosition(const KeyFrame keyFrame, int boneID)
{
	for (s32 i = 0; i < array_get_length(keyFrame.bonePositions); ++i)
	{
		BonePosition bonePosition = keyFrame.bonePositions[i];
		if (bonePosition.boneID == boneID)
			return &keyFrame.bonePositions[i];
	}

	return NULL;
}

static BonePosition* getBonePositionOfBone(KeyFrame k, s32 boneID) {
	for (s32 i = 0; i < array_get_length(k.bonePositions); ++i) {
		BonePosition* bp = &k.bonePositions[i];
		if (bp->boneID == boneID) {
			return bp;
		}
	}

	assert(0);
	return NULL;
}

static void getJointToModelTransforms(Animation animation, Joint* joint, 
	const Mat4 parentBoneToModelTransform, boolean isRoot, KeyFrame k1, KeyFrame k2, r32 interpolation)
{
	s32 boneID = joint->boneID;
	BonePosition* bp1 = getBonePositionOfBone(k1, boneID);
	BonePosition* bp2 = getBonePositionOfBone(k2, boneID);
	Quaternion correctedQuaternion = quaternion_nlerp(&bp1->rotation, &bp2->rotation, interpolation);

	Vec3 correctedPosition = gmAddVec3(gmScalarProductVec3(1.0f - interpolation, bp1->position), 
		gmScalarProductVec3(interpolation, bp2->position));

	Mat4 translationMatrix = (Mat4) {
		1.0f, 0.0f, 0.0f, correctedPosition.x,
			0.0f, 1.0f, 0.0f, correctedPosition.y,
			0.0f, 0.0f, 1.0f, correctedPosition.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	Mat4 CORRECTION = (Mat4){
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, -1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	Mat4 rotationMatrix = quaternion_get_matrix(&correctedQuaternion);

	//if (isRoot) {
	//	translationMatrix.data[1][3] = bp->position.z;
	//	translationMatrix.data[2][3] = bp->position.y;
	//	rotationMatrix = gmMultiplyMat4(&CORRECTION, &rotationMatrix);
	//}

	Mat4 localTransform = gmMultiplyMat4(&translationMatrix, &rotationMatrix);

	Mat4 boneToParentBoneTransform = localTransform;
	Mat4 aux = boneToParentBoneTransform;

	Mat4 boneToModelTransform = gmMultiplyMat4(&parentBoneToModelTransform, &boneToParentBoneTransform);

	joint->aux = boneToModelTransform;

	for (s32 i = 0; i < array_get_length(joint->children); ++i) {
		Joint* child = &joint->children[i];
		getJointToModelTransforms(animation, child, boneToModelTransform, false, k1, k2, interpolation);
	}
}

void fillTransforms(Joint* joint, Mat4* transforms) {
	Mat4 bindModelCoordsToBoneCoords = joint->bindModelSpaceToBoneSpaceTransform;
	Mat4 poseBoneCoordsToModelCoords = joint->aux;
	//assert(gmInverseMat4(&aux, &poseBoneCoordsToModelCoords) == true);
	//transforms[joint->boneID] = gmMultiplyMat4(&bindModelCoordsToBoneCoords, &poseBoneCoordsToModelCoords);
	transforms[joint->boneID] = gmMultiplyMat4(&poseBoneCoordsToModelCoords, &bindModelCoordsToBoneCoords);

	for (s32 i = 0; i < array_get_length(joint->children); ++i) {
		fillTransforms(&joint->children[i], transforms);
	}
}

static s32 currentKeyFrameIndex = 0;
static s32 nextKeyFrameIndex = 1;
static r32 time = 0.0f;
static r32 lastTime = 0.0f;
static boolean lastTimeInitialized = false;

static r64 animationDuration = 0.83333331346511841;

static r32 getCurrentTimelinePosition(Animation animation, KeyFrame* k1, KeyFrame* k2) {
	r32 diff;
	if (!lastTimeInitialized) {
		lastTime = glfwGetTime();
		diff = 0.0f;
		lastTimeInitialized = true;
	}

	r32 currentTime = glfwGetTime();
	diff = currentTime - lastTime;
	lastTime = currentTime;

	time += diff;
	if (time > animationDuration) {
		time -= animationDuration;
		currentKeyFrameIndex = 0;
		nextKeyFrameIndex = 1;
	}

	KeyFrame current = animation[currentKeyFrameIndex];
	KeyFrame next = animation[nextKeyFrameIndex];

	r32 currentKeyFrameStart = current.time;
	r32 nextKeyFrameStart = nextKeyFrameIndex != 0 ? next.time : animationDuration;

	if (time < nextKeyFrameStart) {
		r32 interpolation = (time - currentKeyFrameStart) / (nextKeyFrameStart - currentKeyFrameStart);
		*k1 = animation[currentKeyFrameIndex];
		*k2 = animation[nextKeyFrameIndex];
		return interpolation;
	} else {
		// don't check anything, just advance..
		currentKeyFrameIndex = nextKeyFrameIndex;
		nextKeyFrameIndex = array_get_length(animation) - 1 == nextKeyFrameIndex ? 0 : nextKeyFrameIndex + 1;

		current = animation[currentKeyFrameIndex];
		next = animation[nextKeyFrameIndex];

		currentKeyFrameStart = current.time;
		nextKeyFrameStart = nextKeyFrameIndex != 0 ? next.time : animationDuration;

		r32 interpolation = (time - currentKeyFrameStart) / (nextKeyFrameStart - currentKeyFrameStart);
		*k1 = animation[currentKeyFrameIndex];
		*k2 = animation[nextKeyFrameIndex];
	}
}

void animationGetJointTransforms(Animation animation, Joint* rootJoint, Mat4* transforms)
{
	KeyFrame k1, k2;
	r32 interpolation = getCurrentTimelinePosition(animation, &k1, &k2);

	s32 jointIndex = 0;

	Mat4 identity = (Mat4){
		1.0f, 0.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f, 0.0f,
		0.0f, 0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 0.0f, 1.0f
	};

	getJointToModelTransforms(animation, rootJoint, identity, true, k1, k2, interpolation);

	fillTransforms(rootJoint, transforms);
}
