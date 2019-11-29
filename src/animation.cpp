#include "animation.hpp"
#include <dynamic_array.h>

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