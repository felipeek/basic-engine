#ifndef BASIC_ENGINE_ANIMATION_H
#define BASIC_ENGINE_ANIMATION_H
#include "graphics.hpp"
#include "quaternion.hpp"

typedef struct
{
	s32 boneID;
	Vec3 position;
	Quaternion rotation;
} BonePosition;

typedef struct
{
	BonePosition* bonePositions;
	double time;
} KeyFrame;

typedef KeyFrame* Animation;

void animationCreate(Animation* animation);
void animationDestroy(Animation animation);
void animationAddKeyFrame(Animation* animation, KeyFrame keyFrame);
KeyFrame* animationGetKeyFrame(const Animation animation, double time);
void keyFrameCreate(KeyFrame* keyFrame, double time);
void keyFrameDestroy(KeyFrame keyFrame);
void keyFrameAddBonePosition(KeyFrame* keyFrame, BonePosition bonePosition);
BonePosition* keyFrameGetBonePosition(const KeyFrame keyFrame, int boneID);

#endif