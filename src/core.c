#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include "core.h"
#include "graphics.h"
#include <math.h>
#include "obj.h"
#include <stdio.h>

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define QUAD_VERTEX_SHADER_PATH "./shaders/quad_shader.vs"
#define QUAD_FRAGMENT_SHADER_PATH "./shaders/quad_shader.fs"
#define GIM_ENTITY_COLOR (Vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader phongShader, quadShader;
static PerspectiveCamera camera;
static Light* lights;
static Entity e;

extern s32 windowWidth;
extern s32 windowHeight;

static unsigned int fbo;
static unsigned int fbo_color_tex;

static boolean hdr = true;

static PerspectiveCamera createCamera()
{
	PerspectiveCamera camera;
	Vec4 cameraPosition = (Vec4) {0.5f, 0.5f, -5.0f, 1.0f};
	Vec4 cameraUp = (Vec4) {0.0f, 1.0f, 0.0f, 1.0f};
	Vec4 cameraView = (Vec4) {0.0f, 0.0f, 1.0f, 0.0f};
	r32 cameraNearPlane = -0.01f;
	r32 cameraFarPlane = -1000.0f;
	r32 cameraFov = 45.0f;
	cameraInit(&camera, cameraPosition, cameraUp, cameraView, cameraNearPlane, cameraFarPlane, cameraFov);
	return camera;
}

static Light* createLights()
{
	Light light;
	Light* lights = array_create(Light, 1);

	Vec4 lightPosition = (Vec4) {0.5f, 0.5f, 9.5f, 1.0f};
	Vec4 ambientColor = (Vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	Vec4 diffuseColor = (Vec4) {100.0f, 100.0f, 100.0f, 100.0f};
	Vec4 specularColor = (Vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphicsLightCreate(&light, lightPosition, ambientColor, diffuseColor, specularColor);
	array_push(lights, &light);

	return lights;
}

static unsigned int createHDRFrameBuffer() 
{
	// Create framebuffer
	unsigned int fbo;
	glGenFramebuffers(1, &fbo);
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);

	// Create color attachment (texture)
	glGenTextures(1, &fbo_color_tex);
	glBindTexture(GL_TEXTURE_2D, fbo_color_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, windowWidth, windowHeight, 0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_color_tex, 0);
	glBindTexture(GL_TEXTURE_2D, 0);

	// Create depth/stencil attachments (renderbuffers)
	unsigned int rbo;
	glGenRenderbuffers(1, &rbo);
	glBindRenderbuffer(GL_RENDERBUFFER, rbo);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, windowWidth, windowHeight);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, rbo);

	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
		printf("ERROR::FRAMEBUFFER:: Framebuffer is not complete!\n");

	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	return fbo;
}

extern int coreInit()
{
	// Create shader
	phongShader = graphicsShaderCreate(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
	quadShader = graphicsShaderCreate(QUAD_VERTEX_SHADER_PATH, QUAD_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = createCamera();
	// Create light
	lights = createLights();
	// Create framebuffer
	fbo = createHDRFrameBuffer();

	Mesh m = graphicsMeshCreateFromObjWithColor("./res/cube.obj", 0, (Vec4){0.22f, 0.10f, 0.05f, 0.0f});
	graphicsEntityCreate(&e, m, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f, 1.0f, 1.0f});

	return 0;
}

extern void coreDestroy()
{
	array_release(lights);
}

extern void coreUpdate(r32 deltaTime)
{

}

extern void coreRender()
{
	glBindFramebuffer(GL_FRAMEBUFFER, fbo);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	graphicsEntityRenderPhongShader(phongShader, &camera, &e, lights);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glClear(GL_COLOR_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);
	graphicsQuadRender(quadShader, fbo_color_tex, hdr);
}

extern void coreInputProcess(boolean* keyState, r32 deltaTime)
{
	r32 movementSpeed = 3.0f;
	r32 rotationSpeed = 3.0f;

	if (keyState[GLFW_KEY_LEFT_SHIFT])
		movementSpeed = 0.5f;
	if (keyState[GLFW_KEY_RIGHT_SHIFT])
		movementSpeed = 0.1f;

	if (keyState[GLFW_KEY_W])
		cameraSetPosition(&camera, gmAddVec4(camera.position, gmScalarProductVec4(movementSpeed * deltaTime, gmNormalizeVec4(camera.view))));
	if (keyState[GLFW_KEY_S])
		cameraSetPosition(&camera, gmAddVec4(camera.position, gmScalarProductVec4(-movementSpeed * deltaTime, gmNormalizeVec4(camera.view))));
	if (keyState[GLFW_KEY_A])
		cameraSetPosition(&camera, gmAddVec4(camera.position, gmScalarProductVec4(-movementSpeed * deltaTime, gmNormalizeVec4(camera.xAxis))));
	if (keyState[GLFW_KEY_D])
		cameraSetPosition(&camera, gmAddVec4(camera.position, gmScalarProductVec4(movementSpeed * deltaTime, gmNormalizeVec4(camera.xAxis))));
	if (keyState[GLFW_KEY_X])
	{
		if (keyState[GLFW_KEY_LEFT_SHIFT] || keyState[GLFW_KEY_RIGHT_SHIFT])
		{
			Vec3 rotation = e.worldRotation;
			rotation.x -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
		else
		{
			Vec3 rotation = e.worldRotation;
			rotation.x += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
	}
	if (keyState[GLFW_KEY_Y])
	{
		if (keyState[GLFW_KEY_LEFT_SHIFT] || keyState[GLFW_KEY_RIGHT_SHIFT])
		{
			Vec3 rotation = e.worldRotation;
			rotation.y += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
		else
		{
			Vec3 rotation = e.worldRotation;
			rotation.y -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
	}
	if (keyState[GLFW_KEY_Z])
	{
		if (keyState[GLFW_KEY_LEFT_SHIFT] || keyState[GLFW_KEY_RIGHT_SHIFT])
		{
			Vec3 rotation = e.worldRotation;
			rotation.z += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
		else
		{
			Vec3 rotation = e.worldRotation;
			rotation.z -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(&e, rotation);
		}
	}
	if (keyState[GLFW_KEY_L])
	{
		static boolean wireframe = false;

		if (wireframe)
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		else
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

		wireframe = !wireframe;
		keyState[GLFW_KEY_L] = false;
	}
	if (keyState[GLFW_KEY_H])
	{
		hdr = !hdr;
		keyState[GLFW_KEY_H] = false;
	}
}

extern void coreMouseChangeProcess(boolean reset, r64 xPos, r64 yPos)
{
	static r64 xPosOld, yPosOld;
	// This constant is basically the mouse sensibility.
	// @TODO: Allow mouse sensibility to be configurable.
	static const r32 cameraMouseSpeed = 0.001f;

	if (!reset)
	{
		r64 xDifference = xPos - xPosOld;
		r64 yDifference = yPos - yPosOld;

		r32 pitchAngle = -cameraMouseSpeed * (float)xDifference;
		r32 yawAngle = cameraMouseSpeed * (float)yDifference;

		cameraIncPitch(&camera, pitchAngle);
		cameraIncYaw(&camera, yawAngle);
	}

	xPosOld = xPos;
	yPosOld = yPos;
}

extern void coreMouseClickProcess(s32 button, s32 action, r64 xPos, r64 yPos)
{

}

extern void coreScrollChangeProcess(r64 xOffset, r64 yOffset)
{

}

extern void coreWindowResizeProcess(s32 width, s32 height)
{

}