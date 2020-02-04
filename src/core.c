#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include "core.h"
#include "graphics.h"
#include <math.h>
#include "obj.h"
#include <stdio.h>

#define BASIC_VERTEX_SHADER_PATH "./shaders/basic_shader.vs"
#define BASIC_FRAGMENT_SHADER_PATH "./shaders/basic_shader.fs"
#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define PBR_VERTEX_SHADER_PATH "./shaders/pbr_shader.vs"
#define PBR_FRAGMENT_SHADER_PATH "./shaders/pbr_shader.fs"
#define GIM_ENTITY_COLOR (Vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader pbrShader, basicShader;
static PerspectiveCamera camera;
static Light* lights;
static PBRLight* pbrLights;
static Entity e;
static Mesh lightMesh;
static boolean tweak = false;

static PerspectiveCamera createCamera()
{
	PerspectiveCamera camera;
	Vec4 cameraPosition = (Vec4) {0.0f, 0.0f, 10.0f, 1.0f};
	Vec4 cameraUp = (Vec4) {0.0f, 1.0f, 0.0f, 1.0f};
	Vec4 cameraView = (Vec4) {0.0f, 0.0f, -1.0f, 0.0f};
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

	Vec4 lightPosition = (Vec4) {0.0f, 10.0f, 0.0f, 1.0f};
	Vec4 ambientColor = (Vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	Vec4 diffuseColor = (Vec4) {0.8, 0.8, 0.8, 1.0f};
	Vec4 specularColor = (Vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphicsLightCreate(&light, lightPosition, ambientColor, diffuseColor, specularColor);
	array_push(lights, &light);

	return lights;
}

static PBRLight* createPBRLights()
{
	PBRLight light;
	PBRLight* lights = array_create(PBRLight, 1);

	Vec3 lightPosition = (Vec3) {10.0f, 10.0f, 10.0f};
	Vec3 color = (Vec3) { 500.0f, 0.0, 0.0};
	graphicsPBRLightCreate(&light, lightPosition, color);
	array_push(lights, &light);

	lightPosition = (Vec3) {-10.0f, 10.0f, 10.0f};
	color = (Vec3) {0,500.0f,500.0f};
	graphicsPBRLightCreate(&light, lightPosition, color);
	array_push(lights, &light);

	//lightPosition = (Vec3) {-10.0f, 10.0f, 10.0f};
	//color = (Vec3) {10.0f, 10.0f, 10.0f};
	//graphicsPBRLightCreate(&light, lightPosition, color);
	//array_push(lights, &light);

	//lightPosition = (Vec3) {-10.0f, 10.0f, -10.0f};
	//color = (Vec3) {10.0f, 10.0f, 10.0f};
	//graphicsPBRLightCreate(&light, lightPosition, color);
	//array_push(lights, &light);

	return lights;
}

extern int coreInit()
{
	// Create shader
	pbrShader = graphicsShaderCreate(PBR_VERTEX_SHADER_PATH, PBR_FRAGMENT_SHADER_PATH);
	basicShader = graphicsShaderCreate(BASIC_VERTEX_SHADER_PATH, BASIC_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = createCamera();
	// Create light
	lights = createLights();
	pbrLights = createPBRLights();

	lightMesh = graphicsMeshCreateFromObjWithColor("./res/sphere.obj", 0, (Vec4){1.0f, 1.0f, 1.0f, 1.0f});

	//Mesh m = graphicsMeshCreateFromObjWithColor("./res/horse.obj", 0, (Vec4){1.0f, 0.0f, 0.0f, 0.0f});
	u32 albedoMap = graphicsTextureCreate("./res/rustediron2_basecolor.png");
	u32 metallicMap = graphicsTextureCreate("./res/rustediron2_metallic.png");
	u32 roughnessMap = graphicsTextureCreate("./res/rustediron2_roughness.png");
	Mesh m = graphicsMeshCreateFromObjWithPbrInfo("./res/sphere.obj", 0, albedoMap, metallicMap, roughnessMap);
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
	const r32 GAP = 2.0f;

	for (u32 i = 0; i < 10; ++i) {
		for (u32 j = 0; j < 10; ++j) {
			Vec4 position = (Vec4){i * GAP - 5.0f, j * GAP - 5.0f, 0.0f};
			graphicsEntitySetPosition(&e, position);
			graphicsEntityRenderPbrShader(pbrShader, &camera, &e, pbrLights, tweak);
		}
	}

	for (u32 i = 0; i < array_get_length(pbrLights); ++i) {
		Entity l;
		Vec4 pos = (Vec4){pbrLights[i].position.x, pbrLights[i].position.y, pbrLights[i].position.z, 1.0f};
		graphicsEntityCreate(&l, lightMesh, pos, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){0.2f, 0.2f, 0.2f});
		graphicsEntityRenderBasicShader(basicShader, &camera, &l);
	}
}

extern void coreInputProcess(boolean* keyState, r32 deltaTime)
{
	r32 movementSpeed = 10.0f;
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
	if (keyState[GLFW_KEY_T])
	{
		tweak = !tweak;
		keyState[GLFW_KEY_T] = false;
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