#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include "core.h"
#include "graphics.h"
#include <math.h>
#include "obj.h"
#include <stdio.h>

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define INSTANCING_VERTEX_SHADER_PATH "./shaders/instancing_shader.vs"
#define INSTANCING_FRAGMENT_SHADER_PATH "./shaders/instancing_shader.fs"
#define GIM_ENTITY_COLOR (Vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Shader phongShader, instanceShader;
static PerspectiveCamera camera;
static Light* lights;
static Entity e;

static Mesh steroidMesh, planetMesh;
static Entity steroids, planet;

static PerspectiveCamera createCamera()
{
	PerspectiveCamera camera;
	Vec4 cameraPosition = (Vec4) {0.0f, 0.0f, 1.0f, 1.0f};
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

	Vec4 lightPosition = (Vec4) {0.0f, 0.0f, 1.0f, 1.0f};
	Vec4 ambientColor = (Vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	Vec4 diffuseColor = (Vec4) {0.8, 0.8, 0.8, 1.0f};
	Vec4 specularColor = (Vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphicsLightCreate(&light, lightPosition, ambientColor, diffuseColor, specularColor);
	array_push(lights, &light);

	return lights;
}

static void createSteorids()
{
	const float STEROID_DISTANCE = 1.5;
	const float NOISE_MAG = 0.3;
	const float SCALE = 0.01f;
	Vertex* vertices;
	u32* indexes;
	objParse("./res/sphere.obj", &vertices, &indexes);
	planetMesh = graphicsMeshCreateWithColor(vertices, array_get_length(vertices), indexes,
		array_get_length(indexes), 0, (Vec4){1.0f, 0.0f, 0.0f, 1.0f});

	Entity e;
	graphicsEntityCreate(&e, planetMesh, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f, 1.0f, 1.0f});

	s32 steroidsQtd = 5000;
	Mat4* modelMatrices = array_create(Mat4, steroidsQtd);

	for (s32 i = 0; i < steroidsQtd; ++i) {
		Vec4 direction = (Vec4) {(r32)rand(), 0.0f, (r32)rand(), 0.0f};
		if (rand() % 2) direction.x = -direction.x;
		if (rand() % 2) direction.z = -direction.z;
		direction = gmNormalizeVec4(direction);
		direction = gmScalarProductVec4(STEROID_DISTANCE, direction);
		r32 positionNoise = NOISE_MAG * ((r32)rand() / (r32)RAND_MAX);
		Vec4 noise = gmScalarProductVec4(positionNoise, gmNormalizeVec4(direction));
		direction = gmAddVec4(noise, direction);
		Vec4 worldPos = (Vec4) {direction.x, direction.y, direction.z, 1.0f};

		r32 scale = SCALE * ((r32)rand() / (r32)RAND_MAX);

		Vec3 scaleVec = (Vec3) {scale, scale, scale};

		graphicsEntitySetPosition(&e, worldPos);
		graphicsEntitySetScale(&e, scaleVec);
		
		Mat4 m = gmTransposeMat4(&e.modelMatrix);
		array_push(modelMatrices, &m);
	}

	Vertex* steroidVertices;
	u32* steroidIndexes;
	objParse("./res/sphere.obj", &steroidVertices, &steroidIndexes);
	steroidMesh = graphicsSteroidMeshCreateWithColor(steroidVertices, array_get_length(steroidVertices), steroidIndexes,
		array_get_length(steroidIndexes), NULL, (Vec4){0.0f, 1.0f, 0.0f, 1.0f}, steroidsQtd, modelMatrices);

	graphicsEntityCreate(&planet, planetMesh, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f, 1.0f, 1.0f});
	graphicsEntityCreate(&steroids, steroidMesh, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f, 1.0f, 1.0f});

	array_release(modelMatrices);
}

extern int coreInit()
{
	// Create shader
	phongShader = graphicsShaderCreate(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
	instanceShader = graphicsShaderCreate(INSTANCING_VERTEX_SHADER_PATH, INSTANCING_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = createCamera();
	// Create light
	lights = createLights();

	Mesh m = graphicsMeshCreateFromObjWithColor("./res/horse.obj", 0, (Vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphicsEntityCreate(&e, m, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f, 1.0f, 1.0f});

	createSteorids();

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
	graphicsEntityRenderPhongShader(phongShader, &camera, &planet, lights);
	graphicsEntityRenderInstanceShader(instanceShader, &camera, &steroids, lights);
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