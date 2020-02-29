#include <GLFW/glfw3.h>
#include <dynamic_array.h>
#include "core.h"
#include "graphics.h"
#include <math.h>
#include "obj.h"
#include <stdio.h>
#include <GL/gl.h>

#define PHONG_VERTEX_SHADER_PATH "./shaders/phong_shader.vs"
#define PHONG_FRAGMENT_SHADER_PATH "./shaders/phong_shader.fs"
#define DEPTH_VERTEX_SHADER_PATH "./shaders/depth_shader.vs"
#define DEPTH_FRAGMENT_SHADER_PATH "./shaders/depth_shader.fs"
#define DEPTH_POINT_VERTEX_SHADER_PATH "./shaders/depth_point_shader.vs"
#define DEPTH_POINT_GEOMETRY_SHADER_PATH "./shaders/depth_point_shader.gs"
#define DEPTH_POINT_FRAGMENT_SHADER_PATH "./shaders/depth_point_shader.fs"
#define QUAD_VERTEX_SHADER_PATH "./shaders/quad_shader.vs"
#define QUAD_FRAGMENT_SHADER_PATH "./shaders/quad_shader.fs"
#define BASIC_VERTEX_SHADER_PATH "./shaders/basic_shader.vs"
#define BASIC_FRAGMENT_SHADER_PATH "./shaders/basic_shader.fs"
#define SKYBOX_VERTEX_SHADER_PATH "./shaders/skybox_shader.vs"
#define SKYBOX_FRAGMENT_SHADER_PATH "./shaders/skybox_shader.fs"
#define GIM_ENTITY_COLOR (Vec4) {1.0f, 1.0f, 1.0f, 1.0f}

const u32 SHADOW_WIDTH = 256;
const u32 SHADOW_HEIGHT = 256;

extern s32 windowWidth;
extern s32 windowHeight;

static Shader phongShader, depthShader, depthPointShader, quadShader, basicShader, skyboxShader;
static PerspectiveCamera camera;
static Light* lights;
static Entity e, e2, floorEntity, lightEntity;
static u32 shadowMapFrameBuffer;
static boolean renderDepthMap = false;

static u32 random_tex;

static unsigned int depthCubeMap, depthCubeMapFBO;

static u32 createShadowCubeMapFrameBuffer() {

	glGenFramebuffers(1, &depthCubeMapFBO);

	glGenTextures(1, &depthCubeMap);
	glBindTexture(GL_TEXTURE_CUBE_MAP, depthCubeMap);
	for (unsigned int i = 0; i < 6; ++i) {
		glTexImage2D(GL_TEXTURE_CUBE_MAP_POSITIVE_X + i, 0, GL_DEPTH_COMPONENT, SHADOW_WIDTH, SHADOW_HEIGHT, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
	}

	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_CUBE_MAP, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);

	glBindFramebuffer(GL_FRAMEBUFFER, depthCubeMapFBO);
	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthCubeMap, 0);
	glDrawBuffer(GL_NONE);
	glReadBuffer(GL_NONE);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);  

	return depthCubeMapFBO;
}

static PerspectiveCamera createCamera()
{
	Vec4 lightPosition = (Vec4) {0.0f, 2.0f, 2.0f, 1.0f};

	PerspectiveCamera camera;
	//Vec4 cameraPosition = (Vec4) {0.0f, 0.0f, 1.0f, 1.0f};
	Vec4 cameraPosition = lightPosition;
	Vec4 cameraUp = (Vec4) {0.0f, 1.0f, 0.0f, 1.0f};
	Vec4 cameraView = (Vec4) {-1.0f, -1.0f, 0.0f, 0.0f};
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

	Vec4 lightPosition = (Vec4) {0.0f, 2.0f, 2.0f, 1.0f};
	Vec4 ambientColor = (Vec4) {0.1f, 0.1f, 0.1f, 1.0f};
	Vec4 diffuseColor = (Vec4) {0.8, 0.8, 0.8, 1.0f};
	Vec4 specularColor = (Vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphicsPointLightCreate(&light, lightPosition, ambientColor, diffuseColor, specularColor);
	array_push(lights, &light);

	Vec4 lightDirection = (Vec4) {0.2f, -1.0f, 0.0f, 1.0f};
	ambientColor = (Vec4) {0.0f, 0.0f, 0.0f, 1.0f};
	diffuseColor = (Vec4) {0.8, 0.8, 0.8, 1.0f};
	specularColor = (Vec4) {0.5f, 0.5f, 0.5f, 1.0f};
	graphicsDirectionalLightCreate(&light, lightDirection, ambientColor, diffuseColor, specularColor);
	//array_push(lights, &light);

	return lights;
}

#if 0
static Vertex floorVertices[] = {
	{ -1.0f, 0.0f, -1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, 0.0f, 1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, 0.0f, -1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, 0.0f, 1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f }
};

static u32 floorIndices[] = {
	0, 2, 1,
	1, 2, 3
};
#endif

static Vertex floorVertices[] = {
	// top
	{ -1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, 1.0f, 1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, 1.0f, -1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, 1.0f, 1.0f, 1.0f , 0.0f, 1.0f, 0.0f, 0.0f, 1.0f, 1.0f },

	// bottom
	{ -1.0f, -1.0f, -1.0f, 1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, -1.0f, 1.0f, 1.0f , 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, -1.0f, -1.0f, 1.0f , 0.0f, -1.0f, 0.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, -1.0f, 1.0f, 1.0f , 0.0f, -1.0f, 0.0f, 0.0f, 1.0f, 1.0f },

	// right
	{ 1.0f, -1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
	{ 1.0f, -1.0f, 1.0f, 1.0f , 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, 1.0f, -1.0f, 1.0f , 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, 1.0f, 1.0f, 1.0f , 1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f },

	// left
	{ -1.0f, -1.0f, -1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, -1.0f, 1.0f, 1.0f , -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f },
	{ -1.0f, 1.0f, -1.0f, 1.0f , -1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f },
	{ -1.0f, 1.0f, 1.0f, 1.0f , -1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 1.0f },

	// back
	{ -1.0f, -1.0f, 1.0f, 1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, 1.0f, 1.0f, 1.0f , 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, -1.0f, 1.0f, 1.0f , 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, 1.0f, 1.0f, 1.0f , 0.0f, 0.0f, 1.0f, 0.0f, 1.0f, 1.0f },

	// front
	{ -1.0f, -1.0f, -1.0f, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f },
	{ -1.0f, 1.0f, -1.0f, 1.0f , 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 0.0f },
	{ 1.0f, -1.0f, -1.0f, 1.0f , 0.0f, 0.0f, -1.0f, 0.0f, 0.0f, 1.0f },
	{ 1.0f, 1.0f, -1.0f, 1.0f , 0.0f, 0.0f, -1.0f, 0.0f, 1.0f, 1.0f }
};

static u32 floorIndices[] = {
	0, 1, 2,
	1, 3, 2,

	4, 6, 5,
	5, 6, 7,

	8, 10, 9,
	9, 10, 11,

	12, 13, 14,
	13, 15, 14,

	16, 18, 17,
	17, 18, 19,

	20, 21, 22,
	21, 23, 22
};

static Vec4 FLOOR_COLOR = (Vec4) { 0.4f, 0.4f, 0.4f, 1.0f };
static r32 FLOOR_SIZE = 3.0f;

extern int coreInit()
{
	//createShadowMapFrameBuffer();
	createShadowCubeMapFrameBuffer();
	// Create shader
	phongShader = graphicsShaderCreate(PHONG_VERTEX_SHADER_PATH, PHONG_FRAGMENT_SHADER_PATH);
	depthShader = graphicsShaderCreate(DEPTH_VERTEX_SHADER_PATH, DEPTH_FRAGMENT_SHADER_PATH);
	depthPointShader = graphicsShaderCreateWithGeometry(DEPTH_POINT_VERTEX_SHADER_PATH, DEPTH_POINT_GEOMETRY_SHADER_PATH, DEPTH_POINT_FRAGMENT_SHADER_PATH);
	quadShader = graphicsShaderCreate(QUAD_VERTEX_SHADER_PATH, QUAD_FRAGMENT_SHADER_PATH);
	basicShader = graphicsShaderCreate(BASIC_VERTEX_SHADER_PATH, BASIC_FRAGMENT_SHADER_PATH);
	skyboxShader = graphicsShaderCreate(SKYBOX_VERTEX_SHADER_PATH, SKYBOX_FRAGMENT_SHADER_PATH);
	// Create camera
	camera = createCamera();
	// Create light
	lights = createLights();

#if 1
	Mesh m = graphicsMeshCreateFromObjWithColor("./res/horse.obj", 0, (Vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphicsEntityCreate(&e, m, (Vec4){1.0f, 0.41f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f,1.0f,1.0f});
#else
	Mesh m = graphicsMeshCreateFromObjWithColor("./res/poste.obj", 0, (Vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphicsEntityCreate(&e, m, (Vec4){0.0f, 0.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){1.0f,1.0f,1.0f});
#endif

	Mesh floorMesh = graphicsMeshCreateWithColor(floorVertices, sizeof(floorVertices) / sizeof(Vertex),
		floorIndices, sizeof(floorIndices) / sizeof(u32), NULL, FLOOR_COLOR);
	//graphicsEntityCreate(&floorEntity, floorMesh, (Vec4){0.0f, -0.415f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){FLOOR_SIZE,FLOOR_SIZE,FLOOR_SIZE});

	//Mesh floorMesh = graphicsMeshCreateFromObjWithColor("./res/cube.obj", NULL, FLOOR_COLOR);
	//Mesh floorMesh = graphicsMeshCreateFromObjWithColor("./res/cube.obj", NULL, FLOOR_COLOR);
	//graphicsEntityCreate(&floorEntity, floorMesh, (Vec4){0.0f, -0.415f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){FLOOR_SIZE,0.1f,FLOOR_SIZE});
	float width = 1.0f;
	graphicsEntityCreate(&floorEntity, floorMesh, (Vec4){0.0f, -width, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){FLOOR_SIZE, width, FLOOR_SIZE});

	Mesh cubeMesh = graphicsMeshCreateWithColor(floorVertices, sizeof(floorVertices) / sizeof(Vertex),
		floorIndices, sizeof(floorIndices) / sizeof(u32), NULL, (Vec4){0.0f, 0.3f, 0.5f, 1.0f});
	r32 cubeSizef = 0.2f;
	graphicsEntityCreate(&e2, cubeMesh, (Vec4){0.0f, cubeSizef, 1.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){cubeSizef, cubeSizef, cubeSizef});

	m = graphicsMeshCreateFromObjWithColor("./res/sphere.obj", 0, (Vec4){1.0f, 0.0f, 0.0f, 0.0f});
	graphicsEntityCreate(&lightEntity, m, (Vec4){0.0f, 3.0f, 0.0f, 1.0f}, (Vec3){0.0f, 0.0f, 0.0f}, (Vec3){0.1f, 0.1f, 0.1f});


	random_tex = graphicsTextureCreate("./res/left.jpg");

	return 0;
}

extern void coreDestroy()
{
	array_release(lights);
}

extern void coreUpdate(r32 deltaTime)
{
	lights[0].position = lightEntity.worldPosition;
	//graphicsEntitySetPosition(&lightEntity, lights[0].position);
	//cameraSetPosition(&camera, lights[0].position);
}

extern void coreRender()
{
	// FIRST PASS
	glViewport(0, 0, SHADOW_WIDTH, SHADOW_HEIGHT);
	glBindFramebuffer(GL_FRAMEBUFFER, depthCubeMapFBO);
	glClear(GL_DEPTH_BUFFER_BIT);
	graphicsEntityRenderDepthPointShader(depthPointShader, &camera, &floorEntity, lights, SHADOW_WIDTH, SHADOW_HEIGHT);
	graphicsEntityRenderDepthPointShader(depthPointShader, &camera, &e, lights, SHADOW_WIDTH, SHADOW_HEIGHT);
	graphicsEntityRenderDepthPointShader(depthPointShader, &camera, &e2, lights, SHADOW_WIDTH, SHADOW_HEIGHT);
	glBindFramebuffer(GL_FRAMEBUFFER, 0);

	// SECOND PASS
	glViewport(0, 0, windowWidth, windowHeight);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (!renderDepthMap) {
		graphicsEntityRenderPhongShader(phongShader, &camera, &floorEntity, lights, depthCubeMap);
		graphicsEntityRenderPhongShader(phongShader, &camera, &e, lights, depthCubeMap);
		graphicsEntityRenderPhongShader(phongShader, &camera, &e2, lights, depthCubeMap);
		graphicsEntityRenderPhongShader(phongShader, &camera, &lightEntity, lights, depthCubeMap);
	} else {
		graphicsSkyboxRender(skyboxShader, depthCubeMap, &camera);
	}
}

extern void coreInputProcess(boolean* keyState, r32 deltaTime)
{
	Entity* targetEntity = &lightEntity;
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
			Vec3 rotation = targetEntity->worldRotation;
			rotation.x -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
		}
		else
		{
			Vec3 rotation = targetEntity->worldRotation;
			rotation.x += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
		}
	}
	if (keyState[GLFW_KEY_Y])
	{
		if (keyState[GLFW_KEY_LEFT_SHIFT] || keyState[GLFW_KEY_RIGHT_SHIFT])
		{
			Vec3 rotation = targetEntity->worldRotation;
			rotation.y += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
		}
		else
		{
			Vec3 rotation = targetEntity->worldRotation;
			rotation.y -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
		}
	}
	if (keyState[GLFW_KEY_Z])
	{
		if (keyState[GLFW_KEY_LEFT_SHIFT] || keyState[GLFW_KEY_RIGHT_SHIFT])
		{
			Vec3 rotation = targetEntity->worldRotation;
			rotation.z += rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
		}
		else
		{
			Vec3 rotation = targetEntity->worldRotation;
			rotation.z -= rotationSpeed * deltaTime;
			graphicsEntitySetRotation(targetEntity, rotation);
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
	if (keyState[GLFW_KEY_M])
	{
		renderDepthMap = !renderDepthMap;
		keyState[GLFW_KEY_M] = false;
	}
	if (keyState[GLFW_KEY_P])
	{
		//graphicsEntitySetPosition(targetEntity, gmAddVec4(targetEntity->worldPosition, (Vec4){0.01f, 0.0f, 0.0f, 0.0f}));
		graphicsEntitySetPosition(targetEntity, gmAddVec4(targetEntity->worldPosition, (Vec4){0.00f, 0.0f, 0.01f, 0.0f}));
		//Light* l = &lights[0];
		//l->direction.x += 0.02f;
	}
	if (keyState[GLFW_KEY_O])
	{
		//graphicsEntitySetPosition(targetEntity, gmAddVec4(targetEntity->worldPosition, (Vec4){-0.01f, 0.0f, 0.0f, 0.0f}));
		graphicsEntitySetPosition(targetEntity, gmAddVec4(targetEntity->worldPosition, (Vec4){0.0f, 0.0f, -0.01f, 0.0f}));
		//Light* l = &lights[0];
		//l->direction.x -= 0.02f;
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