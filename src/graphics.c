#include "graphics.h"
#include "util.h"
#include "obj.h"
#include <GL/glew.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <dynamic_array.h>
#include <math.h>

extern ImageData graphicsImageLoad(const s8* imagePath)
{
	ImageData imageData;

	stbi_set_flip_vertically_on_load(1);
	imageData.data = stbi_load(imagePath, &imageData.width, &imageData.height, &imageData.channels, 4);

	imageData.channels = 4;	// @temporary

	return imageData;
}

extern FloatImageData graphicsFloatImageLoad(const s8* imagePath)
{
	ImageData imageData;

	stbi_set_flip_vertically_on_load(1);
	imageData.data = stbi_load(imagePath, &imageData.width, &imageData.height, &imageData.channels, 4);

	imageData.channels = 4;	// @temporary

	FloatImageData fid = graphicsImageDataToFloatImageData(&imageData, 0);

	graphicsImageFree(&imageData);

	return fid;
}

extern FloatImageData graphicsFloatImageCopy(const FloatImageData* imageData)
{
	FloatImageData fid;

	fid = *imageData;

	fid.data = malloc(sizeof(r32) * fid.width * fid.height * fid.channels);
	memcpy(fid.data, imageData->data, sizeof(r32) * fid.width * fid.height * fid.channels);
	
	return fid;
}

extern void graphicsImageFree(ImageData* imageData)
{
	stbi_image_free(imageData->data);
}

extern void graphicsFloatImageFree(FloatImageData* imageData)
{
	free(imageData->data);
}

extern void graphicsImageSave(const s8* imagePath, const ImageData* imageData)
{
	stbi_flip_vertically_on_write(1);
	stbi_write_bmp(imagePath, imageData->width, imageData->height, imageData->channels, imageData->data);
}

extern void graphicsFloatImageSave(const s8* imagePath, const FloatImageData* imageData)
{
	ImageData id = graphicsFloatImageDataToImageData(imageData, 0);
	graphicsImageSave(imagePath, &id);
	graphicsImageFree(&id);
}

extern Shader graphicsShaderCreate(const s8* vertexShaderPath, const s8* fragmentShaderPath)
{
	s8* vertexShaderCode = utilReadFile(vertexShaderPath, 0);
	s8* fragmentShaderCode = utilReadFile(fragmentShaderPath, 0);

	GLint success;
	GLchar infoLogBuffer[1024];
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(vertexShader, 1, (const GLchar *const*)&vertexShaderCode, 0);
	glShaderSource(fragmentShader, 1, (const GLchar *const*)&fragmentShaderCode, 0);

	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 1024, 0, infoLogBuffer);
		printf("Error compiling vertex shader: %s\n", infoLogBuffer);
	}

	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 1024, 0, infoLogBuffer);
		printf("Error compiling fragment shader: %s\n", infoLogBuffer);
	}

	GLuint shaderProgram = glCreateProgram();

	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);

	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(shaderProgram, 1024, 0, infoLogBuffer);
		printf("Error linking program: %s\n", infoLogBuffer);
	}

	free(vertexShaderCode);
	free(fragmentShaderCode);
	return shaderProgram;
}

// Vertices must be Vertex[4]
// Indexes must be u32[6]
static void fillQuadVerticesAndIndexes(r32 size, Vertex* vertices, u32* indices)
{
	vertices[0].position = (Vec4) { 0.0f, 0.0f, 0.0f, 1.0f };
	vertices[0].normal = (Vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[0].textureCoordinates = (Vec2) { 0.0f, 0.0f };

	vertices[1].position = (Vec4) { size, 0.0f, 0.0f, 1.0f };
	vertices[1].normal = (Vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[1].textureCoordinates = (Vec2) { 1.0f, 0.0f };

	vertices[2].position = (Vec4) { 0.0f, size, 0.0f, 1.0f };
	vertices[2].normal = (Vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[2].textureCoordinates = (Vec2) { 0.0f, 1.0f };

	vertices[3].position = (Vec4) { size, size, 0.0f, 1.0f };
	vertices[3].normal = (Vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[3].textureCoordinates = (Vec2) { 1.0f, 1.0f };

	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 2;
	indices[3] = 1;
	indices[4] = 3;
	indices[5] = 2;
}

extern Mesh graphicsQuadCreateWithTexture(u32 texture)
{
	r32 size = 1.0f;
	Vertex vertices[4];
	u32 indices[6];

	fillQuadVerticesAndIndexes(size, vertices, indices);

	return graphicsMeshCreateWithTexture(vertices, sizeof(vertices) / sizeof(Vertex), indices,
		sizeof(indices) / sizeof(u32), 0, texture);
}

extern Mesh graphicsQuadCreateWithColor(Vec4 color)
{
	r32 size = 1.0f;
	Vertex vertices[4];
	u32 indices[6];

	fillQuadVerticesAndIndexes(size, vertices, indices);

	return graphicsMeshCreateWithColor(vertices, sizeof(vertices) / sizeof(Vertex), indices,
		sizeof(indices) / sizeof(u32), 0, color);
}

static Mesh createSimpleMesh(Vertex* vertices, s32 verticesSize, u32* indices, s32 indicesSize, NormalMappingInfo* normalInfo)
{
	Mesh mesh;
	GLuint VBO, EBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, verticesSize * sizeof(Vertex), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, verticesSize * sizeof(Vertex), vertices);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), (void*)(4 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 10 * sizeof(GLfloat), (void*)(8 * sizeof(GLfloat)));
	glEnableVertexAttribArray(2);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indicesSize * sizeof(u32), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indicesSize * sizeof(u32), indices);

	glBindVertexArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	mesh.VAO = VAO;
	mesh.VBO = VBO;
	mesh.EBO = EBO;
	mesh.indexesSize = indicesSize;

	if (!normalInfo)
	{
		mesh.normalInfo.tangentSpace = false;
		mesh.normalInfo.useNormalMap = false;
		mesh.normalInfo.normalMapTexture = 0;
	}
	else
		mesh.normalInfo = *normalInfo;

	return mesh;
}

extern Mesh graphicsMeshCreateWithColor(Vertex* vertices, s32 verticesSize, u32* indices, s32 indicesSize, NormalMappingInfo* normalInfo, Vec4 diffuseColor)
{
	Mesh mesh = createSimpleMesh(vertices, verticesSize, indices, indicesSize, normalInfo);
	mesh.diffuseInfo.useDiffuseMap = false;
	mesh.diffuseInfo.diffuseColor = diffuseColor;
	return mesh;
}

extern Mesh graphicsMeshCreateWithTexture(Vertex* vertices, s32 verticesSize, u32* indices, s32 indicesSize, NormalMappingInfo* normalInfo, u32 diffuseMap)
{
	Mesh mesh = createSimpleMesh(vertices, verticesSize, indices, indicesSize, normalInfo);
	mesh.diffuseInfo.useDiffuseMap = true;
	mesh.diffuseInfo.diffuseMap = diffuseMap;
	return mesh;
}

static s8* buildLightUniformName(s8* buffer, s32 index, const s8* property)
{
	sprintf(buffer, "lights[%d].%s", index, property);
	return buffer;
}

static void lightUpdateUniforms(const Light* lights, Shader shader)
{
	s32 numberOfLights = array_get_length(lights);
	s8 buffer[64];
	glUseProgram(shader);

	for (s32 i = 0; i < numberOfLights; ++i)
	{
		Light light = lights[i];
		GLint lightTypeLocation = glGetUniformLocation(shader, buildLightUniformName(buffer, i, "type"));
		GLint lightPositionOrDirectionLocation = glGetUniformLocation(shader, buildLightUniformName(buffer, i, "positionOrDirection"));
		GLint ambientColorLocation = glGetUniformLocation(shader, buildLightUniformName(buffer, i, "ambientColor"));
		GLint diffuseColorLocation = glGetUniformLocation(shader, buildLightUniformName(buffer, i, "diffuseColor"));
		GLint specularColorLocation = glGetUniformLocation(shader, buildLightUniformName(buffer, i, "specularColor"));
		glUniform1i(lightTypeLocation, light.lightType);
		if (light.lightType == POINT_LIGHT) {
			glUniform4f(lightPositionOrDirectionLocation, light.position.x, light.position.y, light.position.z, light.position.w);
		} else if (light.lightType == DIRECTIONAL_LIGHT) {
			glUniform4f(lightPositionOrDirectionLocation, light.direction.x, light.direction.y, light.direction.z, light.direction.w);
		}
		glUniform4f(ambientColorLocation, light.ambientColor.x, light.ambientColor.y, light.ambientColor.z, light.ambientColor.w);
		glUniform4f(diffuseColorLocation, light.diffuseColor.x, light.diffuseColor.y, light.diffuseColor.z, light.diffuseColor.w);
		glUniform4f(specularColorLocation, light.specularColor.x, light.specularColor.y, light.specularColor.z, light.specularColor.w);
	}

	GLint lightQuantityLocation = glGetUniformLocation(shader, "lightQuantity");
	glUniform1i(lightQuantityLocation, numberOfLights);
}

static void diffuseUpdateUniforms(const DiffuseInfo* diffuseInfo, Shader shader)
{
	glUseProgram(shader);
	GLint useDiffuseMapLocation = glGetUniformLocation(shader, "diffuseInfo.useDiffuseMap");
	GLint diffuseMapLocation = glGetUniformLocation(shader, "diffuseInfo.diffuseMap");
	GLint diffuseColorLocation = glGetUniformLocation(shader, "diffuseInfo.diffuseColor");
	glUniform1i(useDiffuseMapLocation, diffuseInfo->useDiffuseMap);
	if (diffuseInfo->useDiffuseMap)
	{
		glUniform1i(diffuseMapLocation, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, diffuseInfo->diffuseMap);
	}
	else
		glUniform4f(diffuseColorLocation, diffuseInfo->diffuseColor.x, diffuseInfo->diffuseColor.y,
			diffuseInfo->diffuseColor.z, diffuseInfo->diffuseColor.w);
}

static void normalsUpdateUniforms(const NormalMappingInfo* normalInfo, Shader shader)
{
	glUseProgram(shader);
	GLint useNormalMapLocation = glGetUniformLocation(shader, "normalMappingInfo.useNormalMap");
	GLint normalMapTextureLocation = glGetUniformLocation(shader, "normalMappingInfo.normalMapTexture");
	GLint tangentSpaceLocation = glGetUniformLocation(shader, "normalMappingInfo.tangentSpace");
	glUniform1i(useNormalMapLocation, normalInfo->useNormalMap);
	if (normalInfo->useNormalMap)
	{
		glUniform1i(normalMapTextureLocation, 2);
		glUniform1i(tangentSpaceLocation, normalInfo->tangentSpace);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, normalInfo->normalMapTexture);
	}
}

// This function must be re-done.
// This implementation is just temporary, but it's not bug-free and will work only for a limited set of objs.
extern void graphicsMeshRender(Shader shader, Mesh mesh)
{
	glBindVertexArray(mesh.VAO);
	glUseProgram(shader);
	diffuseUpdateUniforms(&mesh.diffuseInfo, shader);
	normalsUpdateUniforms(&mesh.normalInfo, shader);
	glDrawElements(GL_TRIANGLES, mesh.indexesSize, GL_UNSIGNED_INT, 0);
	glUseProgram(0);
	glBindVertexArray(0);
}

extern void graphicsMeshChangeDiffuseMap(Mesh* mesh, u32 diffuseMap, boolean deleteDiffuseMap)
{
	if (deleteDiffuseMap && mesh->diffuseInfo.useDiffuseMap)
		glDeleteTextures(1, &mesh->diffuseInfo.diffuseMap);

	mesh->diffuseInfo.diffuseMap = diffuseMap;
	mesh->diffuseInfo.useDiffuseMap = true;
}

extern void graphicsMeshChangeColor(Mesh* mesh, Vec4 color, boolean deleteDiffuseMap)
{
	if (deleteDiffuseMap && mesh->diffuseInfo.useDiffuseMap)
		glDeleteTextures(1, &mesh->diffuseInfo.diffuseMap);

	mesh->diffuseInfo.useDiffuseMap = false;
	mesh->diffuseInfo.diffuseColor = color;
}

static void recalculateModelMatrix(Entity* entity)
{
	r32 s, c;

	Mat4 scaleMatrix = (Mat4) {
		entity->worldScale.x, 0.0f, 0.0f, 0.0f,
			0.0f, entity->worldScale.y, 0.0f, 0.0f,
			0.0f, 0.0f, entity->worldScale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	s = sinf(entity->worldRotation.x);
	c = cosf(entity->worldRotation.x);
	Mat4 rotXMatrix = (Mat4) {
		1.0f, 0.0f, 0.0f, 0.0f,
			0.0f, c, -s, 0.0f,
			0.0f, s, c, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	s = sinf(entity->worldRotation.y);
	c = cosf(entity->worldRotation.y);
	Mat4 rotYMatrix = (Mat4) {
		c, 0.0f, s, 0.0f,
			0.0f, 1.0f, 0.0f, 0.0f,
			-s, 0.0f, c, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	s = sinf(entity->worldRotation.z);
	c = cosf(entity->worldRotation.z);
	Mat4 rotZMatrix = (Mat4) {
		c, -s, 0.0f, 0.0f,
			s, c, 0.0f, 0.0f,
			0.0f, 0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	Mat4 rotationMatrix = gmMultiplyMat4(&rotYMatrix, &rotXMatrix);
	rotationMatrix = gmMultiplyMat4(&rotZMatrix, &rotationMatrix);

	Mat4 translationMatrix = (Mat4) {
		1.0f, 0.0f, 0.0f, entity->worldPosition.x,
			0.0f, 1.0f, 0.0f, entity->worldPosition.y,
			0.0f, 0.0f, 1.0f, entity->worldPosition.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	entity->modelMatrix = gmMultiplyMat4(&rotationMatrix, &scaleMatrix);
	entity->modelMatrix = gmMultiplyMat4(&translationMatrix, &entity->modelMatrix);
}

extern void graphicsEntityCreate(Entity* entity, Mesh mesh, Vec4 worldPosition, Vec3 worldRotation, Vec3 worldScale)
{
	entity->mesh = mesh;
	entity->worldPosition = worldPosition;
	entity->worldRotation = worldRotation;
	entity->worldScale = worldScale;
	recalculateModelMatrix(entity);
}

extern void graphicsEntityMeshReplace(Entity* entity, Mesh mesh,
	boolean deleteNormalMap , boolean deleteDiffuseMap)
{
	glDeleteBuffers(1, &entity->mesh.VBO);
	glDeleteBuffers(1, &entity->mesh.EBO);
	glDeleteVertexArrays(1, &entity->mesh.VAO);
	if (deleteNormalMap && entity->mesh.normalInfo.useNormalMap)
		glDeleteTextures(1, &entity->mesh.normalInfo.normalMapTexture);
	if (deleteDiffuseMap && entity->mesh.diffuseInfo.useDiffuseMap)
		glDeleteTextures(1, &entity->mesh.diffuseInfo.diffuseMap);

	entity->mesh = mesh;
}

extern void graphicsEntitySetPosition(Entity* entity, Vec4 worldPosition)
{
	entity->worldPosition = worldPosition;
	recalculateModelMatrix(entity);
}

extern void graphicsEntitySetRotation(Entity* entity, Vec3 worldRotation)
{
	entity->worldRotation = worldRotation;
	recalculateModelMatrix(entity);
}

extern void graphicsEntitySetScale(Entity* entity, Vec3 worldScale)
{
	entity->worldScale = worldScale;
	recalculateModelMatrix(entity);
}

static Mat4 getLightSpaceMatrixForLight(const Light* light)
{
	r32 right = 10.0f;
	r32 left = -10.0f;
	r32 top = 10.0f;
	r32 bottom = -10.0f;
	r32 near = 1.0f;
	r32 far = 10.5f;

	Mat4 ortho = {
		2.0f / (right - left), 0.0f, 0.0f, -(right + left)/(right - left),
		0.0f, 2.0f / (top - bottom), 0.0f, -(top + bottom)/(top - bottom),
		0.0f, 0.0f, -2.0f / (far - near), -(far + near)/(far - near),
		0.0f, 0.0f, 0.0f, 1.0f
	};

	const r32 DISTANCE = 5.0f;
	Vec4 lightPosition = gmScalarProductVec4(-1.0f, light->direction);
	lightPosition = gmNormalizeVec4(lightPosition);
	lightPosition = gmScalarProductVec4(DISTANCE, lightPosition);
	lightPosition.w = 1.0f;

	Vec3 upVec3 = (Vec3) { 0.0f, 1.0f, 0.0f };
	Vec4 w = gmScalarProductVec4(-1, gmNormalizeVec4(light->direction));
	Vec3 wVec3 = (Vec3) { w.x, w.y, w.z };
	Vec3 upWCross = gmCrossProduct(upVec3, wVec3);
	Vec4 u = gmNormalizeVec4((Vec4) { upWCross.x, upWCross.y, upWCross.z, 0.0f });
	Vec3 uVec3 = (Vec3) { u.x, u.y, u.z };
	Vec3 vVec3 = gmCrossProduct(wVec3, uVec3);
	Vec4 v = (Vec4) { vVec3.x, vVec3.y, vVec3.z, 0.0f };
	// Useless, but conceptually correct.
	Vec4 worldToCameraVec = gmSubtractVec4(lightPosition, (Vec4) { 0.0f, 0.0f, 0.0f, 1.0f });

	// Need to transpose when sending to shader
	Mat4 view = (Mat4) {
		u.x, u.y, u.z, -gmDotProductVec4(u, worldToCameraVec),
			v.x, v.y, v.z, -gmDotProductVec4(v, worldToCameraVec),
			w.x, w.y, w.z, -gmDotProductVec4(w, worldToCameraVec),
			0.0f, 0.0f, 0.0f, 1.0f
	};

	return gmMultiplyMat4(&ortho, &view);
}

extern void graphicsEntityRenderDepthShader(Shader shader, const PerspectiveCamera* camera, const Entity* entity, const Light* light)
{
	glUseProgram(shader);
	GLint modelMatrixLocation = glGetUniformLocation(shader, "modelMatrix");
	GLint lightSpaceMatrixLocation = glGetUniformLocation(shader, "lightSpaceMatrix");
	glUniformMatrix4fv(modelMatrixLocation, 1, GL_TRUE, (GLfloat*)entity->modelMatrix.data);
	Mat4 lightSpaceMatrix = getLightSpaceMatrixForLight(light);
	glUniformMatrix4fv(lightSpaceMatrixLocation, 1, GL_TRUE, (GLfloat*)lightSpaceMatrix.data);
	graphicsMeshRender(shader, entity->mesh);
	glUseProgram(0);
}

static Mesh createQuadMesh() {
	r32 quadPositions[] = {-1.0f, -1.0f, -1.0f, 1.0f, 1.0f, -1.0f, 1.0f, 1.0f};
	u32 quadIndices[] = {0, 2, 1, 1, 2, 3};
	Mesh mesh;
	GLuint VBO, EBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(quadPositions), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(quadPositions), quadPositions);

	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(quadIndices), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, sizeof(quadIndices), quadIndices);

	glBindVertexArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	mesh.VAO = VAO;
	mesh.VBO = VBO;
	mesh.EBO = EBO;
	mesh.indexesSize = sizeof(quadIndices);

	return mesh;
}

static void quadRender(Shader shader) {
	static Mesh quadMesh;
	static boolean quadMeshInitialised = false;
	if (!quadMeshInitialised) {
		quadMesh = createQuadMesh();
	}

	glBindVertexArray(quadMesh.VAO);
	glDrawElements(GL_TRIANGLES, quadMesh.indexesSize, GL_UNSIGNED_INT, 0);
}

extern void graphicsEntityRenderQuadShader(Shader shader, const PerspectiveCamera* camera, const u32 texture)
{
	glUseProgram(shader);
	GLint texLocation = glGetUniformLocation(shader, "tex");
	glUniform1i(texLocation, 0);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, texture);
	quadRender(shader);
	glUseProgram(0);
}

extern void graphicsEntityRenderBasicShader(Shader shader, const PerspectiveCamera* camera, const Entity* entity)
{
	glUseProgram(shader);
	GLint modelMatrixLocation = glGetUniformLocation(shader, "modelMatrix");
	GLint viewMatrixLocation = glGetUniformLocation(shader, "viewMatrix");
	GLint projectionMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
	glUniformMatrix4fv(modelMatrixLocation, 1, GL_TRUE, (GLfloat*)entity->modelMatrix.data);
	glUniformMatrix4fv(viewMatrixLocation, 1, GL_TRUE, (GLfloat*)camera->viewMatrix.data);
	glUniformMatrix4fv(projectionMatrixLocation, 1, GL_TRUE, (GLfloat*)camera->projectionMatrix.data);
	graphicsMeshRender(shader, entity->mesh);
	glUseProgram(0);
}

extern void graphicsEntityRenderPhongShader(Shader shader, const PerspectiveCamera* camera, const Entity* entity, const Light* lights, u32 shadowMap)
{
	glUseProgram(shader);
	lightUpdateUniforms(lights, shader);
	GLint cameraPositionLocation = glGetUniformLocation(shader, "cameraPosition");
	GLint shinenessLocation = glGetUniformLocation(shader, "objectShineness");
	GLint modelMatrixLocation = glGetUniformLocation(shader, "modelMatrix");
	GLint viewMatrixLocation = glGetUniformLocation(shader, "viewMatrix");
	GLint projectionMatrixLocation = glGetUniformLocation(shader, "projectionMatrix");
	GLint lightSpaceMatrixLocation = glGetUniformLocation(shader, "lightSpaceMatrix");
	GLint shadowMapLocation = glGetUniformLocation(shader, "shadowMap");
	glUniform4f(cameraPositionLocation, camera->position.x, camera->position.y, camera->position.z, camera->position.w);
	glUniform1f(shinenessLocation, 128.0f);
	glUniformMatrix4fv(modelMatrixLocation, 1, GL_TRUE, (GLfloat*)entity->modelMatrix.data);
	glUniformMatrix4fv(viewMatrixLocation, 1, GL_TRUE, (GLfloat*)camera->viewMatrix.data);
	glUniformMatrix4fv(projectionMatrixLocation, 1, GL_TRUE, (GLfloat*)camera->projectionMatrix.data);
	Mat4 lightSpaceMatrix = getLightSpaceMatrixForLight(lights); // will take first light lul
	glUniformMatrix4fv(lightSpaceMatrixLocation, 1, GL_TRUE, (GLfloat*)lightSpaceMatrix.data);
	glUniform1i(shadowMapLocation, 1);
	glActiveTexture(GL_TEXTURE1);
	glBindTexture(GL_TEXTURE_2D, shadowMap);
	graphicsMeshRender(shader, entity->mesh);
	glUseProgram(0);
}

extern u32 graphicsTextureCreateFromData(const ImageData* imageData)
{
	u32 textureId;

	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	if (imageData->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, imageData->width, imageData->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, imageData->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, imageData->width, imageData->height, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData->data);

	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	// Anisotropic Filtering
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0f);

	glBindTexture(GL_TEXTURE_2D, 0);

	return textureId;
}

extern u32 graphicsTextureCreateFromFloatData(const FloatImageData* imageData)
{
	u32 textureId;

	glGenTextures(1, &textureId);
	glBindTexture(GL_TEXTURE_2D, textureId);
	if (imageData->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, imageData->width, imageData->height, 0, GL_RGBA, GL_FLOAT, imageData->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, imageData->width, imageData->height, 0, GL_RGB, GL_FLOAT, imageData->data);
	glGenerateMipmap(GL_TEXTURE_2D);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);

	// Anisotropic Filtering
	glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 4.0f);

	glBindTexture(GL_TEXTURE_2D, 0);

	return textureId;
}

extern u32 graphicsTextureCreate(const s8* texturePath)
{
	ImageData imageData = graphicsImageLoad(texturePath);
	if (imageData.data == NULL) return -1;
	u32 textureId = graphicsTextureCreateFromData(&imageData);
	graphicsImageFree(&imageData);

	return textureId;
}

extern void graphicsTextureDelete(u32 textureId)
{
	glDeleteTextures(1, &textureId);
}

extern void graphicsPointLightCreate(Light* light, Vec4 position, Vec4 ambientColor, Vec4 diffuseColor, Vec4 specularColor)
{
	light->lightType = POINT_LIGHT;
	light->position = position;
	light->ambientColor = ambientColor;
	light->diffuseColor = diffuseColor;
	light->specularColor = specularColor;
}

extern void graphicsDirectionalLightCreate(Light* light, Vec4 direction, Vec4 ambientColor, Vec4 diffuseColor, Vec4 specularColor)
{
	light->lightType = DIRECTIONAL_LIGHT;
	light->direction = direction;
	light->ambientColor = ambientColor;
	light->diffuseColor = diffuseColor;
	light->specularColor = specularColor;
}

// If memory is null, new memory will be allocated
extern FloatImageData graphicsImageDataToFloatImageData(ImageData* imageData, r32* memory)
{
	// @TODO: check WHY this is happening
	s32 imageChannels = imageData->channels;

	if (!memory)
		memory = (r32*)malloc(sizeof(r32) * imageData->height * imageData->width * imageChannels);

	for (s32 i = 0; i < imageData->height; ++i)
	{
		for (s32 j = 0; j < imageData->width; ++j)
		{
			memory[i * imageData->width * imageChannels + j * imageChannels] =
				imageData->data[i * imageData->width * imageData->channels + j * imageData->channels] / 255.0f;
			memory[i * imageData->width * imageChannels + j * imageChannels + 1] =
				imageData->data[i * imageData->width * imageData->channels + j * imageData->channels + 1] / 255.0f;
			memory[i * imageData->width * imageChannels + j * imageChannels + 2] =
				imageData->data[i * imageData->width * imageData->channels + j * imageData->channels + 2] / 255.0f;
			memory[i * imageData->width * imageChannels + j * imageChannels + 3] = 1.0f;
		}
	}

	FloatImageData fid;
	fid.width = imageData->width;
	fid.height = imageData->height;
	fid.channels = imageData->channels;
	fid.data = memory;

	return fid;
}

extern ImageData graphicsFloatImageDataToImageData(const FloatImageData* floatImageData, u8* memory)
{
	// @TODO: check WHY this is happening
	s32 imageChannels = floatImageData->channels;

	if (!memory)
		memory = (u8*)malloc(sizeof(u8) * floatImageData->height * floatImageData->width * imageChannels);

	for (s32 i = 0; i < floatImageData->height; ++i)
	{
		for (s32 j = 0; j < floatImageData->width; ++j)
		{
			memory[i * floatImageData->width * imageChannels + j * imageChannels] = (u8)round(255.0f * floatImageData->data[i * floatImageData->width * imageChannels + j * imageChannels]);
			memory[i * floatImageData->width * imageChannels + j * imageChannels + 1] = (u8)round(255.0f * floatImageData->data[i * floatImageData->width * imageChannels + j * imageChannels + 1]);
			memory[i * floatImageData->width * imageChannels + j * imageChannels + 2] = (u8)round(255.0f * floatImageData->data[i * floatImageData->width * imageChannels + j * imageChannels + 2]);
			if (floatImageData->channels > 3) memory[i * floatImageData->width * imageChannels + j * imageChannels + 3] = 255;
		}
	}

	ImageData id;
	id.width = floatImageData->width;
	id.height = floatImageData->height;
	id.channels = floatImageData->channels;
	id.data = memory;

	return id;
}

extern Mesh graphicsMeshCreateFromObjWithColor(const s8* objPath, NormalMappingInfo* normalInfo, Vec4 diffuseColor)
{
	Vertex* vertices;
	u32* indexes;
	objParse(objPath, &vertices, &indexes);
	Mesh m = graphicsMeshCreateWithColor(vertices, array_get_length(vertices), indexes,
		array_get_length(indexes), normalInfo, diffuseColor);
	array_release(vertices);
	array_release(indexes);
	return m;
}

extern Mesh graphicsMeshCreateFromObjWithTexture(const s8* objPath, NormalMappingInfo* normalInfo, u32 diffuseMap)
{
	Vertex* vertices;
	u32* indexes;
	objParse(objPath, &vertices, &indexes);
	Mesh m = graphicsMeshCreateWithTexture(vertices, array_get_length(vertices), indexes,
		array_get_length(indexes), normalInfo, diffuseMap);
	array_release(vertices);
	array_release(indexes);
	return m;
}