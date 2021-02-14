#include "graphics.h"
#include "util.h"
#include "obj.h"
#include <GL/glew.h>
#include <stb_image.h>
#include <stb_image_write.h>
#include <dynamic_array.h>
#include <math.h>

Image_Data graphics_image_load(const s8* image_path)
{
	Image_Data image_data;

	stbi_set_flip_vertically_on_load(1);
	image_data.data = stbi_load(image_path, &image_data.width, &image_data.height, &image_data.channels, 4);

	image_data.channels = 4;	// @temporary

	return image_data;
}

Float_Image_Data graphics_float_image_load(const s8* image_path)
{
	Image_Data image_data;

	stbi_set_flip_vertically_on_load(1);
	image_data.data = stbi_load(image_path, &image_data.width, &image_data.height, &image_data.channels, 4);

	image_data.channels = 4;	// @temporary

	Float_Image_Data fid = graphics_image_data_to_float_image_data(&image_data, 0);

	graphics_image_free(&image_data);

	return fid;
}

Float_Image_Data graphics_float_image_copy(const Float_Image_Data* image_data)
{
	Float_Image_Data fid;

	fid = *image_data;

	fid.data = malloc(sizeof(r32) * fid.width * fid.height * fid.channels);
	memcpy(fid.data, image_data->data, sizeof(r32) * fid.width * fid.height * fid.channels);
	
	return fid;
}

void graphics_image_free(Image_Data* image_data)
{
	stbi_image_free(image_data->data);
}

void graphics_float_image_free(Float_Image_Data* image_data)
{
	free(image_data->data);
}

void graphics_image_save(const s8* image_path, const Image_Data* image_data)
{
	stbi_flip_vertically_on_write(1);
	stbi_write_bmp(image_path, image_data->width, image_data->height, image_data->channels, image_data->data);
}

void graphics_float_image_save(const s8* image_path, const Float_Image_Data* image_data)
{
	Image_Data id = graphics_float_image_data_to_image_data(image_data, 0);
	graphics_image_save(image_path, &id);
	graphics_image_free(&id);
}

Shader graphics_shader_create(const s8* vertex_shader_path, const s8* fragment_shader_path)
{
	s8* vertex_shader_code = util_read_file(vertex_shader_path, 0);
	s8* fragment_shader_code = util_read_file(fragment_shader_path, 0);

	GLint success;
	GLchar info_log_buffer[1024];
	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(vertex_shader, 1, (const GLchar *const*)&vertex_shader_code, 0);
	glShaderSource(fragment_shader, 1, (const GLchar *const*)&fragment_shader_code, 0);

	glCompileShader(vertex_shader);
	glGetShaderiv(vertex_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertex_shader, 1024, 0, info_log_buffer);
		printf("Error compiling vertex shader: %s\n", info_log_buffer);
	}

	glCompileShader(fragment_shader);
	glGetShaderiv(fragment_shader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragment_shader, 1024, 0, info_log_buffer);
		printf("Error compiling fragment shader: %s\n", info_log_buffer);
	}

	GLuint shader_program = glCreateProgram();

	glAttachShader(shader_program, vertex_shader);
	glAttachShader(shader_program, fragment_shader);
	glLinkProgram(shader_program);

	glGetProgramiv(shader_program, GL_LINK_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(shader_program, 1024, 0, info_log_buffer);
		printf("Error linking program: %s\n", info_log_buffer);
	}

	free(vertex_shader_code);
	free(fragment_shader_code);
	return shader_program;
}

// Vertices must be Vertex[4]
// Indexes must be u32[6]
static void fill_quad_vertices_and_indexes(r32 size, Vertex* vertices, u32* indices)
{
	vertices[0].position = (vec4) { 0.0f, 0.0f, 0.0f, 1.0f };
	vertices[0].normal = (vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[0].texture_coordinates = (vec2) { 0.0f, 0.0f };

	vertices[1].position = (vec4) { size, 0.0f, 0.0f, 1.0f };
	vertices[1].normal = (vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[1].texture_coordinates = (vec2) { 1.0f, 0.0f };

	vertices[2].position = (vec4) { 0.0f, size, 0.0f, 1.0f };
	vertices[2].normal = (vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[2].texture_coordinates = (vec2) { 0.0f, 1.0f };

	vertices[3].position = (vec4) { size, size, 0.0f, 1.0f };
	vertices[3].normal = (vec4) { 0.0f, 0.0f, 1.0f, 0.0f };
	vertices[3].texture_coordinates = (vec2) { 1.0f, 1.0f };

	indices[0] = 0;
	indices[1] = 1;
	indices[2] = 2;
	indices[3] = 1;
	indices[4] = 3;
	indices[5] = 2;
}

static Mesh create_simple_mesh(Vertex* vertices, s32 vertices_size, u32* indices, s32 indices_size)
{
	Mesh mesh;
	GLuint VBO, EBO, VAO;
	glGenVertexArrays(1, &VAO);
	glGenBuffers(1, &VBO);
	glGenBuffers(1, &EBO);

	glBindVertexArray(VAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, vertices_size * sizeof(Vertex), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, vertices_size * sizeof(Vertex), vertices);

	glVertexAttribPointer(0, 4, GL_FLOAT, GL_FALSE, 14 * sizeof(GLfloat), (void*)(0 * sizeof(GLfloat)));
	glEnableVertexAttribArray(0);

	glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 14 * sizeof(GLfloat), (void*)(4 * sizeof(GLfloat)));
	glEnableVertexAttribArray(1);

	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 14 * sizeof(GLfloat), (void*)(8 * sizeof(GLfloat)));
	glEnableVertexAttribArray(2);

	glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, 14 * sizeof(GLfloat), (void*)(10 * sizeof(GLfloat)));
	glEnableVertexAttribArray(3);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices_size * sizeof(u32), 0, GL_STATIC_DRAW);
	glBufferSubData(GL_ELEMENT_ARRAY_BUFFER, 0, indices_size * sizeof(u32), indices);

	glBindVertexArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	mesh.VAO = VAO;
	mesh.VBO = VBO;
	mesh.EBO = EBO;
	mesh.indexes_size = indices_size;

	return mesh;
}

Mesh graphics_mesh_create_with_texture(Vertex* vertices, s32 vertices_size, u32* indices, s32 indices_size, u32 normal_map,
	u32 albedo_map, u32 metallic_map, u32 roughness_map)
{
	Mesh mesh = create_simple_mesh(vertices, vertices_size, indices, indices_size);
	mesh.normal_info.use = false;
	mesh.albedo_info.use = false;
	mesh.albedo_info.albedo = (vec3){1.0f, 0.0f, 0.0f};
	mesh.metallic_info.use = false;
	mesh.metallic_info.metallic = 0.0f;
	mesh.roughness_info.use = false;
	mesh.roughness_info.roughness = 1.0f;
	if (normal_map != -1)
	{
		mesh.normal_info.use = true;
		mesh.normal_info.map = normal_map;
	}
	if (albedo_map != -1)
	{
		mesh.albedo_info.use = true;
		mesh.albedo_info.map = albedo_map;
	}
	if (metallic_map != -1)
	{
		mesh.metallic_info.use = true;
		mesh.metallic_info.map = metallic_map;
	}
	if (roughness_map != -1)
	{
		mesh.roughness_info.use = true;
		mesh.roughness_info.map = roughness_map;
	}
	return mesh;
}

static s8* build_light_uniform_name(s8* buffer, s32 index, const s8* property)
{
	sprintf(buffer, "lights[%d].%s", index, property);
	return buffer;
}

static void light_update_uniforms(const Light* lights, Shader shader)
{
	s32 number_of_lights = array_get_length(lights);
	s8 buffer[64];
	glUseProgram(shader);

	for (s32 i = 0; i < number_of_lights; ++i)
	{
		Light light = lights[i];
		GLint light_position_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "position"));
		GLint color_location = glGetUniformLocation(shader, build_light_uniform_name(buffer, i, "color"));
		glUniform3f(light_position_location, light.position.x, light.position.y, light.position.z);
		glUniform3f(color_location, light.color.x, light.color.y, light.color.z);
	}

	GLint light_quantity_location = glGetUniformLocation(shader, "light_num");
	glUniform1i(light_quantity_location, number_of_lights);
}

static void albedo_update_uniforms(const Albedo_Info* albedo_info, Shader shader)
{
	glUseProgram(shader);
	GLint use_location = glGetUniformLocation(shader, "albedo_info.use");
	GLint albedo_map_location = glGetUniformLocation(shader, "albedo_info.map");
	GLint albedo_location = glGetUniformLocation(shader, "albedo_info.albedo");
	glUniform1i(use_location, albedo_info->use);
	if (albedo_info->use)
	{
		glUniform1i(albedo_map_location, 0);
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, albedo_info->map);
	}
	else
		glUniform3f(albedo_location, albedo_info->albedo.x, albedo_info->albedo.y,
			albedo_info->albedo.z);
}

static void metallic_update_uniforms(const Metallic_Info* metallic_info, Shader shader)
{
	glUseProgram(shader);
	GLint use_location = glGetUniformLocation(shader, "metallic_info.use");
	GLint metallic_map_location = glGetUniformLocation(shader, "metallic_info.map");
	GLint metallic_location = glGetUniformLocation(shader, "metallic_info.metallic");
	glUniform1i(use_location, metallic_info->use);
	if (metallic_info->use)
	{
		glUniform1i(metallic_map_location, 1);
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, metallic_info->map);
	}
	else
		glUniform1f(metallic_location, metallic_info->metallic);
}

static void roughness_update_uniforms(const Roughness_Info* roughness_info, Shader shader)
{
	glUseProgram(shader);
	GLint use_location = glGetUniformLocation(shader, "roughness_info.use");
	GLint roughness_map_location = glGetUniformLocation(shader, "roughness_info.map");
	GLint roughness_location = glGetUniformLocation(shader, "roughness_info.roughness");
	glUniform1i(use_location, roughness_info->use);
	if (roughness_info->use)
	{
		glUniform1i(roughness_map_location, 2);
		glActiveTexture(GL_TEXTURE2);
		glBindTexture(GL_TEXTURE_2D, roughness_info->map);
	}
	else
		glUniform1f(roughness_location, roughness_info->roughness);
}

static void normals_update_uniforms(const Normal_Info* normal_info, Shader shader)
{
	glUseProgram(shader);
	GLint use_normal_map_location = glGetUniformLocation(shader, "normal_info.use");
	GLint normal_map_texture_location = glGetUniformLocation(shader, "normal_info.map");
	glUniform1i(use_normal_map_location, normal_info->use);
	if (normal_info->map)
	{
		glUniform1i(normal_map_texture_location, 3);
		glActiveTexture(GL_TEXTURE3);
		glBindTexture(GL_TEXTURE_2D, normal_info->map);
	}
}

void graphics_mesh_render(Shader shader, Mesh mesh)
{
	glBindVertexArray(mesh.VAO);
	glUseProgram(shader);
	albedo_update_uniforms(&mesh.albedo_info, shader);
	metallic_update_uniforms(&mesh.metallic_info, shader);
	roughness_update_uniforms(&mesh.roughness_info, shader);
	normals_update_uniforms(&mesh.normal_info, shader);
	glDrawElements(GL_TRIANGLES, mesh.indexes_size, GL_UNSIGNED_INT, 0);
	glUseProgram(0);
	glBindVertexArray(0);
}

void graphics_mesh_change_normal_map(Mesh* mesh, u32 normal_map, boolean delete_normal_map)
{
	if (delete_normal_map && mesh->normal_info.use)
		glDeleteTextures(1, &mesh->normal_info.map);

	mesh->normal_info.map = normal_map;
	mesh->normal_info.use = true;
}

void graphics_mesh_change_albedo_map(Mesh* mesh, u32 albedo_map, boolean delete_albedo_map)
{
	if (delete_albedo_map && mesh->albedo_info.use)
		glDeleteTextures(1, &mesh->albedo_info.map);

	mesh->albedo_info.map = albedo_map;
	mesh->albedo_info.use = true;
}

void graphics_mesh_change_metallic_map(Mesh* mesh, u32 metallic_map, boolean delete_metallic_map)
{
	if (delete_metallic_map && mesh->metallic_info.use)
		glDeleteTextures(1, &mesh->metallic_info.map);

	mesh->metallic_info.map = metallic_map;
	mesh->metallic_info.use = true;
}

void graphics_mesh_change_roughness_map(Mesh* mesh, u32 roughness_map, boolean delete_roughness_map)
{
	if (delete_roughness_map && mesh->roughness_info.use)
		glDeleteTextures(1, &mesh->roughness_info.map);

	mesh->roughness_info.map = roughness_map;
	mesh->roughness_info.use = true;
}

static void recalculate_model_matrix(Entity* entity)
{
	r32 s, c;

	mat4 scale_matrix = (mat4) {
		entity->world_scale.x, 0.0f, 0.0f, 0.0f,
			0.0f, entity->world_scale.y, 0.0f, 0.0f,
			0.0f, 0.0f, entity->world_scale.z, 0.0f,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	mat4 rotation_matrix = quaternion_get_matrix(&entity->world_rotation);

	mat4 translation_matrix = (mat4) {
		1.0f, 0.0f, 0.0f, entity->world_position.x,
			0.0f, 1.0f, 0.0f, entity->world_position.y,
			0.0f, 0.0f, 1.0f, entity->world_position.z,
			0.0f, 0.0f, 0.0f, 1.0f
	};

	entity->model_matrix = gm_mat4_multiply(&rotation_matrix, &scale_matrix);
	entity->model_matrix = gm_mat4_multiply(&translation_matrix, &entity->model_matrix);
}

void graphics_entity_create(Entity* entity, Mesh mesh, vec4 world_position, Quaternion world_rotation, vec3 world_scale)
{
	entity->mesh = mesh;
	entity->world_position = world_position;
	entity->world_rotation = world_rotation;
	entity->world_scale = world_scale;
	recalculate_model_matrix(entity);
}

void graphics_entity_mesh_replace(Entity* entity, Mesh mesh, boolean delete_normal_map, boolean delete_albedo_map,
	boolean delete_metallic_map, boolean delete_roughness_map)
{
	glDeleteBuffers(1, &entity->mesh.VBO);
	glDeleteBuffers(1, &entity->mesh.EBO);
	glDeleteVertexArrays(1, &entity->mesh.VAO);
	if (delete_normal_map && entity->mesh.normal_info.use)
		glDeleteTextures(1, &entity->mesh.normal_info.map);
	if (delete_albedo_map && entity->mesh.albedo_info.use)
		glDeleteTextures(1, &entity->mesh.albedo_info.map);
	if (delete_metallic_map && entity->mesh.metallic_info.use)
		glDeleteTextures(1, &entity->mesh.metallic_info.map);
	if (delete_roughness_map && entity->mesh.roughness_info.use)
		glDeleteTextures(1, &entity->mesh.roughness_info.map);

	entity->mesh = mesh;
}

void graphics_entity_set_position(Entity* entity, vec4 world_position)
{
	entity->world_position = world_position;
	recalculate_model_matrix(entity);
}

void graphics_entity_set_rotation(Entity* entity, Quaternion world_rotation)
{
	entity->world_rotation = world_rotation;
	recalculate_model_matrix(entity);
}

void graphics_entity_set_scale(Entity* entity, vec3 world_scale)
{
	entity->world_scale = world_scale;
	recalculate_model_matrix(entity);
}

void graphics_entity_render_basic_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity)
{
	glUseProgram(shader);
	GLint model_matrix_location = glGetUniformLocation(shader, "model_matrix");
	GLint view_matrix_location = glGetUniformLocation(shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(shader, "projection_matrix");
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)entity->model_matrix.data);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)camera->projection_matrix.data);
	graphics_mesh_render(shader, entity->mesh);
	glUseProgram(0);
}

void graphics_entity_render_pbr_shader(Shader shader, const Perspective_Camera* camera, const Entity* entity, const Light* lights)
{
	glUseProgram(shader);
	light_update_uniforms(lights, shader);
	GLint camera_position_location = glGetUniformLocation(shader, "camera_position");
	GLint shineness_location = glGetUniformLocation(shader, "object_shineness");
	GLint model_matrix_location = glGetUniformLocation(shader, "model_matrix");
	GLint view_matrix_location = glGetUniformLocation(shader, "view_matrix");
	GLint projection_matrix_location = glGetUniformLocation(shader, "projection_matrix");
	glUniform3f(camera_position_location, camera->position.x, camera->position.y, camera->position.z);
	glUniform1f(shineness_location, 128.0f);
	glUniformMatrix4fv(model_matrix_location, 1, GL_TRUE, (GLfloat*)entity->model_matrix.data);
	glUniformMatrix4fv(view_matrix_location, 1, GL_TRUE, (GLfloat*)camera->view_matrix.data);
	glUniformMatrix4fv(projection_matrix_location, 1, GL_TRUE, (GLfloat*)camera->projection_matrix.data);
	graphics_mesh_render(shader, entity->mesh);
	glUseProgram(0);
}

u32 graphics_texture_create_from_data(const Image_Data* image_data)
{
	u32 texture_id;

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	if (image_data->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, image_data->width, image_data->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, image_data->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, image_data->width, image_data->height, 0, GL_RGB, GL_UNSIGNED_BYTE, image_data->data);

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

	return texture_id;
}

u32 graphics_texture_create_from_float_data(const Float_Image_Data* image_data)
{
	u32 texture_id;

	glGenTextures(1, &texture_id);
	glBindTexture(GL_TEXTURE_2D, texture_id);
	if (image_data->channels == 4)
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, image_data->width, image_data->height, 0, GL_RGBA, GL_FLOAT, image_data->data);
	else
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, image_data->width, image_data->height, 0, GL_RGB, GL_FLOAT, image_data->data);
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

	return texture_id;
}

u32 graphics_texture_create(const s8* texture_path)
{
	Image_Data image_data = graphics_image_load(texture_path);
	if (image_data.data == NULL) return -1;
	u32 texture_id = graphics_texture_create_from_data(&image_data);
	graphics_image_free(&image_data);

	return texture_id;
}

void graphics_texture_delete(u32 texture_id)
{
	glDeleteTextures(1, &texture_id);
}

void graphics_light_create(Light* light, vec3 position, vec3 color)
{
	light->position = position;
	light->color = color;
}

// If memory is null, new memory will be allocated
Float_Image_Data graphics_image_data_to_float_image_data(Image_Data* image_data, r32* memory)
{
	// @TODO: check WHY this is happening
	s32 image_channels = image_data->channels;

	if (!memory)
		memory = (r32*)malloc(sizeof(r32) * image_data->height * image_data->width * image_channels);

	for (s32 i = 0; i < image_data->height; ++i)
	{
		for (s32 j = 0; j < image_data->width; ++j)
		{
			memory[i * image_data->width * image_channels + j * image_channels] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 1] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 1] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 2] =
				image_data->data[i * image_data->width * image_data->channels + j * image_data->channels + 2] / 255.0f;
			memory[i * image_data->width * image_channels + j * image_channels + 3] = 1.0f;
		}
	}

	Float_Image_Data fid;
	fid.width = image_data->width;
	fid.height = image_data->height;
	fid.channels = image_data->channels;
	fid.data = memory;

	return fid;
}

Image_Data graphics_float_image_data_to_image_data(const Float_Image_Data* float_image_data, u8* memory)
{
	// @TODO: check WHY this is happening
	s32 image_channels = float_image_data->channels;

	if (!memory)
		memory = (u8*)malloc(sizeof(u8) * float_image_data->height * float_image_data->width * image_channels);

	for (s32 i = 0; i < float_image_data->height; ++i)
	{
		for (s32 j = 0; j < float_image_data->width; ++j)
		{
			memory[i * float_image_data->width * image_channels + j * image_channels] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 1] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 1]);
			memory[i * float_image_data->width * image_channels + j * image_channels + 2] = (u8)round(255.0f * float_image_data->data[i * float_image_data->width * image_channels + j * image_channels + 2]);
			if (float_image_data->channels > 3) memory[i * float_image_data->width * image_channels + j * image_channels + 3] = 255;
		}
	}

	Image_Data id;
	id.width = float_image_data->width;
	id.height = float_image_data->height;
	id.channels = float_image_data->channels;
	id.data = memory;

	return id;
}

Mesh graphics_mesh_create_from_obj_with_texture(const s8* obj_path, u32 normal_map, u32 albedo_map, u32 metallic_map, u32 roughness_map)
{
	Vertex* vertices;
	u32* indexes;
	obj_parse(obj_path, &vertices, &indexes);
	Mesh m = graphics_mesh_create_with_texture(vertices, array_get_length(vertices), indexes, array_get_length(indexes), normal_map, albedo_map, metallic_map, roughness_map);
	array_release(vertices);
	array_release(indexes);
	return m;
}