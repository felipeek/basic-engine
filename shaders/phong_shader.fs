#version 330 core

in vec4 fragment_position;
in vec4 fragment_normal;
in vec2 fragment_texture_coords;

// Light
struct Light
{
	vec4 position;
	vec4 ambient_color;
	vec4 diffuse_color;
	vec4 specular_color;
};

// Normal Mapping
struct Normal_Mapping_Info
{
	bool use_normal_map;
	bool tangent_space;	// @not implemented
	sampler2D normal_map_texture;
};

// Diffuse Info
struct Diffuse_Info
{
	bool use_diffuse_map;
	vec4 diffuse_color;
	sampler2D diffuse_map;
};

uniform mat4 model_matrix;
uniform Light lights[16];
uniform int light_quantity;
uniform Normal_Mapping_Info normal_mapping_info;
uniform vec4 camera_position;
uniform float object_shineness;
uniform Diffuse_Info diffuse_info;
// Specular map will not be used (<1,1,1,1> assumed)

out vec4 final_color;

vec4 get_correct_normal()
{
	vec4 normal;

	if (normal_mapping_info.use_normal_map)
	{
		// Sample normal map (range [0, 1])
		normal = texture(normal_mapping_info.normal_map_texture, fragment_texture_coords);
		// Transform normal vector to range [-1, 1]
		// normal = normal * 2.0 - 1.0;
		// W coordinate must be 0
		normal.w = 0;
		// Normalize normal
		normal = normalize(normal);

		vec3 normal_v3 = mat3(inverse(transpose(model_matrix))) * normal.xyz;
		normal = vec4(normal_v3, 0);
		normal = normalize(normal);
	}
	else
		normal = normalize(fragment_normal);

	return normal;
}

vec3 get_point_color_of_light(Light light)
{
	vec4 normal = get_correct_normal();
	vec4 real_diffuse_color = diffuse_info.use_diffuse_map ? texture(diffuse_info.diffuse_map, fragment_texture_coords) :
		diffuse_info.diffuse_color;

	vec4 fragment_to_point_light_vec = normalize(light.position - fragment_position);

	// Ambient Color
	vec4 point_ambient_color = light.ambient_color * real_diffuse_color;

	// Diffuse Color
	float point_diffuse_contribution = max(0, dot(fragment_to_point_light_vec, normal));
	vec4 point_diffuse_color = point_diffuse_contribution * light.diffuse_color * real_diffuse_color;
	
	// Specular Color
	vec4 fragment_to_camera_vec = normalize(camera_position - fragment_position);
	float point_specular_contribution = pow(max(dot(fragment_to_camera_vec, reflect(-fragment_to_point_light_vec, normal)), 0.0), object_shineness);
	vec4 point_specular_color = point_specular_contribution * light.specular_color * vec4(1.0, 1.0, 1.0, 1.0);

	// Attenuation
	float point_light_distance = length(light.position - fragment_position);
	float point_attenuation = 1.0 / (1.0 + 0.0014 * point_light_distance +
		0.000007 * point_light_distance * point_light_distance);

	point_ambient_color *= point_attenuation;
	point_diffuse_color *= point_attenuation;
	point_specular_color *= point_attenuation;

	vec4 point_color = point_ambient_color + point_diffuse_color;// + point_specular_color;
	return point_color.xyz;
	//final_color = vec4(point_color.xyz, 1.0);
}

void main()
{
	final_color = vec4(0.0, 0.0, 0.0, 1.0);

	for (int i = 0; i < light_quantity; ++i)
	{
		vec3 point_color = get_point_color_of_light(lights[i]);
		final_color.x += point_color.x;
		final_color.y += point_color.y;
		final_color.z += point_color.z;
	}
}