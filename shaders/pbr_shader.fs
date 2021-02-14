#version 330 core
#define MAX_NUMBER_OF_LIGHTS 64

const int MAX_NUMBER_OF_POINT_SHADOW_MAPS = 3;
const float MIN_DISTANCE_TO_IGNORE_LIGHT = 5.0;
const float PI_F = 3.14159265359;

out vec4 final_color;

in vec3 fragment_position;
in vec3 fragment_normal;
in vec2 fragment_texture_coords;
in mat4 tangent_matrix;

struct Albedo_Info {
  bool use;
  sampler2D map;
  vec3 albedo;
};

struct Metallic_Info {
  bool use;
  sampler2D map;
  float metallic;
};

struct Roughness_Info {
  bool use;
  sampler2D map;
  float roughness;
};

struct Normal_Info {
  bool use;
  sampler2D map;
};

struct Light {
  vec3 position;
  vec3 color;
};

uniform vec3 camera_position;
uniform Albedo_Info albedo_info;
uniform Metallic_Info metallic_info;
uniform Roughness_Info roughness_info;
uniform Normal_Info normal_info;
uniform int light_num;
uniform Light lights[MAX_NUMBER_OF_LIGHTS];

// GGX's NDF (Normal Distribution Function)
// The NDF gives the distribution of a given normal over the surface
float NDF_GGX(vec3 normal, vec3 H, float roughness)
{
  // Based on observations by Disney and adopted by Epic Games,
  // the lighting looks more correct squaring the roughness in both the geometry and normal distribution function.
  float a = roughness * roughness;

  float a2 = a * a;
  float nDotH = max(dot(normal, H), 0.0);
  float nDotH2 = nDotH * nDotH;
  float num = a2;
  float denom = (nDotH2 * (a2 - 1) + 1);
  denom = PI_F * denom * denom;
  return num / denom;
}

float geometry_smith_GGX(vec3 normal, vec3 view, float roughness)
{
  float NdotV = max(dot(normal, view), 0.0);
  float r = (roughness + 1.0);
  float k = (r*r) / 8.0;

  float num   = NdotV;
  float denom = NdotV * (1.0 - k) + k;

  return num / denom;
}

// Smith's Geometry Function G
// The geometry function accounts for masking aswell as shadowing
float geometry_smith(vec3 N, vec3 V, vec3 L, float roughness) {
  float g_v = geometry_smith_GGX(N, V, roughness);
  float g_l = geometry_smith_GGX(N, L, roughness);
  return g_v * g_l;
}

vec3 fresnel_schlick(vec3 H, vec3 view, vec3 F0)
{
  float cos_theta = max(dot(H, view), 0.0);
  if (cos_theta > 1.0) cos_theta = 1.0;
  return F0 + (1.0 - F0) * pow(1.0 - min(cos_theta, 1.0), 5.0);
}

vec3 shirley_diffuse_term(vec3 albedo, vec3 normal, vec3 fragment_to_light_vec, vec3 view) {
  float NdotL = max(dot(normal, fragment_to_light_vec), 0.0);
  float NdotV = max(dot(normal, view), 0.0);

  return (21.0 / 20.0) * albedo * (1.0 - pow((1.0 - NdotL), 5)) * (1.0 - pow((1.0 - NdotV), 5));
}

vec3 get_correct_normal()
{
  vec3 normal;

  // Check if normal map is being used. In case positive, the normal must be obtained from the normal map.
  // If normal map is not being used, we use the fragment normal.
  if (normal_info.use)
  {
    // Sample normal map (range [0, 1])
    normal = texture(normal_info.map, fragment_texture_coords).xyz;
    // Transform normal vector to range [-1, 1]
    normal = normal * 2.0 - 1.0;
    // We must invert y
    normal.y = -normal.y;
    // Normalize normal
    normal = normalize(normal);
    // Transform normal from tangent space to world space.
    normal = normalize((tangent_matrix * vec4(normal.xyz, 0.0)).xyz);
  }
  else
    normal = normalize(fragment_normal);

  return normal;
}

vec3 get_point_light_evaluation(Light light, vec3 normal, vec3 view, float metallic, vec3 albedo, float roughness, vec3 F0)
{
  // Discard light if distance is greater than MIN_DISTANCE_TO_IGNORE_LIGHT
  vec3 fragment_to_light_vec = normalize(light.position - fragment_position);
  if (length(fragment_to_light_vec) > MIN_DISTANCE_TO_IGNORE_LIGHT)
    return vec3(0.0, 0.0, 0.0);

  // H is the normal of the microfacets that will reflect the light
  // AKA halfway vector
  vec3 H = normalize(view + fragment_to_light_vec);

  // Calculate light attenuation
  float light_distance = length(light.position - fragment_position);
  float light_attenuation = 1.0 / (light_distance * light_distance);

  // The radiance of the incoming light (Li)
  vec3 radiance = light.color * light_attenuation;

  // Smith's Geometry Function G
  // The geometry function accounts for masking aswell as shadowing
  float G = geometry_smith(normal, view, fragment_to_light_vec, roughness);

  // GGX's NDF (Normal Distribution Function)
  // The NDF gives the distribution of a given normal over the surface
  float NDF = NDF_GGX(normal, H, roughness);

  // Schlick approximation to the Fresnel Reflectance
  // The fresnel reflectance gives the relation between reflected light vs refracted light
  vec3 F = fresnel_schlick(H, view, F0);

  // k_s gives the amount of reflected light
  vec3 k_s = F;
  // k_d gives the amount of refracted light
  vec3 k_d = vec3(1.0) - F;
  // enforce k_d == 0 if the surface is metallic
  k_d *= 1.0 - metallic;

  /* Calculate the Cook-Torrance BRDF */
  
  // Specular Part
  vec3 numerator = NDF * G * F;
  float denominator = 4.0 * max(dot(normal, view), 0.0) * max(dot(normal, fragment_to_light_vec), 0.0);
  vec3 specular = numerator / max(denominator, 0.00000001);

  // Diffuse Part
  vec3 diffuse = albedo;
  //vec3 diffuse = shirley_diffuse_term(albedo, normal, fragment_to_light_vec, view);

  float NdotL = max(dot(normal, fragment_to_light_vec), 0.0);
  return (k_d * diffuse / PI_F + specular) * radiance * NdotL;
}

void main()
{
  vec3 albedo = albedo_info.use ?
    pow(texture(albedo_info.map, fragment_texture_coords).rgb, vec3(2.2)) : albedo_info.albedo;
  float metallic = metallic_info.use ?
    texture(metallic_info.map, fragment_texture_coords).r : metallic_info.metallic;
  float roughness = roughness_info.use ?
    texture(roughness_info.map, fragment_texture_coords).r : roughness_info.roughness;

  // The characteristic specular color of the substance.
  // This is the fresnel reflectance when the angle of incidence is 0 degrees.
  // We make the simplifying assumption that most dieletric surfaces look correct with F0 = 0.04
  // For metallic surfaces, we use the albedo as the F0
  vec3 F0 = mix(vec3(0.04), albedo, metallic);

  // Fragment's normal
  vec3 normal = get_correct_normal();

  // View vector
  vec3 view = normalize(camera_position - fragment_position);

  // Lo is the output color
  vec3 Lo = vec3(0.0);

  for (int i = 0; i < light_num; ++i) {
	Lo += get_point_light_evaluation(lights[i], normal, view, metallic, albedo, roughness, F0);
  }

  vec3 ambient = vec3(0.003, 0.003, 0.003) * albedo;
  //Lo += ambient;

  // HDR
  Lo = Lo / (Lo + vec3(1.0));
  // Gamma Correction
  Lo = pow(Lo, vec3(1.0/2.2));

  final_color = vec4(Lo.rgb, 1.0);
}