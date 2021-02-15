#version 330 core

in vec3 tex_coords;
out vec4 final_color;

uniform samplerCube skybox_texture;

void main()
{
	vec3 env_color = texture(skybox_texture, tex_coords).rgb;

	env_color = env_color / (env_color + vec3(1.0));
    env_color = pow(env_color, vec3(1.0/2.2)); 
  
    final_color = vec4(env_color, 1.0);
} 