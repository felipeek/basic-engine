#version 330 core

in vec3 tex_coords;
out vec4 final_color;

uniform samplerCube skybox_texture;

void main()
{
	final_color = texture(skybox_texture, tex_coords).rgba;
} 