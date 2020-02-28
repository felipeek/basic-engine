#version 330 core

in vec2 frag_pos;

out vec4 finalColor;

uniform sampler2D tex;

void main()
{
	vec2 tex_position;
	tex_position.x = (frag_pos.x + 1.0) / 2.0;
	tex_position.y = (frag_pos.y + 1.0) / 2.0;
	vec4 t = texture(tex, tex_position);
	finalColor = vec4(t.r, t.r, t.r, 1.0);
}