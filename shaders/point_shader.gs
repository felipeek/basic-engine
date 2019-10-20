#version 330 core
layout (points) in;
layout (triangle_strip, max_vertices = 5) out;

in vec4 color[];

out vec4 gColor;

void main() {

	gColor = color[0];
	gl_Position = gl_in[0].gl_Position;
	gl_Position.x -= 0.1;
	gl_Position.y -= 0.1;
	EmitVertex();
	gl_Position = gl_in[0].gl_Position;
	gl_Position.x += 0.1;
	gl_Position.y -= 0.1;
	EmitVertex();
	gl_Position = gl_in[0].gl_Position;
	gl_Position.x -= 0.1;
	gl_Position.y += 0.1;
	EmitVertex();
	gl_Position = gl_in[0].gl_Position;
	gl_Position.x += 0.1;
	gl_Position.y += 0.1;
	EmitVertex();
	gl_Position = gl_in[0].gl_Position;
	gl_Position.y += 0.2;
	EmitVertex();
	EndPrimitive();
}