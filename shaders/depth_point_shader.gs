#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices=18) out;

uniform mat4 shadowMatrices[6];

out vec4 fragmentPosition; // FragPos from GS (output per emitvertex)

void main()
{
    for(int face = 0; face < 6; ++face)
    {
        gl_Layer = face; // built-in variable that specifies to which face we render.
        for(int i = 0; i < 3; ++i) // for each triangle's vertices
        {
            fragmentPosition = gl_in[i].gl_Position;
            gl_Position = shadowMatrices[face] * fragmentPosition;
            EmitVertex();
        }    
        EndPrimitive();
    }
}
//void main()
//{
//    for(int face = 0; face < 6; ++face)
//    {
//        gl_Layer = face; // built-in variable that specifies to which face we render.
//
//		fragmentPosition = gl_in[0].gl_Position;
//		gl_Position = vec4(-1.0, -1.0, 0.0, 1.0);
//		EmitVertex();
//
//		fragmentPosition = gl_in[1].gl_Position;
//		gl_Position = vec4(1.0, 1.0, 0.0, 1.0);
//		EmitVertex();
//
//		fragmentPosition = gl_in[2].gl_Position;
//		gl_Position = vec4(-1.0, 1.0, 0.0, 1.0);
//		EmitVertex();
//
//        EndPrimitive();
//    }
//}