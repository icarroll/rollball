#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;
in vec3 thingPos[];
out vec3 FragPos;
out vec3 Normal;
vec3 GetNormal()
{
  vec3 a = thingPos[0] - thingPos[1];
  vec3 b = thingPos[2] - thingPos[1];
  return normalize(cross(a, b));
}
void main() {
  vec3 n = GetNormal();

  FragPos = thingPos[0];
  Normal = n;
  gl_Position = gl_in[0].gl_Position;
  EmitVertex();

  FragPos = thingPos[1];
  Normal = n;
  gl_Position = gl_in[1].gl_Position;
  EmitVertex();

  FragPos = thingPos[2];
  Normal = n;
  gl_Position = gl_in[2].gl_Position;
  EmitVertex();

  EndPrimitive();
}
